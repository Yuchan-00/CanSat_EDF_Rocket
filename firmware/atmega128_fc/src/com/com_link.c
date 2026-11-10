/* com_link.c - see com_link.h */
#include "com_link.h"

#include "../fc_mode.h"
#include "../core/ilink_proto.h"
#include "../core/link_proto.h"
#include "../core/link_proto_v09.h"
#include "../core/onboard.h"
#include "../hal/ilink_uart.h"
#include "../hal/radio.h"
#include "com_simbench.h"

#if FC_LINK_RADIO == FC_RADIO_SX127X
#define COM_STAT_BACKEND IL_STAT_SX127X
#else
#define COM_STAT_BACKEND 0u
#endif
#if (COM_FWD_SLOTS & (COM_FWD_SLOTS - 1u)) != 0 || COM_FWD_SLOTS > 128u
#error "COM_FWD_SLOTS must be a power of two"
#endif
#if (COM_REPLY_WINDOW_MS * COM_TICK_HZ) % 1000u != 0u || COM_REPLY_WINDOW_MS * COM_TICK_HZ / 1000u < 2u
#error "COM_REPLY_WINDOW_MS must be a whole number (>= 2) of COM ticks"
#endif
#if COM_STAT_BOOT_TICKS * 1000u < COM_STAT_BOOT_MS * COM_TICK_HZ || COM_STAT_BOOT_TICKS >= ILINK_STAT_DIV
#error "COM_STAT_BOOT_TICKS must cover COM_STAT_BOOT_MS and stay below one STAT period"
#endif

/* the FCU's reply snapshot in this build (fc_mode.h): IL_SNAP -> TELEM v1 (ballistic, v0.6.3), IL_SNAP2 -> TELEM v2
 * (VTOL: + phase and altitude target, 39-byte frame).  The decoder accepts only this mission's snapshot TYPEs
 * (IL_ACCEPT_COM / the VTOL pair below), so the other mission's snapshot is rejected in its header (bad_len / bad_type).
 * v0.8 VTOL: the FCU with GPS position hold sends IL_SNAP3 (+ GPS flags, satellites, accuracy, position mode, distance
 * to the hold point) and the COM answers with TELEM v3 (45-byte frame); the FCU built without position hold
 * (-D ONB_NO_GPS, the indoor fallback) still sends IL_SNAP2 -> TELEM v2 - one COM image serves both (the two lengths, 55 /
 * 62, are told apart by the header; sim/ilink_proto.py ACCEPT_COM_VTOL | ACCEPT_COM_GPS).
 * v0.9 RP1: SNAP2 57 / SNAP3 65 bytes with the power tail; a power build's snapshot -> TELEM v4 / v5 (com_link.h) */
#if FC_VTOL
#define COM_SNAP_TYPE IL_TYPE_SNAP3
#define COM_ACCEPT (IL_ACCEPT_COM_VTOL | IL_ACCEPT_COM_GPS)
#define COM_TELEM_FRAME_BYTES LP_TELEM5_FRAME_BYTES      /* the longest reply: TELEM v5 (v0.8: TELEM3 45 B) */
typedef union {
    il_snap2_t s2;                                   /* both begin with the SNAP fields (il_snap_t s) */
    il_snap3_t s3;
} com_snap_t;
#define COM_SNAP_SEQ(x) ((x).s2.s.reply_seq)
static bool s_snap_v3;                               /* s_snap holds a SNAP3 (else a SNAP2) */
#else
#define COM_SNAP_TYPE IL_TYPE_SNAP
#define COM_ACCEPT IL_ACCEPT_COM
#define COM_TELEM_FRAME_BYTES LP_TELEM_FRAME_BYTES
typedef il_snap_t com_snap_t;
#define COM_SNAP_SEQ(x) ((x).reply_seq)
#endif

/* decoders (static: nothing large on the stack); the SNAP comes straight out of s_il (il_decoder_push) */
static lp_decoder_t s_lp;
static lp_frame_t s_lp_out[LP_DEC_MAX_OUT];
static il_decoder_t s_il;
static uint8_t s_scratch[IL_STAT_FRAME_BYTES];       /* IL_CMD / IL_STAT encoding (never both at once) */
static uint8_t s_payload[IL_STAT_LEN];

/* D5: reply slots, one per queued IL_CMD (ring, newest at s_fwd_head - 1) */
static struct {
    uint32_t tick;                                   /* COM tick the IL_CMD was queued in */
    uint8_t seq;
    bool open;
} s_fwd[COM_FWD_SLOTS];
static uint8_t s_fwd_head;

/* reply pipeline: 0 idle, 1 SNAP waiting for il_snap_to_tlm_src, 2 waiting for quantisation + encoding */
static uint8_t s_rep_stage;
static uint32_t s_rep_tick;                          /* queue tick of the IL_CMD the reply in the pipeline answers */
static com_snap_t s_snap;
static onb_tlm_src_t s_src;
static uint8_t s_tlm[COM_TELEM_FRAME_BYTES];         /* the pending TELEM (VTOL: TELEM2 / 3, power builds 4 / 5) frame */
static uint8_t s_tlm_n;                              /* 0 = nothing pending */
static uint32_t s_tlm_tick;                          /* ... and the queue tick of the IL_CMD it answers */

/* D4: STAT pacing */
static bool s_stat_due, s_stat_boot;
static uint8_t s_stat_kind;
static uint32_t s_last_stat_tick, s_last_cmd_tick;   /* 0 at boot: "no uplink since boot" counts from tick 0 */
static uint8_t s_reset_cause;

static com_link_stats_t s_st;

void com_link_init(uint8_t reset_cause)
{
    uint8_t i;
    lp_decoder_init(&s_lp, LINK_VEHICLE_ID, LP_ACCEPT_CMD);
    il_decoder_init(&s_il, COM_ACCEPT);              /* SNAP (VTOL: SNAP2 / SNAP3) only: CMD / STAT headers rejected */
    for (i = 0; i < COM_FWD_SLOTS; i++)
        s_fwd[i].open = false;
    s_fwd_head = 0;
    s_rep_stage = 0;
    s_tlm_n = 0;
    s_reset_cause = reset_cause;
    s_stat_due = true;                               /* once after boot, from COM tick COM_STAT_BOOT_TICKS on (R3) */
    s_stat_boot = true;
    s_stat_kind = COM_STAT_KIND_BOOT;
    s_last_stat_tick = s_last_cmd_tick = 0;
}

static bool in_window(uint32_t tick, uint32_t queued)
{
    return tick - queued < COM_REPLY_WINDOW_TICKS;
}

/* ---- uplink: radio bytes -> CMD frames -> IL_CMD to the FCU -------------------------------- */
static void forward_cmd(const lp_frame_t *f, const radio_stats_t *rs, uint32_t tick)
{
    il_cmd_t c;
    uint8_t n;
    c.seq = f->seq;
    lp_unpack_cmd(f->payload, &c.cmd);
    c.rssi_dbm = rs->rssi_dbm;
    c.snr_q4 = rs->snr_q4;
    il_pack_cmd(&c, s_payload);
    n = il_encode(IL_TYPE_CMD, s_payload, IL_CMD_LEN, s_scratch);
    if (n == 0u || !ilink_uart_write(s_scratch, n)) {
        s_st.il_tx_dropped++;
        return;
    }
    s_st.cmd_forwarded++;
    s_fwd[s_fwd_head].tick = tick;                   /* open a reply slot (overwrites the oldest) */
    s_fwd[s_fwd_head].seq = c.seq;
    s_fwd[s_fwd_head].open = true;
    s_fwd_head = (uint8_t)((s_fwd_head + 1u) & (COM_FWD_SLOTS - 1u));
    s_last_cmd_tick = tick;
    if (!s_stat_due && tick - s_last_stat_tick >= ILINK_STAT_DIV) {
        s_stat_due = true;                           /* D4: the STAT rides right behind this IL_CMD */
        s_stat_kind = COM_STAT_KIND_PIGGYBACK;
    }
    COM_SB_CMD_QUEUED(c.seq);
}

void com_link_uplink(uint32_t tick)
{
    uint8_t buf[COM_RADIO_CHUNK], n = radio_read(buf, COM_RADIO_CHUNK), i, k, m;
    const radio_stats_t *rs;
    if (n == 0u)
        return;
    rs = radio_stats();
    for (i = 0; i < n; i++) {
        m = lp_decoder_feed(&s_lp, buf[i], s_lp_out, LP_DEC_MAX_OUT);
        for (k = 0; k < m; k++) {
            if (s_lp_out[k].type != LP_TYPE_CMD || s_lp_out[k].len != LP_CMD_PAYLOAD_LEN)
                continue;                            /* the decoder accepts CMD only; defensive */
            forward_cmd(&s_lp_out[k], rs, tick);
        }
    }
}

/* ---- FCU -> COM: snapshots through the reply gate into the reply pipeline ------------------- */
/* D5: the newest open slot with this seq inside the window -> its queue tick (closes it and every older slot) */
static bool reply_gate(uint8_t seq, uint32_t tick, uint32_t *queued)
{
    uint8_t k, j = s_fwd_head;
    bool hit = false;
    for (k = 0; k < COM_FWD_SLOTS; k++) {
        j = (uint8_t)((j - 1u) & (COM_FWD_SLOTS - 1u));   /* newest first */
        if (hit) {
            s_fwd[j].open = false;                   /* older than the answered IL_CMD: answered too */
        } else if (s_fwd[j].open && s_fwd[j].seq == seq && in_window(tick, s_fwd[j].tick)) {
            *queued = s_fwd[j].tick;
            s_fwd[j].open = false;
            hit = true;
        }
    }
    return hit;
}

void com_link_ilink(uint32_t tick)
{
    uint8_t buf[COM_ILINK_CHUNK], n = ilink_uart_read(buf, COM_ILINK_CHUNK), i;
    for (i = 0; i < n; i++) {
        const il_frame_t *f = il_decoder_push(&s_il, buf[i]);
        uint32_t queued;
        if (f == 0)
            continue;
#if FC_VTOL
        if (f->type != IL_TYPE_SNAP3 && f->type != IL_TYPE_SNAP2) {
#else
        if (f->type != COM_SNAP_TYPE) {
#endif
            s_st.il_wrong_type++;                    /* defensive: COM_ACCEPT returns this build's snapshot only */
            continue;
        }
        /* reply_seq is the SNAP's first payload byte (il_pack_snap): gate before unpacking */
        if (!reply_gate(f->payload[0], tick, &queued)) {
            s_st.replies_refused++;
            COM_SB_SNAP(f->payload[0], false);
            continue;
        }
        if (s_rep_stage != 0u)
            s_st.replies_superseded++;               /* an older snapshot never became TELEM: the newest wins */
#if FC_VTOL
        s_snap_v3 = f->type == IL_TYPE_SNAP3;
        if (s_snap_v3)
            il_unpack_snap3(f->payload, &s_snap.s3); /* v0.8: + the GPS fields */
        else
            il_unpack_snap2(f->payload, &s_snap.s2); /* reply_seq is byte 0 here too (the SNAP fields come first) */
#else
        il_unpack_snap(f->payload, &s_snap);
#endif
        s_rep_tick = queued;
        s_rep_stage = 1;
        s_st.snaps_accepted++;
        COM_SB_SNAP(f->payload[0], true);
    }
}

/* ---- the costly steps, one per call --------------------------------------------------------- */
static void stat_send(uint32_t tick)
{
    il_stat_t st;
    const radio_stats_t *rs = radio_stats();
    const il_dec_stats_t *ds = &s_il.stats;
    uint8_t n;
    st.com_ticks = tick;
    st.reset_cause = s_reset_cause;
    st.flags = (uint8_t)((rs->ok ? IL_STAT_RADIO_OK : 0u) | (s_stat_boot ? IL_STAT_BOOT : 0u) | COM_STAT_BACKEND);
    st.radio_version = rs->version;
    st.rssi_dbm = rs->rssi_dbm;
    st.snr_q4 = rs->snr_q4;
    st.params_tag = FC_PARAMS_TAG;
    st.dec_frames = s_lp.stats.frames;
    st.dec_bad_crc = s_lp.stats.bad_crc;
    st.dec_bad_len = s_lp.stats.bad_len;
    st.dec_other = (uint16_t)(s_lp.stats.wrong_vehicle + s_lp.stats.wrong_type + s_lp.stats.bad_payload_len +
                              s_lp.stats.overflow);
    st.cmd_forwarded = s_st.cmd_forwarded;
    st.radio_rx_errors = rs->rx_errors;
    st.radio_rx_overflow = rs->rx_overflow;
    st.radio_tx_timeouts = rs->tx_timeouts;
    st.replies = s_st.replies;
    st.replies_dropped = (uint16_t)(s_st.replies_superseded + s_st.replies_refused + s_st.replies_expired);
    /* frames the COM rejected: CRC / LEN / TYPE - the LEN / TYPE of a COM -> FCU type included (IL_ACCEPT_COM) - and the
     * defensive wrong-type count (push never overflows) */
    st.il_bad = (uint16_t)(ds->bad_crc + ds->bad_len + ds->bad_type + s_st.il_wrong_type);
    st.il_tx_dropped = s_st.il_tx_dropped;
    st.loop_us_max = s_st.loop_us_max;
    il_pack_stat(&st, s_payload);
    n = il_encode(IL_TYPE_STAT, s_payload, IL_STAT_LEN, s_scratch);
    if (n != 0u && ilink_uart_write(s_scratch, n)) {
        s_stat_boot = false;                         /* BOOT stays set until a STAT really went out */
        s_st.loop_us_max = 0;
        s_last_stat_tick = tick;
        s_st.stats_sent++;
        if (s_stat_kind == COM_STAT_KIND_PIGGYBACK)
            s_st.stats_piggyback++;
        else if (s_stat_kind == COM_STAT_KIND_QUIET)
            s_st.stats_quiet++;
        COM_SB_STAT_SENT(s_stat_kind);
    } else {
        s_st.il_tx_dropped++;                        /* retried: by the next IL_CMD or the quiet divider */
    }
    s_stat_due = false;
}

bool com_link_job(uint32_t tick)
{
    lp_telem_t t;
    switch (s_rep_stage) {
    case 1:                                          /* roll / pitch / yaw from q: three float32 trig calls */
#if FC_VTOL
        if (s_snap_v3)
            il_snap3_to_tlm_src(&s_snap.s3, &s_src); /* src.v2 + v3: + phase, h_tgt_m, the GPS fields */
        else
            il_snap2_to_tlm_src(&s_snap.s2, &s_src); /* src.v2: + phase, h_tgt_m */
#else
        il_snap_to_tlm_src(&s_snap, &s_src);
#endif
        s_rep_stage = 2;
        return true;
    case 2:                                          /* onboard quantisation, TELEM frame with SEQ = reply_seq */
        onboard_telemetry(&s_src, FC_LOOP_HZ, &t);
        if (s_tlm_n != 0u)
            s_st.replies_superseded++;               /* the previous TELEM is still waiting for the radio */
#if FC_VTOL
        {
            /* v0.9 RP1: the snapshot's power tail (both SNAP layouts: the bytes after the SNAP2 / SNAP3 fields);
             * 2026-11-10: its logic-battery byte (SNAP2 spare / SNAP3 spare3) - the rule is lp_encode_vtol_reply's
             * (core/link_proto_v09.h): TELEM v4 / v5 (power), v6 / v7 (logic battery), else v2 / v3 */
            const uint8_t pf = s_snap_v3 ? s_snap.s3.pw_flags : s_snap.s2.pw_flags;
            const uint8_t va = s_snap_v3 ? s_snap.s3.pw_vaux : s_snap.s2.pw_vaux;
            const uint8_t lb = s_snap_v3 ? s_snap.s3.spare3 : s_snap.s2.spare;
            s_tlm_n = lp_encode_vtol_reply(LINK_VEHICLE_ID, COM_SNAP_SEQ(s_snap), &t, s_src.v3, va, pf, lb, s_tlm);
        }
#else
        s_tlm_n = lp_encode_telem(LINK_VEHICLE_ID, COM_SNAP_SEQ(s_snap), &t, s_tlm);
#endif
        s_tlm_tick = s_rep_tick;
        s_rep_stage = 0;
        return true;
    default:
        break;
    }
    if (!s_stat_due && tick - s_last_stat_tick >= ILINK_STAT_DIV && tick - s_last_cmd_tick > COM_STAT_QUIET_TICKS) {
        s_stat_due = true;                           /* D4: no uplink for > 1.5 / stat_hz: the divider sends it */
        s_stat_kind = COM_STAT_KIND_QUIET;
    }
    if (s_stat_due && (s_stat_kind != COM_STAT_KIND_BOOT || tick >= COM_STAT_BOOT_TICKS)) {
        stat_send(tick);                             /* R3: the boot STAT not before COM_STAT_BOOT_MS (com_link.h) */
        return true;
    }
    return false;
}

/* the only place the COM transmits on the radio */
void com_link_send(uint32_t tick)
{
    if (s_tlm_n == 0u)
        return;
    if (!in_window(tick, s_tlm_tick)) {              /* D5: too late for the ground's half-duplex schedule */
        s_st.replies_expired++;
        s_tlm_n = 0;
        COM_SB_TELEM_EXPIRED(s_tlm[5]);
        return;
    }
    if (radio_tx_busy())
        return;
    if (radio_send(s_tlm, s_tlm_n)) {
        s_st.replies++;
        COM_SB_TELEM_SENT(s_tlm, s_tlm_n);
    } else {
        s_st.replies_superseded++;                   /* no radio (sx127x not found): nothing can go out */
    }
    s_tlm_n = 0;
}

void com_link_pass_us(uint16_t us)
{
    if (us > s_st.loop_us_max)
        s_st.loop_us_max = us;
}

uint16_t com_link_cmd_forwarded(void)
{
    return s_st.cmd_forwarded;
}

const com_link_stats_t *com_link_stats(void)
{
    s_st.il_frames = s_il.stats.frames;
    s_st.il_bad_crc = s_il.stats.bad_crc;
    s_st.il_bad_len = s_il.stats.bad_len;
    s_st.il_bad_type = s_il.stats.bad_type;
    s_st.il_skipped = s_il.stats.skipped_bytes;
    return &s_st;
}
