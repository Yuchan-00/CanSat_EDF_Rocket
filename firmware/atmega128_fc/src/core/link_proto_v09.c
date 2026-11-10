/* link_proto_v09.c - see link_proto_v09.h (v0.9 RP1: TELEM v4 / v5; 2026-11-10: TELEM v6 / v7, the COM's reply rule). */
#include "link_proto_v09.h"

uint8_t lp_encode_telem4(uint8_t veh, uint8_t seq, const lp_telem_t *t, uint8_t v_aux, uint8_t pwr_flags,
                         uint8_t out[LP_TELEM4_FRAME_BYTES])
{
    uint8_t pl[LP_TELEM4_PAYLOAD_LEN];
    lp_pack_telem2(t, pl);
    pl[LP_TELEM2_PAYLOAD_LEN] = v_aux;
    pl[LP_TELEM2_PAYLOAD_LEN + 1u] = pwr_flags;
    return lp_encode_frame(veh, LP_TYPE_TELEM4, seq, pl, LP_TELEM4_PAYLOAD_LEN, out);
}

uint8_t lp_encode_telem5(uint8_t veh, uint8_t seq, const lp_telem_t *t, uint8_t v_aux, uint8_t pwr_flags,
                         uint8_t out[LP_TELEM5_FRAME_BYTES])
{
    uint8_t pl[LP_TELEM5_PAYLOAD_LEN];
    lp_pack_telem3(t, pl);
    pl[LP_TELEM3_PAYLOAD_LEN] = v_aux;
    pl[LP_TELEM3_PAYLOAD_LEN + 1u] = pwr_flags;
    return lp_encode_frame(veh, LP_TYPE_TELEM5, seq, pl, LP_TELEM5_PAYLOAD_LEN, out);
}

void lp_unpack_telem4(const uint8_t in[LP_TELEM4_PAYLOAD_LEN], lp_telem_t *t, uint8_t *v_aux, uint8_t *pwr_flags)
{
    lp_unpack_telem2(in, t);
    *v_aux = in[LP_TELEM2_PAYLOAD_LEN];
    *pwr_flags = in[LP_TELEM2_PAYLOAD_LEN + 1u];
}

void lp_unpack_telem5(const uint8_t in[LP_TELEM5_PAYLOAD_LEN], lp_telem_t *t, uint8_t *v_aux, uint8_t *pwr_flags)
{
    lp_unpack_telem3(in, t);
    *v_aux = in[LP_TELEM3_PAYLOAD_LEN];
    *pwr_flags = in[LP_TELEM3_PAYLOAD_LEN + 1u];
}

uint8_t lp_encode_telem6(uint8_t veh, uint8_t seq, const lp_telem_t *t, uint8_t lbat, uint8_t out[LP_TELEM6_FRAME_BYTES])
{
    uint8_t pl[LP_TELEM6_PAYLOAD_LEN];
    lp_pack_telem2(t, pl);
    pl[LP_TELEM2_PAYLOAD_LEN] = lbat;
    return lp_encode_frame(veh, LP_TYPE_TELEM6, seq, pl, LP_TELEM6_PAYLOAD_LEN, out);
}

uint8_t lp_encode_telem7(uint8_t veh, uint8_t seq, const lp_telem_t *t, uint8_t lbat, uint8_t out[LP_TELEM7_FRAME_BYTES])
{
    uint8_t pl[LP_TELEM7_PAYLOAD_LEN];
    lp_pack_telem3(t, pl);
    pl[LP_TELEM3_PAYLOAD_LEN] = lbat;
    return lp_encode_frame(veh, LP_TYPE_TELEM7, seq, pl, LP_TELEM7_PAYLOAD_LEN, out);
}

void lp_unpack_telem6(const uint8_t in[LP_TELEM6_PAYLOAD_LEN], lp_telem_t *t, uint8_t *lbat)
{
    lp_unpack_telem2(in, t);
    *lbat = in[LP_TELEM2_PAYLOAD_LEN];
}

void lp_unpack_telem7(const uint8_t in[LP_TELEM7_PAYLOAD_LEN], lp_telem_t *t, uint8_t *lbat)
{
    lp_unpack_telem3(in, t);
    *lbat = in[LP_TELEM3_PAYLOAD_LEN];
}

uint8_t lp_encode_vtol_reply(uint8_t veh, uint8_t seq, const lp_telem_t *t, bool v3, uint8_t pw_vaux, uint8_t pw_flags,
                             uint8_t lbat, uint8_t out[LP_FRAME_MAX_BYTES])
{
    if (LP_PWR_TAIL_DUE(pw_flags))                     /* a power build: TELEM v5 (47 B) / v4 (41 B) */
        return v3 ? lp_encode_telem5(veh, seq, t, pw_vaux, pw_flags, out) : lp_encode_telem4(veh, seq, t, pw_vaux, pw_flags, out);
    if (LP_LBAT_TAIL_DUE(lbat, pw_flags))              /* the carrier battery build: TELEM v7 (46 B) / v6 (40 B) */
        return v3 ? lp_encode_telem7(veh, seq, t, lbat, out) : lp_encode_telem6(veh, seq, t, lbat, out);
    return v3 ? lp_encode_telem3(veh, seq, t, out) : lp_encode_telem2(veh, seq, t, out);   /* TELEM v3 45 B / v2 39 B */
}
