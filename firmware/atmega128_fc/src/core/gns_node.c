/* gns_node.c - see gns_node.h (mirrors sim/glink_proto.py GnsNode, result_from, gps_data_from, pos_inputs_from). */
#include "gns_node.h"

#include "link_proto.h"

void gns_init(gns_t *g, const gps_nav_params_t *gp, const pos_params_t *pp, bool configure, uint16_t n_wait, uint16_t n_ack,
              uint16_t n_settle, uint16_t n_ver, uint32_t baud)
{
    gps_nav_init(&g->gps, gp, configure, n_wait, n_ack, n_settle, n_ver, baud);
    pos_init(&g->pos, pp);
    gl_decoder_init(&g->dec, GL_ACCEPT_STATE);
    g->n = gp->step_ticks;
    g->has_nt = false;
    g->nt = 0;
    g->has_kstate = false;
    g->k_state = 0;
    g->has_last = false;
    g->fuse_due = false;
    g->fuse_k = 0;
    g->cycles = 0;
    g->st.states = g->st.hold_steps = g->st.gaps = g->st.restarts = 0;
    g->st.imu_bad = g->st.clock_slips = g->st.misaligned = 0;
}

void gns_poll(gns_t *g, uint32_t nt, const uint8_t *rx, uint16_t n_rx, gps_cfg_action_t *act)
{
    if (g->fuse_due)                                   /* the last cycle's fusion first (the reference's order) */
        (void)gns_finish(g);
    g->nt = nt;
    g->has_nt = true;
    gps_nav_poll(&g->gps, nt, rx, n_rx, act);
}

void gns_pos_inputs(const gl_state_t *st, pos_inputs_t *pin)
{
    const uint8_t pf = st->pflags;
    pin->phase = (uint8_t)(pf & NL_PF_PHASE_MASK);
    pin->motor_on = (pf & NL_PF_MOTOR_ON) != 0u;
    pin->h_m = st->h;
    pin->cos_tilt = 1.0f - 2.0f * (st->q[1] * st->q[1] + st->q[2] * st->q[2]);   /* State.cos_tilt */
    pin->rate[0] = st->w[0];
    pin->rate[1] = st->w[1];
    pin->rate[2] = st->w[2];
    pin->k_t = nl_kt_value(st->kt_q);
    pin->vc_sat_high = (pf & NL_PF_SAT_HI) != 0u;
    pin->data = 0;
    pin->set = 0;
    pin->vc_low_alt = (pf & NL_PF_LOW_ALT) != 0u;
    pin->vc_ground = (pf & NL_PF_GROUND) != 0u;
    pin->lifted = (pf & NL_PF_LIFTED) != 0u;
}

void gns_pos_data(const gps_filter_t *f, pos_data_t *d)
{
    uint8_t i;
    d->flags = f->snap.flags;
    d->reset_count = f->reset_count;
    for (i = 0; i < 2u; i++) {
        d->prf[i] = f->p_[i] - f->off[i];              /* the reference's d.p - d.off (float state, not quantised) */
        d->off[i] = f->off[i];
        d->v[i] = f->v[i];
    }
    d->cs[0] = f->c_psi;
    d->cs[1] = f->s_psi;
}

void gns_result_of(const gns_t *g, gl_result_t *r)
{
    const gps_filter_t *f = &g->gps.filter;
    const pos_pub_t *po = &g->pos.last;
    float px = f->p_[0], py = f->p_[1];
    if (po->engaged) {
        r->tilt[0] = nl_tilt_q(po->tilt_l[0]);
        r->tilt[1] = nl_tilt_q(po->tilt_l[1]);
        r->trim[0] = nl_tilt_q(po->trim_l[0]);
        r->trim[1] = nl_tilt_q(po->trim_l[1]);
    } else {
        r->tilt[0] = NL_TILT_NONE;
        r->tilt[1] = 0;
        r->trim[0] = r->trim[1] = 0;
    }
    if (po->has_hold) {                                /* POS relative to the hold point (the FCU's TELEM distance) */
        px = px - po->hold[0];
        py = py - po->hold[1];
    }
    r->flags = f->snap.flags;
    r->num_sv = f->snap.num_sv;
    r->hacc_dm = f->snap.hacc_dm;
    r->pc = nl_pc_byte(po->mode, po->has_hold, (uint8_t)(po->requests & 0x0Fu));
    r->pos[0] = (int16_t)lp_quantize(px, 100u, LP_I16_MIN, LP_I16_MAX);
    r->pos[1] = (int16_t)lp_quantize(py, 100u, LP_I16_MIN, LP_I16_MAX);
    r->vel[0] = (int16_t)lp_quantize(f->v[0], 100u, LP_I16_MIN, LP_I16_MAX);
    r->vel[1] = (int16_t)lp_quantize(f->v[1], 100u, LP_I16_MIN, LP_I16_MAX);
}

void gns_on_state(gns_t *g, uint8_t seq, const gl_state_t *st, uint8_t out[GL_RESULT_FRAME_BYTES])
{
    gps_filter_t *f = &g->gps.filter;
    const uint32_t k = st->tick;
    pos_inputs_t pin;
    pos_data_t d;
    gl_result_t r;
    int64_t dk;
    uint8_t nf;
    if (g->fuse_due)
        (void)gns_finish(g);
    g->st.states++;
    if (g->has_nt) {                                   /* 1. the NAV-tick clock: the frame arrives in the NAV's tick k + 1 */
        const int64_t slip = (int64_t)g->nt - ((int64_t)k + 1);
        if (slip > GL_CLOCK_SLIP_TICKS || slip < -GL_CLOCK_SLIP_TICKS) {
            g->st.clock_slips++;
            g->gps.clock.has_offset = false;           /* the iTOW offset was learnt on the old clock: relearn */
        }
    }
    g->nt = k + 1u;
    g->has_nt = true;
    gns_pos_inputs(st, &pin);
    if (!g->has_kstate) {                              /* 2. the gap rules */
        dk = g->n;
    } else {
        dk = (int64_t)k - (int64_t)g->k_state;
        if (dk <= 0 || dk > GL_GAP_RESTART_TICKS) {
            g->st.restarts++;
            gps_filter_restart_timebase(f);
            g->gps.clock.has_offset = false;
            dk = g->n;
        } else if (dk % g->n) {
            g->st.misaligned++;
        }
    }
    if (!(st->nflags & GL_NF_IMU_OK)) {                /* frozen IMU data: like a lost frame (one hold step) */
        g->st.imu_bad++;
        g->st.hold_steps++;
        (void)pos_step(&g->pos, &pin);
        if (g->has_last) {
            (void)gl_encode(GL_TYPE_RESULT, seq, g->last, out);
        } else {
            gl_result_t z;
            z.tilt[0] = NL_TILT_NONE;
            z.tilt[1] = 0;
            z.flags = z.num_sv = z.pc = 0;
            z.hacc_dm = (uint8_t)LP_HACC_NONE;
            z.pos[0] = z.pos[1] = z.vel[0] = z.vel[1] = z.trim[0] = z.trim[1] = 0;
            (void)gl_encode_result(seq, &z, out);
        }
        return;
    }
    {
        const int64_t missed = dk / g->n - 1;
        if (missed > 0) {
            g->st.gaps++;
            g->st.hold_steps = (uint16_t)(g->st.hold_steps + (uint16_t)missed);
            (void)pos_hold_steps(&g->pos, &pin, (uint16_t)missed);
        }
    }
    g->k_state = k;
    g->has_kstate = true;
    nf = st->n > 0u ? st->n : g->n;                    /* 3. the filter STEP over dk ticks with the frame's mean force */
    gps_filter_gns_step(f, k, st->fsum, nf, (uint32_t)dk, st->q, st->w, (st->nflags & GL_NF_PAD) != 0u);
    gns_pos_data(f, &d);                               /* 4. the position step on the latched state */
    pin.data = &d;
    (void)pos_step(&g->pos, &pin);
    gns_result_of(g, &r);                              /* 5. the RESULT */
    gl_pack_result(&r, g->last);
    g->has_last = true;
    g->cycles++;
    (void)gl_encode(GL_TYPE_RESULT, seq, g->last, out);
    g->fuse_due = true;                                /* 6. after the TX: gns_finish */
    g->fuse_k = k + 1u;
}

bool gns_rx_byte(gns_t *g, uint8_t b, uint8_t out[GL_RESULT_FRAME_BYTES])
{
    gl_state_t st;
    if (gl_decoder_byte(&g->dec, b) != GL_TYPE_STATE)
        return false;
    gl_unpack_state(g->dec.buf, &st);
    gns_on_state(g, g->dec.seq, &st, out);
    return true;
}

bool gns_finish(gns_t *g)
{
    if (!g->fuse_due)
        return false;
    g->fuse_due = false;
    return gps_filter_gns_fuse(&g->gps.filter, g->fuse_k);
}
