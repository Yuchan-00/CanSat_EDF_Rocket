/* vertical_ctrl.c - see vertical_ctrl.h (mirrors sim/vertical_ctrl.py VerticalController v2 step by step).
 *
 * v0.7.1: the altitude reference is the reference's exact int32 micrometre count (one int32 add per tick), replacing
 * v0.7.0's compensated float32 sum; every step, every comparison on it and at_target are now identical in both
 * languages.  The only conversion of a measured float, vc_m_to_um(h) at a segment start, rounds the exact product.
 * Costs cut for the AVR (architect, v0.7.1 decision 11): cos(tilt) from the NAV quaternion (no cosf), u_ff =
 * u_hover / sqrtf(k_T c) for throttle_exponent 2 (powf only as the fallback for another exponent), r = (u inv_u_hover)^2,
 * the reciprocals (1 / u_hover, vz_ki dt, loop_hz 1e-6) precomputed by the generator, the take-off ramp's quotient only
 * while the ramp runs.
 * v0.7.1b (architect after the physics review): the thrust learning k_T (one float division per learning tick, g r
 * formed once and shared with the support signature) and the band gating of the touchdown detector (a fourth counter).
 * RAM: vc_params_t +8 B (4 learning constants in, the fallback's -1/n and the ramp's 1/ramp_ticks out - both are only
 * used on ticks that do not learn), vc_t +6 B (k_T, the fourth counter).
 *
 * Every other expression keeps the reference's operand order (host build: -ffp-contract=off, like avr-gcc), and min /
 * max keep Python's NaN semantics (max(a, b) = b if b > a else a). */
#include "vertical_ctrl.h"

#if VC_PARAMS_FLASH                                    /* v0.8.1 RAM diet: the parameters stay in flash */
#define VC_PAR(c) ((c)->p)
#define VPR(x) FC_FLASH_RD(x)
#else
#define VC_PAR(c) (&(c)->p)
#define VPR(x) (x)
#endif

static bool vc_isfinite(float x)
{
    union {
        float f;
        uint32_t u;
    } v;
    v.f = x;
    return (v.u & 0x7F800000UL) != 0x7F800000UL;       /* exponent not all ones: not NaN, not +-inf */
}

#if FC_V09
static bool vc_nz(float x)                             /* x != 0 without a float compare (+0 / -0 are zero) */
{
    union {
        float f;
        uint32_t u;
    } v;
    v.f = x;
    return (v.u & 0x7FFFFFFFUL) != 0u;
}
#endif

static uint16_t inc16(uint16_t n)                      /* saturating counter (the reference's are unbounded) */
{
    return n < 0xFFFFu ? (uint16_t)(n + 1u) : n;
}

uint8_t vc_mode_of_phase(uint8_t phase)
{
    if (phase == LP_PHASE_CLIMB || phase == LP_PHASE_HOLD)
        return VC_MODE_FLY;
    if (phase == LP_PHASE_LAND)
        return VC_MODE_LAND;
    return VC_MODE_IDLE;
}

float vc_cos_tilt_from_q(const float q[4])
{
    const float x = q[1], y = q[2];
    return 1.0f - 2.0f * (x * x + y * y);
}

/* round_half_away(x 1e6) on the exact product: |x| = mant 2^(ex - 150) (mant 24 bits), 1e6 = 15625 2^6, so
 * |x| 1e6 = K 2^-n with K = mant 15625 < 2^38 and n = 144 - ex; round = (K + 2^(n-1)) >> n, computed on K = A 2^16 + B
 * in uint32 halves (A < 2^22, B < 2^16).  n <= 6 means |x| 1e6 >= 2^23 15625 2^-6 > 2e9 (saturates), n >= 39 means
 * |x| 1e6 < 0.5 (0). */
int32_t vc_m_to_um(float x)
{
    union {
        float f;
        uint32_t u;
    } v;
    uint32_t mant, lo, a, b, r;
    int16_t n;
    uint8_t ex;
    bool neg;
    v.f = x;
    neg = (v.u >> 31) != 0u;
    ex = (uint8_t)(v.u >> 23);
    if (ex == 0xFFu)                                   /* +-inf saturate, NaN -> 0 */
        return (v.u & 0x7FFFFFu) ? 0 : (neg ? -VC_REF_LIMIT_UM : VC_REF_LIMIT_UM);
    if (ex == 0u)                                      /* zero / denormal */
        return 0;
    n = (int16_t)(144 - (int16_t)ex);
    if (n <= 6)
        return neg ? -VC_REF_LIMIT_UM : VC_REF_LIMIT_UM;
    if (n >= 39)
        return 0;
    mant = (v.u & 0x7FFFFFu) | 0x800000UL;
    lo = (mant & 0xFFFFu) * 15625UL;
    a = (mant >> 16) * 15625UL + (lo >> 16);
    b = lo & 0xFFFFu;
    if (n >= 17)
        r = (a + (1UL << (n - 17))) >> (n - 16);
    else if (n == 16)
        r = a + ((b + 0x8000UL) >> 16);
    else
        r = (a << (16 - n)) + ((b + (1UL << (n - 1))) >> n);
    if (r > (uint32_t)VC_REF_LIMIT_UM)
        r = (uint32_t)VC_REF_LIMIT_UM;
    return neg ? -(int32_t)r : (int32_t)r;
}

void vc_reset(vc_t *c)
{
    c->active = false;
    c->ref_um = 0;
    c->integ = 0.0f;
    c->ramp_k = 0;
    c->cnt[0] = c->cnt[1] = c->cnt[2] = c->cnt[3] = 0;
    c->floor_run = 0;
    c->low_in_land = 0;
    c->v_lpf = 0.0f;
    c->d_lpf = 0.0f;
    c->r_thr = 0.0f;
    c->u_last = 0.0f;
    c->k_t = fc_clampf(1.0f, VPR(VC_PAR(c)->kt_min), VPR(VC_PAR(c)->kt_max));   /* 1 at each take-off (kt_reset) */
    c->to_h0 = 0.0f;                                   /* v0.8 take-off check */
    c->to_run = 0;
    c->to_passed = false;
    c->to_h0_ok = false;
    c->to_settle = 0;
#if FC_V09
    c->airborne = false;                               /* v0.9 fix round 4c */
    c->in_land = false;                                /* 1c */
    c->fz_lpf = VPR(VC_PAR(c)->g);                     /* low-passed specific force until the lift-off */
    c->td_bias = 0.0f;                                 /* 5b */
#endif
}

void vc_init(vc_t *c, const vc_params_t *p)
{
#if VC_PARAMS_FLASH
    c->p = p;
#else
    c->p = *p;
#endif
    vc_reset(c);
}

uint16_t vc_takeoff_ticks(const vc_t *c)
{
    return VPR(VC_PAR(c)->takeoff_ticks);
}

/* the supervisor's target in integer decimetres -> um: dm x 1e5 (the reference's target_um = round_half_away(h_tgt x
 * 10) x 1e5 of h_tgt = dm / 10 is exactly that).  |dm| > 21000 saturates to +-2.1e9 (outside the int32 product): beyond
 * VC_REF_LIMIT_UM, so the reference can never reach it and every comparison with it goes the way of the exact value. */
static int32_t target_um(int16_t h_tgt_dm)
{
    const int32_t dm = h_tgt_dm;
    if (dm > 21000L)
        return 2100000000L;
    if (dm < -21000L)
        return -2100000000L;
    return dm * VC_UM_PER_DM;
}

void vc_update(vc_t *c, const vc_inputs_t *in, vc_outputs_t *o)
{
    const vc_params_t *p = VC_PAR(c);
    const uint8_t mode = vc_mode_of_phase(in->phase);
    const float h = in->h_m, vz = in->vz_mps;
    float h_ref, dh_ref, h_err, vz_cmd, cs, kc, u_ff, e, cap, i_try, base, u_try, u_raw, u, x, r_cmd, gr;
    int32_t delta;
    bool at_target = false, touchdown = false, lead, pinned, still, support, floor_, f_ok, cs_ok, learn, above, ramp_over;
    uint8_t sig = 0, td_by = VC_TD_NONE, j;
#if FC_V09
    uint8_t v9;
    uint16_t rt;
    bool was_land, searching = false;
#endif
    if (mode == VC_MODE_IDLE) {
        vc_reset(c);
        o->mode = VC_MODE_IDLE;
        o->sat = 0;
        o->at_target = o->touchdown = false;
        o->sig = 0;
        o->td_by = VC_TD_NONE;
        o->td_count = 0;
        o->low_in_land_ticks = 0;
        o->u = o->h_ref = o->vz_cmd = o->vz_err = o->u_ff = o->integ = o->dh_ref_mps = 0.0f;
        o->cos_tilt = 1.0f;                            /* the reference's IDLE_OUT default (k_T: c->k_t, reset) */
        return;
    }
    if (!c->active) {                                  /* 0. start */
        vc_reset(c);
        c->active = true;
        c->ref_um = vc_isfinite(h) ? vc_m_to_um(h) : 0;
        c->to_h0 = vc_isfinite(h) ? h : 0.0f;          /* v0.8 take-off check */
    }
#if FC_V09
    v9 = VPR(p->v9);
    rt = VPR(p->ramp_total_ticks);
    was_land = c->in_land;                             /* 1c: the previous tick was LAND */
    c->in_land = mode == VC_MODE_LAND;
#endif
    /* 1. reference (exact integer micrometres) and 2. vertical-speed command */
    if (mode == VC_MODE_FLY) {
        const int32_t tgt = target_um(in->h_tgt_dm);
        if (tgt >= c->ref_um - VPR(p->dn_um) && tgt <= c->ref_um + VPR(p->up_um))   /* -dn <= tgt - ref <= up, no overflow */
            delta = tgt - c->ref_um;                   /* lands exactly on the target */
        else
            delta = tgt > c->ref_um ? VPR(p->up_um) : -VPR(p->dn_um);
#if FC_V09
        {                                              /* 1b: first-order approach, the step <= max(|d| >> shift, min) */
            const uint8_t sh = VPR(p->approach_shift);
            if (sh != 0u && tgt != c->ref_um) {
                const uint32_t ad = tgt > c->ref_um ? (uint32_t)tgt - (uint32_t)c->ref_um
                                                    : (uint32_t)c->ref_um - (uint32_t)tgt;   /* |d| < 2^32, exact */
                const uint32_t mn = (uint32_t)VPR(p->approach_min_um);
                uint32_t lim = ad >> sh;
                if (lim < mn)
                    lim = mn;
                if ((uint32_t)(delta >= 0 ? delta : -delta) > lim)
                    delta = delta > 0 ? (int32_t)lim : -(int32_t)lim;
            }
        }
        if ((v9 & VC_V9_REF_HOLD) && !c->airborne && !c->to_passed && rt > 0u && (uint32_t)c->ramp_k + 1u < rt)
            delta = 0;                                 /* 4c: the reference waits on the pad for the lift-off */
#endif
        c->ref_um += delta;                            /* |ref| <= 2e9 + a step: no int32 overflow before the clamp */
        if (c->ref_um > VC_REF_LIMIT_UM)
            c->ref_um = VC_REF_LIMIT_UM;
        else if (c->ref_um < -VC_REF_LIMIT_UM)
            c->ref_um = -VC_REF_LIMIT_UM;
        at_target = c->ref_um == tgt;
    } else {                                           /* LAND (baro healthy or lost: the same law, no flare when lost) */
        bool low_ref;
#if FC_V09
        if ((v9 & VC_V9_LAND_REBASE) && !was_land && vc_isfinite(h)) {   /* 1c: ref = max(ref, h) entering LAND */
            const int32_t hu = vc_m_to_um(h);
            if (hu > c->ref_um)
                c->ref_um = hu;
        }
#endif
        low_ref = (c->ref_um <= VPR(p->flare_um) || h <= VPR(p->flare_altitude_m)) && !in->baro_lost;
        delta = -(low_ref ? VPR(p->td_um) : VPR(p->dn_um));
        c->ref_um += delta;
        if (c->ref_um < -VC_REF_LIMIT_UM)
            c->ref_um = -VC_REF_LIMIT_UM;
    }
    h_ref = (float)c->ref_um * 1e-6f;
    dh_ref = delta != 0 ? (float)delta * VPR(p->um_tick_to_mps) : 0.0f;   /* 0 at the target (v0.7.1b: no int->float there) */
    h_err = VPR(p->alt_kp) * (h_ref - h);                   /* the altitude-feedback part of the command */
    /* the reference's (ff dh_ref) + h_err without the add when the reference does not move (0 + h_err == h_err) */
    vz_cmd = fc_clampf((VPR(p->vz_ff) && delta != 0) ? dh_ref + h_err : h_err, -VPR(p->vz_max_down_mps), VPR(p->vz_max_up_mps));
    /* 3. throttle (k_T of the previous tick) */
    cs = in->cos_tilt;
    cs_ok = vc_isfinite(cs);
    if (!cs_ok)
        cs = 1.0f;
    cs = fc_clampf(cs, VPR(p->cos_tilt_min), 1.0f);
    kc = c->k_t * cs;
    u_ff = VPR(p->throttle_exp_int) == 2 ? VPR(p->u_hover) / sqrtf(kc) : VPR(p->u_hover) * powf(kc, -1.0f / VPR(p->throttle_exp));
    e = vz_cmd - vz;
    if (!vc_isfinite(e))
        e = 0.0f;
#if FC_V09
    {                                                  /* 3b: cap_at(k) - one or two slopes, u_max once the ramp is over */
        const uint32_t jj = (uint32_t)c->ramp_k + 1u;
        const uint16_t k1 = VPR(p->ramp_ticks);
        if (k1 == 0u || jj >= rt)
            cap = VPR(p->u_max);                       /* (the generator checks cap_at(rt - 1) == u_max) */
        else if (!(v9 & VC_V9_TWO_SLOPE))
            cap = VPR(p->u_max) * ((float)jj / (float)k1);
        else if (jj <= k1)
            cap = (float)jj * VPR(p->to_s1);           /* the pad segment: u_start (k + 1) / K1 */
        else
            cap = fc_minf(VPR(p->u_max), VPR(p->to_u_start) + (float)(jj - k1) * VPR(p->to_s2));
    }
#if PM_WIRED
    if (in->u_capped && in->u_cap < cap)               /* 3c W_SAG: the supervisor's frozen power ceiling */
        cap = in->u_cap;
#endif
    ramp_over = rt == 0u || (uint32_t)c->ramp_k + 1u >= rt;
    /* 3b take-off thrust search: FLY, the take-off check not passed, the ramp over, the loop asking for a climb that has
     * not happened (v_z <= search_vz); not once airborne with the take-off learning on (4c) */
    if ((v9 & VC_V9_BOOST) && mode == VC_MODE_FLY && !c->to_passed && ramp_over && e > 0.0f && !(vz > VPR(p->to_search_vz)) &&
        c->k_t > VPR(p->kt_min) && !(c->airborne && VPR(p->kt_alpha_to) > 0.0f)) {
        c->k_t = fc_maxf(c->k_t * VPR(p->to_search_f), VPR(p->kt_min));
        searching = true;
    }
#else
    if (VPR(p->ramp_ticks) == 0u || (uint32_t)c->ramp_k + 1u >= VPR(p->ramp_ticks))
        cap = VPR(p->u_max);
    else                                               /* the ramp's ticks only (no learning then) */
        cap = VPR(p->u_max) * ((float)(c->ramp_k + 1u) / (float)VPR(p->ramp_ticks));
    ramp_over = VPR(p->ramp_ticks) == 0u || (uint32_t)c->ramp_k + 1u >= VPR(p->ramp_ticks);
#endif
    i_try = fc_clampf(c->integ + VPR(p->vz_ki_dt) * e, -VPR(p->vz_i_max), VPR(p->vz_i_max));
    base = u_ff + VPR(p->vz_kp) * e;                        /* (u_ff + vz_kp e) + i: the reference's association, once */
    u_try = base + i_try;
    if (!((u_try > cap && e > 0.0f) || (u_try < VPR(p->u_min) && e < 0.0f))) {
        c->integ = i_try;                              /* conditional integration (anti-windup) */
        u_raw = u_try;
    } else {
        u_raw = base + c->integ;
    }
#if FC_V09
    if (vc_nz(c->td_bias))                             /* 5b: + td_bias (0 except in LAND and the one tick after it) */
        u_raw = u_raw + c->td_bias;
#endif
    /* u = Python min(max(u_raw, u_min), cap) (the ramp ceiling wins over u_min) and sat, with each float compare done
     * once (v0.7.1b cost cut; u_raw is finite: u_ff, e and the integrator are) */
    above = u_raw > cap;
    if (VPR(p->u_min) > u_raw) {                            /* max(u_raw, u_min) = u_min */
        const bool fits = VPR(p->u_min) <= cap;
        u = fits ? VPR(p->u_min) : cap;
        o->sat = (int8_t)(above ? 1 : (fits ? -1 : 0));
    } else {                                           /* max(u_raw, u_min) = u_raw */
        u = above ? cap : u_raw;
        o->sat = (int8_t)(above ? 1 : 0);
    }
    /* 6. take-off check (v0.8): the ramp over (cap = u_max) and pinned at it without having climbed above the h0 latched
     *    after the ramp + the settle time (physics-review fix) */
    if (VPR(p->takeoff_ticks) > 0u && !c->to_passed) {
#if FC_V09
        if (searching) {                               /* 3b: the thrust still rises - h0 waits for it */
            c->to_h0_ok = false;
            c->to_settle = 0;
        }
#endif
        if (!c->to_h0_ok && ramp_over) {
            if (c->to_settle >= VPR(p->takeoff_settle_ticks)) {
                c->to_h0 = vc_isfinite(h) ? h : 0.0f;
                c->to_h0_ok = true;
            } else {
                c->to_settle++;
            }
        }
        if (c->to_h0_ok && vc_isfinite(h) && h >= c->to_h0 + VPR(p->takeoff_climb_m)) {
            c->to_passed = true;
            c->to_run = 0;
        } else if (mode == VC_MODE_FLY && ramp_over && o->sat == 1) {
            if (c->to_run < 0xFFFFu)
                c->to_run++;
        } else {
            c->to_run = 0;
        }
    }
    if (VPR(p->takeoff_ticks) > 0u && c->to_run >= VPR(p->takeoff_ticks))
        sig |= VC_SIG_TAKEOFF_FAIL;
#if FC_V09
    if (c->ramp_k < rt)
        c->ramp_k++;
#else
    if (c->ramp_k < VPR(p->ramp_ticks))
        c->ramp_k++;
#endif
    /* 4. touchdown signals (u_last = the throttle applied since the previous tick) */
    x = c->u_last * VPR(p->inv_u_hover);
    r_cmd = VPR(p->throttle_exp_int) == 2 ? x * x : powf(x, VPR(p->throttle_exp));
    c->r_thr += VPR(p->thrust_alpha) * (r_cmd - c->r_thr);
    f_ok = vc_isfinite(in->f_z_mps2);
    learn = false;
    if (f_ok) {
        gr = VPR(p->g) * c->r_thr;                          /* the model's specific force at k_T = 1 [m/s^2] */
        /* 4b. thrust learning (v0.7.1b): free flight above the flare band, after the take-off ramp, small tilt (the
         * reference's conditions; ordered here so a landing in the band stops at the first float compare) */
#if FC_V09
        {
            const bool learn_to = VPR(p->kt_alpha_to) > 0.0f && c->airborne && !c->to_passed && mode == VC_MODE_FLY &&
                                  cs_ok && c->r_thr >= VC_KT_R_MIN && cs > VPR(p->kt_cos_min);   /* 4c */
            learn = c->ramp_k >= rt && cs_ok && h > VPR(p->flare_altitude_m) && cs > VPR(p->kt_cos_min) &&
                    c->r_thr >= VC_KT_R_MIN && VPR(p->kt_alpha) > 0.0f;
            if (learn || learn_to) {
                c->k_t += (learn_to ? VPR(p->kt_alpha_to) : VPR(p->kt_alpha)) * (in->f_z_mps2 / gr - c->k_t);
                c->k_t = fc_clampf(c->k_t, VPR(p->kt_min), VPR(p->kt_max));
                sig |= VC_SIG_KT_LEARN;
                learn = true;
            }
        }
#else
        learn = c->ramp_k >= VPR(p->ramp_ticks) && cs_ok && h > VPR(p->flare_altitude_m) && cs > VPR(p->kt_cos_min) &&
                c->r_thr >= VC_KT_R_MIN && VPR(p->kt_alpha) > 0.0f;
        if (learn) {
            c->k_t += VPR(p->kt_alpha) * (in->f_z_mps2 / gr - c->k_t);
            c->k_t = fc_clampf(c->k_t, VPR(p->kt_min), VPR(p->kt_max));
            sig |= VC_SIG_KT_LEARN;
        }
#endif
        c->d_lpf += VPR(p->support_alpha) * ((in->f_z_mps2 - c->k_t * gr) - c->d_lpf);
#if FC_V09
        if ((v9 & VC_V9_LIFTOFF) && !c->airborne && mode == VC_MODE_FLY) {   /* 4c lift-off: f_z low-passed above */
            c->fz_lpf += VPR(p->support_alpha) * (in->f_z_mps2 - c->fz_lpf);  /* (1 + airborne_fz) g (pad: f_z = g) */
            if (c->fz_lpf > VPR(p->to_airborne_mps2))
                c->airborne = true;
        }
#endif
    }
    (void)learn;
    if (vc_isfinite(vz))
        c->v_lpf += VPR(p->still_alpha) * (vz - c->v_lpf);
    lead = h_err <= -VPR(p->lead_mps);                      /* the vehicle does not follow the reference down */
    pinned = o->sat == -1 || (c->integ <= -VPR(p->vz_i_max) && vz_cmd <= -VPR(p->vz_max_down_mps));
    c->floor_run = pinned ? inc16(c->floor_run) : 0u;
    still = lead && fabsf(c->v_lpf) <= VPR(p->td_vz_still_mps);          /* lead first: one float compare less */
    support = f_ok && c->d_lpf > VPR(p->support_mps2);
    floor_ = c->floor_run >= VPR(p->floor_ticks);
    c->u_last = u;
    if (still)
        sig |= VC_SIG_STILL;
    if (support)
        sig |= VC_SIG_SUPPORT;
    if (floor_)
        sig |= VC_SIG_FLOOR;
    if (support || floor_)
        sig |= VC_SIG_GROUND;
    /* 5. touchdown detector (v0.7.1b gating: one signature in the flare band, all of them together above it) */
    if (mode == VC_MODE_LAND) {
        const bool low_alt = h <= VPR(p->flare_altitude_m);
        const bool low = low_alt || vz_cmd <= -VPR(p->vz_max_down_mps);
        bool still_ok = still;
        if (low)
            sig |= VC_SIG_LOW;
        if (low_alt)
            sig |= VC_SIG_LOW_ALT;
#if FC_V09
        if (v9 & VC_V9_TD_RELEASE) {                   /* 5b: release the thrust while still in the flare band */
            if (low_alt && still && !in->baro_lost)
                c->td_bias = fc_maxf(c->td_bias - VPR(p->td_step), -VPR(p->td_max));
            else
                c->td_bias = fc_minf(c->td_bias + VPR(p->td_step), 0.0f);
            still_ok = still && c->td_bias <= VPR(p->td_ok);   /* 'still' confirms only with the release complete */
        }
#endif
        c->cnt[0] = (low_alt && still_ok) ? inc16(c->cnt[0]) : 0u;
        c->cnt[1] = (low_alt && support) ? inc16(c->cnt[1]) : 0u;
        c->cnt[2] = (low_alt && floor_) ? inc16(c->cnt[2]) : 0u;
        c->cnt[3] = (low && support && floor_ && (still || in->baro_lost)) ? inc16(c->cnt[3]) : 0u;
        for (j = 0; j < 4u; j++) {
            if (c->cnt[j] >= VPR(p->landed_confirm_ticks)) {
                touchdown = true;
                td_by = (uint8_t)(j + 1u);
                break;
            }
        }
        c->low_in_land = (low_alt && lead && !in->baro_lost) ? inc16(c->low_in_land) : 0u;
    } else {
        c->cnt[0] = c->cnt[1] = c->cnt[2] = c->cnt[3] = 0;
        c->low_in_land = 0;
#if FC_V09
        c->td_bias = 0.0f;                             /* 5b: no release outside LAND */
#endif
    }
    o->mode = mode;
    o->at_target = at_target;
    o->touchdown = touchdown;
    o->sig = sig;
    o->td_by = td_by;
    o->td_count = c->cnt[0] > c->cnt[1] ? c->cnt[0] : c->cnt[1];
    if (c->cnt[2] > o->td_count)
        o->td_count = c->cnt[2];
    if (c->cnt[3] > o->td_count)
        o->td_count = c->cnt[3];
    o->low_in_land_ticks = c->low_in_land;
    o->u = u;
    o->h_ref = h_ref;
    o->vz_cmd = vz_cmd;
    o->vz_err = e;
    o->u_ff = u_ff;
    o->integ = c->integ;
    o->dh_ref_mps = dh_ref;
    o->cos_tilt = cs;
}
