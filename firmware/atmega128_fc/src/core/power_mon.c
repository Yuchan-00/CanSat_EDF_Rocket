/* power_mon.c - see power_mon.h (mirrors sim/onboard.py PowerMonitor line by line). */
#include "power_mon.h"

#if PM_ANY

#if PM_PARAMS_FLASH
#define PMR(x) FC_FLASH_RD(x)
#else
#define PMR(x) (x)
#endif

void pm_init(pm_t *m)
{
    m->fl = 0;
#if PM_WIRED
    m->y_in = 0;
    m->sag_cnt = 0;
    m->rel_cnt = 0;
    m->u_cap = 0.0f;
#endif
#if PM_BUF
    m->y_tet = m->y_buf = 0;
    m->lost_cnt = m->buf_cnt = m->d2_cnt = 0;
    m->res_cnt = 0;
#endif
}

uint16_t pm_cal_code(const pm_params_t *p, uint8_t ch, uint16_t raw)
{
    const uint16_t g = PMR(p->cal_gain_q15[ch]);
    int16_t c;
    if (g == 32768u && PMR(p->cal_offset[ch]) == 0)
        return raw > 1023u ? 1023u : raw;                 /* identity (no calibration): no multiply */
    c = (int16_t)(((uint32_t)raw * g + 16384UL) >> 15) + PMR(p->cal_offset[ch]);
    if (c < 0)
        return 0u;
    return c > 1023 ? 1023u : (uint16_t)c;
}

/* y = 16 x on the first sample, then y += (16 x - y) >> shift; no sample: unchanged */
static int16_t filt(int16_t y, bool have, uint16_t code, uint8_t shift)
{
    const int16_t x = (int16_t)(code << 4);
    if (!have)
        return x;
    return (int16_t)(y + ((int16_t)(x - y) >> shift));
}

static void reset_flight(pm_t *m)
{
#if PM_WIRED
    m->sag_cnt = 0;
    m->rel_cnt = 0;
#endif
#if PM_BUF
    m->lost_cnt = m->buf_cnt = m->d2_cnt = 0;
    m->res_cnt = 0;
#endif
    m->fl &= (uint8_t)~(PM_F_SAG_WARN | PM_F_LOST);       /* u_cap None: the warning bit is its validity */
}

static uint8_t inc8(uint8_t n) { return n < 0xFFu ? (uint8_t)(n + 1u) : n; }
static uint16_t inc16p(uint16_t n) { return n < 0xFFFFu ? (uint16_t)(n + 1u) : n; }

uint8_t pm_sample(pm_t *m, const pm_params_t *p, const pm_inputs_t *in, bool powered, float throttle_prev)
{
    const uint8_t sh = PMR(p->shift), build = PM_BUILD(p);
    uint8_t note = PM_NOTE_NONE;
#if PM_WIRED
    if (in->code[PM_CH_IN] != PM_CODE_NONE) {
        m->y_in = filt(m->y_in, (m->fl & PM_F_HAVE_IN) != 0u, pm_cal_code(p, PM_CH_IN, in->code[PM_CH_IN]), sh);
        m->fl |= PM_F_HAVE_IN;
    }
#endif
#if PM_BUF
    if (in->code[PM_CH_TET] != PM_CODE_NONE) {
        m->y_tet = filt(m->y_tet, (m->fl & PM_F_HAVE_TET) != 0u, pm_cal_code(p, PM_CH_TET, in->code[PM_CH_TET]), sh);
        m->fl |= PM_F_HAVE_TET;
    }
    if (in->code[PM_CH_BUF] != PM_CODE_NONE) {
        m->y_buf = filt(m->y_buf, (m->fl & PM_F_HAVE_BUF) != 0u, pm_cal_code(p, PM_CH_BUF, in->code[PM_CH_BUF]), sh);
        m->fl |= PM_F_HAVE_BUF;
    }
#endif
    if (in->buf_i_pos)
        m->fl |= PM_F_BUF_SHARE;
    else
        m->fl &= (uint8_t)~PM_F_BUF_SHARE;
    if (!powered) {
        reset_flight(m);
        return note;
    }
#if PM_WIRED
    if (build == PWR_SRC_WIRED && (m->fl & PM_F_HAVE_IN)) {
        const int16_t y = m->y_in;
        if (y < (int16_t)PMR(p->sag_q))
            m->sag_cnt = inc16p(m->sag_cnt);
        else if (y > (int16_t)PMR(p->sag_clear_q))
            m->sag_cnt = 0;
        if (!(m->fl & PM_F_SAG_WARN) && m->sag_cnt >= PMR(p->sag_warn_ticks)) {
            m->fl |= PM_F_SAG_WARN;
            m->u_cap = fc_maxf(throttle_prev, 0.0f);      /* max(float(throttle_prev), 0.0) */
            m->rel_cnt = 0;
            note = PM_NOTE_SAG_WARN;
        } else if (m->fl & PM_F_SAG_WARN) {
            m->rel_cnt = y > (int16_t)PMR(p->sag_clear_q) ? (uint8_t)(m->rel_cnt + 1u) : 0u;   /* < release <= 255 */
            if (m->rel_cnt >= PMR(p->sag_release_ticks)) {
                m->fl &= (uint8_t)~PM_F_SAG_WARN;
                m->rel_cnt = 0;
                note = PM_NOTE_SAG_CLEAR;
            }
        }
        return note;
    }
#endif
#if PM_BUF
    if (build == PWR_SRC_WIRED_BUFFER && (m->fl & PM_F_HAVE_TET) && (m->fl & PM_F_HAVE_BUF)) {
        const int16_t yt = m->y_tet, yb = m->y_buf;
        const bool lost_now = yt < (int16_t)PMR(p->lost_q) || yt + (int16_t)PMR(p->lost_margin_q) < yb;
        m->lost_cnt = lost_now ? inc8(m->lost_cnt) : 0u;
        if (m->lost_cnt >= PMR(p->lost_ticks))
            m->fl |= PM_F_LOST;
        m->buf_cnt = yb < (int16_t)PMR(p->buf_low_q) ? inc8(m->buf_cnt) : 0u;
        m->res_cnt = yb < (int16_t)PMR(p->buf_res_q) ? inc16p(m->res_cnt) : 0u;
        m->d2_cnt = (yb > (int16_t)PMR(p->d2_q) && !lost_now) ? inc8(m->d2_cnt) : 0u;
    }
#endif
    (void)build;
    return note;
}

bool pm_sag_land(const pm_t *m, const pm_params_t *p)
{
#if PM_WIRED
    return m->sag_cnt >= PMR(p->sag_land_ticks);
#else
    (void)m; (void)p;
    return false;
#endif
}

bool pm_lost_cond(const pm_t *m, const pm_params_t *p)
{
#if PM_BUF
    return m->lost_cnt >= PMR(p->lost_ticks);
#else
    (void)m; (void)p;
    return false;
#endif
}

bool pm_buf_cond(const pm_t *m, const pm_params_t *p)
{
#if PM_BUF
    return m->buf_cnt >= PMR(p->buf_ticks) || m->res_cnt >= PMR(p->buf_res_ticks);
#else
    (void)m; (void)p;
    return false;
#endif
}

bool pm_d2_cond(const pm_t *m, const pm_params_t *p)
{
#if PM_BUF
    return m->d2_cnt >= PMR(p->d2_ticks);
#else
    (void)m; (void)p;
    return false;
#endif
}

bool pm_i8_buf(const pm_t *m, const pm_params_t *p)
{
#if PM_BUF
    return (m->fl & PM_F_HAVE_BUF) && m->y_buf < (int16_t)PMR(p->i8buf_q);
#else
    (void)m; (void)p;
    return false;
#endif
}

bool pm_arm_ok(const pm_t *m, const pm_params_t *p)
{
    const uint8_t build = PM_BUILD(p);
#if PM_WIRED
    if (build == PWR_SRC_WIRED)
        return (m->fl & PM_F_HAVE_IN) && (int16_t)PMR(p->arm_lo_q) <= m->y_in && m->y_in <= (int16_t)PMR(p->arm_hi_q);
#endif
#if PM_BUF
    if (build == PWR_SRC_WIRED_BUFFER)
        return (m->fl & PM_F_HAVE_TET) && (m->fl & PM_F_HAVE_BUF) && (int16_t)PMR(p->arm_lo_q) <= m->y_tet &&
               m->y_tet <= (int16_t)PMR(p->arm_hi_q) && (int16_t)PMR(p->buf_arm_lo_q) <= m->y_buf &&
               m->y_buf <= (int16_t)PMR(p->buf_arm_hi_q);
#endif
    (void)m; (void)build;
    return true;
}

uint8_t pm_flags(const pm_t *m, const pm_params_t *p)
{
    const uint8_t build = PM_BUILD(p);
    uint8_t f = 0;
#if PM_WIRED
    if (build == PWR_SRC_WIRED) {
        if ((m->fl & PM_F_HAVE_IN) && m->y_in >= (int16_t)PMR(p->lost_q))
            f |= PM_PF_TETHER_OK;
        if (m->fl & PM_F_SAG_WARN)
            f |= PM_PF_SAG;
    }
#endif
#if PM_BUF
    if (build == PWR_SRC_WIRED_BUFFER) {
        const bool tb = (m->fl & PM_F_HAVE_TET) && (m->fl & PM_F_HAVE_BUF);
        if (tb && !(m->y_tet < (int16_t)PMR(p->lost_q) || m->y_tet + (int16_t)PMR(p->lost_margin_q) < m->y_buf))
            f |= PM_PF_TETHER_OK;
        if ((m->fl & PM_F_HAVE_BUF) && m->y_buf < (int16_t)PMR(p->buf_low_q))
            f |= PM_PF_BUF_LOW;
        if (m->fl & PM_F_BUF_SHARE)
            f |= PM_PF_BUF_SHARE;
        if (m->fl & PM_F_LOST)
            f |= PM_PF_LOST;
        if ((m->fl & PM_F_HAVE_BUF) && m->y_buf > (int16_t)PMR(p->d2_q))
            f |= PM_PF_D2;
    }
#endif
    return (uint8_t)((f & 0x3Fu) | ((build & 0x03u) << PM_PF_BUILD_SHIFT));
}

uint8_t pm_vaux_q(const pm_t *m, const pm_params_t *p)
{
    const uint8_t build = PM_BUILD(p);
    int16_t y = -1;
#if PM_BUF
    if (build == PWR_SRC_WIRED_BUFFER && (m->fl & PM_F_HAVE_BUF))
        y = m->y_buf;
#endif
#if PM_WIRED
    if (build != PWR_SRC_WIRED_BUFFER && (m->fl & PM_F_HAVE_IN))
        y = m->y_in;
#endif
    (void)build;
    if (y <= 0)
        return 0u;                                         /* quant_v(None) / v <= 0 -> 0 */
    {
        const uint32_t q = ((uint32_t)y * PMR(p->vaux_mul) + PMR(p->vaux_add)) >> PMR(p->vaux_shift);
        return q > 255u ? 255u : (uint8_t)q;
    }
}

#endif /* PM_ANY */
