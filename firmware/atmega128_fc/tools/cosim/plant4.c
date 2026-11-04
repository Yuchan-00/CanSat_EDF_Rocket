/* plant4.c - the closed-loop plant of the four-core co-simulation (cosim4.c): see cosim4.h for the model and what it
 * leaves out.  Ported from the single-chip FCU bench's synthetic plant (src/simbench.c plant_step: a vertical point mass
 * flown by the ESC command through the EDF lag, thrust g (u / u_hover)^n per unit mass, the pad carrying the weight) and
 * extended by the attitude (the vane moments of the image's own mixer model on a rigid body) and the horizontal motion
 * (the thrust's tilt and a constant disturbance), so the NAV's sensors, the GNS's GPS and the FCU's controller close the
 * loop.  Host C99, double precision, explicit Euler at the step the harness calls it with (250 us).
 *
 * Invariant: inputs = the FCU's compare registers only; outputs = sensor truths and the antenna position only.
 */
#include <math.h>
#include <string.h>

#include "cosim4.h"

static double urand(uint64_t *s)                 /* xorshift64 -> [-1, 1) */
{
    uint64_t x = *s;
    x ^= x << 13;
    x ^= x >> 7;
    x ^= x << 17;
    *s = x;
    return (double)(x >> 11) * (2.0 / 9007199254740992.0) - 1.0;
}

static void q_from_euler(double roll, double pitch, double yaw, double q[4])
{
    const double cr = cos(0.5 * roll), sr = sin(0.5 * roll), cp = cos(0.5 * pitch), sp = sin(0.5 * pitch),
                 cy = cos(0.5 * yaw), sy = sin(0.5 * yaw);
    q[0] = cr * cp * cy + sr * sp * sy;
    q[1] = sr * cp * cy - cr * sp * sy;
    q[2] = cr * sp * cy + sr * cp * sy;
    q[3] = cr * cp * sy - sr * sp * cy;
}

/* v_out = R(q) v (body -> level) */
static void q_rot(const double q[4], const double v[3], double o[3])
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    o[0] = (1 - 2 * (y * y + z * z)) * v[0] + 2 * (x * y - w * z) * v[1] + 2 * (x * z + w * y) * v[2];
    o[1] = 2 * (x * y + w * z) * v[0] + (1 - 2 * (x * x + z * z)) * v[1] + 2 * (y * z - w * x) * v[2];
    o[2] = 2 * (x * z - w * y) * v[0] + 2 * (y * z + w * x) * v[1] + (1 - 2 * (x * x + y * y)) * v[2];
}

/* v_out = R(q)^T v (level -> body) */
static void q_rot_t(const double q[4], const double v[3], double o[3])
{
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    o[0] = (1 - 2 * (y * y + z * z)) * v[0] + 2 * (x * y + w * z) * v[1] + 2 * (x * z - w * y) * v[2];
    o[1] = 2 * (x * y - w * z) * v[0] + (1 - 2 * (x * x + z * z)) * v[1] + 2 * (y * z + w * x) * v[2];
    o[2] = 2 * (x * z + w * y) * v[0] + 2 * (y * z - w * x) * v[1] + (1 - 2 * (x * x + y * y)) * v[2];
}

static double mass(const plant_par_t *par)
{
    return par->t_max * pow(par->u_hover, par->n_exp) / par->g0;
}

static void ground(plant_t *p, const plant_par_t *par)
{
    const double up[3] = {0.0, 0.0, par->g0};
    p->h = p->vz = p->vx = p->vy = 0.0;
    q_from_euler(par->pad_roll, par->pad_pitch, 0.0, p->q);
    p->w[0] = p->w[1] = p->w[2] = 0.0;
    p->airborne = 0;
    q_rot_t(p->q, up, p->f_b);                   /* the pad carries the weight: the accelerometer reads 1 g up */
}

void plant_init(plant_t *p, const plant_par_t *par)
{
    memset(p, 0, sizeof *p);
    p->rng = par->seed ? par->seed : 0x9E3779B97F4A7C15ull;
    ground(p, par);
}

void plant_step(plant_t *p, const plant_par_t *par, const double pulse_us[5], double dt)
{
    const double m = mass(par);
    double u = 0.0, k, dx, dy, dz, mom[3], tb[3] = {0.0, 0.0, 0.0}, tw[3], a[3];
    int i;
    p->t += dt;
    if (pulse_us[4] > 0.0) {
        u = (pulse_us[4] - par->esc_min_us) / (par->esc_max_us - par->esc_min_us);
        u = u < 0.0 ? 0.0 : u > 1.0 ? 1.0 : u;
    }
    p->u_cmd = u;
    if (u > 0.05) p->motor_seen = 1;
    p->u_lag += (u - p->u_lag) * (dt / (par->tau + dt));
    p->thrust_n = par->t_max * pow(p->u_lag > 0.0 ? p->u_lag : 0.0, par->n_exp);
    for (i = 0; i < 4; i++)
        p->delta[i] = pulse_us[i] > 0.0 ? (pulse_us[i] - par->c0_us[i]) / par->k_us_per_rad[i] : 0.0;
    tb[2] = p->thrust_n;
    q_rot(p->q, tb, tw);
    if (!p->airborne) {
        if (tw[2] <= m * par->g0) {              /* on the ground: the weight is carried, nothing moves */
            ground(p, par);
            return;
        }
        p->airborne = 1;
        p->liftoffs++;
    }
    /* translation (level frame, z up) */
    a[0] = tw[0] / m + par->wind_ax - par->drag_k * p->vx;
    a[1] = tw[1] / m + par->wind_ay - par->drag_k * p->vy;
    a[2] = tw[2] / m - par->g0;
    p->vx += a[0] * dt;
    p->vy += a[1] * dt;
    p->vz += a[2] * dt;
    p->x += p->vx * dt;
    p->y += p->vy * dt;
    p->h += p->vz * dt;
    if (p->h > p->h_max) p->h_max = p->h;
    /* rotation: the vane moments of the image's forward mixer model, rigid body with a diagonal inertia */
    k = p->thrust_n * par->s_v * par->cl_a / (2.0 * par->a_e);
    dx = 0.5 * (p->delta[0] - p->delta[2]);
    dy = 0.5 * (p->delta[1] - p->delta[3]);
    dz = 0.25 * (p->delta[0] + p->delta[1] + p->delta[2] + p->delta[3]);
    mom[0] = 2.0 * par->l_v * k * dx;
    mom[1] = 2.0 * par->l_v * k * dy;
    mom[2] = 4.0 * par->r_v * k * dz;
    {
        const double I[3] = {par->ixx, par->iyy, par->izz};
        const double Iw[3] = {I[0] * p->w[0], I[1] * p->w[1], I[2] * p->w[2]};
        const double c[3] = {p->w[1] * Iw[2] - p->w[2] * Iw[1], p->w[2] * Iw[0] - p->w[0] * Iw[2],
                             p->w[0] * Iw[1] - p->w[1] * Iw[0]};
        double dq[4], n;
        for (i = 0; i < 3; i++) p->w[i] += (mom[i] - c[i]) / I[i] * dt;
        dq[0] = -0.5 * (p->q[1] * p->w[0] + p->q[2] * p->w[1] + p->q[3] * p->w[2]);
        dq[1] = 0.5 * (p->q[0] * p->w[0] + p->q[2] * p->w[2] - p->q[3] * p->w[1]);
        dq[2] = 0.5 * (p->q[0] * p->w[1] + p->q[3] * p->w[0] - p->q[1] * p->w[2]);
        dq[3] = 0.5 * (p->q[0] * p->w[2] + p->q[1] * p->w[1] - p->q[2] * p->w[0]);
        for (i = 0; i < 4; i++) p->q[i] += dq[i] * dt;
        n = sqrt(p->q[0] * p->q[0] + p->q[1] * p->q[1] + p->q[2] * p->q[2] + p->q[3] * p->q[3]);
        for (i = 0; i < 4; i++) p->q[i] /= n;
    }
    {
        const double tilt = plant_tilt_deg(p);
        if (tilt > p->tilt_max_deg) p->tilt_max_deg = tilt;
    }
    /* specific force in the body frame: R^T (a + g z) */
    {
        const double sf[3] = {a[0], a[1], a[2] + par->g0};
        q_rot_t(p->q, sf, p->f_b);
    }
    if (p->h <= 0.0 && p->vz < 0.0) {            /* touchdown */
        if (-p->vz > p->td_v) p->td_v = -p->vz;
        p->touchdowns++;
        ground(p, par);
    }
}

void plant_sensors(plant_t *p, const plant_par_t *par, double gyro[3], double accel[3], double *p_pa)
{
    /* ISA, the constants of src/core/ms5611_math.h: h = T0 / L (1 - (p / P0)^(R L / g)) */
    const double T0 = 288.15, L = 0.0065, R = 287.05287, P0 = 101325.0;
    const double expo = R * L / par->g0;
    const double h_g = T0 / L * (1.0 - pow(par->p_ground_pa / P0, expo));
    int i;
    for (i = 0; i < 3; i++) {
        gyro[i] = p->w[i] + par->gyro_noise * urand(&p->rng);
        accel[i] = p->f_b[i] + par->accel_noise * urand(&p->rng);
    }
    /* the thrust-induced baro error the image's altitude filter compensates (comp_m T / m g), in the air only */
    const double h_b = p->h + (p->airborne ? par->baro_comp_m * p->thrust_n / (mass(par) * par->g0) : 0.0);
    *p_pa = P0 * pow(1.0 - (h_g + h_b) * L / T0, 1.0 / expo) + par->baro_noise_pa * urand(&p->rng);
}

void plant_gps(const plant_t *p, const plant_par_t *par, double *north, double *east, double *vn, double *ve)
{
    double r[3], wl[3], vl[3];
    const double c = cos(par->psi0), s = sin(par->psi0);
    const double pos[3] = {p->x, p->y, p->h};
    const double wxl[3] = {p->w[1] * par->lever_b[2] - p->w[2] * par->lever_b[1],
                           p->w[2] * par->lever_b[0] - p->w[0] * par->lever_b[2],
                           p->w[0] * par->lever_b[1] - p->w[1] * par->lever_b[0]};
    double xl, yl, vxl, vyl;
    q_rot(p->q, par->lever_b, r);
    q_rot(p->q, wxl, wl);
    xl = pos[0] + r[0];
    yl = pos[1] + r[1];
    vl[0] = p->vx + wl[0];
    vl[1] = p->vy + wl[1];
    vl[2] = p->vz + wl[2];
    vxl = vl[0];
    vyl = vl[1];
    /* W (x north, y west) = Rz(psi0) level */
    *north = c * xl - s * yl;
    *east = -(s * xl + c * yl);
    *vn = c * vxl - s * vyl;
    *ve = -(s * vxl + c * vyl);
}

double plant_tilt_deg(const plant_t *p)
{
    const double cz = 1.0 - 2.0 * (p->q[1] * p->q[1] + p->q[2] * p->q[2]);
    return acos(cz > 1.0 ? 1.0 : cz < -1.0 ? -1.0 : cz) * 57.29577951308232;
}

void plant_euler(const plant_t *p, double *roll, double *pitch, double *yaw)
{
    const double w = p->q[0], x = p->q[1], y = p->q[2], z = p->q[3];
    const double sp = 2.0 * (w * y - z * x);
    *roll = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
    *pitch = asin(sp > 1.0 ? 1.0 : sp < -1.0 ? -1.0 : sp);
    *yaw = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
}
