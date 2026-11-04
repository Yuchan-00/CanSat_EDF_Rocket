/* ms5611_math.c - see ms5611_math.h */
#include "ms5611_math.h"

#include <math.h>

uint8_t ms5611_crc4(const uint16_t prom_in[8])
{
    uint16_t prom[8];
    uint16_t n_rem = 0;
    uint8_t cnt, n_bit;
    for (cnt = 0; cnt < 8; cnt++)
        prom[cnt] = prom_in[cnt];
    prom[7] = (uint16_t)(0xFF00u & prom[7]);
    for (cnt = 0; cnt < 16; cnt++) {
        if (cnt % 2 == 1)
            n_rem ^= (uint16_t)(prom[cnt >> 1] & 0x00FFu);
        else
            n_rem ^= (uint16_t)(prom[cnt >> 1] >> 8);
        for (n_bit = 8; n_bit > 0; n_bit--) {
            if (n_rem & 0x8000u)
                n_rem = (uint16_t)((n_rem << 1) ^ 0x3000u);
            else
                n_rem = (uint16_t)(n_rem << 1);
        }
    }
    n_rem = (uint16_t)(0x000Fu & (n_rem >> 12));
    return (uint8_t)n_rem;
}

bool ms5611_prom_ok(const uint16_t prom[8])
{
    bool all0 = true, all1 = true;
    uint8_t i;
    for (i = 1; i <= 6; i++) {
        if (prom[i] != 0x0000u) all0 = false;
        if (prom[i] != 0xFFFFu) all1 = false;
    }
    if (all0 || all1)
        return false;
    return ms5611_crc4(prom) == (prom[7] & 0x000Fu);
}

void ms5611_compensate(const uint16_t prom[8], uint32_t d1, uint32_t d2, int32_t *temp_c100, int32_t *p_pa)
{
    int32_t dT = (int32_t)d2 - ((int32_t)prom[5] << 8);
    int32_t temp = 2000 + (int32_t)(((int64_t)dT * prom[6]) / 8388608LL);          /* 2^23 */
    int64_t off = ((int64_t)prom[2] << 16) + ((int64_t)prom[4] * dT) / 128;        /* 2^7  */
    int64_t sens = ((int64_t)prom[1] << 15) + ((int64_t)prom[3] * dT) / 256;       /* 2^8  */
    if (temp < 2000) {
        int32_t t2 = (int32_t)(((int64_t)dT * dT) / 2147483648LL);                  /* 2^31 */
        int64_t dt2 = (int64_t)(temp - 2000) * (temp - 2000);
        int64_t off2 = 5 * dt2 / 2;
        int64_t sens2 = 5 * dt2 / 4;
        if (temp < -1500) {
            int64_t dt3 = (int64_t)(temp + 1500) * (temp + 1500);
            off2 += 7 * dt3;
            sens2 += 11 * dt3 / 2;
        }
        temp -= t2;
        off -= off2;
        sens -= sens2;
    }
    *temp_c100 = temp;
    *p_pa = (int32_t)((((int64_t)d1 * sens) / 2097152LL - off) / 32768LL);     /* 2^21, 2^15 */
}

float ms5611_pressure_altitude(float p_pa, float g0)
{
    const float expo = ISA_R_J_KGK * ISA_LAPSE_K_M / g0;
    return (ISA_T0_K / ISA_LAPSE_K_M) * (1.0f - powf(p_pa / ISA_P0_PA, expo));
}
