/* bmp280_math.c - see bmp280_math.h.  The statements follow the datasheet code (3.11.3) line by line. */
#include "bmp280_math.h"

static uint16_t le16(const uint8_t *r)
{
    return (uint16_t)(((uint16_t)r[1] << 8) | r[0]);
}

void bmp280_calib_parse(const uint8_t raw[BMP280_CALIB_LEN], bmp280_calib_t *c)
{
    c->t1 = le16(raw + 0);
    c->t2 = (int16_t)le16(raw + 2);
    c->t3 = (int16_t)le16(raw + 4);
    c->p1 = le16(raw + 6);
    c->p2 = (int16_t)le16(raw + 8);
    c->p3 = (int16_t)le16(raw + 10);
    c->p4 = (int16_t)le16(raw + 12);
    c->p5 = (int16_t)le16(raw + 14);
    c->p6 = (int16_t)le16(raw + 16);
    c->p7 = (int16_t)le16(raw + 18);
    c->p8 = (int16_t)le16(raw + 20);
    c->p9 = (int16_t)le16(raw + 22);
}

bool bmp280_calib_ok(const bmp280_calib_t *c)
{
    if (c->t1 == 0u || c->p1 == 0u)
        return false;
    if (c->t1 == 0xFFFFu && c->p1 == 0xFFFFu && c->t2 == -1 && c->p2 == -1)
        return false;
    return true;
}

uint32_t bmp280_raw20(const uint8_t r[3])
{
    return ((uint32_t)r[0] << 12) | ((uint32_t)r[1] << 4) | (uint32_t)(r[2] >> 4);
}

int32_t bmp280_t_fine(const bmp280_calib_t *c, uint32_t adc_t)
{
    const int32_t a = (int32_t)adc_t;
    const int32_t var1 = ((((a >> 3) - ((int32_t)c->t1 << 1))) * ((int32_t)c->t2)) >> 11;
    const int32_t d = (a >> 4) - (int32_t)c->t1;
    const int32_t var2 = (((d * d) >> 12) * ((int32_t)c->t3)) >> 14;
    return var1 + var2;
}

int32_t bmp280_temp_c100(int32_t t_fine)
{
    return (t_fine * 5 + 128) >> 8;
}

bool bmp280_p_part1(const bmp280_calib_t *c, int32_t t_fine, bmp280_pwork_t *w)
{
    const int64_t v = (int64_t)t_fine - 128000;
    const int64_t vv = v * v;
    int64_t var1, var2;
    var2 = vv * (int64_t)c->p6;
    var2 = var2 + ((v * (int64_t)c->p5) << 17);
    var2 = var2 + (((int64_t)c->p4) << 35);
    var1 = ((vv * (int64_t)c->p3) >> 8) + ((v * (int64_t)c->p2) << 12);
    var1 = ((((int64_t)1 << 47) + var1) * (int64_t)c->p1) >> 33;
    w->var1 = var1;
    w->var2 = var2;
    return var1 != 0;
}

uint32_t bmp280_p_part2(const bmp280_calib_t *c, uint32_t adc_p, const bmp280_pwork_t *w)
{
    int64_t p = 1048576 - (int64_t)adc_p;
    int64_t var1, var2;
    p = (((p << 31) - w->var2) * 3125) / w->var1;
    var1 = (((int64_t)c->p9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)c->p8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)c->p7) << 4);
    return (uint32_t)p;
}

bool bmp280_compensate(const bmp280_calib_t *c, uint32_t adc_p, uint32_t adc_t, int32_t *temp_c100, uint32_t *p_q8)
{
    bmp280_pwork_t w;
    const int32_t t_fine = bmp280_t_fine(c, adc_t);
    if (!bmp280_p_part1(c, t_fine, &w))
        return false;
    *temp_c100 = bmp280_temp_c100(t_fine);
    *p_q8 = bmp280_p_part2(c, adc_p, &w);
    return true;
}
