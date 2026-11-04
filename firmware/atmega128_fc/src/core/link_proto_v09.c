/* link_proto_v09.c - see link_proto_v09.h (v0.9 RP1: TELEM v4 / v5). */
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
