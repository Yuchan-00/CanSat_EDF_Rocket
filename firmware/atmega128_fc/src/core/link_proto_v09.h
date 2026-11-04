/* link_proto_v09.h - v0.9 TELEM v4 / v5 power tail (fw09 SPEC section 6.1 RP1; sim/link_proto.py TYPE_TELEM4 /
 * TYPE_TELEM5, sim/power_link.py PowerTelem).  Started as the FCU port's prototype; RP1 adopted it into the reference
 * link (sim/link_proto.py: TYPEs 0x23 / 0x24, LEN_MAX 42) and wires it into the COM (com/com_link.c): the COM sends
 * TELEM v4 / v5 when the FCU's SNAP2 / SNAP3 power byte pw_flags carries a power build (LP_PWR_TAIL_DUE), TELEM v2 / v3
 * otherwise.
 *
 *   TELEM v4 = TELEM v2 payload (31 B) + v_aux u8 (0.125 V) + pwr_flags u8 (PF_* | build << 6): 33 B, 41-byte frame
 *   TELEM v5 = TELEM v3 payload (37 B) + the same 2 bytes: 39 B, 47-byte frame
 *
 * The FCU's ilink SNAP2 / SNAP3 carry the same 2 bytes (core/ilink_proto.h il_snap2_t / il_snap3_t pw_vaux / pw_flags,
 * from onb_tlm_src_t pw_vaux / pw_flags, filled by onboard_update in the power builds, 0 / 0 in the battery build); the
 * COM appends them unchanged.  Invariant: the tail bytes are the reference's PowerTelem.encode() exactly, and a frame
 * encoded here decodes in sim/link_proto.py to the Telemetry with the same v_aux_q / pwr_flags.  Not compiled into the
 * ballistic images (no caller; LP_RP1 0 has no TELEM4 / TELEM5 in its decoder). */
#ifndef FC_LINK_PROTO_V09_H
#define FC_LINK_PROTO_V09_H

#include "link_proto.h"

#define LP_V09_LEN_MAX (LP_HEADER_BYTES + LP_TELEM5_PAYLOAD_LEN)               /* 42 = LP_LEN_MAX of the v0.9 builds */
#if LP_RP1 && LP_V09_LEN_MAX != LP_LEN_MAX
#error "LP_LEN_MAX must take TELEM v5 (42) in the v0.9 builds"
#endif

/* the frames (lp_encode_frame; LP_MAX_PAYLOAD_LEN >= 39 in the v0.9 builds); return the frame length, 0 if impossible */
uint8_t lp_encode_telem4(uint8_t veh, uint8_t seq, const lp_telem_t *t, uint8_t v_aux, uint8_t pwr_flags,
                         uint8_t out[LP_TELEM4_FRAME_BYTES]);
uint8_t lp_encode_telem5(uint8_t veh, uint8_t seq, const lp_telem_t *t, uint8_t v_aux, uint8_t pwr_flags,
                         uint8_t out[LP_TELEM5_FRAME_BYTES]);
/* payloads -> fields (the ground bridge's / the host tests' decode; the TELEM v2 / v3 part as lp_unpack_telem2 / 3) */
void    lp_unpack_telem4(const uint8_t in[LP_TELEM4_PAYLOAD_LEN], lp_telem_t *t, uint8_t *v_aux, uint8_t *pwr_flags);
void    lp_unpack_telem5(const uint8_t in[LP_TELEM5_PAYLOAD_LEN], lp_telem_t *t, uint8_t *v_aux, uint8_t *pwr_flags);

#endif /* FC_LINK_PROTO_V09_H */
