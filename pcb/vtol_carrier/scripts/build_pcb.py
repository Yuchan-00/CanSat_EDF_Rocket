"""IR + kicad/placement.json -> unrouted ``vtol_carrier.kicad_pcb`` (KiCad 10).

Invariants
----------
* The board body comes from the AI_EDA engine's ``PCBCompiler`` (footprints embedded verbatim from the
  KiCad libraries on disk, pad nets and pin types from the IR / library symbol, deterministic UUIDs).
* This script only adds what the engine cannot express yet (README_ko.md, "engine gaps"):
  hierarchical sheet paths on footprints (schematic parity), DNP attributes, a rounded Edge.Cuts outline,
  documentation drawings of the module bodies on Dwgs.User, the ESP-12E antenna rule area (no copper, no
  vias, no parts on either layer: spec esp12e.antenna / board.keepouts), the board stackup (1 oz, 1.6 mm,
  finish) and the LOCKED pre-routed copper of the MCU support clusters (place.CLUSTER_ROUTES: VCC / AVCC
  decoupling and crystal nets, review F1 / F2) and of the ESP-12E VCC decoupling (pin 8 -> 100 nF -> own
  GND via) that Freerouting must keep. Everything is a pure function of the spec + placement -> byte-identical.
* Zones: GND on F.Cu and B.Cu over the whole board, written unfilled (KiCad fills them in route.py / DRC);
  through-hole pads get thermal reliefs, SMD pads connect solid (review F7). Exception (rev 2c, rev-2b review PF-M1):
  the THT pads that carry a whole supply into a plane connect SOLID to every zone (SOLID_PADS) - on rev 2b the Pololu
  5 V output reached the In2.Cu +5V plane through one 0.5 mm thermal spoke.
"""

from __future__ import annotations

import json
import shutil
import sys
from pathlib import Path

from ai_eda.compilers import ids
from ai_eda.compilers.base import CompileContext
from ai_eda.compilers.pcb import PCBCompiler
from ai_eda.ir import BoardOutline, BoardSide, Layer, PCBDesign, Placement, Provenance, ProvenanceKind, Zone
from ai_eda.tools.kicad import sexpr
from ai_eda.tools.kicad.library import KicadLibrary
from ai_eda.tools.kicad.sexpr import Q, S

import build_schematic as bs
import common

BX0, BY0, W, H = 62.0, 65.0, 76.0, 70.0
#: relayout step (review J2): the zone outlines stay 0.4 mm inside the two 76 mm edges (y 65 / y 135) that carry the
#: JLC edge rails (JLC: copper >= 0.4 mm from a V-cut edge, F from the reviewer's reading). The 0.3 mm board rule is
#: unchanged; the other two edges keep it.
RAIL_EDGE_PULLBACK = 0.4
CORNER_R = 1.0  # spec board.outline_mm.corner_radius_mm (A)


def placement_data() -> dict:
    import os

    src = Path(os.environ["CARRIER_PLACEMENT"]) if os.environ.get("CARRIER_PLACEMENT") else common.KICAD_DIR / "placement.json"
    return json.loads(src.read_text(encoding="utf-8"))


def ir_with_pcb(spec: dict, library: KicadLibrary):
    ir = common.build_ir(spec, library)
    pdata = placement_data()
    prov = Provenance(kind=ProvenanceKind.DERIVED, tool="scripts/place.py", note="deterministic placement (kicad/placement.json)",
                      created_at=common.FIXED_TIME)
    placements = [
        Placement(component_ref=ref, x_mm=v["x"], y_mm=v["y"], rotation_deg=float(v["rot"]),
                  side=BoardSide.TOP if v["side"] == "top" else BoardSide.BOTTOM, provenance=prov)
        for ref, v in pdata["placements"].items()
    ]
    iy = RAIL_EDGE_PULLBACK
    poly = [(BX0, BY0 + iy), (BX0 + W, BY0 + iy), (BX0 + W, BY0 + H - iy), (BX0, BY0 + H - iy)]
    zprov = Provenance(kind=ProvenanceKind.ASSUMPTION, note="GND pour on both outer layers (task: GND pour on both)", created_at=common.FIXED_TIME)
    zones = [Zone(net="GND", layer="F.Cu", polygon=poly, clearance_mm=0.25, provenance=zprov),
             Zone(net="GND", layer="B.Cu", polygon=poly, clearance_mm=0.25, provenance=zprov)]
    layers = [Layer(name="F.Cu", kind="signal"), Layer(name="B.Cu", kind="signal")]
    if common.LAYERS == 4:
        # 4-layer fallback (spec board.layer_plan.four_layer_fallback): In1.Cu solid GND, In2.Cu power (+5V pour; the
        # power-class tracks of +3V3 / SERVO_5V / GND_PWR / VBAT_LOGIC* may run on it, signals never - route.py)
        p4 = Provenance(kind=ProvenanceKind.USER_REQUIREMENT, created_at=common.FIXED_TIME,
                        note="user decision 2026-11-05: 4-layer fallback, F.Cu signal / In1.Cu GND / In2.Cu power / B.Cu signal")
        zones += [Zone(net="GND", layer="In1.Cu", polygon=poly, clearance_mm=0.25, provenance=p4),
                  Zone(net="+5V", layer="In2.Cu", polygon=poly, clearance_mm=0.3, provenance=p4)]
        layers = [Layer(name="F.Cu", kind="signal"), Layer(name="In1.Cu", kind="power"), Layer(name="In2.Cu", kind="power"),
                  Layer(name="B.Cu", kind="signal")]
    ir.pcb = PCBDesign(
        layers=layers,
        outline=BoardOutline(width_mm=W, height_mm=H, origin_x_mm=BX0, origin_y_mm=BY0),
        placements=placements,
        zones=zones,
    )
    return ir, pdata


def rounded_outline() -> list[list]:
    r = CORNER_R
    x0, y0, x1, y1 = BX0, BY0, BX0 + W, BY0 + H
    k = r * (1 - 0.7071067811865476)
    items = []
    lines = [((x0 + r, y0), (x1 - r, y0)), ((x1, y0 + r), (x1, y1 - r)), ((x1 - r, y1), (x0 + r, y1)), ((x0, y1 - r), (x0, y0 + r))]
    arcs = [((x1 - r, y0), (x1 - k, y0 + k), (x1, y0 + r)), ((x1, y1 - r), (x1 - k, y1 - k), (x1 - r, y1)),
            ((x0 + r, y1), (x0 + k, y1 - k), (x0, y1 - r)), ((x0, y0 + r), (x0 + k, y0 + k), (x0 + r, y0))]
    for i, (a, b) in enumerate(lines):
        items.append(S("gr_line", S("start", round(a[0], 4), round(a[1], 4)), S("end", round(b[0], 4), round(b[1], 4)),
                       S("stroke", S("width", 0.05), S("type", "solid")), S("layer", Q("Edge.Cuts")),
                       S("uuid", Q(ids.net_item_uuid(common.PROJECT_ID, "outline_line", i)))))
    for i, (a, m, b) in enumerate(arcs):
        items.append(S("gr_arc", S("start", round(a[0], 4), round(a[1], 4)), S("mid", round(m[0], 4), round(m[1], 4)),
                       S("end", round(b[0], 4), round(b[1], 4)), S("stroke", S("width", 0.05), S("type", "solid")),
                       S("layer", Q("Edge.Cuts")), S("uuid", Q(ids.net_item_uuid(common.PROJECT_ID, "outline_arc", i)))))
    return items


def drawings(pdata: dict) -> list[list]:
    out = []
    for i, b in enumerate(pdata["placement_checks"]["module_bodies"]):
        x0, y0, x1, y1 = b["rect"]
        out.append(S("gr_rect", S("start", x0, y0), S("end", x1, y1), S("stroke", S("width", 0.1), S("type", "dash")),
                     S("fill", False), S("layer", Q("Dwgs.User")), S("uuid", Q(ids.net_item_uuid(common.PROJECT_ID, "body_rect", i)))))
        out.append(S("gr_text", Q(b["name"].split(" (")[0] + f" - {b['rule']}"), S("at", round(x0 + 0.5, 3), round(y0 + 1.2, 3), 0),
                     S("layer", Q("Dwgs.User")), S("uuid", Q(ids.net_item_uuid(common.PROJECT_ID, "body_text", i))),
                     S("effects", S("font", S("size", 0.6, 0.6), S("thickness", 0.08)), S("justify", "left"))))
    # tie band
    out.append(S("gr_rect", S("start", 97.0, 65.0), S("end", 103.0, 135.0), S("stroke", S("width", 0.1), S("type", "dash_dot")),
                 S("fill", False), S("layer", Q("Dwgs.User")), S("uuid", Q(ids.net_item_uuid(common.PROJECT_ID, "tie_band")))))
    out.append(S("gr_text", Q("cable-tie band x -3..+3 mm: no components (J3 ESP flash, ISP J19-J22, reset-hold J24-J26 footprints unpopulated)"),
                 S("at", 97.3, 66.5, 90), S("layer", Q("Dwgs.User")), S("uuid", Q(ids.net_item_uuid(common.PROJECT_ID, "tie_text"))),
                 S("effects", S("font", S("size", 0.6, 0.6), S("thickness", 0.08)), S("justify", "right"))))
    notes = ["rev 3: on-board 5 V boost (U8 / L5 / C57 / C43 / C41) + 3.3 V LDO U6 in the +x/-y corner (the Pololu module J6 is removed)",
             "No parts on the bottom side (3 mm foam tape); the 5th ATmega128A is removed (user decision 2026-11-05)",
             "J3 ESP flash 2x3 footprint: unpopulated, top of the tie band beside the ESP-12F module (press a header in to flash)",
             "J14 (1S) + J15 (power switch) in the -x/+y corner next to battery spot A; J13 release header removed (VTOL)"]
    for i, t in enumerate(notes):
        out.append(S("gr_text", Q(t), S("at", 62.0, round(137.4 + 1.1 * i, 2), 0), S("layer", Q("Dwgs.User")),
                     S("uuid", Q(ids.net_item_uuid(common.PROJECT_ID, "note_text", i))),
                     S("effects", S("font", S("size", 0.7, 0.7), S("thickness", 0.09)), S("justify", "left"))))
    out.append(S("gr_text", Q("VTOL carrier rev 3 - NOT_VERIFIED: never built or powered"), S("at", 100.0, 136.2, 0), S("layer", Q("Dwgs.User")),
                 S("uuid", Q(ids.net_item_uuid(common.PROJECT_ID, "title_text"))),
                 S("effects", S("font", S("size", 0.8, 0.8), S("thickness", 0.1)))))
    return out


def antenna_keepout(pdata: dict) -> list:
    """ESP-12E antenna rule area: from the +y board edge to the end of the module's antenna part, 4 mm wider than the
    module on both sides; no tracks, vias, pads, pours or footprints on either copper layer (spec esp12e.antenna:
    footprint keep-out + 4 mm, A). The footprint's own keep-out (RF_Module:ESP-12E) stays inside it."""
    x0, y0, x1, y1 = pdata["antenna_rule_area"]
    pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    cu = [Q("F.Cu"), Q("B.Cu")] if common.LAYERS == 2 else [Q("F.Cu"), Q("In1.Cu"), Q("In2.Cu"), Q("B.Cu")]
    return S(
        "zone", S("net", Q("")), S("layers", *cu), S("uuid", Q(ids.net_item_uuid(common.PROJECT_ID, "esp_antenna_keepout"))),
        S("name", Q("ESP12E_antenna_keepout")), S("hatch", "edge", 0.5), S("connect_pads", S("clearance", 0)), S("min_thickness", 0.25),
        S("filled_areas_thickness", False),
        # footprints "allowed": the ESP-12E's own antenna end lies inside; place.py keeps every other part out
        S("keepout", S("tracks", "not_allowed"), S("vias", "not_allowed"), S("pads", "not_allowed"), S("copperpour", "not_allowed"),
          S("footprints", "allowed")),
        S("placement", S("enabled", False), S("sheetname", Q(""))),
        S("fill", S("thermal_gap", 0.5), S("thermal_bridge_width", 0.5)),
        S("polygon", S("pts", *[S("xy", round(x, 3), round(y, 3)) for x, y in pts])),
    )


def place_footprint_zones(fp_node: list) -> int:
    """Engine gap: ``PCBCompiler`` copies a library footprint's (zone ...) with footprint-local points, but a
    .kicad_pcb stores footprint zones in board coordinates (KiCad demos, e.g. RoyalBlue54L-Feather.kicad_pcb).
    Transform them with the footprint's placement (top side only; a bottom-side zone is refused)."""
    from ai_eda.tools.kicad.geometry import to_board

    at = sexpr.find(fp_node, "at")
    x, y = sexpr.to_float(at[1]), sexpr.to_float(at[2])
    rot = sexpr.to_float(at[3]) if len(at) > 3 else 0.0
    if str(sexpr.find(fp_node, "layer")[1]) != "F.Cu":
        if sexpr.find(fp_node, "zone") is not None:
            raise ValueError("bottom-side footprint with a zone: no transform rule")
        return 0
    pl = Placement(component_ref="Z", x_mm=x, y_mm=y, rotation_deg=rot, side=BoardSide.TOP)
    n = 0
    for z in sexpr.find_all(fp_node, "zone"):
        for poly in sexpr.find_all(z, "polygon"):
            pts = sexpr.find(poly, "pts")
            for xy in sexpr.find_all(pts, "xy"):
                bx, by = to_board(pl, sexpr.to_float(xy[1]), sexpr.to_float(xy[2]))
                xy[1], xy[2] = round(bx, 4), round(by, 4)
                n += 1
    return n


#: supply pads that feed a plane: solid zone connection on every layer (rev-2b review PF-M1, 1.34 A boost peak, E)
#: rev 3: the Pololu header J6 is gone (the boost output reaches In2.Cu through vias at C57 / C43, SMD pads connect
#: solid anyway); J14.2 1S pack minus -> In1.Cu / outer GND
SOLID_PADS = {("J14", "2")}

STACKUP_NOTE = "1.6 mm FR4, 2 layers, 1 oz (35 um) outer copper, HASL lead-free (A: JLCPCB 2-layer default; state it in the order)"
#: 4-layer: JLCPCB standard 1.6 mm 4-layer (their JLC04161H-7628 class: 7628 prepreg ~0.21 mm, core ~1.065 mm,
#: 1 oz outer / 0.5 oz inner) - A, from the JLC default stackup as remembered, NOT fetched: name the stackup in the order
STACKUP_NOTE_4 = ("1.6 mm FR4, 4 layers F.Cu signal / In1.Cu GND / In2.Cu power / B.Cu signal, 1 oz outer / 0.5 oz inner, "
                  "JLCPCB standard 4-layer stackup (JLC04161H-7628 class, A - check in the order), lead-free HASL "
                  "(order guide D16, user decision 2026-11-12; ENIG before)")
#: KiCad stackup ``copper_finish`` text of the board (and of the gerber job file's GeneralSpecs.Finish). User decision
#: 2026-11-12: lead-free HASL (JLC order page 'LeadFree HASL'); the name is KiCad's own list entry, as rev 2c had it.
#: The finish is text only - no geometry depends on it. route.py --reuse-ses accepts an unrouted board that differs
#: from the recorded one ONLY in this value (kicad/work/ses_replay.json unrouted_board_sha256_copper_finish_blanked).
COPPER_FINISH = "HAL lead-free"


def stackup() -> list:
    if common.LAYERS == 4:
        return S(
            "stackup",
            S("layer", Q("F.SilkS"), S("type", Q("Top Silk Screen"))),
            S("layer", Q("F.Paste"), S("type", Q("Top Solder Paste"))),
            S("layer", Q("F.Mask"), S("type", Q("Top Solder Mask")), S("thickness", 0.01)),
            S("layer", Q("F.Cu"), S("type", Q("copper")), S("thickness", 0.035)),
            S("layer", Q("dielectric 1"), S("type", Q("prepreg")), S("thickness", 0.2104), S("material", Q("FR4")),
              S("epsilon_r", 4.4), S("loss_tangent", 0.02)),
            S("layer", Q("In1.Cu"), S("type", Q("copper")), S("thickness", 0.0152)),
            S("layer", Q("dielectric 2"), S("type", Q("core")), S("thickness", 1.065), S("material", Q("FR4")),
              S("epsilon_r", 4.6), S("loss_tangent", 0.02)),
            S("layer", Q("In2.Cu"), S("type", Q("copper")), S("thickness", 0.0152)),
            S("layer", Q("dielectric 3"), S("type", Q("prepreg")), S("thickness", 0.2104), S("material", Q("FR4")),
              S("epsilon_r", 4.4), S("loss_tangent", 0.02)),
            S("layer", Q("B.Cu"), S("type", Q("copper")), S("thickness", 0.035)),
            S("layer", Q("B.Mask"), S("type", Q("Bottom Solder Mask")), S("thickness", 0.01)),
            S("layer", Q("B.Paste"), S("type", Q("Bottom Solder Paste"))),
            S("layer", Q("B.SilkS"), S("type", Q("Bottom Silk Screen"))),
            S("copper_finish", Q(COPPER_FINISH)),
            S("dielectric_constraints", "no"),
        )
    return S(
        "stackup",
        S("layer", Q("F.SilkS"), S("type", Q("Top Silk Screen"))),
        S("layer", Q("F.Paste"), S("type", Q("Top Solder Paste"))),
        S("layer", Q("F.Mask"), S("type", Q("Top Solder Mask")), S("thickness", 0.01)),
        S("layer", Q("F.Cu"), S("type", Q("copper")), S("thickness", 0.035)),
        S("layer", Q("dielectric 1"), S("type", Q("core")), S("thickness", 1.51), S("material", Q("FR4")),
          S("epsilon_r", 4.5), S("loss_tangent", 0.02)),
        S("layer", Q("B.Cu"), S("type", Q("copper")), S("thickness", 0.035)),
        S("layer", Q("B.Mask"), S("type", Q("Bottom Solder Mask")), S("thickness", 0.01)),
        S("layer", Q("B.Paste"), S("type", Q("Bottom Solder Paste"))),
        S("layer", Q("B.SilkS"), S("type", Q("Bottom Silk Screen"))),
        S("copper_finish", Q("ENIG")),
        S("dielectric_constraints", "no"),
    )


def esp_preroutes(pdata: dict, library: KicadLibrary, spec: dict) -> tuple[list[dict], list[dict]]:
    """ESP-12E pin 8 (VCC) -> C54 pad 1 on F.Cu, C54 pad 2 -> its own GND via (spec layout_rules: ESP-12E VCC caps at
    pin 8). Pad centres come from the library footprints and the placement (ai_eda geometry), never typed in."""
    from ai_eda.ir import LibraryRef
    from ai_eda.tools.kicad.geometry import pad_center

    parts = {p["refdes"]: p for p in spec["parts"]}

    def pad(ref: str, num: str) -> tuple[float, float]:
        v = pdata["placements"][ref]
        lib, name = common.split_lib_id(common.footprint_of(parts[ref]))
        fp = library.load_footprint(LibraryRef(library=lib, name=name))
        pl = Placement(component_ref=ref, x_mm=v["x"], y_mm=v["y"], rotation_deg=float(v["rot"]), side=BoardSide.TOP)
        x, y = pad_center(pl, fp.pad(num))
        return round(x, 4), round(y, 4)

    p8, c1, c2 = pad("U7", "8"), pad("C54", "1"), pad("C54", "2")
    if abs(p8[1] - c1[1]) > 0.01:
        raise ValueError(f"C54 pad 1 {c1} is not level with ESP pin 8 {p8}")
    via = (c2[0], round(c2[1] + 0.95, 4))
    segs = [{"net": "+3V3", "width": 0.5, "start": list(c1), "end": list(p8), "block": "ESP"},
            {"net": "GND", "width": 0.4, "start": list(c2), "end": list(via), "block": "ESP"}]
    vias = [{"net": "GND", "at": list(via), "size": 0.6, "drill": 0.3, "block": "ESP"}]
    return segs, vias


#: rev 3 battery trunk on B.Cu, board mm (KiCad frame), (x, y, width of the segment that ENDS here). Plan section 13:
#: J15 -> L5 "pour or >= 2.0 mm, target <= 0.035 Ohm". Shape = the rev 2c path (left edge, between U3 and U4, tie
#: band between J21 and J26, diagonal under U1) because that topology routed on rev 2c; it then runs east under the
#: U1 +x pad column and down the +x edge so that the boost block keeps B.Cu free for its plane vias; it ends 1.7 mm
#: above the centre of L5 pad 1 (relayout step: the two trunk vias sit right above the pad).
#: The tie-band crossing is a 1.56 mm neck: every gap between the THT pads of J3 / J19-J26 is 1.86-1.87 mm (M, pad
#: table of the board), so no single-layer >= 2.0 mm crossing exists - reported by routing_checks.py, not hidden.
VBAT_TRUNK = [
    (66.7, 76.0, 2.4), (66.7, 120.5, 4.0), (70.0, 123.8, 2.4), (77.3, 123.8, 2.4), (90.485, 110.615, 2.4),
    (96.6, 110.615, 2.4), (103.4, 110.615, 1.56), (106.035, 110.615, 2.4), (120.9, 125.48, 2.4), (135.2, 125.48, 2.4),
    (136.5, 126.78, 2.0), (136.5, 130.7, 2.0),
]
#: F.Cu rule area between the pads of L5 (plan section 13: no signal trace under L5 on F.Cu), offsets from the L5 pads
L5_KEEPOUT_INSET = 0.15


def power_preroutes(pdata: dict, library: KicadLibrary, spec: dict) -> tuple[list[dict], list[dict]]:
    """rev 3 locked copper of the battery path and the boost block (plan section 13; re-laid by the relayout step after
    the rev 3 reviews E-R1 / E-R2 / E-R3 / J2). Pad centres come from the library footprints + placement; the offsets
    below were checked by kicad-cli DRC on the unrouted board.

    Block geometry (place.POWER_BLOCK): U8 above the input capacitor C41, the output capacitors C57 / C43 upright
    between U8 and L5 with their +5V pads on top and their GND pads at the bottom.

    * VBAT_LOGIC_RAW J14.1 -> Q10.3 on F.Cu (2.0 mm, 1.0 mm into the SOT-23 pad); VBAT_LOGIC Q10.2 -> J15.1 on F.Cu,
      2.0 mm all the way (enters pad 2 from its free side).
    * VBAT_LOGIC_SW J15.2 -> L5.1: B.Cu trunk (VBAT_TRUNK) + 2 vias right above L5.1 + a 1.5 mm F.Cu strip into the pad
      (the pad is 1.5 mm wide; the L5 rule area and the board edge leave 1.85 mm, so no 2.0 mm track can enter it).
    * BOOST_SW U8.5 -> L5.2: F.Cu only, no via, between the pads of C57 and C43 (under the two capacitor bodies).
    * +5V: U8.6 -> C57.1 0.95 mm, one 1.0 mm bar over C57.1 / C43.1, 3 vias into In2.Cu; R63.1 (feedback divider) is a
      0.2 mm sense track to the C57.1 end of that bar without a via.
    * GND: U8.4 -> C41.2 0.53 mm, one 1.0 mm bar over C41.2 / C57.2 / C43.2 (ONE F.Cu GND piece), own vias into In1.Cu
      at U8.4, C41.2, C57.2 and C43.2 (the two under the capacitor bodies sit between BOOST_SW and the GND pads).
    * VBAT_LOGIC_SW at the IC: U8.3 -> C41.1 on F.Cu (1.6 mm); C41.1 -> via under the C41 body -> B.Cu along the board
      edge -> via -> L5.1 (the input capacitor's path to the inductor; carries no battery DC current).
    * BOOST_FB complete on F.Cu: U8.1 -> R63.2 -> R64.1 -> C58.2, west of U8 (not inside a capacitor loop).
    * BOOST_EN: U8.2 -> own via 1.0 mm west of the pin (the pin is enclosed by FB / VIN copper on F.Cu); the rest of
      the net is routed by Freerouting.
    """
    from ai_eda.ir import LibraryRef
    from ai_eda.tools.kicad.geometry import pad_center

    parts = {p["refdes"]: p for p in spec["parts"]}

    def pad(ref: str, num: str) -> tuple[float, float]:
        v = pdata["placements"][ref]
        lib, name = common.split_lib_id(common.footprint_of(parts[ref]))
        fp = library.load_footprint(LibraryRef(library=lib, name=name))
        pl = Placement(component_ref=ref, x_mm=v["x"], y_mm=v["y"], rotation_deg=float(v["rot"]), side=BoardSide.TOP)
        x, y = pad_center(pl, fp.pad(num))
        return round(x, 4), round(y, 4)

    segs: list[dict] = []
    vias: list[dict] = []

    def seg(net: str, a, b, w: float, layer: str = "F.Cu") -> None:
        segs.append({"net": net, "width": w, "start": [round(a[0], 4), round(a[1], 4)],
                     "end": [round(b[0], 4), round(b[1], 4)], "block": "PWR", "layer": layer})

    def via(net: str, at, size: float = 0.6, drill: float = 0.3) -> None:
        vias.append({"net": net, "at": [round(at[0], 4), round(at[1], 4)], "size": size, "drill": drill, "block": "PWR"})

    def off(p, dx: float, dy: float) -> tuple[float, float]:
        return (round(p[0] + dx, 4), round(p[1] + dy, 4))

    # --- battery input, F.Cu ---------------------------------------------------------------------------------
    j14, q3, q2, j151, j152 = pad("J14", "1"), pad("Q10", "3"), pad("Q10", "2"), pad("J15", "1"), pad("J15", "2")
    raw_y = round((pad("J14", "2")[1] + j152[1]) / 2 + 0.03, 3)    # middle of the gap J14.2 / J15 pads
    seg("VBAT_LOGIC_RAW", j14, (j14[0], raw_y), 2.0)
    seg("VBAT_LOGIC_RAW", (j14[0], raw_y), (q3[0] - 1.0, raw_y), 2.0)
    seg("VBAT_LOGIC_RAW", (q3[0] - 1.0, raw_y), q3, 0.6)
    # Q10.2 -> J15.1: 2.0 mm from inside pad 2 (relayout step: was 0.6 / 1.0 mm for 3.9 mm). The track leaves the pad
    # toward +x (its free side), 0.145 mm above the R14.2 pad, turns down 0.175 mm beside it and cuts the lower corner
    # with a 45 degree piece (M, first relayout try: a square corner at x + 1.25 touched the routed +3V3 track at R48)
    lg_y = round(j152[1] + 3.55, 3)
    vx_l = round(q2[0] + 1.25, 4)
    a_l = off(q2, 0.0, 0.1425)
    b_l = off(q2, 0.6, 0.1425)
    c_l = (vx_l, round(b_l[1] + (vx_l - b_l[0]), 4))
    d_l = (vx_l, round(lg_y - 0.8, 4))
    e_l = (round(vx_l - 0.8, 4), lg_y)
    for p_, q_ in ((a_l, b_l), (b_l, c_l), (c_l, d_l), (d_l, e_l), (e_l, (j151[0], lg_y)), ((j151[0], lg_y), j151)):
        seg("VBAT_LOGIC", p_, q_, 2.0)
    # --- battery trunk, B.Cu ---------------------------------------------------------------------------------
    prev = j152
    for x, y, w in VBAT_TRUNK:
        seg("VBAT_LOGIC_SW", prev, (x, y), w, "B.Cu")
        prev = (x, y)
    l51, l52 = pad("L5", "1"), pad("L5", "2")
    if abs(VBAT_TRUNK[-1][0] - (l51[0] - 0.25)) > 0.01 or not (l51[1] - 2.0 < VBAT_TRUNK[-1][1] < l51[1] - 1.4):
        raise ValueError("VBAT_TRUNK does not end above L5 pad 1 - the power block moved")
    # two 0.8 / 0.4 vias 0.25 mm above the pad, on the trunk (not higher: their In1 anti-pads would pinch off plane
    # slivers at the top of the block box - routing_checks in1_under_block, M on the first relayout try)
    vy = round(l51[1] - 1.9, 4)
    for dx in (-0.45, 0.45):
        via("VBAT_LOGIC_SW", (round(l51[0] + dx, 4), vy), 0.8, 0.4)
    sx = round(l51[0] + 0.05, 4)                  # 1.5 mm strip: 0.15 mm clear of the U6 tab corner and of the L5 rule area
    seg("VBAT_LOGIC_SW", (sx, round(vy - 0.15, 4)), (sx, round(l51[1] - 1.2, 4)), 1.5)
    # --- boost block -----------------------------------------------------------------------------------------
    u81, u82, u83 = pad("U8", "1"), pad("U8", "2"), pad("U8", "3")
    u84, u85, u86 = pad("U8", "4"), pad("U8", "5"), pad("U8", "6")
    c571, c572, c431, c432 = pad("C57", "1"), pad("C57", "2"), pad("C43", "1"), pad("C43", "2")
    c411, c412 = pad("C41", "1"), pad("C41", "2")
    r631, r632, r641, c582 = pad("R63", "1"), pad("R63", "2"), pad("R64", "1"), pad("C58", "2")
    if not (abs(c571[1] - c431[1]) < 0.01 and abs(c572[1] - c432[1]) < 0.01 and c571[1] < u86[1] < u85[1] < u84[1] < c572[1]
            and u86[0] < c571[0] < c431[0] < l52[0] and abs(c412[0] - u84[0]) < 0.5 and c412[1] > u84[1]
            and c411[0] < u83[0] and r632[0] < u81[0] and r641[1] > r632[1] and c582[0] < r641[0]):
        raise ValueError("boost block pads are not in the relayout arrangement - re-derive power_preroutes")
    # BOOST_SW: 0.1 mm above the middle of the capacitor pads (room for the GND vias under the bodies)
    y_sw = round((c571[1] + c572[1]) / 2 - 0.1, 4)
    sw_a = off(u85, 0.7875, 0.0)
    sw_b = (round(sw_a[0] + (y_sw - u85[1]), 4), y_sw)
    sw_e = (round(l52[0] - 0.35, 4), y_sw)
    seg("BOOST_SW", u85, sw_a, 0.35)
    seg("BOOST_SW", sw_a, sw_b, 0.35)
    seg("BOOST_SW", sw_b, sw_e, 0.35)
    if not (sw_b[0] < c571[0] - 0.5 and abs(y_sw - l52[1]) < 1.2):
        raise ValueError("BOOST_SW pre-route does not pass between the capacitor pads into L5 pad 2")
    # +5V: U8.6 -> C57.1, bar over C57.1 / C43.1, three vias into In2 beyond the pads
    bar0 = off(c571, -0.6, 0.0)
    bar1 = off(c431, 0.5, 0.0)
    seg("+5V", u86, (round(bar0[0] - 0.4, 4), u86[1]), 0.35)
    seg("+5V", (round(bar0[0] - 0.4, 4), u86[1]), bar0, 0.35)
    seg("+5V", bar0, bar1, 1.0)
    # 1.65 mm apart: closer vias merge their In1 anti-pads into one slot (M on the first relayout try)
    for v in (off(c571, -0.25, -0.975), (round((c571[0] + c431[0]) / 2 + 0.25, 4), round(c571[1] - 0.975, 4)),
              off(c431, 0.75, -0.975)):
        seg("+5V", v, (v[0], round(c571[1] - 0.3, 4)), 0.5)
        via("+5V", v)
    # feedback divider top R63.1: 0.2 mm sense track to the C57.1 end of the bar, no via on it (the divider senses the
    # output capacitor, not the plane); it passes between the U1 pad row and the R63 pads, then above U8
    s_y1 = round(r631[1] - 0.73, 4)
    s_y2 = round(u86[1] - 0.83, 4)
    s2 = (round(u81[0] - 0.5375, 4), s_y1)
    s3 = (round(s2[0] + (s_y2 - s_y1), 4), s_y2)
    s4 = (round(bar0[0] - (bar0[1] - s_y2), 4), s_y2)
    for a, b in ((r631, (r631[0], s_y1)), ((r631[0], s_y1), s2), (s2, s3), (s3, s4), (s4, bar0)):
        seg("+5V", a, b, 0.2)
    # BOOST_FB, complete: U8.1 -> R63.2 -> R64.1 -> C58.2
    f1 = off(u81, -0.4875, 0.0)
    seg("BOOST_FB", u81, f1, 0.2)
    seg("BOOST_FB", f1, off(f1, -0.5, -0.5), 0.2)
    g0 = off(r632, -0.225, 0.4)
    g1 = (round(r641[0] + 0.12, 4), round(g0[1] + (g0[0] - (r641[0] + 0.12)), 4))
    seg("BOOST_FB", g0, g1, 0.2)
    seg("BOOST_FB", g1, (round(c582[0] + 0.2, 4), g1[1]), 0.2)
    if not (abs(g1[1] - r641[1]) < 0.35 and abs(g1[1] - c582[1]) < 0.4):
        raise ValueError("BOOST_FB pre-route does not end inside R64.1 / C58.2")
    # feed-forward capacitor C58.1 (+5V): F.Cu track to the sense node R63.1 (C58 lies across R63). M, relayout try T2:
    # the fan-out found no via spot at this pad (U9 pads, R64.2, board edge) and the pad stayed open. Path: west of the
    # C58 pads (0.16 mm to the U9 pads, 0.165 mm to C58.2), then between the C59 / R63 pads and the C58.2 / R64.1 pads
    c581 = pad("C58", "1")
    x_w = round(c581[0] - 0.74, 4)
    y_c = round(r631[1] + 1.0, 4)
    ff = [c581, (x_w, c581[1]), (x_w, round(y_c + 0.2, 4)), (round(x_w + 0.2, 4), y_c), (round(r631[0] - 0.675, 4), y_c),
          off(r631, -0.125, 0.45)]
    if not (c581[1] > c582[1] > y_c > r631[1] and x_w < c582[0] - 0.6):
        raise ValueError("C58.1 sense pre-route: C58 / R63 are not in the relayout arrangement")
    for p_, q_ in zip(ff, ff[1:]):
        seg("+5V", p_, q_, 0.2)
    # BOOST_EN: own via west of pin 2
    en_v = off(u82, -0.9875, 0.0)
    seg("BOOST_EN", u82, en_v, 0.2)
    via("BOOST_EN", en_v)
    # VIN: U8.3 -> C41.1 on F.Cu; C41.1 -> via under the C41 body -> B.Cu along the edge -> via -> L5.1
    n1 = off(u83, -0.5375, 0.0)
    seg("VBAT_LOGIC_SW", u83, n1, 0.3)
    seg("VBAT_LOGIC_SW", n1, off(n1, -0.45, 0.9), 0.3)
    vc = off(c411, 1.025, 0.3)
    seg("VBAT_LOGIC_SW", vc, off(c411, 0.4, 0.3), 0.5)
    via("VBAT_LOGIC_SW", vc)
    vl = off(l51, 0.0, 1.75)
    k1 = (round(vc[0] + (vl[1] - vc[1]), 4), vl[1])
    seg("VBAT_LOGIC_SW", vc, k1, 0.8, "B.Cu")
    seg("VBAT_LOGIC_SW", k1, vl, 0.8, "B.Cu")
    via("VBAT_LOGIC_SW", vl)
    seg("VBAT_LOGIC_SW", vl, off(l51, 0.0, 1.0), 0.5)
    # GND: U8.4 -> C41.2, bar over C41.2 / C57.2 / C43.2, own vias at U8.4, C41.2, C57.2, C43.2
    seg("GND", u84, (u84[0], round(c412[1] - 0.65, 4)), 0.3)
    gb0 = (round(c412[0] - 0.025, 4), round(c572[1] + 0.025, 4))
    seg("GND", gb0, (round(c432[0] + 0.4, 4), gb0[1]), 1.0)
    g_u = off(u84, 0.6875, 0.3)
    seg("GND", g_u, off(c412, 0.275, -0.7), 0.4)
    via("GND", g_u)
    g_c = off(c412, -1.025, -0.4)
    seg("GND", g_c, off(c412, -0.3, -0.4), 0.4)
    via("GND", g_c)
    for c2 in (c572, c432):
        gv = off(c2, 0.0, -0.955)
        seg("GND", gv, off(c2, 0.0, -0.375), 0.4)
        via("GND", gv)
    # --- small taps of the battery net (attempt a1, M: Freerouting could not enter these pads with the 1.0 mm /
    # 0.8 mm-via PowerHigh class and left 5 connections open) - every pad of the net is wired here, so the net never
    # goes to the autorouter. None of them carries the boost input current (bulk / bypass capacitors, dividers).
    diag = lambda x: round(x + (VBAT_TRUNK[8][1] - VBAT_TRUNK[8][0]), 4)   # y of the 45 deg trunk piece under U1
    if abs((VBAT_TRUNK[8][1] - VBAT_TRUNK[7][1]) - (VBAT_TRUNK[8][0] - VBAT_TRUNK[7][0])) > 1e-6:
        raise ValueError("VBAT_TRUNK piece 7 -> 8 is no longer a 45 degree diagonal - re-derive the tap ends")
    r151 = pad("R15", "1")                       # FCU battery divider top: via on the trunk beside the pad
    seg("VBAT_LOGIC_SW", r151, (round(r151[0] - 1.125, 4), VBAT_TRUNK[6][1]), 0.3)
    via("VBAT_LOGIC_SW", (round(r151[0] - 1.125, 4), VBAT_TRUNK[6][1]))
    c591, u96 = pad("C59", "1"), pad("U9", "6")  # supervisor supply + its bypass: one via between the two pads
    ux = c591[0]
    uvia = (ux, round((c591[1] + 0.475 + u96[1] - 0.3) / 2, 4))
    seg("VBAT_LOGIC_SW", c591, uvia, 0.4)
    seg("VBAT_LOGIC_SW", uvia, (ux, u96[1]), 0.4)
    via("VBAT_LOGIC_SW", uvia)
    seg("VBAT_LOGIC_SW", uvia, (ux, diag(ux)), 0.5, "B.Cu")
    r651 = pad("R65", "1")                       # UVLO divider top: via left of the pad, B.Cu under C60 / U9
    rvia = off(r651, -1.1, 0.005)
    seg("VBAT_LOGIC_SW", r651, rvia, 0.3)
    via("VBAT_LOGIC_SW", rvia)
    seg("VBAT_LOGIC_SW", rvia, (round(ux - 0.6, 4), rvia[1]), 0.3, "B.Cu")
    seg("VBAT_LOGIC_SW", (round(ux - 0.6, 4), rvia[1]), (ux, round(rvia[1] - 0.6, 4)), 0.3, "B.Cu")
    seg("VBAT_LOGIC_SW", (ux, round(rvia[1] - 0.6, 4)), uvia, 0.3, "B.Cu")
    c421 = pad("C42", "1")                       # bulk capacitor at the switch: via below the pad, B.Cu to J15.2
    cvia = off(c421, 0.4, 1.325)
    seg("VBAT_LOGIC_SW", off(c421, 0.4, 0.1), cvia, 0.6)
    via("VBAT_LOGIC_SW", cvia, 0.8, 0.4)
    seg("VBAT_LOGIC_SW", cvia, (round(j152[0] + (j152[1] - cvia[1]), 4), cvia[1]), 1.0, "B.Cu")
    seg("VBAT_LOGIC_SW", (round(j152[0] + (j152[1] - cvia[1]), 4), cvia[1]), j152, 1.0, "B.Cu")
    return segs, vias


def l5_keepout(pdata: dict, library: KicadLibrary, spec: dict) -> list:
    """F.Cu rule area between the two pads of L5: no tracks and no vias under the inductor (plan section 13).
    Pours stay allowed (the F.Cu GND pour is no signal). Rectangle = the copper-free strip between the pad edges."""
    from ai_eda.ir import LibraryRef
    from ai_eda.tools.kicad.geometry import pad_center

    parts = {p["refdes"]: p for p in spec["parts"]}
    v = pdata["placements"]["L5"]
    lib, name = common.split_lib_id(common.footprint_of(parts["L5"]))
    fp = library.load_footprint(LibraryRef(library=lib, name=name))
    pl = Placement(component_ref="L5", x_mm=v["x"], y_mm=v["y"], rotation_deg=float(v["rot"]), side=BoardSide.TOP)
    if abs(float(v["rot"])) % 180 > 0.01:
        raise ValueError("L5 keep-out assumes L5 at 0 / 180 degrees")
    xs, ys, half_w, half_h = [], [], 0.0, 0.0
    for num in ("1", "2"):
        p = fp.pad(num)
        x, y = pad_center(pl, p)
        xs.append(x)
        ys.append(y)
        half_w, half_h = p.size_w / 2, p.size_h / 2
    x0 = round(min(xs) + half_w + L5_KEEPOUT_INSET, 3)
    x1 = round(max(xs) - half_w - L5_KEEPOUT_INSET, 3)
    y0 = round(min(ys) - half_h - L5_KEEPOUT_INSET, 3)
    y1 = round(max(ys) + half_h + L5_KEEPOUT_INSET, 3)
    pts = [(x0, y0), (x1, y0), (x1, y1), (x0, y1)]
    return S(
        "zone", S("net", Q("")), S("layers", Q("F.Cu")), S("uuid", Q(ids.net_item_uuid(common.PROJECT_ID, "l5_keepout"))),
        S("name", Q("L5_no_track_under_inductor")), S("hatch", "edge", 0.5), S("connect_pads", S("clearance", 0)),
        S("min_thickness", 0.25), S("filled_areas_thickness", False),
        S("keepout", S("tracks", "not_allowed"), S("vias", "not_allowed"), S("pads", "allowed"), S("copperpour", "allowed"),
          S("footprints", "allowed")),
        S("placement", S("enabled", False), S("sheetname", Q(""))),
        S("fill", S("thermal_gap", 0.5), S("thermal_bridge_width", 0.5)),
        S("polygon", S("pts", *[S("xy", x, y) for x, y in pts])),
    )


def preroutes(pdata: dict, library: KicadLibrary, spec: dict) -> list:
    """Locked copper: MCU support clusters (place.CLUSTER_ROUTES) + ESP VCC decoupling + the rev 3 battery path and
    boost block (power_preroutes) - Freerouting keeps them."""
    from place import cluster_routes_board, cluster_vias_board

    out = []
    esp_segs, esp_vias = esp_preroutes(pdata, library, spec)
    pwr_segs, pwr_vias = power_preroutes(pdata, library, spec)
    esp_vias = cluster_vias_board(pdata["placements"]) + esp_vias + pwr_vias
    for i, sg in enumerate(cluster_routes_board(pdata["placements"]) + esp_segs + pwr_segs):
        out.append(S("segment", S("start", *sg["start"]), S("end", *sg["end"]), S("width", sg["width"]), S("locked", "yes"),
                     S("layer", Q(sg.get("layer", "F.Cu"))), S("net", Q(sg["net"])),
                     S("uuid", Q(ids.net_item_uuid(common.PROJECT_ID, "preroute", sg["block"], i)))))
    for i, v in enumerate(esp_vias):
        out.append(S("via", S("at", *v["at"]), S("size", v["size"]), S("drill", v["drill"]), S("layers", Q("F.Cu"), Q("B.Cu")),
                     S("locked", "yes"), S("net", Q(v["net"])),
                     S("uuid", Q(ids.net_item_uuid(common.PROJECT_ID, "preroute_via", v["block"], i)))))
    return out


def nc_net_names() -> dict[tuple[str, str], str]:
    """``(ref, pad) -> "unconnected-(...)"`` net names KiCad gives no-connect pins, read from the netlist that
    kicad-cli exports from the schematic (so the board's single-pad nets match --schematic-parity exactly)."""
    import subprocess

    net = common.WORK_DIR / f"{common.PROJECT_ID}.net"
    common.WORK_DIR.mkdir(parents=True, exist_ok=True)
    r = subprocess.run([str(common.KICAD_CLI), "sch", "export", "netlist", "--format", "kicadsexpr", "-o", str(net),
                        str(common.KICAD_DIR / f"{common.PROJECT_ID}.kicad_sch")],
                       capture_output=True, text=True, encoding="utf-8", errors="replace")
    if r.returncode != 0:
        raise RuntimeError(f"netlist export failed: {r.stderr}")
    tree = sexpr.parse_file(net)
    out = {}
    for n in sexpr.find_all(sexpr.find(tree, "nets"), "net"):
        name = str(sexpr.get(n, "name"))
        if name.startswith("unconnected-("):
            for node in sexpr.find_all(n, "node"):
                out[(str(sexpr.get(node, "ref")), str(sexpr.get(node, "pin")))] = name
    return out


def patch_board(node: list, spec: dict, pdata: dict, library: KicadLibrary) -> None:
    nc_nets = nc_net_names()
    blocks = common.block_of(spec)
    parts = {p["refdes"]: p for p in spec["parts"]}
    sheet_by_block = {}
    for name, stem, blks, _t in common.SHEETS:
        for b in blks:
            sheet_by_block[b] = (name, stem)
    new = []
    for item in node:
        h = sexpr.head(item)
        if h == "setup" and sexpr.find(item, "stackup") is None:
            item.insert(1, stackup())
        if h == "title_block":
            # revision in the title block (gerber job file 'Revision'; rev-2b review PF-m7)
            item[:] = [S("title_block", S("title", Q("VTOL avionics carrier")), S("rev", Q("3")),
                         S("comment", 1, Q("NOT_VERIFIED: never built or powered")))][0]
        if h == "zone":
            cp = sexpr.find(item, "connect_pads")
            if cp is not None and len(cp) > 1 and str(cp[1]) == "yes":
                cp[1] = "thru_hole_only"   # thermal reliefs on THT pads (hand soldering), solid SMD (review F7)
        if h == "gr_rect" and sexpr.get(item, "layer") == "Edge.Cuts":
            new.extend(rounded_outline())
            continue
        if h == "footprint":
            place_footprint_zones(item)
            ref = None
            for prop in sexpr.find_all(item, "property"):
                if str(prop[1]) == "Reference":
                    ref = str(prop[2])
            name, stem = sheet_by_block[blocks[ref]]
            sheet_uuid = bs.sheet_symbol_uuid(stem)
            for j, child in enumerate(item):
                hh = sexpr.head(child)
                if hh == "path":
                    item[j] = S("path", Q(f"/{sheet_uuid}/{ids.symbol_uuid(common.PROJECT_ID, ref)}"))
                elif hh == "sheetname":
                    item[j] = S("sheetname", Q(f"/{name}/"))
                elif hh == "sheetfile":
                    item[j] = S("sheetfile", Q(f"{stem}.kicad_sch"))
                elif hh == "attr" and not common.populated(parts[ref]):
                    if "dnp" not in [str(a) for a in child[1:]]:
                        child.append("dnp")
                elif hh == "property" and str(child[1]) == "Reference":
                    # reference designators go to the fab (assembly) layer: the board is too dense for silk text
                    lay = sexpr.find(child, "layer")
                    if lay is not None and str(lay[1]) == "F.SilkS":
                        lay[1] = Q("F.Fab")
                elif hh == "pad":
                    key = (ref, str(child[1]))
                    if key in nc_nets and sexpr.find(child, "net") is None:
                        pos = [sexpr.head(c) for c in child].index("pintype")
                        child.insert(pos, S("net", Q(nc_nets[key])))
                    if key in SOLID_PADS and sexpr.find(child, "zone_connect") is None:
                        child.append(S("zone_connect", 2))  # 2 = solid (KiCad file format, e.g. demos/cm5_minima)
            if not common.populated(parts[ref]) and sexpr.find(item, "attr") is None:
                idx = [sexpr.head(c) for c in item].index("sheetfile") + 1
                item.insert(idx, S("attr", "dnp"))
        new.append(item)
    # insert drawings and keep-out before the trailing (embedded_fonts)
    tail = new.pop() if sexpr.head(new[-1]) == "embedded_fonts" else None
    new.extend(drawings(pdata))
    new.append(antenna_keepout(pdata))
    new.append(l5_keepout(pdata, library, spec))
    new.extend(preroutes(pdata, library, spec))
    if tail is not None:
        new.append(tail)
    node[:] = new


def main() -> int:
    spec = common.load_spec()
    library = KicadLibrary()
    ir, pdata = ir_with_pcb(spec, library)
    node = PCBCompiler().build(ir, CompileContext(workdir=common.WORK_DIR, tools={"kicad_library": library}))
    patch_board(node, spec, pdata, library)
    common.BUILD_WORK_DIR.mkdir(parents=True, exist_ok=True)
    unrouted = common.BUILD_WORK_DIR / f"{common.PROJECT_ID}_unrouted.kicad_pcb"
    unrouted.write_bytes(sexpr.dumps(node).encode("utf-8"))
    ir_dir = common.KICAD_DIR / "ir" if common.BUILD_WORK_DIR == common.WORK_DIR else common.BUILD_WORK_DIR
    ir_hash = common.dump_ir(ir, ir_dir / f"{common.PROJECT_ID}.ir_with_pcb.json")
    info = {"ir_content_hash": ir_hash, "unrouted_board": unrouted.name, "sha256": common.sha256_file(unrouted),
            "placement_sha256": common.sha256_file(common.KICAD_DIR / "placement.json")}
    info["copper_layers"] = common.LAYERS
    (common.BUILD_WORK_DIR / "unrouted_manifest.json").write_bytes((json.dumps(info, indent=1) + "\n").encode("utf-8"))
    print(json.dumps(info, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
