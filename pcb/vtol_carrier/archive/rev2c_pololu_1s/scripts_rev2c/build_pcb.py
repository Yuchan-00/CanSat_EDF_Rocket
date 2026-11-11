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
    inset = 0.0
    poly = [(BX0 + inset, BY0 + inset), (BX0 + W - inset, BY0 + inset), (BX0 + W - inset, BY0 + H - inset), (BX0 + inset, BY0 + H - inset)]
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
    notes = ["J6: Pololu S9V11F5S6CMA 5 V buck-boost module soldered flat on a 1x5 header (VOUT GND VIN EN PG), trimmer on top",
             "No parts on the bottom side (3 mm foam tape); the 5th ATmega128A is removed (user decision 2026-11-05)",
             "J3 ESP flash 2x3 footprint: unpopulated, top of the tie band beside the ESP-12E (press a header in to flash)",
             "J14 (1S) + J15 (power switch) in the -x/+y corner next to battery spot A; J13 release header removed (VTOL)"]
    for i, t in enumerate(notes):
        out.append(S("gr_text", Q(t), S("at", 62.0, round(137.4 + 1.1 * i, 2), 0), S("layer", Q("Dwgs.User")),
                     S("uuid", Q(ids.net_item_uuid(common.PROJECT_ID, "note_text", i))),
                     S("effects", S("font", S("size", 0.7, 0.7), S("thickness", 0.09)), S("justify", "left"))))
    out.append(S("gr_text", Q("VTOL carrier rev 2c - NOT_VERIFIED: never built or powered"), S("at", 100.0, 136.2, 0), S("layer", Q("Dwgs.User")),
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
#: J6.1 Pololu VOUT -> In2.Cu +5V; J6.2 Pololu GND and J14.2 1S pack minus -> In1.Cu / outer GND
SOLID_PADS = {("J6", "1"), ("J6", "2"), ("J14", "2")}

STACKUP_NOTE = "1.6 mm FR4, 2 layers, 1 oz (35 um) outer copper, HASL lead-free (A: JLCPCB 2-layer default; state it in the order)"
#: 4-layer: JLCPCB standard 1.6 mm 4-layer (their JLC04161H-7628 class: 7628 prepreg ~0.21 mm, core ~1.065 mm,
#: 1 oz outer / 0.5 oz inner) - A, from the JLC default stackup as remembered, NOT fetched: name the stackup in the order
STACKUP_NOTE_4 = ("1.6 mm FR4, 4 layers F.Cu signal / In1.Cu GND / In2.Cu power / B.Cu signal, 1 oz outer / 0.5 oz inner, "
                  "JLCPCB standard 4-layer stackup (JLC04161H-7628 class, A - check in the order), HASL lead-free")


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
            S("copper_finish", Q("HAL lead-free")),
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
        S("copper_finish", Q("HAL lead-free")),
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


def preroutes(pdata: dict, library: KicadLibrary, spec: dict) -> list:
    """Locked copper: MCU support clusters (place.CLUSTER_ROUTES) + ESP VCC decoupling - Freerouting keeps them."""
    from place import cluster_routes_board, cluster_vias_board

    out = []
    esp_segs, esp_vias = esp_preroutes(pdata, library, spec)
    esp_vias = cluster_vias_board(pdata["placements"]) + esp_vias
    for i, sg in enumerate(cluster_routes_board(pdata["placements"]) + esp_segs):
        out.append(S("segment", S("start", *sg["start"]), S("end", *sg["end"]), S("width", sg["width"]), S("locked", "yes"),
                     S("layer", Q("F.Cu")), S("net", Q(sg["net"])),
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
            item[:] = [S("title_block", S("title", Q("VTOL avionics carrier")), S("rev", Q("2c")),
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
