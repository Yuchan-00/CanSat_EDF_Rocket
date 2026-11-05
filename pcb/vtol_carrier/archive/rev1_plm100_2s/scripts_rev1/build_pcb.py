"""IR + kicad/placement.json -> unrouted ``vtol_carrier.kicad_pcb`` (KiCad 10).

Invariants
----------
* The board body comes from the AI_EDA engine's ``PCBCompiler`` (footprints embedded verbatim from the
  KiCad libraries on disk, pad nets and pin types from the IR / library symbol, deterministic UUIDs).
* This script only adds what the engine cannot express yet (README_ko.md, "engine gaps"):
  hierarchical sheet paths on footprints (schematic parity), DNP attributes, a rounded Edge.Cuts outline,
  documentation drawings of the module bodies on Dwgs.User, a no-pour rule area under the PLM100
  u.FL end, the board stackup (1 oz, 1.6 mm, finish) and the LOCKED pre-routed copper of the MCU support
  clusters (place.CLUSTER_ROUTES: VCC / AVCC decoupling and crystal nets, review F1 / F2) that Freerouting
  must keep. Everything is a pure function of the spec + placement -> byte-identical output.
* Zones: GND on F.Cu and B.Cu over the whole board, written unfilled (KiCad fills them in route.py / DRC);
  through-hole pads get thermal reliefs, SMD pads connect solid (review F7).
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
    return json.loads((common.KICAD_DIR / "placement.json").read_text(encoding="utf-8"))


def ir_with_pcb(spec: dict, library: KicadLibrary):
    ir = common.build_ir(spec, library)
    pdata = placement_data()
    prov = Provenance(kind=ProvenanceKind.DERIVED, tool="scripts/place.py", note="deterministic placement (kicad/placement.json)",
                      created_at=common.FIXED_TIME)
    placements = [
        Placement(component_ref=ref, x_mm=v["x"], y_mm=v["y"], rotation_deg=float(v["rot"]), side=BoardSide.TOP, provenance=prov)
        for ref, v in pdata["placements"].items()
    ]
    inset = 0.0
    poly = [(BX0 + inset, BY0 + inset), (BX0 + W - inset, BY0 + inset), (BX0 + W - inset, BY0 + H - inset), (BX0 + inset, BY0 + H - inset)]
    zprov = Provenance(kind=ProvenanceKind.ASSUMPTION, note="GND pour on both layers (task: 2-layer, GND pour on both)", created_at=common.FIXED_TIME)
    zones = [Zone(net="GND", layer="F.Cu", polygon=poly, clearance_mm=0.25, provenance=zprov),
             Zone(net="GND", layer="B.Cu", polygon=poly, clearance_mm=0.25, provenance=zprov)]
    ir.pcb = PCBDesign(
        layers=[Layer(name="F.Cu", kind="signal"), Layer(name="B.Cu", kind="signal")],
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
    out.append(S("gr_text", Q("cable-tie band x -3..+3 mm: no components (ISP footprints J19-J23 unpopulated)"),
                 S("at", 97.3, 66.5, 90), S("layer", Q("Dwgs.User")), S("uuid", Q(ids.net_item_uuid(common.PROJECT_ID, "tie_text"))),
                 S("effects", S("font", S("size", 0.6, 0.6), S("thickness", 0.08)), S("justify", "right"))))
    out.append(S("gr_text", Q("VTOL carrier - NOT_VERIFIED: never built or powered"), S("at", 100.0, 136.2, 0), S("layer", Q("Dwgs.User")),
                 S("uuid", Q(ids.net_item_uuid(common.PROJECT_ID, "title_text"))),
                 S("effects", S("font", S("size", 0.8, 0.8), S("thickness", 0.1)))))
    return out


def ufl_keepout() -> list:
    """No copper pour under the PLM100 u.FL corner (module antenna end, top-left of the module)."""
    from place import PLM_CX, PLM_CY

    x0, y0 = PLM_CX - 11.0, PLM_CY - 9.5
    # 5 x 2.6 mm at the module corner that carries the u.FL receptacle (datasheet fig 4); kept clear of every pad
    # (R48 pad 1 sits at x 80.18, the zone ends at 79.9)
    pts = [(x0, y0), (x0 + 5.0, y0), (x0 + 5.0, y0 + 2.6), (x0, y0 + 2.6)]
    return S(
        "zone", S("net", Q("")), S("layers", Q("F.Cu"), Q("B.Cu")), S("uuid", Q(ids.net_item_uuid(common.PROJECT_ID, "ufl_keepout"))),
        S("name", Q("PLM100_uFL_no_pour")), S("hatch", "edge", 0.5), S("connect_pads", S("clearance", 0)), S("min_thickness", 0.25),
        S("filled_areas_thickness", False),
        S("keepout", S("tracks", "allowed"), S("vias", "allowed"), S("pads", "allowed"), S("copperpour", "not_allowed"),
          S("footprints", "allowed")),
        S("placement", S("enabled", False), S("sheetname", Q(""))),
        S("fill", S("thermal_gap", 0.5), S("thermal_bridge_width", 0.5)),
        S("polygon", S("pts", *[S("xy", round(x, 3), round(y, 3)) for x, y in pts])),
    )


STACKUP_NOTE = "1.6 mm FR4, 2 layers, 1 oz (35 um) outer copper, HASL lead-free (A: JLCPCB 2-layer default; state it in the order)"


def stackup() -> list:
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


def preroutes(pdata: dict) -> list:
    """Locked F.Cu segments of the MCU support clusters (place.CLUSTER_ROUTES) - Freerouting keeps them."""
    from place import cluster_routes_board

    out = []
    for i, sg in enumerate(cluster_routes_board(pdata["placements"])):
        out.append(S("segment", S("start", *sg["start"]), S("end", *sg["end"]), S("width", sg["width"]), S("locked", "yes"),
                     S("layer", Q("F.Cu")), S("net", Q(sg["net"])),
                     S("uuid", Q(ids.net_item_uuid(common.PROJECT_ID, "preroute", sg["block"], i)))))
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


def patch_board(node: list, spec: dict, pdata: dict) -> None:
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
        if h == "zone":
            cp = sexpr.find(item, "connect_pads")
            if cp is not None and len(cp) > 1 and str(cp[1]) == "yes":
                cp[1] = "thru_hole_only"   # thermal reliefs on THT pads (hand soldering), solid SMD (review F7)
        if h == "gr_rect" and sexpr.get(item, "layer") == "Edge.Cuts":
            new.extend(rounded_outline())
            continue
        if h == "footprint":
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
            if not common.populated(parts[ref]) and sexpr.find(item, "attr") is None:
                idx = [sexpr.head(c) for c in item].index("sheetfile") + 1
                item.insert(idx, S("attr", "dnp"))
        new.append(item)
    # insert drawings and keep-out before the trailing (embedded_fonts)
    tail = new.pop() if sexpr.head(new[-1]) == "embedded_fonts" else None
    new.extend(drawings(pdata))
    new.append(ufl_keepout())
    new.extend(preroutes(pdata))
    if tail is not None:
        new.append(tail)
    node[:] = new


def main() -> int:
    spec = common.load_spec()
    library = KicadLibrary()
    ir, pdata = ir_with_pcb(spec, library)
    node = PCBCompiler().build(ir, CompileContext(workdir=common.WORK_DIR, tools={"kicad_library": library}))
    patch_board(node, spec, pdata)
    common.WORK_DIR.mkdir(parents=True, exist_ok=True)
    unrouted = common.WORK_DIR / f"{common.PROJECT_ID}_unrouted.kicad_pcb"
    unrouted.write_bytes(sexpr.dumps(node).encode("utf-8"))
    ir_hash = common.dump_ir(ir, common.KICAD_DIR / "ir" / f"{common.PROJECT_ID}.ir_with_pcb.json")
    info = {"ir_content_hash": ir_hash, "unrouted_board": unrouted.name, "sha256": common.sha256_file(unrouted),
            "placement_sha256": common.sha256_file(common.KICAD_DIR / "placement.json")}
    (common.WORK_DIR / "unrouted_manifest.json").write_bytes((json.dumps(info, indent=1) + "\n").encode("utf-8"))
    print(json.dumps(info, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
