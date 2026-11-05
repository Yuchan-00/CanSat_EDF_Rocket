"""Spec -> IR -> hierarchical KiCad 10 schematic (+ .kicad_pro with the board design rules).

Invariants
----------
* Every sheet body is produced by the AI_EDA engine's library-backed
  ``SchematicCompiler`` (library symbols embedded verbatim, pin numbers / types
  checked against the KiCad library on disk, every pin either in a net or
  no-connect, refusal of any layout where KiCad could merge nets). This script
  only (a) splits the IR per block, (b) lays big symbols and small symbols out
  on two grids instead of one (same geometry checks run afterwards), and
  (c) wraps the sheets into a hierarchy: sub-sheet symbol instance paths,
  DNP flags, PWR_FLAGs, root sheet. Those are the engine gaps reported in
  README_ko.md (the engine writes one flat sheet, no DNP, no power flags).
* Connectivity is by global labels named after the spec nets, so the
  netlist reproduces ``spec/carrier_spec.json`` exactly.
* Deterministic: all UUIDs are uuid5 (``ai_eda.compilers.ids``), no time stamps.
"""

from __future__ import annotations

import json
import sys

from ai_eda.compilers.ids import net_item_uuid, pin_uuid, stable_uuid, symbol_uuid
from ai_eda.compilers.schematic import SchematicCompiler, _effects, _global_label, _property, _wire
from ai_eda.compilers.schematic_layout import (
    Extent,
    grid_positions,
    label_orientation,
    layout_pitch,
    natural_ref_key,
    snap,
    stub_end,
    symbol_extent,
)
from ai_eda.compilers.pins import load_verified_symbol, pad_pin_types
from ai_eda.ir import CircuitIR, LibraryRef, Net, PinElectricalType
from ai_eda.tools.kicad import sexpr
from ai_eda.tools.kicad.library import KicadLibrary
from ai_eda.tools.kicad.sexpr import Q, S

import common

ROOT_UUID = stable_uuid(common.PROJECT_ID, "sheet", "root")  # == ai_eda.compilers.ids.sheet_uuid
#: net-class track widths (mm); README_ko.md states them and the routing trade-off measurements
POWER_W = 0.5
SERVO_W = 1.5
#: signal track / clearance: JLCPCB 2-layer standard (0.127 mm min; 0.2 mm tracks, 0.13 mm clearance) on both layer
#: counts. The user allowed JLC's finer 4-layer limits (0.09 mm) only where the 2-layer rules do not route; measured on
#: this placement (scratch try8 vs try9, Freerouting signals stage): 0.15 / 0.1 mm left 35 open, 0.2 / 0.13 mm 34 ->
#: no gain, so the 2-layer rules stay.
SIG_W = 0.2
CLR = 0.13
#: board minimum track: 0.127 mm (JLC 2-layer); on 4 layers 0.1 mm so Freerouting's own short neck-downs into fine-pitch
#: pads stay as routed (widening them to 0.127 mm broke the 0.13 mm clearance next to R42, M) - JLC 4-layer minimum
#: 0.09 mm (A), allowed by the user for 4 layers where needed
MIN_TRACK = 0.127 if common.LAYERS == 2 else 0.1
MIN_CLR = 0.127


def sheet_symbol_uuid(stem: str) -> str:
    return stable_uuid(common.PROJECT_ID, "sheet_symbol", stem)


def sheet_file_uuid(stem: str) -> str:
    return stable_uuid(common.PROJECT_ID, "sheet_file", stem)


class BlockSchematicCompiler(SchematicCompiler):
    """Engine compiler with a two-grid layout: multi-pin symbols in a top row, small parts below."""

    BIG_PINS = 9

    def _check_geometry(self, placed, pins):  # type: ignore[override]
        """Library symbols with stacked pins (ATmega128A VCC 21/52, GND 22/53/63) are allowed only when
        every pin of the stack is on the same net; the stack is then checked once (engine gap: it refuses
        all stacked pins)."""
        groups: dict[tuple[str, tuple[float, float]], list[tuple[str, str]]] = {}
        for key, pp in pins.items():
            groups.setdefault((key[0], pp.position), []).append(key)
        filtered = {}
        self._stack_dupes = set()
        for (ref, _pos), keys in groups.items():
            keys.sort(key=lambda k: natural_ref_key(k[1]))
            if len(keys) > 1:
                nets = {self._pin_nets.get(k) for k in keys}
                if len(nets) != 1 or None in nets:
                    raise ValueError(f"stacked pins {keys} are on different nets {nets}")
                self._stack_dupes.update(keys[1:])
            filtered[keys[0]] = pins[keys[0]]
        self._engine_check_geometry(placed, filtered)

    @staticmethod
    def _engine_check_geometry(placed, pins) -> None:
        """Same checks as ``SchematicCompiler._check_geometry`` with a sort key that accepts mixed
        pin names ("1" and "MP" of a MountingPin connector - engine gap: its sort raises TypeError)."""
        from ai_eda.compilers.schematic_layout import point_on_segment
        from ai_eda.errors import CompileError

        refs = sorted(placed, key=_safe_key)
        for i, a in enumerate(refs):
            for b in refs[i + 1:]:
                if placed[a].extent.overlaps(placed[b].extent):
                    raise CompileError(f"symbols {a} and {b} overlap on the schematic grid")
        points: dict = {}
        stubs: dict = {}
        for key in sorted(pins, key=lambda k: (_safe_key(k[0]), _safe_key(k[1]))):
            pp = pins[key]
            label = f"{pp.ref}.{pp.pin.number}"
            end = stub_end(pp.position, pp.body_dir)
            for point, what in ((pp.position, f"pin {label}"), (end, f"stub end of {label}")):
                if point in points:
                    raise CompileError(f"{what} at {point} coincides with {points[point]}; KiCad would connect them")
                points[point] = what
            stubs[key] = (pp.position, end)
        for key, (a, b) in stubs.items():
            for point, what in points.items():
                if point in (a, b):
                    continue
                if point_on_segment(point, a, b):
                    raise CompileError(f"{what} at {point} lies on the wire stub of {key[0]}.{key[1]}; KiCad would connect them")

    def _connectivity(self, ir, placed, pins):  # type: ignore[override]
        no_connects, wires, labels = self._engine_connectivity(ir, placed, pins)
        # stacked duplicates: drop the second wire / label at the same coordinates
        seen_w, out_w = set(), []
        for w in wires:
            key = sexpr.dumps(sexpr.find(w, "pts"))
            if key not in seen_w:
                seen_w.add(key)
                out_w.append(w)
        seen_l, out_l = set(), []
        for lab in labels:
            at = sexpr.find(lab, "at")
            key = (str(lab[1]), str(at[1]), str(at[2]))
            if key not in seen_l:
                seen_l.add(key)
                out_l.append(lab)
        return no_connects, out_w, out_l

    def _engine_connectivity(self, ir, placed, pins):
        """``SchematicCompiler._connectivity`` with the mixed-pin-name sort key (see above)."""
        from ai_eda.compilers.schematic import _no_connect
        from ai_eda.errors import CompileError

        project_id = ir.project.id
        owner: dict = {}
        wires: list = []
        labels: list = []
        for net in sorted(ir.nets, key=lambda n: n.name):
            for pref in sorted(net.pins, key=lambda p: (_safe_key(p.component_ref), _safe_key(p.pin_number))):
                key = (pref.component_ref, pref.pin_number)
                if key not in pins:
                    raise CompileError(f"net {net.name!r}: {pref.component_ref} has no pin {pref.pin_number!r}")
                if key in owner:
                    if owner[key] == net.name:
                        continue
                    raise CompileError(f"pin {key} is in both nets {owner[key]!r} and {net.name!r}")
                owner[key] = net.name
                pp = pins[key]
                end = stub_end(pp.position, pp.body_dir)
                angle, justify = label_orientation(pp.body_dir)
                wires.append(_wire(pp.position, end, net_item_uuid(project_id, "wire", pp.ref, pp.pin.number)))
                labels.append(_global_label(net.name, end, angle, justify,
                                            net_item_uuid(project_id, "global_label", net.name, pp.ref, pp.pin.number)))
        no_connects: list = []
        for key in sorted(pins, key=lambda k: (_safe_key(k[0]), _safe_key(k[1]))):
            if key in owner:
                continue
            ref, number = key
            ir_pin = placed[ref].component.pin(number)
            if ir_pin is not None and ir_pin.electrical_type is PinElectricalType.NO_CONNECT:
                no_connects.append(_no_connect(pins[key].position, net_item_uuid(project_id, "no_connect", ref, number)))
                continue
            raise CompileError(f"pin {ref}.{number} is not in any net and not no_connect")
        return no_connects, wires, labels

    def _place(self, components, library, pin_nets):  # type: ignore[override]
        from ai_eda.compilers.schematic import _PlacedSymbol

        self._pin_nets = dict(pin_nets)

        resolved = {}
        extents: dict[str, Extent] = {}
        for c in components:
            symbol = load_verified_symbol(c, library)
            pad_pin_types(c, symbol)
            label_lengths = {p.number: len(pin_nets[(c.ref, p.number)]) for p in symbol.pins if (c.ref, p.number) in pin_nets}
            resolved[c.ref] = symbol
            extents[c.ref] = symbol_extent(symbol, label_lengths)
        big = [c.ref for c in components if len(resolved[c.ref].pins) >= self.BIG_PINS]
        small = [c.ref for c in components if c.ref not in big]
        positions: dict[str, tuple[float, float]] = {}
        origin_x, origin_y = 50.8, 50.8
        y_next = origin_y
        if big:
            pitch = layout_pitch([extents[r] for r in big], min_pitch=25.4)
            # one row of big symbols, origin shifted so the most negative extent clears the frame
            ymin = min(extents[r].ymin for r in big)
            ymax = max(extents[r].ymax for r in big)
            xmin = min(extents[r].xmin for r in big)
            oy = snap(_ceil(origin_y - ymin))
            ox = snap(_ceil(origin_x - xmin))
            positions.update(grid_positions(big, (ox, oy), pitch, columns=max(1, len(big))))
            y_next = oy + ymax + 7.62
        if small:
            pitch = layout_pitch([extents[r] for r in small], min_pitch=15.24)
            ymin = min(extents[r].ymin for r in small)
            xmin = min(extents[r].xmin for r in small)
            oy = snap(_ceil(y_next - ymin))
            ox = snap(_ceil(origin_x - xmin))
            cols = 8 if len(small) > 24 else 6
            positions.update(grid_positions(small, (ox, oy), pitch, columns=cols))
        placed = {}
        for c in components:
            x, y = positions[c.ref]
            placed[c.ref] = _PlacedSymbol(
                component=c, symbol=resolved[c.ref], footprint_id=self._footprint_id(c, library),
                x=x, y=y, extent=extents[c.ref].shifted(x, y),
            )
        return placed


def _safe_key(s: str) -> tuple:
    return tuple((0, p, "") if isinstance(p, int) else (1, 0, p) for p in natural_ref_key(s))


def _ceil(v: float, step: float = 2.54) -> float:
    import math

    return snap(math.ceil(round(v / step, 6)) * step)


def sub_ir(ir: CircuitIR, refs: set[str]) -> CircuitIR:
    comps = [c for c in ir.components if c.ref in refs]
    nets = []
    for n in ir.nets:
        pins = [p for p in n.pins if p.component_ref in refs]
        if pins:
            nets.append(Net(name=n.name, kind=n.kind, pins=pins, net_class=n.net_class, provenance=n.provenance))
    return ir.model_copy(update={"components": comps, "nets": nets})


def needs_flag(ir: CircuitIR, library: KicadLibrary) -> list[str]:
    """Nets with a power_in pin and no power_out pin: ERC needs a PWR_FLAG declaring the external source."""
    types = {}
    for c in ir.components:
        for p in c.pins:
            types[(c.ref, p.number)] = p.electrical_type
    out = []
    for n in sorted(ir.nets, key=lambda n: n.name):
        ts = [types[(p.component_ref, p.pin_number)] for p in n.pins]
        if PinElectricalType.POWER_IN in ts and PinElectricalType.POWER_OUT not in ts:
            out.append(n.name)
    return out


PAPERS = [("A4", 297, 210), ("A3", 420, 297), ("A2", 594, 420), ("A1", 841, 594), ("A0", 1189, 841)]


def paper_for(node: list) -> str:
    xs, ys = [0.0], [0.0]
    for item in node:
        h = sexpr.head(item)
        if h in ("symbol", "global_label", "no_connect"):
            at = sexpr.find(item, "at")
            if at is not None:
                xs.append(sexpr.to_float(at[1]))
                ys.append(sexpr.to_float(at[2]))
    w, h_ = max(xs) + 80, max(ys) + 50
    for name, pw, ph in PAPERS:
        if w <= pw and h_ <= ph:
            return name
    return "A0"


def patch_sheet(node: list, stem: str, title: str, dnp_refs: set[str], no_bom: set[str] = frozenset()) -> None:
    """Turn an engine root sheet into a sub-sheet of the hierarchy (in place)."""
    sub_path = f"/{ROOT_UUID}/{sheet_symbol_uuid(stem)}"
    for i, item in enumerate(node):
        h = sexpr.head(item)
        if h == "uuid":
            node[i] = S("uuid", Q(sheet_file_uuid(stem)))
        elif h == "title_block":
            node[i] = S("title_block", S("title", Q(f"VTOL carrier - {title}")), S("rev", Q("2c")),
                        S("comment", 1, Q("generated by scripts/build_schematic.py via ai_eda SchematicCompiler from spec/carrier_spec.json")),
                        S("comment", 2, Q("NOT_VERIFIED: never built or powered")))
        elif h == "symbol":
            ref = None
            for prop in sexpr.find_all(item, "property"):
                if str(prop[1]) == "Reference":
                    ref = str(prop[2])
            inst = sexpr.find(item, "instances")
            proj = sexpr.find(inst, "project")
            path = sexpr.find(proj, "path")
            path[1] = Q(sub_path)
            for j, child in enumerate(item):
                if sexpr.head(child) == "dnp" and ref in dnp_refs:
                    item[j] = S("dnp", True)
                elif sexpr.head(child) == "in_bom" and ref in no_bom:
                    # the library footprint says exclude_from_bom (net tie, solder jumper): keep parity
                    item[j] = S("in_bom", False)
    # drop the per-file sheet_instances (only the root carries them)
    node[:] = [it for it in node if sexpr.head(it) != "sheet_instances"]
    for i, item in enumerate(node):
        if sexpr.head(item) == "paper":
            node[i] = S("paper", Q(paper_for(node)))


def add_power_flags(node: list, nets: list[str], library: KicadLibrary, stem: str) -> None:
    """PWR_FLAG + stub + global label per net, in a row under everything else on the sheet."""
    flag = library.load_symbol(LibraryRef(library="power", name="PWR_FLAG"))
    libs = sexpr.find(node, "lib_symbols")
    entries = [e for e in libs[1:]] + [flag.lib_symbols_entry()]
    entries.sort(key=lambda e: str(e[1]))
    libs[1:] = entries
    ys = [sexpr.to_float(sexpr.find(it, "at")[2]) for it in node if sexpr.head(it) in ("symbol", "global_label")]
    y0 = _ceil(max(ys) + 25.4)
    x0 = 50.8
    sub_path = f"/{ROOT_UUID}/{sheet_symbol_uuid(stem)}"
    insert_at = max(i for i, it in enumerate(node) if sexpr.head(it) == "symbol") + 1
    new = []
    for k, net in enumerate(nets):
        x = snap(x0 + k * 22.86)
        y = y0
        ref = f"#FLG{k + 1:02d}"
        pin_pos = (x, y)
        end = stub_end(pin_pos, (0.0, -1.0))  # body is above the pin: stub goes down
        angle, justify = label_orientation((0.0, -1.0))
        new.append(_wire(pin_pos, end, net_item_uuid(common.PROJECT_ID, "wire", ref, "1")))
        new.append(_global_label(net, end, angle, justify, net_item_uuid(common.PROJECT_ID, "global_label", net, ref, "1")))
        new.append(S(
            "symbol",
            S("lib_id", Q("power:PWR_FLAG")),
            S("at", x, y, 0),
            S("unit", 1),
            S("exclude_from_sim", False),
            S("in_bom", True),
            S("on_board", True),
            S("dnp", False),
            S("uuid", Q(symbol_uuid(common.PROJECT_ID, ref))),
            S("property", Q("Reference"), Q(ref), S("at", x, y + 1.905, 0), S("hide", True), _effects()),
            S("property", Q("Value"), Q("PWR_FLAG"), S("at", x, snap(y - 3.81), 0), _effects()),
            _property("Footprint", "", x, y, hide=True),
            _property("Datasheet", "", x, y, hide=True),
            _property("Description", f"ERC: {net} is supplied from outside this net's pins", x, y, hide=True),
            S("pin", Q("1"), S("uuid", Q(pin_uuid(common.PROJECT_ID, ref, "1")))),
            S("instances", S("project", Q(common.PROJECT_ID), S("path", Q(sub_path), S("reference", Q(ref)), S("unit", 1)))),
        ))
    node[insert_at:insert_at] = new


def root_sheet(pages: list[tuple[str, str, str]]) -> list:
    sheets = []
    for k, (name, stem, title) in enumerate(pages):
        col, row = k % 4, k // 4
        x, y = snap(25.4 + col * 66.04), snap(40.64 + row * 50.8)
        w, h = 50.8, 30.48
        sheets.append(S(
            "sheet",
            S("at", x, y),
            S("size", w, h),
            S("exclude_from_sim", False),
            S("in_bom", True),
            S("on_board", True),
            S("dnp", False),
            S("fields_autoplaced", True),
            S("stroke", S("width", 0.1524), S("type", "solid")),
            S("fill", S("color", 0, 0, 0, 0.0)),
            S("uuid", Q(sheet_symbol_uuid(stem))),
            S("property", Q("Sheetname"), Q(name), S("at", x, snap(y - 0.7116), 0),
              S("effects", S("font", S("size", 1.27, 1.27)), S("justify", "left", "bottom"))),
            S("property", Q("Sheetfile"), Q(f"{stem}.kicad_sch"), S("at", x, snap(y + h + 0.5846), 0),
              S("effects", S("font", S("size", 1.27, 1.27)), S("justify", "left", "top"))),
            S("instances", S("project", Q(common.PROJECT_ID), S("path", Q(f"/{ROOT_UUID}"), S("page", Q(str(k + 2)))))),
        ))
    texts = [
        "VTOL avionics carrier rev 2c (ESP-12E, 1S logic battery + Pololu 5 V buck-boost module, 5th ATmega128A and release header removed), 76 x 70 x 1.6 mm - generated from spec/carrier_spec.json",
        "Nets connect by global labels (net names = spec nets). Logic GND and GND_PWR meet only at NT1 (Power sheet).",
        "NOT_VERIFIED: never built or powered. Module pin orders (GY-521, BMP280, OpenLog, boost module, NEO-7M lead, 1S pack plug) are assumptions - check silkscreen.",
    ]
    text_nodes = [
        S("text", Q(t), S("exclude_from_sim", False), S("at", 25.4, snap(20.32 + 5.08 * i), 0),
          S("effects", S("font", S("size", 1.778, 1.778)), S("justify", "left", "bottom")),
          S("uuid", Q(net_item_uuid(common.PROJECT_ID, "root_text", i))))
        for i, t in enumerate(texts)
    ]
    return S(
        "kicad_sch",
        S("version", 20260101),
        S("generator", Q("eeschema")),
        S("generator_version", Q("10.0")),
        S("uuid", Q(ROOT_UUID)),
        S("paper", Q("A4")),
        S("title_block", S("title", Q("VTOL avionics carrier")), S("rev", Q("2c")),
          S("comment", 1, Q("generated by scripts/build_schematic.py (hierarchy) + ai_eda SchematicCompiler (sheets)")),
          S("comment", 2, Q("NOT_VERIFIED: never built or powered"))),
        S("lib_symbols"),
        *text_nodes,
        *sheets,
        S("sheet_instances", S("path", Q("/"), S("page", Q("1")))),
        S("embedded_fonts", False),
    )


def project_file(pages: list[tuple[str, str, str]]) -> dict:
    """``.kicad_pro``: JLCPCB 2-layer standard limits as minimums, wider net-class widths (README_ko.md)."""
    classes = [
        # one clearance for every class, 0.13 mm (JLCPCB 2-layer minimum 0.127 mm, A; all nets <= 5 V): Freerouting
        # applied the smaller clearance between classes while KiCad DRC uses the larger one (164 errors with 0.2 mm on
        # the power classes, scratch DRC, M)
        {"name": "Default", "clearance": CLR, "track_width": SIG_W, "via_diameter": 0.6, "via_drill": 0.3, "priority": 2147483647},
        # logic rails +5V / +3V3 / GPS_VCC / AVCC: 5 V 683 mA peak, 3.3 V 448 mA peak (spec power, E)
        {"name": "Power", "clearance": CLR, "track_width": POWER_W, "via_diameter": 0.6, "via_drill": 0.3, "priority": 2},
        # logic-battery path to the boost module: 1.34 A peak at a 3.0 V cell (spec power, E) -> >= 1.0 mm (spec layout_rules)
        {"name": "PowerHigh", "clearance": CLR, "track_width": 1.0, "via_diameter": 0.8, "via_drill": 0.4, "priority": 1},
        # servo rail + return (4 x MG90S, stall ~0.7 A each, A): >= 1.5 mm (spec layout_rules), 1.2 / 0.6 mm vias
        {"name": "Servo", "clearance": CLR, "track_width": SERVO_W, "via_diameter": 1.2, "via_drill": 0.6, "priority": 0},
    ]
    for c in classes:
        c.update({"bus_width": 12, "diff_pair_gap": 0.25, "diff_pair_via_gap": 0.25, "diff_pair_width": 0.2, "line_style": 0,
                  "microvia_diameter": 0.3, "microvia_drill": 0.1, "pcb_color": "rgba(0, 0, 0, 0.000)",
                  "schematic_color": "rgba(0, 0, 0, 0.000)", "wire_width": 6})
    patterns = [{"netclass": "Servo", "pattern": n} for n in common.SERVO_NETS]
    patterns += [{"netclass": "PowerHigh", "pattern": n} for n in common.POWER_NETS_WIDE]
    patterns += [{"netclass": "Power", "pattern": n} for n in common.POWER_NETS]
    patterns += [{"netclass": "Power", "pattern": "*_AVCC"}]
    return {
        "board": {
            "design_settings": {
                "defaults": {"board_outline_line_width": 0.05, "copper_line_width": 0.2, "silk_line_width": 0.12,
                             "silk_text_size_h": 0.8, "silk_text_size_v": 0.8, "silk_text_thickness": 0.12},
                "rules": {
                    "allow_blind_buried_vias": False,
                    "allow_microvias": False,
                    "max_error": 0.005,
                    "min_clearance": MIN_CLR,
                    "min_connection": 0.0,
                    "min_copper_edge_clearance": 0.3,
                    "min_groove_width": 0.0,
                    "min_hole_clearance": 0.25,
                    "min_hole_to_hole": 0.5,
                    "min_microvia_diameter": 0.3,
                    "min_microvia_drill": 0.1,
                    "min_resolved_spokes": 1,
                    "min_silk_clearance": 0.0,
                    "min_text_height": 0.8,
                    "min_text_thickness": 0.08,
                    "min_through_hole_diameter": 0.3,
                    "min_track_width": MIN_TRACK,
                    "min_via_annular_width": 0.13,
                    "min_via_diameter": 0.6,
                    "solder_mask_to_copper_clearance": 0.0,
                    "use_height_for_length_calcs": True,
                },
                "track_widths": [0.0, SIG_W, POWER_W, 1.0, SERVO_W],
                "via_dimensions": [{"diameter": 0.0, "drill": 0.0}, {"diameter": 0.6, "drill": 0.3}, {"diameter": 0.8, "drill": 0.4},
                                   {"diameter": 1.2, "drill": 0.6}],
                "meta": {"version": 2},
            },
        },
        "boards": [],
        "libraries": {"pinned_footprint_libs": [], "pinned_symbol_libs": []},
        "meta": {"filename": f"{common.PROJECT_ID}.kicad_pro", "version": 3},
        "net_settings": {"classes": classes, "meta": {"version": 4}, "net_colors": None,
                         "netclass_assignments": None, "netclass_patterns": patterns},
        "pcbnew": {"last_paths": {}, "page_layout_descr_file": ""},
        "schematic": {"legacy_lib_dir": "", "legacy_lib_list": []},
        "sheets": [[ROOT_UUID, "Root"]] + [[sheet_symbol_uuid(stem), name] for name, stem, _t in pages],
        "text_variables": {},
    }


def main() -> int:
    spec = common.load_spec()
    library = KicadLibrary()
    ir = common.build_ir(spec, library)
    ir_hash = common.dump_ir(ir, common.KICAD_DIR / "ir" / f"{common.PROJECT_ID}.ir.json")
    blocks = common.block_of(spec)
    dnp = {p["refdes"] for p in spec["parts"] if not common.populated(p)}
    flags = needs_flag(ir, library)
    no_bom = {c.ref for c in ir.components if "exclude_from_bom" in library.load_footprint(c.footprint).attributes}
    pages = []
    compiler = BlockSchematicCompiler()
    written = []
    for name, stem, block_names, title in common.SHEETS:
        refs = {r for r, b in blocks.items() if b in block_names}
        node = compiler.build(sub_ir(ir, refs), library)
        patch_sheet(node, stem, title, dnp, no_bom)
        if stem == "power":
            add_power_flags(node, flags, library, stem)
        path = common.KICAD_DIR / f"{stem}.kicad_sch"
        path.write_bytes(sexpr.dumps(node).encode("utf-8"))
        written.append(path)
        pages.append((name, stem, title))
    root = root_sheet(pages)
    rp = common.KICAD_DIR / f"{common.PROJECT_ID}.kicad_sch"
    rp.write_bytes(sexpr.dumps(root).encode("utf-8"))
    pro = common.KICAD_DIR / f"{common.PROJECT_ID}.kicad_pro"
    pro.write_bytes((json.dumps(project_file(pages), indent=2) + "\n").encode("utf-8"))
    manifest = {
        "ir_content_hash": ir_hash,
        "spec_sha256": common.sha256_file(common.SPEC_PATH),
        "power_flags": flags,
        "files": {p.name: common.sha256_file(p) for p in [rp, pro, *written]},
    }
    (common.KICAD_DIR / "schematic_manifest.json").write_bytes((json.dumps(manifest, indent=1) + "\n").encode("utf-8"))
    print(json.dumps(manifest, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
