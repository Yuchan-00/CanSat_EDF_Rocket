"""Carrier rev 3: coloured STEP models of the user-fitted modules for the sensor-mounting renders.

Invariant this module enforces: every dimension comes from ``spec/module_dimensions.json`` (each number tagged F =
named document or A = assumed) and every connector pad position from the on-disk KiCad library footprint (a mismatch
is an error, never a guess).  The models are PICTURES for ``render_rev3.py``; they are not fit evidence - the
OpenLog / BMP280 fit is decided by the fit check from calliper dimensions (plan D15).

Each mounted model is written directly in the 3D frame of its mating connector footprint (x = footprint x,
y = -footprint y, z up from the board top, origin = footprint origin), so ``render_rev3.py`` attaches it with
offset (0, 0, 0) and rotation (0, 0, 0) and KiCad applies the footprint rotation.

Outputs (deterministic: same inputs -> byte-identical files; OCCT's two run-dependent parts - the header timestamp
and the order of the colour records - are normalised by ``export_step``):
    kicad/3d/gy521_on_J1.step, bmp280_in_J2.step, openlog_in_J5.step, gh_plug_in_J7.step,
    molex_22035025_J14.step, neo7m_module.step, height_limit_frame.step, models_manifest.json

Run (CadQuery 2.8.0 lives in the main checkout's venv; this script and its outputs stay in the worktree):
    set PYTHONDONTWRITEBYTECODE=1
    "B:/0)AI_Cad/CanSat_EDF_Rocket/.venv/Scripts/python.exe" -B -X utf8 scripts/make_module_models.py
Options: --out DIR (default kicad/3d); --keep-stamp (canonical colour records, real timestamp: two such runs differ
in the FILE_NAME header line only); --raw (exactly what OCCT wrote).  Neither demonstration mode writes the manifest.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import sys
from dataclasses import dataclass, field
from pathlib import Path

import cadquery as cq

ROOT = Path(__file__).resolve().parents[1]
SPEC = ROOT / "spec" / "module_dimensions.json"
OUT_DIR = ROOT / "kicad" / "3d"
FIXED_STAMP = "1970-01-01T00:00:00"
STAMP_RE = re.compile(r"'\d{4}-\d\d-\d\dT\d\d:\d\d:\d\d'")
PAD_RE = re.compile(r'\(pad\s+"([^"]*)"\s+\w+\s+\w+\s*\(at\s+(-?[\d.]+)\s+(-?[\d.]+)')

# colour groups (r, g, b); one assembly child per group and model
COLORS: dict[str, tuple[float, float, float]] = {
    "pcb_blue": (0.05, 0.22, 0.62),
    "pcb_purple": (0.36, 0.10, 0.52),
    "pcb_red": (0.72, 0.06, 0.09),
    "pcb_green": (0.05, 0.40, 0.18),
    "ic_dark": (0.12, 0.12, 0.13),
    "plastic_black": (0.07, 0.07, 0.08),
    "plastic_white": (0.93, 0.92, 0.86),
    "pin_gold": (0.85, 0.68, 0.22),
    "metal": (0.78, 0.79, 0.80),
    "silk_white": (0.96, 0.96, 0.96),
    "passive_tan": (0.70, 0.56, 0.38),
    "led_green": (0.15, 0.80, 0.25),
    "led_blue": (0.15, 0.35, 0.95),
    "card_blue": (0.10, 0.16, 0.42),
    "card_label": (0.90, 0.90, 0.92),
    "ceramic": (0.86, 0.82, 0.70),
    "foam": (0.55, 0.57, 0.60),
    "wire_red": (0.82, 0.08, 0.08),
    "wire_yellow": (0.93, 0.78, 0.10),
    "wire_green": (0.10, 0.60, 0.20),
    "wire_black": (0.05, 0.05, 0.05),
    "coax": (0.20, 0.20, 0.22),
    "limit_red": (0.95, 0.10, 0.10),
}


# ------------------------------------------------------------------------------------------------- inputs
def load_dims(path: Path = SPEC) -> dict:
    """The dimension file; every leaf used here must carry tag F or A (checked by ``val``)."""
    return json.loads(path.read_text(encoding="utf-8"))


def val(dims: dict, section: str, key: str):
    """One dimension value; refuses an entry without an F / A tag (no untagged number enters a model)."""
    e = dims[section][key]
    if not isinstance(e, dict) or e.get("tag") not in ("F", "A") or "v" not in e or not e.get("src"):
        raise ValueError(f"module_dimensions.json {section}.{key}: needs v, tag F|A and src")
    return e["v"]


def kicad_share() -> Path:
    """KiCad library root (``<root>/share/kicad``), discovered - never assumed to exist."""
    cands: list[Path] = []
    for env in ("KICAD10_FOOTPRINT_DIR", "KICAD_FOOTPRINT_DIR"):
        if os.environ.get(env):
            cands.append(Path(os.environ[env]).parent)
    cli = shutil.which("kicad-cli")
    if cli:
        cands.append(Path(cli).resolve().parent.parent / "share" / "kicad")
    cands.append(Path(r"C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/share/kicad"))
    cands.append(Path("/usr/share/kicad"))
    for c in cands:
        if (c / "footprints").is_dir():
            return c
    raise FileNotFoundError("KiCad footprint libraries not found (set KICAD10_FOOTPRINT_DIR)")


def library_pads(lib: str, name: str) -> dict[str, tuple[float, float]]:
    """Numbered pad centres of a library footprint, read from the .kicad_mod on disk (footprint frame, y down)."""
    path = kicad_share() / "footprints" / f"{lib}.pretty" / f"{name}.kicad_mod"
    if not path.is_file():
        raise FileNotFoundError(f"library footprint missing: {path}")
    pads: dict[str, tuple[float, float]] = {}
    for num, x, y in PAD_RE.findall(path.read_text(encoding="utf-8")):
        if num and num not in pads:
            pads[num] = (float(x), float(y))
    return pads


def row_pads(lib: str, name: str, n: int, pitch: float, axis: str) -> list[tuple[float, float]]:
    """Pads 1..n of a single-row connector in the 3D frame (y inverted); checks count and pitch against the spec."""
    pads = library_pads(lib, name)
    missing = [str(i) for i in range(1, n + 1) if str(i) not in pads]
    if missing:
        raise ValueError(f"{lib}:{name}: pads {missing} not in the library footprint")
    pts = [pads[str(i)] for i in range(1, n + 1)]
    for a, b in zip(pts[:-1], pts[1:]):
        d = (b[0] - a[0], b[1] - a[1])
        want = (pitch, 0.0) if axis == "x" else (0.0, pitch)
        if abs(d[0] - want[0]) > 1e-6 or abs(d[1] - want[1]) > 1e-6:
            raise ValueError(f"{lib}:{name}: pad step {d} differs from the spec pitch {want}")
    return [(x, -y) for x, y in pts]


# ------------------------------------------------------------------------------------------------- geometry
def box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> cq.Shape:
    xa, xb, ya, yb, za, zb = min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1), min(z0, z1), max(z0, z1)
    return cq.Solid.makeBox(xb - xa, yb - ya, zb - za, cq.Vector(xa, ya, za))


def cyl_z(x: float, y: float, z0: float, z1: float, r: float) -> cq.Shape:
    return cq.Solid.makeCylinder(r, abs(z1 - z0), cq.Vector(x, y, min(z0, z1)), cq.Vector(0, 0, 1))


def cbox(cx: float, cy: float, sx: float, sy: float, z0: float, z1: float) -> cq.Shape:
    return box(cx - sx / 2, cx + sx / 2, cy - sy / 2, cy + sy / 2, z0, z1)


@dataclass
class Map2:
    """Axis-aligned map from a module's own plan coordinates (u, v) to the connector 3D frame (x, y)."""
    ox: float
    oy: float
    u: tuple[int, int]
    v: tuple[int, int]

    def pt(self, a: float, b: float) -> tuple[float, float]:
        return (self.ox + a * self.u[0] + b * self.v[0], self.oy + a * self.u[1] + b * self.v[1])

    def rect(self, a0: float, a1: float, b0: float, b1: float) -> tuple[float, float, float, float]:
        (x0, y0), (x1, y1) = self.pt(a0, b0), self.pt(a1, b1)
        return (min(x0, x1), max(x0, x1), min(y0, y1), max(y0, y1))

    def box(self, a0: float, a1: float, b0: float, b1: float, z0: float, z1: float) -> cq.Shape:
        return box(*self.rect(a0, a1, b0, b1), z0, z1)

    def cbox(self, ca: float, cb: float, sa: float, sb: float, z0: float, z1: float) -> cq.Shape:
        return self.box(ca - sa / 2, ca + sa / 2, cb - sb / 2, cb + sb / 2, z0, z1)

    @property
    def face_up_is_local_top(self) -> bool:
        """True when the module's own top face (u right, v up, seen from above) is up after the mapping."""
        return self.u[0] * self.v[1] - self.u[1] * self.v[0] > 0


@dataclass
class Model:
    """One STEP file: solids grouped by colour, plus the numbers the render / report steps need."""
    name: str
    attach_ref: str | None
    groups: dict[str, list[cq.Shape]] = field(default_factory=dict)
    info: dict = field(default_factory=dict)

    def add(self, group: str, *shapes: cq.Shape) -> None:
        if group not in COLORS:
            raise KeyError(f"unknown colour group {group}")
        self.groups.setdefault(group, []).extend(shapes)

    def bbox(self) -> dict[str, list[float]]:
        lo, hi = [1e9] * 3, [-1e9] * 3
        for shapes in self.groups.values():
            for s in shapes:
                b = s.BoundingBox()
                lo = [min(lo[0], b.xmin), min(lo[1], b.ymin), min(lo[2], b.zmin)]
                hi = [max(hi[0], b.xmax), max(hi[1], b.ymax), max(hi[2], b.zmax)]
        return {"min": [round(v, 4) for v in lo], "max": [round(v, 4) for v in hi]}

    def assembly(self) -> cq.Assembly:
        asm = cq.Assembly(name=self.name)
        for group in sorted(self.groups):
            asm.add(cq.Compound.makeCompound(self.groups[group]), name=f"{self.name}__{group}", color=cq.Color(*COLORS[group]))
        return asm


# ------------------------------------------------------------------------------------------------- pin strips
def straight_strip(m: Model, pins: list[tuple[float, float]], d: dict) -> float:
    """Straight 2.54 mm strip soldered into the carrier under a flat module; returns the plastic top z."""
    ph, pw, sq = val(d, "pin_strip_2p54", "straight_plastic_h"), val(d, "pin_strip_2p54", "straight_plastic_w"), \
        val(d, "pin_strip_2p54", "pin_square")
    top, tail = val(d, "pin_strip_2p54", "straight_pin_above_board"), val(d, "pin_strip_2p54", "straight_tail_below_board")
    xs, ys = [p[0] for p in pins], [p[1] for p in pins]
    m.add("plastic_black", box(min(xs) - pw / 2, max(xs) + pw / 2, min(ys) - pw / 2, max(ys) + pw / 2, 0.0, ph))
    for x, y in pins:
        m.add("pin_gold", cbox(x, y, sq, sq, -tail, top))
    return ph


def right_angle_header(m: Model, pins: list[tuple[float, float]], d: dict, pcb_t: float) -> dict:
    """Right-angle strip on a module lying flat beside a horizontal socket (socket pins at x = 0, module at -x).

    The header plastic lies between the socket face and the module, on the module face that looks at the carrier;
    returns the seating numbers (hole row x, module underside z)."""
    sq = val(d, "pin_strip_2p54", "pin_square")
    pl = val(d, "pin_strip_2p54", "ra_plastic")
    mate = val(d, "pin_strip_2p54", "ra_mating_pin_len")
    h2p = val(d, "pin_strip_2p54", "ra_hole_to_plastic")
    tail = val(d, "pin_strip_2p54", "ra_tail_through_module")
    x_open = val(d, "socket_j2_j5", "opening_x")
    z_axis = val(d, "socket_j2_j5", "contact_axis_z")
    x_row = x_open - pl - h2p
    z_under = z_axis + pl / 2
    ys = [p[1] for p in pins]
    m.add("plastic_black", box(x_open - pl, x_open, min(ys) - 1.27, max(ys) + 1.27, z_axis - pl / 2, z_axis + pl / 2))
    for _x, y in pins:
        m.add("pin_gold", box(x_row - sq / 2, x_open + mate, y - sq / 2, y + sq / 2, z_axis - sq / 2, z_axis + sq / 2))
        m.add("pin_gold", cbox(x_row, y, sq, sq, z_axis - sq / 2, z_under + tail))
        m.add("pin_gold", cbox(x_row, y, 1.6, 1.6, z_under + pcb_t, z_under + pcb_t + 0.04))     # solder pad, upper face
    return {"hole_row_x": round(x_row, 4), "underside_z": round(z_under, 4), "pin_tip_z": round(z_under + tail, 4),
            "contact_axis_z": z_axis}


# ------------------------------------------------------------------------------------------------- models
def gy521_on_j1(d: dict) -> Model:
    """GY-521 soldered flat on its own straight 1x8 strip in J1 (component side up, body toward footprint +x)."""
    g = "gy521"
    n, pitch = val(d, g, "header_n"), val(d, "pin_strip_2p54", "pitch")
    pins = row_pads("Connector_PinHeader_2.54mm", "PinHeader_1x08_P2.54mm_Vertical", n, pitch, "y")
    m = Model("gy521_on_J1", "J1")
    zu = straight_strip(m, pins, d)
    L, W, t, inset = val(d, g, "pcb_l"), val(d, g, "pcb_w"), val(d, g, "pcb_t"), val(d, g, "header_row_inset")
    if d["mounting"]["J1"]["body_side"]["v"] != "+x":
        raise ValueError("mounting.J1.body_side: only +x (toward the board centre on the anchored J1) is modelled")
    # module frame: u along the row from pin 1, v from the row into the body; (u, v, up) right-handed -> component side up
    mp = Map2(pins[0][0], pins[0][1], (0, -1), (1, 0))
    assert mp.face_up_is_local_top
    span = pitch * (n - 1)
    u0, u1, v0, v1 = span / 2 - L / 2, span / 2 + L / 2, -inset, W - inset
    pcb = mp.box(u0, u1, v0, v1, zu, zu + t)
    hd, hi, hs = val(d, g, "hole_d"), val(d, g, "hole_inset"), val(d, g, "hole_spacing")
    for du in (-hs / 2, hs / 2):
        x, y = mp.pt(span / 2 + du, v1 - hi)
        pcb = pcb.cut(cyl_z(x, y, zu - 1, zu + t + 1, hd / 2))
    m.add("pcb_blue", pcb)
    zt = zu + t
    for x, y in pins:
        m.add("pin_gold", cbox(x, y, 1.7, 1.7, zt, zt + 0.04))
    ic = val(d, g, "ic_mpu6050")
    hmax = val(d, g, "part_h_max")
    m.add("ic_dark", mp.cbox(span / 2 - 1.0, 6.6, ic[0], ic[1], zt, zt + ic[2]))                 # MPU-6050 (position A)
    m.add("silk_white", cyl_z(*mp.pt(span / 2 - 1.0 - ic[0] / 2 + 0.6, 6.6 + ic[1] / 2 - 0.6), zt + ic[2], zt + ic[2] + 0.03, 0.25))
    m.add("ic_dark", mp.cbox(span / 2 + 6.2, 4.6, 1.6, 2.9, zt, zt + hmax))                      # 3.3 V regulator
    m.add("passive_tan", mp.cbox(span / 2 + 6.4, 8.6, 1.6, 3.2, zt, zt + hmax))                  # tantalum
    m.add("passive_tan", mp.cbox(span / 2 - 6.4, 8.6, 1.6, 3.2, zt, zt + hmax))
    for k in range(4):
        m.add("passive_tan", mp.cbox(span / 2 - 5.5 + 1.7 * k, 2.9, 0.8, 1.6, zt, zt + 0.5))    # 0603 row
    m.add("led_green", mp.cbox(span / 2 - 7.2, 4.4, 0.8, 1.6, zt, zt + 0.6))
    m.add("silk_white", mp.cbox(0.0, 1.9, 1.3, 0.9, zt, zt + 0.03))                              # pin-1 (VCC) mark
    sh, ss = val(d, g, "support_h"), val(d, g, "support_size")
    m.add("foam", mp.cbox(span / 2, v1 - hi, ss[0], ss[1], 0.0, sh))                             # support under the far edge
    m.info = {"underside_z": zu, "pcb_top_z": round(zt, 4), "parts_top_z": round(zt + hmax, 4),
              "body_rect_fp_frame": [round(c, 4) for c in _fp_rect(mp.rect(u0, u1, v0, v1))],
              "pin1": "VCC at J1 pad 1 (A)", "face_up": "component side (A)"}
    return m


def _fp_rect(r3d: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    """3D-frame rectangle (x0, x1, y0, y1) -> footprint frame (x0, x1, y0, y1) (y inverted)."""
    return (r3d[0], r3d[1], -r3d[3], -r3d[2])


def _flat_module_frame(d: dict, pins: list[tuple[float, float]], pcb_t: float, m: Model) -> tuple[Map2, dict]:
    """Common part of the two modules lying beside a horizontal socket: header + the module plan map.

    Module frame as in its own drawing of the header-side face: u along the row from socket pin 1, v from the hole
    row into the body.  The mapping turns that face toward the carrier (see right_angle_header)."""
    seat = right_angle_header(m, pins, d, pcb_t)
    mp = Map2(seat["hole_row_x"], pins[0][1], (0, -1), (-1, 0))
    assert not mp.face_up_is_local_top          # the header-side face looks at the carrier
    return mp, seat


def bmp280_in_j2(d: dict) -> Model:
    """BMP280 module with a right-angle 1x6 header plugged into the right-angle socket J2, lying flat."""
    g = "bmp280"
    if val(d, g, "header_type") != "right_angle":
        raise ValueError("bmp280.header_type: only 'right_angle' is modelled (a straight header stands upright in J2)")
    n, pitch = val(d, g, "header_n"), val(d, "pin_strip_2p54", "pitch")
    pins = row_pads("Connector_PinSocket_2.54mm", "PinSocket_1x06_P2.54mm_Horizontal", n, pitch, "y")
    m = Model("bmp280_in_J2", "J2")
    L, W, t, inset = val(d, g, "pcb_l"), val(d, g, "pcb_w"), val(d, g, "pcb_t"), val(d, g, "header_row_inset")
    mp, seat = _flat_module_frame(d, pins, t, m)
    zu = seat["underside_z"]
    span = pitch * (n - 1)
    u0, u1, v0, v1 = span / 2 - L / 2, span / 2 + L / 2, -inset, W - inset
    pcb = mp.box(u0, u1, v0, v1, zu, zu + t)
    hd, hi = val(d, g, "hole_d"), val(d, g, "hole_inset")
    hx, hy = mp.pt(span / 2 - 4.2, v1 - hi)
    pcb = pcb.cut(cyl_z(hx, hy, zu - 1, zu + t + 1, hd / 2))
    m.add("pcb_purple", pcb)
    can = val(d, g, "sensor_can")
    face = val(d, g, "face_up")
    if face not in ("back", "component"):
        raise ValueError("bmp280.face_up must be 'back' or 'component'")
    cu, cv = span / 2 + 1.5, 5.6
    if face == "back":       # component side looks at the carrier: sensor and passives hang under the board
        m.add("metal", mp.cbox(cu, cv, can[0], can[1], zu - can[2], zu))
        m.add("passive_tan", mp.cbox(cu - 4.0, cv, 0.8, 1.6, zu - 0.5, zu), mp.cbox(cu + 3.2, cv, 0.8, 1.6, zu - 0.5, zu))
    else:
        zt = zu + t
        m.add("metal", mp.cbox(cu, cv, can[0], can[1], zt, zt + can[2]))
        m.add("passive_tan", mp.cbox(cu - 4.0, cv, 0.8, 1.6, zt, zt + 0.5), mp.cbox(cu + 3.2, cv, 0.8, 1.6, zt, zt + 0.5))
    zt = zu + t
    m.add("silk_white", mp.cbox(0.0, 1.9, 1.3, 0.9, zt, zt + 0.03))                              # pin-1 (VCC) mark
    m.info = {**seat, "pcb_top_z": round(zt, 4), "top_z": seat["pin_tip_z"],
              "under_parts_bottom_z": round(zu - can[2], 4) if face == "back" else zu,
              "body_rect_fp_frame": [round(c, 4) for c in _fp_rect(mp.rect(u0, u1, v0, v1))],
              "pin1": "VCC at J2 pad 1 (A: printed order not checked)", "face_up": f"{face} (A)"}
    return m


def openlog_in_j5(d: dict) -> Model:
    """SparkFun OpenLog with a user-soldered right-angle 1x6 header plugged into J5, microSD side up."""
    g = "openlog"
    if val(d, g, "face_up") != "microsd":
        raise ValueError("openlog.face_up: only 'microsd' puts GRN on J5 pin 1 (OpenLog.brd silk order)")
    n, pitch = val(d, g, "header_n"), val(d, "pin_strip_2p54", "pitch")
    pins = row_pads("Connector_PinSocket_2.54mm", "PinSocket_1x06_P2.54mm_Horizontal", n, pitch, "y")
    hx = val(d, g, "header_x")
    if len(hx) != n or any(abs((b - a) - pitch) > 1e-6 for a, b in zip(hx[:-1], hx[1:])):
        raise ValueError("openlog.header_x does not match the 2.54 mm socket row")
    m = Model("openlog_in_J5", "J5")
    L, W, t, ry = val(d, g, "pcb_l"), val(d, g, "pcb_w"), val(d, g, "pcb_t"), val(d, g, "header_row_y")
    mp, seat = _flat_module_frame(d, pins, t, m)
    zu = seat["underside_z"]
    zt = zu + t

    def e(ex: float, ey: float) -> tuple[float, float]:     # Eagle board coordinates -> module frame (u, v)
        return (ex - hx[0], ey - ry)

    (u0, v0), (u1, v1) = e(0.0, 0.0), e(L, W)
    m.add("pcb_red", mp.box(u0, u1, v0, v1, zu, zu + t))
    # ATmega side (Eagle top) looks at the carrier
    parts = val(d, g, "top_parts")
    tq, tqh = val(d, g, "tqfp32_body"), val(d, g, "tqfp32_h_max")
    cu, cv = e(*parts["U1_atmega328_tqfp32"])
    m.add("ic_dark", mp.cbox(cu, cv, tq[0], tq[1], zu - tqh, zu - (tqh - tq[2])))
    m.add("metal", mp.cbox(cu, cv, tq[0] + 2.0, tq[1] - 1.4, zu - 0.25, zu - 0.1), mp.cbox(cu, cv, tq[0] - 1.4, tq[1] + 2.0, zu - 0.25, zu - 0.1))
    cu, cv = e(*parts["U2_ldo_sot23_5"])
    m.add("ic_dark", mp.cbox(cu, cv, 2.9, 1.6, zu - 1.1, zu))
    cu, cv = e(*parts["Q1_resonator"])
    m.add("passive_tan", mp.cbox(cu, cv, 1.3, 3.2, zu - 0.9, zu))
    for key, grp in (("D3_led_green", "led_green"), ("D1_led_blue", "led_blue")):
        cu, cv = e(*parts[key])
        m.add(grp, mp.cbox(cu, cv, 1.6, 0.8, zu - 0.6, zu))
    for key in ("C4", "R2", "C3", "C2", "C1", "R6", "R1"):
        cu, cv = e(*parts[key])
        m.add("passive_tan", mp.cbox(cu, cv, 0.8, 1.6, zu - 0.5, zu))
    # microSD side (Eagle bottom) is up
    sx0, sy0, sx1, sy1 = val(d, g, "microsd_socket_xy")
    sh = val(d, g, "microsd_socket_h")
    (a0, b0), (a1, b1) = e(sx0, sy0), e(sx1, sy1)
    shell = mp.box(a0, a1, b0, b1, zt, zt + sh)
    (w0, c0), (w1, c1) = e(sx0 + 1.6, sy0 + 3.0), e(sx1 - 1.6, sy1 - 4.5)
    shell = shell.cut(mp.box(w0, w1, c0, c1, zt + sh - 0.5, zt + sh + 1.0))                      # window in the lid: card visible
    m.add("metal", shell)
    card = val(d, g, "microsd_card")
    end_y = val(d, g, "microsd_drawing_end_y")
    ccx = (sx0 + sx1) / 2
    (k0, l0), (k1, l1) = e(ccx - card[0] / 2, end_y - card[1]), e(ccx + card[0] / 2, end_y)
    zc0 = zt + 0.45
    m.add("card_blue", mp.box(k0, k1, l0, l1, zc0, zc0 + card[2]))
    (q0, r0), (q1, r1) = e(ccx - card[0] / 2 + 1.0, end_y - 4.2), e(ccx + card[0] / 2 - 1.0, end_y - 1.0)
    m.add("card_label", mp.box(q0, q1, r0, r1, zc0 + card[2], zc0 + card[2] + 0.03))
    m.add("silk_white", mp.cbox(0.0, 1.9, 1.3, 0.9, zt, zt + 0.03))                              # pin-1 (GRN) mark
    card_rect = _fp_rect(mp.rect(k0, k1, l0, l1))
    m.info = {**seat, "pcb_top_z": round(zt, 4), "top_z": seat["pin_tip_z"], "microsd_top_z": round(zt + sh, 4),
              "under_parts_bottom_z": round(zu - tqh, 4),
              "body_rect_fp_frame": [round(c, 4) for c in _fp_rect(mp.rect(u0, u1, v0, v1))],
              "card_rect_fp_frame": [round(c, 4) for c in card_rect],
              "card_overhang_past_pcb": round(end_y - W, 4),
              "pin1": "GRN at J5 pad 1 (F: OpenLog.brd silk order + plan pin list)", "face_up": "microSD side (F-derived)"}
    return m


def gh_plug_in_j7(d: dict) -> Model:
    """JST GHR-04V-S plug mated in the top-entry header J7 with the first part of the 4-wire lead."""
    g = "gh_plug"
    pads = row_pads("Connector_JST", "JST_GH_BM04B-GHS-TBT_1x04-1MP_P1.25mm_Vertical", 4, val(d, g, "pitch"), "x")
    m = Model("gh_plug_in_J7", "J7")
    w, ln, dep, top = val(d, g, "housing_w"), val(d, g, "housing_len"), val(d, g, "housing_depth"), val(d, g, "mated_h")
    cy = -val(d, g, "body_centre_y")
    cx = (pads[0][0] + pads[-1][0]) / 2
    z0 = top - ln
    m.add("plastic_white", cbox(cx, cy, w, dep, z0, top))
    m.add("plastic_white", cbox(cx, cy + dep / 2 + 0.25, 2.6, 0.5, top - 3.4, top - 0.4))        # latch rib (illustrative)
    od, rise, bend, run = val(d, g, "wire_od"), val(d, g, "lead_rise"), val(d, g, "lead_bend_r"), val(d, g, "lead_run")
    z1 = top + rise
    for (x, _y), grp in zip(pads, ("wire_red", "wire_yellow", "wire_green", "wire_black")):
        ring = cq.Wire.makeCircle(od / 2, cq.Vector(x, cy, z1), cq.Vector(0, 0, 1))
        elbow = cq.Solid.revolve(ring, [], 90.0, cq.Vector(1, cy + bend, z1), cq.Vector(0, cy + bend, z1))
        m.add(grp, cyl_z(x, cy, top - 0.6, z1, od / 2), elbow,
              cq.Solid.makeCylinder(od / 2, run, cq.Vector(x, cy + bend, z1 + bend), cq.Vector(0, 1, 0)))
    lead_top = z1 + bend + od / 2
    limit = val(d, "height_limit", "limit_above_board")
    if lead_top > limit:
        raise ValueError(f"gh_plug: the drawn lead tops out at {lead_top:.2f} mm, above the {limit} mm limit")
    m.info = {"housing_top_z": top, "housing_bottom_z": round(z0, 4), "lead_top_z": round(lead_top, 4),
              "header_h": val(d, g, "header_h"),
              "wire_order": "pad 1 GPS_VCC red, 2 GPS_TXD yellow, 3 GPS_RXD green, 4 GND black (colours A)"}
    return m


def molex_j14(d: dict) -> Model:
    """Molex 22035025 (5267-02A) header: project model, because the library model file is not on disk."""
    g = "j14_molex"
    pads = row_pads("Connector_Molex", "Molex_SPOX_5267-02A_1x02_P2.50mm_Vertical", 2, val(d, g, "pitch"), "x")
    m = Model("molex_22035025_J14", "J14")
    bx, by, bh = val(d, g, "body_x"), val(d, g, "body_y"), val(d, g, "body_h")
    fx0, fy0 = val(d, g, "fab_x0"), val(d, g, "fab_y0")
    x0, x1 = fx0, fx0 + bx
    y0, y1 = -(fy0 + by), -fy0                      # footprint y fy0 .. fy0 + by  ->  3D y
    base, wt = val(d, g, "base_h"), val(d, g, "wall_t")
    m.add("plastic_white", box(x0, x1, y0, y1, 0.0, base))
    m.add("plastic_white", box(x0, x1, y1 - wt, y1, base, bh))                                   # friction-lock wall
    m.add("plastic_white", box(x0, x0 + wt, y0 + 1.6, y1, base, bh - 1.2), box(x1 - wt, x1, y0 + 1.6, y1, base, bh - 1.2))
    sq, pt, tail = val(d, g, "pin_square"), val(d, g, "pin_top_z"), val(d, g, "tail_below_board")
    for x, y in pads:
        m.add("pin_gold", cbox(x, y, sq, sq, -tail, pt))
    m.info = {"top_z": bh, "note": "outer size F (JLC record), inner shape illustrative (A)"}
    return m


def neo7m_board_parts(d: dict) -> Model:
    """NEO-7M receiver board in its own frame (origin = board centre, z = 0 underside, long side along x)."""
    g = "neo7m"
    m = Model("neo7m_board", None)
    L, W, t = val(d, g, "pcb_l"), val(d, g, "pcb_w"), val(d, g, "pcb_t")
    pcb = cbox(0, 0, L, W, 0, t)
    hd, hi = val(d, g, "hole_d"), val(d, g, "hole_inset")
    for sx in (-1, 1):
        for sy in (-1, 1):
            pcb = pcb.cut(cyl_z(sx * (L / 2 - hi), sy * (W / 2 - hi), -1, t + 1, hd / 2))
    m.add("pcb_blue", pcb)
    can = val(d, g, "ublox_can")
    m.add("metal", cbox(-1.5, 0.0, can[1], can[0], t, t + can[2]))
    m.add("silk_white", cbox(-1.5, 0.0, can[1] - 3.0, can[0] - 3.0, t + can[2], t + can[2] + 0.03))
    cell = val(d, g, "backup_cell")
    m.add("pin_gold", cyl_z(11.0, -5.0, t, t + cell[1], cell[0] / 2))
    m.add("ic_dark", cbox(11.0, 5.5, 5.0, 4.0, t, t + 1.5))                                      # EEPROM
    m.add("pin_gold", cbox(-13.5, 6.0, 2.6, 2.6, t, t + 1.2))                                    # u.FL / IPEX socket
    m.add("led_green", cbox(-13.5, -6.5, 1.6, 0.8, t, t + 0.6))
    n = val(d, g, "header_n")
    for k in range(n):
        m.add("pin_gold", cbox(L / 2 - 1.6, (k - (n - 1) / 2) * 2.54, 1.7, 1.7, t, t + 0.04))    # header pads (short edge)
    m.info = {"stack_h": val(d, g, "stack_h"), "frame": "own frame, not attached to the carrier"}
    return m


def neo7m_antenna_parts(d: dict) -> Model:
    """Ceramic patch antenna of the NEO-7M in its own frame (origin = centre, z = 0 underside)."""
    g = "neo7m"
    m = Model("neo7m_antenna", None)
    a, h, ch = val(d, g, "antenna_l"), val(d, g, "antenna_h"), val(d, g, "antenna_ceramic_h")
    base = h - ch
    m.add("metal", cbox(0, 0, a - 3.0, a - 3.0, 0, base - 0.8))                                  # shield can with the LNA
    m.add("pcb_green", cbox(0, 0, a, a, base - 0.8, base))
    m.add("ceramic", cbox(0, 0, a, a, base, h))
    m.add("metal", cbox(0, 0, a - 4.0, a - 4.0, h, h + 0.05))                                    # top electrode
    m.add("pin_gold", cyl_z(2.5, 2.5, h, h + 0.25, 0.9))                                         # feed pin
    m.info = {"antenna_h": h}
    return m


def neo7m_module(d: dict) -> Model:
    """Board and antenna side by side joined by the antenna lead (one file; the vehicle scene places them itself)."""
    if not val(d, "neo7m", "antenna_on_ufl_lead"):
        raise ValueError("neo7m.antenna_on_ufl_lead false: an on-board antenna is not modelled")
    m = Model("neo7m_module", None)
    L, a = val(d, "neo7m", "pcb_l"), val(d, "neo7m", "antenna_l")
    off = L / 2 + 8.0 + a / 2
    for grp, shapes in neo7m_board_parts(d).groups.items():
        m.add(grp, *shapes)
    for grp, shapes in neo7m_antenna_parts(d).groups.items():
        m.add(grp, *[s.translate(cq.Vector(-off, 0, 0)) for s in shapes])
    t = val(d, "neo7m", "pcb_t")
    m.add("coax", cq.Solid.makeCylinder(0.57, off - 13.5 - a / 2 + 1.0, cq.Vector(-13.5, 6.0, t + 0.9), cq.Vector(-1, 0, 0)))
    m.info = {"frame": "own frame (board centre), not attached to the carrier; used by render_d2_scene.py",
              "stack_h": val(d, "neo7m", "stack_h"), "antenna_h": val(d, "neo7m", "antenna_h")}
    return m


def height_limit_frame(d: dict) -> Model:
    """Marker for the side elevation: a thin frame on the board outline at the height limit (frame = board centre)."""
    g = "height_limit"
    m = Model("height_limit_frame", None)
    bx, by, z, bar = val(d, g, "board_x"), val(d, g, "board_y"), val(d, g, "limit_above_board"), val(d, g, "bar")
    for sy in (-1, 1):
        m.add("limit_red", cbox(0, sy * by / 2, bx + bar, bar, z, z + bar))
    for sx in (-1, 1):
        m.add("limit_red", cbox(sx * bx / 2, 0, bar, by + bar, z, z + bar))
    m.info = {"frame": "board centre, z up from the board top; the underside of the bars is the limit",
              "limit_above_board": z, "board_x": bx, "board_y": by}
    return m


# library footprint each attached model was written for (render_rev3.py refuses any other footprint)
FOOTPRINTS = {
    "gy521_on_J1": "Connector_PinHeader_2.54mm:PinHeader_1x08_P2.54mm_Vertical",
    "bmp280_in_J2": "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Horizontal",
    "openlog_in_J5": "Connector_PinSocket_2.54mm:PinSocket_1x06_P2.54mm_Horizontal",
    "gh_plug_in_J7": "Connector_JST:JST_GH_BM04B-GHS-TBT_1x04-1MP_P1.25mm_Vertical",
    "molex_22035025_J14": "Connector_Molex:Molex_SPOX_5267-02A_1x02_P2.50mm_Vertical",
}
BUILDERS = (gy521_on_j1, bmp280_in_j2, openlog_in_j5, gh_plug_in_j7, molex_j14, neo7m_module, height_limit_frame)


def build_models(dims: dict | None = None) -> dict[str, Model]:
    dims = dims or load_dims()
    return {m.name: m for m in (b(dims) for b in BUILDERS)}


# ------------------------------------------------------------------------------------------------- export
STYLE_ROOT = "MECHANICAL_DESIGN_GEOMETRIC_PRESENTATION_REPRESENTATION"
REF_RE = re.compile(r"#(\d+)")


def canonical_styles(text: str, name: str) -> str:
    """Put the colour records at the end of an OCCT STEP file into one canonical order.

    Measured here (OCCT 7.9, two runs of the same model): the geometry records are identical, but the writer emits
    the presentation-style blocks in the iteration order of a pointer-keyed map, so their order and numbers change
    from run to run.  The blocks are re-ordered by the (stable) geometry record they colour and renumbered; every
    statement of that tail is written on one line.  Anything unexpected in the tail is an error, not a guess."""
    lines = text.split("\n")
    start = next((i for i, ln in enumerate(lines) if re.match(rf"#\d+ = {STYLE_ROOT}\(", ln)), None)
    if start is None:
        raise RuntimeError(f"{name}: no colour records in the STEP file")
    end = next(i for i in range(start, len(lines)) if lines[i].strip() == "ENDSEC;")
    stmts: dict[int, str] = {}
    for raw in "\n".join(lines[start:end]).split(";\n"):
        s = "".join(part.strip() for part in raw.split("\n")).rstrip(";")
        if not s:
            continue
        mt = re.match(r"#(\d+) = (.+)$", s)
        if not mt:
            raise RuntimeError(f"{name}: unparsed STEP statement in the colour tail: {s[:60]}")
        stmts[int(mt.group(1))] = mt.group(2)
    first = min(stmts)
    if sorted(stmts) != list(range(first, first + len(stmts))):
        raise RuntimeError(f"{name}: colour tail record numbers are not contiguous")

    def local_refs(body: str) -> list[int]:
        return [int(r) for r in REF_RE.findall(body) if int(r) in stmts]

    def target(item: int) -> int:               # the geometry record a STYLED_ITEM colours (last reference)
        body = stmts[item]
        if not body.startswith("STYLED_ITEM("):
            raise RuntimeError(f"{name}: expected STYLED_ITEM, got {body[:40]}")
        return int(REF_RE.findall(body)[-1])

    roots: list[tuple[tuple[int, int], int, str]] = []
    for num, body in stmts.items():
        if not body.startswith(STYLE_ROOT + "("):
            continue
        mt = re.match(rf"{STYLE_ROOT}\('',\(([#\d,]+)\),#(\d+)\)$", body)
        if not mt:
            raise RuntimeError(f"{name}: unexpected {STYLE_ROOT} record: {body[:80]}")
        items = sorted((int(r) for r in REF_RE.findall(mt.group(1))), key=target)
        new_body = f"{STYLE_ROOT}('',({','.join(f'#{i}' for i in items)}),#{mt.group(2)})"
        roots.append(((target(items[0]), int(mt.group(2))), num, new_body))
        stmts[num] = new_body
    order: list[int] = []
    seen: set[int] = set()

    def visit(num: int) -> None:
        if num in seen:
            return
        seen.add(num)
        order.append(num)
        for r in local_refs(stmts[num]):
            visit(r)

    for _key, num, _body in sorted(roots):
        visit(num)
    if len(order) != len(stmts):
        raise RuntimeError(f"{name}: {len(stmts) - len(order)} colour tail records are not reachable from a style root")
    renum = {old: first + k for k, old in enumerate(order)}
    out = [f"#{renum[old]} = " + REF_RE.sub(lambda mo: f"#{renum.get(int(mo.group(1)), int(mo.group(1)))}", stmts[old]) + ";"
           for old in order]
    return "\n".join(lines[:start] + out + lines[end:])


def export_step(model: Model, path: Path, raw: bool = False, keep_stamp: bool = False) -> str:
    """Write the coloured STEP with LF line ends; unless ``raw``, canonical colour records and a fixed timestamp."""
    model.assembly().export(str(path), exportType="STEP")
    text = path.read_text(encoding="utf-8", errors="strict").replace("\r\n", "\n")
    if not raw:
        text = canonical_styles(text, path.name)
        if not keep_stamp:
            head, sep, tail = text.partition("ENDSEC;")
            head, nsub = STAMP_RE.subn(f"'{FIXED_STAMP}'", head)
            if nsub != 1:
                raise RuntimeError(f"{path.name}: expected exactly one timestamp in the STEP header, found {nsub}")
            text = head + sep + tail
    path.write_bytes(text.encode("utf-8"))
    return hashlib.sha256(path.read_bytes()).hexdigest()


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--out", type=Path, default=OUT_DIR)
    ap.add_argument("--raw", action="store_true", help="write the files exactly as OCCT emits them (demonstration only)")
    ap.add_argument("--keep-stamp", action="store_true",
                    help="canonical colour records but the real STEP timestamp (determinism demonstration)")
    args = ap.parse_args(argv)
    dims = load_dims()
    args.out.mkdir(parents=True, exist_ok=True)
    manifest: dict = {"schema": "vtol_carrier.module_models/1", "generated_by": "scripts/make_module_models.py",
                      "dimensions_file": "spec/module_dimensions.json",
                      "dimensions_sha256": hashlib.sha256(SPEC.read_bytes()).hexdigest(),
                      "cadquery": cq.__version__, "frame": dims["mounting"]["frame"],
                      "label": "pictures only; all module dimensions are A or F as tagged in the dimensions file",
                      "models": {}}
    for name, model in build_models(dims).items():
        path = args.out / f"{name}.step"
        sha = export_step(model, path, raw=args.raw, keep_stamp=args.keep_stamp)
        bb = model.bbox()
        manifest["models"][name] = {"file": path.name, "attach_ref": model.attach_ref,
                                    "footprint": FOOTPRINTS.get(name), "sha256": sha,
                                    "bbox": bb, "top_z": bb["max"][2], "info": model.info}
        print(f"{path.name}: {path.stat().st_size} B  z {bb['min'][2]:.2f}..{bb['max'][2]:.2f}  sha256 {sha[:16]}")
    if not (args.raw or args.keep_stamp):
        (args.out / "models_manifest.json").write_bytes(
            (json.dumps(manifest, indent=1, sort_keys=True, ensure_ascii=False) + "\n").encode("utf-8"))
    return 0


if __name__ == "__main__":
    sys.exit(main())
