"""Deterministic placement of the VTOL carrier (writes kicad/placement.json).

Invariants
----------
* Pure function of ``spec/carrier_spec.json`` + the KiCad footprint libraries:
  no randomness, no time, fixed iteration order -> the same placement every run.
* Courtyards come from the library footprints (``ai_eda.tools.kicad.library``);
  two courtyards never overlap (a placement that would is refused, it is not
  "fixed up" silently), every courtyard stays inside the 76 x 70 mm outline,
  nothing sits in the cable-tie band x -3..+3 mm (spec board.mounting.tie_band).
* Connectors are anchored on the edges the spec assigns (spec board.edges);
  the result is checked against those rules and the check is written next to
  the placement (``placement_checks``).
* Each MCU is placed first; its crystal, load caps, decoupling, AVCC filter,
  reset parts, LED and ISP header are then placed greedily as close as possible
  to the MCU pins they connect to (search on a 0.25 mm grid, rotations 0/90/180/270,
  ties broken by fixed order) - so every chip is grouped with its own support parts.

Board frame: spec (x, y) with +y = CAD +y maps to KiCad (100 + x, 100 - y).
"""

from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass

from ai_eda.ir import BoardSide, LibraryRef, Placement
from ai_eda.tools.kicad.geometry import pad_center, to_board
from ai_eda.tools.kicad.library import KicadLibrary

import common

CX, CY = 100.0, 100.0  # board centre in KiCad coordinates
BX0, BX1, BY0, BY1 = 62.0, 138.0, 65.0, 135.0  # outline (76 x 70)
TIE_X0, TIE_X1 = 97.0, 103.0  # tie band x -3..+3 mm
EDGE_MARGIN = 0.25  # courtyard of non-edge parts stays this far inside the outline
GAP = 0.0  # courtyards may touch but never overlap (KiCad courtyards_overlap rule)
STEP = 0.2  # search grid
CELL = 0.05  # occupancy bitmap resolution

BROAD_NETS = {"GND", "+5V", "+3V3", "GND_PWR", "SERVO_5V"}


def spec_xy(x: float, y: float) -> tuple[float, float]:
    """KiCad -> spec frame."""
    return round(x - CX, 3), round(CY - y, 3)


@dataclass
class Rect:
    x0: float
    y0: float
    x1: float
    y1: float

    def inflate(self, d: float) -> "Rect":
        return Rect(self.x0 - d, self.y0 - d, self.x1 + d, self.y1 + d)

    def overlaps(self, o: "Rect") -> bool:
        return self.x0 < o.x1 and o.x0 < self.x1 and self.y0 < o.y1 and o.y0 < self.y1

    def as_list(self) -> list[float]:
        return [round(self.x0, 3), round(self.y0, 3), round(self.x1, 3), round(self.y1, 3)]


class Occupancy:
    """Bitmap of occupied board area (rows are Python ints)."""

    def __init__(self) -> None:
        self.nx = int(round((BX1 - BX0) / CELL))
        self.ny = int(round((BY1 - BY0) / CELL))
        self.rows = [0] * self.ny

    def _cells(self, r: Rect, conservative: bool) -> tuple[int, int, int, int]:
        f = math.floor if conservative else math.ceil
        c = math.ceil if conservative else math.floor
        i0 = max(0, f((r.x0 - BX0) / CELL - 1e-9))
        i1 = min(self.nx, c((r.x1 - BX0) / CELL + 1e-9))
        j0 = max(0, f((r.y0 - BY0) / CELL - 1e-9))
        j1 = min(self.ny, c((r.y1 - BY0) / CELL + 1e-9))
        return i0, i1, j0, j1

    def free(self, r: Rect) -> bool:
        i0, i1, j0, j1 = self._cells(r, conservative=True)
        if i1 <= i0 or j1 <= j0:
            return True
        mask = ((1 << (i1 - i0)) - 1) << i0
        for j in range(j0, j1):
            if self.rows[j] & mask:
                return False
        return True

    def mark(self, r: Rect) -> None:
        i0, i1, j0, j1 = self._cells(r, conservative=True)
        if i1 <= i0 or j1 <= j0:
            return
        mask = ((1 << (i1 - i0)) - 1) << i0
        for j in range(j0, j1):
            self.rows[j] |= mask


# --------------------------------------------------------------------------- anchors
# (x, y, rotation) in KiCad coordinates; every value is a decision recorded with its reason.
PLM_CX, PLM_CY = 85.9, 75.6  # PLM100 module centre (module 22 x 19 mm, long axis along x, antenna end -x)

ANCHORS: dict[str, tuple[float, float, float, str]] = {
    # -x edge: vane servo headers, right-angle, pins toward -x (spec board.edges -x)
    # review MF-2: the mated plug in the top slot (pin 1 at y 72.15) would touch the intake spigot bore (r 57.17 > 57.0,
    # E) -> the fitted vanes take the four lower slots, the DNP release header J13 the top one
    "J13": (72.80, 72.15, 180, "edge -x: release servo (DNP) in the top slot (plug would touch the spigot bore)"),
    "J8": (72.80, 80.87, 180, "edge -x: vane servo 1 (spec board.edges)"),
    "J9": (72.80, 89.59, 180, "edge -x: vane servo 2"),
    "J10": (72.80, 98.31, 180, "edge -x: vane servo 3"),
    "J11": (72.80, 107.03, 180, "edge -x: vane servo 4"),
    "J18": (72.80, 122.00, 180, "edge -x, y < -12 mm: 6S power-sense lead"),
    # -y edge, x < -5: GPS lead JST-GH VERTICAL (review MF-6): plugged from above, nothing beyond the -y edge
    "J7": (67.00, 131.20, 0, "edge -y, x < -5 mm: GPS lead, vertical JST-GH (lead hole at x -23.8, y -28.3)"),
    # PLM100 sockets: datasheet fig 5 (p.12) top view, rows 11.60 mm apart, pin 1/24 2.82 mm from the far edge,
    # module rotated so the antenna end (pins 12/13, u.FL) points -x: J3 (pins 1..12) row at module y -5.8,
    # pins advance toward -x; J4 (pins 13..24) at module y +5.8, pins advance toward +x.
    "J3": (round(PLM_CX + 8.18, 3), round(PLM_CY - 5.8, 3), -90, "PLM100 pins 1-12 (datasheet fig 5), antenna end toward -x/+y corner"),
    "J4": (round(PLM_CX - 5.79, 3), round(PLM_CY + 5.8, 3), 90, "PLM100 pins 13-24 (datasheet fig 5)"),
    # +y edge, x > +5: arm switch lead, right-angle, pins toward +y (up)
    "J16": (121.40, 75.90, 90, "edge +y, x > +5 mm: arm switch lead (top cap hole at 90 deg)"),
    # +x edge, y > +14: logic battery (JST-PH), buzzer, ESC lead (pins toward +x)
    # review MF-4: vertical JST-PH in the +x/+y corner (lead leaves upward); MF-3: J12 in the upper slot so its mated
    # plug keeps >= 2 mm from the motor-lead joint zone, J17 below it
    "J14": (133.15, 67.50, 0, "edge +x, y > +14 mm: logic battery JST-PH vertical (top entry)"),
    "J12": (127.50, 73.85, 0, "edge +x, y > +14 mm: ESC lead (signal, BEC 5 V, GND), upper slot"),
    "J17": (133.15, 83.00, 0, "edge +x, y > +14 mm: buzzer lead, vertical JST-PH below the ESC lead (no plug toward the motor-lead joint zone)"),
    "NT1": (124.40, 78.93, 0, "single GND <-> GND_PWR tie right next to J12 pin 3 (spec grounding)"),
    # BMP280 right-angle socket in the bottom-right corner, opening toward +y (up): the module lies flat over a
    # part-free keep-out (drawn on Dwgs.User) between the socket and the GY-521
    "J2": (136.20, 133.00, -90, "BMP280 socket; module lies flat toward +y over a part-free keep-out"),
    # MCUs (TQFP-64, courtyard 17.3 mm)
    "U3": (86.00, 97.20, 180, "COM next to the PLM100 (rotated: UART1 PD2/PD3 face the module, PE0/PE1/PB1 face the band)"),
    "U4": (86.00, 117.30, 0, "GNS next to the GPS lead corner"),
    "U2": (111.85, 93.20, 0, "FCU (top-side support parts sit under the buck module body)"),
    "U5": (129.25, 96.50, 0, "5th ATmega128A (DNP) - only power, crystal, reset, ISP nets"),
    "U1": (111.85, 120.60, 0, "NAV"),
    # GY-521 header along x between FCU and NAV: IMU near the board centre (spec: near centre, outside tie band)
    "J1": (104.85, 108.80, 90, "GY-521 header between FCU and NAV, module body over the FCU (low parts only)"),
    # ISP footprints (2x3 2.54 mm, unpopulated) inside the cable-tie band, next to their MCUs
    "J20": (98.73, 84.67, 0, "FCU ISP footprint in the tie band (unpopulated, pogo / held header)"),
    "J21": (98.73, 93.47, 0, "COM ISP footprint in the tie band (unpopulated)"),
    "J23": (98.73, 102.27, 0, "SPARE ISP footprint in the tie band (unpopulated, DNP in the spec too)"),
    "J22": (98.73, 111.07, 0, "GNS ISP footprint in the tie band (unpopulated)"),
    "J19": (98.73, 119.87, 0, "NAV ISP footprint in the tie band (unpopulated)"),
    # servo rail bulk capacitor (5.8 mm tall: never under a module) near the servo headers / GPS corner
    "C46": (77.00, 131.15, 0, "SERVO_5V bulk 220 uF next to the -x servo headers (no room near J12)"),
    # power block in the top-right region, next to the logic battery input J14
    "J6": (104.80, 67.20, 90, "5 V buck module header (VIN, GND, VOUT, EN - A)"),
}

#: module bodies (A unless stated): name, centre-relative rectangle in KiCad coords, rule
def module_bodies(anch: dict[str, tuple[float, float, float, str]]) -> list[dict]:
    j1x, j1y = anch["J1"][0], anch["J1"][1]
    j2x, j2y = anch["J2"][0], anch["J2"][1]
    j6x, j6y = anch["J6"][0], anch["J6"][1]
    return [
        {"name": "PLM100 module 22 x 19 mm (datasheet p.7 table 1, fig 4)", "rect": [PLM_CX - 11, PLM_CY - 9.5, PLM_CX + 11, PLM_CY + 9.5],
         "rule": "parts <= 2.0 mm high only (module sits on 1.27 mm sockets, A)", "max_h": 2.0},
        # GY-521 21.2 x 16.4 (A), header row 1.3 mm inside the long edge; J1 pins run +x from (j1x, j1y); body toward -y (over FCU)
        {"name": "GY-521 module ~21.2 x 16.4 mm (A)", "rect": [j1x - 1.5, j1y - 15.1, j1x + 19.7, j1y + 1.3],
         "rule": "parts <= 2.0 mm high only (module on a 2.54 mm header spacer, A)", "max_h": 2.0},
        # BMP280 ~15.2 x 11.5 mm (A) beyond the socket opening; J2 rotated -90 so its opening faces -y (up)
        {"name": "BMP280 module ~15.2 x 11.5 mm (A) in front of the right-angle socket", "rect": [j2x - 6.35 - 7.6, j2y - 10.55 - 11.5, j2x - 6.35 + 7.6, j2y - 10.55],
         "rule": "parts <= 1.1 mm high only (module ~2.5 mm above the board in a right-angle socket, A)", "max_h": 1.1},
        {"name": "5 V buck module ~17 x 11 mm class (A)", "rect": [j6x - 1.5, j6y - 1.5, j6x + 9.7, j6y + 15.7],
         "rule": "parts <= 2.0 mm high only (module on a 2.54 mm header spacer, A)", "max_h": 2.0},
    ]


#: parts that must stay reachable (headers / shunts / jumpers) - never under a module body
ACCESS = {"J15", "J19", "J20", "J21", "J22", "J23", "J24", "J25", "JP1", "J5"}

#: body height above the board per footprint (mm, A: typical package heights, not measured)
FP_HEIGHT = {
    "R_0603_1608Metric": 0.55, "R_0402_1005Metric": 0.4, "C_0402_1005Metric": 0.55, "C_0603_1608Metric": 0.9, "LED_0603_1608Metric": 0.8, "C_0805_2012Metric": 1.35,
    "L_0805_2012Metric": 1.0, "C_1206_3216Metric": 1.8, "SOT-23": 1.1, "SOT-223-3_TabPin2": 1.8,
    "Crystal_SMD_3225-4Pin_3.2x2.5mm": 0.9, "D_SOD-123": 1.35, "TQFP-64_14x14mm_P0.8mm": 1.2,
    "NetTie-2_SMD_Pad0.5mm": 0.05, "SolderJumper-3_P1.3mm_Bridged12_RoundedPad1.0x1.5mm": 0.05,
    "CP_Elec_6.3x5.8": 5.8, "CP_EIA-3216-18_Kemet-A": 1.8,
}

# --------------------------------------------------------------------------- MCU support clusters (review F1 / F2)
# MCU-local coordinates (MCU at rotation 0, KiCad y down): TQFP-64 pads 17-32 at y +7.662, 49-64 at y -7.662,
# pitch 0.8 mm, pad ends at +-8.4, courtyard +-8.65 (Package_QFP:TQFP-64_14x14mm_P0.8mm on disk). Each part sits with its
# courtyard touching the MCU courtyard, so its pads are 0.4 mm from the pin ends:
#   dec21: 0402 100 nF, pad 1 (+5V) on the pin-21 (VCC) axis, pad 2 (GND) on the pin-22 (GND) axis
#   dec52: 0402 100 nF, pad 1 on pin 52 (VCC), pad 2 on pin 53 (GND)
#   avcc : 0402 100 nF, pad 1 on pin 64 (AVCC), pad 2 on pin 63 (GND)
#   xtal : 3225 crystal rotated 90 deg: pad 3 (XTAL2) under pin 23, pad 1 (XTAL1) diagonal; pin 24 runs down the
#          0.5 mm channel between the pad columns (0.15 mm each side)
#   cx1 / cx2: 0402 22 pF load caps beside crystal pads 1 / 3
CLUSTER: dict[str, tuple[float, float, float]] = {
    "dec21": (-2.32, 9.11, 0), "dec52": (3.12, -9.11, 180), "avcc": (-5.52, -9.11, 0),
    "xtal": (-0.40, 11.67, 90), "cx2": (-3.06, 10.57, 180), "cx1": (2.26, 12.77, 0),
}
#: pre-routed copper (net key, width mm, MCU-local polyline); "XTAL1" / "XTAL2" / "AVCC" get the block prefix
CLUSTER_ROUTES: list[tuple[str, float, list[tuple[float, float]]]] = [
    ("+5V", 0.3, [(-2.8, 7.662), (-2.8, 9.11)]),            # pin 21 -> dec21 pad 1
    ("GND", 0.25, [(-2.0, 7.662), (-2.0, 9.11)]),           # pin 22 -> dec21 pad 2
    ("+5V", 0.3, [(3.6, -7.662), (3.6, -9.11)]),            # pin 52 -> dec52 pad 1
    ("GND", 0.25, [(2.8, -7.662), (2.8, -9.11)]),           # pin 53 -> dec52 pad 2
    ("AVCC", 0.3, [(-6.0, -7.662), (-6.0, -9.11)]),         # pin 64 -> avcc pad 1
    ("GND", 0.25, [(-5.2, -7.662), (-5.2, -9.11)]),         # pin 63 -> avcc pad 2
    ("XTAL2", 0.2, [(-1.2, 7.662), (-1.2, 10.57)]),         # pin 23 -> crystal pad 3
    ("XTAL2", 0.2, [(-2.58, 10.57), (-1.25, 10.57)]),       # cx2 pad 1 -> crystal pad 3
    ("XTAL1", 0.2, [(-0.4, 7.662), (-0.4, 12.77), (1.78, 12.77)]),  # pin 24 -> crystal pad 1 -> cx1 pad 1
]
#: blocks that get the cluster (the DNP spare chip keeps the greedy placement: nothing is fitted there)
CLUSTER_BLOCKS = {"NAV": "U1", "FCU": "U2", "COM": "U3", "GNS": "U4"}


def cluster_members(parts: dict, pin_net: dict) -> dict[str, dict[str, str]]:
    """block -> {cluster key: refdes}, found from the spec roles / nets (never from refdes numbers)."""
    out: dict[str, dict[str, str]] = {}
    for block in CLUSTER_BLOCKS:
        m: dict[str, str] = {}
        for ref, p in sorted(parts.items()):
            if p["block"] != block:
                continue
            role = p["role"]
            if role == f"{block} VCC pin 21 decoupling":
                m["dec21"] = ref
            elif role == f"{block} VCC pin 52 decoupling":
                m["dec52"] = ref
            elif role == f"{block} AVCC decoupling":
                m["avcc"] = ref
            elif role == f"{block} crystal":
                m["xtal"] = ref
            elif role == f"{block} crystal load cap":
                m["cx1" if pin_net.get((ref, "1")) == f"{block}_XTAL1" else "cx2"] = ref
        if set(m) != set(CLUSTER):
            raise ValueError(f"{block}: cluster parts not found by role: {m}")
        out[block] = m
    return out


def cluster_routes_board(placements: dict) -> list[dict]:
    """Pre-routed segments in board coordinates for every clustered MCU (placements: ref -> {x, y, rot})."""
    segs = []
    for block, u in CLUSTER_BLOCKS.items():
        pu = placements[u]
        pl = Placement(component_ref=u, x_mm=pu["x"], y_mm=pu["y"], rotation_deg=float(pu["rot"]), side=BoardSide.TOP)
        for key, w, pts in CLUSTER_ROUTES:
            net = key if key in ("+5V", "GND") else f"{block}_{key}"
            b = [to_board(pl, x, y) for x, y in pts]
            for a, c in zip(b, b[1:]):
                segs.append({"net": net, "width": w, "start": [round(a[0], 4), round(a[1], 4)],
                             "end": [round(c[0], 4), round(c[1], 4)], "block": block})
    return segs

#: where to aim parts whose nets are all broad rails (ref -> (ref, pad))
TARGET_HINT = {
    "C41": ("J6", "1"), "C42": ("J6", "3"), "C43": ("U6", "3"), "C44": ("U6", "2"), "C46": ("J12", "2"),
    "C51": ("J3", "1"), "C52": ("J3", "1"), "Q10": ("J14", "1"), "C54": ("J7", "1"), "D6": ("J17", "2"),
    "J15": ("J14", "1"), "U6": ("J6", "3"),
}
#: parts kept out of the +x keep-out band (|y| < 14 mm within 12 mm of the +x edge): crystals, I2C, radio
EMI_SENSITIVE_PREFIX = ("Y",)
EMI_SENSITIVE = {"Q1", "Q2", "R38", "R39", "R40", "R41", "Q3", "R42", "R43"}


class Placer:
    def __init__(self, spec: dict, library: KicadLibrary) -> None:
        self.spec = spec
        self.lib = library
        self.parts = {p["refdes"]: p for p in spec["parts"]}
        self.ir = common.build_ir(spec, library)
        self.fp = {}
        for p in spec["parts"]:
            lib, name = common.split_lib_id(common.footprint_of(p))
            self.fp[p["refdes"]] = library.load_footprint(LibraryRef(library=lib, name=name))
        self.pin_net = {}
        self.net_pins: dict[str, list[tuple[str, str]]] = {}
        for n in self.ir.nets:
            for pr in n.pins:
                self.pin_net[(pr.component_ref, pr.pin_number)] = n.name
                self.net_pins.setdefault(n.name, []).append((pr.component_ref, pr.pin_number))
        self.placed: dict[str, dict] = {}
        self.rects: dict[str, Rect] = {}
        self.occ = Occupancy()
        self.bodies = module_bodies(ANCHORS)
        self.problems: list[str] = []

    # --- geometry ------------------------------------------------------------------
    def courtyard(self, ref: str, x: float, y: float, rot: float) -> Rect:
        key = (ref, rot)
        rel = self._cy_cache.get(key) if hasattr(self, "_cy_cache") else None
        if rel is None:
            if not hasattr(self, "_cy_cache"):
                self._cy_cache = {}
            r0 = self._courtyard_exact(ref, 0.0, 0.0, rot)
            rel = self._cy_cache[key] = r0
        return Rect(round(rel.x0 + x, 4), round(rel.y0 + y, 4), round(rel.x1 + x, 4), round(rel.y1 + y, 4))

    def _courtyard_exact(self, ref: str, x: float, y: float, rot: float) -> Rect:
        fp = self.fp[ref]
        pl = Placement(component_ref=ref, x_mm=x, y_mm=y, rotation_deg=rot, side=BoardSide.TOP)
        if fp.courtyard is not None:
            bb = fp.courtyard
            pts = [(bb.x1, bb.y1), (bb.x1, bb.y2), (bb.x2, bb.y1), (bb.x2, bb.y2)]
        else:  # e.g. NetTie: pads + 0.15 mm
            pts = []
            for p in fp.pads:
                pts += [(p.x - p.size_w / 2 - 0.15, p.y - p.size_h / 2 - 0.15), (p.x + p.size_w / 2 + 0.15, p.y + p.size_h / 2 + 0.15)]
        b = [to_board(pl, px, py) for px, py in pts]
        return Rect(min(q[0] for q in b), min(q[1] for q in b), max(q[0] for q in b), max(q[1] for q in b))

    def pad_xy(self, ref: str, number: str) -> tuple[float, float] | None:
        if ref not in self.placed:
            return None
        pl = self.placed[ref]
        fp = self.fp[ref]
        pad = fp.pad(number)
        if pad is None:
            return None
        return pad_center(Placement(component_ref=ref, x_mm=pl["x"], y_mm=pl["y"], rotation_deg=pl["rot"], side=BoardSide.TOP), pad)

    # --- placing ---------------------------------------------------------------------
    def put(self, ref: str, x: float, y: float, rot: float, how: str, why: str) -> None:
        r = self.courtyard(ref, x, y, rot)
        for other, orc in self.rects.items():
            if r.overlaps(orc):
                self.problems.append(f"courtyard overlap {ref} / {other}")
        self.placed[ref] = {"x": round(x, 3), "y": round(y, 3), "rot": rot, "how": how, "why": why}
        self.rects[ref] = r
        self.occ.mark(r)

    def allowed(self, ref: str, r: Rect) -> bool:
        if r.x0 < BX0 + EDGE_MARGIN or r.x1 > BX1 - EDGE_MARGIN or r.y0 < BY0 + EDGE_MARGIN or r.y1 > BY1 - EDGE_MARGIN:
            return False
        if r.x0 < TIE_X1 and r.x1 > TIE_X0 and ref not in common.ISP_IN_BAND:
            return False
        h = self.height(ref) if common.populated(self.parts[ref]) else 0.0  # DNP parts are not fitted
        for b in self.bodies:
            br = Rect(*b["rect"])
            if r.overlaps(br) and (ref in ACCESS or h > b["max_h"]):
                return False
        if ref.startswith(EMI_SENSITIVE_PREFIX) or ref in EMI_SENSITIVE:
            band = Rect(BX1 - 12.0, CY - 14.0, BX1, CY + 14.0)
            if r.overlaps(band):
                return False
        return self.occ.free(r.inflate(GAP))

    def height(self, ref: str) -> float:
        return FP_HEIGHT.get(self.fp[ref].name, 99.0)

    def target(self, ref: str) -> tuple[float, float] | None:
        """Mean position of the placed pads this part connects to (signal nets first, broad nets by role)."""
        pts = []
        role = self.parts[ref]["role"]
        m = None
        import re

        m = re.search(r"pin (\d+)", role)
        comp = self.ir.component(ref)
        if ref in TARGET_HINT:
            xy = self.pad_xy(*TARGET_HINT[ref])
            if xy is not None:
                return xy
        for pin in comp.pins:
            net = self.pin_net.get((ref, pin.number))
            if net is None or net in BROAD_NETS:
                continue
            for oref, onum in self.net_pins[net]:
                if oref == ref:
                    continue
                xy = self.pad_xy(oref, onum)
                if xy is not None:
                    pts.append(xy)
        if m and not pts:
            # decoupling cap "VCC pin 21": the MCU of the same block
            block = self.parts[ref]["block"]
            mcu = {"NAV": "U1", "FCU": "U2", "COM": "U3", "GNS": "U4", "SPARE": "U5"}.get(block)
            if mcu:
                xy = self.pad_xy(mcu, m.group(1))
                if xy:
                    pts.append(xy)
        if not pts:
            for pin in comp.pins:
                net = self.pin_net.get((ref, pin.number))
                if net is None:
                    continue
                for oref, onum in self.net_pins[net]:
                    if oref != ref:
                        xy = self.pad_xy(oref, onum)
                        if xy is not None:
                            pts.append(xy)
        if not pts:
            return None
        return (sum(p[0] for p in pts) / len(pts), sum(p[1] for p in pts) / len(pts))

    def auto(self, ref: str, max_r: float = 40.0) -> bool:
        t = self.target(ref)
        if t is None:
            self.problems.append(f"{ref}: no placed neighbour to aim at")
            return False
        tx, ty = round(t[0] / STEP) * STEP, round(t[1] / STEP) * STEP
        best = None
        n = int(max_r / STEP)
        # rings of increasing Chebyshev radius; within a ring pick the smallest Euclidean distance
        for k in range(0, n + 1):
            ring = []
            for i in range(-k, k + 1):
                for j in (-k, k) if abs(i) != k else range(-k, k + 1):
                    ring.append((i, j))
            ring.sort(key=lambda ij: (ij[0] ** 2 + ij[1] ** 2, ij[0], ij[1]))
            for i, j in ring:
                x, y = tx + i * STEP, ty + j * STEP
                d2 = (i * i + j * j)
                if best is not None and d2 > best[0]:
                    continue
                for rot in (0, 90, 180, 270):
                    r = self.courtyard(ref, x, y, rot)
                    if self.allowed(ref, r):
                        cand = (d2, round(x, 3), round(y, 3), rot)
                        if best is None or cand < best:
                            best = cand
                        break
            if best is not None and (k + 1) ** 2 > best[0]:
                break
        if best is None:
            self.problems.append(f"{ref}: no free spot within {max_r} mm of {t}")
            return False
        _, x, y, rot = best
        self.put(ref, x, y, rot, "auto", f"greedy: nearest free spot to its connected pads at {tuple(round(v, 2) for v in t)}")
        return True

    def run(self) -> None:
        for ref, (x, y, rot, why) in ANCHORS.items():
            self.put(ref, x, y, rot, "anchor", why)
        # MCU support clusters (fixed geometry relative to the MCU, pre-routed in build_pcb.py)
        self.clusters = cluster_members(self.parts, self.pin_net)
        for block, m in self.clusters.items():
            u = CLUSTER_BLOCKS[block]
            pu = self.placed[u]
            pl = Placement(component_ref=u, x_mm=pu["x"], y_mm=pu["y"], rotation_deg=float(pu["rot"]), side=BoardSide.TOP)
            for key, (lx, ly, lrot) in CLUSTER.items():
                x, y = to_board(pl, lx, ly)
                rot = (pu["rot"] + lrot) % 360
                r = self.courtyard(m[key], round(x, 4), round(y, 4), rot)
                if r.x0 < BX0 + EDGE_MARGIN or r.x1 > BX1 - EDGE_MARGIN or r.y0 < BY0 + EDGE_MARGIN or r.y1 > BY1 - EDGE_MARGIN:
                    self.problems.append(f"cluster part {m[key]} outside the outline")
                self.put(m[key], round(x, 4), round(y, 4), rot, "cluster",
                         f"{block} support cluster '{key}' at fixed MCU-local ({lx}, {ly}) rot {lrot} (pre-routed)")
        # per-MCU support parts in a fixed order: crystal, load caps, decoupling, AVCC, AREF, reset, LED, ISP
        # each MCU's crystal / decoupling / AVCC / AREF / reset first (closest to their pins), then the
        # reachable headers / shunts, then LEDs, shifters, links, IO and power parts
        order = []
        critical = ["crystal", "load cap", "decoupling", "local bulk", "AVCC", "AREF", "reset"]
        for block in ("NAV", "FCU", "COM", "GNS", "SPARE"):
            for key in critical:
                for r in sorted(self.parts, key=common_key):
                    p = self.parts[r]
                    if p["block"] == block and r not in self.placed and r not in order and r not in ACCESS                             and key.lower() in p["role"].lower():
                        order.append(r)
        order += sorted([r for r in ACCESS if r not in self.placed], key=common_key)
        for block in ("NAV", "FCU", "COM", "GNS", "SPARE"):
            refs = [r for r, p in self.parts.items() if p["block"] == block and r not in self.placed and r not in order]
            def rank(r: str) -> tuple:
                role = self.parts[r]["role"]
                keys = ["crystal", "load cap", "decoupling", "local bulk", "AVCC", "AREF", "reset", "LED", "ISP"]
                for i, k in enumerate(keys):
                    if k.lower() in role.lower():
                        return (i, common_key(r))
                return (len(keys), common_key(r))
            order += sorted(refs, key=rank)
        rest_blocks = ("PWR", "SHIFT", "LINK", "IO", "LOG")
        for block in rest_blocks:
            order += sorted([r for r, p in self.parts.items() if p["block"] == block and r not in self.placed and r not in order], key=common_key)
        for ref in order:
            self.auto(ref)


def common_key(ref: str) -> tuple:
    import re

    m = re.match(r"([A-Z]+)(\d+)", ref)
    return (m.group(1), int(m.group(2))) if m else (ref, 0)


def checks(pl: Placer) -> dict:
    out: dict = {"problems": list(pl.problems)}
    # courtyard overlaps (independent re-check)
    refs = sorted(pl.rects, key=common_key)
    ov = []
    for i, a in enumerate(refs):
        for b in refs[i + 1:]:
            if pl.rects[a].overlaps(pl.rects[b]):
                ov.append([a, b])
    out["courtyard_overlaps"] = ov
    out["outside_outline"] = [r for r in refs if pl.rects[r].x0 < BX0 - 0.05 or pl.rects[r].x1 > BX1 + 0.05
                              or pl.rects[r].y0 < BY0 - 0.05 or pl.rects[r].y1 > BY1 + 0.05]
    out["in_tie_band"] = [r for r in refs if pl.rects[r].x0 < TIE_X1 and pl.rects[r].x1 > TIE_X0 and r not in common.ISP_IN_BAND]
    out["unpopulated_isp_footprints_in_tie_band"] = list(common.ISP_IN_BAND)
    edges = {
        "-x": (["J8", "J9", "J10", "J11", "J13", "J18"], lambda r: r.x0 <= BX0 + 0.5),
        "-x,y<-12": (["J18"], lambda r: spec_xy(0, r.y0)[1] < -12),
        "+x,y>+14": (["J12", "J14", "J17"], lambda r: r.x1 >= BX1 - 0.5 and spec_xy(0, r.y1)[1] > 13.0),
        "+y,x>+5": (["J16"], lambda r: r.y0 <= BY0 + 0.7 and spec_xy(r.x0, 0)[0] > 3.0),
        "-y,x<-5": (["J7"], lambda r: r.y1 >= BY1 - 0.5 and spec_xy(r.x1, 0)[0] < -5),
    }
    out["edge_rules"] = {k: {ref: bool(fn(pl.rects[ref])) for ref in refs_} for k, (refs_, fn) in edges.items()}
    out["edge_notes"] = {
        "J12": f"courtyard bottom at spec y {spec_xy(0, pl.rects['J12'].y1)[1]} mm (motor-lead band is |y| < 14)",
        "J1": f"GY-521 header centre at spec ({spec_xy(pl.placed['J1']['x'] + 8.89, pl.placed['J1']['y'])})",
    }
    bodies = []
    for b in pl.bodies:
        br = Rect(*b["rect"])
        under = sorted([r for r in refs if pl.rects[r].overlaps(br)], key=common_key)
        bodies.append({**b, "rect": [round(v, 3) for v in b["rect"]], "parts_under": under,
                       "inside_outline": br.x0 >= BX0 and br.x1 <= BX1 and br.y0 >= BY0 and br.y1 <= BY1})
    out["module_bodies"] = bodies
    area = sum((r.x1 - r.x0) * (r.y1 - r.y0) for r in pl.rects.values())
    out["courtyard_area_mm2"] = round(area, 1)
    out["board_area_minus_tie_band_mm2"] = round((BX1 - BX0) * (BY1 - BY0) - (TIE_X1 - TIE_X0) * (BY1 - BY0), 1)
    return out


def main() -> int:
    spec = common.load_spec()
    pl = Placer(spec, KicadLibrary())
    pl.run()
    chk = checks(pl)
    data = {
        "frame": "KiCad coordinates (mm, y down); board outline x 62..138, y 65..135; spec (x, y) = (X - 100, 100 - Y)",
        "generator": "scripts/place.py (deterministic greedy placement around anchored parts)",
        "spec_sha256": common.sha256_file(common.SPEC_PATH),
        "placements": {r: pl.placed[r] for r in sorted(pl.placed, key=common_key)},
        "courtyards": {r: pl.rects[r].as_list() for r in sorted(pl.rects, key=common_key)},
        "clusters": {b: {"mcu": CLUSTER_BLOCKS[b], **m} for b, m in sorted(pl.clusters.items())},
        "cluster_routes": cluster_routes_board({r: pl.placed[r] for r in pl.placed}),
        "placement_checks": chk,
    }
    path = common.KICAD_DIR / "placement.json"
    path.write_bytes((json.dumps(data, indent=1) + "\n").encode("utf-8"))
    print(json.dumps({k: v for k, v in chk.items() if k != "module_bodies"}, indent=1))
    for b in chk["module_bodies"]:
        print(b["name"], b["parts_under"], b["inside_outline"])
    unplaced = sorted(set(pl.parts) - set(pl.placed), key=common_key)
    print("placed", len(pl.placed), "unplaced", unplaced)
    return 0 if not unplaced and not chk["courtyard_overlaps"] else 1


if __name__ == "__main__":
    sys.exit(main())
