"""CanSat EDF Rocket - Fusion 360 API build script (docs/SPEC.md sections 1-6).

Invariant enforced by this module: it contains geometry *construction* logic
only.  Every dimension, station, angle and mass comes from ``cad/layout.py``
(``Layout``) or ``params.json`` - nothing dimensional is typed here.  Fusion API
lengths are centimetres: every millimetre value passes through ``cm()`` before it
reaches the API; angles are radians.

Run inside Fusion 360 (Utilities > Scripts and Add-Ins).  ``run(context)``:
  1. locates the project root (project_root.txt next to this file, else ../../..),
  2. imports ``layout`` from ``<root>/cad`` and applies ``<root>/cad/fusion360/preset.txt``,
  3. creates a new design, mirrors params.json leaves as user parameters,
  4. builds every component of SPEC sections 3-6 (each wrapped in try/except),
  5. measures mass properties per the SPEC section 6 recipe,
  6. exports STL/STEP/f3d/preview and writes ``cad/exports/fusion360_report.json``
     plus ``cad/exports/fusion/run.log``.

Environment overrides (used by tools/fusion_mock_run.py): ``CANSAT_PROJECT_ROOT``,
``CANSAT_EXPORT_ROOT`` (replaces ``<root>/cad/exports``), ``CANSAT_FUSION_DIRECT=1``.

v0.9 CAD-2 (mission.config): ``<root>/cad/fusion360/config.txt`` (or ``CANSAT_CONFIG`` / ``CANSAT_POWER``) selects the
hardware configuration - 'rocket' (default) or 'vtol' [battery | tether | hybrid].  The VTOL build (``component_specs_vtol``)
has the top cap (``build_top_cap``), the carrier PCB, the GPS on the cap, the windowed avionics module, the fin holes and
the lightened tray, and writes ``fusion_vtol*/`` and ``fusion360_report_vtol*.json``; the rocket build is unchanged.

0.9.6 livery (user decisions 2026-11-04, ``Layout.has_livery``; mirrors rocket_geometry.livery_inlay_solids): the avionics
module ends with the 0.42 mm logo pocket (``build_livery`` op CUT: per copy azimuth and logo island one ``CB.inlay`` - an
annular sector NEW body r [body_r - depth, body_r + OVERLAP] INTERSECTED with the island's pre-warped (v, z) outline
sketched on the tprism plane and extruded along the copy's radial direction, then Combine/Cut from the main body); the new
printed component ``avionics_logo`` is the orange inlay (the same construction, op NEW, r up to body_r: one body per
island).  Display colours (``Ctx.apply_appearance``, colour only): printed parts white (printer.color_hex), the inlay
orange (livery.colours.logo.hex), the launch pad black (printer.color_exceptions) - copies of a library plastic
appearance; a refused appearance is logged, never a failure.  replay_x3 models CB.inlay.

v0.9 fix round (script 0.9.2, mirrors cad/cadquery/rocket_geometry.py): intake - lip-pocket slit (``lip_slit``), motor-lead
tie anchors + their tunnels (``tie_anchor_k`` / ``tie_tunnel_k``), groove skirt cut, bulkhead hole of its own width; vane -
D-flat on the +v side; coupler - round bore + flat restored to ``dd_u_flat_end``, horn socket slot + hub relief, set-screw tap;
servo_horn = double-arm bar + hub + screw head; hinge_bushing_k placeholders (bore = bushing OD); top cap - load-washer
recess; VTOL tether / hybrid - tether kit boxes, eye bolt, load washer, jam nut placeholders.  Fins have no lightening hole
in the VTOL configuration any more (Layout.has_fin_hole False); the tray plane / windows / gland move with params.

v0.5 (SPEC 3.7, 5.1, 6): with ``edf.mount`` = ``tabs`` (``Layout.tabs``) the propulsion module gets the
cylindrical outer skin, thin liner (tube + top ring), one screw lug per ``boss_z_ranges`` entry and tab,
keyways, tangential tab-screw holes (clearance, counterbore to ``tab_screw_t_cbore_end``, tap) and the
two-box wire slot; the intake module gets the wire bulge, channel, shelf notch and the bulkhead wire
hole; ``edf_unit`` gets the bell ring, motor and tab plates.  Every v0.5 feature is gated on
``Layout.tabs``: ``lip_clamp`` presets build the v0.4 geometry, except for the two v0.5d fixes below, the v0.6 COM
mount and the 0.6.1 intake band.

v0.5d (SPEC 3.2, 3.7, 5.1, 5.2): the tabs sit at ``Layout.tab_thetas`` and the leads leave at
``Layout.wire_theta`` (every feature follows Layout; no angle is typed here); the intake wire channel gets a
45 deg **floor wedge** (triangle ``wire_ch_floor_rz`` over the channel width at ``wire_theta``, unioned after
the channel / notch cuts).  For EVERY preset: a **tray stop** block in each avionics rail slot (between the
two ribs of the pair, Z [``z_tray_stop_bottom``, ``z_tray_bottom``]) and **blind propulsion spigot joint
holes** (Ø ``joint_hole_spigot_d`` from ``spigot_od``/2 - ``prop_joint_hole_depth`` out past the spigot OD).

Every v0.5 / v0.5d cut / join is a *required* feature (``CB.required``): a failure is logged as ERROR, the
rest of the component is still built for inspection, and the component then raises so it is listed in the
report's ``failed_components`` (never a soft failure).  Cylinder cutters along an arbitrary axis
(``cut_between``) are identified from the feature that created them (``feature.bodies.item(0)``), their
placement is verified, the Combine/Cut must remove material from the target (its volume must drop), and on
any failure exactly that body is deleted; ``combine_all`` refuses to join a cutter body into a part.  A
component whose mass properties cannot be measured is listed in ``failed_components`` as well.

v0.6 (SPEC 5.3 / 6, two ATmega128A): with ``Layout.has_com_board`` the payload module gets four COM-board bosses
under its bulkhead (``Layout.com_boss_xy``, Z [``z_com_boss_tip``, ``z_com_boss_top``], joined OVERLAP into the
bulkhead) and a blind tap hole in each (to ``z_com_hole_end``), all ``CB.required``.  The holes are small (Ø2.5 x 4.5,
22 mm^3 - below Fusion's volume accuracy), so ``CB.cut_verified`` judges them by ``BRepBody.pointContainment``: a point
on the hole axis must leave the body and a point in the floor above the hole end must stay material (the volume drop
is only the fallback when the API cannot answer); each boss is confirmed by a point in its wall
(``CB.verify_points``).  A new placeholder component ``com`` (the COM-board box, mass ``electronics.com.mass_g``)
hangs from the boss tips.

v0.6 servo leads (SPEC 3.8 / 5.1 / 6, ``Layout.servo_leads``: tab-mount preset only): the propulsion module gets the
four pod lead windows, four entry slots (shoulder ring / cone skin), the deck notches (deck box + spigot box) and the
spigot-ring rebate (``sector_z`` cut); the intake module gets the servo bulge (joined before its cuts), the servo channel
and shelf notch, the 45 deg servo floor wedge (triangle ``sl_ch_floor_rz``, joined after) and the bulkhead servo hole at
``sl_gather_theta``; the report gets the lumped part ``servo_leads`` at ``Layout.sl_lead_com``.  Every one is
``CB.required``; every cut goes through ``CB.cut_verified`` (a point in the removed material leaves the body, a point
in the wall it must leave stays) and the wedge through ``CB.verify_points``.

0.6.1 (SPEC 5.1, both presets; user 2026-10-15: the 1.2 mm intake struts broke when the print supports came off): the
intake's windowed band (``z_shelf_top`` .. ``z_intake_bulkhead_bottom``) is ``window_wall_t`` thick - the revolved
profile's inner radius there is ``Layout.r_band_in`` (outer surface = body OD) - and the windows are radial sectors
(``Layout.window_sectors``, ``CB.sector_z``) instead of two orthogonal through-slab cuts, so every strut keeps strut_w at
body_r with radial side faces.  The band thickness (``CB.verify_points``) and every window cut (``CB.cut_verified``: the
band at the window centre leaves the body; the struts beside it at their inner and outer faces and the margin rings
stay) are ``CB.required`` for every preset (report ``derived.extra.intake_windows``).

0.6.2 (SPEC 5.1 / 6, both presets; user 2026-10-15): (1) the LOWER end of every window (its ceiling when the intake prints
flipped) is a truncated gable (``intake_module.window_bottom_end``: flat over ``bridge_w`` at z_window_bottom, planar faces
rising at ``slope_deg`` to the struts in the window's own frame).  ``gable_window_cut`` builds each window tool as the
radial sector (``CB.sector_z``, NEW body) INTERSECT the hexagon ``Layout.window_gable_profile_yz`` sketched on
``vplane(window theta + 90)`` (the vertical plane through the axis perpendicular to the window centre: 135/225/315/45 deg
on BUDGET - the angle family the webs already build on - and 0/90 base planes on KAIST), extruded symmetrically with
``participantBodies`` = the tool (``FeatureOperations.IntersectFeatureOperation``), then Combine/Cuts it from the main body
(tool registered in ``cutters``, deleted on any failure).  Each window stays ``CB.required`` / ``CB.cut_verified``: points
just above the bridge and just above both gable faces must leave the body, points inside both gussets and just below the
bridge must stay; unanswered points are listed in ``intake_windows.containment_unanswered`` as in 0.6.1.  (2) A new
placeholder component ``radio_log`` (Layout.radio_log_x/y/z, mass ``electronics.radio_log.mass_g``) hangs from the
payload-bulkhead underside beside the COM board.

0.6.3 (SPEC 6, both presets; user 2026-10-15: three ATmega128A): only the placeholders change - the component ``fc`` is
``nav`` (Layout.nav_x/y/z, mass ``electronics.nav.mass_g``; the tray's board holes stay on its centre, Layout.nav_center_z
= the old fc_center_z) and the new component ``fcu`` (Layout.fcu_x/y/z, mass ``electronics.fcu.mass_g``) lies flat on the
intake bulkhead's top face on foam tape.  No printed part's construction changes.

0.8.0 (v0.8 GPS pod, payload_module.gps_pod / avionics_module.gps_entry, both presets): ``build_gps_pod`` applies the GPS
pod to the finished payload module in the CadQuery twin's order (rocket_geometry.gps_pod_features): outer box (JOIN),
cavity ((u, z) prism above the 45 deg floor line, CUT), the two lid-screw bosses (JOIN), everything below the floor's
underside (prism, CUT), the two corner chamfers (``CB.poly_pod``, a new helper: plan polygon in the pod frame on a
horizontal sketch, the box_pod construction), the tube-wall sector back (``CB.sector_z``, JOIN), the bore over the pod
(revolve_z, CUT), the blind lid-screw holes, the raceway + connector pocket boxes (JOIN) and their voids (CUT) - every one
``CB.required``, the holes / voids through ``CB.cut_verified``, the shell confirmed by points in its walls, floor, a boss
and the restored tube (``CB.verify_points``; report ``derived.extra.gps_pod``).  The avionics module gets the lead entry
hole (``CB.cut_verified``, ``derived.extra.gps_entry_hole``).  New components: the printed ``gps_pod_lid``
(``build_gps_pod_lid``: plate, tube arc, chamfers, three posts, post holes, screw holes), the placeholders ``gps`` (patch +
board, two bodies) and ``gps_ground_plane``; new lumped part ``gps_lead`` at Layout.gps_lead_com.  The vplane family used
(theta 192 deg -> 12 deg mod 180) is the release pod's, which the v0.6.3 Fusion run built.

0.8.1 (v0.8.1 GNS board - electronics.gns / avionics_module.gns_mount, both presets; the GPS pod params thinner:
payload_module.gps_pod.wall_t / floor_t / lid.t, no code change there): ``build_gns_mount`` adds the four GNS-board bosses to
the finished avionics module in the CadQuery twin's terms (rocket_geometry.gns_bosses / gns_hole_cutters): each boss is a
diamond (Layout.gns_diamond_vz) in the vertical plane perpendicular to the radial direction gns_theta, extruded radially
from the board's back face u gns_u_back past the tube OD (``CB.tprism``, a new helper: sketch on vplane(theta + 90,
-u_mid) - for theta 180 the yZ construction plane offset by u_mid, the 0/90 family - symmetric extrude), JOIN; then
everything outside the OD over the bosses' Z range is cut (``gns_clip``, revolve_z, as the rail clip); the four blind tap
holes go in with ``CB.cut_between`` (the tab-screw-hole construction the v0.6.3 Fusion run built: cylinder between two
model points, free move, Combine/Cut, probe point on the axis) and are confirmed by a point beyond the hole end that must
stay material (``CB.verify_points``).  All ``CB.required``; report ``derived.extra.gns_mount``.  New placeholder component
``gns`` (Layout.gns_x/y/z, mass Layout.gns_mass_g = electronics.gns.mass_g_ASSUMED + the screws).  The GPS lead lump
(Layout.gps_lead_com) now ends at the GNS board.
"""
from __future__ import annotations

import datetime
import hashlib
import json
import math
import os
import re
import sys
import time
import traceback

import adsk.core
import adsk.fusion

SCRIPT_VERSION = "0.9.6"   # livery (2026-11-04): white printed parts, orange YUCHAN inlay part + logo pocket (was 0.9.5)
# (0.9.5 wire09c clock mount (2026-10-30): keyways round the circle, lead passage + blister)
# (0.9.2 v0.9 fix round: lip slit, tie anchors, coupler socket + set screw, bushings, tether boxes)
# (0.9.1 v0.9 CAD-2: mission.config vtol (config.txt / CANSAT_CONFIG), power sources)
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PRESET_FILE_NAME = "preset.txt"
CONFIG_FILE_NAME = "config.txt"     # v0.9 CAD-2: 'rocket' | 'vtol' [battery | tether | hybrid]
DIRECT_MODE_FILE_NAME = "direct_mode.txt"
# Modelling helpers (v0.5 features; not design dimensions): union overlap / cutter over-travel into
# material that is solid (union) or already void (cut), so the result equals the SPEC solid exactly.
OVERLAP = 1.0                   # mm
# Numbers SPEC.md states as literals (not in params.json / Layout):
WIRE_BULGE_EMBED = 0.6          # SPEC 5.1: wire bulge starts at r = body_r - 0.6 (inside the tube wall)
WIRE_NOTCH_PAST_CH_R0 = 0.5     # SPEC 5.1: shelf notch r in [wire_notch_r0, wire_ch_r0 + 0.5]
EAR_HOLE_DEPTH = 6.0            # SPEC 3.4 / 5.3: servo ear screw holes "6 mm deep" from the rim top
EDF_HUB_LEN = 70.0              # SPEC 6: hub cylinder from z_edf_housing_bottom to +70 mm
EDF_TAB_EMBED = 0.5             # modelling helper (same as the CadQuery twin): the edf_unit tab plates start 0.5 mm
                                # inside the housing wall and are joined to it (a flat plate on r = tab_r_in only
                                # touches the housing cylinder along a line)
# Components that leave the vehicle at ejection (SPEC section 6, post_ejection).
EJECTED = ("cansat_payload", "nose_cone", "chute_pack")
# Ground equipment: built and exported, never part of the vehicle mass.
NOT_IN_VEHICLE = ("launch_pad",)


# --------------------------------------------------------------------------- #
# small helpers
# --------------------------------------------------------------------------- #
def cm(mm: float) -> float:
    """Millimetres (params/layout) -> centimetres (Fusion API)."""
    return float(mm) / 10.0


def _num(x, default: float = 0.0) -> float:
    """Coerce an API value to float; never raises (mock objects, None)."""
    try:
        return float(x)
    except Exception:
        return default


def VI(real: float):
    return adsk.core.ValueInput.createByReal(float(real))


def P(x_mm: float, y_mm: float, z_mm: float):
    """Model-space Point3D from millimetres."""
    return adsk.core.Point3D.create(cm(x_mm), cm(y_mm), cm(z_mm))


def OC():
    return adsk.core.ObjectCollection.create()


def pod_xy(theta_deg: float, u: float, v: float) -> tuple[float, float]:
    """Pod frame (u radial, v tangential toward +e_theta) -> global XY (mm)."""
    t = math.radians(theta_deg)
    return (u * math.cos(t) - v * math.sin(t), u * math.sin(t) + v * math.cos(t))


def unique_mod180(thetas) -> list[float]:
    """Diametrically opposite angles share one through-cut; keep one per line."""
    seen: list[float] = []
    for t in thetas:
        k = round(t % 180.0, 6)
        if k not in seen:
            seen.append(k)
    return seen


def sanitize_param_name(name: str) -> str:
    s = re.sub(r"[^0-9A-Za-z_]", "_", name)
    if not s or not (s[0].isalpha() or s[0] == "_"):
        s = "p_" + s
    return s


def find_project_root(script_dir: str) -> str:
    """project_root.txt beside the script wins, then $CANSAT_PROJECT_ROOT, then ../../.."""
    marker = os.path.join(script_dir, "project_root.txt")
    candidates: list[str] = []
    if os.path.isfile(marker):
        with open(marker, "r", encoding="utf-8") as f:
            txt = f.read().strip()
        if txt:
            candidates.append(txt)
    env = os.environ.get("CANSAT_PROJECT_ROOT", "").strip()
    if env:
        candidates.append(env)
    candidates.append(os.path.normpath(os.path.join(script_dir, "..", "..", "..")))
    for c in candidates:
        if os.path.isfile(os.path.join(c, "params.json")) and os.path.isfile(os.path.join(c, "cad", "layout.py")):
            return os.path.normpath(c)
    raise FileNotFoundError("project root not found (need params.json and cad/layout.py); tried: " + "; ".join(candidates))


def read_preset(root: str) -> str | None:
    env = os.environ.get("CANSAT_PRESET", "").strip()
    if env:
        return env
    path = os.path.join(root, "cad", "fusion360", PRESET_FILE_NAME)
    if not os.path.isfile(path):
        return None
    with open(path, "r", encoding="utf-8") as f:
        for line in f:
            s = line.strip()
            if s and not s.startswith("#"):
                return s
    return None


def read_config(root: str) -> tuple[str | None, str | None]:
    """v0.9 CAD-2: hardware configuration and VTOL power source to build - $CANSAT_CONFIG / $CANSAT_POWER, else
    cad/fusion360/config.txt (first non-comment line: 'rocket' | 'vtol', optionally followed by the power source
    'battery' | 'tether' | 'hybrid'), else params.json mission.config (None, None)."""
    cfg = os.environ.get("CANSAT_CONFIG", "").strip() or None
    pw = os.environ.get("CANSAT_POWER", "").strip() or None
    if cfg is None:
        path = os.path.join(root, "cad", "fusion360", CONFIG_FILE_NAME)
        if os.path.isfile(path):
            with open(path, "r", encoding="utf-8") as f:
                for line in f:
                    s = line.split("#", 1)[0].split()
                    if s:
                        cfg = s[0]
                        pw = pw or (s[1] if len(s) > 1 else None)
                        break
    return cfg, pw


def config_file_suffix(lay) -> str:
    """v0.9 CAD-2: export-folder / report suffix of the built configuration ('' for the rocket)."""
    if not getattr(lay, "vtol", False):
        return ""
    return "_vtol" + ("" if lay.power_source == "battery" else f"_{lay.power_source}")


class RunLog:
    """Plain-text run log: one timestamped line per event, flushed immediately."""

    def __init__(self, path: str):
        self.path = path
        self.t0 = time.time()
        self.warnings: list[str] = []
        self._f = open(path, "w", encoding="utf-8", newline="\n")
        self.info(f"CanSatEDFRocket.py v{SCRIPT_VERSION} started {datetime.datetime.now().isoformat(timespec='seconds')}")

    def _w(self, level: str, msg: str) -> None:
        line = f"[{time.time() - self.t0:9.3f}s] {level:<5} {msg}\n"
        self._f.write(line)
        self._f.flush()

    def info(self, msg: str) -> None:
        self._w("INFO", msg)

    def warn(self, msg: str) -> None:
        self.warnings.append(msg)
        self._w("WARN", msg)

    def error(self, msg: str) -> None:
        self._w("ERROR", msg)

    def feat(self, comp: str, kind: str, label: str, **params) -> None:
        detail = " ".join(f"{k}={_fmt(v)}" for k, v in params.items())
        self._w("FEAT", f"{comp} {kind} '{label}' {detail}")

    def close(self) -> None:
        try:
            self._f.close()
        except Exception:
            pass


def _fmt(v) -> str:
    if isinstance(v, float):
        return f"{v:.4g}"
    if isinstance(v, (list, tuple)):
        return "[" + ",".join(_fmt(x) for x in v) + "]"
    return str(v)


# --------------------------------------------------------------------------- #
# sketch wrapper
# --------------------------------------------------------------------------- #
class Sk:
    """A sketch addressed in global model millimetres.

    Every point goes through ``Sketch.modelToSketchSpace`` so the sketch plane's
    2-D frame never has to be known; the out-of-plane coordinate is dropped.
    """

    def __init__(self, cb: "CB", sketch, label: str):
        self.cb = cb
        self.sk = sketch
        self.label = label

    def pt(self, x_mm: float, y_mm: float, z_mm: float):
        s = self.sk.modelToSketchSpace(P(x_mm, y_mm, z_mm))
        if abs(_num(s.z)) > 1e-3:
            self.cb.log.warn(f"{self.cb.name}:{self.label} point ({x_mm:.3f},{y_mm:.3f},{z_mm:.3f}) mm is {_num(s.z) * 10:.3f} mm off the sketch plane")
        return adsk.core.Point3D.create(_num(s.x), _num(s.y), 0.0)

    def polygon(self, pts_mm) -> list:
        """Closed polygon through model points [(x,y,z)...]; returns the SketchLines."""
        lines = self.sk.sketchCurves.sketchLines
        n = len(pts_mm)
        if n < 3:
            raise ValueError("polygon needs >= 3 points")
        first = lines.addByTwoPoints(self.pt(*pts_mm[0]), self.pt(*pts_mm[1]))
        out = [first]
        prev = first
        for i in range(2, n):
            ln = lines.addByTwoPoints(prev.endSketchPoint, self.pt(*pts_mm[i]))
            out.append(ln)
            prev = ln
        out.append(lines.addByTwoPoints(prev.endSketchPoint, first.startSketchPoint))
        return out

    def line(self, a_mm, b_mm):
        return self.sk.sketchCurves.sketchLines.addByTwoPoints(self.pt(*a_mm), self.pt(*b_mm))

    def circle(self, c_mm, d_mm: float):
        return self.sk.sketchCurves.sketchCircles.addByCenterRadius(self.pt(*c_mm), cm(d_mm) / 2.0)

    def annular_sector(self, r_in: float, r_out: float, th_lo_deg: float, th_hi_deg: float, z_mm: float) -> None:
        """Closed annular-sector profile (inner arc, radial line, outer arc, radial line) in a horizontal
        sketch at height z_mm.  The four corners are created first as SketchPoints and every curve is attached
        to them explicitly: Fusion stores a SketchArc counter-clockwise in sketch space, so an arc's
        start/endSketchPoint may be swapped relative to the creation order (the 2026-10-14 run joined the
        wrong corners and got a self-crossing loop).  Arcs by three points: no sweep sign either."""
        def pz(r, th):
            t = math.radians(th)
            return (r * math.cos(t), r * math.sin(t), z_mm)
        thm = 0.5 * (th_lo_deg + th_hi_deg)
        sp = self.sk.sketchPoints
        arcs = self.sk.sketchCurves.sketchArcs
        lines = self.sk.sketchCurves.sketchLines
        a_in_lo = sp.add(self.pt(*pz(r_in, th_lo_deg)))
        a_in_hi = sp.add(self.pt(*pz(r_in, th_hi_deg)))
        a_out_hi = sp.add(self.pt(*pz(r_out, th_hi_deg)))
        a_out_lo = sp.add(self.pt(*pz(r_out, th_lo_deg)))
        arcs.addByThreePoints(a_in_lo, self.pt(*pz(r_in, thm)), a_in_hi)
        lines.addByTwoPoints(a_in_hi, a_out_hi)
        arcs.addByThreePoints(a_out_hi, self.pt(*pz(r_out, thm)), a_out_lo)
        lines.addByTwoPoints(a_out_lo, a_in_lo)

    def spline_closed(self, pts_mm, close_via_mm) -> None:
        """Fitted spline through pts, closed by straight lines through close_via points."""
        coll = OC()
        for p in pts_mm:
            coll.add(self.pt(*p))
        spl = self.sk.sketchCurves.sketchFittedSplines.add(coll)
        lines = self.sk.sketchCurves.sketchLines
        prev_pt = spl.endSketchPoint
        for q in close_via_mm:
            ln = lines.addByTwoPoints(prev_pt, self.pt(*q))
            prev_pt = ln.endSketchPoint
        lines.addByTwoPoints(prev_pt, spl.startSketchPoint)

    def done(self) -> None:
        try:
            self.sk.isComputeDeferred = False
        except Exception:
            pass

    def profile(self, pick: str = "single"):
        """'single' (expect one; else largest), 'largest', or 'all' (ObjectCollection)."""
        profs = self.sk.profiles
        n = int(_num(profs.count, 0))
        if n < 1:
            raise RuntimeError(f"{self.cb.name}:{self.label}: sketch produced no closed profile")
        if pick == "all":
            coll = OC()
            for i in range(n):
                coll.add(profs.item(i))
            return coll
        if n == 1:
            return profs.item(0)
        if pick == "single":
            self.cb.log.warn(f"{self.cb.name}:{self.label}: expected 1 profile, got {n}; using the largest")
        best, best_a = None, -1.0
        for i in range(n):
            p = profs.item(i)
            a = _num(p.areaProperties().area, 0.0)
            if a > best_a:
                best, best_a = p, a
        return best


# --------------------------------------------------------------------------- #
# component builder
# --------------------------------------------------------------------------- #
class CB:
    """One Fusion component in global vehicle coordinates (identity occurrence).

    Construction planes are validated at run time: a probe sketch checks that a
    known model point lies on the plane (|sketch z| < 1e-4 cm) and the sign of
    the angle/offset is flipped when Fusion's convention differs.  All extrudes
    are symmetric about a mid-plane so the plane normal direction never matters.
    """

    def __init__(self, ctx: "Ctx", name: str):
        self.ctx = ctx
        self.log = ctx.log
        self.lay = ctx.lay
        self.name = name
        self.occ = ctx.root.occurrences.addNewComponent(adsk.core.Matrix3D.create())
        self.comp = self.occ.component
        self.comp.name = name
        self._planes: dict = {}
        self.body = None
        self.bodies: list = []
        self.cutters: list = []           # every tool body made by cut_between (must never survive into a part)
        self.required_ok = 0              # CB.required bookkeeping (v0.5 features)
        self.required_failed: list[str] = []
        self.unverified: list[str] = []   # v0.6 cut_verified / verify_points labels whose containment went unanswered
        self.n_sketches = 0
        self.n_features = 0

    # ---- feature operation names for the log ----
    def opname(self, op) -> str:
        return self.ctx.opnames.get(op, str(op))

    # ---- planes ----
    def _plane_ok(self, plane, test_mm) -> bool:
        sk = self.comp.sketches.add(plane)
        try:
            s = sk.modelToSketchSpace(P(*test_mm))
            return abs(_num(s.z, 1.0)) < 1e-4
        finally:
            try:
                sk.deleteMe()
            except Exception:
                pass

    def hplane(self, z_mm: float):
        """Plane parallel to XY at global Z = z_mm."""
        key = ("h", round(z_mm, 4))
        if key in self._planes:
            return self._planes[key]
        if abs(z_mm) < 1e-9:
            plane = self.comp.xYConstructionPlane
        else:
            plane = None
            for sgn in (1.0, -1.0):
                inp = self.comp.constructionPlanes.createInput()
                inp.setByOffset(self.comp.xYConstructionPlane, VI(cm(sgn * z_mm)))
                cand = self.comp.constructionPlanes.add(inp)
                try:
                    cand.name = f"Z {z_mm:g} mm"
                except Exception:
                    pass
                if self._plane_ok(cand, (3.0, 7.0, z_mm)):
                    plane = cand
                    if sgn < 0:
                        self.log.warn(f"{self.name}: XY offset sign flipped for Z={z_mm:g} mm")
                    break
                self.log.info(f"{self.name}: plane Z={z_mm:g} with offset sign {sgn:+.0f} rejected by probe")
                try:
                    cand.deleteMe()
                except Exception:
                    pass
            if plane is None:
                raise RuntimeError(f"{self.name}: could not create horizontal plane at Z={z_mm:g} mm")
        self._planes[key] = plane
        return plane

    def vplane(self, phi_deg: float, offset_mm: float = 0.0):
        """Vertical plane containing direction t=(cos phi, sin phi, 0) and Z, shifted
        offset_mm along its normal n=(-sin phi, cos phi, 0)."""
        key = ("v", round(phi_deg % 360.0, 4), round(offset_mm, 4))
        if key in self._planes:
            return self._planes[key]
        t = (math.cos(math.radians(phi_deg)), math.sin(math.radians(phi_deg)))
        n = (-t[1], t[0])
        test = (offset_mm * n[0] + 10.0 * t[0], offset_mm * n[1] + 10.0 * t[1], 7.0)
        pm = round(phi_deg % 180.0, 6)
        plane = None
        # Fusion's setByAngle convention (which side, and from which reference the angle is measured) is not
        # documented; 0/45/90-family angles cannot tell the conventions apart, an arbitrary angle (13.13 deg,
        # v0.5 screw lugs) can.  Try every candidate and keep the first that the probe confirms.
        angle_cands = (0.0,) if pm in (0.0, 90.0) else (phi_deg, -phi_deg, 90.0 - phi_deg, phi_deg - 90.0)
        off_signs = (1.0,) if abs(offset_mm) < 1e-9 else (1.0, -1.0)
        for ang in angle_cands:
            sa = 1.0 if ang == phi_deg or pm in (0.0, 90.0) else -1.0
            if pm == 0.0:
                base = self.comp.xZConstructionPlane
            elif pm == 90.0:
                base = self.comp.yZConstructionPlane
            else:
                inp = self.comp.constructionPlanes.createInput()
                inp.setByAngle(self.comp.zConstructionAxis, VI(math.radians(ang)), self.comp.xZConstructionPlane)
                base = self.comp.constructionPlanes.add(inp)
                try:
                    base.name = f"V {phi_deg:g} deg"
                except Exception:
                    pass
            made_base = pm not in (0.0, 90.0)
            if abs(offset_mm) < 1e-9:
                if self._plane_ok(base, test):
                    plane = base
                    if sa < 0:
                        self.log.info(f"{self.name}: vertical plane phi={phi_deg:g} built with setByAngle {ang:g} deg")
                    break
                self.log.info(f"{self.name}: vertical plane phi={phi_deg:g} setByAngle {ang:g} rejected by probe")
                if made_base:
                    try:
                        base.deleteMe()
                    except Exception:
                        pass
                continue
            for so in off_signs:
                inp2 = self.comp.constructionPlanes.createInput()
                inp2.setByOffset(base, VI(cm(so * offset_mm)))
                cand = self.comp.constructionPlanes.add(inp2)
                try:
                    cand.name = f"V {phi_deg:g} deg off {offset_mm:g} mm"
                except Exception:
                    pass
                if self._plane_ok(cand, test):
                    plane = cand
                    if sa < 0 or so < 0:
                        self.log.info(f"{self.name}: vertical plane phi={phi_deg:g} off={offset_mm:g} built with setByAngle {ang:g}, offset sign {so:+.0f}")
                    break
                self.log.info(f"{self.name}: vertical plane phi={phi_deg:g} off={offset_mm:g} (setByAngle {ang:g}, offset {so:+.0f}) rejected by probe")
                try:
                    cand.deleteMe()
                except Exception:
                    pass
            if plane is not None:
                break
            if made_base:
                try:
                    base.deleteMe()
                except Exception:
                    pass
        if plane is None:
            raise RuntimeError(f"{self.name}: could not create vertical plane phi={phi_deg:g} offset={offset_mm:g} mm")
        self._planes[key] = plane
        return plane

    # ---- sketches ----
    def sketch(self, plane, label: str) -> Sk:
        sk = self.comp.sketches.add(plane)
        self.n_sketches += 1
        try:
            sk.name = f"{self.name}:{label}"
        except Exception:
            pass
        try:
            sk.isComputeDeferred = True
        except Exception:
            pass
        return Sk(self, sk, label)

    # ---- feature bookkeeping ----
    def _after(self, feat, op, label: str):
        self.n_features += 1
        if op == self.ctx.NEW:
            bodies = feat.bodies
            nb = int(_num(bodies.count, 0))
            for i in range(nb):
                b = bodies.item(i)
                self.bodies.append(b)
                if self.body is None:
                    self.body = b
                    try:
                        b.name = self.name
                    except Exception:
                        pass
                else:
                    try:
                        b.name = f"{self.name}:{label}"
                    except Exception:
                        pass
        try:
            msg = feat.errorOrWarningMessage
            if isinstance(msg, str) and msg.strip():
                self.log.warn(f"{self.name}:{label}: feature message: {msg.strip()}")
        except Exception:
            pass
        return feat

    def _participants(self, op, participants):
        if participants is not None:
            return list(participants)
        if op in (self.ctx.CUT, self.ctx.JOIN) and self.body is not None:
            return [self.body]
        return None

    # ---- primitive features ----
    def extrude(self, sk: Sk, op, length_mm: float, label: str, participants=None, pick: str = "single"):
        sk.done()  # profiles are only computed once isComputeDeferred is cleared
        prof = sk.profile(pick)
        ext = self.comp.features.extrudeFeatures
        inp = ext.createInput(prof, op)
        inp.setSymmetricExtent(VI(cm(length_mm)), True)
        parts = self._participants(op, participants)
        if parts:
            inp.participantBodies = parts
        self.log.feat(self.name, "extrude", label, op=self.opname(op), full_length_mm=length_mm, sketch=sk.label)
        return self._after(ext.add(inp), op, label)

    def revolve(self, sk: Sk, axis, op, label: str, participants=None, pick: str = "single", half_angle_deg: float | None = None):
        """Full 360 deg revolve; with half_angle_deg a symmetric partial revolve of +-half_angle_deg about
        the sketch plane (Fusion: a symmetric angle is the extent on ONE side), so the sign of the
        revolve direction never matters."""
        sk.done()  # profiles are only computed once isComputeDeferred is cleared
        prof = sk.profile(pick)
        revs = self.comp.features.revolveFeatures
        inp = revs.createInput(prof, axis, op)
        if half_angle_deg is None:
            inp.setAngleExtent(False, VI(2.0 * math.pi))
        else:
            inp.setAngleExtent(True, VI(math.radians(half_angle_deg)))
        parts = self._participants(op, participants)
        if parts:
            inp.participantBodies = parts
        if half_angle_deg is None:
            self.log.feat(self.name, "revolve", label, op=self.opname(op), sketch=sk.label)
        else:
            self.log.feat(self.name, "revolve", label, op=self.opname(op), sketch=sk.label, symmetric_half_angle_deg=half_angle_deg)
        return self._after(revs.add(inp), op, label)

    # ---- composite shapes (all dimensions in mm, global coordinates) ----
    def revolve_z(self, pts_rz, op, label: str, participants=None):
        """Axisymmetric solid: closed (r, z) polygon in the XZ half-plane revolved about Z."""
        sk = self.sketch(self.vplane(0.0), label)
        sk.polygon([(r, 0.0, z) for (r, z) in pts_rz])
        self.log.feat(self.name, "revolve_z-profile", label, pts_rz=pts_rz)
        return self.revolve(sk, self.comp.zConstructionAxis, op, label, participants)

    def revolve_sector(self, theta_lo_deg: float, theta_hi_deg: float, pts_rz, op, label: str, participants=None):
        """Angular sector of an axisymmetric solid: closed (r, z) polygon revolved about Z from theta_lo
        to theta_hi.  Sketched in the radial plane at the mid angle and revolved symmetrically by half
        the sector on each side (no dependence on Fusion's revolve-direction sign)."""
        if theta_hi_deg <= theta_lo_deg:
            raise ValueError(f"{self.name}:{label}: empty sector {theta_lo_deg:g}..{theta_hi_deg:g} deg")
        mid = 0.5 * (theta_lo_deg + theta_hi_deg)
        half = 0.5 * (theta_hi_deg - theta_lo_deg)
        t = math.radians(mid)
        c, s = math.cos(t), math.sin(t)
        sk = self.sketch(self.vplane(mid), label)
        sk.polygon([(r * c, r * s, z) for (r, z) in pts_rz])
        self.log.feat(self.name, "revolve_sector-profile", label, theta=(theta_lo_deg, theta_hi_deg), pts_rz=pts_rz)
        return self.revolve(sk, self.comp.zConstructionAxis, op, label, participants, half_angle_deg=half)

    def radial_cyl(self, theta_deg: float, z_mm: float, r0: float, r1: float, d: float, op, label: str, participants=None):
        """Cylinder of diameter d on the horizontal radial line (theta, z) from r0 to r1
        (r may be negative = opposite side).  Revolve of a half-rectangle about the line."""
        t = math.radians(theta_deg)
        c, s = math.cos(t), math.sin(t)
        sk = self.sketch(self.vplane(theta_deg), label)
        pts = [(r0 * c, r0 * s, z_mm), (r1 * c, r1 * s, z_mm), (r1 * c, r1 * s, z_mm + d / 2.0), (r0 * c, r0 * s, z_mm + d / 2.0)]
        lines = sk.polygon(pts)
        self.log.feat(self.name, "radial_cyl-profile", label, theta=theta_deg, z=z_mm, r0=r0, r1=r1, d=d)
        return self.revolve(sk, lines[0], op, label, participants)

    # ---- body identity (API equality, never a position in comp.bRepBodies) ----
    @staticmethod
    def _same(a, b) -> bool:
        """API-object equality (Fusion's Base.__eq__ compares the underlying entity); False on error."""
        if a is None or b is None:
            return False
        try:
            return bool(a == b)
        except Exception:  # noqa: BLE001 - an invalidated proxy may refuse the comparison
            return False

    def _alive(self, body) -> bool:
        """True while ``body`` is still one of this component's bodies (looked up by API equality)."""
        if body is None:
            return False
        try:
            if not body.isValid:
                return False
        except Exception:  # noqa: BLE001
            return False
        bodies = self.comp.bRepBodies
        for i in range(int(_num(bodies.count, 0))):
            if self._same(bodies.item(i), body):
                return True
        return False

    def _delete_body(self, body, label: str) -> bool:
        """Delete exactly ``body`` (BRepBody.deleteMe, then a Remove feature as the fallback); True when it
        is gone.  Never touches any other body."""
        if not self._alive(body):
            return True
        if self._same(body, self.body):
            self.log.error(f"{self.name}:{label}: refusing to delete the main body")
            return False
        try:
            body.deleteMe()
        except Exception as exc:  # noqa: BLE001
            self.log.warn(f"{self.name}:{label}: deleteMe on the cutter body failed ({exc!r})")
        if not self._alive(body):
            self.log.warn(f"{self.name}:{label}: removed the cutter body")
            return True
        try:
            self.comp.features.removeFeatures.add(body)
        except Exception as exc:  # noqa: BLE001
            self.log.warn(f"{self.name}:{label}: Remove feature on the cutter body failed ({exc!r})")
        if not self._alive(body):
            self.log.warn(f"{self.name}:{label}: removed the cutter body (Remove feature)")
            return True
        self.log.error(f"{self.name}:{label}: the cutter body could NOT be removed")
        return False

    def _new_body_of(self, feat, label: str):
        """The single body a feature created (NewBody revolve) or moved (MoveFeature): ``feat.bodies.item(0)``.
        Any other count raises (after deleting whatever bodies that feature reported - never the main body)."""
        bodies = feat.bodies if feat is not None else None
        n = int(_num(bodies.count, 0)) if bodies is not None else 0
        if n == 1:
            return bodies.item(0)
        made = [bodies.item(i) for i in range(n)] if bodies is not None else []
        for b in made:
            self._delete_body(b, label)
        raise RuntimeError(f"{self.name}:{label}: the feature reported {n} bodies (expected 1)")

    def _cyl_body_between(self, a_mm, b_mm, d: float, label: str, participants=None):
        """Cylinder of diameter d between model points A and B as a NEW body; returns (|AB| mm, body).

        Built along +X from the origin (sketch on the XZ plane, which never needs a sign
        probe) and then placed with a free-move transform mapping the X axis onto AB.
        Angled-and-offset construction planes are avoided: Fusion 2705 rejected all four
        sign combinations for them in the 2026-10-12 run.  The body is taken from the revolve
        feature that created it (``feature.bodies.item(0)``); if the move or the placement check
        fails, exactly that body is deleted and the error is raised."""
        ax, ay, az = a_mm
        bx, by, bz = b_mm
        dx, dy, dz = bx - ax, by - ay, bz - az
        length = math.sqrt(dx * dx + dy * dy + dz * dz)
        if length < 1e-6:
            raise ValueError(f"{self.name}:{label}: zero-length cylinder")
        u = (dx / length, dy / length, dz / length)
        ref = (0.0, 0.0, 1.0) if abs(u[2]) < 0.9 else (1.0, 0.0, 0.0)
        v = (u[1] * ref[2] - u[2] * ref[1], u[2] * ref[0] - u[0] * ref[2], u[0] * ref[1] - u[1] * ref[0])
        vl = math.sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2])
        v = (v[0] / vl, v[1] / vl, v[2] / vl)
        w = (u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0])
        feat = self.radial_cyl(0.0, 0.0, 0.0, length, d, self.ctx.NEW, label, participants)
        body = self._new_body_of(feat, label)
        try:
            mat = adsk.core.Matrix3D.create()
            mat.setToAlignCoordinateSystems(
                adsk.core.Point3D.create(0.0, 0.0, 0.0), adsk.core.Vector3D.create(1.0, 0.0, 0.0),
                adsk.core.Vector3D.create(0.0, 1.0, 0.0), adsk.core.Vector3D.create(0.0, 0.0, 1.0),
                P(ax, ay, az), adsk.core.Vector3D.create(u[0], u[1], u[2]),
                adsk.core.Vector3D.create(v[0], v[1], v[2]), adsk.core.Vector3D.create(w[0], w[1], w[2]))
            ents = OC()
            ents.add(body)
            moves = self.comp.features.moveFeatures
            minp = moves.createInput2(ents)
            minp.defineAsFreeMove(mat)
            self.log.feat(self.name, "move", label, A=a_mm, B=b_mm, d=d, length_mm=length)
            mfeat = moves.add(minp)
            self.n_features += 1
            if not body.isValid:
                # the proxy did not survive the move: take the moved body from the move feature itself
                body = self._new_body_of(mfeat, label)
            # verify the placement: the moved body's bounding box must contain A and B
            bb = body.boundingBox
            lo = (_num(bb.minPoint.x) * 10.0, _num(bb.minPoint.y) * 10.0, _num(bb.minPoint.z) * 10.0)
            hi = (_num(bb.maxPoint.x) * 10.0, _num(bb.maxPoint.y) * 10.0, _num(bb.maxPoint.z) * 10.0)
            for pnt in (a_mm, b_mm):
                if any(pnt[k] < lo[k] - d or pnt[k] > hi[k] + d for k in range(3)):
                    raise RuntimeError(f"moved body bbox {_fmt(lo)}..{_fmt(hi)} mm does not contain {_fmt(pnt)}")
        except Exception:
            if not self._delete_body(body, label):
                self.log.error(f"{self.name}:{label}: a misplaced cylinder body remains in the component")
            raise
        return length, body

    def cyl_between(self, a_mm, b_mm, d: float, op, label: str, participants=None) -> float:
        """Cylinder of diameter d between model points A and B (NEW body); returns |AB| (mm)."""
        if op != self.ctx.NEW:
            raise ValueError("cyl_between supports NEW bodies only")
        return self._cyl_body_between(a_mm, b_mm, d, label, participants)[0]

    def _contains(self, body, p_mm) -> str:
        """'in' / 'out' / 'on' / '?' for a model point (mm) against a body (BRepBody.pointContainment)."""
        try:
            pc = body.pointContainment(P(*p_mm))
        except Exception:  # noqa: BLE001 - API not available (mock) or invalid proxy
            return "?"
        names = {getattr(adsk.fusion.PointContainment, "PointInsidePointContainment", 0): "in",
                 getattr(adsk.fusion.PointContainment, "PointOnPointContainment", 1): "on",
                 getattr(adsk.fusion.PointContainment, "PointOutsidePointContainment", 2): "out"}
        return names.get(pc, "?")

    def cut_between(self, a_mm, b_mm, d: float, label: str, target=None, probe_mm=None) -> float:
        """Cut a cylinder of diameter d between model points A and B out of ``target`` (default: the
        main body); returns |AB| (mm).  The cutter is the body created by _cyl_body_between (NEW body
        along +X, free move onto AB, identified from its own feature) and is subtracted with a
        Combine/Cut that consumes it.  The cut must remove material: the target's volume must drop by a
        finite amount > 1e-3 mm^3 (whether or not Fusion still lists the tool body afterwards), otherwise the
        cut raises - a required cut that removes nothing is a failed feature.  On any failure exactly that
        cutter body is deleted; it is also registered in ``self.cutters`` so combine_all can never join it
        into the part."""
        if self.body is None:
            raise RuntimeError(f"{self.name}:{label}: no main body (the cutter would become the part)")
        target = self.body if target is None else target
        length, tool = self._cyl_body_between(a_mm, b_mm, d, label)
        self.cutters.append(tool)
        self.bodies = [b for b in self.bodies if not self._same(b, tool)]
        try:
            if self._same(tool, target):
                raise RuntimeError("the cutter body is the target body")
            tools = OC()
            tools.add(tool)
            comb = self.comp.features.combineFeatures
            inp = comb.createInput(target, tools)
            inp.operation = self.ctx.CUT
            inp.isKeepToolBodies = False
            vol_before = _num(getattr(target, "volume", None), math.nan)       # cm^3
            pc_before = self._contains(target, probe_mm) if probe_mm is not None else "?"
            if pc_before == "out":
                raise RuntimeError(f"the probe point {probe_mm} is not in the target's material before the cut")
            self.log.feat(self.name, "combine", label, op=self.opname(self.ctx.CUT), A=a_mm, B=b_mm, d=d)
            cfeat = comb.add(inp)
            self.n_features += 1
            err = self._health_error(cfeat)
            if err is not None:
                raise RuntimeError(f"the Combine/Cut computed with an error: {err}")
            vol_after = _num(getattr(target, "volume", None), math.nan) if self._alive(target) else math.nan
            removed_mm3 = (vol_before - vol_after) * 1000.0
            # The cut is judged by point containment, which needs no volume accuracy: a point on the cutter axis
            # inside the material must be outside the target afterwards (BRepBody.volume has no documented
            # accuracy; Fusion's VeryHigh band, +-0.01 % = 34 mm^3 on this part, exceeds a 24.5 mm^3 tap hole).
            # Without a probe point (or when the API cannot answer) the volume drop is the fallback test.
            pc_after = self._contains(target, probe_mm) if probe_mm is not None else "?"
            if pc_before == "in" and pc_after in ("in", "on"):
                raise RuntimeError(f"the Combine/Cut left the probe point {probe_mm} inside the target (removed volume "
                                   f"reads {removed_mm3:.3f} mm^3)")
            if pc_after == "?" or pc_before == "?":
                if not (math.isfinite(removed_mm3) and removed_mm3 > 1e-3):
                    raise RuntimeError(f"the Combine/Cut removed no material from the target (volume before "
                                       f"{vol_before * 1000.0:.4f}, after {vol_after * 1000.0:.4f} mm^3; removed "
                                       f"{removed_mm3:.4f} mm^3; tool body still listed: {self._alive(tool)})")
            elif not (math.isfinite(removed_mm3) and removed_mm3 > 0.0):
                self.log.warn(f"{self.name}:{label}: probe point cut through, but the volume reads {removed_mm3:.3f} mm^3 "
                              f"removed (BRepBody.volume accuracy) - accepted")
            if self._alive(tool):
                # Fusion may keep listing a consumed tool body; the target did lose material, so delete the leftover
                self.log.warn(f"{self.name}:{label}: the cutter body is still listed after the Combine/Cut, but the "
                              f"target lost {removed_mm3:.2f} mm^3; deleting the leftover cutter")
                if not self._delete_body(tool, label):
                    raise RuntimeError("the leftover cutter body could not be removed")
        except Exception:
            if not self._delete_body(tool, label):
                raise RuntimeError(f"{self.name}:{label}: cut failed AND its cutter body could not be removed")
            raise
        return length

    def verify_points(self, label: str, inside_mm=(), outside_mm=()) -> bool:
        """BRepBody.pointContainment evidence for a small feature on the main body: every ``inside_mm`` point must be
        inside it and every ``outside_mm`` point outside ('on' counts as neither).  Raises on any contradiction;
        returns True when every point was answered, False when the API could not answer one (no evidence either way:
        the caller records it)."""
        body = self.body
        if body is None:
            raise RuntimeError(f"{self.name}:{label}: no main body to probe")
        res_in = [self._contains(body, p) for p in inside_mm]
        res_out = [self._contains(body, p) for p in outside_mm]
        bad = ([f"{_fmt(p)} should be inside ({r})" for p, r in zip(inside_mm, res_in) if r in ("out", "on")]
               + [f"{_fmt(p)} should be outside ({r})" for p, r in zip(outside_mm, res_out) if r in ("in", "on")])
        if bad:
            raise RuntimeError(f"{self.name}:{label}: point containment contradicts the feature: " + "; ".join(bad))
        return "?" not in res_in + res_out

    def cut_verified(self, label: str, make_cut, gone_mm, kept_mm=()):
        """A small CUT on the main body (v0.6 COM blind holes) judged by BRepBody.pointContainment, not by volume
        (BRepBody.volume has no documented accuracy; a Ø2.5 x 4.5 hole is 22 mm^3).  ``make_cut()`` runs the cut feature
        and returns it.  Every ``gone_mm`` point (inside the material the cut removes) must be inside the body before
        and outside after; every ``kept_mm`` point (e.g. the floor above a blind hole's end) must still be inside
        after.  Only when the API cannot answer a ``gone_mm`` point is the volume drop (> 1e-3 mm^3) the fallback
        evidence of the cut; an unanswered ``kept_mm`` point is recorded (``self.unverified``), never assumed.
        Raises on any contradiction; returns the feature."""
        body = self.body
        if body is None:
            raise RuntimeError(f"{self.name}:{label}: no main body (the cut has nothing to act on)")
        before = [self._contains(body, p) for p in gone_mm]
        not_in = [p for p, r in zip(gone_mm, before) if r in ("out", "on")]
        if not_in:
            raise RuntimeError(f"{self.name}:{label}: probe point(s) {_fmt(not_in)} not in the material before the cut")
        vol_before = _num(getattr(body, "volume", None), math.nan)
        feat = make_cut()
        err = self._health_error(feat)
        if err is not None:
            raise RuntimeError(f"{self.name}:{label}: the cut computed with an error: {err}")
        after = [self._contains(body, p) for p in gone_mm]
        left = [p for p, b, a in zip(gone_mm, before, after) if b == "in" and a in ("in", "on")]
        if left:
            raise RuntimeError(f"{self.name}:{label}: the cut left {_fmt(left)} inside the body")
        kept = [self._contains(body, p) for p in kept_mm]
        lost = [p for p, k in zip(kept_mm, kept) if k in ("out", "on")]
        if lost:
            raise RuntimeError(f"{self.name}:{label}: {_fmt(lost)} is no longer material (the cut ran past its end)")
        if "?" in before + after:
            removed_mm3 = (vol_before - _num(getattr(body, "volume", None), math.nan)) * 1000.0
            if not (math.isfinite(removed_mm3) and removed_mm3 > 1e-3):
                raise RuntimeError(f"{self.name}:{label}: point containment unanswered and the volume reads {removed_mm3:.4f} "
                                   f"mm^3 removed: no evidence the cut removed material")
            self.log.warn(f"{self.name}:{label}: point containment unanswered; cut accepted on the volume drop "
                          f"({removed_mm3:.3f} mm^3)")
        if "?" in before + after + kept:
            self.unverified.append(label)
        if "?" in kept:
            # no evidence either way that the material beside / beyond the cut survived: logged here and listed by the
            # caller's report (``containment_unanswered``), never counted as confirmed
            self.log.warn(f"{self.name}:{label}: point containment unanswered for {kept.count('?')} of {len(kept)} "
                          f"point(s) that must stay material; recorded as unverified")
        return feat

    @staticmethod
    def _health_error(feat) -> str | None:
        """Fusion's message when ``feat`` (a feature object) computed with an error health state, else None."""
        try:
            if feat.healthState == adsk.fusion.FeatureHealthStates.ErrorFeatureHealthState:
                return str(feat.errorOrWarningMessage or "error health state")
        except Exception:  # noqa: BLE001 - not a feature (e.g. a length) or no health information
            return None
        return None

    def required(self, label: str, fn) -> bool:
        """Run a feature the SPEC requires (v0.5 tab mount / wire path; v0.5d floor wedge, tray stop, blind
        propulsion joint holes).  A feature that raises, or returns
        a feature computed with an error health state, fails: log an ERROR, record it and return False so
        the remaining features are still built for inspection; Ctx.build_component then raises for this
        component (-> report ``failed_components``)."""
        try:
            out = fn()
            err = None if isinstance(out, (int, float)) or out is None else self._health_error(out)
            if err is not None:
                raise RuntimeError(f"feature computed with an error: {err}")
        except Exception as exc:  # noqa: BLE001 - recorded and re-raised per component by build_component
            self.log.error(f"{self.name}:{label}: REQUIRED feature failed: {exc!r}")
            self.required_failed.append(f"{label}: {exc!r}")
            return False
        self.required_ok += 1
        return True

    def prism(self, theta_deg: float, pts_rz, width_mm: float, op, label: str, participants=None):
        """Plate in the vertical radial plane at theta: (r, z) polygon extruded symmetrically
        by width_mm in the tangential direction."""
        t = math.radians(theta_deg)
        c, s = math.cos(t), math.sin(t)
        sk = self.sketch(self.vplane(theta_deg), label)
        sk.polygon([(r * c, r * s, z) for (r, z) in pts_rz])
        self.log.feat(self.name, "prism-profile", label, theta=theta_deg, pts_rz=pts_rz, width=width_mm)
        return self.extrude(sk, op, width_mm, label, participants)

    def plate(self, theta_deg: float, outer_rz, holes_rz, width_mm: float, op, label: str, participants=None):
        """prism() with through-holes: outer (r, z) polygon minus the hole polygons (each strictly inside
        it), one sketch, extruded symmetrically by width_mm tangentially.  The plate-with-holes is the
        largest of the sketch's profiles."""
        t = math.radians(theta_deg)
        c, s = math.cos(t), math.sin(t)
        sk = self.sketch(self.vplane(theta_deg), label)
        sk.polygon([(r * c, r * s, z) for (r, z) in outer_rz])
        for hole in holes_rz:
            sk.polygon([(r * c, r * s, z) for (r, z) in hole])
        self.log.feat(self.name, "plate-profile", label, theta=theta_deg, outer_rz=outer_rz, holes=len(holes_rz), width=width_mm)
        return self.extrude(sk, op, width_mm, label, participants, pick="largest" if holes_rz else "single")

    def box_pod(self, theta_deg: float, u0: float, u1: float, v0: float, v1: float, z0: float, z1: float, op, label: str, participants=None):
        """Box in the pod frame at theta (u radial, v tangential), Z from z0 to z1."""
        zm = (z0 + z1) / 2.0
        sk = self.sketch(self.hplane(zm), label)
        corners = [pod_xy(theta_deg, u0, v0), pod_xy(theta_deg, u1, v0), pod_xy(theta_deg, u1, v1), pod_xy(theta_deg, u0, v1)]
        sk.polygon([(x, y, zm) for (x, y) in corners])
        self.log.feat(self.name, "box_pod-profile", label, theta=theta_deg, u=(u0, u1), v=(v0, v1), z=(z0, z1))
        return self.extrude(sk, op, abs(z1 - z0), label, participants)

    def box_xy(self, x0: float, x1: float, y0: float, y1: float, z0: float, z1: float, op, label: str, participants=None):
        return self.box_pod(0.0, x0, x1, y0, y1, z0, z1, op, label, participants)

    def tprism(self, theta_deg: float, u_mid: float, pts_vz, length_mm: float, op, label: str, participants=None):
        """v0.8.1: polygon in the vertical plane perpendicular to the radial direction theta at u = u_mid (pod frame:
        (v tangential, z)), extruded symmetrically by length_mm along the radial direction (u_mid +- length / 2).
        Sketched on vplane(theta + 90, -u_mid): that plane holds the tangential direction and Z and sits at u = u_mid (its
        normal is -e_u); for theta 180 it is the yZ construction plane offset by u_mid (the 0/90 family, sign probed)."""
        sk = self.sketch(self.vplane(theta_deg + 90.0, -u_mid), label)
        sk.polygon([(pod_xy(theta_deg, u_mid, v)[0], pod_xy(theta_deg, u_mid, v)[1], z) for (v, z) in pts_vz])
        self.log.feat(self.name, "tprism-profile", label, theta=theta_deg, u_mid=u_mid, pts_vz=pts_vz, length=length_mm)
        return self.extrude(sk, op, length_mm, label, participants)

    def inlay(self, theta_deg: float, rings_vz, r_in: float, r_out: float, sector, u_mid: float, length_mm: float, op,
              label: str):
        """0.9.6 livery (rocket_geometry.livery_inlay_solids): one logo island at copy azimuth theta - the annular sector
        r [r_in, r_out] over ``sector`` = (theta_lo, theta_hi, z0, z1) (CB.sector_z, NEW body) INTERSECTED with the
        island's outline: the pre-warped (v, z) rings (outer first, then holes, Layout.livery_polys_vz) sketched on the
        tprism plane vplane(theta + 90, -u_mid) and extruded symmetrically by length_mm along the radial direction with
        participantBodies = [that body] (the v0.6.2 gable-window construction).  op NEW: the result stays as a body of
        this component (the orange inlay part, avionics_logo).  op CUT: it is the pocket tool - registered in ``cutters``,
        subtracted from the main body by a Combine/Cut that consumes it, deleted on any failure (the white body).
        The sketch has one profile per hole besides the island; the island is the largest ('largest' pick)."""
        ctx = self.ctx
        if op not in (ctx.NEW, ctx.CUT):
            raise ValueError(f"{self.name}:{label}: inlay op must be NewBody or Cut")
        if op == ctx.CUT and self.body is None:
            raise RuntimeError(f"{self.name}:{label}: no main body (the pocket tool would become the part)")
        lo, hi, z0, z1 = sector
        feat = self.sector_z(lo, hi, r_in, r_out, z0, z1, ctx.NEW, f"{label}_band")
        tool = self._new_body_of(feat, label)
        if op == ctx.CUT:
            self.cutters.append(tool)
            self.bodies = [b for b in self.bodies if not self._same(b, tool)]
        try:
            sk = self.sketch(self.vplane(theta_deg + 90.0, -u_mid), f"{label}_outline")
            for ring in rings_vz:
                sk.polygon([(pod_xy(theta_deg, u_mid, v)[0], pod_xy(theta_deg, u_mid, v)[1], z) for (v, z) in ring])
            self.log.feat(self.name, "inlay-profile", label, theta=theta_deg, rings=len(rings_vz),
                          points=sum(len(r) for r in rings_vz), r=(r_in, r_out), sector=tuple(sector), u_mid=u_mid,
                          length=length_mm, op=self.opname(op))
            ifeat = self.extrude(sk, ctx.INTERSECT, length_mm, f"{label}_outline", participants=[tool], pick="largest")
            err = self._health_error(ifeat)
            if err is not None:
                raise RuntimeError(f"the outline intersect computed with an error: {err}")
            if not self._alive(tool):
                raise RuntimeError("the outline intersect left no inlay body")
            if op == ctx.NEW:
                return ifeat
            tools = OC()
            tools.add(tool)
            comb = self.comp.features.combineFeatures
            inp = comb.createInput(self.body, tools)
            inp.operation = ctx.CUT
            inp.isKeepToolBodies = False
            self.log.feat(self.name, "combine", label, op=self.opname(ctx.CUT), tool="logo pocket")
            cfeat = comb.add(inp)
            self.n_features += 1
            err = self._health_error(cfeat)
            if err is not None:
                raise RuntimeError(f"the pocket Combine/Cut computed with an error: {err}")
            if self._alive(tool):
                self.log.warn(f"{self.name}:{label}: the pocket tool is still listed after the Combine/Cut; deleting it")
                if not self._delete_body(tool, label):
                    raise RuntimeError("the leftover pocket tool body could not be removed")
            return cfeat
        except Exception:
            if op == ctx.CUT and not self._delete_body(tool, label):
                raise RuntimeError(f"{self.name}:{label}: logo pocket failed AND its tool body could not be removed")
            raise

    def poly_pod(self, theta_deg: float, pts_uv, z0: float, z1: float, op, label: str, participants=None):
        """v0.8: plan polygon in the pod frame at theta (u radial, v tangential), extruded from z0 to z1 (horizontal
        sketch at mid height, symmetric extrude - the box_pod construction with any polygon)."""
        zm = (z0 + z1) / 2.0
        sk = self.sketch(self.hplane(zm), label)
        sk.polygon([(x, y, zm) for (x, y) in (pod_xy(theta_deg, u, v) for (u, v) in pts_uv)])
        self.log.feat(self.name, "poly_pod-profile", label, theta=theta_deg, pts_uv=pts_uv, z=(z0, z1))
        return self.extrude(sk, op, abs(z1 - z0), label, participants)

    def sector_z(self, th_lo_deg: float, th_hi_deg: float, r_in: float, r_out: float, z0: float, z1: float, op, label: str,
                 participants=None):
        """Annular sector r_in..r_out, th_lo..th_hi (deg), extruded from z0 to z1: sketched in the horizontal
        plane at mid height and extruded symmetrically (no tilted construction plane - Fusion rejected
        setByAngle planes at 13.13 deg in the 2026-10-14 run - and no extrude-direction sign)."""
        if th_hi_deg <= th_lo_deg:
            raise ValueError(f"{self.name}:{label}: empty sector {th_lo_deg:g}..{th_hi_deg:g} deg")
        zm = (z0 + z1) / 2.0
        sk = self.sketch(self.hplane(zm), label)
        sk.annular_sector(r_in, r_out, th_lo_deg, th_hi_deg, zm)
        self.log.feat(self.name, "sector_z-profile", label, theta=(th_lo_deg, th_hi_deg), r=(r_in, r_out), z=(z0, z1))
        sk.done()
        n_prof = int(_num(sk.sk.profiles.count, 1))
        if n_prof != 1:
            raise RuntimeError(f"{self.name}:{label}: annular sector gave {n_prof} profiles (expected exactly 1)")
        # (the geometry itself is verified afterwards by the slice comparison with the CadQuery twin, X3)
        return self.extrude(sk, op, abs(z1 - z0), label, participants)

    def cyl_z(self, x: float, y: float, z0: float, z1: float, d: float, op, label: str, participants=None):
        """Vertical cylinder (axis parallel to Z) of diameter d from z0 to z1."""
        zm = (z0 + z1) / 2.0
        sk = self.sketch(self.hplane(zm), label)
        sk.circle((x, y, zm), d)
        self.log.feat(self.name, "cyl_z-profile", label, x=x, y=y, z=(z0, z1), d=d)
        return self.extrude(sk, op, abs(z1 - z0), label, participants)

    def through_cut(self, theta_normal_deg: float, draw, length_mm: float, label: str, participants=None):
        """Through-cut whose tool is extruded symmetrically along the horizontal direction
        at theta_normal.  ``draw(sk, t)`` draws the tool profile(s) with model points
        s*t + z*Z (t = in-plane horizontal unit vector, returned to the callback)."""
        phi = theta_normal_deg - 90.0
        t = (math.cos(math.radians(phi)), math.sin(math.radians(phi)))
        sk = self.sketch(self.vplane(phi), label)
        draw(sk, t)
        self.log.feat(self.name, "through_cut-profile", label, theta_normal=theta_normal_deg, length=length_mm)
        return self.extrude(sk, self.ctx.CUT, length_mm, label, participants, pick="all")

    def through_circle(self, theta_normal_deg: float, z_mm: float, d: float, length_mm: float, label: str, flat_z: float | None = None, participants=None):
        """Through-cut circle centred on the vehicle axis at height z (optionally a D-profile
        with a flat at z = flat_z, keeping the larger region)."""
        def draw(sk: Sk, t):
            sk.circle((0.0, 0.0, z_mm), d)
            if flat_z is not None:
                hc = math.sqrt(max((d / 2.0) ** 2 - (z_mm - flat_z) ** 2, 0.0)) + 0.5
                sk.line((-hc * t[0], -hc * t[1], flat_z), (hc * t[0], hc * t[1], flat_z))
        phi = theta_normal_deg - 90.0
        tt = (math.cos(math.radians(phi)), math.sin(math.radians(phi)))
        sk = self.sketch(self.vplane(phi), label)
        draw(sk, tt)
        self.log.feat(self.name, "through_circle-profile", label, theta_normal=theta_normal_deg, z=z_mm, d=d, flat_z=flat_z, length=length_mm)
        return self.extrude(sk, self.ctx.CUT, length_mm, label, participants, pick="largest" if flat_z is not None else "single")

    # ---- edges / faces ----
    def edges_at_z(self, body, z_mm: float, min_len_mm: float, tol_mm: float = 0.05) -> list:
        out = []
        for e in body.edges:
            bb = e.boundingBox
            zmin, zmax = _num(bb.minPoint.z) * 10.0, _num(bb.maxPoint.z) * 10.0
            if abs(zmin - z_mm) < tol_mm and abs(zmax - z_mm) < tol_mm and _num(e.length) * 10.0 >= min_len_mm:
                out.append(e)
        return out

    def planar_face_at_z(self, body, z_mm: float, tol_mm: float = 0.05):
        plane_type = adsk.core.Plane.classType()
        for f in body.faces:
            try:
                if f.geometry.objectType != plane_type:
                    continue
            except Exception:
                continue
            bb = f.boundingBox
            zmin, zmax = _num(bb.minPoint.z) * 10.0, _num(bb.maxPoint.z) * 10.0
            if abs(zmin - z_mm) < tol_mm and abs(zmax - z_mm) < tol_mm:
                return f
        return None

    def fillet(self, edges, r_mm: float, label: str):
        coll = OC()
        for e in edges:
            coll.add(e)
        fil = self.comp.features.filletFeatures
        inp = fil.createInput()
        inp.addConstantRadiusEdgeSet(coll, VI(cm(r_mm)), False)
        self.log.feat(self.name, "fillet", label, radius=r_mm, edges=len(edges))
        self.n_features += 1
        return fil.add(inp)

    def shell_remove_face(self, face, wall_mm: float, label: str):
        coll = OC()
        coll.add(face)
        shells = self.comp.features.shellFeatures
        inp = shells.createInput(coll, False)
        inp.insideThickness = VI(cm(wall_mm))
        self.log.feat(self.name, "shell", label, wall=wall_mm)
        self.n_features += 1
        return shells.add(inp)

    def combine_all(self, label: str = "combine_all") -> int:
        """Join every body of the component into the main body; returns body count after.  Raises instead
        of joining when a cutter body made by cut_between is still present (it would fill a hole)."""
        n = int(_num(self.comp.bRepBodies.count, 0))
        if n <= 1 or self.body is None:
            return n
        left = sum(1 for c in self.cutters if self._alive(c))
        if left:
            raise RuntimeError(f"{self.name}: {left} cutter body(ies) still present; refusing to join them into the part")
        tools = OC()
        for i in range(n):
            b = self.comp.bRepBodies.item(i)
            if b != self.body:
                tools.add(b)
        comb = self.comp.features.combineFeatures
        inp = comb.createInput(self.body, tools)
        inp.operation = self.ctx.JOIN
        inp.isKeepToolBodies = False
        self.log.feat(self.name, "combine", label, tools=n - 1)
        comb.add(inp)
        self.n_features += 1
        return int(_num(self.comp.bRepBodies.count, 0))

    def soft(self, label: str, fn):
        """Run an optional feature; log and continue on failure."""
        try:
            return fn()
        except Exception as exc:  # noqa: BLE001 - optional geometry must never kill the component
            self.log.warn(f"{self.name}:{label}: optional feature failed: {exc!r}")
            self.ctx.soft_failures.append(f"{self.name}:{label}: {exc!r}")
            return None


# --------------------------------------------------------------------------- #
# shared sub-builders
# --------------------------------------------------------------------------- #
def shoulder_t(L) -> float:
    """Thickness of the ring that ties a male spigot to its tube (SPEC 2): the body wall."""
    return L.body_wall


def shoulder_r_in(L) -> float:
    """Inner radius of that ring: mid-wall of the spigot, so it overlaps the spigot by a face."""
    return L.spigot_od / 2.0 - L.p["body"]["spigot_wall"] / 2.0


def skin_between(L, z_a: float, z_b: float) -> list[tuple[float, float]]:
    """(r, z) points of the EDF section's outer skin from z_a up to z_b: both end points
    (Layout.cone_outer_radius) plus every vertex of Layout.outer_skin_profile() strictly between them
    (v0.5 tabs: the cone -> cylinder kink at cone_z1).  With no interior vertex this is exactly the
    two-point edge v0.4 used."""
    pts = [(L.cone_outer_radius(z_a), z_a)]
    for (r, z) in L.outer_skin_profile():
        if z_a + 1e-9 < z < z_b - 1e-9:
            pts.append((r, z))
    pts.append((L.cone_outer_radius(z_b), z_b))
    return pts


def build_pod(cb: CB, theta: float, base_r: float, r_in: float, r_out: float, w: float, z0: float, rim_z: float,
              cav_r_in: float, cav_r_out: float, cav_w: float, floor_top_z: float, axis_r: float, tan_ch: float, servo: dict,
              label: str) -> None:
    """SPEC 3.4 / 5.3 servo cup: box + 45 deg underside gusset/wedge, open cavity, ear holes.

    The chamfer line runs from (u=base_r, z=z0-(r_out-base_r)*tan) to (r_out, z0); the
    box is extruded from the (clamped) foot of that line so the wedge cut leaves a
    support-free 45 deg underside (see spec_conflicts note in the report)."""
    z_a = z0 - (r_out - base_r) * tan_ch
    z_lo = max(z_a, 0.0)
    cb.box_pod(theta, r_in, r_out, -w / 2.0, w / 2.0, z_lo, rim_z, cb.ctx.JOIN, f"{label}_box")
    # wedge below the chamfer line
    u_lo = base_r + (z_lo - z_a) / tan_ch if tan_ch > 1e-9 else base_r
    u_hi = r_out + 1.0
    z_hi = z_a + (u_hi - base_r) * tan_ch
    wedge = [(u_lo, z_lo - 1.0), (u_lo, z_lo), (u_hi, z_hi), (u_hi, z_lo - 1.0)]
    cb.soft(f"{label}_wedge", lambda: cb.prism(theta, wedge, w + 2.0, cb.ctx.CUT, f"{label}_wedge"))
    # cavity, open at the top
    cb.box_pod(theta, cav_r_in, cav_r_out, -cav_w / 2.0, cav_w / 2.0, floor_top_z, rim_z + 1.0, cb.ctx.CUT, f"{label}_cavity")
    # servo ear screw holes from the rim top face downward
    pitch = servo["ear_hole_pitch"]
    depth = EAR_HOLE_DEPTH
    for sgn in (-1.0, 1.0):
        x, y = pod_xy(theta, axis_r, sgn * pitch / 2.0)
        cb.soft(f"{label}_ear_hole", lambda x=x, y=y: cb.cyl_z(x, y, rim_z - depth, rim_z + 1.0, servo["ear_hole_d"], cb.ctx.CUT, f"{label}_ear_hole"))


def build_servo(cb: CB, theta: float, axis_r: float, bottom_z: float, rim_z: float, arm_z_bottom: float) -> None:
    """SPEC 4.3 servo placeholder: case + ears + output boss, one body."""
    L = cb.lay
    s = L.p["servo"]
    bw, bl = s["body_w"], s["body_l"]
    case_top = bottom_z + s["case_top_from_bottom"]
    cb.box_pod(theta, axis_r - bw / 2.0, axis_r + bw / 2.0, -bl / 2.0, bl / 2.0, bottom_z, case_top, cb.ctx.NEW, "case")
    cb.box_pod(theta, axis_r - bw / 2.0, axis_r + bw / 2.0, -s["ear_span"] / 2.0, s["ear_span"] / 2.0, rim_z, rim_z + s["ear_t"], cb.ctx.JOIN, "ears")
    x, y = pod_xy(theta, axis_r, L.servo_shaft_tangential_offset)
    cb.cyl_z(x, y, case_top - 1.0, arm_z_bottom, s["boss_d"], cb.ctx.JOIN, "boss")


def tab_boss_sector(L, tab_theta: float) -> tuple[float, float]:
    """SPEC 3.7 screw-boss sector (deg): head side at tab_theta + head_side * block_theta_head, far
    (tap) side at tab_theta - head_side * block_theta_far; returned as (low, high)."""
    a_head = tab_theta + L.tab_screw_head_side * L.block_theta_head
    a_far = tab_theta - L.tab_screw_head_side * L.block_theta_far
    return min(a_head, a_far), max(a_head, a_far)


def tab_screw_cuts(L) -> list[tuple[str, float, float, float]]:
    """SPEC 3.7 holes along each tab-screw axis: (name, t_start, t_end, diameter), t in mm along
    Layout.tab_screw_point: clearance Ø tab_screw_d_clear for [t_keyway, t_head], counterbore
    Ø tab_screw_d_cbore for [t_head, tab_screw_t_cbore_end] (its innermost edge has left the skin there),
    tap Ø tab_screw_d_tap for [t_tap_end, -t_keyway].  The clearance and tap holes run OVERLAP into the
    keyway / counterbore voids (cutter over-travel; removes nothing more than the SPEC ranges)."""
    return [
        ("clear", L.tab_screw_t_keyway - OVERLAP, L.tab_screw_t_head + OVERLAP, L.tab_screw_d_clear),
        ("cbore", L.tab_screw_t_head, L.tab_screw_t_cbore_end, L.tab_screw_d_cbore),
        ("tap", L.tab_screw_t_tap_end, -L.tab_screw_t_keyway + OVERLAP, L.tab_screw_d_tap),
    ]


def tab_screw_probe_t(L, kind: str) -> float:
    """t (mm along Layout.tab_screw_point) of a point on the cut's axis that lies in lug material before that cut
    and in its void after: mid clearance hole, the counterbore well inside the skin, mid tap hole."""
    if kind == "clear":
        return 0.5 * (L.tab_screw_t_keyway + L.tab_screw_t_head)
    if kind == "cbore":
        return 0.5 * (L.tab_screw_t_head + min(L.tab_screw_t_cbore_end, L.tab_screw_t_out - 3.0))
    if kind == "tap":
        return 0.5 * (L.tab_screw_t_tap_end - L.tab_screw_t_keyway)
    raise ValueError(kind)


def tab_lug_r_in(L) -> float:
    """Inner radius of the screw-lug sectors as modelled: block_r_in (= sleeve_r) moved OVERLAP (at most
    half the sleeve wall) into the solid sleeve, so the union shares volume instead of a face.  The
    region added is sleeve material, so the united solid is exactly SPEC 3.7's r in [block_r_in, skin]."""
    return L.block_r_in - min(OVERLAP, 0.5 * (L.block_r_in - L.edf_bore_r))


def liner_profile(L, z_from: float) -> list[tuple[float, float]]:
    """SPEC 3.7 thin liner as one closed (r, z) polygon: tube r [duct_r, liner_r_out] from z_from (the
    tube runs from z_edf_shoulder; z_from may start lower, inside the solid shoulder ring) up to
    z_liner_top, plus the top ring r [duct_r, sleeve_r] over [z_liner_ring_bottom, z_liner_top]."""
    return [(L.duct_r, z_from), (L.liner_r_out, z_from), (L.liner_r_out, L.z_liner_ring_bottom),
            (L.sleeve_r, L.z_liner_ring_bottom), (L.sleeve_r, L.z_liner_top), (L.duct_r, L.z_liner_top)]


def wire_slot_boxes(L) -> list[tuple[str, float, float, float, float]]:
    """SPEC 3.7 wire slot at wire_theta (tangential +-wire_slot_w/2) as (label, r0, r1, z0, z1):
    (a) r [wire_slot_r_in, wire_slot_lower_r_out], Z [wire_slot_z0, z_prop_spigot_bottom] - stops short of
    the skin; (b) r [wire_slot_r_in, wire_slot_r_out], Z [z_prop_spigot_bottom, wire_slot_z1] - through the
    spigot ring, open at the top.  (a) over-travels OVERLAP up into (b) when its radial range lies inside
    (b)'s (a region (b) removes anyway, so the two voids never meet on a shared face)."""
    zsb = L.z_prop_spigot_bottom
    za1 = min(zsb + OVERLAP, L.wire_slot_z1) if L.wire_slot_lower_r_out <= L.wire_slot_r_out else zsb
    return [("wire_slot_lower", L.wire_slot_r_in, L.wire_slot_lower_r_out, L.wire_slot_z0, za1),
            ("wire_slot_spigot", L.wire_slot_r_in, L.wire_slot_r_out, zsb, L.wire_slot_z1)]


class ReqGroup:
    """Runs a group of CB.required features and counts them for the report (derived.extra)."""

    def __init__(self, cb: CB):
        self.cb = cb
        self.labels: list[str] = []
        self.n_ok = 0
        self.failed: list[str] = []

    def __call__(self, label: str, fn) -> bool:
        self.labels.append(label)
        ok = self.cb.required(label, fn)
        if ok:
            self.n_ok += 1
        else:
            self.failed.append(self.cb.required_failed[-1])
        return ok

    def summary(self) -> dict:
        return {"features_ok": self.n_ok, "features_total": len(self.labels), "failed": list(self.failed)}


def clock_col_wall_rz(L) -> tuple[list, list]:
    """wire09c: (r, z) polygons of a servo-lead column's U channel - the outer one (the web rule: sleeve_r - OVERLAP ..
    outer skin - 1.0, shoulder-ring top - OVERLAP .. deck bottom + OVERLAP; rocket_geometry.clock_col_wall_rz) and the
    channel between the walls (exactly the sleeve -> skin cavity, shoulder-ring top .. deck bottom)."""
    z0 = L.z_edf_shoulder - OVERLAP
    z1 = L.z_prop_spigot_bottom - shoulder_t(L) + OVERLAP
    sk = skin_between(L, z0, z1)
    outer = [(L.sleeve_r - OVERLAP, z0)] + [(r - 1.0, z) for (r, z) in sk] + [(L.sleeve_r - OVERLAP, z1)]
    c0, c1 = L.z_edf_shoulder, L.z_prop_spigot_bottom - shoulder_t(L)
    sk2 = skin_between(L, c0, c1)
    chan = [(L.sleeve_r, c0)] + [(r - L.cone_wall, z) for (r, z) in sk2] + [(L.sleeve_r, c1)]
    return outer, chan


def build_clock_mount(ctx: "Ctx", cb: CB) -> None:
    """wire09c clock mount (propulsion_module.tab_mount 'clock_keyways', user 2026-10-30): thin liner, a floor pad under
    every keyway (prism of Layout.ck_pad_rz, 45 deg corbel), the servo-lead column guide walls (U prism joined, its
    channel cut back to the cavity), the lead-passage blister (poly_pod of Layout.lp_blister_xy) and its 45 deg V keel
    (tprisms of Layout.lp_keel), then the keyways round the circle (box_pod) and the lead passage (poly_pod of
    Layout.lp_fan_xy).  No tab screws / lugs.  Every feature CB.required (mirrors rocket_geometry.propulsion_module)."""
    L = ctx.lay
    req = ReqGroup(cb)
    try:
        liner = liner_profile(L, L.z_edf_shoulder - OVERLAP)
        req("liner", lambda: cb.revolve_z(liner, ctx.JOIN, "liner"))
        for a in L.ck_keyways:
            lbl = f"floor_pad_{a:g}"
            req(lbl, lambda a=a, lbl=lbl: cb.prism(a, [tuple(q) for q in L.ck_pad_rz], L.ck_pad_w, ctx.JOIN, lbl))
        if getattr(L, "ck_col_walls", None):
            outer, chan = clock_col_wall_rz(L)
            cols = sorted({c for c, _, _ in L.ck_col_walls})
            for c in cols:
                vs = [(v0, v1) for c2, v0, v1 in L.ck_col_walls if c2 == c]
                w_out = 2.0 * max(abs(v) for pair in vs for v in pair)
                w_in = 2.0 * min(abs(v) for pair in vs for v in pair)
                lbl = f"col_wall_{c:g}"
                req(lbl, lambda c=c, w_out=w_out, lbl=lbl: cb.prism(c, outer, w_out, ctx.JOIN, lbl))
                lbl = f"col_channel_{c:g}"
                req(lbl, lambda c=c, w_in=w_in, lbl=lbl: cb.prism(c, chan, w_in, ctx.CUT, lbl))
        req("lead_blister", lambda: cb.poly_pod(0.0, [tuple(q) for q in L.lp_blister_xy], L.lp_z_blister_bottom,
                                                L.lp_z_blister_top, ctx.JOIN, "lead_blister"))
        zb, ov = L.lp_z_blister_bottom, L.lp_keel_overlap
        for i, (u0, u1, hk) in enumerate(L.lp_keel):
            lbl = f"blister_keel_{i}"
            req(lbl, lambda u0=u0, u1=u1, hk=hk, lbl=lbl: cb.tprism(L.wire_theta, 0.5 * (u0 + u1),
                                                                     [(-hk, zb + ov), (hk, zb + ov), (0.0, zb - hk)],
                                                                     u1 - u0, ctx.JOIN, lbl))
        h = L.ck_kw / 2.0
        for a in L.ck_keyways:
            lbl = f"keyway_{a:g}"
            req(lbl, lambda a=a, lbl=lbl: cb.box_pod(a, L.keyway_r_in, L.keyway_r_out, -h, h, L.lp_z_floor, L.keyway_z1,
                                                       ctx.CUT, lbl))
        req("lead_passage", lambda: cb.poly_pod(0.0, [tuple(q) for q in L.lp_fan_xy], L.lp_z_fan_floor,
                                                L.z_prop_top + OVERLAP, ctx.CUT, "lead_passage"))
    finally:
        ctx.report_extra["tab_mount"] = {
            "edf_mount": L.edf_mount, "tab_mount": "clock_keyways", "tab_thetas_deg (nominal seat)": list(L.tab_thetas),
            "keyways_deg": list(L.ck_keyways), "keyway_floor_z_mm": L.lp_z_floor, "pad_t_mm": L.ck_pad_t,
            "lead_passage_theta_deg": L.wire_theta, "lead_passage_E_design_deg": L.lp_E_design,
            "blister_r_out_mm": L.lp_blister_r_out, "keel_steps": len(L.lp_keel), **req.summary()}
    ctx.log.info(f"{cb.name}: clock mount {req.n_ok}/{len(req.labels)} required features "
                 f"({len(L.ck_keyways)} keyways)")


def build_tab_mount(ctx: "Ctx", cb: CB) -> None:
    """SPEC 3.7 (edf.mount = tabs): thin liner, screw lugs, keyways, tangential tab-screw holes, wire slot.
    Every feature is CB.required: one failure fails the propulsion module (never a soft skip).  wire09c: with
    tab_mount 'clock_keyways' the clock mount (build_clock_mount) instead."""
    L = ctx.lay
    if getattr(L, "clock_mount", False):
        build_clock_mount(ctx, cb)
        return
    req = ReqGroup(cb)
    screws = {"ok": 0, "total": 0}
    try:
        # liner: tube + top ring (closed cavity behind the tube); the tube starts OVERLAP down inside the
        # solid shoulder ring so the union shares volume
        liner = liner_profile(L, L.z_edf_shoulder - OVERLAP)
        req("liner", lambda: cb.revolve_z(liner, ctx.JOIN, "liner"))
        # screw lugs: one angular sector per tab and per boss_z_ranges entry, r tab_lug_r_in .. outer skin
        # (the lugs lie in the cylindrical section, so the outer arc r = body_r IS the skin); the lowest lug
        # reaches below keyway_z0 = keyway floor.  Horizontal annular-sector extrudes (no tilted planes).
        r_in = tab_lug_r_in(L)
        for th in L.tab_thetas:
            lo, hi = tab_boss_sector(L, th)
            for k, (z0, z1) in enumerate(L.boss_z_ranges):
                if any(abs(r - L.body_r) > 1e-9 for r, _ in skin_between(L, z0, z1)):
                    raise RuntimeError(f"lug {th:g}/{k} is not inside the cylindrical section")
                lbl = f"tab_lug_{th:g}_{k}"
                req(lbl, lambda lo=lo, hi=hi, z0=z0, z1=z1, lbl=lbl:
                    cb.sector_z(lo, hi, r_in, L.body_r, z0, z1, ctx.JOIN, lbl))
        # keyways: open at the top; the cut starts at keyway_z0 so the lug below is the tab's axial seat
        hw = L.keyway_w / 2.0
        for th in L.tab_thetas:
            lbl = f"keyway_{th:g}"
            req(lbl, lambda th=th, lbl=lbl: cb.box_pod(th, L.keyway_r_in, L.keyway_r_out, -hw, hw, L.keyway_z0, L.keyway_z1, ctx.CUT, lbl))
        # tab screws: clearance, counterbore (leaves through the skin) and tap hole along the tangential axis
        for th in L.tab_thetas:
            for z in L.tab_screw_zs:
                for (kind, t0, t1, d) in tab_screw_cuts(L):
                    screws["total"] += 1
                    a, b = L.tab_screw_point(th, z, t0), L.tab_screw_point(th, z, t1)
                    pr = L.tab_screw_point(th, z, tab_screw_probe_t(L, kind))   # in material before this cut
                    lbl = f"tab_screw_{th:g}_z{z:g}_{kind}"
                    if req(lbl, lambda a=a, b=b, d=d, lbl=lbl, pr=pr: cb.cut_between(a, b, d, lbl, probe_mm=pr)):
                        screws["ok"] += 1
        # motor-lead wire slot: lower box through sleeve and deck, upper box through the spigot ring
        hs = L.wire_slot_w / 2.0
        for (lbl, r0, r1, z0, z1) in wire_slot_boxes(L):
            req(lbl, lambda lbl=lbl, r0=r0, r1=r1, z0=z0, z1=z1: cb.box_pod(L.wire_theta, r0, r1, -hs, hs, z0, z1, ctx.CUT, lbl))
        # v0.9 lead-exit round (user 2026-10-29; photo round: the heat-shrink bundle from the motor's front ring passes
        # under the rear rim): lead notch + lower slot, one (u, z) polygon prism (Layout.ml_low_cut_uz, ml_low_cut_w wide)
        # cut after the liner (joined above)
        if getattr(L, "ml_low_cut_uz", None):
            req("wire_low_cut", lambda: cb.prism(L.wire_theta, [tuple(q) for q in L.ml_low_cut_uz], L.ml_low_cut_w,
                                                 ctx.CUT, "wire_low_cut"))
    finally:
        ctx.report_extra["tab_mount"] = {
            "edf_mount": L.edf_mount, "tab_thetas_deg": list(L.tab_thetas), "screw_zs_mm": list(L.tab_screw_zs),
            "boss_sectors_deg": [list(tab_boss_sector(L, th)) for th in L.tab_thetas],
            "lug_z_ranges_mm": [list(r) for r in L.boss_z_ranges],
            "cbore_t_mm": [L.tab_screw_t_head, L.tab_screw_t_cbore_end],
            "screw_hole_cuts_ok": screws["ok"], "screw_hole_cuts_total": screws["total"],
            **req.summary()}
    ctx.log.info(f"{cb.name}: tab mount {req.n_ok}/{len(req.labels)} required features, "
                 f"{screws['ok']}/{screws['total']} tab-screw cuts")


def printed_density(L, name: str, root: str) -> float:
    """v0.9 (printer.mass_model): density (g/cm^3) of a printed component - mode 'slicer_part_g': the effective density of
    its part in the slicer table (part grams / the CAD solid volume it was sliced from), else its class's
    effective_density_fallback_g_cm3; mode 'solid' (or none): printer.density_g_cm3.  The CadQuery twin uses the table's
    grams for the identical solid; Fusion's own volume x this density agrees with it to the volume difference."""
    pr = L.p["printer"]
    mm = pr.get("mass_model") or {}
    if mm.get("mode") != "slicer_part_g":
        return float(pr["density_g_cm3"])
    stem = name.rsplit("_", 1)
    base = stem[0] if len(stem) == 2 and stem[1].isdigit() else name
    try:
        with open(os.path.join(root, mm["table"]), "r", encoding="utf-8") as f:
            ent = json.load(f).get("parts", {}).get(base)
    except (OSError, ValueError, KeyError):
        ent = None
    if ent and ent.get("part_g") and ent.get("solid_volume_mm3"):
        return float(ent["part_g"]) * 1000.0 / float(ent["solid_volume_mm3"])
    cls = (pr.get("part_class") or {}).get(base)
    return float(pr["print_classes"][cls]["effective_density_fallback_g_cm3"]) if cls else float(pr["density_g_cm3"])


def fin_pt(L, k: int, u: float, v: float, z: float) -> tuple[float, float, float]:
    """Model point (mm) in the frame of fin k (u radial at its theta, v tangential, Z) - Layout.fin_xyz."""
    return tuple(L.fin_xyz(k, u, v, z))


def build_fairings(ctx: "Ctx", cb: CB) -> None:
    """v0.9 (SPEC 3.4 v0.9, Layout.fair_* / sl_pipe_points): per fin the fairing box (fin frame, from fin_root_r - into the
    duct wall - to fair_u[1], v +-fair_hv, Z fair_z) and its 45 deg keel (tprism of Layout.fair_keel_vz over the box's
    radial length), JOINed; with the servo-lead route, the lead pipe and its web pipe (cyl_between NEW bodies, then
    combine_all) - before the pocket / tunnel cuts."""
    L = ctx.lay
    req = ReqGroup(cb)
    um, ln = 0.5 * (L.fair_u[0] + L.fair_u[1]), L.fair_u[1] - L.fair_u[0]
    for k, th in enumerate(L.fin_thetas):
        req(f"fairing_{th:g}", lambda th=th: cb.box_pod(th, L.fair_u[0], L.fair_u[1], -L.fair_hv, L.fair_hv, L.fair_z[0],
                                                         L.fair_z[1], ctx.JOIN, f"fairing_{th:g}"))
        req(f"fairing_keel_{th:g}", lambda th=th: cb.tprism(th, um, L.fair_keel_vz, ln, ctx.JOIN, f"fairing_keel_{th:g}"))
    if getattr(L, "servo_leads", False):
        for k, th in enumerate(L.fin_thetas):
            a, b = L.sl_pipe_points(k)
            wa, wb = L.sl_web_points(k)
            req(f"lead_pipe_{th:g}", lambda a=a, b=b, th=th: cb.cyl_between(a, b, L.sl_pipe_d, ctx.NEW, f"lead_pipe_{th:g}"))
            req(f"lead_web_{th:g}", lambda a=wa, b=wb, th=th: cb.cyl_between(a, b, L.sl_pipe_d, ctx.NEW, f"lead_web_{th:g}"))
        req("lead_pipes_join", lambda: cb.combine_all("lead_pipes_join"))
    ctx.report_extra["fairings"] = {"fin_thetas_deg": list(L.fin_thetas), "u_mm": list(L.fair_u), "half_w_mm": L.fair_hv,
                                    "z_mm": list(L.fair_z), "keel_z0_mm": L.fair_keel_z0, **req.summary()}
    ctx.log.info(f"{cb.name}: fin-root fairings {req.n_ok}/{len(req.labels)} required features")


def build_fairing_cuts(ctx: "Ctx", cb: CB) -> None:
    """v0.9: per fin the servo pocket (closed top), the service opening under the lintel, the lower ear tab's floor slot
    (box_pod CUTs) and the two cover-screw tap holes (cut_between from 1 mm outside the service face to the tap end)."""
    L = ctx.lay
    req = ReqGroup(cb)
    ss, hv, pv = L.dd_service_side, L.fair_hv, L.dd_pocket_hv
    v0, v1 = (-pv, hv + OVERLAP) if ss > 0 else (-hv - OVERLAP, pv)
    for k, th in enumerate(L.fin_thetas):
        req(f"servo_pocket_{th:g}", lambda th=th: cb.box_pod(th, L.dd_pocket_u[0], L.dd_pocket_u[1], -pv, pv, L.dd_pocket_z[0],
                                                              L.dd_pocket_z[1], ctx.CUT, f"servo_pocket_{th:g}"))
        req(f"servo_opening_{th:g}", lambda th=th: cb.box_pod(th, L.dd_pocket_u[0], L.dd_pocket_u[1], v0, v1, L.dd_pocket_z[0],
                                                               L.dd_open_z1, ctx.CUT, f"servo_opening_{th:g}"))
        req(f"ear_slot_{th:g}", lambda th=th: cb.box_pod(th, L.dd_slot_u[0], L.dd_slot_u[1], v0, v1, L.dd_slot_z[0],
                                                          L.dd_pocket_z[0] + OVERLAP, ctx.CUT, f"ear_slot_{th:g}"))
        for j, (u, z) in enumerate(L.cov_screws_uz):
            a = fin_pt(L, k, u, ss * (hv + OVERLAP), z)
            b = fin_pt(L, k, u, ss * (hv - L.cov_tap_depth), z)
            req(f"cover_tap_{th:g}_{j}", lambda a=a, b=b: cb.cut_between(a, b, L.cov_d_tap, f"cover_tap"))
    ctx.report_extra["fairing_cuts"] = {"pocket_u_mm": list(L.dd_pocket_u), "pocket_z_mm": list(L.dd_pocket_z),
                                        "open_z1_mm": L.dd_open_z1, "slot_u_mm": list(L.dd_slot_u),
                                        "cover_screws_uz_mm": [list(q) for q in L.cov_screws_uz], **req.summary()}
    ctx.log.info(f"{cb.name}: servo pockets {req.n_ok}/{len(req.labels)} required cuts")


def build_servo_lead_cuts(ctx: "Ctx", cb: CB) -> None:
    """SPEC 3.8 (v0.6, Layout.servo_leads): pod lead windows (far end wall), entry slots through the shoulder ring /
    cone skin, deck notches ((a) deck box to spigot_od/2 - the skirt seat outside stays, (b) spigot box out past the
    spigot OD up into the rebate) and the spigot-ring rebate (horizontal annular-sector cut, no tilted plane).  Every
    cut is CB.required and CB.cut_verified: a point in the material it removes must leave the body, a point in the wall
    it must leave (window roof, ring beside the slot, skirt seat, spigot ring inside the notch / rebate, spigot ring
    outside the rebate's sector) must stay."""
    L = ctx.lay
    req = ReqGroup(cb)

    def pt(th: float, u: float, v: float, z: float) -> tuple[float, float, float]:
        x, y = pod_xy(th, u, v)
        return (x, y, z)

    try:
        if getattr(L, "direct_drive", False):
            # v0.9: each lead's closed tunnel (O sl_tunnel_d) along the lead pipe, 3 mm from inside the pocket to 1 mm
            # past the bottom of its entry slot (rocket_geometry.lead_tunnel)
            for k, th in enumerate(L.fin_thetas):
                a, b = L.sl_pipe_points(k)
                d = [b[i] - a[i] for i in range(3)]
                n = math.sqrt(sum(x * x for x in d))
                ua = tuple(a[i] - 3.0 * d[i] / n for i in range(3))
                ub = tuple(b[i] + OVERLAP * d[i] / n for i in range(3))
                lbl = f"servo_tunnel_{th:g}"
                req(lbl, lambda ua=ua, ub=ub, lbl=lbl: cb.cut_between(ua, ub, L.sl_tunnel_d, lbl))
        else:
            a, b = L.pod_cavity_w / 2.0 - OVERLAP, L.pod_w / 2.0 + OVERLAP
            v0, v1 = (a, b) if L.sl_side > 0 else (-b, -a)
            v_wall = L.sl_side * 0.5 * (L.pod_cavity_w / 2.0 + L.pod_w / 2.0)        # mid end wall
            for th in L.pod_thetas:
                lbl = f"servo_window_{th:g}"
                gone = [pt(th, L.servo_axis_r, v_wall, 0.5 * (L.sl_win_z0 + L.sl_win_z1))]
                kept = [pt(th, L.servo_axis_r, v_wall, L.sl_win_z1 + 0.5 * (L.pod_rim_z - EAR_HOLE_DEPTH - L.sl_win_z1))]
                req(lbl, lambda th=th, lbl=lbl, gone=gone, kept=kept: cb.cut_verified(
                    lbl, lambda: cb.box_pod(th, L.sl_win_u0, L.sl_win_u1, v0, v1, L.sl_win_z0, L.sl_win_z1, ctx.CUT, lbl),
                    gone, kept))
        he, hn = L.sl_entry_w / 2.0, L.sl_notch_w / 2.0
        zsb, so = L.z_prop_spigot_bottom, L.spigot_od / 2.0
        ue = 0.5 * (L.sl_entry_u0 + L.sl_entry_u1)
        un = 0.5 * (L.sl_notch_u0 + so)
        z_ring = min(L.z_edf_shoulder, L.outer_skin_z_at_radius(L.sl_entry_u1)) - 0.5   # solid ring around the slot
        z_deck = zsb - 0.5 * L.body_wall
        z_spig = 0.5 * (zsb + L.sl_rebate_z0)
        for th in L.sl_col_thetas:
            lbl = f"servo_entry_{th:g}"
            req(lbl, lambda th=th, lbl=lbl: cb.cut_verified(
                lbl, lambda: cb.box_pod(th, L.sl_entry_u0, L.sl_entry_u1, -he, he, L.sl_entry_z0, L.sl_entry_z_open + OVERLAP,
                                        ctx.CUT, lbl),
                [pt(th, ue, 0.0, z_ring)], [pt(th, L.sl_entry_u0 - 1.0, 0.0, z_ring)]))
            lbl = f"servo_deck_notch_{th:g}"
            req(lbl, lambda th=th, lbl=lbl: cb.cut_verified(
                lbl, lambda: cb.box_pod(th, L.sl_notch_u0, so, -hn, hn, zsb - L.body_wall - OVERLAP, zsb + OVERLAP, ctx.CUT, lbl),
                [pt(th, un, 0.0, z_deck)], [pt(th, 0.5 * (so + L.body_r), 0.0, z_deck)]))
            lbl = f"servo_spigot_notch_{th:g}"
            req(lbl, lambda th=th, lbl=lbl: cb.cut_verified(
                lbl, lambda: cb.box_pod(th, L.sl_notch_u0, so + OVERLAP, -hn, hn, zsb, L.sl_rebate_z0 + OVERLAP, ctx.CUT, lbl),
                [pt(th, un, 0.0, z_spig)], [pt(th, L.sl_notch_u0 - 1.0, 0.0, z_spig)]))
        z_reb = 0.5 * (L.sl_rebate_z0 + L.z_prop_spigot_top)
        r_reb = 0.5 * (L.sl_rebate_r0 + so)
        # outside the rebate: midway between the motor wire slot's edge and the rebate's start (spigot ring stays)
        th_out = L.sl_rebate_kept_theta      # Layout: between the lead passage and the rebate, clear of every keyway
        gone = [pt(t, r_reb, 0.0, z_reb) for t in (L.sl_rebate_th0 + 1.0, L.sl_gather_theta, L.sl_rebate_th1 - 1.0)]
        kept = [pt(L.sl_gather_theta, L.sl_rebate_r0 - 1.0, 0.0, z_reb), pt(th_out, r_reb, 0.0, z_reb)]
        req("servo_rebate", lambda: cb.cut_verified(
            "servo_rebate", lambda: cb.sector_z(L.sl_rebate_th0, L.sl_rebate_th1, L.sl_rebate_r0, so + OVERLAP, L.sl_rebate_z0,
                                                L.z_prop_spigot_top + OVERLAP, ctx.CUT, "servo_rebate"), gone, kept))
    finally:
        ctx.report_extra["servo_leads"] = {
            "pod_thetas_deg": list(L.fin_thetas if getattr(L, "direct_drive", False) else L.pod_thetas),
            "column_thetas_deg": list(L.sl_col_thetas),
            "rebate_theta_deg": [L.sl_rebate_th0, L.sl_rebate_th1], "rebate_r_mm": [L.sl_rebate_r0, L.spigot_od / 2.0],
            "rebate_z_mm": [L.sl_rebate_z0, L.z_prop_spigot_top], "gather_theta_deg": L.sl_gather_theta,
            "lead_mass_g": L.sl_lead_mass_g, "lead_com_mm": list(L.sl_lead_com),
            "containment_unanswered": [x for x in cb.unverified if x.startswith("servo_")], **req.summary()}
    ctx.log.info(f"{cb.name}: servo lead route {req.n_ok}/{len(req.labels)} required cuts; point containment unanswered "
                 f"for {sum(1 for x in cb.unverified if x.startswith('servo_'))}")


# --------------------------------------------------------------------------- #
# component builders (SPEC sections 3-6)
# --------------------------------------------------------------------------- #
def build_propulsion_module(ctx: "Ctx", cb: CB) -> None:
    """SPEC 3 (+ 3.7 when Layout.tabs).  The outer skin (45 deg chamfer, linear cone and, v0.5 tabs, the
    cylinder above cone_z1) comes only from Layout.cone_outer_radius / outer_skin_profile (skin_between)."""
    L = ctx.lay
    body = L.p["body"]

    # 3.1 exhaust duct
    cb.revolve_z([(L.duct_r, L.z_exit), (L.vane_section_r, L.z_exit), (L.vane_section_r, L.z_vane_section_top), (L.duct_r, L.z_vane_section_top)], ctx.NEW, "exhaust_duct")
    # 3.2 shoulder ring (solid between duct bore and cone outer surface)
    cb.revolve_z(L.shoulder_ring_profile(), ctx.JOIN, "shoulder_ring")
    # cone shell (wall measured radially inward); v0.5 tabs: plus the cylinder cone_z1 .. spigot bottom
    # (Layout.outer_skin_profile).  lip_clamp: cone_z1 == z_prop_spigot_bottom -> the v0.4 trapezoid.
    skin = skin_between(L, L.cone_z0, L.z_prop_spigot_bottom)
    cb.revolve_z([(skin[0][0] - L.cone_wall, skin[0][1])] + skin + [(r - L.cone_wall, z) for (r, z) in reversed(skin[1:])], ctx.JOIN, "cone_shell")
    # inner sleeve from 1 mm below the EDF shoulder (overlaps the solid shoulder ring); starting it lower would
    # leave an unsupported ring outside the 45 deg chamfer when the sleeve is wider than the vane section
    zs0 = L.z_sleeve_bottom - 1.0
    cb.revolve_z([(L.edf_bore_r, zs0), (L.sleeve_r, zs0), (L.sleeve_r, L.z_prop_spigot_bottom), (L.edf_bore_r, L.z_prop_spigot_bottom)], ctx.JOIN, "sleeve")
    # spigot ring, with a 1 mm notch down into the sleeve wall for a guaranteed overlap
    zsb, zst = L.z_prop_spigot_bottom, L.z_prop_spigot_top
    cb.revolve_z([(L.edf_bore_r, zsb - 1.0), (L.sleeve_r, zsb - 1.0), (L.sleeve_r, zsb), (L.spigot_od / 2.0, zsb), (L.spigot_od / 2.0, zst), (L.edf_bore_r, zst)], ctx.JOIN, "spigot_ring")
    # deck: solid annulus EDF bore -> cone outer surface just below the spigot ring (SPEC 3.2);
    # ties the spigot ring to the cone shell and is the seat of the intake skirt
    t_deck = shoulder_t(L)
    cb.revolve_z([(L.edf_bore_r, zsb - t_deck)] + skin_between(L, zsb - t_deck, zsb) + [(L.edf_bore_r, zsb)], ctx.JOIN, "deck")
    # webs: sleeve OD -> cone inner surface (1 mm overlaps into both walls); the outer edge follows the
    # skin, including the v0.5 cone -> cylinder kink
    z0w, z1w = L.z_sleeve_bottom, L.z_prop_spigot_bottom
    for th in L.web_thetas:
        cb.prism(th, [(L.sleeve_r - 1.0, z0w)] + [(r - 1.0, z) for (r, z) in skin_between(L, z0w, z1w)] + [(L.sleeve_r - 1.0, z1w)],
                 L.web_t, ctx.JOIN, f"web_{th:g}")
    # 3.7 v0.5 tab mount: thin liner, screw lugs, keyways, tab-screw holes, two-box wire slot
    if L.tabs:
        build_tab_mount(ctx, cb)
    # 3.3 hinge bosses and bores
    for th in L.vane_thetas:
        cb.radial_cyl(th, L.vane_hinge_z, L.duct_r, L.hinge_boss_r_out, L.hinge_boss_od, ctx.JOIN, f"hinge_boss_{th:g}")
    direct = getattr(L, "direct_drive", False)
    if direct:
        # v0.9 (vane_drive.mode 'direct'): fin-root fairings + 45 deg keels (+ the lead pipes) united BEFORE the hinge
        # bores, so the bores run through the fairing's inner wall into the pocket
        build_fairings(ctx, cb)
    r_bore = (L.dd_pocket_u[0] + 1.0) if direct else (L.hinge_boss_r_out + 1.0)

    def hinge_bores() -> None:
        for th in unique_mod180(L.vane_thetas):
            cb.radial_cyl(th, L.vane_hinge_z, -r_bore, r_bore, L.hinge_bore_d, ctx.CUT, f"hinge_bore_{th:g}")
    if not direct:
        hinge_bores()
    if not direct:
        # 3.4 servo pods
        tan_ch = math.tan(math.radians(L.p["pod"]["underside_chamfer_deg"]))
        for th in L.pod_thetas:
            build_pod(cb, th, L.vane_section_r, L.pod_r_in, L.pod_r_out, L.pod_w, L.pod_z0, L.pod_rim_z,
                      L.pod_cavity_r_in, L.pod_cavity_r_out, L.pod_cavity_w, L.pod_floor_top_z, L.servo_axis_r, tan_ch, L.p["servo"], f"pod_{th:g}")
        # re-cut the duct bore so no pod material intrudes into the exhaust
        if L.pod_r_in < L.duct_r:   # only needed when a pod box can reach into the exhaust bore
            cb.soft("duct_recut", lambda: cb.cyl_z(0.0, 0.0, L.z_exit - 1.0, L.z_vane_section_top, L.duct_id, ctx.CUT, "duct_recut"))
    # 3.5 fins
    for th in L.fin_thetas:
        cb.prism(th, [(L.fin_root_r, L.fin_te_z), (L.fin_root_r, L.fin_root_le_z), (L.fin_tip_r, L.fin_tip_le_z), (L.fin_tip_r, L.fin_te_z)], L.fin_t, ctx.JOIN, f"fin_{th:g}")
    if direct:
        # v0.9: the fins lie on the vane axes (fin theta0 0), so the hinge bores are cut AFTER the fin plates - else the
        # plate fills the bore between the fairing's inner face and the pocket (X3 replay: 78.9 mm3 per fin, the vane
        # shaft would not pass) - then the servo pockets, service openings, ear slots and cover-screw tap holes
        hinge_bores()
        build_fairing_cuts(ctx, cb)
    # spigot screw holes (self-tapping M3)
    zj = L.z_prop_spigot_bottom + L.joint_screw_dz
    so = L.spigot_od / 2.0
    if L.prop_joint_hole_depth:
        # v0.5d (SPEC 3.2, every preset): the spigot ring is solid down to the EDF bore, so the holes are blind -
        # one radial cutter per joint_theta from r = spigot_od/2 - prop_joint_hole_depth out past the spigot OD
        # (OVERLAP into air: nothing of this module lies outside the spigot OD above the deck), so an over-long
        # joint screw bottoms in plastic instead of reaching the EDF housing.  CB.required (never a soft skip).
        r0 = so - L.prop_joint_hole_depth
        if r0 <= L.edf_bore_r:
            raise RuntimeError(f"blind joint hole r0={r0:g} mm reaches the EDF bore r={L.edf_bore_r:g} mm")
        jreq = ReqGroup(cb)
        for th in L.joint_thetas:
            lbl = f"spigot_hole_{th:g}"
            jreq(lbl, lambda th=th, lbl=lbl: cb.radial_cyl(th, zj, r0, so + OVERLAP, body["joint_hole_spigot_d"], ctx.CUT, lbl))
        ctx.report_extra["prop_joint_holes"] = {"blind": True, "thetas_deg": list(L.joint_thetas), "z_mm": zj,
                                                "r_mm": [r0, so], "depth_mm": L.prop_joint_hole_depth,
                                                "d_mm": body["joint_hole_spigot_d"], **jreq.summary()}
    else:
        # no prop_joint_hole_depth in params: v0.4 through holes (one cut per diametral line)
        for th in unique_mod180(L.joint_thetas):
            cb.soft("spigot_hole", lambda th=th: cb.radial_cyl(th, zj, -(so + 1.0), so + 1.0, body["joint_hole_spigot_d"], ctx.CUT, f"spigot_hole_{th:g}"))
    # 3.8 v0.6 vane-servo lead route: pod windows, entry slots, deck notches, spigot-ring rebate (after every other cut)
    if getattr(L, "servo_leads", False):
        build_servo_lead_cuts(ctx, cb)
    # v0.9 CAD-2 VTOL: one lightening hole per fin (Layout.fin_hole_uz, pentagon with a 45 deg gable), cut last
    # (rocket_geometry.fin_hole_cutters: the prism +-(fin_t/2 + OVERLAP) in the fin plane); CB.required
    if getattr(L, "has_fin_hole", False):
        freq = ReqGroup(cb)
        for th in L.fin_thetas:
            lbl = f"fin_hole_{th:g}"
            freq(lbl, lambda th=th, lbl=lbl: cb.prism(th, [tuple(q) for q in L.fin_hole_uz], L.fin_t + 2 * OVERLAP, ctx.CUT, lbl))
        ctx.report_extra["fin_holes"] = {"uz_mm": [list(q) for q in L.fin_hole_uz], "area_mm2": L.fin_hole_area_mm2,
                                         **freq.summary()}


def build_intake_module(ctx: "Ctx", cb: CB) -> None:
    """SPEC 5.1 (+ the v0.5 wire path when Layout.tabs: bulge, channel, shelf notch, v0.5d floor wedge,
    bulkhead wire hole - all CB.required)."""
    L = ctx.lay
    R, ri = L.body_r, L.body_id / 2.0
    so, si = L.spigot_od / 2.0, L.spigot_id / 2.0
    rb = L.r_band_in          # v0.6.1: the windowed band z_shelf_top .. bulkhead is window_wall_t thick (inward)
    pts = [(ri, L.z_intake_bottom), (R, L.z_intake_bottom), (R, L.z_intake_bulkhead_top), (so, L.z_intake_bulkhead_top),
           (so, L.z_intake_top), (si, L.z_intake_top), (si, L.z_intake_bulkhead_top), (0.0, L.z_intake_bulkhead_top),
           (0.0, L.z_intake_bulkhead_bottom), (rb, L.z_intake_bulkhead_bottom), (rb, L.z_shelf_top), (L.shelf_id / 2.0, L.z_shelf_top),
           (L.shelf_id / 2.0, L.z_lip_pocket_top), (L.lip_pocket_id / 2.0, L.z_lip_pocket_top), (L.lip_pocket_id / 2.0, L.z_intake_skirt_top),
           (ri, L.z_intake_skirt_top)]
    # (v0.5 tabs: the same profile gives the 12.5 mm lip pocket up to z_lip_pocket_top = z_edf_lip_top +
    # lip_stop_gap and the back-up shelf above it - every station is a Layout attribute)
    cb.revolve_z(pts, ctx.NEW, "intake_body")
    req = ReqGroup(cb) if L.tabs else None
    win = ReqGroup(cb)        # v0.6.1 band + windows: required for every preset
    try:
        _build_intake_rest(ctx, cb, req, win)
    finally:
        # features_ok counts features that built without error; a containment check the API could not answer is
        # listed separately (containment_unanswered) so it never reads as confirmed geometry
        win_unanswered = [x for x in cb.unverified if x == "intake_band" or x.startswith("window_")]
        ctx.report_extra["intake_windows"] = {
            "band_r_mm": [L.r_band_in, L.body_r], "band_z_mm": [L.z_shelf_top, L.z_intake_bulkhead_bottom],
            "window_wall_t_mm": L.window_wall_t, "window_sectors_deg": [list(s) for s in L.window_sectors],
            "window_z_mm": [L.z_window_bottom, L.z_window_top], "strut_thetas_deg": list(L.strut_thetas),
            "strut_w_mm": L.strut_w, "strut_w_inner_mm": L.strut_w_in,
            "inlet_area_mm2_min": L.inlet_area_mm2, "inlet_area_mm2_outer": L.inlet_area_outer_mm2,
            "inlet_area_ratio_min": L.inlet_area_ratio, "inlet_area_ratio_outer": L.inlet_area_ratio_outer,
            # v0.6.2 window lower end (truncated gable; None = square windows)
            "window_bottom_end": ({"slope_deg": L.gable_slope_deg, "bridge_w_mm": L.gable_bridge_w,
                                   "gable_h_inner_mm": L.gable_h_inner, "gable_h_outer_mm": L.gable_h_outer,
                                   "edge_slope_min_deg": L.gable_edge_slope_min_deg,
                                   "profile_yz_mm": [list(q) for q in L.window_gable_profile_yz],
                                   "gusset_volume_mm3_layout": L.gusset_volume_mm3}
                                  if getattr(L, "has_window_gable", False) else None),
            "containment_unanswered": win_unanswered, **win.summary()}
        ctx.log.info(f"{cb.name}: band + windows {win.n_ok}/{len(win.labels)} required features; point containment "
                     f"unanswered for {len(win_unanswered)}")
        if req is not None:
            ctx.report_extra["intake_wire"] = {"wire_theta_deg": L.wire_theta, "channel_z_mm": [L.wire_ch_z0, L.wire_ch_z1],
                                               "channel_r_mm": [L.wire_ch_r0, L.wire_ch_r1],
                                               "bulge_z_mm": [L.wire_bulge_z0, L.wire_bulge_z1], "bulge_r_out_mm": L.wire_bulge_r_out,
                                               "floor_wedge_rz_mm": [list(p) for p in (L.wire_ch_floor_rz or [])],
                                               "lip_pocket_id_mm": L.lip_pocket_id, **req.summary()}
            ctx.log.info(f"{cb.name}: wire path {req.n_ok}/{len(req.labels)} required features")


def gable_window_cut(ctx: "Ctx", cb: CB, lo: float, hi: float, th: float, label: str):
    """v0.6.2 window cut with the truncated-gable lower end: the window's radial sector (CB.sector_z, NEW body: r
    [r_band_in - OVERLAP, body_r + OVERLAP], Z [z_window_bottom, z_window_top]) is INTERSECTED with the gable prism - the
    hexagon Layout.window_gable_profile_yz sketched on the vertical plane through the axis perpendicular to the window
    centre (vplane(th + 90): it holds the tangential direction y' and Z; the 45-deg-family planes are the ones the webs
    already use), extruded symmetrically along the window centre direction over 2 x window_gable_u_half with
    participantBodies = [that tool body] - and the tool is then subtracted from the main body by a Combine/Cut that
    consumes it.  The tool is identified from its own feature, registered in ``cutters`` (combine_all never joins it)
    and deleted on any failure; the intersect must have removed the gussets from the tool (a point in a gusset leaves the
    tool, a point above the gable stays; volume drop when the API cannot answer).  Returns the Combine feature."""
    L = ctx.lay
    if cb.body is None:
        raise RuntimeError(f"{cb.name}:{label}: no main body (the window tool would become the part)")
    feat = cb.sector_z(lo, hi, L.r_band_in - OVERLAP, L.body_r + OVERLAP, L.z_window_bottom, L.z_window_top, ctx.NEW,
                       f"{label}_tool")
    tool = cb._new_body_of(feat, label)
    cb.cutters.append(tool)
    cb.bodies = [b for b in cb.bodies if not cb._same(b, tool)]
    try:
        if cb._same(tool, cb.body):
            raise RuntimeError("the window tool body is the main body")
        t = math.radians(th)
        ey = (-math.sin(t), math.cos(t))                           # y' (tangential) direction of this window
        rm = 0.5 * (L.r_band_in + L.body_r)
        ang = math.radians(0.5 * L.window_angle_deg - 2.0)          # 2 deg inside the strut side
        y_g = rm * math.sin(ang)
        z_g = L.gable_boundary_z(y_g)
        pg = (*pod_xy(th + math.degrees(ang), rm, 0.0), 0.5 * (L.z_window_bottom + z_g))     # in a gusset
        pa = (*pod_xy(th, rm, 0.0), 0.5 * (L.z_window_bottom + L.z_window_top))               # window centre, mid height
        # (th + 90) mod 360: on BUDGET exactly the 45 / 135 / 225 / 315 deg vplane angles the propulsion webs build on
        sk = cb.sketch(cb.vplane((th + 90.0) % 360.0), f"{label}_gable")
        sk.polygon([(y * ey[0], y * ey[1], z) for (y, z) in L.window_gable_profile_yz])
        cb.log.feat(cb.name, "gable-profile", label, theta=th, profile_yz=L.window_gable_profile_yz,
                    u_half=L.window_gable_u_half)
        vol0 = _num(getattr(tool, "volume", None), math.nan)
        before = (cb._contains(tool, pg), cb._contains(tool, pa))
        ifeat = cb.extrude(sk, ctx.INTERSECT, 2.0 * L.window_gable_u_half, f"{label}_gable", participants=[tool])
        err = cb._health_error(ifeat)
        if err is not None:
            raise RuntimeError(f"the gable intersect computed with an error: {err}")
        if not cb._alive(tool):
            raise RuntimeError("the gable intersect left no window tool body")
        after = (cb._contains(tool, pg), cb._contains(tool, pa))
        if before[0] == "in" and after[0] in ("in", "on"):
            raise RuntimeError(f"the gable intersect left the gusset point {_fmt(pg)} in the window tool")
        if after[1] in ("out", "on"):
            raise RuntimeError(f"the gable intersect removed the window centre {_fmt(pa)} from the tool")
        if "?" in before + after:
            dropped = (vol0 - _num(getattr(tool, "volume", None), math.nan)) * 1000.0
            if not (math.isfinite(dropped) and dropped > 1e-3):
                raise RuntimeError(f"point containment unanswered and the tool volume dropped {dropped:.4f} mm^3: no "
                                   f"evidence the gable intersect trimmed the window tool")
            cb.log.warn(f"{cb.name}:{label}: tool containment unanswered; gable intersect accepted on the tool's volume "
                        f"drop ({dropped:.3f} mm^3)")
        tools = OC()
        tools.add(tool)
        comb = cb.comp.features.combineFeatures
        inp = comb.createInput(cb.body, tools)
        inp.operation = ctx.CUT
        inp.isKeepToolBodies = False
        cb.log.feat(cb.name, "combine", label, op=cb.opname(ctx.CUT), tool="gable window")
        cfeat = comb.add(inp)
        cb.n_features += 1
        err = cb._health_error(cfeat)
        if err is not None:
            raise RuntimeError(f"the window Combine/Cut computed with an error: {err}")
        if cb._alive(tool):
            cb.log.warn(f"{cb.name}:{label}: the window tool is still listed after the Combine/Cut; deleting it")
            if not cb._delete_body(tool, label):
                raise RuntimeError("the leftover window tool body could not be removed")
    except Exception:
        if not cb._delete_body(tool, label):
            raise RuntimeError(f"{cb.name}:{label}: gable window cut failed AND its tool body could not be removed")
        raise
    return cfeat


def _build_intake_windows(ctx: "Ctx", cb: CB, win: "ReqGroup") -> None:
    """SPEC 5.1 v0.6.1: the band's thickness is confirmed by point containment (a point just inside r_band_in in each
    margin ring is material, a point just inside the bore is not), then one radial-sector window per
    Layout.window_sectors (CB.sector_z: horizontal annular-sector sketch, symmetric extrude - any angle, no tilted
    plane), r [r_band_in - OVERLAP, body_r + OVERLAP], Z [z_window_bottom, z_window_top], each CB.required and
    CB.cut_verified: a point in the band at the window centre leaves the body; the struts on both sides (at their inner
    and outer faces) and the margin rings below / above the window stay.  A point the API cannot answer is not
    evidence: it is logged as a WARN and listed in report ``intake_windows.containment_unanswered``.
    v0.6.2 (Layout.has_window_gable): each window is cut by gable_window_cut (sector INTERSECT gable prism); cut_verified
    also needs a point just above the bridge (window centre) and just above each gable face near the struts to leave the
    body, and a point inside each gusset (half its height, near the strut) and one just below the bridge to stay."""
    L = ctx.lay
    R, rb = L.body_r, L.r_band_in
    rm = 0.5 * (rb + R)
    zm = 0.5 * (L.z_window_bottom + L.z_window_top)
    z_lo = 0.5 * (L.z_shelf_top + L.z_window_bottom)
    z_hi = 0.5 * (L.z_window_top + L.z_intake_bulkhead_bottom)
    e = 0.25                                                       # mm inside the faces the points pin
    gable = bool(getattr(L, "has_window_gable", False))

    def gp(th: float, r: float, z: float) -> tuple[float, float, float]:
        x, y = pod_xy(th, r, 0.0)
        return (x, y, z)

    def band_points():
        th = L.window_thetas[0]
        if not cb.verify_points("intake_band", inside_mm=[gp(th, rb + e, z_lo), gp(th, R - e, z_lo), gp(th, rb + e, z_hi)],
                                outside_mm=[gp(th, rb - e, z_lo), gp(th, rb - e, z_hi)]):
            cb.unverified.append("intake_band")
            ctx.log.warn(f"{cb.name}:intake_band: point containment unanswered; the band thickness r "
                         f"{rb:g}..{R:g} mm is NOT confirmed (report intake_windows.containment_unanswered)")
        return None
    win("intake_band", band_points)
    hs = 0.5 * L.strut_angle_deg
    for (lo, hi), th in zip(L.window_sectors, L.window_thetas):
        lbl = f"window_{th:g}"
        gone = [gp(th, rm, zm), gp(th, rb + e, zm), gp(th, R - e, zm)]
        d_in, d_out = math.degrees(e / rb), math.degrees(e / R)
        kept = [gp(lo - hs, rm, zm), gp(hi + hs, rm, zm),                                   # strut centres
                gp(lo - d_in, rb + e, zm), gp(hi + d_in, rb + e, zm),                       # strut edges, inner face
                gp(lo - d_out, R - e, zm), gp(hi + d_out, R - e, zm),                       # strut edges, outer face
                gp(th, rm, z_lo), gp(th, rm, z_hi)]                                         # margin rings
        if gable:
            gone.append(gp(th, rm, L.z_window_bottom + e))                                  # just above the bridge
            kept.append(gp(th, rm, L.z_window_bottom - e))                                  # just below the bridge
            for sgn in (1.0, -1.0):
                dphi = 0.5 * L.window_angle_deg - 2.0                                       # 2 deg inside the strut side
                zb = L.gable_boundary_z(rm * math.sin(math.radians(dphi)))
                gone.append(gp(th + sgn * dphi, rm, zb + e))                                # just above the gable face
                kept.append(gp(th + sgn * dphi, rm, 0.5 * (L.z_window_bottom + zb)))        # inside the gusset

            def make_cut(lo=lo, hi=hi, th=th, lbl=lbl):
                return gable_window_cut(ctx, cb, lo, hi, th, lbl)
        else:
            def make_cut(lo=lo, hi=hi, lbl=lbl):
                return cb.sector_z(lo, hi, rb - OVERLAP, R + OVERLAP, L.z_window_bottom, L.z_window_top, ctx.CUT, lbl)
        win(lbl, lambda lbl=lbl, gone=gone, kept=kept, make_cut=make_cut: cb.cut_verified(lbl, make_cut, gone, kept))


def _build_intake_rest(ctx: "Ctx", cb: CB, req, win: "ReqGroup") -> None:
    """build_intake_module after the revolved body; ``req`` (ReqGroup) is given only for edf.mount = tabs, ``win``
    (v0.6.1 band + windows) always."""
    L = ctx.lay
    body = L.p["body"]
    R = L.body_r
    so = L.spigot_od / 2.0
    hw = L.wire_ch_w / 2.0 if L.tabs else 0.0
    if req is not None and getattr(L, "wire_ch_open", False):
        # v0.9 (intake wire_channel.open, rocket_geometry.intake_wire_bulge_parts / intake_wire_channel / intake_wire_notch /
        # intake_wire_cover_taps): side walls, roof wedge and cover-screw pads united first, then the open groove (to the
        # shelf top, out past the side walls), the gate (wire_gate_w, out through the band) and the four tap holes cut
        th, sw, ri = L.wire_theta, L.wire_side_wall, R - WIRE_BULGE_EMBED
        for s in (1.0, -1.0):
            v0, v1 = (hw, hw + sw) if s > 0 else (-hw - sw, -hw)
            req(f"groove_wall_{s:+.0f}", lambda v0=v0, v1=v1, s=s: cb.box_pod(th, ri, L.wire_ch_r1, v0, v1, L.z_intake_bottom,
                                                                             L.wire_ch_z1, ctx.JOIN, f"groove_wall_{s:+.0f}"))
        req("groove_roof", lambda: cb.prism(th, L.wire_roof_rz, 2.0 * (hw + sw), ctx.JOIN, "groove_roof"))
        for s in (1.0, -1.0):
            for z in L.wire_pad_zs:
                a, b = L.wire_pad_v
                v0, v1 = (a, b) if s > 0 else (-b, -a)
                req(f"groove_pad_{s:+.0f}_{z:.0f}", lambda v0=v0, v1=v1, z=z: cb.box_pod(
                    th, ri, L.wire_ch_r1, v0, v1, z - L.wire_pad_h / 2.0, z + L.wire_pad_h / 2.0, ctx.JOIN, "groove_pad"))
        # v0.9 fix round (rocket_geometry.tie_anchors): the motor-lead tie anchors on the 0 deg strut's inner face (united
        # with the walls / roof / pads, before every cut) - (u, z) profile = block + 45 deg roof toward the bulkhead
        for k_a, zc in enumerate(getattr(L, "ml_anchor_z", []) or []):
            u0a, u1a, ha = L.ml_anchor_u, L.ml_anchor_u1, L.ml_anchor_h
            rise = (u1a - u0a) * math.tan(math.radians(L.ml_anchor_wedge))
            pts_a = [(u0a, zc - ha / 2.0), (u1a, zc - ha / 2.0), (u1a, zc + ha / 2.0 + rise), (u0a, zc + ha / 2.0)]
            lbl = f"tie_anchor_{k_a + 1}"
            req(lbl, lambda pts_a=pts_a, lbl=lbl: cb.prism(th, pts_a, L.ml_anchor_w, ctx.JOIN, lbl))
        req("wire_groove", lambda: cb.box_pod(th, L.wire_ch_r0, L.wire_ch_r1 + OVERLAP, -hw, hw, L.wire_ch_z0 - OVERLAP,
                                              L.wire_groove_z1, ctx.CUT, "wire_groove"))
        # v0.9 fix round: the groove below the skirt top from the skirt bore's chord (no sliver), the lip-pocket wall slit,
        # the anchors' cable-tie tunnels (rocket_geometry.intake_wire_skirt_cut / intake_lip_slit / tie_anchor_tunnels)
        if getattr(L, "wire_skirt_cut_r0", L.wire_ch_r0) < L.wire_ch_r0 - 1e-9:
            req("wire_groove_skirt", lambda: cb.box_pod(th, L.wire_skirt_cut_r0, L.wire_ch_r1 + OVERLAP, -hw, hw,
                                                        L.wire_ch_z0 - OVERLAP, L.z_intake_skirt_top, ctx.CUT, "wire_groove_skirt"))
        if getattr(L, "wire_lip_slit_w", 0.0):
            hsl = L.wire_lip_slit_w / 2.0
            req("lip_slit", lambda: cb.box_pod(th, L.lip_pocket_id / 2.0 - 0.5, L.wire_ch_r0 + 0.5, -hsl, hsl,
                                               L.z_intake_skirt_top - 1.0, L.z_lip_pocket_top + OVERLAP, ctx.CUT, "lip_slit"))
        for k_a, zc in enumerate(getattr(L, "ml_anchor_z", []) or []):
            tt, th_t = L.ml_anchor_tunnel
            u0t = L.ml_anchor_u + L.ml_anchor_wall
            hwa = L.ml_anchor_w / 2.0 + OVERLAP
            lbl = f"tie_tunnel_{k_a + 1}"
            req(lbl, lambda zc=zc, u0t=u0t, tt=tt, th_t=th_t, hwa=hwa, lbl=lbl: cb.box_pod(
                th, u0t, u0t + tt, -hwa, hwa, zc - th_t / 2.0, zc + th_t / 2.0, ctx.CUT, lbl))
        gw = L.wire_gate_w / 2.0
        req("wire_gate", lambda: cb.box_pod(th, L.wire_notch_r0, L.wire_ch_r1 + OVERLAP, -gw, gw, L.wire_notch_z0,
                                            L.wire_ch_z1, ctx.CUT, "wire_gate"))
        for s in (1.0, -1.0):
            for z in L.wire_pad_zs:
                a = tuple(L.ml_xyz(L.wire_ch_r1 + OVERLAP, s * L.wire_screw_v, z))
                b = tuple(L.ml_xyz(L.wire_ch_r1 - L.wire_cov_tap_depth, s * L.wire_screw_v, z))
                req(f"groove_tap_{s:+.0f}_{z:.0f}", lambda a=a, b=b: cb.cut_between(a, b, L.wire_cov_d_tap, "groove_tap"))
    elif req is not None:
        # SPEC 5.1 v0.5 wire channel at wire_theta: the external bulge is united FIRST so the voids cut
        # after it leave its bottom, top and outer walls closed
        hb = L.wire_bulge_w / 2.0
        req("wire_bulge", lambda: cb.box_pod(L.wire_theta, R - WIRE_BULGE_EMBED, L.wire_bulge_r_out, -hb, hb,
                                             L.wire_bulge_z0, L.wire_bulge_z1, ctx.JOIN, "wire_bulge"))
        req("wire_channel", lambda: cb.box_pod(L.wire_theta, L.wire_ch_r0, L.wire_ch_r1, -hw, hw,
                                               L.wire_ch_z0, L.wire_ch_z1, ctx.CUT, "wire_channel"))
        # shelf notch: the leads step back in above the lip face (overlaps the channel radially)
        req("wire_shelf_notch", lambda: cb.box_pod(L.wire_theta, L.wire_notch_r0, L.wire_ch_r0 + WIRE_NOTCH_PAST_CH_R0, -hw, hw,
                                                   L.wire_notch_z0, L.wire_ch_z1, ctx.CUT, "wire_shelf_notch"))
        # v0.5d floor wedge (solid, unioned AFTER the channel / notch cuts): the (r, z) triangle wire_ch_floor_rz in
        # the radial plane at wire_theta, extruded symmetrically over the channel width (+-wire_ch_w/2), so the
        # channel floor rises at 45 deg from the skirt's inner face (the intake prints flipped: no support inside
        # the closed channel).  The triangle reaches 0.5 mm into the bulge's outer wall, so the union shares volume.
        req("wire_floor_wedge", lambda: cb.prism(L.wire_theta, L.wire_ch_floor_rz, L.wire_ch_w, ctx.JOIN, "wire_floor_wedge"))
    if req is not None and L.servo_leads:
        # SPEC 5.1 v0.6 servo channel at sl_gather_theta, the motor channel's pattern: bulge FIRST (its walls stay
        # closed), then the channel and shelf-notch voids (cut_verified: a point in the removed pocket wall / shelf
        # leaves the body, the bulge's outer wall, the pocket wall and the shelf beside the notch stay), then the 45 deg
        # floor wedge (verify_points: a point in the triangle is material).  The bulkhead servo hole is cut below.
        tg, hs, hb = L.sl_gather_theta, L.sl_ch_w / 2.0, L.sl_bulge_w / 2.0

        def gp(r: float, z: float, dth: float = 0.0) -> tuple[float, float, float]:
            x, y = pod_xy(tg + dth, r, 0.0)
            return (x, y, z)

        z_pk = 0.5 * (L.z_intake_skirt_top + L.z_lip_pocket_top)                 # beside the EDF bell
        z_sh = 0.5 * (L.z_lip_pocket_top + L.z_shelf_top)                        # in the shelf
        r_sh = 0.5 * (L.shelf_id / 2.0 + L.body_id / 2.0)
        dth_sh = math.degrees(math.atan2(hb + 2.0, r_sh))                          # beside the bulge
        req("servo_bulge", lambda: cb.box_pod(tg, R - WIRE_BULGE_EMBED, L.sl_bulge_r_out, -hb, hb, L.sl_bulge_z0,
                                              L.sl_bulge_z1, ctx.JOIN, "servo_bulge"))
        req("servo_channel", lambda: cb.cut_verified(
            "servo_channel", lambda: cb.box_pod(tg, L.sl_ch_r0, L.sl_ch_r1, -hs, hs, L.sl_ch_z0, L.sl_ch_z1, ctx.CUT,
                                                "servo_channel"),
            [gp(0.5 * (L.sl_ch_r0 + L.sl_ch_r1), z_pk)],
            [gp(0.5 * (L.sl_ch_r1 + L.sl_bulge_r_out), z_pk), gp(0.5 * (L.lip_pocket_id / 2.0 + L.sl_ch_r0), z_pk)]))
        req("servo_shelf_notch", lambda: cb.cut_verified(
            "servo_shelf_notch", lambda: cb.box_pod(tg, L.sl_notch_r0, L.sl_ch_r0 + WIRE_NOTCH_PAST_CH_R0, -hs, hs,
                                                    L.z_lip_pocket_top, L.sl_ch_z1, ctx.CUT, "servo_shelf_notch"),
            [gp(r_sh, z_sh)], [gp(r_sh, z_sh, dth_sh)]))
        (ra, za), (rb, _), (_, zc) = L.sl_ch_floor_rz
        wedge_pt = gp((2.0 * rb + ra) / 3.0, (2.0 * za + zc) / 3.0)                # centroid of the triangle

        def servo_wedge():
            feat = cb.prism(tg, L.sl_ch_floor_rz, L.sl_ch_w, ctx.JOIN, "servo_floor_wedge")
            if not cb.verify_points("servo_floor_wedge", inside_mm=[wedge_pt]):
                cb.unverified.append("servo_floor_wedge")
            return feat
        req("servo_floor_wedge", servo_wedge)
        ctx.report_extra["intake_servo_leads"] = {"gather_theta_deg": tg, "channel_r_mm": [L.sl_ch_r0, L.sl_ch_r1],
                                                  "channel_z_mm": [L.sl_ch_z0, L.sl_ch_z1], "bulge_r_out_mm": L.sl_bulge_r_out,
                                                  "floor_wedge_rz_mm": [list(p) for p in L.sl_ch_floor_rz],
                                                  "bulkhead_hole_r_mm": [L.sl_hole_r0, L.sl_hole_r1],
                                                  "containment_unanswered": [x for x in cb.unverified if x.startswith("servo_")]}
    # v0.6.1 windows: one radial-sector cut per window (struts keep strut_w at body_r with radial sides)
    _build_intake_windows(ctx, cb, win)
    # bulkhead holes
    zb0, zb1 = L.z_intake_bulkhead_bottom, L.z_intake_bulkhead_top
    cb.soft("cable_hole", lambda: cb.cyl_z(0.0, 0.0, zb0 - 1.0, zb1 + 1.0, L.cable_hole_d, ctx.CUT, "cable_hole"))
    for th in L.vent_hole_thetas:
        x, y = pod_xy(th, L.vent_hole_r, 0.0)
        cb.soft("vent_hole", lambda x=x, y=y, th=th: cb.cyl_z(x, y, zb0 - 1.0, zb1 + 1.0, L.vent_hole_d, ctx.CUT, f"vent_hole_{th:g}"))
    if req is not None:
        # SPEC 5.1 v0.5 bulkhead wire hole (box at wire_theta, r wire_hole_r0 .. r1) through the bulkhead
        hwh = getattr(L, "wire_hole_w", L.wire_ch_w) / 2.0        # v0.9 fix round: the hole's own width
        req("bulkhead_wire_hole", lambda: cb.box_pod(L.wire_theta, L.wire_hole_r0, L.wire_hole_r1, -hwh, hwh,
                                                     zb0 - OVERLAP, zb1 + OVERLAP, ctx.CUT, "bulkhead_wire_hole"))
    if req is not None and L.servo_leads:
        # SPEC 5.1 v0.6 bulkhead servo hole (box at sl_gather_theta, r sl_hole_r0 .. r1)
        hs, tg, zbm = L.sl_ch_w / 2.0, L.sl_gather_theta, 0.5 * (zb0 + zb1)
        r_h = 0.5 * (L.sl_hole_r0 + L.sl_hole_r1)
        dth = math.degrees(math.atan2(hs + 3.0, r_h))                              # bulkhead beside the hole
        req("bulkhead_servo_hole", lambda: cb.cut_verified(
            "bulkhead_servo_hole", lambda: cb.box_pod(tg, L.sl_hole_r0, L.sl_hole_r1, -hs, hs, zb0 - OVERLAP, zb1 + OVERLAP,
                                                      ctx.CUT, "bulkhead_servo_hole"),
            [(*pod_xy(tg, r_h, 0.0), zbm)], [(*pod_xy(tg + dth, r_h, 0.0), zbm)]))
    # joint screw holes: skirt (through, 3.2) and spigot (2.5)
    z_skirt = L.z_intake_bottom + L.joint_screw_dz
    z_spig = L.z_intake_spigot_bottom + L.joint_screw_dz
    for th in unique_mod180(L.joint_thetas):
        cb.soft("skirt_hole", lambda th=th: cb.radial_cyl(th, z_skirt, -(R + 1.0), R + 1.0, body["joint_hole_outer_d"], ctx.CUT, f"skirt_hole_{th:g}"))
        cb.soft("spigot_hole", lambda th=th: cb.radial_cyl(th, z_spig, -(so + 1.0), so + 1.0, body["joint_hole_spigot_d"], ctx.CUT, f"spigot_hole_{th:g}"))


def build_avionics_module(ctx: "Ctx", cb: CB) -> None:
    """SPEC 5.2: tube + spigot, two pairs of tray rails, (v0.5d, every preset) the tray stop in each rail slot,
    vent slots, joint holes."""
    L = ctx.lay
    body = L.p["body"]
    R, ri = L.body_r, L.body_id / 2.0
    so, si = L.spigot_od / 2.0, L.spigot_id / 2.0
    zsb = L.z_avionics_spigot_bottom
    sh_t, sh_r = shoulder_t(L), shoulder_r_in(L)
    pts = [(ri, L.z_avionics_bottom), (R, L.z_avionics_bottom), (R, zsb), (so, zsb),
           (so, L.z_avionics_spigot_top), (si, L.z_avionics_spigot_top), (si, zsb), (sh_r, zsb), (sh_r, zsb - sh_t), (ri, zsb - sh_t)]
    cb.revolve_z(pts, ctx.NEW, "avionics_tube")
    # tray rails: two ribs per wall point, embedded out to the tube OD so they always attach
    for dx in (-(L.rail_gap / 2.0 + L.rail_w / 2.0), (L.rail_gap / 2.0 + L.rail_w / 2.0)):
        xc = L.tray_x + dx
        for sgn in (1.0, -1.0):
            y_in, y_out = sgn * (L.rail_y - L.rail_depth), sgn * R
            cb.box_xy(xc - L.rail_w / 2.0, xc + L.rail_w / 2.0, min(y_in, y_out), max(y_in, y_out), L.z_rail_bottom, L.z_rail_top, ctx.JOIN, f"rail_x{xc:g}_y{sgn:+.0f}")
    # v0.5d tray stop (SPEC 5.2, every preset): in each rail slot a block between the two ribs of the pair - x from
    # the ribs' inner faces (tray_x +- rail_gap/2), the ribs' y extent (embedded out to y = +-R, trimmed at the OD
    # by rail_clip below, like the ribs), Z [z_tray_stop_bottom, z_tray_bottom] - so the tray rests at its design
    # height.  Same box helper as the ribs; CB.required (a missing stop lets the tray slide onto the bulkhead).
    if L.z_tray_stop_bottom is not None:
        if not (L.z_rail_bottom - 1.0 <= L.z_tray_stop_bottom < L.z_tray_bottom <= L.z_rail_top):
            raise RuntimeError(f"tray stop Z {L.z_tray_stop_bottom:g}..{L.z_tray_bottom:g} mm is outside the rail run "
                               f"{L.z_rail_bottom:g}..{L.z_rail_top:g} mm (rail_clip would not trim it)")
        sreq = ReqGroup(cb)
        x0, x1 = L.tray_x - L.rail_gap / 2.0, L.tray_x + L.rail_gap / 2.0
        for sgn in (1.0, -1.0):
            y_in, y_out = sgn * (L.rail_y - L.rail_depth), sgn * R
            lbl = f"tray_stop_y{sgn:+.0f}"
            sreq(lbl, lambda y_in=y_in, y_out=y_out, lbl=lbl: cb.box_xy(x0, x1, min(y_in, y_out), max(y_in, y_out),
                                                                       L.z_tray_stop_bottom, L.z_tray_bottom, ctx.JOIN, lbl))
        ctx.report_extra["tray_stop"] = {"x_mm": [x0, x1], "z_mm": [L.z_tray_stop_bottom, L.z_tray_bottom],
                                         "y_inner_mm": L.rail_y - L.rail_depth, **sreq.summary()}
    # the rail boxes (and tray stops) run out to y = +-R; trim everything outside the tube OD
    cb.revolve_z([(R, L.z_rail_bottom - 1.0), (R + 10.0, L.z_rail_bottom - 1.0), (R + 10.0, L.z_rail_top + 1.0), (R, L.z_rail_top + 1.0)], ctx.CUT, "rail_clip")
    # vent slots: radial through-cuts of the WALL only (SPEC 5.2, same as the CadQuery twin) - one box per slot in the
    # pod frame from the tube's inner face out past the OD.  (Before v0.5d one full-diameter slab per opposite pair
    # also cut 40 mm out of the tray-rail ribs at +-61 deg, found by the X3 centroid check.)
    for th in L.vent_slot_thetas:
        cb.soft("vent_slot", lambda th=th: cb.box_pod(th, L.body_id / 2.0 - 0.5, R + 1.0, -L.vent_slot_w / 2.0, L.vent_slot_w / 2.0,
                                                      L.z_vent_slot_bottom, L.z_vent_slot_bottom + L.vent_slot_h, ctx.CUT,
                                                      f"vent_slot_{th:g}"))
    z_skirt = L.z_avionics_bottom + L.joint_screw_dz
    z_spig = L.z_avionics_spigot_bottom + L.joint_screw_dz
    for th in unique_mod180(L.joint_thetas):
        cb.soft("skirt_hole", lambda th=th: cb.radial_cyl(th, z_skirt, -(R + 1.0), R + 1.0, body["joint_hole_outer_d"], ctx.CUT, f"skirt_hole_{th:g}"))
        cb.soft("spigot_hole", lambda th=th: cb.radial_cyl(th, z_spig, -(so + 1.0), so + 1.0, body["joint_hole_spigot_d"], ctx.CUT, f"spigot_hole_{th:g}"))
    if getattr(L, "has_gps_pod", False):
        # v0.8: the GPS lead's entry hole through the wall under the connector pocket (CB.required, CB.cut_verified: the
        # wall on the hole axis leaves the body, the wall beside the hole stays)
        req = ReqGroup(cb)
        th, d, z = L.gps_entry_theta, L.gps_entry_d, L.gps_entry_z
        r_mid = 0.5 * (ri + R)
        gx, gy = pod_xy(th, r_mid, 0.0)
        kx, ky = pod_xy(th, r_mid, 0.5 * d + 1.0)
        req("gps_entry_hole", lambda: cb.cut_verified(
            "gps_entry_hole", lambda: cb.radial_cyl(th, z, ri - 2.0 * OVERLAP, R + 2.0 * OVERLAP, d, ctx.CUT, "gps_entry_hole"),
            [(gx, gy, z)], [(kx, ky, z)]))
        ctx.report_extra["gps_entry_hole"] = {"theta_deg": th, "d_mm": d, "z_mm": z,
                                              "containment_unanswered": [x for x in cb.unverified if x.startswith("gps")],
                                              **req.summary()}
    if getattr(L, "has_gns", False):
        build_gns_mount(ctx, cb)
    if getattr(L, "has_av_windows", False):
        # v0.9 CAD-2 VTOL: hexagonal windows (Layout.avw_vz, 45 deg pointed top / bottom) cut radially through the wall
        # at avw_thetas - CB.tprism over u [body_id/2 - OVERLAP, body_r + OVERLAP] (rocket_geometry.avionics_window_cutters)
        wreq = ReqGroup(cb)
        u0, u1 = ri - OVERLAP, R + OVERLAP
        for th in L.avw_thetas:
            lbl = f"av_window_{th:g}"
            wreq(lbl, lambda th=th, lbl=lbl: cb.tprism(th, 0.5 * (u0 + u1), [tuple(q) for q in L.avw_vz], u1 - u0, ctx.CUT, lbl))
        ctx.report_extra["avionics_windows"] = {"thetas_deg": list(L.avw_thetas), "vz_mm": [list(q) for q in L.avw_vz],
                                                **wreq.summary()}
    if getattr(L, "has_livery", False):
        # 0.9.6 livery (user 2026-11-04): the 0.42 mm logo pocket of the WHITE body, cut last
        # (rocket_geometry.avionics_module with_livery: livery_inlay_solids out to body_r + OVERLAP)
        build_livery(ctx, cb, L.livery_r + OVERLAP, ctx.CUT, "logo_pocket")


def build_livery(ctx: "Ctx", cb: CB, r_out: float, op, tag: str) -> None:
    """0.9.6 livery: every island of every copy through CB.inlay (op CUT: the white body's pocket, r_out = body_r +
    OVERLAP; op NEW: the orange inlay part, r_out = body_r).  All CB.required; report derived.extra.<tag>."""
    L = ctx.lay
    req = ReqGroup(cb)
    u0, u1 = L.livery_u
    for th in L.livery_thetas:
        for k, (outer, holes) in enumerate(L.livery_polys_vz):
            lbl = f"{tag}_{th:g}_{k}"
            rings = [list(outer)] + [list(h) for h in holes]
            req(lbl, lambda th=th, rings=rings, lbl=lbl: cb.inlay(th, rings, L.livery_r_in, r_out, L.livery_sector[th],
                                                               0.5 * (u0 + u1), u1 - u0, op, lbl))
    ctx.report_extra[tag] = {"thetas_deg": list(L.livery_thetas), "islands": len(L.livery_polys_vz),
                             "depth_mm": L.livery_depth, "r_mm": [L.livery_r_in, r_out], "logo_file": L.livery_file,
                             "logo_file_sha256": L.livery_file_sha256, **req.summary()}


def build_avionics_logo(ctx: "Ctx", cb: CB) -> None:
    """0.9.6 livery: the orange inlay part (rocket_geometry.avionics_logo) - one NEW body per island and copy, each the
    annular sector r [livery_r_in, body_r] INTERSECT the island outline; it fills the white body's pocket exactly."""
    build_livery(ctx, cb, ctx.lay.livery_r, ctx.NEW, "logo_inlay")


def build_gns_mount(ctx: "Ctx", cb: CB) -> None:
    """v0.8.1 GNS-board bosses on the avionics tube wall (Layout._gns; rocket_geometry.gns_bosses / gns_hole_cutters),
    applied to the finished avionics module: per boss a diamond (Layout.gns_diamond_vz, faces at 45 deg) extruded radially
    from u gns_u_back to body_r + OVERLAP (CB.tprism, JOIN); CUT everything outside the OD over the bosses' Z range
    (gns_clip, as the rail clip); per boss a blind tap hole from OVERLAP inside the bay to gns_hole_u[1] (CB.cut_between
    with a probe point on its axis), then a point beyond the hole end must still be material.  Every feature is
    CB.required; each boss is confirmed by a point in it next to its hole."""
    L = ctx.lay
    th, R = L.gns_theta, L.body_r
    req = ReqGroup(cb)
    u0, u1 = L.gns_u_back, R + OVERLAP

    def P_(u: float, v: float, z: float) -> tuple[float, float, float]:
        x, y = pod_xy(th, u, v)
        return (x, y, z)
    for k, (vh, zh) in enumerate(L.gns_hole_vz):
        lbl = f"gns_boss_{k + 1}"
        inside = P_(u0 + 0.5, vh, zh + 0.5 * (L.gns_hole_d / 2 + L.gns_boss_hd / math.sqrt(2.0)))   # boss wall beside the hole

        def boss(k=k, lbl=lbl, inside=inside):
            feat = cb.tprism(th, 0.5 * (u0 + u1), L.gns_diamond_vz(k), u1 - u0, ctx.JOIN, lbl)
            if not cb.verify_points(lbl, inside_mm=[inside]):
                cb.unverified.append(lbl)
            return feat
        req(lbl, boss)
    z0, z1 = L.gns_boss_z[0] - OVERLAP, L.gns_boss_z[1] + OVERLAP
    req("gns_clip", lambda: cb.revolve_z([(R, z0), (R + 10.0, z0), (R + 10.0, z1), (R, z1)], ctx.CUT, "gns_clip"))
    for k, (vh, zh) in enumerate(L.gns_hole_vz):
        lbl = f"gns_hole_{k + 1}"
        a, b = P_(L.gns_u_back - OVERLAP, vh, zh), P_(L.gns_hole_u[1], vh, zh)
        probe = P_(0.5 * (L.gns_hole_u[0] + L.gns_hole_u[1]), vh, zh)
        beyond = P_(L.gns_hole_u[1] + 0.5, vh, zh)

        def hole(a=a, b=b, lbl=lbl, probe=probe, beyond=beyond):
            length = cb.cut_between(a, b, L.gns_hole_d, lbl, probe_mm=probe)
            if not cb.verify_points(lbl, inside_mm=[beyond]):
                cb.unverified.append(lbl)
            return length
        req(lbl, hole)
    ctx.report_extra["gns_mount"] = {
        "theta_deg": th, "hole_vz_mm": [list(q) for q in L.gns_hole_vz], "boss_half_diag_mm": L.gns_boss_hd,
        "boss_u_mm": [u0, R], "hole_u_mm": list(L.gns_hole_u), "hole_d_mm": L.gns_hole_d,
        "board_box_mm": [list(L.gns_x), list(L.gns_y), list(L.gns_z)],
        "containment_unanswered": [x for x in cb.unverified if x.startswith("gns")], **req.summary()}
    ctx.log.info(f"{cb.name}: GNS mount {req.n_ok}/{len(req.labels)} required features")


def build_avionics_tray(ctx: "Ctx", cb: CB) -> None:
    L = ctx.lay
    am = L.p["avionics_module"]
    cb.box_xy(L.tray_x - L.tray_t / 2.0, L.tray_x + L.tray_t / 2.0, -L.tray_w / 2.0, L.tray_w / 2.0, L.z_tray_bottom, L.z_tray_top, ctx.NEW, "tray_plate")
    hp = L.fc_hole_pattern / 2.0
    sw, sh = am["strap_slot_w"], am["strap_slot_h"]
    esc_mid = (L.esc_z[0] + L.esc_z[1]) / 2.0
    slots = [(40.0, L.battery_z[0] + 25.0), (-40.0, L.battery_z[0] + 25.0), (40.0, L.battery_z[1] - 25.0), (-40.0, L.battery_z[1] - 25.0),
             (35.0, esc_mid), (-35.0, esc_mid)]

    def draw(sk: Sk, t):
        if getattr(L, "tray_nav_holes", True):            # v0.9 CAD-2: no NAV board / holes in the VTOL configuration
            for sy in (-1.0, 1.0):
                for sz in (-1.0, 1.0):
                    yc, zc = sy * hp, L.nav_center_z + sz * hp     # v0.6.3: the NAV board at the v0.5 FC spot
                    sk.circle((yc * t[0], yc * t[1], zc), L.fc_hole_d)
        if getattr(L, "has_tray_holes", False):           # v0.9 CAD-2 VTOL: two tall lightening holes
            (yi, yo), (hz0, hz1) = L.tray_hole_y, L.tray_hole_z
            for a, b in ((yi, yo), (-yo, -yi)):
                sk.polygon([(a * t[0], a * t[1], hz0), (b * t[0], b * t[1], hz0), (b * t[0], b * t[1], hz1), (a * t[0], a * t[1], hz1)])
        for (yc, zc) in slots:
            # SPEC 5.2: a horizontal strap round the Z-long battery -> strap width (sw) along Z, thickness (sh) along y
            sk.polygon([((yc - sh / 2) * t[0], (yc - sh / 2) * t[1], zc - sw / 2), ((yc + sh / 2) * t[0], (yc + sh / 2) * t[1], zc - sw / 2),
                        ((yc + sh / 2) * t[0], (yc + sh / 2) * t[1], zc + sw / 2), ((yc - sh / 2) * t[0], (yc - sh / 2) * t[1], zc + sw / 2)])
    # all holes/slots go through the plate along X (normal of the tray)
    cb.soft("tray_cuts", lambda: cb.through_cut(0.0, draw, 2.0 * (L.tray_x + 10.0), "fc_holes_and_strap_slots"))


def build_com_mount(ctx: "Ctx", cb: CB) -> None:
    """v0.6 SPEC 5.3: four COM-board bosses under the payload bulkhead (vertical cylinders on horizontal offset-plane
    sketches, joined OVERLAP into the bulkhead so they share volume with it) and a blind tap hole in each (cut from
    OVERLAP below the boss tip - air - up to z_com_hole_end).  Every feature is CB.required; a boss is confirmed by a
    point in its wall, a hole by CB.cut_verified (axis point leaves the body, floor point above the hole end stays)."""
    L = ctx.lay
    req = ReqGroup(cb)
    r_wall = 0.25 * (L.com_boss_d + L.com_hole_d)           # mid-wall radius of the boss around its hole
    for (x, y) in L.com_boss_xy:
        lbl = f"com_boss_{x:+g}_{y:+g}"
        wall_pt = (x + r_wall, y, L.z_com_boss_tip + 0.5 * L.com_boss_h)

        def boss(x=x, y=y, lbl=lbl, wall_pt=wall_pt):
            feat = cb.cyl_z(x, y, L.z_com_boss_tip, L.z_com_boss_top + OVERLAP, L.com_boss_d, ctx.JOIN, lbl)
            if not cb.verify_points(lbl, inside_mm=[wall_pt]):
                cb.unverified.append(lbl)
            return feat
        req(lbl, boss)
    for (x, y) in L.com_boss_xy:
        lbl = f"com_hole_{x:+g}_{y:+g}"
        gone = [(x, y, L.z_com_boss_tip + 0.5 * L.com_hole_depth)]                     # on the axis, mid-hole
        kept = [(x, y, 0.5 * (L.z_com_hole_end + L.z_payload_bulkhead_top))]          # floor above the hole end
        req(lbl, lambda x=x, y=y, lbl=lbl, gone=gone, kept=kept: cb.cut_verified(
            lbl, lambda: cb.cyl_z(x, y, L.z_com_boss_tip - OVERLAP, L.z_com_hole_end, L.com_hole_d, ctx.CUT, lbl), gone, kept))
    ctx.report_extra["com_mount"] = {
        "boss_xy_mm": [list(b) for b in L.com_boss_xy], "boss_z_mm": [L.z_com_boss_tip, L.z_com_boss_top],
        "boss_d_mm": L.com_boss_d, "hole_d_mm": L.com_hole_d, "hole_end_z_mm": L.z_com_hole_end,
        "hole_skin_mm": L.com_hole_skin, "containment_unanswered": list(cb.unverified), **req.summary()}
    ctx.log.info(f"{cb.name}: COM mount {req.n_ok}/{len(req.labels)} required features; point containment "
                 f"unanswered for {len(cb.unverified)}")


def gps_corner_pts(L, sign: float) -> list[tuple[float, float]]:
    """v0.8: plan triangle (pod frame) outside the 45 deg chamfer of the GPS pod's outer vertical edge on side ``sign``
    (+1 / -1), extended 1 mm past both faces (the CadQuery twin's gps_corner_pts)."""
    u1, w, c = L.gps_u_out, L.gps_w_h, L.gps_corner_c
    return [(u1 - c - 1.0, sign * (w + 1.0)), (u1 + 1.0, sign * (w + 1.0)), (u1 + 1.0, sign * (w - c - 1.0))]


def build_gps_pod(ctx: "Ctx", cb: CB) -> None:
    """v0.8 GPS pod on the payload tube (Layout._gps_pod), applied to the finished payload module in the CadQuery twin's
    order (rocket_geometry.gps_pod_features): JOIN the outer box; CUT the cavity ((u, z) prism above the floor's top line,
    |v| <= cav_hw); JOIN the two corner bosses; CUT everything below the floor's underside line; CUT the two corner
    chamfers; JOIN the tube-wall sector back (the cavity and underside cuts opened it); CUT the payload bore over the pod
    (the outer box and the floor reach into it); CUT the two blind lid-screw holes; JOIN the raceway and connector-pocket
    boxes; CUT their voids.  Every feature is CB.required; the holes and voids go through CB.cut_verified and the shell
    is confirmed by points in its walls, floor, bosses and the restored tube (CB.verify_points)."""
    L = ctx.lay
    th = L.gps_theta
    req = ReqGroup(cb)
    J, C = ctx.JOIN, ctx.CUT

    def P_(u: float, v: float, z: float) -> tuple[float, float, float]:
        x, y = pod_xy(th, u, v)
        return (x, y, z)
    u0 = L.gps_u_back - 1.0
    cav = [(u0, L.gps_floor_top_z(u0)), (L.gps_u_cav_out, L.gps_z_floor_top_out), (L.gps_u_cav_out, L.gps_cut_z[1]),
           (u0, L.gps_cut_z[1])]
    ua, ub = L.gps_u_back - 1.0, L.gps_u_out + 1.0
    under = [(ua, L.gps_floor_under_z(ua)), (ub, L.gps_floor_under_z(ub)), (ub, L.gps_cut_z[0]), (ua, L.gps_cut_z[0])]
    (bu0, bu1), (bv0, bv1) = L.gps_boss_u, L.gps_boss_v
    a = L.gps_restore_half_deg
    r_in = L.body_id / 2.0
    req("gps_pod_outer", lambda: cb.box_pod(th, L.gps_u_back, L.gps_u_out, -L.gps_w_h, L.gps_w_h, L.gps_z_lo, L.gps_z_rim, J,
                                            "gps_pod_outer"))
    req("gps_pod_cavity", lambda: cb.prism(th, cav, 2.0 * L.gps_cav_hw, C, "gps_pod_cavity"))
    for lo, hi, nm in ((bv0, bv1, "gps_boss_pos"), (-bv1, -bv0, "gps_boss_neg")):
        req(nm, lambda lo=lo, hi=hi, nm=nm: cb.box_pod(th, bu0, bu1, lo, hi, L.gps_boss_z0, L.gps_z_rim, J, nm))
    req("gps_pod_underside", lambda: cb.prism(th, under, 2.0 * (L.gps_w_h + 1.0), C, "gps_pod_underside"))
    for s, nm in ((1.0, "gps_corner_pos"), (-1.0, "gps_corner_neg")):
        req(nm, lambda s=s, nm=nm: cb.poly_pod(th, gps_corner_pts(L, s), L.gps_cut_z[0], L.gps_cut_z[1], C, nm))
    req("gps_tube_restore", lambda: cb.sector_z(th - a, th + a, r_in, L.body_r, L.gps_cut_z[0], L.gps_cut_z[1], J,
                                                "gps_tube_restore"))
    req("gps_bore_recut", lambda: cb.revolve_z([(0.0, L.gps_cut_z[0]), (r_in, L.gps_cut_z[0]), (r_in, L.gps_cut_z[1]),
                                                (0.0, L.gps_cut_z[1])], C, "gps_bore_recut"))
    for k, (u, v) in enumerate(L.gps_boss_hole_uv):
        lbl = f"gps_boss_hole_{k + 1}"
        x, y = pod_xy(th, u, v)
        gone = [(x, y, 0.5 * (L.gps_boss_hole_z0 + L.gps_z_rim))]
        kept = [(x, y, L.gps_boss_hole_z0 - 0.5)]                               # boss material under the hole's end
        req(lbl, lambda x=x, y=y, lbl=lbl, gone=gone, kept=kept: cb.cut_verified(
            lbl, lambda: cb.cyl_z(x, y, L.gps_boss_hole_z0, L.gps_z_rim + OVERLAP, L.gps_boss_hole_d, C, lbl), gone, kept))
    (su0, su1), shv = L.gps_rw_solid_u, L.gps_rw_solid_hv
    (pu0, pu1), phv = L.gps_pk_solid_u, L.gps_pk_solid_hv
    req("gps_raceway_solid", lambda: cb.box_pod(th, su0, su1, -shv, shv, L.gps_rw_z[0], L.gps_rw_z[1], J, "gps_raceway_solid"))
    req("gps_pocket_solid", lambda: cb.box_pod(th, pu0, pu1, -phv, phv, L.gps_pk_solid_z[0], L.gps_pk_solid_z[1], J,
                                               "gps_pocket_solid"))
    (vu0, vu1), vhv = L.gps_rw_void_u, L.gps_rw_void_hv
    (ku0, ku1), khv = L.gps_pk_void_u, L.gps_pk_void_hv
    z_mid_rw = 0.5 * (L.gps_pk_solid_z[1] + L.gps_floor_under_z(su1))
    req("gps_raceway_void", lambda: cb.cut_verified(
        "gps_raceway_void", lambda: cb.box_pod(th, vu0, vu1, -vhv, vhv, L.gps_rw_void_z[0], L.gps_rw_void_z[1], C,
                                               "gps_raceway_void"),
        [P_(0.5 * (vu0 + vu1), 0.0, z_mid_rw)], [P_(0.5 * (vu1 + su1), 0.0, z_mid_rw)]))
    req("gps_pocket_void", lambda: cb.cut_verified(
        "gps_pocket_void", lambda: cb.box_pod(th, ku0, ku1, -khv, khv, L.gps_pk_z[0] - OVERLAP, L.gps_pk_z[1], C,
                                              "gps_pocket_void"),
        [P_(0.5 * (ku0 + ku1), 0.0, 0.5 * (L.gps_pk_z[0] + L.gps_pk_z[1]))],
        [P_(0.5 * (ku1 + pu1), 0.0, 0.5 * (L.gps_pk_z[0] + L.gps_pk_z[1]))]))
    # positive evidence of the shell: outer wall, side wall, floor, a boss, the restored tube wall; the cavity, the space
    # under the floor and the bore over the pod are empty
    z_w = L.gps_z_rim - 2.0
    inside = [P_(L.gps_u_out - 0.5 * L.gps_wall_t, 0.0, z_w), P_(L.gps_u_c, L.gps_cav_hw + 0.5 * L.gps_wall_t, z_w),
              P_(L.gps_u_c, 0.0, L.gps_floor_top_z(L.gps_u_c) - 0.5 * L.gps_floor_dz),
              P_(0.5 * (bu0 + bu1), 0.5 * (bv0 + bv1), L.gps_z_rim - 0.5),
              P_(0.5 * (r_in + L.body_r), 0.0, 0.5 * (L.gps_floor_top_z(L.body_r) + L.gps_z_rim))]
    outside = [P_(L.gps_u_c, 0.0, 0.5 * (L.gps_z_gp_top + L.gps_z_rim)),
               P_(L.gps_u_c, 0.0, L.gps_floor_under_z(L.gps_u_c) - 1.0),
               P_(r_in - 1.0, 0.0, 0.5 * (L.gps_floor_top_z(L.body_r) + L.gps_z_rim))]

    def shell_points():
        if not cb.verify_points("gps_pod_shell", inside_mm=inside, outside_mm=outside):
            cb.unverified.append("gps_pod_shell")
    req("gps_pod_shell_points", shell_points)
    ctx.report_extra["gps_pod"] = {
        "theta_deg": th, "u_mm": [L.gps_u_back, L.gps_u_out], "half_width_mm": L.gps_w_h, "z_mm": [L.gps_z_foot, L.gps_z_rim],
        "lid_top_z_mm": L.gps_z_lid_top, "antenna_phase_centre_mm": list(L.gps_antenna_xyz),
        "raceway_z_mm": list(L.gps_rw_z), "pocket_z_mm": list(L.gps_pk_z),
        "containment_unanswered": [x for x in cb.unverified if x.startswith("gps")], **req.summary()}
    ctx.log.info(f"{cb.name}: GPS pod {req.n_ok}/{len(req.labels)} required features")


def build_gps_pod_lid(ctx: "Ctx", cb: CB) -> None:
    """v0.8 printed part gps_pod_lid (rocket_geometry.gps_pod_lid): plate flush with the pod, inner edge the arc r =
    gps_lid_r_in, corner chamfers, three posts that carry the ground plane (blind holes from below), two lid-screw
    clearance holes.  Every feature is CB.required; the holes go through CB.cut_verified."""
    L = ctx.lay
    th = L.gps_theta
    req = ReqGroup(cb)
    J, C = ctx.JOIN, ctx.CUT
    z0, z1 = L.gps_z_rim, L.gps_z_lid_top
    cb.box_pod(th, L.gps_lid_u_back - 1.0, L.gps_u_out, -L.gps_w_h, L.gps_w_h, z0, z1, ctx.NEW, "gps_lid_plate")
    req("gps_lid_tube_arc", lambda: cb.revolve_z([(0.0, z0 - OVERLAP), (L.gps_lid_r_in, z0 - OVERLAP),
                                                  (L.gps_lid_r_in, z1 + OVERLAP), (0.0, z1 + OVERLAP)], C, "gps_lid_tube_arc"))
    for s, nm in ((1.0, "gps_lid_corner_pos"), (-1.0, "gps_lid_corner_neg")):
        req(nm, lambda s=s, nm=nm: cb.poly_pod(th, gps_corner_pts(L, s), z0 - OVERLAP, z1 + OVERLAP, C, nm))
    for k, (u, v) in enumerate(L.gps_post_uv):
        x, y = pod_xy(th, u, v)
        lbl = f"gps_lid_post_{k + 1}"
        req(lbl, lambda x=x, y=y, lbl=lbl: cb.cyl_z(x, y, L.gps_post_z[0], L.gps_post_z[1] + 0.5 * L.gps_lid_t,
                                                    L.gps_post_d, J, lbl))
    r_wall = 0.25 * (L.gps_post_d + L.gps_post_hole_d)
    for k, (u, v) in enumerate(L.gps_post_uv):
        x, y = pod_xy(th, u, v)
        lbl = f"gps_lid_post_hole_{k + 1}"
        gone = [(x, y, 0.5 * (L.gps_post_z[0] + L.gps_post_hole_z1))]
        kept = [(x, y, 0.5 * (L.gps_post_hole_z1 + z1)), (x + r_wall, y, 0.5 * (L.gps_post_z[0] + L.gps_post_hole_z1))]
        req(lbl, lambda x=x, y=y, lbl=lbl, gone=gone, kept=kept: cb.cut_verified(
            lbl, lambda: cb.cyl_z(x, y, L.gps_post_z[0] - OVERLAP, L.gps_post_hole_z1, L.gps_post_hole_d, C, lbl), gone, kept))
    for k, (u, v) in enumerate(L.gps_boss_hole_uv):
        x, y = pod_xy(th, u, v)
        lbl = f"gps_lid_screw_hole_{k + 1}"
        req(lbl, lambda x=x, y=y, lbl=lbl: cb.cut_verified(
            lbl, lambda: cb.cyl_z(x, y, z0 - OVERLAP, z1 + OVERLAP, L.gps_lid_screw_d, C, lbl), [(x, y, 0.5 * (z0 + z1))]))
    ctx.report_extra["gps_pod_lid"] = {"posts_uv_mm": [list(q) for q in L.gps_post_uv], "screws_uv_mm": [list(q) for q in L.gps_boss_hole_uv],
                                       "z_mm": [L.gps_post_z[0], z1],
                                       "containment_unanswered": [x for x in cb.unverified if x.startswith("gps")], **req.summary()}


def build_gps_module(ctx: "Ctx", cb: CB) -> None:
    """v0.8 placeholder ``gps``: the patch on the ground plane and the board under it, two bodies in one component
    (Layout.gps_patch_box / gps_board_box, pod frame; mass electronics.gps.board_mass_g spread over both by volume)."""
    L = ctx.lay
    th = L.gps_theta
    for (u, v, z), nm in ((L.gps_patch_box, "gps_patch"), (L.gps_board_box, "gps_board")):
        cb.box_pod(th, u[0], u[1], v[0], v[1], z[0], z[1], ctx.NEW, nm)


def build_gps_ground_plane(ctx: "Ctx", cb: CB) -> None:
    """v0.8 placeholder ``gps_ground_plane``: the FR4 plate (Layout.gps_gp_box) minus the two outer corner notches."""
    L = ctx.lay
    th = L.gps_theta
    (u0, u1), (v0, v1), (z0, z1) = L.gps_gp_box
    n = L.gps_gp_notch
    cb.box_pod(th, u0, u1, v0, v1, z0, z1, ctx.NEW, "gps_ground_plane")
    for a, b, nm in ((v1 - n, v1 + OVERLAP, "gps_gp_notch_pos"), (v0 - OVERLAP, v0 + n, "gps_gp_notch_neg")):
        cb.box_pod(th, u1 - n, u1 + OVERLAP, a, b, z0 - OVERLAP, z1 + OVERLAP, ctx.CUT, nm)


def build_top_cap(ctx: "Ctx", cb: CB) -> None:
    """v0.9 CAD-2 VTOL top cap (Layout._top_cap; rocket_geometry.top_cap): skirt (body_id/2..body_r over the avionics
    spigot, into the plate by OVERLAP), plate (full disc, flat top), hub / rim ring / radial ribs under it reaching half the
    plate thickness into it; then the hardpoint hole (axis), the plate holes (Layout.cap_holes) and one skirt joint hole per
    joint theta.  Every feature CB.required."""
    L = ctx.lay
    R, ri = L.body_r, L.body_id / 2.0
    zp, zt = L.z_cap_plate_bottom, L.z_cap_top
    zin = zp + 0.5 * L.cap_plate_t
    req = ReqGroup(cb)
    req("cap_skirt", lambda: cb.revolve_z([(ri, L.z_cap_skirt_bottom), (R, L.z_cap_skirt_bottom), (R, zp + OVERLAP), (ri, zp + OVERLAP)],
                                          ctx.NEW, "cap_skirt"))
    req("cap_plate", lambda: cb.cyl_z(0.0, 0.0, zp, zt, 2.0 * R, ctx.JOIN, "cap_plate"))
    req("cap_hub", lambda: cb.cyl_z(0.0, 0.0, L.z_cap_under, zin, L.cap_hub_d, ctx.JOIN, "cap_hub"))
    r0, r1 = L.cap_ring_r
    req("cap_ring", lambda: cb.revolve_z([(r0, L.z_cap_ring_bottom), (r1, L.z_cap_ring_bottom), (r1, zin), (r0, zin)],
                                         ctx.JOIN, "cap_ring"))
    for th in L.cap_rib_thetas:
        lbl = f"cap_rib_{th:g}"
        req(lbl, lambda th=th, lbl=lbl: cb.box_pod(th, L.cap_rib_r[0], L.cap_rib_r[1], -L.cap_rib_t / 2.0, L.cap_rib_t / 2.0,
                                                   L.z_cap_under, zin, ctx.JOIN, lbl))
    req("cap_hardpoint_hole", lambda: cb.cyl_z(0.0, 0.0, L.z_cap_under - OVERLAP, zt + OVERLAP, L.cap_hardpoint_d, ctx.CUT,
                                               "cap_hardpoint_hole"))
    if getattr(L, "cap_hardpoint", False):     # v0.9 fix round: the load washer's flush recess in the hub / rib bottoms
        req("cap_washer_recess", lambda: cb.cyl_z(0.0, 0.0, L.z_cap_under - OVERLAP, L.z_cap_under + L.cap_washer_recess[1],
                                                  L.cap_washer_recess[0], ctx.CUT, "cap_washer_recess"))
    for nm, (x, y, d) in L.cap_holes.items():
        lbl = "cap_hole_" + sanitize_param_name(nm)
        req(lbl, lambda x=x, y=y, d=d, lbl=lbl: cb.cyl_z(x, y, zp - OVERLAP, zt + OVERLAP, d, ctx.CUT, lbl))
    zj = L.z_cap_skirt_bottom + L.joint_screw_dz
    for th in L.joint_thetas:
        lbl = f"cap_skirt_hole_{th:g}"
        req(lbl, lambda th=th, lbl=lbl: cb.radial_cyl(th, zj, ri - 2.0 * OVERLAP, R + 2.0 * OVERLAP,
                                                      L.p["body"]["joint_hole_outer_d"], ctx.CUT, lbl))
    ctx.report_extra["top_cap"] = {"z_mm": [L.z_cap_skirt_bottom, zp, zt], "holes": {k: list(v) for k, v in L.cap_holes.items()},
                                   "tether_mm": dict(L.cap_tether), **req.summary()}


def build_cap_gps_module(ctx: "Ctx", cb: CB) -> None:
    """v0.9 CAD-2 VTOL placeholder ``gps``: the patch on the ground plane and the board taped on the cap top (two bodies)."""
    L = ctx.lay
    (px, py), (bx, by) = L.cap_patch_xy, L.cap_board_xy
    cb.box_xy(px[0], px[1], py[0], py[1], L.cap_patch_z[0], L.cap_patch_z[1], ctx.NEW, "gps_patch")
    cb.box_xy(bx[0], bx[1], by[0], by[1], L.cap_board_z[0], L.cap_board_z[1], ctx.NEW, "gps_board")


def build_payload_module(ctx: "Ctx", cb: CB) -> None:
    L = ctx.lay
    pl = L.p["payload_module"]
    body = L.p["body"]
    R, ri = L.body_r, L.body_id / 2.0
    so, si = L.spigot_od / 2.0, L.spigot_id / 2.0
    boss_r = L.spring_seat_boss_d / 2.0
    boss_top = L.z_payload_bulkhead_top + pl["spring_seat_boss_h"]
    zsb = L.z_payload_spigot_bottom
    sh_t, sh_r = shoulder_t(L), shoulder_r_in(L)
    pts = [(ri, L.z_payload_bottom), (R, L.z_payload_bottom), (R, zsb), (so, zsb),
           (so, L.z_payload_spigot_top), (si, L.z_payload_spigot_top), (si, zsb), (sh_r, zsb), (sh_r, zsb - sh_t), (ri, zsb - sh_t),
           (ri, L.z_payload_bulkhead_top), (boss_r, L.z_payload_bulkhead_top), (boss_r, boss_top), (0.0, boss_top),
           (0.0, L.z_payload_bulkhead_bottom), (ri, L.z_payload_bulkhead_bottom)]
    cb.revolve_z(pts, ctx.NEW, "payload_tube_bulkhead")
    # bulkhead holes (SPEC 5.3: theta = 45 + k*90)
    n_h = int(pl["bulkhead_hole_count"])
    for k in range(n_h):
        th = 45.0 + k * 360.0 / n_h
        x, y = pod_xy(th, pl["bulkhead_hole_r"], 0.0)
        cb.soft("bulkhead_hole", lambda x=x, y=y, th=th: cb.cyl_z(x, y, L.z_payload_bulkhead_bottom - 1.0, L.z_payload_bulkhead_top + 1.0, pl["bulkhead_hole_d"], ctx.CUT, f"bulkhead_hole_{th:g}"))
    if L.has_com_board:
        build_com_mount(ctx, cb)
    z_skirt = L.z_payload_bottom + L.joint_screw_dz
    for th in unique_mod180(L.joint_thetas):
        cb.soft("skirt_hole", lambda th=th: cb.radial_cyl(th, z_skirt, -(R + 1.0), R + 1.0, body["joint_hole_outer_d"], ctx.CUT, f"skirt_hole_{th:g}"))
    # release servo pod on the body tube
    tan_ch = math.tan(math.radians(L.p["pod"]["underside_chamfer_deg"]))
    build_pod(cb, L.release_pod_theta, L.body_r, L.release_pod_r_in, L.release_pod_r_out, L.pod_w, L.release_pod_z0, L.release_pod_rim_z,
              L.release_pod_cavity_r_in, L.release_pod_cavity_r_out, L.pod_cavity_w, L.release_pod_floor_top_z, L.release_servo_axis_r, tan_ch, L.p["servo"], "release_pod")
    if getattr(L, "has_gps_pod", False):   # v0.8: the GPS pod, raceway and connector pocket, last
        build_gps_pod(ctx, cb)


def build_piston(ctx: "Ctx", cb: CB) -> None:
    L = ctx.lay
    pl = L.p["payload_module"]
    zb, zt = L.z_piston_bottom, L.z_piston_top
    rr, rd = pl["piston_recess_d"] / 2.0, pl["piston_recess_depth"]
    cb.revolve_z([(0.0, zb + rd), (rr, zb + rd), (rr, zb), (L.piston_d / 2.0, zb), (L.piston_d / 2.0, zt), (0.0, zt)], ctx.NEW, "piston")


def build_nose_cone(ctx: "Ctx", cb: CB) -> None:
    L = ctx.lay
    no = L.p["nose"]
    R = L.body_r
    outer = L.nose_profile_points(24)  # (r, z_global) base -> tip
    # Haack body (solid), then shell with the base face removed
    sk = cb.sketch(cb.vplane(0.0), "nose_outer")
    sk.spline_closed([(r, 0.0, z) for (r, z) in outer], [(0.0, 0.0, L.z_nose_base)])
    ctx.log.feat(cb.name, "revolve_z-spline", "nose_outer", pts_rz=outer)
    cb.revolve(sk, cb.comp.zConstructionAxis, ctx.NEW, "nose_solid")
    v_solid = _num(cb.body.volume) * 1000.0 if cb.body is not None else 0.0
    shelled = False
    face = cb.planar_face_at_z(cb.body, L.z_nose_base) if cb.body is not None else None
    if face is not None:
        try:
            cb.shell_remove_face(face, L.nose_wall, "nose_shell")
            v_after = _num(cb.body.volume) * 1000.0
            if v_after < 0.9 * v_solid and v_after > 0.0:
                shelled = True
            else:
                ctx.log.warn(f"nose_cone: shell volume check failed ({v_solid:.0f} -> {v_after:.0f} mm3)")
        except Exception as exc:  # noqa: BLE001
            ctx.log.warn(f"nose_cone: ShellFeature failed ({exc!r}); using inner-profile cut")
    else:
        ctx.log.warn("nose_cone: base face not found; using inner-profile cut")
    if not shelled:
        # inner profile r_in(x) = r_out(x) - wall for x <= L - 8, solid near the tip (SPEC 5.4)
        x_end = L.nose_len - 8.0
        n = 20
        inner = []
        for i in range(n + 1):
            x = x_end * i / n
            inner.append((max(L.nose_outer_radius(x) - L.nose_wall, 0.0), L.z_nose_base + x))
        sk2 = cb.sketch(cb.vplane(0.0), "nose_inner")
        sk2.spline_closed([(r, 0.0, z) for (r, z) in inner], [(0.0, 0.0, L.z_nose_base + x_end), (0.0, 0.0, L.z_nose_base - 1.0), (inner[0][0], 0.0, L.z_nose_base - 1.0)])
        ctx.log.feat(cb.name, "revolve_z-spline", "nose_inner_cut", pts_rz=inner)
        cb.revolve(sk2, cb.comp.zConstructionAxis, ctx.CUT, "nose_inner_cut")
    ctx.report_extra["nose_shell_method"] = "ShellFeature" if shelled else "inner_profile_cut"
    # skirt
    cb.revolve_z([(L.nose_skirt_id / 2.0, L.z_nose_skirt_bottom), (R, L.z_nose_skirt_bottom), (R, L.z_nose_skirt_top), (L.nose_skirt_id / 2.0, L.z_nose_skirt_top)], ctx.JOIN, "nose_skirt")
    # latch lug on the skirt exterior
    cb.box_pod(L.latch_lug_theta, L.lug_r_in - 1.0, L.lug_r_out, -L.lug_w / 2.0, L.lug_w / 2.0, L.z_lug_bottom, L.z_lug_top, ctx.JOIN, "latch_lug")
    # anchor bar across the bore along X, with the tether eye
    z0b = L.z_anchor_bar
    z1b = z0b + no["anchor_bar_h"]
    r_in_top = L.nose_outer_radius(z1b - L.z_nose_base) - L.nose_wall
    hl = r_in_top + 1.0
    cb.box_xy(-hl, hl, -no["anchor_bar_w"] / 2.0, no["anchor_bar_w"] / 2.0, z0b, z1b, ctx.JOIN, "anchor_bar")
    cb.soft("tether_eye", lambda: cb.cyl_z(0.0, 0.0, z0b - 1.0, z1b + 1.0, no["tether_eye_d"], ctx.CUT, "tether_eye"))
    ctx.log.info(f"nose_cone: tip blunt r={no['tip_blunt_r']} mm not applied (optional in SPEC 5.4)")
    n_bodies = cb.combine_all("nose_combine")
    if n_bodies > 1:
        ctx.log.warn(f"nose_cone: {n_bodies} bodies remain after combine")


def build_vane(ctx: "Ctx", cb: CB, theta: float) -> None:
    L = ctx.lay
    v = L.p["vane"]
    cb.prism(theta, [(L.vane_r_in, L.vane_te_z), (L.vane_r_in, L.vane_le_z), (L.vane_r_out, L.vane_le_z), (L.vane_r_out, L.vane_te_z)], L.vane_t, ctx.NEW, "vane_plate")
    # LE / TE rounds (long edges only)
    def do_fillet():
        edges = cb.edges_at_z(cb.body, L.vane_le_z, 10.0) + cb.edges_at_z(cb.body, L.vane_te_z, 10.0)
        if len(edges) < 2:
            raise RuntimeError(f"found {len(edges)} LE/TE edges")
        cb.fillet(edges, v["edge_radius"], "vane_edge_rounds")
    cb.soft("vane_edge_rounds", do_fillet)
    # integral shaft on the hinge axis
    cb.radial_cyl(theta, L.vane_hinge_z, L.vane_shaft_r_in, L.vane_shaft_r_out, L.vane_shaft_d, ctx.JOIN, "vane_shaft")
    # D-flat for r >= d_flat_r_in (v0.9: Layout.vane_d_flat_r_in, derived from the coupler in direct drive); on the -Z side,
    # v0.9 fix round (vane_drive.flat_side '+v'): on the +v (service) side at neutral, for the coupler's set screw
    flat_off = L.vane_shaft_d / 2.0 - v["d_flat_depth"]
    fr = getattr(L, "vane_d_flat_r_in", v["d_flat_r_in"])
    if getattr(L, "dd_flat_side", "-z") == "+v":
        cb.soft("vane_d_flat", lambda: cb.box_pod(theta, fr, L.vane_shaft_r_out + 1.0, flat_off, L.vane_shaft_d / 2.0 + 1.0,
                                                  L.vane_hinge_z - L.vane_shaft_d / 2.0 - 1.0,
                                                  L.vane_hinge_z + L.vane_shaft_d / 2.0 + 1.0, ctx.CUT, "vane_d_flat"))
    else:
        flat_z = L.vane_hinge_z - flat_off
        cb.soft("vane_d_flat", lambda: cb.box_pod(theta, fr, L.vane_shaft_r_out + 1.0, -(L.vane_shaft_d / 2.0 + 1.0), (L.vane_shaft_d / 2.0 + 1.0),
                                                  L.vane_hinge_z - L.vane_shaft_d / 2.0 - 1.0, flat_z, ctx.CUT, "vane_d_flat"))


def build_vane_coupler(ctx: "Ctx", cb: CB, theta: float) -> None:
    """v0.9 printed vane_coupler (rocket_geometry.vane_coupler): hub O dd_hub_d + flange O dd_flange_d on the hinge axis,
    the D-bore (O dd_bore_d, flat dd_bore_flat below the axis) through both."""
    L = ctx.lay
    zh = L.vane_hinge_z
    cb.radial_cyl(theta, zh, L.dd_u_hub0, L.dd_u_hub1, L.dd_hub_d, ctx.NEW, "coupler_hub")
    cb.radial_cyl(theta, zh, L.dd_u_hub1, L.dd_u_flange1, L.dd_flange_d, ctx.JOIN, "coupler_flange")
    if getattr(L, "dd_flat_side", "-z") != "+v":
        cb.soft("coupler_d_bore", lambda: cb.through_circle(theta, zh, L.dd_bore_d, 2.0 * (L.dd_u_flange1 + 2.0),
                                                            "coupler_d_bore", flat_z=zh - L.dd_bore_flat))
        return
    # v0.9 fix round (rocket_geometry.vane_coupler): the round bore through, the flat restored on the +v side from the hub
    # end to dd_u_flat_end (just past the shaft end - beyond it the bore stays round for the horn screw head), the socket
    # slot across the flange (open along v), the relief for the horn's round hub, the set screw's tap hole along -v
    req = ReqGroup(cb)
    req("coupler_bore", lambda: cb.radial_cyl(theta, zh, L.dd_u_hub0 - OVERLAP, L.dd_u_flange1 + OVERLAP, L.dd_bore_d,
                                              ctx.CUT, "coupler_bore"))
    req("coupler_flat", lambda: cb.box_pod(theta, L.dd_u_hub0, L.dd_u_flat_end, L.dd_bore_flat, L.dd_bore_d / 2.0 + 0.1,
                                           zh - L.dd_bore_d / 2.0 - 0.1, zh + L.dd_bore_d / 2.0 + 0.1, ctx.JOIN, "coupler_flat"))
    req("coupler_socket_slot", lambda: cb.box_pod(theta, L.dd_u_hub1, L.dd_u_flange1 + OVERLAP, -L.dd_flange_d, L.dd_flange_d,
                                                  zh - L.dd_slot_hw, zh + L.dd_slot_hw, ctx.CUT, "coupler_socket_slot"))
    req("coupler_hub_relief", lambda: cb.radial_cyl(theta, zh, L.dd_u_hub1, L.dd_u_flange1 + OVERLAP, L.dd_collar_relief_d,
                                                    ctx.CUT, "coupler_hub_relief"))
    a = (*pod_xy(theta, L.dd_set_screw_u, 0.0), zh)
    b = (*pod_xy(theta, L.dd_set_screw_u, L.dd_hub_d / 2.0 + OVERLAP), zh)
    req("coupler_set_screw_tap", lambda: cb.cut_between(a, b, L.dd_set_screw_d_tap, "coupler_set_screw_tap"))
    ctx.report_extra.setdefault("vane_couplers", {})[f"{theta:g}"] = req.summary()


def build_servo_cover(k: int):
    def fn(ctx: "Ctx", cb: CB) -> None:
        """v0.9 printed servo_cover of fin k (rocket_geometry.servo_cover): plate + the two clearance holes."""
        L = ctx.lay
        th = L.fin_thetas[k]
        cb.box_pod(th, L.cov_u[0], L.cov_u[1], L.cov_v[0], L.cov_v[1], L.cov_z[0], L.cov_z[1], ctx.NEW, "servo_cover")
        for j, (u, z) in enumerate(L.cov_screws_uz):
            a, b = fin_pt(L, k, u, L.cov_v[0] - OVERLAP, z), fin_pt(L, k, u, L.cov_v[1] + OVERLAP, z)
            cb.soft(f"cover_hole_{j}", lambda a=a, b=b, j=j: cb.cut_between(a, b, L.cov_d_clear, f"cover_hole_{j}"))
    return fn


def build_wire_cover(ctx: "Ctx", cb: CB) -> None:
    """v0.9 printed wire_cover (rocket_geometry.wire_cover): plate on the groove's side walls + four clearance holes."""
    L = ctx.lay
    th = L.wire_theta
    cb.box_pod(th, L.wire_cov_u[0], L.wire_cov_u[1], -L.wire_cov_hv, L.wire_cov_hv, L.wire_cov_z[0], L.wire_cov_z[1], ctx.NEW,
               "wire_cover")
    for s in (1.0, -1.0):
        for z in L.wire_pad_zs:
            v = s * L.wire_screw_v
            a = tuple(L.ml_xyz(L.wire_cov_u[0] - OVERLAP, v, z))
            b = tuple(L.ml_xyz(L.wire_cov_u[1] + OVERLAP, v, z))
            cb.soft("wire_cover_hole", lambda a=a, b=b: cb.cut_between(a, b, L.wire_cov_d_clear, "wire_cover_hole"))


def build_servo_direct(k: int):
    def fn(ctx: "Ctx", cb: CB) -> None:
        """v0.9 placeholder servo_vane_k in the fairing (rocket_geometry.servo_direct): case, ears, output boss."""
        L = ctx.lay
        th, w = L.fin_thetas[k], L.dd_case_w / 2.0
        cb.box_pod(th, L.dd_u_case_top, L.dd_u_case_bot, -w, w, L.dd_z_case[0], L.dd_z_case[1], ctx.NEW, "servo_case")
        cb.box_pod(th, L.dd_u_ear[0], L.dd_u_ear[1], -w, w, L.dd_z_ear[0], L.dd_z_ear[1], ctx.JOIN, "servo_ears")
        # v0.9 fix round: the output boss starts where the horn's spline collar ends
        cb.radial_cyl(th, L.vane_hinge_z, getattr(L, "dd_u_collar1", L.dd_u_horn1), L.dd_u_case_top + OVERLAP, L.dd_boss_d,
                      ctx.JOIN, "servo_boss")
    return fn


def build_servo_horn(k: int):
    def fn(ctx: "Ctx", cb: CB) -> None:
        """v0.9 placeholder servo_horn_k.  Fix round (rocket_geometry.servo_horn): the stock double-arm horn cut to
        dd_horn_arm_len - arm bar along v at neutral, the round hub / spline collar, the horn screw head."""
        L = ctx.lay
        th, zh = L.fin_thetas[k], L.vane_hinge_z
        if not hasattr(L, "dd_u_arm0"):
            cb.radial_cyl(th, zh, L.dd_u_flange1, L.dd_u_horn1, L.dd_horn_d, ctx.NEW, "servo_horn")
            return
        hl, hw = L.dd_horn_arm_len / 2.0, L.dd_horn_arm_w / 2.0
        cb.box_pod(th, L.dd_u_arm0, L.dd_u_horn1, -hl, hl, zh - hw, zh + hw, ctx.NEW, "horn_arms")
        cb.radial_cyl(th, zh, L.dd_u_arm0, L.dd_u_collar1, L.dd_horn_collar_d, ctx.JOIN, "horn_hub")
        cb.radial_cyl(th, zh, L.dd_u_screw0, L.dd_u_arm0 + OVERLAP, L.dd_horn_screw_d, ctx.JOIN, "horn_screw_head")
    return fn


def build_hinge_bushing(k: int):
    def fn(ctx: "Ctx", cb: CB) -> None:
        """v0.9 fix round placeholder hinge_bushing_k (rocket_geometry.hinge_bushing): the brass tube in the hinge bore."""
        L = ctx.lay
        th, zh = L.vane_thetas[k], L.vane_hinge_z
        cb.radial_cyl(th, zh, L.dd_bush_u[0], L.dd_bush_u[1], L.dd_bush_od, ctx.NEW, "bushing")
        cb.radial_cyl(th, zh, L.dd_bush_u[0] - OVERLAP, L.dd_bush_u[1] + OVERLAP, L.dd_bush_id, ctx.CUT, "bushing_bore")
    return fn


def build_tether_eyebolt(ctx: "Ctx", cb: CB) -> None:
    """v0.9 fix round placeholder tether_eyebolt (rocket_geometry.tether_eyebolt): M5 shank from the cap top down
    cap_bolt_len, the eye above the cap (a D cap_eye_od x 4 mm ring on a stem)."""
    L = ctx.lay
    zt, r = L.z_cap_top, L.cap_eye_od / 2.0
    ze = zt + L.cap_eye_h - r
    cb.cyl_z(0.0, 0.0, L.z_bolt_end, zt + OVERLAP, L.cap_bolt_d, ctx.NEW, "eyebolt_shank")
    cb.radial_cyl(90.0, ze, -2.0, 2.0, L.cap_eye_od, ctx.JOIN, "eyebolt_eye")
    cb.box_xy(-0.5 * L.cap_bolt_d - 1.0, 0.5 * L.cap_bolt_d + 1.0, -2.0, 2.0, zt, ze, ctx.JOIN, "eyebolt_stem")
    cb.radial_cyl(90.0, ze, -2.0 - OVERLAP, 2.0 + OVERLAP, L.cap_eye_od - 8.0, ctx.CUT, "eyebolt_eye_hole")


def build_tether_load_washer(ctx: "Ctx", cb: CB) -> None:
    """v0.9 fix round placeholder tether_load_washer: D cap_washer_d x cap_washer_t ring in the hub recess."""
    L = ctx.lay
    cb.revolve_z([(L.cap_hardpoint_d / 2.0, L.z_cap_washer[0]), (L.cap_washer_d / 2.0, L.z_cap_washer[0]),
                  (L.cap_washer_d / 2.0, L.z_cap_washer[1]), (L.cap_hardpoint_d / 2.0, L.z_cap_washer[1])], ctx.NEW, "load_washer")


def build_tether_nut(ctx: "Ctx", cb: CB) -> None:
    """v0.9 fix round placeholder tether_nut: the M5 jam nut (hexagon cap_nut_af across flats) with its hole."""
    L = ctx.lay
    rc = L.cap_nut_af / math.sqrt(3.0)
    pts = [(rc * math.cos(math.radians(60.0 * i)), rc * math.sin(math.radians(60.0 * i))) for i in range(6)]
    cb.poly_pod(0.0, pts, L.z_cap_nut[0], L.z_cap_nut[1], ctx.NEW, "jam_nut")
    cb.cyl_z(0.0, 0.0, L.z_cap_nut[0] - OVERLAP, L.z_cap_nut[1] + OVERLAP, L.cap_bolt_d + 0.1, ctx.CUT, "jam_nut_hole")


def build_vane_horn(ctx: "Ctx", cb: CB, theta: float) -> None:
    L = ctx.lay
    v = L.p["vane"]
    cb.prism(theta, [(L.horn_r_in, L.horn_tail_z), (L.horn_r_in, L.horn_tip_z), (L.horn_r_out, L.horn_tip_z), (L.horn_r_out, L.horn_tail_z)], v["horn_w"], ctx.NEW, "horn_plate")
    # D-bore matching the shaft flat (flat at shaft_d/2 - d_flat_depth below the axis)
    flat_z = L.vane_hinge_z - (L.vane_shaft_d / 2.0 - v["d_flat_depth"]) - (L.hinge_bore_d - L.vane_shaft_d) / 2.0
    cb.soft("horn_d_bore", lambda: cb.through_circle(theta, L.vane_hinge_z, L.hinge_bore_d, 2.0 * (L.horn_r_out + 2.0), "horn_d_bore", flat_z=flat_z))
    # pushrod clevis pin hole, radial, at horn_tip_z - 2
    cb.soft("horn_pin_hole", lambda: cb.radial_cyl(theta, L.horn_tip_z - 2.0, L.horn_r_in - 1.0, L.horn_r_out + 1.0, v["horn_pin_hole_d"], ctx.CUT, "horn_pin_hole"))


def build_latch_arm(ctx: "Ctx", cb: CB) -> None:
    L = ctx.lay
    pl = L.p["payload_module"]
    ax, ay = pod_xy(L.release_pod_theta, L.release_servo_axis_r, L.servo_shaft_tangential_offset)
    lug_r = (L.lug_r_in + L.lug_r_out) / 2.0
    lx, ly = pod_xy(L.latch_lug_theta, lug_r, 0.0)
    dx, dy = lx - ax, ly - ay
    d = math.hypot(dx, dy) or 1.0
    ux, uy = dx / d, dy / d
    px, py = -uy, ux
    z0, z1 = L.z_latch_arm_bottom, L.z_latch_arm_top
    w = L.latch_arm_w / 2.0
    cb.cyl_z(ax, ay, z0, z1, 6.0, ctx.NEW, "latch_hub")  # SPEC 5.3: O6 hub at the servo axis
    tip_x, tip_y = ax + ux * L.latch_arm_len, ay + uy * L.latch_arm_len
    zm = (z0 + z1) / 2.0
    sk = cb.sketch(cb.hplane(zm), "latch_arm_plate")
    sk.polygon([(ax - px * w, ay - py * w, zm), (tip_x - px * w, tip_y - py * w, zm), (tip_x + px * w, tip_y + py * w, zm), (ax + px * w, ay + py * w, zm)])
    ctx.log.feat(cb.name, "latch_arm-profile", "latch_arm_plate", axis=(ax, ay), tip=(tip_x, tip_y), lug_centre=(lx, ly), len=L.latch_arm_len)
    cb.extrude(sk, ctx.JOIN, z1 - z0, "latch_arm_plate")
    ctx.report_extra["latch_arm"] = {"servo_axis_xy_mm": [ax, ay], "lug_centre_xy_mm": [lx, ly], "axis_to_lug_mm": d,
                                     "tip_xy_mm": [tip_x, tip_y], "released_rotation_deg": L.p["mission"]["deploy_servo_travel_deg"]}


def build_launch_pad(ctx: "Ctx", cb: CB) -> None:
    L = ctx.lay
    lp = L.p["launch_pad"]
    t = L.pad_plate_t
    cb.revolve_z([(L.pad_plate_id / 2.0, -t), (L.pad_plate_od / 2.0, -t), (L.pad_plate_od / 2.0, 0.0), (L.pad_plate_id / 2.0, 0.0)], ctx.NEW, "pad_plate")
    s, fs = lp["leg_size"], lp["foot_size"]
    z_leg_bottom = -(t + L.pad_leg_h + lp["foot_t"])
    for th in L.pad_leg_thetas:
        cx, cy = pod_xy(th, lp["leg_r"], 0.0)
        cb.box_xy(cx - s / 2.0, cx + s / 2.0, cy - s / 2.0, cy + s / 2.0, z_leg_bottom, -t + 1.0, ctx.JOIN, f"leg_{th:g}")
        cb.box_xy(cx - fs / 2.0, cx + fs / 2.0, cy - fs / 2.0, cy + fs / 2.0, z_leg_bottom, -(t + L.pad_leg_h), ctx.JOIN, f"foot_{th:g}")
    rw, rh, rl, rr = lp["fin_locator_rib_w"], lp["fin_locator_rib_h"], lp["fin_locator_rib_len"], lp["fin_locator_rib_r_in"]
    off = lp["fin_locator_gap"] / 2.0 + rw / 2.0
    for th in L.fin_thetas:
        for sgn in (-1.0, 1.0):
            cb.soft("fin_rib", lambda th=th, sgn=sgn: cb.box_pod(th, rr, rr + rl, sgn * off - rw / 2.0, sgn * off + rw / 2.0, -1.0, rh, ctx.JOIN, f"fin_rib_{th:g}{sgn:+.0f}"))


# ---- placeholders (SPEC section 6) ----
def build_edf_unit(ctx: "Ctx", cb: CB) -> None:
    """SPEC 6 edf_unit: lip + housing ring + hub; v0.5 tabs: the lip is a ring (bell envelope), the two tab
    plates (SPEC 3.7, three slots each) are joined to the housing, and the motor cylinder is its own body
    (face contact with the hub only, so the volume equals the twin's fused hub + motor)."""
    L = ctx.lay
    e = L.p["edf"]
    hi, ho, lr = e["housing_id"] / 2.0, e["housing_od"] / 2.0, e["lip_od"] / 2.0
    zh, zl0, zl1 = L.z_edf_housing_bottom, L.z_edf_lip_bottom, L.z_edf_lip_top
    if L.tabs:
        cb.revolve_z([(hi, zh), (ho, zh), (ho, zl0), (lr, zl0), (lr, zl1), (hi, zl1)], ctx.NEW, "edf_housing_and_lip")
    else:
        cb.revolve_z([(hi, zh), (ho, zh), (ho, zl0), (lr, zl0), (lr, zl1), (0.0, zl1), (0.0, zl0), (hi, zl0)], ctx.NEW, "edf_housing_and_lip")
    hub_len = EDF_HUB_LEN  # SPEC section 6 table: hub from housing bottom to +70 mm
    cb.revolve_z([(0.0, zh), (e["hub_d_ASSUMED"] / 2.0, zh), (e["hub_d_ASSUMED"] / 2.0, zh + hub_len), (0.0, zh + hub_len)], ctx.NEW, "edf_hub")
    if L.tabs:
        rm, zm0 = e["motor_d"] / 2.0, zh - e["motor_protrusion_rear"]
        cb.revolve_z([(0.0, zm0), (rm, zm0), (rm, zh), (0.0, zh)], ctx.NEW, "edf_motor")
        hs, hw = L.tab_slot_len_radial / 2.0, L.tab_slot_w / 2.0
        r_in = L.tab_r_in - EDF_TAB_EMBED
        outer = [(r_in, L.tab_z0), (L.tab_r_out, L.tab_z0), (L.tab_r_out, L.tab_z1), (r_in, L.tab_z1)]
        slots = [[(L.tab_slot_r - hs, zs - hw), (L.tab_slot_r + hs, zs - hw), (L.tab_slot_r + hs, zs + hw), (L.tab_slot_r - hs, zs + hw)]
                 for zs in L.tab_slot_zs]
        for th in L.tab_thetas:
            cb.plate(th, outer, slots, L.tab_t, ctx.JOIN, f"edf_tab_{th:g}")


def build_box_placeholder(xr, yr, zr):
    def fn(ctx: "Ctx", cb: CB) -> None:
        cb.box_xy(xr[0], xr[1], yr[0], yr[1], zr[0], zr[1], ctx.NEW, cb.name)
    return fn


def build_misc_electronics(ctx: "Ctx", cb: CB) -> None:
    L = ctx.lay
    # SPEC section 6: box at Layout.misc_x/y/z (tray -x face above the battery)
    cb.box_xy(L.misc_x[0], L.misc_x[1], L.misc_y[0], L.misc_y[1], L.misc_z[0], L.misc_z[1], ctx.NEW, "misc_electronics")


def build_servo_vane(theta: float):
    def fn(ctx: "Ctx", cb: CB) -> None:
        L = ctx.lay
        build_servo(cb, theta, L.servo_axis_r, L.servo_bottom_z, L.pod_rim_z, L.servo_arm_z_bottom)
    return fn


def build_servo_release(ctx: "Ctx", cb: CB) -> None:
    L = ctx.lay
    build_servo(cb, L.release_pod_theta, L.release_servo_axis_r, L.release_servo_bottom_z, L.release_pod_rim_z, L.z_latch_arm_bottom)


def build_servo_arm(theta: float):
    def fn(ctx: "Ctx", cb: CB) -> None:
        L = ctx.lay
        pod = L.p["pod"]
        v = L.servo_shaft_tangential_offset
        cb.box_pod(theta, L.servo_axis_r, L.servo_arm_tip_r, v - pod["arm_w"] / 2.0, v + pod["arm_w"] / 2.0, L.servo_arm_z_bottom, L.servo_arm_z_top, ctx.NEW, "servo_arm")
    return fn


def build_pushrod(theta_pod: float, theta_vane: float):
    def fn(ctx: "Ctx", cb: CB) -> None:
        L = ctx.lay
        ax, ay = pod_xy(theta_pod, L.servo_arm_tip_r, L.servo_shaft_tangential_offset)
        az = (L.servo_arm_z_bottom + L.servo_arm_z_top) / 2.0
        bx, by = pod_xy(theta_vane, (L.horn_r_in + L.horn_r_out) / 2.0, 0.0)
        bz = L.horn_tip_z - 2.0
        length = cb.cyl_between((ax, ay, az), (bx, by, bz), L.p["pod"]["pushrod_d"], ctx.NEW, "pushrod")
        ctx.report_extra.setdefault("pushrod_len_mm_each", []).append(length)
    return fn


def build_cansat_payload(ctx: "Ctx", cb: CB) -> None:
    L = ctx.lay
    # layout.py overwrites z_chute_pack_top with the nose chute pack; recompute the can's chute block top
    z_top = L.z_can_top + L.p["payload"]["chute_pack_h"]
    cb.revolve_z([(0.0, L.z_can_bottom), (L.can_d / 2.0, L.z_can_bottom), (L.can_d / 2.0, z_top), (0.0, z_top)], ctx.NEW, "cansat_can_and_chute_block")


def build_spring(ctx: "Ctx", cb: CB) -> None:
    L = ctx.lay
    pl = L.p["payload_module"]
    ro = pl["spring_od"] / 2.0
    cb.revolve_z([(ro - pl["spring_wire_d"], L.z_spring_bottom), (ro, L.z_spring_bottom), (ro, L.z_spring_top), (ro - pl["spring_wire_d"], L.z_spring_top)], ctx.NEW, "spring_tube")


def build_chute_pack(ctx: "Ctx", cb: CB) -> None:
    L = ctx.lay
    cb.cyl_z(0.0, 0.0, L.z_chute_pack_bottom, L.z_chute_pack_top, L.chute_pack_d, ctx.NEW, "chute_pack")


# --------------------------------------------------------------------------- #
# run context: design, components, mass properties, exports, report
# --------------------------------------------------------------------------- #
class Ctx:
    def __init__(self, app, ui, log: RunLog, root_dir: str, export_root: str, lay, preset: str | None, direct_mode: bool):
        self.app, self.ui, self.log = app, ui, log
        self.root_dir, self.export_root = root_dir, export_root
        self.fusion_dir = os.path.join(export_root, "fusion" + config_file_suffix(lay))   # v0.9 CAD-2: fusion_vtol...
        self.stl_dir = os.path.join(self.fusion_dir, "stl")
        self.lay, self.preset, self.direct_mode = lay, preset, direct_mode
        self.design = None
        self.root = None
        self.NEW = adsk.fusion.FeatureOperations.NewBodyFeatureOperation
        self.JOIN = adsk.fusion.FeatureOperations.JoinFeatureOperation
        self.CUT = adsk.fusion.FeatureOperations.CutFeatureOperation
        self.INTERSECT = adsk.fusion.FeatureOperations.IntersectFeatureOperation     # v0.6.2 gable window tool
        self.opnames = {self.NEW: "NewBody", self.JOIN: "Join", self.CUT: "Cut", self.INTERSECT: "Intersect"}
        self.components: dict[str, CB] = {}
        self.failed: dict[str, str] = {}
        self.soft_failures: list[str] = []
        self.timings: dict[str, float] = {}
        self.report_extra: dict = {}
        self.inertia_convention: str = "unknown"
        self.inertia_probe: dict = {}
        self._appearances: dict = {}          # 0.9.6 livery: 'white' / 'orange' / 'black' -> design Appearance (or None)

    # ---- 0.9.6 livery: display colours (appearance only, never geometry or mass) ----
    def appearance_spec(self, key: str) -> tuple[str, str]:
        """(appearance name, Bambu hex) of a colour key: printer.color / color_hex (white printed parts), the livery's
        logo colour (orange inlay), printer.color_exceptions.launch_pad (the pad stays black)."""
        pr = self.lay.p["printer"]
        if key == "orange":
            c = self.lay.p["livery"]["colours"]["logo"]
            return f"CanSat PLA Matte {c['name']} {c['hex']}", c["hex"]
        if key == "black":
            c = (pr.get("color_exceptions") or {}).get("launch_pad") or {}
            return f"CanSat PLA Matte {c.get('bambu_name', 'Black')} {c.get('hex', '#000000')}", c.get("hex", "#000000")
        return f"CanSat PLA Matte {pr.get('color', 'White')} {pr.get('color_hex', '#FFFFFF')}", pr.get("color_hex", "#FFFFFF")

    def appearance(self, key: str):
        """The design appearance of a colour key: a copy (Design.appearances.addByCopy) of a plastic appearance from
        the first appearance library, its colour property set to the Bambu hex; cached; None (logged) when the API
        refuses - a missing colour never fails a component."""
        if key in self._appearances:
            return self._appearances[key]
        name, hexv = self.appearance_spec(key)
        made = None
        try:
            lib = None
            libs = self.app.materialLibraries
            for i in range(int(_num(libs.count, 0))):
                cand = libs.item(i)
                if "appearance" in str(cand.name).lower():
                    lib = cand
                    break
            if lib is None:
                raise RuntimeError("no appearance library")
            src = None
            for nm in ("Plastic - Matte (White)", "Plastic - Glossy (White)", "Plastic - Translucent Matte (White)"):
                try:
                    src = lib.appearances.itemByName(nm)
                except Exception:  # noqa: BLE001
                    src = None
                if src is not None:
                    break
            if src is None:
                aps = lib.appearances
                for i in range(int(_num(aps.count, 0))):
                    a_ = aps.item(i)
                    if str(a_.name).lower().startswith("plastic"):
                        src = a_
                        break
            if src is None:
                raise RuntimeError("no plastic appearance in the library")
            made = self.design.appearances.addByCopy(src, name)
            rgb = (int(hexv[1:3], 16), int(hexv[3:5], 16), int(hexv[5:7], 16))
            done = False
            for pid in ("opaque_albedo", "generic_diffuse"):
                try:
                    prop = made.appearanceProperties.itemById(pid)
                except Exception:  # noqa: BLE001
                    prop = None
                if prop is not None:
                    adsk.core.ColorProperty.cast(prop).value = adsk.core.Color.create(rgb[0], rgb[1], rgb[2], 255)
                    done = True
                    break
            if not done:
                self.log.warn(f"appearance {name}: no colour property found; the copied library colour is kept")
            self.log.info(f"appearance {name} ({hexv}) from library appearance {src.name!s}")
        except Exception as exc:  # noqa: BLE001 - display colour only
            self.log.warn(f"appearance {name}: {exc!r} (display colour only; geometry and mass unaffected)")
            made = None
        self._appearances[key] = made
        return made

    def apply_appearance(self, name: str, kind: str, cb: "CB") -> None:
        """Printed components get the white appearance, the orange inlay part the logo colour, the launch pad black
        (report derived.extra.appearances)."""
        if kind != "printed":
            return
        inlay = getattr(self.lay, "livery_inlay_part", None)
        key = "orange" if name == inlay else "black" if name == "launch_pad" else "white"
        ap = self.appearance(key)
        rec = self.report_extra.setdefault("appearances", {})
        if ap is None:
            rec[name] = f"{key} (not applied)"
            return
        try:
            bodies = cb.comp.bRepBodies
            for i in range(int(_num(bodies.count, 0))):
                bodies.item(i).appearance = ap
            rec[name] = key
        except Exception as exc:  # noqa: BLE001
            self.log.warn(f"{name}: appearance {key} not applied ({exc!r})")
            rec[name] = f"{key} (not applied)"

    # ---- design ----
    def create_design(self) -> None:
        doc = self.app.documents.add(adsk.core.DocumentTypes.FusionDesignDocumentType)
        design = adsk.fusion.Design.cast(self.app.activeProduct)
        if design is None:
            design = adsk.fusion.Design.cast(doc.products.itemByProductType("DesignProductType"))
        if design is None:
            raise RuntimeError("could not obtain the Design product of the new document")
        self.design = design
        try:
            design.designType = adsk.fusion.DesignTypes.DirectDesignType if self.direct_mode else adsk.fusion.DesignTypes.ParametricDesignType
        except Exception as exc:  # noqa: BLE001
            self.log.warn(f"designType could not be set ({exc!r})")
        try:
            design.fusionUnitsManager.distanceDisplayUnits = adsk.fusion.DistanceUnits.MillimeterDistanceUnits
        except Exception as exc:  # noqa: BLE001
            self.log.warn(f"display units could not be set to mm ({exc!r})")
        self.root = design.rootComponent
        try:
            self.root.name = "CanSatEDFRocket"
        except Exception:
            pass
        self.log.info(f"design created: type={'direct' if self.direct_mode else 'parametric'}, units=mm, preset={self.preset}")

    # ---- user parameters ----
    def add_user_parameters(self) -> int:
        ups = self.design.userParameters
        n = 0
        taken: set[str] = set()
        unitless = re.compile(r"(count|cells|^kv$|fraction|exponent|throttle|_g$|_N$|_N_|_A$|_W$|_V$|_s$|_per_rad|haack_c|_ratio)")
        for section in ("body", "edf", "propulsion_module", "vane", "fin", "intake_module", "payload_module", "nose"):
            for key, val in self.lay.p.get(section, {}).items():
                if isinstance(val, bool) or not isinstance(val, (int, float)):
                    continue
                name = sanitize_param_name(f"{section}_{key}")
                if name in taken:
                    continue
                taken.add(name)
                if key.endswith("_deg"):
                    units, expr = "deg", f"{val:g} deg"
                elif unitless.search(key):
                    units, expr = "", f"{val:g}"
                else:
                    units, expr = "mm", f"{val:g} mm"
                try:
                    ups.add(name, adsk.core.ValueInput.createByString(expr), units, f"params.json {section}.{key} (mirror; geometry is driven by layout.py)")
                    n += 1
                except Exception as exc:  # noqa: BLE001
                    self.log.warn(f"user parameter {name}={expr!r} rejected: {exc!r}")
        self.log.info(f"user parameters added: {n}")
        return n

    # ---- inertia sign-convention probe (SPEC section 6: verify on a test cube) ----
    def detect_inertia_convention(self) -> None:
        cx, cy, cz, s = 30.0, 20.0, 10.0, 10.0  # mm; cube centre and size
        try:
            cb = CB(self, "inertia_probe")
            cb.box_xy(cx - s / 2, cx + s / 2, cy - s / 2, cy + s / 2, cz - s / 2, cz + s / 2, self.NEW, "probe_cube")
            pp = cb.body.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)
            m = _num(pp.mass)  # kg
            vals = list(pp.getXYZMomentsOfInertia())
            ok, xx, yy, zz, xy, yz, xz = (bool(vals[0]), *[_num(v) for v in vals[1:7]])
            c = (cm(cx), cm(cy), cm(cz))
            a = cm(s)
            exp_xx = m * (a * a / 6.0 + c[1] ** 2 + c[2] ** 2)  # kg cm^2 about the origin
            exp_xy_product = m * c[0] * c[1]
            r_xx = xx / exp_xx if exp_xx else 0.0
            r_xy = xy / exp_xy_product if exp_xy_product else 0.0
            if r_xy > 0.9 and r_xy < 1.1:
                self.inertia_convention = "product_integral"   # Fusion returns +integral(xy dm)
            elif r_xy < -0.9 and r_xy > -1.1:
                self.inertia_convention = "tensor"             # Fusion returns -integral(xy dm)
            else:
                self.inertia_convention = "unknown"
                self.log.warn(f"inertia probe: off-diagonal convention not identified (xy ratio {r_xy:.3f}); assuming tensor")
            if not (0.95 < r_xx < 1.05):
                self.log.warn(f"inertia probe: Ixx ratio {r_xx:.3f} (expected 1.0 for kg*cm^2 about the origin)")
            self.inertia_probe = {"ok": ok, "mass_kg": m, "density_kg_cm3": _num(pp.density), "xx_ratio": r_xx, "xy_ratio": r_xy,
                                  "convention": self.inertia_convention}
            self.log.info(f"inertia probe: {self.inertia_probe}")
            try:
                cb.occ.deleteMe()
            except Exception as exc:  # noqa: BLE001
                self.log.warn(f"inertia probe component could not be deleted: {exc!r}")
        except Exception as exc:  # noqa: BLE001
            self.log.warn(f"inertia probe failed: {exc!r}; assuming tensor convention")
            self.inertia_convention = "unknown"

    # ---- components ----
    def component_specs_vtol(self) -> list[tuple[str, str, object, float | None]]:
        """v0.9 CAD-2 VTOL component list (rocket_geometry._build_all_vtol): no payload chain, nose, NAV / COM / GNS /
        radio box, release servo or launch pad; top cap, carrier PCB, GPS on the cap, battery / buffer pack by power
        source."""
        L = self.lay
        p = L.p
        specs: list[tuple[str, str, object, float | None]] = [
            ("propulsion_module", "printed", build_propulsion_module, None),
            ("intake_module", "printed", build_intake_module, None),
            ("avionics_module", "printed", build_avionics_module, None),
            *((("avionics_logo", "printed", build_avionics_logo, None),) if getattr(L, "has_livery", False) else ()),
            ("avionics_tray", "printed", build_avionics_tray, None),
            ("top_cap", "printed", build_top_cap, None),
        ]
        for i, th in enumerate(L.vane_thetas, start=1):
            specs.append((f"vane_{i}", "printed", (lambda ctx, cb, th=th: build_vane(ctx, cb, th)), None))
        for i, th in enumerate(L.vane_thetas, start=1):
            specs.append((f"vane_coupler_{i}", "printed", (lambda ctx, cb, th=th: build_vane_coupler(ctx, cb, th)), None))
        for i in range(1, L.vane_count + 1):
            specs.append((f"servo_cover_{i}", "printed", build_servo_cover(i - 1), None))
        specs.append(("wire_cover", "printed", build_wire_cover, None))
        specs.append(("edf_unit", "placeholder", build_edf_unit, p["edf"]["mass_g"]))
        if L.has_battery and not getattr(L, "buffer_z", None):
            specs.append(("battery", "placeholder", build_box_placeholder(L.battery_x, L.battery_y, L.battery_z), p["battery"]["mass_g"]))
        if getattr(L, "buffer_z", None):
            specs.append(("buffer_pack", "placeholder", build_box_placeholder(L.buffer_x, L.buffer_y, L.buffer_z),
                          p["power"]["hybrid_extra_ASSUMED"]["buffer_pack"]["mass_g"]))
        specs += [
            ("esc", "placeholder", build_box_placeholder(L.esc_x, L.esc_y, L.esc_z), p["electronics"]["esc"]["mass_g"]),
            ("misc_electronics", "placeholder", build_misc_electronics, p["electronics"]["misc"]["mass_g"]),
            (L.fcu_part, "placeholder", build_box_placeholder(L.fcu_x, L.fcu_y, L.fcu_z), L.fcu_mass_g),
            ("gps", "placeholder", build_cap_gps_module, L.gps_board_mass_g),
            ("gps_ground_plane", "placeholder", build_box_placeholder(L.cap_gp_xy[0], L.cap_gp_xy[1], L.cap_gp_z), L.gps_gp_mass_g),
        ]
        for i in range(1, L.vane_count + 1):
            specs.append((f"servo_vane_{i}", "placeholder", build_servo_direct(i - 1), p["servo"]["mass_g"]))
        for i in range(1, L.vane_count + 1):
            specs.append((f"servo_horn_{i}", "placeholder", build_servo_horn(i - 1), p["vane_drive"]["horn"]["mass_g"]))
        if getattr(L, "dd_bushing", False):      # v0.9 fix round: brass hinge bushings
            for i in range(1, L.vane_count + 1):
                specs.append((f"hinge_bushing_{i}", "placeholder", build_hinge_bushing(i - 1), L.dd_bush_mass_g))
        # v0.9 fix round: the tether kit's boxes and the hardpoint stack (power.source tether / hybrid)
        for nm, (bx_, by_, bz_, m_) in getattr(L, "tether_boxes", {}).items():
            specs.append((nm, "placeholder", build_box_placeholder(bx_, by_, bz_), m_))
        if L.power_source in ("tether", "hybrid") and getattr(L, "cap_hardpoint", False):
            specs += [("tether_eyebolt", "placeholder", build_tether_eyebolt, L.cap_bolt_mass_g),
                      ("tether_load_washer", "placeholder", build_tether_load_washer, L.cap_washer_mass_g),
                      ("tether_nut", "placeholder", build_tether_nut, L.cap_nut_mass_g)]
        return specs

    def component_specs(self) -> list[tuple[str, str, object, float | None]]:
        L = self.lay
        p = L.p
        if getattr(L, "vtol", False):      # v0.9 CAD-2
            return self.component_specs_vtol()
        rec = p["recovery"]
        specs: list[tuple[str, str, object, float | None]] = [
            ("propulsion_module", "printed", build_propulsion_module, None),
            ("intake_module", "printed", build_intake_module, None),
            ("avionics_module", "printed", build_avionics_module, None),
            *((("avionics_logo", "printed", build_avionics_logo, None),) if getattr(L, "has_livery", False) else ()),
            ("avionics_tray", "printed", build_avionics_tray, None),
            ("payload_module", "printed", build_payload_module, None),
            ("piston", "printed", build_piston, None),
            ("nose_cone", "printed", build_nose_cone, None),
        ]
        for i, th in enumerate(L.vane_thetas, start=1):
            specs.append((f"vane_{i}", "printed", (lambda ctx, cb, th=th: build_vane(ctx, cb, th)), None))
        direct = getattr(L, "direct_drive", False)
        if direct:   # v0.9: couplers and servo covers instead of horns
            for i, th in enumerate(L.vane_thetas, start=1):
                specs.append((f"vane_coupler_{i}", "printed", (lambda ctx, cb, th=th: build_vane_coupler(ctx, cb, th)), None))
            for i in range(1, L.vane_count + 1):
                specs.append((f"servo_cover_{i}", "printed", build_servo_cover(i - 1), None))
        else:
            for i, th in enumerate(L.vane_thetas, start=1):
                specs.append((f"vane_horn_{i}", "printed", (lambda ctx, cb, th=th: build_vane_horn(ctx, cb, th)), None))
        if L.tabs and getattr(L, "wire_ch_open", False):   # v0.9: the motor-lead groove cover
            specs.append(("wire_cover", "printed", build_wire_cover, None))
        specs.append(("latch_arm", "printed", build_latch_arm, None))
        if getattr(L, "has_gps_pod", False):   # v0.8: the GPS pod lid (the pod is part of payload_module)
            specs.append(("gps_pod_lid", "printed", build_gps_pod_lid, None))
        specs += [
            ("launch_pad", "printed", build_launch_pad, None),
            ("edf_unit", "placeholder", build_edf_unit, p["edf"]["mass_g"]),
            ("battery", "placeholder", build_box_placeholder(L.battery_x, L.battery_y, L.battery_z), p["battery"]["mass_g"]),
            ("esc", "placeholder", build_box_placeholder(L.esc_x, L.esc_y, L.esc_z), p["electronics"]["esc"]["mass_g"]),
            ("nav", "placeholder", build_box_placeholder(L.nav_x, L.nav_y, L.nav_z), p["electronics"]["nav"]["mass_g"]),
            ("misc_electronics", "placeholder", build_misc_electronics, p["electronics"]["misc"]["mass_g"]),
        ]
        if L.has_com_board:   # v0.6: COM board hanging from the payload bulkhead's bosses
            specs.append(("com", "placeholder", build_box_placeholder(L.com_board_x, L.com_board_y, L.com_board_z),
                          p["electronics"]["com"]["mass_g"]))
        if getattr(L, "has_radio_log", False):   # v0.6.2: radio / logger box beside the COM board
            specs.append(("radio_log", "placeholder", build_box_placeholder(L.radio_log_x, L.radio_log_y, L.radio_log_z),
                          p["electronics"]["radio_log"]["mass_g"]))
        if getattr(L, "has_fcu", False):   # v0.6.3: FCU board flat on the intake bulkhead (foam tape)
            specs.append(("fcu", "placeholder", build_box_placeholder(L.fcu_x, L.fcu_y, L.fcu_z), p["electronics"]["fcu"]["mass_g"]))
        if getattr(L, "has_gns", False):   # v0.8.1: GNS board on its bosses on the -x tube wall (board + screws)
            specs.append(("gns", "placeholder", build_box_placeholder(L.gns_x, L.gns_y, L.gns_z), L.gns_mass_g))
        if getattr(L, "has_gps_pod", False):   # v0.8: GPS board + patch and the ground plane in the pod
            specs.append(("gps", "placeholder", build_gps_module, L.gps_board_mass_g))
            specs.append(("gps_ground_plane", "placeholder", build_gps_ground_plane, L.gps_gp_mass_g))
        if direct:   # v0.9: servo in the fin-root fairing, horn disc on its spline
            for i in range(1, L.vane_count + 1):
                specs.append((f"servo_vane_{i}", "placeholder", build_servo_direct(i - 1), p["servo"]["mass_g"]))
            specs.append(("servo_release", "placeholder", build_servo_release, p["servo"]["mass_g"]))
            for i in range(1, L.vane_count + 1):
                specs.append((f"servo_horn_{i}", "placeholder", build_servo_horn(i - 1), p["vane_drive"]["horn"]["mass_g"]))
            if getattr(L, "dd_bushing", False):      # v0.9 fix round: brass hinge bushings
                for i in range(1, L.vane_count + 1):
                    specs.append((f"hinge_bushing_{i}", "placeholder", build_hinge_bushing(i - 1), L.dd_bush_mass_g))
        else:
            for i, th in enumerate(L.pod_thetas, start=1):
                specs.append((f"servo_vane_{i}", "placeholder", build_servo_vane(th), p["servo"]["mass_g"]))
            specs.append(("servo_release", "placeholder", build_servo_release, p["servo"]["mass_g"]))
            for i, th in enumerate(L.pod_thetas, start=1):
                specs.append((f"servo_arm_{i}", "placeholder", build_servo_arm(th), 1.0))
            for i, (tp, tv) in enumerate(zip(L.pod_thetas, L.vane_thetas), start=1):
                specs.append((f"pushrod_{i}", "placeholder", build_pushrod(tp, tv), 1.0))
        specs += [
            ("cansat_payload", "placeholder", build_cansat_payload, p["payload"]["mass_g"]),
            ("spring", "placeholder", build_spring, 0.5 * p["hardware_mass_g"]["spring_piston_hardware"]),
            ("chute_pack", "placeholder", build_chute_pack, rec["chute_mass_g"] + rec["lines_swivel_mass_g"] + rec["shock_cord_mass_g"]),
        ]
        return specs

    def build_component(self, name: str, kind: str, fn, mass_g: float | None) -> None:
        t0 = time.time()
        self.log.info(f"=== component {name} ({kind}) start")
        cb = None
        try:
            cb = CB(self, name)
            fn(self, cb)
            if cb.required_failed:
                # a SPEC-required (v0.5) cut / join failed: the part is wrong, so the component fails (and is
                # not combined - a leftover cutter must never be joined into it)
                n_req = cb.required_ok + len(cb.required_failed)
                raise RuntimeError(f"{len(cb.required_failed)}/{n_req} required feature(s) failed: " + "; ".join(cb.required_failed))
            nb = int(_num(cb.comp.bRepBodies.count, 0))
            if kind == "printed" and nb > 1:
                nb = cb.combine_all(f"{name}_combine")
                if nb > 1:
                    self.log.warn(f"{name}: printed part has {nb} bodies after combine")
            self.components[name] = cb
            self.apply_appearance(name, kind, cb)       # 0.9.6 livery: display colour only
            dt = time.time() - t0
            self.timings[name] = dt
            self.log.info(f"=== component {name} done in {dt:.2f}s: {cb.n_features} features, {cb.n_sketches} sketches, {nb} bodies")
        except Exception:
            dt = time.time() - t0
            self.timings[name] = dt
            tb = traceback.format_exc()
            first = tb.strip().splitlines()[-1] if tb.strip() else "unknown error"
            self.failed[name] = first
            self.log.error(f"=== component {name} FAILED after {dt:.2f}s: {first}\n{tb}")
            if cb is not None:
                self.components[name] = cb  # keep partial geometry for inspection / mass props

    # ---- mass properties (SPEC section 6 recipe) ----
    def _to_tensor_offdiag(self, xy: float, yz: float, xz: float) -> tuple[float, float, float]:
        if self.inertia_convention == "product_integral":
            return (-xy, -yz, -xz)
        return (xy, yz, xz)

    def measure_component(self, name: str, kind: str, mass_g: float | None) -> dict:
        cb = self.components[name]
        comp = cb.comp
        nb = int(_num(comp.bRepBodies.count, 0))
        bodies = [comp.bRepBodies.item(i) for i in range(nb)]
        acc = adsk.fusion.CalculationAccuracy.HighCalculationAccuracy
        raw = []
        v_total = 0.0
        bbox = None
        for b in bodies:
            pp = b.getPhysicalProperties(acc)
            V = _num(pp.volume)  # cm^3
            c = pp.centerOfMass
            com = (_num(c.x), _num(c.y), _num(c.z))  # cm
            rho_f = _num(pp.density)  # kg/cm^3 (Fusion's current material)
            try:
                vals = list(pp.getXYZMomentsOfInertia())
                mom = [_num(v) for v in vals[1:7]]
                if len(mom) != 6:
                    raise ValueError("moments tuple too short")
            except Exception as exc:  # noqa: BLE001
                self.log.warn(f"{name}: getXYZMomentsOfInertia failed ({exc!r}); inertia set to 0")
                mom = [0.0] * 6
            bb = b.boundingBox
            lo = (_num(bb.minPoint.x) * 10, _num(bb.minPoint.y) * 10, _num(bb.minPoint.z) * 10)
            hi = (_num(bb.maxPoint.x) * 10, _num(bb.maxPoint.y) * 10, _num(bb.maxPoint.z) * 10)
            bbox = [list(lo), list(hi)] if bbox is None else [[min(a, c2) for a, c2 in zip(bbox[0], lo)], [max(a, c2) for a, c2 in zip(bbox[1], hi)]]
            raw.append((V, com, rho_f, mom))
            v_total += V
        if kind == "printed":
            rho = printed_density(self.lay, name, self.root_dir)       # v0.9: printer.mass_model
        else:
            rho = (mass_g / v_total) if v_total > 0 else 0.0
            if v_total <= 0:
                self.log.warn(f"{name}: placeholder has zero volume; density undefined")
        parts = []  # (m_kg, com_m, I_com 3x3)
        for V, com, rho_f, mom in raw:
            m = rho / 1000.0 * V  # kg
            com_m = tuple(ci / 100.0 for ci in com)
            if rho_f > 0:
                scale = (rho / 1000.0) / rho_f * 1e-4  # kg cm^2 (Fusion density) -> kg m^2 (target density)
            else:
                scale = 0.0
                self.log.warn(f"{name}: Fusion density is 0; inertia unavailable for this body")
            xx, yy, zz = mom[0] * scale, mom[1] * scale, mom[2] * scale
            xy, yz, xz = self._to_tensor_offdiag(mom[3] * scale, mom[4] * scale, mom[5] * scale)
            dx, dy, dz = com_m
            # move from the world origin to the body COM (tensor convention)
            I_com = [[xx - m * (dy * dy + dz * dz), xy + m * dx * dy, xz + m * dx * dz],
                     [xy + m * dx * dy, yy - m * (dx * dx + dz * dz), yz + m * dy * dz],
                     [xz + m * dx * dz, yz + m * dy * dz, zz - m * (dx * dx + dy * dy)]]
            parts.append((m, com_m, I_com))
        m_tot, com_tot, I_tot = combine_inertia(parts)
        return {"kind": kind, "volume_mm3": v_total * 1000.0, "density_g_cm3": rho, "mass_g": m_tot * 1000.0,
                "bbox_mm": bbox if bbox is not None else [[0.0, 0.0, 0.0], [0.0, 0.0, 0.0]],
                "com_mm": [ci * 1000.0 for ci in com_tot], "inertia_about_com_kg_m2": I_tot, "bodies": nb}

    def lumped_part(self, mass_g: float, z_mm: float, x_mm: float = 0.0, y_mm: float = 0.0) -> dict:
        """Point mass (SPEC 6 lumped item): on the axis unless x / y are given (v0.6 servo_leads)."""
        return {"kind": "lumped", "volume_mm3": 0.0, "density_g_cm3": 0.0, "mass_g": mass_g,
                "bbox_mm": [[x_mm, y_mm, z_mm], [x_mm, y_mm, z_mm]], "com_mm": [x_mm, y_mm, z_mm],
                "inertia_about_com_kg_m2": [[0.0] * 3 for _ in range(3)], "bodies": 0}

    def build_report(self, parts: dict, params_sha: str, pushrod_len: float) -> dict:
        L = self.lay
        def totals_of(names):
            sel = [(parts[n]["mass_g"] / 1000.0, tuple(c / 1000.0 for c in parts[n]["com_mm"]), parts[n]["inertia_about_com_kg_m2"]) for n in names]
            m, com, I = combine_inertia(sel)
            return m, com, I
        vehicle = [n for n in parts if n not in NOT_IN_VEHICLE]
        m, com, I = totals_of(vehicle)
        printed_mass = sum(parts[n]["mass_g"] for n in vehicle if parts[n]["kind"] == "printed")
        post = [n for n in vehicle if n not in EJECTED]
        mp, comp_, Ip = totals_of(post)
        descent_mass = m * 1000.0 - parts.get("cansat_payload", {}).get("mass_g", 0.0)
        warnings = list(self.log.warnings)
        if self.inertia_convention == "unknown":
            warnings.append("inertia off-diagonal sign convention unverified; only diagonal terms are trustworthy")
        return {
            "generator": "fusion360",
            "generator_version": f"CanSatEDFRocket.py {SCRIPT_VERSION}; Fusion {self.app_version()}",
            "preset": L.p["meta"].get("preset"),
            "params_sha256": params_sha,
            "parts": parts,
            "totals": {"mass_g": m * 1000.0, "printed_mass_g": printed_mass, "com_mm": [c * 1000.0 for c in com],
                       "inertia_about_com_kg_m2": I,
                       "post_ejection": {"mass_g": mp * 1000.0, "com_mm": [c * 1000.0 for c in comp_], "inertia_about_com_kg_m2": Ip,
                                         "descent_mass_g": descent_mass}},
            **({"config": L.config, "power_source": L.power_source} if getattr(L, "vtol", False) else {}),
            "derived": {"pushrod_len_mm": pushrod_len, "nose_cavity_volume_L": getattr(L, "nose_cavity_volume_L", 0.0),
                        "footprint_d_mm": L.footprint_d,
                        "vehicle_h_mm": L.vehicle_h, "inlet_area_ratio": L.inlet_area_ratio,
                        "inertia_offdiag_convention": self.inertia_convention, "inertia_probe": self.inertia_probe,
                        "excluded_from_totals": list(NOT_IN_VEHICLE), "extra": self.report_extra},
            "failed_components": dict(self.failed),
            "soft_failures": list(self.soft_failures),
            "timings_s": {k: round(v, 3) for k, v in self.timings.items()},
            "warnings": warnings,
        }

    def app_version(self) -> str:
        try:
            return str(self.app.version)
        except Exception:
            return "unknown"

    # ---- exports ----
    def export_all(self) -> dict:
        em = self.design.exportManager
        done: dict[str, str] = {}
        stl_map = {name: name for name in self.lay.printed_parts}
        stl_map["vane"] = "vane_1"
        stl_map["vane_horn"] = "vane_horn_1"
        # v0.9 CAD-2: every multi-instance printed part exports its first instance (the v0.9 couplers / covers were
        # looked up by their base name and skipped with a "component missing" warning)
        for base in list(stl_map):
            if base not in self.components and f"{base}_1" in self.components:
                stl_map[base] = f"{base}_1"
        for stl_name, comp_name in stl_map.items():
            cb = self.components.get(comp_name)
            if cb is None:
                self.log.warn(f"STL {stl_name}: component {comp_name} missing")
                continue
            path = os.path.join(self.stl_dir, f"{stl_name}.stl")
            try:
                opts = em.createSTLExportOptions(cb.comp, path)
                opts.meshRefinement = adsk.fusion.MeshRefinementSettings.MeshRefinementHigh
                opts.isBinaryFormat = True
                em.execute(opts)
                self._check_file(path, f"STL {stl_name}", done)
            except Exception as exc:  # noqa: BLE001
                self.log.warn(f"STL export {stl_name} failed: {exc!r}")
        step = os.path.join(self.fusion_dir, "CanSatEDFRocket.step")
        try:
            em.execute(em.createSTEPExportOptions(step, self.root))
            self._check_file(step, "STEP", done)
        except Exception as exc:  # noqa: BLE001
            self.log.warn(f"STEP export failed: {exc!r}")
        f3d = os.path.join(self.fusion_dir, "CanSatEDFRocket.f3d")
        try:
            em.execute(em.createFusionArchiveExportOptions(f3d))
            self._check_file(f3d, "f3d", done)
        except Exception as exc:  # noqa: BLE001
            self.log.warn(f"f3d export failed: {exc!r}")
        png = os.path.join(self.fusion_dir, "preview.png")
        try:
            vp = self.app.activeViewport
            vp.fit()
            vp.saveAsImageFile(png, 1600, 1200)
            self._check_file(png, "preview", done)
        except Exception as exc:  # noqa: BLE001
            self.log.warn(f"preview image failed: {exc!r}")
        return done

    def _check_file(self, path: str, label: str, done: dict) -> None:
        if os.path.isfile(path) and os.path.getsize(path) > 0:
            done[label] = path
            self.log.info(f"exported {label}: {path} ({os.path.getsize(path)} bytes)")
        else:
            self.log.warn(f"{label}: export call returned but file not found: {path}")


def combine_inertia(parts) -> tuple[float, tuple[float, float, float], list]:
    """parts: [(m_kg, com_m(x,y,z), I_about_own_com 3x3)] -> (M, COM, I about COM), tensor convention."""
    M = sum(p[0] for p in parts)
    if M <= 0:
        return 0.0, (0.0, 0.0, 0.0), [[0.0] * 3 for _ in range(3)]
    com = tuple(sum(p[0] * p[1][k] for p in parts) / M for k in range(3))
    I = [[0.0] * 3 for _ in range(3)]
    for m, c, Ic in parts:
        d = (c[0] - com[0], c[1] - com[1], c[2] - com[2])
        d2 = d[0] ** 2 + d[1] ** 2 + d[2] ** 2
        for i in range(3):
            for j in range(3):
                I[i][j] += Ic[i][j] + m * ((d2 if i == j else 0.0) - d[i] * d[j])
    return M, com, I


# --------------------------------------------------------------------------- #
# entry point
# --------------------------------------------------------------------------- #
def run(context):  # noqa: ARG001 - Fusion passes a context string
    t_start = time.time()
    app = ui = None
    log = None
    report_path = ""
    try:
        app = adsk.core.Application.get()
        ui = app.userInterface
        root_dir = find_project_root(SCRIPT_DIR)
        export_root = os.environ.get("CANSAT_EXPORT_ROOT", "").strip() or os.path.join(root_dir, "cad", "exports")
        cad_dir = os.path.join(root_dir, "cad")
        if cad_dir not in sys.path:
            sys.path.insert(0, cad_dir)
        import importlib
        import layout  # noqa: E402 - must follow the sys.path insert
        # Fusion keeps one interpreter for the whole session: reload so edits to cad/layout.py
        # made since the last run are picked up (a stale module silently used old geometry rules)
        layout = importlib.reload(layout)
        preset = read_preset(root_dir)
        cfg, pw = read_config(root_dir)        # v0.9 CAD-2: hardware configuration / VTOL power source
        lay = layout.build(preset) if cfg is None and pw is None else layout.build(preset, config=cfg, power=pw)
        sfx = config_file_suffix(lay)
        fusion_dir = os.path.join(export_root, "fusion" + sfx)
        os.makedirs(os.path.join(fusion_dir, "stl"), exist_ok=True)
        log = RunLog(os.path.join(fusion_dir, "run.log"))
        log.info(f"project root: {root_dir}")
        log.info(f"export root: {export_root}")
        log.info(f"configuration: {getattr(lay, 'config', 'rocket')} (power {getattr(lay, 'power_source', 'battery')}), "
                 f"exports in {fusion_dir}")
        log.info(f"layout loaded: preset={lay.p['meta'].get('preset')} vehicle_h={lay.vehicle_h:g} mm footprint={lay.footprint_d:g} mm")
        with open(os.path.join(root_dir, "params.json"), "rb") as f:
            params_sha = hashlib.sha256(f.read()).hexdigest()
        direct_mode = os.environ.get("CANSAT_FUSION_DIRECT", "") == "1" or os.path.isfile(os.path.join(root_dir, "cad", "fusion360", DIRECT_MODE_FILE_NAME))
        ctx = Ctx(app, ui, log, root_dir, export_root, lay, preset, direct_mode)
        ctx.create_design()
        ctx.add_user_parameters()
        ctx.detect_inertia_convention()
        specs = ctx.component_specs()
        for name, kind, fn, mass_g in specs:
            ctx.build_component(name, kind, fn, mass_g)
        # mass properties
        parts: dict = {}
        for name, kind, fn, mass_g in specs:
            if name not in ctx.components:
                continue
            try:
                parts[name] = ctx.measure_component(name, kind, mass_g)
            except Exception as exc:  # noqa: BLE001 - recorded as a failed component (the report is then refused downstream)
                msg = f"mass properties failed: {exc!r}"
                log.error(f"{name}: {msg}")
                ctx.failed[name] = f"{ctx.failed[name]}; {msg}" if name in ctx.failed else msg
        hw = lay.p["hardware_mass_g"]
        vtol = getattr(lay, "vtol", False)
        joint_zs = [lay.z_prop_spigot_bottom, lay.z_intake_spigot_bottom, lay.z_avionics_spigot_bottom]
        if not vtol:          # v0.9 CAD-2: the VTOL stack ends at the avionics / top-cap joint
            joint_zs.append(lay.z_payload_spigot_bottom)
        parts["joint_screws_inserts"] = ctx.lumped_part(hw["joint_screws_inserts"], sum(z + lay.joint_screw_dz for z in joint_zs) / len(joint_zs))
        if getattr(lay, "direct_drive", False):   # v0.9: cover / coupler / horn screws in the fairings
            parts["vane_drive_hardware"] = ctx.lumped_part(lay.p["vane_drive"]["hardware_g_ASSUMED"],
                                                           0.5 * (lay.fair_z[0] + lay.fair_z[1]))
        else:
            parts["pushrods_horns_clevises"] = ctx.lumped_part(hw["pushrods_horns_clevises"], lay.vane_hinge_z)
        if getattr(lay, "has_motor_leads", False):   # v0.9: motor leads (3 x 12 AWG + bullets) at their centroid
            mx, my, mz = lay.ml_com
            parts["motor_leads"] = ctx.lumped_part(lay.ml_mass_g, mz, mx, my)
        if getattr(lay, "servo_leads", False):   # SPEC 3.8 / 6 v0.6: extension leads at the route's centroid
            sx, sy, sz = lay.sl_lead_com
            parts["servo_leads"] = ctx.lumped_part(lay.sl_lead_mass_g, sz, sx, sy)
        if getattr(lay, "has_gps_pod", False) or vtol:   # v0.8: GPS lead + connector + hardware at Layout.gps_lead_com
            gx, gy, gz = lay.gps_lead_com                 # (v0.9 CAD-2 VTOL: cap top -> carrier)
            parts["gps_lead"] = ctx.lumped_part(lay.gps_lead_mass_g, gz, gx, gy)
        if vtol:   # v0.9 CAD-2: the tether kit (power.source tether / hybrid), lumped at its Layout stations
            for nm, (m_g, (tx, ty, tz)) in lay.tether_items.items():
                parts[nm] = ctx.lumped_part(m_g, tz, tx, ty)
        pr = ctx.report_extra.get("pushrod_len_mm_each", [])
        pushrod_len = pr[0] if pr else 0.0
        exported = ctx.export_all()
        report = ctx.build_report(parts, params_sha, pushrod_len)
        report["exports"] = exported
        report["elapsed_s"] = round(time.time() - t_start, 2)
        report_path = os.path.join(export_root, f"fusion360_report{sfx}.json")   # v0.9 CAD-2: _vtol for the VTOL build
        with open(report_path, "w", encoding="utf-8", newline="\n") as f:
            json.dump(report, f, indent=2, ensure_ascii=False)
        log.info(f"report written: {report_path}")
        n_ok = len(specs) - len(ctx.failed)
        log.info(f"finished in {time.time() - t_start:.1f}s: {n_ok}/{len(specs)} components OK, {len(ctx.failed)} failed, {len(ctx.soft_failures)} optional features skipped")
        log.close()
        msg = (f"CanSat EDF Rocket built in {time.time() - t_start:.0f} s\n"
               f"Components: {n_ok}/{len(specs)} OK" + (f" (failed: {', '.join(ctx.failed)})" if ctx.failed else "") + "\n"
               f"Optional features skipped: {len(ctx.soft_failures)}\n"
               f"Vehicle mass: {report['totals']['mass_g']:.0f} g (printed {report['totals']['printed_mass_g']:.0f} g), "
               f"CG z = {report['totals']['com_mm'][2]:.1f} mm\n"
               f"Exports: {len(exported)} files in {fusion_dir}\n"
               f"Report: {report_path}\nLog: {log.path}")
        ui.messageBox(msg, "CanSatEDFRocket")
    except Exception:
        tb = traceback.format_exc()
        first = tb.strip().splitlines()[-1] if tb.strip() else "unknown error"
        if log is not None:
            log.error(tb)
            log.close()
            where = log.path
        else:
            where = "(log not created)"
        if ui is not None:
            ui.messageBox(f"CanSatEDFRocket FAILED: {first}\nLog: {where}", "CanSatEDFRocket")
        elif app is not None:
            app.log(tb)
