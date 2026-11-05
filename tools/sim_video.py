"""Flight-simulation video renderer (v0.7): a log in, an MP4 + a poster PNG out.

Renders a simulated flight of the CanSat EDF vehicle as an H.264 MP4 (default
1920 x 1080, 30 fps, real time; ``--speed`` for faster / slower) with PyVista
off-screen rendering, plus a poster PNG and a JSON sidecar (frame count, render
speed, warnings).  It is a *viewer*: it never runs the flight software and never
changes a number of the log; everything on screen is either a logged value, a
value resampled from the log (linear / slerp / zero-order hold) or a quantity
derived from logged values by the formula written next to it.

Log contract (``.npz``, written by the VTOL simulation; keys after the colon are
optional, the loader fills documented defaults and records a warning):

  t [N] s (strictly increasing)  pos [N,3] world m  q [N,4] (w,x,y,z body->world)
  : vel [N,3] m/s  delta [N,4] actual vane angles rad  throttle [N]  thrust [N] N
    state [N] (LP_STATE: SAFE 0 ARMED 1 POWERED 2 UNPOWERED 3 RELEASED 4 ABORT 5)
    phase [N] (NONE 0 SPOOL 1 CLIMB 2 HOLD 3 LAND 4 LANDED 5)  h_target [N] m
    h_ref [N] m  h_est [N] m  tilt_deg [N]  wind [N,3] world m/s incl. gusts
    gust_active [N] bool  ground_z scalar m (<= 0)  event_t / event_text
    cmd_t / cmd_name (ground commands)  meta (JSON string, see below)

  meta keys used: scenario, title, seed, mode ('vtol' | 'ballistic'), params_sha256,
  notes (model caveats - shown on the title and end cards: put "no ground-effect
  model" etc. here), pos_ref + z_cg_m, t_max_n (thrust of the full-length plume,
  else the log maximum), limits {u_max (saturation flag, default 1), hover_throttle,
  tilt_abort_deg, ceiling_m, geofence_m}, poster_t, state_names / phase_names
  ({"code": "NAME"} overrides).  v2 (all optional): title_ko / notes_ko (Korean title
  and caveats for --lang ko; without notes_ko the cards list the caveats recognised in
  ``notes`` in Korean and print ``notes`` itself small), criteria.gust_band_deg (the
  push-recovery band, default 2), profile ('field' | 'demo').

  v2 optional keys (no warning when absent):
    push_force [N,3] world N, the force the person applies to the vehicle (0 when not pushing)
    push_point [N,3] world m, the contact point (same origin as pos; default: the body axis
                     at 0.45 x the body length, with a warning)
    push_active [N] bool (default |push_force| > 0.05 N, with a warning)
    wind_mean [N,3] world m/s, the deterministic wind (mean + a swinging direction, no
                     turbulence, no gusts) for the compass and the wind-direction plot;
                     default: ``wind`` with gust samples held at the last calm value and a
                     1 s centred Gaussian smoothing (labelled "smoothed" on screen)

  v0.8 optional keys (GPS position hold; no warning when absent, every one optional on its own):
    hold_point [3] or [N,3] world m, the hold point (x, y used; drawn on the ground).  [3]: one point,
                     shown from the first HOLD sample on (the v0.8 rule: the position at the end of the climb;
                     from the start when the log has no HOLD); [N,3]: per sample, NaN while there is none
    gps_ok [N] bool  the NAV's "GPS usable" flag          gps_sats [N] int  satellites used (numSV)
    gps_hacc_m [N] m the receiver's horizontal accuracy estimate hAcc (NaN = not reported)
    gps_fix [N] int  fix type (0 none, 2 2-D, 3 3-D)
    pos_est [N,3] world m, the NAV's estimate of the CG position (x, y used; same origin as pos)
    tilt_sp_deg [N] deg  the position controller's tilt setpoint (magnitude; [N,2] components accepted)
  meta (v0.8, optional): gps_on (the vehicle carries the GPS: the antenna pod is drawn even without GPS keys),
  criteria.station_keeping_max_m (a number or {"field": .., "demo": ..}: the HUD distance turns amber above it).
  Derived (formula): CG = pos + R(q) (0, 0, meta.z_cg_m [default 0.39 m]); the distance from the hold point is
  |CG_xy - hold_xy| (the sim's station-keeping definition: true CG against the hold point).

Frames: world z up, origin = the pad's top = the duct exit plane at liftoff.
``pos`` is the vehicle's body origin = the centre of the duct exit plane (the
CadQuery STLs' Z = 0, ``params.meta.coordinate_system``); a log that stores the
CG instead says ``meta.pos_ref = "cg"`` and gives ``meta.z_cg_m``.

Picture (1920 x 1080; other sizes scale): left 2/3 the 3-D chase view - vehicle
(printed modules + nose), 4 vanes, launch pad at the origin, textured ground at
``ground_z`` (1 m / 5 m / 10 m lines, distance rings around the pad), exhaust
plume (length ~ thrust, cut at the ground, splash disc), shadow, 3-D trail,
ground track, plumb line, wind (blue) + gust (red) arrows; HUD (time, mode,
phase, state, height + target + reference, v_z, tilt, throttle + saturation,
wind, gust, last ground command), event ticker.  Right 1/3: time plots (height /
target / reference, tilt, throttle, vane angles) with a moving cursor (future
dimmed), a top view of the drift track, and the vane close-up with the true
attitude and each vane's side force on the vehicle.  The camera (fixed azimuth
across the drift, 12 deg down) keeps vehicle + ground below it + pad in view
while that needs <= 70 m distance, then vehicle + ground, then the vehicle
(``--framing`` forces a level), smoothed zero-phase, never behind an overlay.
A ``SIMULATION - not flight footage`` tag is always on screen.

Speed (measured 2026-10-20, Intel UHD 750, 1920 x 1080, MSAA 4x, incl. H.264
encoding; v2 with the Korean overlays and the compass): framing vehicle 0.082,
all 0.086-0.092, demo 0.091-0.092 (with the person, pole and arrow), ground 0.096 s
per frame + ~2 s setup, i.e. a 30 fps real-time video costs ~2.5-2.9 s of wall time
per simulated second (v1: 0.077-0.082 s per frame).  The sidecar ``<out>.json``
records the measured ``s_per_frame`` of every render.

Vanes (SPEC 8.2): vane i sits at theta_i (``params.vane``), hinge axis radial at
Z = ``vane.hinge_z``.  delta_i > 0 is a side force on the vehicle along
+e_theta = (-sin theta, cos theta, 0): the trailing edge (below the hinge) moves
toward -e_theta and turns the jet that way.  ``vane_local_matrix`` therefore
rotates the vane about its radial hinge axis by -delta (right-hand rule about
+e_r); ``tools/test_sim_video.py`` checks the sign on all four vanes and against
an attitude correction of a real ``sim.sim6dof`` run.

v2 (2026-10-20): Korean overlays by default (``--lang ko``; ``--lang en`` keeps the English):
Malgun Gothic (C:/Windows/Fonts/malgun.ttf, malgunbd.ttf; else a Korean font found by
matplotlib's font manager) for PIL overlays, matplotlib plots and VTK's font-file text
property.  Hand push (push_* keys): a padded pole from the person to the contact point, a
force arrow + "밀기 12 N" label while active, a fading marker where it was pushed, push
intervals on the tilt plot and a per-push recovery table (peak tilt, recovery time with the
``sim.sim6dof.gust_recovery`` rule: time after the push END until the true tilt is back
below the band for the rest of the window = push end + 5 s, cut at the next push and at the
end of POWERED).  Wind: a compass rose (current mean wind + the last 10 s of its tip),
rotating wind arrows on the top view and a wind-direction plot when the direction varies.
``--framing demo``: close camera for a 1-2 m hover (vehicle large, ground + pad + person in
view, calm camera) with a 1.7 m reference figure "사람 1.7 m" at the pusher's position (or
beside the pad when there is no push), made translucent in any frame where it would
cover the vehicle.

v3 (2026-10-20): ``--style realistic`` (the default; ``--style schematic`` is the v2 look above) - scenery and
lighting from ``tools/render_scene.py``, all procedural (fixed seeds, nothing downloaded; generated images cached as
PNG in ``sim/out/video/_assets``, ``--assets``): PBR materials (printed parts matte Bambu PLA Matte colours - a chosen
palette, the project names none -, metallic pole), a generated equirectangular sky (gradient, sun disk + glow, soft
clouds, horizon haze, hills and a tree line, hazy far field below the horizon) as the visible background and as the
image-based-lighting environment, a warm directional sun; the vehicle's, the person's and the pad's shadows are the
blurred union of their triangles projected along the sun onto the ground (exact silhouette for a flat ground,
penumbra growing with height, a contact blob near the ground; VTK's shadow-map pass was tried and dropped: it does
not combine with the skybox / translucent jet here, and its light frustum cannot cover a 100 m drift at a useful
texel size).  ``meta.profile == 'demo'`` (or a scenario / title saying demo, or ``--scene hall``) gives an indoor sports
hall (maple strip floor, court lines, padded walls, high windows, ceiling light panels), else an outdoor field (mown
grass with 2 m stripes, a 2 m concrete launch slab, chalk distance rings near the pad, aerial-perspective fog on the
ground).  The jet is a few nested translucent cones (alpha falling along and across the jet, deterministic flicker),
nearly invisible like a real cold-air jet; where it reaches the ground (strength = thrust fraction x
(1 - h / 2.2 m)^1.5) a ripple disc (grass flattened by the wall jet / a faint pressure ring indoors) and grit / dust
particles from a fixed-seed RNG.  The person (v4, 2026-10-21) is a rigged anatomical 1.75 m human (CC0 Blender Studio
base mesh, smooth pale blue-grey satin, label "사람 1.75 m"; tools/human_figure.py: numpy LBS + IK over
render/human/human_lbs.npz): idle with the weight on one leg, head looking at the vehicle, walking between push
positions, and during a push both fists closed on the pole (IK onto POLE_GRIP_M along it), front foot forward,
leaning in - blended over PERSON_BLEND_S before / after each push; its shadow is its posed 2.5k-triangle shadow mesh.
Without that npz the earlier jointed capsule mannequin is drawn (with a warning).  ``--camera spectator``: a person filming from eye height 1.6 m, a few metres from the pad, panning /
tilting / zooming to follow the vehicle with a slight deterministic shake (else the chase camera, in the realistic
style with a 38 deg view and a calmer follow).  ``--hud full|minimal|none``: full = as above; minimal = the 3-D view
over the whole frame with the title and a compact HUD; none = the clean 3-D view.  The simulation tag is drawn in every
mode.  ``--motion-blur N``: the 3-D view averaged over N sub-frames (half a frame interval), N x slower.
Speed (measured 2026-10-20, Intel UHD 750, 1920 x 1080, MSAA 8x in the realistic style, incl. H.264): see the
sidecar's ``s_per_frame``; realistic full HUD ~0.16-0.2 s per frame, schematic ~0.09.

v5 (2026-10-21, v0.8 GPS position hold): the GPS antenna pod (``cad/exports/cadquery/stl/gps_pod*.stl`` or any
``*gps*.stl`` there, PLA Matte Black, moving with the body; without it a labelled stand-in box at the option-B place
of the v0.8 CAD study, with a warning) on logs of a vehicle that carries the GPS (``--gps-pod auto``: any GPS key or
meta.gps_on); a hold-point marker on the ground (a thin ring + cross, light blue) and a thin line from it to the
vehicle's ground point (CG below the vehicle); HUD rows "유지점" (distance from the hold point) and "GPS" (fix,
satellites, hAcc; red when GPS is not usable); the position loop's tilt setpoint dashed in the tilt plot; a
"distance from the hold point" plot; the hold point, the line and the NAV estimate in the top view; the title / end
cards state the GPS caveats (open sky only - no position hold indoors; ~2 m wander; quoted ~1 m RMS / 2.5 m max until
a real module is measured) and the station keeping read from the log.  A GPS log picks the outdoor field in
``--scene auto`` (GPS needs open sky; ``--scene hall`` with GPS warns).  ``GhostVehicle``: a translucent second
vehicle in the same scene (used by tools/compare_video.py ``--layout shared``).

v6 (2026-10-28, v0.9 hardware): the renderer draws the vehicle the run flew.  Log keys (all optional; each may be a
top-level npz key or a meta key of the same name, the npz key wins):
    hardware_config  'rocket' | 'vtol'.  ABSENT (every log written before v0.9, history/ runs): the pre-v0.9 drawing,
                     byte for byte (BODY_PARTS + vane + pad from ``cad/exports/cadquery/stl``, the v0.8 GPS pod) - the
                     legacy set.  'rocket': the same STL set plus the v0.9 parts outside the body (the four direct-drive
                     servo covers on the fin roots, the motor-lead groove cover).  'vtol': ``stl_vtol`` (power 'battery'),
                     ``stl_vtol_tether`` / ``stl_vtol_hybrid`` (the power-source variants; ``stl_vtol`` with a warning when
                     the variant directory is missing): propulsion (solid fins), intake, avionics, top cap, groove cover,
                     servo covers; on the top cap the GPS ground plane, patch and board and the arming toggle (boxes /
                     cylinders at params ``configs.vtol.top_cap`` places, not CAD meshes); tether / hybrid: the M5 eye bolt
                     (eye OD / height params ``top_cap.tether``, ASSUMED there).  The launch pad is drawn for a VTOL log only
                     when ``ground_z`` < -5 mm (the sim's pad top at z = 0 above the ground).
    power_source     'battery' | 'tether' | 'hybrid' (aliases 'wired' / 'wired_buffer'); with power_source tether / hybrid
                     and no hardware_config the config is taken as 'vtol' (warning).
    tether_anchor_m [3] world m (same origin as pos): the fixed anchor = the pulley's lowest point on the overhead boom.
    tether_attach_body_m [3] body frame m, relative to the body origin of ``pos`` (exit-plane centre; with meta.pos_ref
                     'cg' relative to the CG): the top-cap hardpoint the tether is tied to.
    tether_tension_N [N] N at the vehicle end    tether_connected [N] bool (mechanical link to the vehicle)
    supply_v_V / supply_i_A [N] (aliases v_bus_V, v_esc_V, v_tether_V / i_supply_A, i_tether_A)   supply_state [N] int
                     (0 normal, 1 current limit, 2 off / lost)   buffer_v_V / buffer_i_A (discharge > 0) / buffer_soc [N]
                     (0..1)   v_cell [N] V per cell of the flight pack (HUD 'battery' row, non-legacy logs only)
    meta.tether_w_n_per_m  cable weight per metre (default TETHER_W_N_PER_M, ASSUMED: 2 x 10 AWG silicone + Dyneema)
  Tether drawing (power tether / hybrid): a light-stand boom (tripod with sandbags, column, a 1.5 m boom over the anchor -
  docs/유선전원_검토.md "기둥 3.5 m + 붐 1.5 m", the stand placed behind-right of the first camera azimuth), the pulley at
  the anchor, the cable along the boom and down the column to the supply box on the ground; the tether from the anchor
  to the hardpoint as a catenary through both points whose tension at the vehicle end is the logged one (taut branch;
  below the smallest feasible tension the shape of that tension; vertical = straight), with the bungee section (0.4 m
  ASSUMED + T / 600 N/m) and the spring boot (eye top to plate + boot_len 100 mm, the CAD keep-out check's boot top) at
  the vehicle end; while ``tether_connected`` is False the free cable
  is a deterministic chain simulation (Verlet + length constraints + ground contact) started from the drawn shape at the
  disconnect with the vehicle's velocity at its end.  Without tether keys on a tether / hybrid log: anchor 3.8 m above
  the first position, attach point = the CAD hardpoint, tension 1 N (all ASSUMED, warning).  ``--framing all`` on a
  tether log also keeps the rig in view (the pulley and the stand's foot, when the camera distance allows; the other
  framings are unchanged).  HUD rows (when present):
  테더 (tension, connected), 전원 (supply V / A / state), 버퍼 (V, V per cell, A, state), 배터리 (V per cell).

Usage:
  python tools/sim_video.py --log LOG.npz --out OUT.mp4 [--fps 30] [--size 1920x1080] [--speed 1]
                            [--start S] [--end S] [--preview N] [--at t1,t2,...] [--lang ko|en]
                            [--framing all|ground|vehicle|demo] [--figure auto|on|off]
                            [--style realistic|schematic] [--hud full|minimal|none] [--camera chase|spectator]
                            [--motion-blur N] [--scene auto|field|hall] [--assets DIR] [--gps-pod auto|on|off]
  python tools/sim_video.py --export-dev S2 [--log sim/out/video/dev_S2.npz] [--out ...]
      (development log from an existing ballistic scenario, mapped onto the contract keys)

Invariant: the renderer reads only the log, ``params.json`` (vane geometry, duct
radius, v0.9 top-cap item places) and the CAD STLs; when an STL is missing it draws a labelled stand-in
and says so in the warnings, never silently.  A log without the v0.9 keys renders exactly as before v6.
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import sys
import textwrap
import time
from dataclasses import dataclass, field
from typing import Any, Callable, Sequence

import numpy as np

TOOLS_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_ROOT = os.environ.get("SIM_VIDEO_ROOT") or os.path.dirname(TOOLS_DIR)
STL_DIR = os.path.join(PROJECT_ROOT, "cad", "exports", "cadquery", "stl")
OUT_DIR = os.path.join(PROJECT_ROOT, "sim", "out", "video")
PARAMS_PATH = os.path.join(PROJECT_ROOT, "params.json")
ASSET_DIR = os.path.join(OUT_DIR, "_assets")          # generated sky / texture PNGs (deterministic, a cache)
STYLES = ("realistic", "schematic")
GPS_POD_MODES = ("auto", "on", "off")
HUDS = ("full", "minimal", "none")
CAMERAS = ("chase", "spectator")
SCENES = ("auto", "field", "hall")


def _render_scene() -> Any:
    """The realistic-style helper module (tools/render_scene.py), imported on first use from this file's directory
    (the module may itself be loaded by path, e.g. from sim/test_vtol_scenarios.py)."""
    if TOOLS_DIR not in sys.path:
        sys.path.insert(0, TOOLS_DIR)
    import render_scene
    return render_scene


def _human_figure() -> Any:
    """The rigged-human runtime (tools/human_figure.py: LBS + IK over render/human/human_lbs.npz), imported on first
    use from this file's directory."""
    if TOOLS_DIR not in sys.path:
        sys.path.insert(0, TOOLS_DIR)
    import human_figure
    return human_figure


def scene_profile(meta: dict[str, Any], scene: str = "auto", gps: bool = False) -> str:
    """'hall' (indoor sports hall) for meta.profile == 'demo' or a scenario / title naming a demo, else 'field';
    ``scene`` forces one.  ``gps`` (v0.8: the vehicle holds position with GPS): always 'field' in auto - GPS needs
    open sky, a GPS demo is flown outdoors (params mission.vtol.position_hold.demo_rules)."""
    if scene not in SCENES:
        raise ValueError("scene must be " + " | ".join(SCENES))
    if scene != "auto":
        return scene
    if gps:
        return "field"
    if str(meta.get("profile", "")).lower() == "demo":
        return "hall"
    words = (str(meta.get("scenario", "")) + " " + str(meta.get("title", ""))).lower()
    return "hall" if ("demo" in words.replace("_", " ").split() or "indoor" in words) else "field"

STATE_NAMES = {0: "SAFE", 1: "ARMED", 2: "POWERED", 3: "UNPOWERED", 4: "RELEASED", 5: "ABORT"}
PHASE_NAMES = {0: "NONE", 1: "SPOOL", 2: "CLIMB", 3: "HOLD", 4: "LAND", 5: "LANDED"}
STATE_POWERED = 2
META_KEYS = ("scenario", "title", "seed", "mode", "params_sha256", "notes", "pos_ref", "z_cg_m", "t_max_n",
             "limits", "poster_t", "state_names", "phase_names", "source")

# printed modules + nose drawn as the vehicle body (CadQuery exports, vehicle frame, mm, Z = 0 at the duct exit)
BODY_PARTS: tuple[tuple[str, str], ...] = (
    ("propulsion_module", "#5b6573"),
    ("intake_module", "#aab4c0"),
    ("avionics_module", "#e8eaed"),
    ("payload_module", "#f2a541"),
    ("nose_cone", "#d64535"),
)
PAD_PART = ("launch_pad", "#6f6a62")
VANE_PART = "vane"
VANE_COLORS = ("#2a78d6", "#eb6834", "#1baf7a", "#eda100")   # run_scenarios.PALETTE slots 1-4
INK, INK2, MUTED, GRID, SURFACE = "#0b0b0b", "#52514e", "#898781", "#e1e0d9", "#fcfcfb"
SKY_TOP, SKY_BOTTOM = "#8fbbe6", "#e9f1f8"
GROUND, GRID_MINOR, GRID_MAJOR, RING = "#dde3d1", "#b8c1a9", "#8f9a80", "#5f7f9c"
WIND_C, GUST_C, PLUME_C, TRACK_C = "#2a78d6", "#e34948", "#ffb066", "#4a3aa7"
PHASE_COLORS = {0: "#ffffff", 1: "#fde68a", 2: "#bfdbfe", 3: "#bbf7d0", 4: "#fed7aa", 5: "#e5e7eb"}
STATE_COLORS = {0: "#ffffff", 1: "#fde68a", 2: "#bfdbfe", 3: "#e5e7eb", 4: "#ddd6fe", 5: "#fecaca"}
TONES = {"normal": (235, 238, 242), "muted": (160, 168, 178), "ok": (110, 220, 160), "warn": (245, 190, 60),
         "alert": (255, 105, 100), "accent": (140, 190, 255)}
PUSH_C, POLE_C, PAD_FOAM_C, FIGURE_C = "#d9480f", "#8a6a45", "#ffd43b", "#7d8fa6"
PUSH_EPS_N = 0.05                       # |push_force| above this = pushing (when push_active is absent)
PUSH_ARROW_M_PER_N = 0.035              # force arrow length per newton (12 N -> 0.42 m)
PUSH_AFTERGLOW_S = 1.5                  # the "pushed here" marker fades over this time after a push
PUSH_WINDOW_S = 5.0                     # recovery is looked for this long after a push (sim6dof.GUST_RECOVERY_WINDOW_S)
WIND_TRACE_S = 10.0                     # compass: the last 10 s of the mean-wind tip
WIND_SWING_MIN_DEG = 20.0               # wind-direction plot when the mean direction spans more than this
WIND_SWING_MIN_MPS = 0.3                # ... counted only while the mean wind is at least this strong

# figure (the 1.7 m human reference), figure frame: x toward the vehicle, z up, origin between the feet
FIGURE_HEIGHT_M = 1.70
FIGURE_HANDS = np.array([0.45, 0.0, 1.25])      # where the pole is held while pushing
POLE_REACH_M = 1.10                    # horizontal length of the pole between the hands and the contact point
FIGURE_IDLE_OFFSET = (-1.6, 0.6)        # no push: 1.6 m to the screen-left of the pad, 0.6 m beyond it
POLE_GRIP_M = (-0.12, 0.10)             # realistic person: rear / front hand along the pole from its held end (m)
POLE_BUTT_M = 0.25                      # realistic pole: length behind the held end
POLE_APPROACH_M = 0.25                  # realistic person: the pad backs off the body this far while the pole comes up

# v0.8 GPS position hold
HOLD_C, EST_C = "#4fb8f0", "#1baf7a"    # hold-point marker / error line (light blue), NAV position estimate (green)
HOLD_RING_M, HOLD_CROSS_M = 0.30, 0.42  # marker: ring radius, cross half-length (a subtle mark, not a painted line)
Z_CG_DEFAULT_M = 0.39                   # CG above the exit plane when meta.z_cg_m is absent (as push_point's default)
GPS_POD_NAMES = ("gps_pod", "gps_antenna_pod", "gps_pod_lid")      # tried first, then any *gps*.stl in STL_DIR
# stand-in pod (drawn only when no GPS STL exists, with a warning): the option-B place of the v0.8 CAD study
# (scratchpad v08/report_gps-cad-feasibility.json; params electronics.gps.mount): theta 192 deg, on the payload tube
# (outer radius 60 mm), 29.6 mm radial protrusion, half angle 18.5 deg at the body radius, z 679.8-700.3 mm
GPS_POD_STANDIN = {"theta_deg": 192.0, "r0_m": 0.057, "r1_m": 0.0896, "half_w_m": 0.0209, "z0_m": 0.6798, "z1_m": 0.7003}
# the GPS statements on the cards (sources: params electronics.gps.accuracy_note "the position WANDERS ~2 m ... GPS needs
# OPEN SKY - no position hold indoors"; mission.vtol.position_hold.review_v08_decisions (9) "quote station keeping as
# ~1 m RMS / 2.5 m max until a real module is measured")
GPS_QUOTE_RMS_M, GPS_QUOTE_MAX_M, GPS_WANDER_M = 1.0, 2.5, 2.0

# --------------------------------------------------------------------------- #
# language (overlays, plot labels, cards).  Code, identifiers and logs stay English.
# --------------------------------------------------------------------------- #
LANGS = ("ko", "en")
KO_FONT_FILES = (r"C:\Windows\Fonts\malgun.ttf", r"C:\Windows\Fonts\malgunbd.ttf")
KO_FONT_FAMILIES = ("Malgun Gothic", "NanumGothic", "Noto Sans CJK KR", "Noto Sans KR", "AppleGothic")
STATE_NAMES_KO = {0: "안전", 1: "준비", 2: "동력 비행", 3: "무동력", 4: "분리", 5: "비상정지"}
PHASE_NAMES_KO = {0: "대기", 1: "시동", 2: "상승", 3: "호버링", 4: "착륙", 5: "착지"}
CMD_GLOSS_KO = {"ON": "이륙", "OFF": "착륙", "UP": "고도 +", "DOWN": "고도 -", "KILL": "긴급정지", "ARM": "무장",
                "DISARM": "무장 해제", "LAUNCH": "발사", "ABORT": "중단"}
TXT: dict[str, dict[str, str]] = {
    "sim_tag": {"en": "SIMULATION - not flight footage", "ko": "시뮬레이션 - 실제 비행 영상 아님"},
    "standin": {"en": "STAND-IN GEOMETRY: CAD STLs missing", "ko": "대체 형상: CAD STL 파일 없음"},
    "standin_pod": {"en": "GPS POD: stand-in box (CAD STL missing)", "ko": "GPS 포드: 대체 형상 (CAD STL 없음)"},
    "TIME": {"en": "TIME", "ko": "시간"}, "MODE": {"en": "MODE", "ko": "모드"},
    "PHASE": {"en": "PHASE", "ko": "비행 단계"}, "STATE": {"en": "STATE", "ko": "상태"},
    "ALT": {"en": "ALT", "ko": "고도"}, "V_Z": {"en": "V_Z", "ko": "수직속도"}, "TILT": {"en": "TILT", "ko": "기울기"},
    "THR": {"en": "THR", "ko": "스로틀"}, "WIND": {"en": "WIND", "ko": "바람"}, "GUST": {"en": "GUST", "ko": "돌풍"},
    "PUSH": {"en": "PUSH", "ko": "밀기"}, "CMD": {"en": "CMD", "ko": "명령"},
    "target": {"en": "target", "ko": "목표"}, "ref": {"en": "ref", "ko": "기준"},
    "saturated": {"en": "SATURATED", "ko": "최대(포화)"}, "calm": {"en": "calm", "ko": "무풍"},
    "toward": {"en": "toward", "ko": "→"}, "active": {"en": "ACTIVE", "ko": "작동"},
    "none_yet": {"en": "none yet", "ko": "아직 없음"}, "ago": {"en": "ago", "ko": "전"},
    "mode_vtol": {"en": "VTOL", "ko": "수직이착륙 (VTOL)"}, "mode_ballistic": {"en": "BALLISTIC", "ko": "탄도 비행"},
    "push_label": {"en": "push {f:.0f} N", "ko": "밀기 {f:.0f} N"},
    "person": {"en": "person 1.7 m", "ko": "사람 1.7 m"},
    "person_real": {"en": "person 1.75 m", "ko": "사람 1.75 m"},     # the rigged realistic figure (1.75 m tall)
    "compass_title": {"en": "WIND (top view)", "ko": "바람 (위에서 본 방향)"},
    "compass_trace": {"en": "trace: last 10 s", "ko": "자취: 최근 10초"},
    "compass_smoothed": {"en": "1 s smoothed", "ko": "1초 평활값"},
    "telemetry": {"en": "TELEMETRY  (sim truth unless noted)", "ko": "원격 측정  (별도 표기 외에는 시뮬레이션 참값)"},
    "h_axis": {"en": "height of duct exit\nabove pad top [m]", "ko": "높이 [m]"},
    "h_truth": {"en": "height (truth)", "ko": "배기구 높이 (참값, 패드 윗면 기준)"}, "h_est": {"en": "onboard estimate", "ko": "기체 추정값"},
    "h_target": {"en": "target", "ko": "목표"}, "h_ref": {"en": "reference", "ko": "기준 궤적"},
    "ground": {"en": "ground", "ko": "지면"}, "ceiling": {"en": "ceiling", "ko": "상한 고도"},
    "geofence": {"en": "geofence", "ko": "지오펜스"},
    "tilt_axis": {"en": "tilt [deg]", "ko": "기울기 [°]"}, "tilt_truth": {"en": "tilt (truth)", "ko": "기울기 (참값)"},
    "abort_lim": {"en": "abort {v:g} deg", "ko": "비상정지 {v:g}°"}, "band": {"en": "band {v:g} deg", "ko": "{v:g}° 기준"},
    "push_span": {"en": "push", "ko": "밀기"},
    "thr_axis": {"en": "throttle", "ko": "스로틀"}, "thr_cmd": {"en": "throttle cmd", "ko": "스로틀 명령"},
    "at_umax": {"en": "at u_max", "ko": "u_max 도달"}, "hover": {"en": "hover {v:.2f}", "ko": "호버링 {v:.2f}"},
    "vane_axis": {"en": "vane angle [deg]", "ko": "베인 각도 [°]"}, "vane_n": {"en": "vane {i}", "ko": "베인 {i}"},
    "wdir_axis": {"en": "wind toward [deg]", "ko": "풍향 [°]"},
    "wdir_mean": {"en": "mean wind direction (toward)", "ko": "평균 바람 방향 (바람이 향하는 쪽)"},
    "time_axis": {"en": "sim time [s]", "ko": "시뮬레이션 시간 [s]"},
    "bg_phase": {"en": "background = phase", "ko": "배경 = 비행 단계"},
    "bg_state": {"en": "background = state", "ko": "배경 = 상태"},
    "top_view": {"en": "TOP VIEW", "ko": "위에서 본 모습"}, "drift_sub": {"en": "drift from the pad", "ko": "패드로부터 이동"},
    "drift_val": {"en": "drift {d:6.2f} m", "ko": "이동 {d:6.2f} m"}, "wind_val": {"en": "wind {w:4.1f} m/s", "ko": "바람 {w:4.1f} m/s"},
    "gust_val": {"en": "gust +{g:3.1f}", "ko": "돌풍 +{g:3.1f}"}, "cam": {"en": "cam", "ko": "카메라"},
    "vanes_title": {"en": "VANES  (true attitude, seen from below)", "ko": "베인  (실제 자세, 아래에서 본 모습)"},
    "vanes_sub": {"en": "arrows: force on vehicle, black = sum (x0.5)", "ko": "화살표: 기체에 작용하는 힘, 검정 = 합력 (x0.5)"},
    "events": {"en": "EVENTS / GROUND COMMANDS", "ko": "이벤트 / 지상 명령"},
    "ground_cmd": {"en": "GROUND -> {c}", "ko": "지상국 → {c}"},
    "push_ev": {"en": "push #{i} start ({f:.1f} N)", "ko": "밀기 #{i} 시작 ({f:.1f} N)"},
    "push_table": {"en": "PUSH RESPONSE  (true tilt; recovery = after the push ends, band {b:g} deg)",
                   "ko": "밀기 반응  (실제 기울기, 복귀 = 밀기가 끝난 뒤 {b:g}° 이내로 돌아온 시간)"},
    "push_row": {"en": "#{i} {f:4.1f} N {d:.2f} s   peak {p:4.1f} deg   {rec}   moved {m:.2f} m",
                 "ko": "#{i}  {f:4.1f} N · {d:.2f} s   최대 {p:4.1f}°   {rec}   이동 {m:.2f} m"},
    "rec_ok": {"en": "back in {r:.2f} s", "ko": "{r:.2f} s 만에 복귀"},
    "rec_never_out": {"en": "stayed in band", "ko": "기준 이내 유지"},
    "rec_none": {"en": "NOT back in window", "ko": "구간 내 미복귀"},
    "rec_pending": {"en": "recovering...", "ko": "복귀 중..."},
    # v0.8 GPS position hold
    "HOLD": {"en": "HOLD PT", "ko": "유지점"}, "GPS": {"en": "GPS", "ko": "GPS"},
    "hold_val": {"en": "{d:5.2f} m from the hold point", "ko": "{d:5.2f} m 떨어짐"},
    "hold_unset": {"en": "not set yet (end of the climb)", "ko": "아직 없음 (상승이 끝나면 설정)"},
    "gps_fix3": {"en": "3D fix", "ko": "3D 고정"}, "gps_fix2": {"en": "2D fix", "ko": "2D 고정"},
    "gps_nofix": {"en": "NO FIX", "ko": "신호 없음"}, "gps_usable": {"en": "usable", "ko": "사용 가능"},
    "gps_unusable": {"en": "NOT USABLE", "ko": "사용 불가"},
    "gps_sats": {"en": "{n} sats", "ko": "위성 {n}"}, "gps_hacc": {"en": "hAcc {a:.1f} m", "ko": "정확도 {a:.1f} m"},
    "gps_lost_hint": {"en": "attitude only", "ko": "자세만 유지"},
    "tilt_sp": {"en": "position-loop tilt setpoint", "ko": "위치 제어 기울기 목표"},
    "hold_axis": {"en": "hold pt [m]", "ko": "유지점 [m]"},
    "vane_axis_short": {"en": "vane [deg]", "ko": "베인 [°]"},
    "hold_dist_l": {"en": "distance from the hold point (truth, CG)", "ko": "유지점까지 거리 (참값, 무게중심)"},
    "hold_crit": {"en": "criterion {v:g} m", "ko": "기준 {v:g} m"},
    "gps_lost_span": {"en": "GPS not usable", "ko": "GPS 사용 불가"},
    "hold_pt_short": {"en": "hold pt {d:5.2f} m", "ko": "유지점 {d:5.2f} m"},
    "est_short": {"en": "o = NAV's belief (vs hold point)", "ko": "o = 기체가 아는 위치 (유지점 기준)"},
    # v0.9 hardware (v6): the vehicle the run flew, the tether and the power rows
    "TETHER": {"en": "TETHER", "ko": "테더"}, "SUPPLY": {"en": "SUPPLY", "ko": "전원"},
    "BUFFER": {"en": "BUFFER", "ko": "버퍼"}, "BATT": {"en": "BATTERY", "ko": "배터리"},
    "teth_conn": {"en": "connected", "ko": "연결"}, "teth_off": {"en": "DISCONNECTED", "ko": "분리됨"},
    "teth_tension": {"en": "tension {f:4.1f} N", "ko": "장력 {f:4.1f} N"},
    "sup_cl": {"en": "CURRENT LIMIT", "ko": "전류 제한"}, "sup_off": {"en": "SUPPLY OFF", "ko": "전원 끊김"},
    "buf_idle": {"en": "idle", "ko": "대기"}, "buf_dis": {"en": "discharging", "ko": "방전 중"},
    "buf_chg": {"en": "CHARGING", "ko": "충전(이상)"},
    "per_cell": {"en": "{v:.2f} V/cell", "ko": "셀당 {v:.2f} V"},
    "hw_rocket": {"en": "rocket configuration (v0.9)", "ko": "로켓 구성 (v0.9)"},
    "hw_vtol": {"en": "VTOL configuration (v0.9)", "ko": "VTOL 구성 (v0.9)"},
    "pw_battery": {"en": "battery", "ko": "배터리 전원"}, "pw_tether": {"en": "tether, no battery", "ko": "유선 전원 (배터리 없음)"},
    "pw_hybrid": {"en": "tether + 6S 650 mAh buffer", "ko": "유선 + 6S 650 mAh 버퍼 (하이브리드)"},
    "tether_label": {"en": "tether", "ko": "테더"},
}


def tr(key: str, lang: str = "en", **fmt: Any) -> str:
    """The text of ``key`` in ``lang`` (English when the key has no translation), formatted with ``fmt``."""
    d = TXT.get(key)
    s = key if d is None else d.get(lang, d["en"])
    return s.format(**fmt) if fmt else s


def cmd_label(name: str, lang: str = "en") -> str:
    """A ground command as shown: the button name, with a Korean gloss in Korean (``UP (고도 +)``)."""
    g = CMD_GLOSS_KO.get(str(name).upper())
    return f"{name} ({g})" if lang == "ko" and g else str(name)


def event_gloss_ko(text: str) -> str:
    """A short Korean reading of a logged event (supervisor / ground-station / sim lines), '' when not recognised.
    The logged English text is always shown next to it: this is a reading aid, not a translation."""
    import re
    s = str(text)
    names = {**{v: STATE_NAMES_KO[k] for k, v in STATE_NAMES.items()}, **{v: PHASE_NAMES_KO[k] for k, v in PHASE_NAMES.items()}}

    def ko(w: str) -> str:
        return names.get(w, w)
    m = re.match(r"GS phase (\S+) -> (\S+)", s)
    if m:
        return f"지상국: 단계 {ko(m.group(1))} → {ko(m.group(2))}"
    m = re.match(r"GS target (\S+) -> (\S+) m", s)
    if m:
        return f"지상국: 목표 고도 {m.group(1)} → {m.group(2)} m"
    m = re.match(r"GS (TX|ACK|queue) (\w+)", s)
    if m:
        verb = {"TX": "송신", "ACK": "수신 확인", "queue": "조작자 입력"}[m.group(1)]
        rej = " (거부)" if "rejected" in s else ""
        return f"지상국 {verb}: {cmd_label(m.group(2), 'ko')}{rej}"
    if s.startswith("GS "):
        return "지상국"
    m = re.match(r"(\w+): phase (\w+) -> (\w+)", s)
    if m:
        return f"단계 {ko(m.group(2))} → {ko(m.group(3))}"
    m = re.match(r"(\w+) -> (\w+)", s)
    if m and m.group(1) in STATE_NAMES.values() and m.group(2) in STATE_NAMES.values():
        extra = " (착지 확인)" if "touchdown" in s else ""
        return f"{ko(m.group(1))} → {ko(m.group(2))}{extra}"
    m = re.match(r"\w+: (UP|DOWN) \(seq \d+\) target (\S+) -> (\S+) m", s)
    if m:
        return f"{cmd_label(m.group(1), 'ko')}: 목표 {m.group(2)} → {m.group(3)} m"
    m = re.match(r"REJECT (\w+)", s)
    if m:
        return f"명령 거부: {cmd_label(m.group(1), 'ko')}"
    m = re.match(r"touchdown #(\d+)", s)
    if m:
        return f"착지 #{m.group(1)}"
    if s.startswith("lift-off again"):
        return "다시 이륙"
    if s.startswith("liftoff"):
        return "이륙"
    if s.startswith("ejection"):
        return "낙하산 사출"
    if s.startswith("FAULT"):
        return "고장 주입"
    low = s.lower()
    if "fence" in low:
        return "위치 펜스 초과"
    if "hold point" in low or "hold_point" in low:
        return "유지점"
    if "take-off check" in low or "takeoff check" in low:
        return "이륙 점검"
    if "gps" in low:
        if "lost" in low or "loss" in low or "timeout" in low:
            return "GPS 신호 상실"
        if "plausib" in low:
            return "GPS-관성 불일치"
        return "GPS"
    if "push" in s.lower():
        return "밀기"
    if "gust" in s.lower():
        return "돌풍"
    return ""


def korean_font_path(bold: bool = False) -> str | None:
    """Malgun Gothic from C:/Windows/Fonts, else a Korean family matplotlib's font manager finds, else None."""
    p = KO_FONT_FILES[1 if bold else 0]
    if os.path.exists(p):
        return p
    if bold and os.path.exists(KO_FONT_FILES[0]):
        return KO_FONT_FILES[0]
    from matplotlib import font_manager
    for fam in KO_FONT_FAMILIES:
        try:
            return font_manager.findfont(font_manager.FontProperties(family=fam, weight="bold" if bold else "normal"),
                                         fallback_to_default=False)
        except ValueError:
            continue
    return None


def mpl_korean_family() -> str | None:
    """Register the Korean font with matplotlib's font manager; returns its family name (None: not found)."""
    from matplotlib import font_manager
    name = None
    for bold in (False, True):
        p = korean_font_path(bold)
        if p is None:
            continue
        try:
            font_manager.fontManager.addfont(p)
        except (OSError, RuntimeError):
            continue
        if name is None:
            name = font_manager.FontProperties(fname=p).get_name()
    return name


def has_hangul(s: str) -> bool:
    return any(0xAC00 <= ord(ch) <= 0xD7A3 or 0x3130 <= ord(ch) <= 0x318F for ch in str(s))


def _rel(path: str) -> str:
    """Path relative to the project when possible (another drive on Windows: absolute)."""
    try:
        return os.path.relpath(path, PROJECT_ROOT)
    except ValueError:
        return os.path.abspath(path)


# --------------------------------------------------------------------------- #
# quaternions / transforms (pure numpy, vectorised over leading axes)
# --------------------------------------------------------------------------- #
def quat_normalize(q: np.ndarray) -> np.ndarray:
    q = np.asarray(q, dtype=float)
    n = np.linalg.norm(q, axis=-1, keepdims=True)
    return q / np.where(n < 1e-12, 1.0, n)


def quat_to_matrix(q: np.ndarray) -> np.ndarray:
    """R(q) body -> world for q = (w, x, y, z), shape (..., 4) -> (..., 3, 3) (``sim.quat.to_matrix``)."""
    q = np.asarray(q, dtype=float)
    w, x, y, z = q[..., 0], q[..., 1], q[..., 2], q[..., 3]
    ww, xx, yy, zz = w * w, x * x, y * y, z * z
    R = np.empty(q.shape[:-1] + (3, 3))
    R[..., 0, 0] = ww + xx - yy - zz
    R[..., 0, 1] = 2.0 * (x * y - w * z)
    R[..., 0, 2] = 2.0 * (x * z + w * y)
    R[..., 1, 0] = 2.0 * (x * y + w * z)
    R[..., 1, 1] = ww - xx + yy - zz
    R[..., 1, 2] = 2.0 * (y * z - w * x)
    R[..., 2, 0] = 2.0 * (x * z - w * y)
    R[..., 2, 1] = 2.0 * (y * z + w * x)
    R[..., 2, 2] = ww - xx - yy + zz
    return R


def slerp(q0: np.ndarray, q1: np.ndarray, s: np.ndarray | float) -> np.ndarray:
    """Spherical linear interpolation, shortest arc (q and -q are the same rotation), unit result.
    Shapes broadcast: q0, q1 (..., 4), s (...)."""
    q0 = quat_normalize(q0)
    q1 = quat_normalize(q1)
    s = np.asarray(s, dtype=float)[..., None]
    dot = np.sum(q0 * q1, axis=-1, keepdims=True)
    q1 = np.where(dot < 0.0, -q1, q1)
    dot = np.abs(dot)
    near = dot > 0.9995
    theta = np.arccos(np.clip(dot, -1.0, 1.0))
    sin_t = np.sin(theta)
    sin_t = np.where(near, 1.0, sin_t)
    a = np.where(near, 1.0 - s, np.sin((1.0 - s) * theta) / sin_t)
    b = np.where(near, s, np.sin(s * theta) / sin_t)
    return quat_normalize(a * q0 + b * q1)


def tilt_deg_of(q: np.ndarray) -> np.ndarray:
    """Angle between body z and world up (``sim.quat.tilt``), degrees."""
    q = np.asarray(q, dtype=float)
    c = 1.0 - 2.0 * (q[..., 1] ** 2 + q[..., 2] ** 2)
    return np.degrees(np.arccos(np.clip(c, -1.0, 1.0)))


def rot_x(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def rot_z(a: float) -> np.ndarray:
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])


def homogeneous(R: np.ndarray | None = None, t: Sequence[float] | None = None) -> np.ndarray:
    M = np.eye(4)
    if R is not None:
        M[:3, :3] = R
    if t is not None:
        M[:3, 3] = np.asarray(t, dtype=float)
    return M


def body_matrix(pos: np.ndarray, q: np.ndarray) -> np.ndarray:
    """4x4 body (m) -> world: x_w = R(q) x_b + pos."""
    return homogeneous(quat_to_matrix(q), pos)


def e_theta(theta_rad: float) -> np.ndarray:
    """SPEC 8.2: the tangential unit vector of the vane at theta (positive delta = force along it)."""
    return np.array([-math.sin(theta_rad), math.cos(theta_rad), 0.0])


def vane_local_matrix(theta_rad: float, delta_rad: float, hinge_z_m: float, theta_stl_rad: float = 0.0) -> np.ndarray:
    """4x4 that carries the vane STL (lying along theta_stl, metres) to vane theta deflected by delta.

    Rz(-theta_stl) brings the STL onto +x, the hinge rotation about +x (the radial hinge axis at
    Z = hinge_z) by **-delta** moves the trailing edge (below the hinge) toward -e_theta for delta > 0
    (SPEC 8.2: the jet turns toward the trailing edge, the vehicle feels +e_theta), Rz(theta) places it.
    """
    T_up = homogeneous(None, (0.0, 0.0, hinge_z_m))
    T_dn = homogeneous(None, (0.0, 0.0, -hinge_z_m))
    return (homogeneous(rot_z(theta_rad)) @ T_up @ homogeneous(rot_x(-delta_rad)) @ T_dn
            @ homogeneous(rot_z(-theta_stl_rad)))


def apply_matrix(M: np.ndarray, pts: np.ndarray) -> np.ndarray:
    pts = np.asarray(pts, dtype=float)
    return pts @ M[:3, :3].T + M[:3, 3]


# --------------------------------------------------------------------------- #
# geometry constants from params.json
# --------------------------------------------------------------------------- #
@dataclass(frozen=True)
class VehicleGeom:
    vane_thetas_rad: tuple[float, ...] = (0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi)
    hinge_z_m: float = 0.037
    duct_r_m: float = 0.045
    vane_force_r_m: float = 0.028
    delta_cmd_max_rad: float = math.radians(20.0)
    pad_height_m: float = 0.110            # plate + legs + feet below the pad top
    source: str = "defaults"

    @classmethod
    def from_params(cls, path: str = PARAMS_PATH) -> "VehicleGeom":
        try:
            with open(path, "r", encoding="utf-8") as f:
                p = json.load(f)
            v = p["vane"]
            n = int(v["count"])
            thetas = tuple(math.radians(float(v["theta0_deg"]) + i * 360.0 / n) for i in range(n))
            duct_id = float(p.get("propulsion_module", {}).get("duct_id", 90.0))
            lp = p.get("launch_pad", {})
            pad_h = (float(lp.get("plate_t", 5.0)) + float(lp.get("leg_h", 100.0)) + float(lp.get("foot_t", 5.0))) * 1e-3
            return cls(thetas, float(v["hinge_z"]) * 1e-3, 0.5 * duct_id * 1e-3, float(v.get("force_centroid_r", 28.0)) * 1e-3,
                       math.radians(float(v.get("max_deflection_cmd_deg", 20.0))), pad_h, _rel(path))
        except (OSError, KeyError, ValueError, TypeError, json.JSONDecodeError):
            return cls()


# --------------------------------------------------------------------------- #
# v0.9 hardware configuration (v6): which vehicle the run flew
# --------------------------------------------------------------------------- #
HW_LEGACY, HW_ROCKET, HW_VTOL = "legacy", "rocket", "vtol"
HW_CONFIGS = (HW_ROCKET, HW_VTOL)
POWER_SOURCES = ("battery", "tether", "hybrid")
POWER_ALIASES = {"wired": "tether", "wired_buffer": "hybrid", "battery_less": "tether", "batteryless": "tether"}
VTOL_STL_DIRS = {"battery": "stl_vtol", "tether": "stl_vtol_tether", "hybrid": "stl_vtol_hybrid"}
VTOL_BODY_PARTS: tuple[tuple[str, str], ...] = (
    ("propulsion_module", "#5b6573"),
    ("intake_module", "#aab4c0"),
    ("avionics_module", "#e8eaed"),
    ("top_cap", "#d64535"),
)
V09_BODY_EXTRAS: tuple[tuple[str, str], ...] = (("wire_cover", "#8f9aa6"),        # motor-lead groove cover (outside)
                                                 ("avionics_tray", "#9aa3ad"))      # electronics tray (seen in the bay)
FIN_PART = ("servo_cover", "#4b5563")     # one STL (the fin of vane.stl), drawn at every vane theta (direct drive)
BUFFER_CELLS = 6                          # hybrid buffer: 6S 650 mAh (params power.source_note)
TETHER_W_N_PER_M = 1.35                   # ASSUMED cable weight: 2 x 10 AWG silicone (1050 x 0.08 mm tinned Cu, OD 5.5,
                                          # silicone 1.20 g/cm3 - tether report wire_model) ~ 2 x 69 g/m + Dyneema 2 mm
TETHER_DEFAULT_ANCHOR_H_M = 3.8           # ASSUMED (no tether keys): pulley 3.8 m above the first position (3.5-4.0 m)
TETHER_DEFAULT_TENSION_N = 1.0            # ASSUMED (no tension key): handler / balancer 0.5-2.0 N (tether report)
BUNGEE_LEN_M, BUNGEE_K_N_PER_M = 0.40, 600.0   # ASSUMED: 0.3-0.5 m bungee section at the vehicle end, 600 N/m
BOOM_LEN_M = 1.5                          # docs/유선전원_검토.md: "기둥 3.5 m + 붐 1.5 m" (column + boom)


@dataclass(frozen=True)
class Hardware:
    """The vehicle a log was flown with.  ``config`` 'legacy' = the log names none (every pre-v0.9 log): drawn exactly
    as before v6 (the rocket STL set, BODY_PARTS)."""
    config: str = HW_LEGACY
    power: str = "battery"
    source: str = "none"                  # 'none' | 'meta' | 'npz' | 'inferred'

    @property
    def legacy(self) -> bool:
        return self.config == HW_LEGACY

    @property
    def vtol(self) -> bool:
        return self.config == HW_VTOL

    @property
    def tethered(self) -> bool:
        return self.config == HW_VTOL and self.power in ("tether", "hybrid")

    def label(self, lang: str = "en") -> str:
        if self.legacy:
            return ""
        s = tr("hw_" + self.config, lang)
        return s + (" · " + tr("pw_" + self.power, lang) if self.vtol else "")


def _scalar_str(v: Any) -> str | None:
    if v is None:
        return None
    a = np.asarray(v)
    if a.size != 1:
        return None
    x = a.reshape(-1)[0]
    if isinstance(x, bytes):
        x = x.decode("utf-8", errors="replace")
    s = str(x).strip()
    return s or None


def hardware_of(meta: dict[str, Any], data: dict[str, Any] | None = None,
                warnings: list[str] | None = None) -> Hardware:
    """The ``Hardware`` of a log from ``hardware_config`` / ``power_source`` (npz key first, then meta).  No key: the
    legacy drawing.  power_source tether / hybrid without hardware_config: 'vtol' (the rocket is always battery
    powered).  Unknown values: a warning and the legacy drawing / 'battery'."""
    w = warnings if warnings is not None else []
    data = data or {}
    raw_c, src = _scalar_str(data.get("hardware_config")), "npz"
    if raw_c is None:
        raw_c, src = _scalar_str(meta.get("hardware_config")), "meta"
    raw_p = _scalar_str(data.get("power_source"))
    if raw_p is None:
        raw_p = _scalar_str(meta.get("power_source"))
    pw = None
    if raw_p is not None:
        pw = POWER_ALIASES.get(raw_p.lower(), raw_p.lower())
        if pw not in POWER_SOURCES:
            w.append(f"power_source {raw_p!r} not in {POWER_SOURCES}: drawn as 'battery'")
            pw = "battery"
    if raw_c is None:
        if pw in ("tether", "hybrid"):
            w.append(f"power_source {pw!r} without hardware_config: drawn as the VTOL configuration")
            return Hardware(HW_VTOL, pw, "inferred")
        return Hardware()
    cfg = raw_c.lower()
    if cfg not in HW_CONFIGS:
        w.append(f"hardware_config {raw_c!r} not in {HW_CONFIGS}: drawn as before v0.9 (legacy rocket set)")
        return Hardware()
    pw = pw or "battery"
    if cfg == HW_ROCKET and pw != "battery":
        w.append(f"hardware_config 'rocket' with power_source {pw!r}: the rocket is always battery powered")
        pw = "battery"
    return Hardware(cfg, pw, src)


# --------------------------------------------------------------------------- #
# log loading
# --------------------------------------------------------------------------- #
@dataclass
class SimLog:
    t: np.ndarray
    pos: np.ndarray
    vel: np.ndarray
    q: np.ndarray
    delta: np.ndarray
    throttle: np.ndarray
    thrust: np.ndarray
    state: np.ndarray
    phase: np.ndarray
    h_target: np.ndarray
    h_ref: np.ndarray
    h_est: np.ndarray
    tilt_deg: np.ndarray
    wind: np.ndarray
    gust_active: np.ndarray
    ground_z: float
    event_t: np.ndarray
    event_text: list[str]
    cmd_t: np.ndarray
    cmd_name: list[str]
    meta: dict[str, Any]
    warnings: list[str] = field(default_factory=list)
    source: str = ""
    sha256: str = ""
    push_force: np.ndarray | None = None      # (N, 3) or None (no push keys in the log)
    push_point: np.ndarray | None = None
    push_active: np.ndarray | None = None
    wind_mean: np.ndarray | None = None       # (N, 3), filled by load_log (from the log or smoothed)
    wind_mean_source: str = "smoothed"        # 'log' | 'smoothed'
    # v0.8 (all None when absent)
    hold_xy: np.ndarray | None = None         # (N, 2) the hold point per sample, NaN while there is none
    gps_ok: np.ndarray | None = None          # (N,) bool
    gps_sats: np.ndarray | None = None        # (N,) int
    gps_hacc: np.ndarray | None = None        # (N,) m
    gps_fix: np.ndarray | None = None         # (N,) int
    pos_est: np.ndarray | None = None         # (N, 3) NAV CG estimate
    tilt_sp_deg: np.ndarray | None = None     # (N,) deg
    # v0.9 (v6): the vehicle the run flew (hardware_of) and the tether / power keys (None when absent)
    hardware: "Hardware | None" = None
    tether_anchor: np.ndarray | None = None   # (3,) world m
    tether_attach_b: np.ndarray | None = None  # (3,) body frame m from the exit-plane centre
    tether_tension: np.ndarray | None = None  # (N,) N
    tether_connected: np.ndarray | None = None  # (N,) bool
    power: dict[str, np.ndarray] = field(default_factory=dict)   # supply_v / supply_i / supply_state / buffer_v /
                                                                 # buffer_i / buffer_soc / v_cell, each (N,)

    @property
    def n(self) -> int:
        return int(self.t.shape[0])

    @property
    def hw(self) -> "Hardware":
        """The hardware configuration (the legacy rocket drawing when the log names none)."""
        return self.hardware if self.hardware is not None else Hardware()

    @property
    def has_gps(self) -> bool:
        """The vehicle carries the GPS (v0.8): any GPS key in the log or meta.gps_on."""
        if any(a is not None for a in (self.gps_ok, self.gps_sats, self.gps_hacc, self.gps_fix, self.pos_est,
                                       self.hold_xy, self.tilt_sp_deg)):
            return True
        return bool(self.meta.get("gps_on", False))

    @property
    def has_hold(self) -> bool:
        return self.hold_xy is not None and bool(np.any(np.isfinite(self.hold_xy[:, 0])))

    @property
    def z_cg_m(self) -> float:
        return float(self.meta.get("z_cg_m", Z_CG_DEFAULT_M))

    def cg_xy(self) -> np.ndarray:
        """(N, 2) horizontal CG position: pos + R(q) (0, 0, z_cg)."""
        return cg_xy_of(self.pos, quat_to_matrix(self.q), self.z_cg_m)

    def hold_dist(self) -> np.ndarray | None:
        """(N,) |CG_xy - hold_xy| (NaN while no hold point), None without a hold point."""
        if not self.has_hold:
            return None
        return np.hypot(*(self.cg_xy() - self.hold_xy).T)

    def est_rel_xy(self) -> np.ndarray | None:
        """(N, 2) where the vehicle believes it is, drawn relative to the hold point: hold_xy + (pos_est - pos_est at
        the hold capture) - the NAV's absolute GPS offset (metres, common to the hold point and the estimate) cancels,
        what is left is the vehicle's own view of its distance from the hold point; its difference from the true CG
        is the drift of the GPS / NAV error since the capture.  NaN while no hold point; None without pos_est / hold."""
        if self.pos_est is None or not self.has_hold:
            return None
        cached = getattr(self, "_est_rel_cache", None)
        if cached is not None:
            return cached
        h = self.hold_xy
        out = np.full((self.n, 2), np.nan)
        fin = np.all(np.isfinite(h), axis=1)
        i = 0
        while i < self.n:
            if not fin[i]:
                i += 1
                continue
            j = i
            while j + 1 < self.n and fin[j + 1] and np.array_equal(h[j + 1], h[i]):
                j += 1
            ok = np.flatnonzero(np.all(np.isfinite(self.pos_est[i:j + 1, :2]), axis=1))
            if ok.size:
                e0 = self.pos_est[i + ok[0], :2]
                out[i:j + 1] = h[i] + (self.pos_est[i:j + 1, :2] - e0)
            i = j + 1
        self._est_rel_cache = out
        return out

    def station_keeping_limit(self) -> float | None:
        """meta.criteria.station_keeping_max_m (a number, or the entry of this log's profile in a {field, demo} dict)."""
        return criterion_value((self.meta.get("criteria") or {}).get("station_keeping_max_m"), self.meta.get("profile"))

    @property
    def mode(self) -> str:
        return str(self.meta.get("mode", "?"))

    @property
    def has_push(self) -> bool:
        return self.push_active is not None and bool(np.any(self.push_active))

    def state_name(self, code: int, lang: str = "en") -> str:
        names = self.meta.get("state_names") or {}
        if lang == "ko" and str(int(code)) not in names and int(code) in STATE_NAMES_KO:
            return STATE_NAMES_KO[int(code)]
        return str(names.get(str(int(code)), STATE_NAMES.get(int(code), f"S{int(code)}")))

    def phase_name(self, code: int, lang: str = "en") -> str:
        names = self.meta.get("phase_names") or {}
        if lang == "ko" and str(int(code)) not in names and int(code) in PHASE_NAMES_KO:
            return PHASE_NAMES_KO[int(code)]
        return str(names.get(str(int(code)), PHASE_NAMES.get(int(code), f"P{int(code)}")))

    @property
    def has_phase(self) -> bool:
        return bool(np.any(self.phase != 0))

    @property
    def u_max(self) -> float:
        lim = self.meta.get("limits") or {}
        return float(lim.get("u_max", 1.0))

    @property
    def t_ref_n(self) -> float:
        """Thrust that draws the full-length plume: meta.t_max_n, else the log maximum."""
        tm = self.meta.get("t_max_n")
        if tm:
            return float(tm)
        m = float(np.nanmax(self.thrust)) if self.n else 0.0
        return m if m > 1e-6 else 1.0


def cg_xy_of(pos: np.ndarray, R: np.ndarray, z_cg_m: float) -> np.ndarray:
    """(N, 2) horizontal CG: exit-plane centre + the body axis (third column of R) times z_cg."""
    return np.asarray(pos, dtype=float)[:, :2] + np.asarray(R, dtype=float)[:, :2, 2] * float(z_cg_m)


def criterion_value(v: Any, profile: Any = None) -> float | None:
    """A criterion given as a number or as {"field": .., "demo": ..} (the entry of ``profile``, else 'field', else the
    only one); None when absent / not a number."""
    if v is None:
        return None
    if isinstance(v, dict):
        for k in (str(profile) if profile else None, "field"):
            if k is not None and k in v:
                return criterion_value(v[k])
        return criterion_value(next(iter(v.values()))) if len(v) == 1 else None
    try:
        f = float(v)
    except (TypeError, ValueError):
        return None
    return f if math.isfinite(f) else None


def hold_xy_series(raw: np.ndarray | None, phase: np.ndarray, n: int) -> np.ndarray | None:
    """The hold point per sample (N, 2) from a [3] point (shown from the first HOLD sample on, from the start when the
    log has no HOLD) or an [N, 3] series (NaN = none); None without the key."""
    if raw is None:
        return None
    a = np.asarray(raw, dtype=float)
    if a.shape == (3,) or a.size == 3:
        out = np.full((n, 2), np.nan)
        if not np.all(np.isfinite(a.reshape(-1)[:2])):
            return out
        ih = np.flatnonzero(np.asarray(phase) == 3)
        i0 = int(ih[0]) if ih.size else 0
        out[i0:] = a.reshape(-1)[:2]
        return out
    if a.shape == (n, 3) or a.shape == (n, 2):
        out = a[:, :2].copy()
        out[~np.all(np.isfinite(out), axis=1)] = np.nan
        return out
    raise ValueError(f"hold_point has shape {a.shape}, expected (3,) or ({n}, 3)")


def _str_list(a: np.ndarray) -> list[str]:
    return [str(x) for x in np.atleast_1d(a).tolist()]


def _parse_meta(raw: Any, warnings: list[str]) -> dict[str, Any]:
    if raw is None:
        warnings.append("no meta in the log")
        return {}
    arr = np.asarray(raw)
    s = arr.item() if arr.shape == () else arr.reshape(-1)[0]
    if isinstance(s, bytes):
        s = s.decode("utf-8", errors="replace")
    try:
        m = json.loads(str(s))
        if not isinstance(m, dict):
            raise ValueError("meta is not a JSON object")
        return m
    except (ValueError, TypeError) as e:
        warnings.append(f"meta is not valid JSON ({e}); shown raw")
        return {"notes": str(s)}


def load_log(path: str) -> SimLog:
    """Load and validate a log of the contract above.  Raises ValueError on a log that cannot be drawn."""
    with open(path, "rb") as f:
        raw = f.read()
    sha = hashlib.sha256(raw).hexdigest()
    warnings: list[str] = []
    with np.load(path, allow_pickle=False) as d:
        data = {k: d[k] for k in d.files}
    for k in ("t", "pos", "q"):
        if k not in data:
            raise ValueError(f"log {path!r} lacks the required key {k!r}")
    t = np.asarray(data["t"], dtype=float).reshape(-1)
    n = t.shape[0]
    if n < 2:
        raise ValueError("log has fewer than 2 samples")
    pos = np.asarray(data["pos"], dtype=float)
    q = np.asarray(data["q"], dtype=float)
    if pos.shape != (n, 3) or q.shape != (n, 4):
        raise ValueError(f"pos {pos.shape} / q {q.shape} do not match t ({n})")
    meta = _parse_meta(data.get("meta"), warnings)

    def arr(key: str, shape: tuple[int, ...], default: Callable[[], np.ndarray], dtype: Any = float) -> np.ndarray:
        if key not in data:
            warnings.append(f"{key} missing -> default")
            return default()
        a = np.asarray(data[key]).astype(dtype)
        if a.shape != shape:
            if a.size == int(np.prod(shape)):
                a = a.reshape(shape)
            else:
                raise ValueError(f"{key} has shape {a.shape}, expected {shape}")
        return a

    if not np.all(np.isfinite(t)) or np.any(np.diff(t) <= 0.0):
        raise ValueError("t must be finite and strictly increasing")
    vel = arr("vel", (n, 3), lambda: np.gradient(pos, t, axis=0))
    delta = arr("delta", (n, 4), lambda: np.zeros((n, 4)))
    throttle = arr("throttle", (n,), lambda: np.zeros(n))
    thrust = arr("thrust", (n,), lambda: np.zeros(n))
    state = arr("state", (n,), lambda: np.zeros(n, dtype=int), dtype=int)
    phase = arr("phase", (n,), lambda: np.zeros(n, dtype=int), dtype=int)
    h_target = arr("h_target", (n,), lambda: np.full(n, np.nan))
    h_ref = arr("h_ref", (n,), lambda: np.full(n, np.nan))
    h_est = np.asarray(data["h_est"], dtype=float).reshape(n) if "h_est" in data else np.full(n, np.nan)
    wind = arr("wind", (n, 3), lambda: np.zeros((n, 3)))
    gust = arr("gust_active", (n,), lambda: np.zeros(n, dtype=bool), dtype=bool)
    if "ground_z" in data:
        ground_z = float(np.asarray(data["ground_z"], dtype=float).reshape(-1)[0])
    else:
        ground_z = -VehicleGeom.from_params().pad_height_m
        warnings.append(f"ground_z missing -> the pad's foot level {ground_z:.3f} m")
    if ground_z > 1e-9:
        warnings.append(f"ground_z = {ground_z:.3f} m is above the pad top (contract: <= 0)")

    # v2 optional keys: no warning when absent
    def opt(key: str, shape: tuple[int, ...], dtype: Any = float) -> np.ndarray | None:
        if key not in data:
            return None
        a = np.asarray(data[key]).astype(dtype)
        if a.shape != shape:
            if a.size == int(np.prod(shape)):
                return a.reshape(shape)
            raise ValueError(f"{key} has shape {a.shape}, expected {shape}")
        return a

    push_force = opt("push_force", (n, 3))
    push_point = opt("push_point", (n, 3))
    push_active = opt("push_active", (n,), dtype=bool)
    wind_mean = opt("wind_mean", (n, 3))
    # v0.8 optional keys (GPS position hold)
    hold_raw = np.asarray(data["hold_point"], dtype=float) if "hold_point" in data else None
    gps_ok = opt("gps_ok", (n,), dtype=bool)
    gps_sats = opt("gps_sats", (n,))
    gps_hacc = opt("gps_hacc_m", (n,))
    gps_fix = opt("gps_fix", (n,))
    pos_est = None
    if "pos_est" in data:
        pe = np.asarray(data["pos_est"], dtype=float)
        if pe.shape not in ((n, 3), (n, 2)):
            raise ValueError(f"pos_est has shape {pe.shape}, expected ({n}, 3)")
        pos_est = np.full((n, 3), np.nan)
        pos_est[:, :pe.shape[1]] = pe
    tilt_sp = None
    if "tilt_sp_deg" in data:
        ts = np.asarray(data["tilt_sp_deg"], dtype=float)
        if ts.shape == (n, 2):
            ts = np.hypot(ts[:, 0], ts[:, 1])
        elif ts.size == n:
            ts = ts.reshape(n)
        else:
            raise ValueError(f"tilt_sp_deg has shape {ts.shape}, expected ({n},) or ({n}, 2)")
        tilt_sp = ts
    # v0.9 (v6): hardware configuration, tether, power (all optional, no warning when absent)
    hw = hardware_of(meta, data, warnings)

    def vec3(key: str) -> np.ndarray | None:
        a = data[key] if key in data else meta.get(key)
        if a is None:
            return None
        a = np.asarray(a, dtype=float).reshape(-1)
        if a.size != 3 or not np.all(np.isfinite(a)):
            raise ValueError(f"{key} must be 3 finite numbers, got {a!r}")
        return a

    def first(keys: Sequence[str], dtype: Any = float) -> np.ndarray | None:
        for kk in keys:
            if kk in data:
                return opt(kk, (n,), dtype=dtype)
        return None
    t_anchor = vec3("tether_anchor_m")
    t_attach = vec3("tether_attach_body_m")
    if t_attach is not None and str(meta.get("pos_ref", "exit_plane")) == "cg":
        t_attach = t_attach + np.array([0.0, 0.0, float(meta.get("z_cg_m", Z_CG_DEFAULT_M))])
    t_tension = first(("tether_tension_N", "tether_tension_n", "tether_tension"))
    t_conn = first(("tether_connected",), dtype=bool)
    if hw.legacy and any(a is not None for a in (t_anchor, t_attach, t_tension, t_conn)):
        warnings.append("tether keys without hardware_config / power_source: drawn as the VTOL tether configuration")
        hw = Hardware(HW_VTOL, "tether", "inferred")
    power: dict[str, np.ndarray] = {}
    for name, keys, dt_ in () if hw.legacy else (("supply_v", ("supply_v_V", "v_bus_V", "v_esc_V", "v_tether_V", "supply_v"), float),
                            ("supply_i", ("supply_i_A", "i_supply_A", "i_tether_A", "tether_i_A", "supply_i"), float),
                            ("supply_state", ("supply_state",), float),
                            ("buffer_v", ("buffer_v_V", "v_buffer_V", "buffer_v"), float),
                            ("buffer_i", ("buffer_i_A", "i_buffer_A", "buffer_i"), float),
                            ("buffer_soc", ("buffer_soc", "buffer_soc_frac"), float),
                            ("v_cell", ("v_cell",), float)):
        a = first(keys, dt_)
        if a is not None:
            power[name] = a
    if push_force is not None or push_active is not None:
        if push_force is None:
            raise ValueError("push_active without push_force")
        push_force = np.where(np.isfinite(push_force), push_force, 0.0)
        if push_active is None:
            push_active = np.linalg.norm(push_force, axis=1) > PUSH_EPS_N
            warnings.append(f"push_active missing -> |push_force| > {PUSH_EPS_N} N")
        if push_point is None:
            warnings.append("push_point missing -> the body axis at z_cg (meta.z_cg_m, else 0.39 m)")

    # non-finite samples (a sim that blew up): keep the finite prefix and say so
    bad = ~(np.all(np.isfinite(pos), axis=1) & np.all(np.isfinite(q), axis=1))
    if np.any(bad):
        k = int(np.argmax(bad))
        if k < 2:
            raise ValueError("log is non-finite from its first samples")
        warnings.append(f"non-finite pos/q from t = {t[k]:.3f} s: log truncated there")
        sl = slice(0, k)
        t, pos, q, vel, delta = t[sl], pos[sl], q[sl], vel[sl], delta[sl]
        throttle, thrust, state, phase = throttle[sl], thrust[sl], state[sl], phase[sl]
        h_target, h_ref, h_est, wind, gust = h_target[sl], h_ref[sl], h_est[sl], wind[sl], gust[sl]
        push_force, push_point, push_active, wind_mean = (None if a is None else a[sl] for a in
                                                          (push_force, push_point, push_active, wind_mean))
        gps_ok, gps_sats, gps_hacc, gps_fix, pos_est, tilt_sp = (None if a is None else a[sl] for a in
                                                                 (gps_ok, gps_sats, gps_hacc, gps_fix, pos_est, tilt_sp))
        if hold_raw is not None and hold_raw.ndim == 2 and hold_raw.shape[0] == n:
            hold_raw = hold_raw[sl]
        t_tension, t_conn = (None if a is None else a[sl] for a in (t_tension, t_conn))
        power = {kk: a[sl] for kk, a in power.items()}
        n = k
    qn = np.linalg.norm(q, axis=1)
    if np.max(np.abs(qn - 1.0)) > 1e-3:
        warnings.append(f"q not unit (max | |q| - 1 | = {np.max(np.abs(qn - 1.0)):.2e}); normalised")
    q = quat_normalize(q)
    # hemisphere continuity (q and -q are the same attitude; slerp takes the short arc anyway)
    for i in range(1, n):
        if float(np.dot(q[i], q[i - 1])) < 0.0:
            q[i] = -q[i]
    if "tilt_deg" in data:
        tilt = np.asarray(data["tilt_deg"], dtype=float).reshape(-1)[:n]
    else:
        warnings.append("tilt_deg missing -> from q")
        tilt = tilt_deg_of(q)
    pos_ref = str(meta.get("pos_ref", "exit_plane"))
    if pos_ref == "cg":
        if "z_cg_m" not in meta:
            raise ValueError("meta.pos_ref = 'cg' needs meta.z_cg_m")
        pos = pos - quat_to_matrix(q) @ np.array([0.0, 0.0, float(meta["z_cg_m"])])
    elif pos_ref != "exit_plane":
        raise ValueError(f"meta.pos_ref {pos_ref!r} not in ('exit_plane', 'cg')")
    ev_t = np.asarray(data.get("event_t", np.zeros(0)), dtype=float).reshape(-1)
    ev_x = _str_list(data["event_text"]) if "event_text" in data else []
    if len(ev_x) != ev_t.shape[0]:
        warnings.append(f"event_t ({ev_t.shape[0]}) / event_text ({len(ev_x)}) lengths differ; truncated")
        m = min(len(ev_x), ev_t.shape[0])
        ev_t, ev_x = ev_t[:m], ev_x[:m]
    c_t = np.asarray(data.get("cmd_t", np.zeros(0)), dtype=float).reshape(-1)
    c_x = _str_list(data["cmd_name"]) if "cmd_name" in data else []
    if len(c_x) != c_t.shape[0]:
        warnings.append(f"cmd_t ({c_t.shape[0]}) / cmd_name ({len(c_x)}) lengths differ; truncated")
        m = min(len(c_x), c_t.shape[0])
        c_t, c_x = c_t[:m], c_x[:m]
    oe, oc = np.argsort(ev_t, kind="stable"), np.argsort(c_t, kind="stable")
    if push_force is not None and push_point is None:
        zc = float(meta.get("z_cg_m", 0.39))
        push_point = pos + quat_to_matrix(q) @ np.array([0.0, 0.0, zc])
    if push_point is not None:
        bad_p = ~np.all(np.isfinite(push_point), axis=1)
        push_point = np.where(bad_p[:, None], pos, push_point)
    if wind_mean is not None and np.all(np.isfinite(wind_mean)):
        wm, wm_src = wind_mean, "log"
    else:
        if wind_mean is not None:
            warnings.append("wind_mean has non-finite samples -> smoothed wind")
        wm, wm_src = smoothed_wind(t, wind, gust), "smoothed"
    hold_xy = hold_xy_series(hold_raw, phase, n)
    if hold_xy is not None and not np.any(np.isfinite(hold_xy[:, 0])):
        warnings.append("hold_point has no finite sample: no hold-point marker")
    if pos_est is not None and not np.any(np.isfinite(pos_est[:, 0])):
        pos_est = None
    return SimLog(t=t, pos=pos, vel=vel, q=q, delta=delta, throttle=throttle, thrust=thrust, state=state, phase=phase,
                  h_target=h_target, h_ref=h_ref, h_est=h_est, tilt_deg=tilt, wind=wind, gust_active=gust,
                  ground_z=ground_z, event_t=ev_t[oe], event_text=[ev_x[i] for i in oe], cmd_t=c_t[oc],
                  cmd_name=[c_x[i] for i in oc], meta=meta, warnings=warnings, source=os.path.abspath(path), sha256=sha,
                  push_force=push_force, push_point=push_point, push_active=push_active, wind_mean=wm,
                  wind_mean_source=wm_src, hold_xy=hold_xy, gps_ok=gps_ok, gps_sats=gps_sats, gps_hacc=gps_hacc,
                  gps_fix=gps_fix, pos_est=pos_est, tilt_sp_deg=tilt_sp, hardware=None if hw.legacy else hw,
                  tether_anchor=t_anchor, tether_attach_b=t_attach, tether_tension=t_tension, tether_connected=t_conn,
                  power=power)


def smoothed_wind(t: np.ndarray, wind: np.ndarray, gust_active: np.ndarray, sigma_s: float = 1.0) -> np.ndarray:
    """The default ``wind_mean``: gust samples replaced by the last calm sample (the whole wind while no calm sample
    precedes), then a centred Gaussian of ``sigma_s`` (zero-phase; the log is known in advance)."""
    n = t.shape[0]
    w = np.asarray(wind, dtype=float).copy()
    if n == 0:
        return w
    g = np.asarray(gust_active, dtype=bool)
    last_calm = np.maximum.accumulate(np.where(~g, np.arange(n), -1))
    use = g & (last_calm >= 0)
    w[use] = w[last_calm[use]]
    dt = float(np.median(np.diff(t))) if n > 1 else 1.0
    return _smooth(w, sigma_s / max(dt, 1e-9))


@dataclass
class PushInfo:
    """One push (a contiguous run of push_active samples) and the vehicle's response, from logged values only.
    Recovery follows ``sim.sim6dof.gust_recovery``: time after the push END until the true tilt is back below
    ``band_deg`` for the rest of the window (end + PUSH_WINDOW_S, cut at the next push and at the end of POWERED);
    0 = it never left the band after the push ended, None = still outside at the window end."""

    i: int
    t0: float
    t1: float
    f_peak_n: float
    impulse_ns: float
    dir_deg: float                  # horizontal direction of the force (toward), deg from +x toward +y
    point: np.ndarray               # world contact point at the force peak
    baseline_tilt_deg: float | None  # mean true tilt over the second before the push
    peak_tilt_deg: float
    t_peak: float
    recovery_s: float | None
    window_end: float
    moved_m: float                  # largest horizontal displacement from the push-start position in the window

    @property
    def duration_s(self) -> float:
        return self.t1 - self.t0


def find_pushes(log: SimLog, band_deg: float | None = None, window_s: float = PUSH_WINDOW_S) -> list[PushInfo]:
    if log.push_active is None or log.push_force is None or not np.any(log.push_active):
        return []
    if band_deg is None:
        band_deg = float((log.meta.get("criteria") or {}).get("gust_band_deg", 2.0))
    t, a = log.t, log.push_active.astype(bool)
    edges = np.diff(np.concatenate([[0], a.astype(int), [0]]))
    starts, ends = np.flatnonzero(edges == 1), np.flatnonzero(edges == -1)        # [start, end) sample ranges
    powered = np.flatnonzero(log.state == STATE_POWERED)
    t_pw_end = float(t[powered[-1]]) if powered.size else float(t[-1])
    fmag = np.linalg.norm(log.push_force, axis=1)
    dt = np.diff(np.append(t, t[-1]))
    out: list[PushInfo] = []
    for j, (s, e) in enumerate(zip(starts, ends)):
        t0 = float(t[s])
        t1 = float(t[e]) if e < t.size else float(t[-1])
        nxt = float(t[starts[j + 1]]) if j + 1 < starts.size else np.inf
        w_end = min(t1 + window_s, nxt, max(t_pw_end, t1), float(t[-1]))
        seg = slice(s, e)
        k_pk = s + int(np.argmax(fmag[seg]))
        F = log.push_force[k_pk]
        inwin = (t >= t0) & (t <= w_end)
        before = (t >= t0 - 1.0) & (t < t0)
        tl = log.tilt_deg
        k_tp = int(np.flatnonzero(inwin)[np.argmax(tl[inwin])])
        after = (t >= t1) & (t <= w_end)
        ta, aa = t[after], tl[after]
        out_idx = np.flatnonzero(aa > band_deg)
        if ta.size == 0:
            rec = None
        elif out_idx.size == 0:
            rec = 0.0
        elif out_idx[-1] + 1 < ta.size:
            rec = float(ta[out_idx[-1] + 1] - t1)
        else:
            rec = None
        moved = float(np.max(np.hypot(log.pos[inwin, 0] - log.pos[s, 0], log.pos[inwin, 1] - log.pos[s, 1])))
        out.append(PushInfo(i=j + 1, t0=t0, t1=t1, f_peak_n=float(fmag[k_pk]),
                            impulse_ns=float(np.sum(fmag[seg] * dt[seg])),
                            dir_deg=math.degrees(math.atan2(float(F[1]), float(F[0]))) % 360.0,
                            point=np.asarray(log.push_point[k_pk], dtype=float).copy(),
                            baseline_tilt_deg=float(tl[before].mean()) if np.any(before) else None,
                            peak_tilt_deg=float(tl[k_tp]), t_peak=float(t[k_tp]), recovery_s=rec, window_end=float(w_end),
                            moved_m=moved))
    return out


def wind_dir_deg_series(w: np.ndarray) -> np.ndarray:
    """Direction the (mean) wind blows toward, deg from +x toward +y, unwrapped (continuous through 0/360)."""
    w = np.asarray(w, dtype=float)
    return np.degrees(np.unwrap(np.arctan2(w[:, 1], w[:, 0])))


def wind_dir_varies(log: SimLog, min_span_deg: float = WIND_SWING_MIN_DEG, min_mps: float = WIND_SWING_MIN_MPS) -> bool:
    """True when the mean wind direction spans more than ``min_span_deg`` (5th to 95th percentile, so residual
    turbulence in a smoothed default does not count) while the wind is at least ``min_mps``."""
    wm = log.wind_mean if log.wind_mean is not None else log.wind
    sp = np.hypot(wm[:, 0], wm[:, 1])
    sel = sp >= min_mps
    if np.count_nonzero(sel) < 2:
        return False
    d = wind_dir_deg_series(wm[sel])
    lo, hi = np.percentile(d, [5.0, 95.0])
    return bool(float(hi - lo) > min_span_deg)


# --------------------------------------------------------------------------- #
# resampling to video frames
# --------------------------------------------------------------------------- #
def frame_times(t0: float, t1: float, fps: float, speed: float = 1.0) -> np.ndarray:
    """Sim times of the video frames: t0 + k speed / fps for every k with t <= t1 (+1e-9)."""
    if fps <= 0 or speed <= 0:
        raise ValueError("fps and speed must be > 0")
    if t1 < t0:
        raise ValueError("end before start")
    n = int(math.floor((t1 - t0) * fps / speed + 1e-9)) + 1
    return t0 + np.arange(n) * (speed / fps)


def interp_linear(t: np.ndarray, y: np.ndarray, tq: np.ndarray) -> np.ndarray:
    """Linear interpolation per column, clamped at the ends (NaN neighbours give NaN)."""
    y = np.asarray(y, dtype=float)
    if y.ndim == 1:
        return np.interp(tq, t, y)
    return np.stack([np.interp(tq, t, y[:, j]) for j in range(y.shape[1])], axis=1)


def hold_index(t: np.ndarray, tq: np.ndarray) -> np.ndarray:
    """Index of the last log sample at or before each tq (zero-order hold), clipped to the log."""
    return np.clip(np.searchsorted(t, tq, side="right") - 1, 0, t.shape[0] - 1)


def interp_hold(t: np.ndarray, y: np.ndarray, tq: np.ndarray) -> np.ndarray:
    return np.asarray(y)[hold_index(t, tq)]


def slerp_resample(t: np.ndarray, q: np.ndarray, tq: np.ndarray) -> np.ndarray:
    i = np.clip(np.searchsorted(t, tq, side="right") - 1, 0, t.shape[0] - 2)
    s = np.clip((tq - t[i]) / (t[i + 1] - t[i]), 0.0, 1.0)
    return slerp(q[i], q[i + 1], s)


@dataclass
class Frames:
    """The log resampled at the video frame times (arrays with a leading frame axis)."""

    t: np.ndarray
    pos: np.ndarray
    vel: np.ndarray
    q: np.ndarray
    R: np.ndarray
    delta: np.ndarray
    throttle: np.ndarray
    thrust: np.ndarray
    state: np.ndarray
    phase: np.ndarray
    h_target: np.ndarray
    h_ref: np.ndarray
    tilt_deg: np.ndarray
    wind: np.ndarray
    wind_steady: np.ndarray
    gust_active: np.ndarray
    wind_mean: np.ndarray | None = None        # (N, 3)
    push_active: np.ndarray | None = None      # (N,) bool
    push_force: np.ndarray | None = None       # (N, 3)
    push_point: np.ndarray | None = None       # (N, 3)
    # v0.8 (None when the log lacks them)
    cg_xy: np.ndarray | None = None            # (N, 2) horizontal CG (pos + R e_z z_cg)
    hold_xy: np.ndarray | None = None          # (N, 2) zero-order hold, NaN = no hold point
    gps_ok: np.ndarray | None = None           # (N,) zero-order hold
    gps_sats: np.ndarray | None = None
    gps_hacc: np.ndarray | None = None
    gps_fix: np.ndarray | None = None
    pos_est: np.ndarray | None = None          # (N, 3) linear
    tilt_sp_deg: np.ndarray | None = None      # (N,) linear
    est_rel: np.ndarray | None = None          # (N, 2) SimLog.est_rel_xy, linear (NaN while no hold point)
    # v0.9 (v6; None / {} when the log lacks them)
    tether_tension: np.ndarray | None = None   # (N,) linear
    tether_connected: np.ndarray | None = None  # (N,) zero-order hold
    power: dict[str, np.ndarray] = field(default_factory=dict)   # supply_state zero-order hold, the rest linear

    @property
    def n(self) -> int:
        return int(self.t.shape[0])

    def hold_dist(self) -> np.ndarray | None:
        """(N,) |CG_xy - hold_xy| per frame (NaN while no hold point); None without a hold point."""
        if self.hold_xy is None or self.cg_xy is None:
            return None
        return np.hypot(*(self.cg_xy - self.hold_xy).T)


def push_window_resample(t: np.ndarray, active: np.ndarray, force: np.ndarray, point: np.ndarray, tq: np.ndarray,
                         window: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Push state of each frame: active when any log sample within +-window/2 of the frame time is active (a 0.1 s
    push is not skipped at --speed 4); the force and point are those of the strongest active sample in that window,
    else the zero-order-held sample."""
    tq = np.asarray(tq, dtype=float)
    a = np.asarray(active, dtype=bool)
    cs = np.concatenate([[0], np.cumsum(a)])
    half = 0.5 * max(float(window), 0.0)
    lo = np.searchsorted(t, tq - half - 1e-12, side="left")
    hi = np.searchsorted(t, tq + half + 1e-12, side="right")
    hold = hold_index(t, tq)
    lo = np.minimum(lo, hold)
    hi = np.maximum(hi, hold + 1)
    act = (cs[hi] - cs[lo]) > 0
    fq = force[hold].copy()
    pq = point[hold].copy()
    fmag = np.linalg.norm(force, axis=1) * a
    for k in np.flatnonzero(act):
        j = lo[k] + int(np.argmax(fmag[lo[k]:hi[k]]))
        fq[k], pq[k] = force[j], point[j]
    fq[~act] = 0.0
    return act, fq, pq


def resample(log: SimLog, tq: np.ndarray, window: float | None = None) -> Frames:
    """Continuous signals: linear (q: slerp); discrete (state, phase, gust flag, h_target - a step
    command): zero-order hold.  ``wind_steady`` = the wind of the last sample without an active gust
    (so the gust component = wind - wind_steady); the whole wind while no gust-free sample precedes.
    Push keys: ``push_window_resample`` over ``window`` (default: the median frame spacing)."""
    t = log.t
    tq = np.asarray(tq, dtype=float)
    q = slerp_resample(t, log.q, tq)
    hi = hold_index(t, tq)
    no_gust = ~log.gust_active
    last_calm = np.where(no_gust, np.arange(log.n), -1)
    last_calm = np.maximum.accumulate(last_calm)
    calm_i = last_calm[hi]
    wind = interp_linear(t, log.wind, tq)
    steady = np.where((calm_i >= 0)[:, None] & log.gust_active[hi][:, None], log.wind[np.maximum(calm_i, 0)], wind)
    pa = pf = pp = None
    if log.push_active is not None and log.push_force is not None and log.push_point is not None:
        win = window if window is not None else (float(np.median(np.diff(tq))) if tq.size > 1 else 0.0)
        pa, pf, pp = push_window_resample(t, log.push_active, log.push_force, log.push_point, tq, win)
    pos = interp_linear(t, log.pos, tq)
    R = quat_to_matrix(q)

    def held(a: np.ndarray | None) -> np.ndarray | None:
        return None if a is None else np.asarray(a)[hi]
    return Frames(
        t=tq, pos=pos, vel=interp_linear(t, log.vel, tq), q=q,
        R=R, delta=interp_linear(t, log.delta, tq), throttle=interp_linear(t, log.throttle, tq),
        thrust=interp_linear(t, log.thrust, tq), state=log.state[hi], phase=log.phase[hi],
        h_target=log.h_target[hi], h_ref=interp_linear(t, log.h_ref, tq), tilt_deg=interp_linear(t, log.tilt_deg, tq),
        wind=wind, wind_steady=steady, gust_active=log.gust_active[hi],
        wind_mean=interp_linear(t, log.wind_mean, tq) if log.wind_mean is not None else None,
        push_active=pa, push_force=pf, push_point=pp,
        cg_xy=cg_xy_of(pos, R, log.z_cg_m), hold_xy=held(log.hold_xy), gps_ok=held(log.gps_ok),
        gps_sats=held(log.gps_sats), gps_hacc=held(log.gps_hacc), gps_fix=held(log.gps_fix),
        pos_est=interp_linear(t, log.pos_est, tq) if log.pos_est is not None else None,
        tilt_sp_deg=interp_linear(t, log.tilt_sp_deg, tq) if log.tilt_sp_deg is not None else None,
        est_rel=interp_linear(t, log.est_rel_xy(), tq) if log.est_rel_xy() is not None else None,
        tether_tension=interp_linear(t, log.tether_tension, tq) if log.tether_tension is not None else None,
        tether_connected=held(log.tether_connected),
        power={k: (np.asarray(a)[hi] if k == "supply_state" else interp_linear(t, a, tq)) for k, a in log.power.items()})


def last_command(cmd_t: np.ndarray, cmd_name: Sequence[str], t: float) -> tuple[str, float] | None:
    """(name, time) of the last ground command sent at or before t."""
    if len(cmd_name) == 0:
        return None
    i = int(np.searchsorted(cmd_t, t + 1e-9, side="right")) - 1
    return None if i < 0 else (cmd_name[i], float(cmd_t[i]))


def recent_events(event_t: np.ndarray, event_text: Sequence[str], t: float, max_n: int = 6) -> list[tuple[float, str]]:
    """The last ``max_n`` events at or before t (oldest first)."""
    i = int(np.searchsorted(event_t, t + 1e-9, side="right"))
    return [(float(event_t[j]), event_text[j]) for j in range(max(0, i - max_n), i)]


# --------------------------------------------------------------------------- #
# HUD text (pure)
# --------------------------------------------------------------------------- #
def _fmt_num(v: float, fmt: str, none: str = "--") -> str:
    return none if v is None or not np.isfinite(v) else format(v, fmt)


def wind_heading_deg(w: np.ndarray) -> float:
    """Direction the air moves toward, from +x toward +y (``sim6dof.Scenario.wind_dir_deg``)."""
    return math.degrees(math.atan2(float(w[1]), float(w[0]))) % 360.0


def gps_text(gps: dict[str, Any], lang: str = "en") -> tuple[str, str]:
    """(text, tone) of the GPS status: fix type (else usable / not usable), satellites, hAcc; 'alert' when not usable.
    ``gps``: ok (bool), sats (int | None), hacc (m | None), fix (int | None)."""
    ok = bool(gps.get("ok", False))
    fix = gps.get("fix")
    parts: list[str] = []
    if fix is not None and np.isfinite(float(fix)):
        f = int(fix)
        parts.append(tr("gps_fix3" if f >= 3 else ("gps_fix2" if f == 2 else "gps_nofix"), lang))
        if not ok and f >= 2:
            parts.append(tr("gps_unusable", lang))
    else:
        parts.append(tr("gps_usable" if ok else "gps_unusable", lang))
    sats = gps.get("sats")
    if sats is not None and np.isfinite(float(sats)):
        parts.append(tr("gps_sats", lang, n=int(sats)))
    hacc = gps.get("hacc")
    if ok and hacc is not None and np.isfinite(float(hacc)):
        parts.append(tr("gps_hacc", lang, a=float(hacc)))
    if not ok:
        parts.append(tr("gps_lost_hint", lang))
    return " · ".join(parts), ("ok" if ok else "alert")


def gps_hud_rows(s: dict[str, Any], lang: str = "en") -> list[tuple[str, str, str]]:
    """The v0.8 HUD rows: distance from the hold point (``hold`` key: False = not set yet, a number = metres, absent =
    no row; amber above ``hold_lim``) and the GPS status (``gps`` key, absent = no row)."""
    rows: list[tuple[str, str, str]] = []
    if "hold" in s:
        d = s["hold"]
        if d is None or d is False or not np.isfinite(float(d)):
            rows.append((tr("HOLD", lang), tr("hold_unset", lang), "muted"))
        else:
            lim = s.get("hold_lim")
            rows.append((tr("HOLD", lang), tr("hold_val", lang, d=float(d)),
                         "warn" if (lim is not None and float(d) > float(lim)) else "accent"))
    if s.get("gps") is not None:
        txt, tone = gps_text(s["gps"], lang)
        rows.append((tr("GPS", lang), txt, tone))
    return rows


def frame_gps_sample(fr: "Frames", k: int, log: "SimLog") -> dict[str, Any]:
    """The v0.8 HUD keys of frame k (none when the log has no GPS / hold keys): ``hold`` and ``hold_lim``, ``gps``."""
    out: dict[str, Any] = {}
    if fr.hold_xy is not None and log.has_hold:
        hd = fr.hold_dist()
        out["hold"] = float(hd[k]) if np.isfinite(hd[k]) else False
        out["hold_lim"] = log.station_keeping_limit()
    if fr.gps_ok is not None or fr.gps_sats is not None or fr.gps_fix is not None:
        def v(a: np.ndarray | None) -> Any:
            return None if a is None else a[k]
        ok = bool(fr.gps_ok[k]) if fr.gps_ok is not None else bool(fr.gps_fix is not None and fr.gps_fix[k] >= 3)
        out["gps"] = {"ok": ok, "sats": v(fr.gps_sats), "hacc": v(fr.gps_hacc), "fix": v(fr.gps_fix)}
    return out


def frame_power_sample(fr: "Frames", k: int, log: "SimLog") -> dict[str, Any]:
    """The v0.9 HUD keys of frame k (none for a legacy log): ``tether`` {connected, tension}, ``supply`` {v, i, state},
    ``buffer`` {v, i, soc}, ``batt`` {v_cell} - each only when the log carries its keys (the battery row only for a
    battery-powered vehicle)."""
    out: dict[str, Any] = {}
    hw = log.hw
    if hw.legacy:
        return out
    p = fr.power

    def g(name: str) -> float | None:
        a = p.get(name)
        return None if a is None or not np.isfinite(a[k]) else float(a[k])
    if fr.tether_connected is not None or fr.tether_tension is not None or hw.tethered:
        out["tether"] = {"connected": None if fr.tether_connected is None else bool(fr.tether_connected[k]),
                         "tension": None if fr.tether_tension is None or not np.isfinite(fr.tether_tension[k])
                         else float(fr.tether_tension[k])}
    if any(x in p for x in ("supply_v", "supply_i", "supply_state")):
        out["supply"] = {"v": g("supply_v"), "i": g("supply_i"), "state": g("supply_state")}
    if any(x in p for x in ("buffer_v", "buffer_i", "buffer_soc")):
        out["buffer"] = {"v": g("buffer_v"), "i": g("buffer_i"), "soc": g("buffer_soc")}
    if "v_cell" in p and hw.power == "battery":
        out["batt"] = {"v_cell": g("v_cell")}
    return out


def power_hud_rows(s: dict[str, Any], lang: str = "en") -> list[tuple[str, str, str]]:
    """The v0.9 HUD rows (``frame_power_sample`` keys; absent key = no row): tether (tension, connected; alert when
    disconnected), supply (V, A, state 1 = current limit warn / 2 = off alert), buffer (V, V per cell of the 6S buffer,
    A: > 0.5 A discharging (warn), < -0.5 A charging (alert: the ideal diode must block that), SOC), battery (V/cell)."""
    rows: list[tuple[str, str, str]] = []
    te = s.get("tether")
    if te is not None:
        parts = []
        if te.get("tension") is not None:
            parts.append(tr("teth_tension", lang, f=float(te["tension"])))
        con = te.get("connected")
        if con is not None:
            parts.append(tr("teth_conn" if con else "teth_off", lang))
        rows.append((tr("TETHER", lang), " · ".join(parts) if parts else "--", "alert" if con is False else "accent"))
    su = s.get("supply")
    if su is not None:
        parts = [f"{su['v']:5.1f} V" if su.get("v") is not None else "-- V",
                 f"{su['i']:5.1f} A" if su.get("i") is not None else "-- A"]
        tone = "normal"
        st = su.get("state")
        if st is not None and int(round(st)) == 1:
            parts.append(tr("sup_cl", lang))
            tone = "warn"
        elif st is not None and int(round(st)) >= 2:
            parts.append(tr("sup_off", lang))
            tone = "alert"
        rows.append((tr("SUPPLY", lang), "  ".join(parts), tone))
    bu = s.get("buffer")
    if bu is not None:
        parts = []
        if bu.get("v") is not None:
            parts.append(f"{bu['v']:5.1f} V ({tr('per_cell', lang, v=bu['v'] / BUFFER_CELLS)})")
        tone = "normal"
        i = bu.get("i")
        if i is not None:
            if i > 0.5:
                parts.append(f"{i:4.1f} A {tr('buf_dis', lang)}")
                tone = "warn"
            elif i < -0.5:
                parts.append(f"{-i:4.1f} A {tr('buf_chg', lang)}")
                tone = "alert"
            else:
                parts.append(tr("buf_idle", lang))
        if bu.get("soc") is not None:
            parts.append(f"SOC {100.0 * bu['soc']:3.0f} %")
        rows.append((tr("BUFFER", lang), "  ".join(parts) if parts else "--", tone))
    ba = s.get("batt")
    if ba is not None and ba.get("v_cell") is not None:
        rows.append((tr("BATT", lang), tr("per_cell", lang, v=ba["v_cell"]), "normal"))
    return rows


def hud_lines(s: dict[str, Any], lang: str = "en") -> list[tuple[str, str, str]]:
    """HUD rows (label, value, tone) for one frame.  Keys of ``s``: t, speed, mode, phase (name or None),
    state (name), state_code (optional, else from the English name), alt, h_target, h_ref, vz, tilt, throttle,
    u_max, wind (3,), gust (3,), gust_active, last_cmd ((name, t) or None), push (force (3,) or None); v0.8 (optional,
    ``gps_hud_rows``): hold (m, or False = not set yet), hold_lim, gps ({ok, sats, hacc, fix})."""
    ko = lang == "ko"
    deg = "°" if ko else " deg"
    rows: list[tuple[str, str, str]] = []
    sp = float(s.get("speed", 1.0))
    rows.append((tr("TIME", lang), f"{s['t']:7.2f} s" + ("" if abs(sp - 1.0) < 1e-9 else f"   (x{sp:g})"), "normal"))
    mode = str(s.get("mode", "?"))
    mode_txt = tr(f"mode_{mode}", lang) if f"mode_{mode}" in TXT else mode.upper()
    rows.append((tr("MODE", lang), mode_txt, "accent"))
    if s.get("phase") is not None:
        rows.append((tr("PHASE", lang), str(s["phase"]), "accent"))
    st = str(s["state"])
    code = s.get("state_code")
    is_abort = (int(code) == 5) if code is not None else st == "ABORT"
    is_pw = (int(code) == STATE_POWERED) if code is not None else st == "POWERED"
    rows.append((tr("STATE", lang), st, "alert" if is_abort else ("ok" if is_pw else "normal")))
    tgt = s.get("h_target", float("nan"))
    ref = s.get("h_ref", float("nan"))
    alt_txt = f"{s['alt']:6.2f} m"
    if np.isfinite(tgt) or np.isfinite(ref):
        alt_txt += f"   {tr('target', lang)} {_fmt_num(tgt, '.1f')}   {tr('ref', lang)} {_fmt_num(ref, '.2f')}"
    rows.append((tr("ALT", lang), alt_txt, "normal"))
    rows.append((tr("V_Z", lang), f"{s['vz']:+6.2f} m/s", "normal"))
    tilt = float(s["tilt"])
    rows.append((tr("TILT", lang), f"{tilt:6.1f}{deg}", "alert" if tilt >= 30.0 else ("warn" if tilt >= 10.0 else "normal")))
    u, um = float(s["throttle"]), float(s.get("u_max", 1.0))
    sat = u >= um - 1e-3 and u > 0.0
    rows.append((tr("THR", lang), f"{u:6.3f}" + (f"   {tr('saturated', lang)}" if sat else ""), "warn" if sat else "normal"))
    rows += gps_hud_rows(s, lang)
    rows += power_hud_rows(s, lang)
    w = np.asarray(s["wind"], dtype=float)
    wmag = float(np.hypot(w[0], w[1]))
    if wmag <= 0.05:
        wtxt = tr("calm", lang)
    elif ko:
        wtxt = f"{wmag:4.1f} m/s  → {wind_heading_deg(w):3.0f}°"
    else:
        wtxt = f"{wmag:4.1f} m/s toward {wind_heading_deg(w):3.0f} deg"
    rows.append((tr("WIND", lang), wtxt, "normal"))
    if s.get("gust_active"):
        g = np.asarray(s.get("gust", np.zeros(3)), dtype=float)
        rows.append((tr("GUST", lang), f"{tr('active', lang)}  +{float(np.hypot(g[0], g[1])):.1f} m/s", "alert"))
    pf = s.get("push")
    if pf is not None:
        pf = np.asarray(pf, dtype=float)
        fm = float(np.linalg.norm(pf))
        dtxt = f"→ {wind_heading_deg(pf):3.0f}°" if ko else f"toward {wind_heading_deg(pf):3.0f} deg"
        rows.append((tr("PUSH", lang), f"{fm:5.1f} N  {dtxt}", "warn"))
    lc = s.get("last_cmd")
    if lc is None:
        rows.append((tr("CMD", lang), tr("none_yet", lang), "muted"))
    elif ko:
        rows.append((tr("CMD", lang), f"{cmd_label(lc[0], lang)}   ({s['t'] - lc[1]:.1f} s 전)", "accent"))
    else:
        rows.append((tr("CMD", lang), f"{lc[0]}   ({s['t'] - lc[1]:.1f} s ago)", "accent"))
    return rows


# --------------------------------------------------------------------------- #
# camera planning (pure)
# --------------------------------------------------------------------------- #
@dataclass
class CamView:
    position: np.ndarray
    focal: np.ndarray
    up: np.ndarray
    view_angle_deg: float


def camera_basis(azimuth_deg: float, elevation_deg: float) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Look direction f (camera -> focal point), right r, up u for a camera looking along azimuth
    (from +x toward +y) and pitched down by elevation."""
    a, e = math.radians(azimuth_deg), math.radians(elevation_deg)
    f = np.array([math.cos(e) * math.cos(a), math.cos(e) * math.sin(a), -math.sin(e)])
    r = np.cross(f, [0.0, 0.0, 1.0])
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    return f, r, u


def project(points: np.ndarray, cam: CamView, width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
    """Perspective projection of world points to pixel (x right, y down) as VTK does with a vertical
    view angle; returns (px [..., 2], depth along the view axis)."""
    f = cam.focal - cam.position
    f = f / np.linalg.norm(f)
    r = np.cross(f, cam.up)
    r /= np.linalg.norm(r)
    u = np.cross(r, f)
    d = np.asarray(points, dtype=float) - cam.position
    xc, yc, zc = d @ r, d @ u, d @ f
    th = math.tan(math.radians(cam.view_angle_deg) / 2.0)
    aspect = width / height
    zc_safe = np.where(np.abs(zc) < 1e-9, 1e-9, zc)
    nx = xc / (zc_safe * th * aspect)
    ny = yc / (zc_safe * th)
    px = np.stack([(nx + 1.0) * 0.5 * width, (1.0 - ny) * 0.5 * height], axis=-1)
    return px, zc


def vehicle_points(fr: Frames, length_m: float) -> np.ndarray:
    """(N, 3, 3): exit plane centre, mid body, nose tip of every frame (world)."""
    body = np.outer([0.0, 0.5 * length_m, length_m], [0.0, 0.0, 1.0])     # (3 points, xyz) in the body frame
    return fr.pos[:, None, :] + np.einsum("nij,kj->nki", fr.R, body)


def _running_max(x: np.ndarray, w: int) -> np.ndarray:
    if w <= 1:
        return x.copy()
    from scipy.ndimage import maximum_filter1d
    return maximum_filter1d(x, size=w, mode="nearest")


def _smooth(x: np.ndarray, sigma: float) -> np.ndarray:
    """Zero-phase Gaussian smoothing along axis 0; the ends are extended by point reflection about the
    end samples, so a linear trend passes unchanged (no lag or bias at the first / last frames)."""
    if sigma <= 0.0 or x.shape[0] < 3:
        return x.copy()
    from scipy.ndimage import gaussian_filter1d
    x = np.asarray(x, dtype=float)
    m = min(x.shape[0] - 1, int(math.ceil(4.0 * sigma)))
    left = 2.0 * x[0] - x[m:0:-1]
    right = 2.0 * x[-1] - x[-2:-m - 2:-1]
    xp = np.concatenate([left, x, right], axis=0)
    return gaussian_filter1d(xp, sigma=sigma, axis=0, mode="nearest")[m:m + x.shape[0]]


@dataclass
class CameraPlan:
    focal: np.ndarray       # (N, 3)
    dist: np.ndarray        # (N,)
    azimuth_deg: np.ndarray  # (N,) deg (unwrapped; constant unless the camera orbits to face each push)
    elevation_deg: float
    view_angle_deg: float
    framing: np.ndarray     # (N,) 3 = + person, 2 = vehicle + ground + pad, 1 = vehicle + ground, 0 = vehicle only

    def view(self, k: int) -> CamView:
        f, _, u = camera_basis(float(self.azimuth_deg[k]), self.elevation_deg)
        foc = self.focal[k]
        return CamView(foc - self.dist[k] * f, foc.copy(), u, self.view_angle_deg)


def auto_azimuth(log: SimLog) -> float:
    """Look direction (deg) perpendicular to the horizontal drift (else the mean wind), so the drift
    crosses the screen left -> right."""
    d = log.pos[-1, :2] - log.pos[0, :2]
    if float(np.hypot(*d)) < 0.5:
        d = np.mean(log.wind[:, :2], axis=0)
    if float(np.hypot(*d)) < 1e-6:
        d = np.array([1.0, 0.0])
    return (math.degrees(math.atan2(d[1], d[0])) + 90.0) % 360.0


def ang_diff_deg(a: float, b: float) -> float:
    """a - b wrapped to [-180, 180)."""
    return (float(a) - float(b) + 180.0) % 360.0 - 180.0


def push_azimuths(dirs_deg: Sequence[float]) -> list[float]:
    """A camera azimuth per push that sees the push from the side (the pole and arrow run across the screen,
    the person stands beside the vehicle, not in front of it): dir + 90 deg for the first (push left -> right),
    then whichever of dir +- 90 deg needs the smaller turn from the previous one."""
    out: list[float] = []
    cur = None
    for d in dirs_deg:
        cands = [float(d) + 90.0, float(d) - 90.0]
        a = cands[0] if cur is None else min(cands, key=lambda c: abs(ang_diff_deg(c, cur)))
        a = a if cur is None else cur + ang_diff_deg(a, cur)            # unwrapped, continuous
        out.append(a)
        cur = a
    return out


def azimuth_track(t: np.ndarray, pushes: Sequence[tuple[float, float, float]], lead_s: float = 0.4,
                  turn_s: float = 1.6) -> np.ndarray:
    """Pure.  Camera azimuth per frame (deg, unwrapped) for logs with pushes (t0, t1, direction deg): held at
    ``push_azimuths`` of each push from ``lead_s`` before it starts, turning (smoothstep) during up to ``turn_s``
    between the end of one push and the lead time of the next - a camera operator walking round to film each
    push from the side."""
    t = np.asarray(t, dtype=float)
    if not pushes:
        raise ValueError("azimuth_track needs at least one push")
    azs = push_azimuths([p[2] for p in pushes])
    out = np.full(t.shape, azs[0])
    for i in range(1, len(pushes)):
        tb = pushes[i][0] - lead_s
        ta = max(pushes[i - 1][1] + 0.3, tb - turn_s)
        if ta >= tb:
            ta = tb - 0.3
        s = np.clip((t - ta) / (tb - ta), 0.0, 1.0)
        s = s * s * (3.0 - 2.0 * s)
        out = np.where(t >= ta, azs[i - 1] + s * (azs[i] - azs[i - 1]), out)
    return out


NDC = tuple[float, float, float, float]          # x0, x1, y0, y1 in normalised device coordinates (y up)
FRAME_BOX: NDC = (-0.82, 0.82, -0.58, 0.66)       # where the framed set goes: between the banner and the ticker


def _ndc(points: np.ndarray, cam: CamView, aspect: float) -> tuple[np.ndarray, np.ndarray]:
    px, z = project(points, cam, 1000.0 * aspect, 1000.0)
    return np.column_stack([px[:, 0] / (1000.0 * aspect) * 2.0 - 1.0, 1.0 - px[:, 1] / 1000.0 * 2.0]), z


FRAMINGS = ("all", "ground", "vehicle", "demo")
# demo: a close, calm camera for a 1-2 m hover (a tripod-like view of the vehicle, the ground, the pad and the person)
DEMO_CAMERA = {"elevation_deg": 8.0, "d_min": 2.5, "d_max": 14.0, "margin": 1.12, "smooth_s": 1.5, "follow_sigma_s": 1.2}


def plan_camera(fr: Frames, ground_z: float, fps: float, aspect: float, length_m: float, framing: str = "all",
                azimuth_deg: float | np.ndarray = 90.0, elevation_deg: float = 12.0, view_angle_deg: float = 30.0,
                d_min: float = 4.0, d_max: float = 70.0, box: NDC = FRAME_BOX, avoid: Sequence[NDC] = (),
                margin: float = 1.08, smooth_s: float = 0.6, extra_pts: np.ndarray | None = None,
                follow_sigma_s: float = 0.12) -> CameraPlan:
    """Smoothed chase camera (fixed azimuth / elevation; focal point + distance per frame).

    Per frame it frames the largest feasible set of {vehicle, ground below it, pad, ``extra_pts`` (the person)}
    inside ``box`` (NDC, above the inset band of the layout): the distance that fits a set (``margin`` x its
    projected extent) must not exceed ``d_max`` (beyond that the vehicle shrinks below ~25 px on 1080 lines),
    else the person, the pad and then the ground drop out.  The raw focal points and distances are smoothed
    zero-phase (the log is known in advance; a running maximum before the smoothing keeps the distance from
    undershooting), then every frame is checked: where a vehicle point leaves the central 92 % of the view or
    falls into an ``avoid`` rectangle (the insets, the HUD), that frame's distance grows and its focal point
    moves toward centring the vehicle in ``box``, and the smoothing is redone.  ``follow_sigma_s`` smooths the
    vehicle path the focal point follows (0.12 s: a chase camera; ``demo`` 1.2 s: a calm, tripod-like view)."""
    if framing not in FRAMINGS:
        raise ValueError("framing must be " + " | ".join(FRAMINGS))
    az = np.broadcast_to(np.asarray(azimuth_deg, dtype=float), (fr.n,)).copy()
    bases = [camera_basis(float(a), elevation_deg) for a in az]
    F_ = np.array([b[0] for b in bases]).reshape(-1, 3)
    R_ = np.array([b[1] for b in bases]).reshape(-1, 3)
    U_ = np.array([b[2] for b in bases]).reshape(-1, 3)
    th_v = math.tan(math.radians(view_angle_deg) / 2.0)
    th_h = th_v * aspect
    bx0, bx1, by0, by1 = box
    bcx, bcy = 0.5 * (bx0 + bx1), 0.5 * (by0 + by1)
    vp = vehicle_points(fr, length_m)                               # (N, 3, 3)
    n = fr.n
    pad = np.array([[0.0, 0.0, 0.0], [0.0, 0.0, ground_z]])
    foc = np.zeros((n, 3))
    dist = np.zeros(n)
    level = np.zeros(n, dtype=int)
    allowed = {"all": 2, "ground": 1, "vehicle": 0, "demo": 2}[framing]
    if extra_pts is not None and framing in ("all", "demo"):
        allowed = 3

    def aim(c: np.ndarray, d: float, k: int) -> np.ndarray:
        """Focal point that puts the world point c at the box centre at distance d (frame k's camera axes)."""
        return c - R_[k] * (bcx * d * th_h) - U_[k] * (bcy * d * th_v)

    for k in range(n):
        f, r, u = F_[k], R_[k], U_[k]
        veh = vp[k]
        below = np.array([[veh[0, 0], veh[0, 1], ground_z]])
        sets = [veh, np.vstack([veh, below]), np.vstack([veh, below, pad])]
        if allowed == 3:
            sets.append(np.vstack([veh, below, pad, extra_pts[k]]))
        for lv in range(allowed, -1, -1):
            P = sets[lv]
            pr, pu, pf = P @ r, P @ u, P @ f
            w = float(pr.max() - pr.min())
            h = float(pu.max() - pu.min())
            d = max(h / ((by1 - by0) * th_v), w / ((bx1 - bx0) * th_h)) * margin + 0.5 * float(pf.max() - pf.min())
            if d <= d_max or lv == 0:
                d = max(d, d_min)
                c = (0.5 * (pr.max() + pr.min())) * r + (0.5 * (pu.max() + pu.min())) * u + float(pf.mean()) * f
                foc[k], dist[k], level[k] = aim(c, d, k), d, lv
                break
    sig = smooth_s * fps
    win = max(1, int(round(1.0 * fps)))
    # the focal point = the vehicle path (lightly smoothed: it is smooth physics, an apogee arc must not
    # leave the frame) + the framing offset from it (heavily smoothed: set changes, pad entering/leaving)
    ref = fr.pos + np.array([0.0, 0.0, 0.5 * length_m])
    sig_ref = follow_sigma_s * fps

    def focal_of(raw: np.ndarray) -> np.ndarray:
        return _smooth(ref, sig_ref) + _smooth(raw - ref, sig)

    plan = CameraPlan(focal_of(foc), _smooth(_running_max(dist, win), sig), az, elevation_deg, view_angle_deg, level)
    for _ in range(10):
        grow = np.ones(n)
        for k in range(n):
            q, z = _ndc(vp[k], plan.view(k), aspect)
            if float(z.min()) <= 0.0:
                grow[k] = 1.5
                continue
            worst = float(np.max(np.abs(q)))
            hit = any(np.any((q[:, 0] > a0) & (q[:, 0] < a1) & (q[:, 1] > c0) & (q[:, 1] < c1)) for a0, a1, c0, c1 in avoid)
            if worst > 0.92 or hit:
                grow[k] = max(1.12, worst / 0.85)
        if np.all(grow == 1.0):
            break
        bad = grow > 1.0
        dist = np.where(bad, dist * grow, dist)
        recentre = np.array([aim(vp[k, 1], dist[k], k) for k in range(n)])
        foc = np.where(bad[:, None], 0.5 * foc + 0.5 * recentre, foc)
        plan.dist = _smooth(_running_max(dist, win), sig)
        plan.focal = focal_of(foc)
    return plan


def interp_plan(plan: "CameraPlan | FreeCamPlan", t_grid: np.ndarray, tq: np.ndarray) -> "CameraPlan | FreeCamPlan":
    """The plan of the video grid evaluated at other times (preview frames match the video's)."""
    if isinstance(plan, FreeCamPlan):
        return FreeCamPlan(interp_linear(t_grid, plan.position, tq), interp_linear(t_grid, plan.focal, tq),
                           interp_linear(t_grid, plan.view_angle, tq), interp_linear(t_grid, plan.azimuth_deg, tq),
                           interp_linear(t_grid, plan.dist, tq), interp_hold(t_grid, plan.framing, tq))
    return CameraPlan(interp_linear(t_grid, plan.focal, tq), interp_linear(t_grid, plan.dist, tq),
                      interp_linear(t_grid, plan.azimuth_deg, tq), plan.elevation_deg, plan.view_angle_deg, interp_hold(t_grid, plan.framing, tq))


@dataclass
class FreeCamPlan:
    """Per-frame camera position, focal point and vertical view angle (the spectator camera)."""

    position: np.ndarray     # (N, 3)
    focal: np.ndarray        # (N, 3)
    view_angle: np.ndarray   # (N,) deg
    azimuth_deg: np.ndarray  # (N,) look direction, deg from +x toward +y
    dist: np.ndarray         # (N,) camera -> focal point
    framing: np.ndarray      # (N,)

    @property
    def elevation_deg(self) -> float:
        d = self.focal - self.position
        return float(np.degrees(np.arctan2(-d[:, 2], np.hypot(d[:, 0], d[:, 1]))).mean()) if d.size else 0.0

    def view(self, k: int) -> CamView:
        return CamView(self.position[k].copy(), self.focal[k].copy(), np.array([0.0, 0.0, 1.0]), float(self.view_angle[k]))


SPECTATOR = {"eye_h_m": 1.6, "fov_max_deg": 45.0, "fov_min_deg": 5.0, "frame_frac": 0.15, "follow_sigma_s": 0.35,
             "zoom_sigma_s": 1.0, "shake_deg": (0.16, 0.10)}


def plan_spectator(fr: Frames, ground_z: float, fps: float, length_m: float, azimuth_deg: float,
                   standoff_m: float | None = None, eye_h_m: float = 1.6, fov_max_deg: float = 45.0,
                   fov_min_deg: float = 5.0, frame_frac: float = 0.15, follow_sigma_s: float = 0.35,
                   zoom_sigma_s: float = 1.0, shake_deg: tuple[float, float] = (0.16, 0.10),
                   lateral_m: float = 0.0) -> FreeCamPlan:
    """Pure.  A person filming from eye height ``eye_h_m`` above the ground, ``standoff_m`` (default
    clamp(h_max + 3, 5, 14) m) from the pad against the look azimuth: pans / tilts to follow the vehicle's centre
    (Gaussian-smoothed, ``follow_sigma_s``) and zooms so the vehicle fills about ``frame_frac`` of the frame height
    (vertical view angle in [fov_min, fov_max], smoothed ``zoom_sigma_s``), with a slight deterministic handheld
    shake (sum of three sines, amplitude ``shake_deg`` (yaw, pitch), at most 3 % of the view angle)."""
    n = fr.n
    h_max = float(np.max(fr.pos[:, 2] - ground_z)) if n else 1.0
    so = float(standoff_m) if standoff_m is not None else float(np.clip(h_max + 3.0, 5.0, 14.0))
    a = math.radians(azimuth_deg)
    fh = np.array([math.cos(a), math.sin(a), 0.0])
    rh = np.array([math.sin(a), -math.cos(a), 0.0])
    cam = -so * fh + lateral_m * rh
    cam[2] = ground_z + eye_h_m
    P = np.tile(cam, (n, 1))
    centre = fr.pos + np.einsum("nij,j->ni", fr.R, np.array([0.0, 0.0, 0.5 * length_m]))
    tgt = _smooth(centre, follow_sigma_s * fps)
    d = np.linalg.norm(tgt - P, axis=1)
    fov = np.degrees(2.0 * np.arctan(length_m / (2.0 * frame_frac * np.maximum(d, 1e-3))))
    fov = np.clip(_smooth(np.clip(fov, fov_min_deg, fov_max_deg), zoom_sigma_s * fps), fov_min_deg, fov_max_deg)
    t = fr.t
    s1 = np.sin(2 * math.pi * 0.37 * t + 1.1) + 0.6 * np.sin(2 * math.pi * 0.83 * t + 0.4) + 0.3 * np.sin(2 * math.pi * 1.9 * t + 2.0)
    s2 = np.sin(2 * math.pi * 0.29 * t + 2.3) + 0.6 * np.sin(2 * math.pi * 0.71 * t + 1.7) + 0.3 * np.sin(2 * math.pi * 2.3 * t + 0.2)
    yaw_j = np.minimum(shake_deg[0], 0.03 * fov) * s1 / 1.9
    pit_j = np.minimum(shake_deg[1], 0.03 * fov) * s2 / 1.9
    look = tgt - P
    az = np.arctan2(look[:, 1], look[:, 0]) + np.radians(yaw_j)
    el = np.arctan2(look[:, 2], np.hypot(look[:, 0], look[:, 1])) + np.radians(pit_j)
    dirs = np.column_stack([np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)])
    foc = P + dirs * d[:, None]
    return FreeCamPlan(P, foc, fov, np.degrees(np.unwrap(az)), d, np.full(n, 3, dtype=int))


# --------------------------------------------------------------------------- #
# fonts / small drawing helpers (PIL)
# --------------------------------------------------------------------------- #
_FONT_CACHE: dict[tuple[str, int], Any] = {}


def _font_path(family: str, text: str = "") -> str:
    from matplotlib import font_manager
    if text and any(ord(ch) > 0x2FFF for ch in text):
        for fam in ("Malgun Gothic", "NanumGothic", "Noto Sans CJK KR", "Noto Sans KR", "AppleGothic"):
            try:
                return font_manager.findfont(font_manager.FontProperties(family=fam), fallback_to_default=False)
            except ValueError:
                continue
    return font_manager.findfont(font_manager.FontProperties(family=family))


def font(size: int, mono: bool = False, bold: bool = False, text: str = "", lang: str = "en") -> Any:
    """PIL font: Korean (Malgun Gothic, tabular digits) for lang 'ko' or Hangul text, else DejaVu Sans (Mono)."""
    from PIL import ImageFont
    path = None
    if lang == "ko" or has_hangul(text):
        path = korean_font_path(bold)
    if path is None:
        fam = "DejaVu Sans Mono" if mono else "DejaVu Sans"
        path = _font_path(fam, text)
        if bold and "DejaVu" in os.path.basename(path):
            path = path.replace("DejaVuSansMono.ttf", "DejaVuSansMono-Bold.ttf").replace("DejaVuSans.ttf", "DejaVuSans-Bold.ttf")
    key = (path, int(size))
    if key not in _FONT_CACHE:
        _FONT_CACHE[key] = ImageFont.truetype(path, int(size))
    return _FONT_CACHE[key]


def hex_rgb(c: str) -> tuple[int, int, int]:
    c = c.lstrip("#")
    return int(c[0:2], 16), int(c[2:4], 16), int(c[4:6], 16)


def nice_step(x: float) -> float:
    """1, 2, 2.5, 5 x 10^k at or above x."""
    if x <= 0:
        return 1.0
    e = math.floor(math.log10(x))
    for m in (1.0, 2.0, 2.5, 5.0, 10.0):
        if m * 10 ** e >= x - 1e-12:
            return m * 10 ** e
    return 10.0 ** (e + 1)


def wrap_px(d: Any, text: str, fnt: Any, max_w: float) -> list[str]:
    """Greedy word wrap by rendered width (a word longer than the line is cut)."""
    words = str(text).split(" ")
    out: list[str] = []
    cur = ""
    for w in words:
        trial = w if not cur else cur + " " + w
        if d.textlength(trial, font=fnt) <= max_w or not cur:
            cur = trial
        else:
            out.append(cur)
            cur = w
        while d.textlength(cur, font=fnt) > max_w and len(cur) > 1:
            k = len(cur)
            while k > 1 and d.textlength(cur[:k], font=fnt) > max_w:
                k -= 1
            out.append(cur[:k])
            cur = cur[k:]
    out.append(cur)
    return out


def wrap(text: str, width: int) -> list[str]:
    out: list[str] = []
    for para in str(text).splitlines() or [""]:
        out.extend(textwrap.wrap(para, width) or [""])
    return out


# --------------------------------------------------------------------------- #
# side panel: pre-rendered time plots + moving cursor
# --------------------------------------------------------------------------- #
class PlotPanel:
    """Matplotlib plots rendered once; per frame a cursor, the future dimmed and dots on the curves."""

    def __init__(self, log: SimLog, t0: float, t1: float, width: int, height: int, scale: float,
                 bottom_reserve_px: int = 0, lang: str = "en", pushes: Sequence[PushInfo] = (),
                 wind_dir_plot: bool | None = None):
        import matplotlib
        matplotlib.use("Agg")
        rc: dict[str, Any] = {}
        if lang == "ko":
            fam = mpl_korean_family()
            if fam:
                rc = {"font.family": [fam, "DejaVu Sans"], "axes.unicode_minus": False}
        with matplotlib.rc_context(rc):
            self._build(log, t0, t1, width, height, scale, bottom_reserve_px, lang, pushes,
                        wind_dir_varies(log) if wind_dir_plot is None else bool(wind_dir_plot))

    def _build(self, log: SimLog, t0: float, t1: float, width: int, height: int, scale: float, bottom_reserve_px: int,
               lang: str, pushes: Sequence[PushInfo], wind_plot: bool) -> None:
        from matplotlib.backends.backend_agg import FigureCanvasAgg
        from matplotlib.figure import Figure
        L = lang
        self.w, self.h = int(width), int(height)
        self.t0, self.t1 = float(t0), float(max(t1, t0 + 1e-3))
        self.reserve = int(bottom_reserve_px)
        dpi = 100.0
        fig = Figure(figsize=(self.w / dpi, self.h / dpi), dpi=dpi, facecolor=SURFACE)
        FigureCanvasAgg(fig)
        fs = 10.5 * scale
        sel = (log.t >= self.t0 - 1e-9) & (log.t <= self.t1 + 1e-9)
        t = log.t[sel]
        top = 1.0 - 118.0 * scale / self.h
        bottom = (self.reserve + 56.0 * scale) / self.h
        hold_plot = log.has_hold
        n_ax = 4 + int(wind_plot) + int(hold_plot)
        ratios = [1.35, 1.0, 1.0, 1.0] + ([0.85] if wind_plot else []) + ([1.0] if hold_plot else [])
        gs = fig.add_gridspec(n_ax, 1, height_ratios=ratios, left=0.13, right=0.97, top=top, bottom=bottom,
                              hspace=0.42 if n_ax == 4 else (0.5 if n_ax == 5 else 0.56))
        axes = [fig.add_subplot(gs[i, 0]) for i in range(n_ax)]
        z = log.pos[sel, 2]
        series: list[list[tuple[np.ndarray, str]]] = [[] for _ in range(n_ax)]
        lim = log.meta.get("limits") or {}
        # band shading: VTOL phase when the log has one, else the LP state
        codes = log.phase[sel] if log.has_phase else log.state[sel]
        cmap = PHASE_COLORS if log.has_phase else STATE_COLORS
        namef = (lambda c: log.phase_name(c, L)) if log.has_phase else (lambda c: log.state_name(c, L))
        seen: list[int] = []
        if t.size:
            edges = np.flatnonzero(np.diff(codes)) + 1
            starts = np.concatenate([[0], edges])
            ends = np.concatenate([edges, [t.size]])
            for a, b in zip(starts, ends):
                c = int(codes[a])
                ta, tb = t[a], (t[b] if b < t.size else t[-1])
                for ax in axes:
                    ax.axvspan(ta, tb, color=cmap.get(c, "#f3f4f6"), alpha=0.6, lw=0, zorder=0)
                if c not in seen:
                    seen.append(c)

        def header(ax: Any) -> None:
            lg = ax.legend(loc="lower left", bbox_to_anchor=(0.0, 1.0), fontsize=fs * 0.74, frameon=False, ncol=4,
                           borderaxespad=0.15, handlelength=1.6, columnspacing=1.1, handletextpad=0.5)
            for txt in lg.get_texts():
                txt.set_color(INK2)

        ax = axes[0]
        ax.plot(t, z, color=PALETTE_LINE[0], lw=1.6 * scale, label=tr("h_truth", L), zorder=3)
        series[0].append((z, PALETTE_LINE[0]))
        if np.any(np.isfinite(log.h_est[sel])):
            ax.plot(t, log.h_est[sel], color=PALETTE_LINE[2], lw=1.0 * scale, alpha=0.9, label=tr("h_est", L), zorder=3)
        if np.any(np.isfinite(log.h_target[sel])):
            ax.step(t, log.h_target[sel], where="post", color=INK2, lw=1.2 * scale, ls="--", label=tr("h_target", L),
                    zorder=2)
            series[0].append((log.h_target[sel], INK2))
        if np.any(np.isfinite(log.h_ref[sel])):
            ax.plot(t, log.h_ref[sel], color=PALETTE_LINE[1], lw=1.2 * scale, label=tr("h_ref", L), zorder=2)
            series[0].append((log.h_ref[sel], PALETTE_LINE[1]))
        ax.axhline(log.ground_z, color=MUTED, lw=0.8 * scale, ls=":", zorder=1, label=tr("ground", L))
        z_top = float(np.nanmax(z)) if z.size else 0.0
        for key, lab in (("ceiling_m", "ceiling"), ("geofence_m", "geofence")):
            if key in lim and float(lim[key]) <= 1.6 * max(z_top, 1.0):     # only when it would not flatten the plot
                ax.axhline(float(lim[key]), color=GUST_C, lw=0.9 * scale, ls="--" if key == "geofence_m" else "-.",
                           zorder=2, label=f"{tr(lab, L)} {float(lim[key]):g} m")
        self._style(ax, tr("h_axis", L), fs)
        header(ax)
        ax = axes[1]
        tl = log.tilt_deg[sel]
        ax.plot(t, tl, color=PALETTE_LINE[3], lw=1.4 * scale, zorder=3, label=tr("tilt_truth", L))
        series[1].append((tl, PALETTE_LINE[3]))
        tsp = log.tilt_sp_deg[sel] if log.tilt_sp_deg is not None else None
        if tsp is not None and np.any(np.isfinite(tsp)):
            # the position loop's tilt setpoint (v0.8): what the position controller asks the attitude loop for
            ax.plot(t, tsp, color=HOLD_C, lw=1.2 * scale, ls=(0, (4, 2)), zorder=3, label=tr("tilt_sp", L))
            series[1].append((tsp, HOLD_C))
            tl = np.concatenate([tl, tsp[np.isfinite(tsp)]])
        if "tilt_abort_deg" in lim:
            ax.axhline(float(lim["tilt_abort_deg"]), color=GUST_C, lw=0.9 * scale, ls="--", zorder=2,
                       label=tr("abort_lim", L, v=float(lim["tilt_abort_deg"])))
        y_hi = max(5.0, 1.1 * float(np.nanmax(tl)) if tl.size else 5.0, 1.08 * float(lim.get("tilt_abort_deg", 0.0)))
        if pushes:
            band = float((log.meta.get("criteria") or {}).get("gust_band_deg", 2.0))
            # with pushes the interesting range is a few degrees: zoom to the response, keep the abort line only if near
            y_push = max(6.0, 1.25 * max(p.peak_tilt_deg for p in pushes), 1.15 * float(np.nanmax(tl)) if tl.size else 0.0)
            if y_push < 0.6 * y_hi:
                y_hi = y_push
            ax.axhline(band, color=PUSH_C, lw=0.9 * scale, ls=":", zorder=2, label=tr("band", L, v=band))
            for j, p in enumerate(pushes):
                ax.axvspan(p.t0, max(p.t1, p.t0 + 0.02), color=PUSH_C, alpha=0.35, lw=0, zorder=1,
                           label=tr("push_span", L) if j == 0 else None)
                ax.text(p.t0, 0.98, f"#{p.i}", transform=ax.get_xaxis_transform(), fontsize=fs * 0.7, color=PUSH_C,
                        ha="center", va="top", weight="bold", zorder=4)
        self._style(ax, tr("tilt_axis", L), fs)
        ax.set_ylim(0.0, y_hi)
        header(ax)
        ax = axes[2]
        u = log.throttle[sel]
        ax.plot(t, u, color=PALETTE_LINE[4], lw=1.4 * scale, zorder=3, label=tr("thr_cmd", L))
        series[2].append((u, PALETTE_LINE[4]))
        ax.axhline(log.u_max, color=GUST_C, lw=0.9 * scale, ls="--", zorder=2, label=f"u_max {log.u_max:g}")
        satm = u >= log.u_max - 1e-3
        if np.any(satm):
            ax.fill_between(t, 0, 1, where=satm, transform=ax.get_xaxis_transform(), color=GUST_C, alpha=0.13, lw=0,
                            label=tr("at_umax", L))
        if "hover_throttle" in lim:
            ax.axhline(float(lim["hover_throttle"]), color=INK2, lw=0.9 * scale, ls=":", zorder=2,
                       label=tr("hover", L, v=float(lim["hover_throttle"])))
        self._style(ax, tr("thr_axis", L), fs)
        ax.set_ylim(-0.03, max(1.06, float(np.nanmax(u)) + 0.05) if u.size else 1.06)
        header(ax)
        ax = axes[3]
        dd = np.degrees(log.delta[sel])
        for i in range(4):
            ax.plot(t, dd[:, i], color=VANE_COLORS[i], lw=1.0 * scale, label=tr("vane_n", L, i=i + 1), zorder=3)
            series[3].append((dd[:, i], VANE_COLORS[i]))
        ax.axhline(0.0, color=MUTED, lw=0.6 * scale)
        self._style(ax, tr("vane_axis" if n_ax < 6 else "vane_axis_short", L), fs)     # 6 plots: short (they are low)
        header(ax)
        if wind_plot:
            ax = axes[4]
            wm = log.wind_mean[sel] if log.wind_mean is not None else log.wind[sel]
            sp = np.hypot(wm[:, 0], wm[:, 1])
            wd = wind_dir_deg_series(wm)
            ok = sp >= WIND_SWING_MIN_MPS
            c0 = float(np.median(wd[ok])) if np.any(ok) else 0.0
            wd = np.where(ok, wd, np.nan)
            ax.plot(t, wd, color=WIND_C, lw=1.4 * scale, zorder=3,
                    label=tr("wdir_mean", L) + (f" ({tr('compass_smoothed', L)})" if log.wind_mean_source != "log" else ""))
            series[4].append((wd, WIND_C))
            lo_, hi_ = np.nanmin(wd) if np.any(ok) else c0 - 30, np.nanmax(wd) if np.any(ok) else c0 + 30
            pad_ = max(10.0, 0.1 * (hi_ - lo_))
            ax.set_ylim(lo_ - pad_, hi_ + pad_)
            from matplotlib.ticker import FuncFormatter
            ax.yaxis.set_major_formatter(FuncFormatter(lambda v, _p: f"{v % 360.0:.0f}"))
            self._style(ax, tr("wdir_axis", L), fs)
            header(ax)
        if hold_plot:
            # v0.8: distance of the true CG from the hold point (NaN before the hold point is set)
            ax = axes[-1]
            j = len(axes) - 1
            hd = log.hold_dist()[sel]
            ax.plot(t, hd, color=HOLD_C, lw=1.5 * scale, zorder=3, label=tr("hold_dist_l", L))
            series[j].append((hd, HOLD_C))
            crit = log.station_keeping_limit()
            hmax = float(np.nanmax(hd)) if np.any(np.isfinite(hd)) else 1.0
            crit_shown = crit is not None and crit <= 3.0 * max(hmax, 0.5)    # not when it would flatten the curve
            if crit_shown:
                ax.axhline(crit, color=GUST_C, lw=0.9 * scale, ls="--", zorder=2, label=tr("hold_crit", L, v=crit))
            if log.gps_ok is not None:
                bad = (~log.gps_ok[sel]) & (log.state[sel] == STATE_POWERED)
                if np.any(bad):
                    ax.fill_between(t, 0, 1, where=bad, transform=ax.get_xaxis_transform(), color=GUST_C, alpha=0.16,
                                    lw=0, label=tr("gps_lost_span", L))
            ax.set_ylim(0.0, max(1.0, 1.12 * hmax, 1.15 * crit if crit_shown else 0.0))
            self._style(ax, tr("hold_axis", L), fs)
            header(ax)
        axes[-1].set_xlabel(tr("time_axis", L), fontsize=fs, color=INK2)
        for a in axes:
            a.set_xlim(self.t0, self.t1)
        for a in axes[:-1]:
            a.tick_params(labelbottom=False)
        fig.text(0.03, 1.0 - 22.0 * scale / self.h, tr("telemetry", L), fontsize=fs * 1.15,
                 color=INK, weight="bold", va="center")
        if seen:
            from matplotlib.patches import Patch
            fig.legend(handles=[Patch(facecolor=cmap.get(c, "#f3f4f6"), edgecolor=GRID, label=namef(c)) for c in seen],
                       loc="upper left", bbox_to_anchor=(0.025, 1.0 - 34.0 * scale / self.h), ncol=6, frameon=False,
                       fontsize=fs * 0.74, title=tr("bg_phase" if log.has_phase else "bg_state", L),
                       title_fontsize=fs * 0.74, alignment="left", handlelength=1.2, columnspacing=0.9)
        fig.canvas.draw()
        buf = np.asarray(fig.canvas.buffer_rgba())[:, :, :3]
        self.bg = np.full((self.h, self.w, 3), hex_rgb(SURFACE), dtype=np.uint8)
        hh, ww = min(self.h, buf.shape[0]), min(self.w, buf.shape[1])
        self.bg[:hh, :ww] = buf[:hh, :ww]
        self.boxes: list[tuple[int, int, int, int, float, float]] = []    # x0, x1, row0, row1, y_lo, y_hi
        for a in axes:
            bb = a.get_window_extent()
            y0, y1 = a.get_ylim()
            self.boxes.append((int(round(bb.x0)), int(round(bb.x1)), int(round(self.h - bb.y1)), int(round(self.h - bb.y0)),
                               float(y0), float(y1)))
        self.series = series
        self.t_sel = t
        self.scale = scale

    @staticmethod
    def _style(ax: Any, ylabel: str, fs: float) -> None:
        ax.set_facecolor(SURFACE)
        for side in ("top", "right"):
            ax.spines[side].set_visible(False)
        for side in ("left", "bottom"):
            ax.spines[side].set_color(GRID)
        ax.grid(True, color=GRID, lw=0.6)
        ax.tick_params(colors=INK2, labelsize=fs * 0.85)
        ax.set_ylabel(ylabel, fontsize=fs * 0.9, color=INK2)

    def x_px(self, t: float) -> float:
        x0, x1 = self.boxes[0][0], self.boxes[0][1]
        return x0 + (min(max(t, self.t0), self.t1) - self.t0) / (self.t1 - self.t0) * (x1 - x0)

    def y_px(self, i: int, v: float) -> float:
        _, _, r0, r1, lo, hi = self.boxes[i]
        return r0 + (hi - v) / (hi - lo) * (r1 - r0)

    def frame(self, t: float) -> np.ndarray:
        img = self.bg.copy()
        x = int(round(self.x_px(t)))
        for (x0, x1, r0, r1, _, _) in self.boxes:
            if x + 1 < x1:
                reg = img[r0:r1, x + 1:x1].astype(np.float32)
                img[r0:r1, x + 1:x1] = (reg * 0.45 + 255.0 * 0.55).astype(np.uint8)
            img[r0:r1, max(x0, x - 1):min(x1, x + 1) + 1] = hex_rgb(INK)
        from PIL import Image, ImageDraw
        im = Image.fromarray(img)
        dr = ImageDraw.Draw(im)
        if self.t_sel.size:
            j = int(np.clip(np.searchsorted(self.t_sel, t), 0, self.t_sel.size - 1))
            rad = max(2.0, 3.2 * self.scale)
            for i, ser in enumerate(self.series):
                _, _, r0, r1, _, _ = self.boxes[i]
                for y, col in ser:
                    v = float(y[j])
                    if not np.isfinite(v):
                        continue
                    yy = min(max(self.y_px(i, v), r0), r1)
                    dr.ellipse([x - rad, yy - rad, x + rad, yy + rad], fill=col, outline=INK)
        return np.asarray(im)


PALETTE_LINE = ("#2a78d6", "#eb6834", "#1baf7a", "#4a3aa7", "#e87ba4")


# --------------------------------------------------------------------------- #
# top-down drift inset (PIL)
# --------------------------------------------------------------------------- #
class DriftInset:
    """Top-down view (x right, y up): distance rings around the pad, the whole track faint, the flown
    part bold, the current wind (steady blue + gust red, stacked; a field of light arrows across the map
    that turns with the mean wind), push markers (where and which way the vehicle was pushed) and the
    camera's look direction."""

    def __init__(self, xy_all: np.ndarray, size: int, cam_azimuth_deg: float, scale: float, lang: str = "en"):
        from PIL import Image, ImageDraw
        self.lang = lang
        self.s = int(size)
        self.scale = sc = scale
        self.hh = int(46 * sc)                    # header band
        self.fh = int(26 * sc)                    # footer band
        self.H = self.s + self.hh + self.fh
        r_max = float(np.max(np.hypot(xy_all[:, 0], xy_all[:, 1]))) if xy_all.size else 0.0
        self.R = nice_step(max(2.0, 1.1 * r_max))
        self.step = nice_step(self.R / 4.0)
        self.k = (self.s / 2.0 - 10 * sc) / self.R
        self.c = np.array([self.s / 2.0, self.hh + self.s / 2.0])
        bg = Image.new("RGBA", (self.s, self.H), (0, 0, 0, 0))
        d = ImageDraw.Draw(bg)
        d.rounded_rectangle([0, 0, self.s - 1, self.H - 1], radius=int(10 * sc), fill=(252, 252, 251, 235),
                            outline=(120, 120, 120, 255), width=max(1, int(sc)))
        L = lang
        f_small = font(int(11 * sc), lang=L)
        cx, cy = self.c
        r = self.step
        while r <= self.R + 1e-9:
            rr = r * self.k
            d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=hex_rgb(RING) + (160,), width=1)
            lab = f"{r:g} m"
            d.text((cx - rr * 0.7071 - d.textlength(lab, font=f_small) - 1 * sc, cy - rr * 0.7071 - 12 * sc), lab,
                   fill=hex_rgb(RING) + (255,), font=f_small)
            r += self.step
        d.line([cx - 7 * sc, cy, cx + 7 * sc, cy], fill=(60, 60, 60, 255), width=1)
        d.line([cx, cy - 7 * sc, cx, cy + 7 * sc], fill=(60, 60, 60, 255), width=1)
        ps = 4 * sc
        d.rectangle([cx - ps, cy - ps, cx + ps, cy + ps], outline=hex_rgb(PAD_PART[1]) + (255,), width=max(1, int(1.5 * sc)))
        if xy_all.shape[0] > 1:
            d.line([self._px(p) for p in xy_all[:: max(1, xy_all.shape[0] // 600)]], fill=(150, 150, 165, 170),
                   width=max(1, int(1.2 * sc)))
        # axes (footer right) + camera look direction (on the map edge, pointing in)
        ax0 = (self.s - 58 * sc, self.H - 8 * sc)
        d.line([ax0, (ax0[0] + 22 * sc, ax0[1])], fill=(70, 70, 70, 255), width=max(1, int(1.5 * sc)))
        d.text((ax0[0] + 25 * sc, ax0[1] - 9 * sc), "x", fill=(70, 70, 70, 255), font=f_small)
        d.line([ax0, (ax0[0], ax0[1] - 16 * sc)], fill=(70, 70, 70, 255), width=max(1, int(1.5 * sc)))
        d.text((ax0[0] - 12 * sc, ax0[1] - 20 * sc), "y", fill=(70, 70, 70, 255), font=f_small)
        self.cam_az = float(cam_azimuth_deg)
        self.f_small = f_small
        d.text((10 * sc, 7 * sc), tr("top_view", L), fill=(20, 20, 20, 255), font=font(int(13 * sc), bold=True, lang=L))
        d.text((10 * sc, 25 * sc), tr("drift_sub", L), fill=(90, 90, 90, 255), font=f_small)
        d.line([(6 * sc, self.hh - 2), (self.s - 6 * sc, self.hh - 2)], fill=(200, 200, 200, 255), width=1)
        self.bg = bg

    def _px(self, xy: np.ndarray) -> tuple[float, float]:
        return (float(self.c[0] + float(xy[0]) * self.k), float(self.c[1] - float(xy[1]) * self.k))

    def frame(self, xy_past: np.ndarray, wind: np.ndarray, gust: np.ndarray, gust_active: bool,
              wind_mean: np.ndarray | None = None, pushes: Sequence[tuple[np.ndarray, np.ndarray, float]] = (),
              cam_az: float | None = None, hold: np.ndarray | None = None, hold_d: float | None = None,
              est: np.ndarray | None = None) -> Any:
        """``pushes``: (contact point xy, force xy, age s; age < 0 = active now) of the pushes started so far;
        ``cam_az``: the main camera's look direction this frame (deg; default the one given at construction);
        v0.8: ``hold`` the hold point xy (None / NaN = none yet) with ``hold_d`` the CG's distance from it (footer),
        ``est`` where the NAV believes the vehicle is relative to the hold point (``SimLog.est_rel_xy``; a green
        ring)."""
        from PIL import Image, ImageDraw
        im = self.bg.copy()
        sc = self.scale
        L = self.lang
        a = math.radians(self.cam_az if cam_az is None else float(cam_az))
        cx, cy = self.c
        dcam = ImageDraw.Draw(im)
        e = np.array([cx - math.cos(a) * (self.s / 2 - 22 * sc), cy + math.sin(a) * (self.s / 2 - 22 * sc)])
        tip = e + np.array([math.cos(a), -math.sin(a)]) * 14 * sc
        self._arrow(dcam, e, tip, "#5a5f66", sc)
        dcam.text((e[0] - 10 * sc, e[1] + 2 * sc), tr("cam", L), fill=(90, 90, 90, 255), font=self.f_small)
        # wind field: light arrows on a grid across the map, turning with the mean wind (length ~ speed)
        wm = np.asarray(wind if wind_mean is None else wind_mean, dtype=float)
        wmag_m = float(np.hypot(wm[0], wm[1]))
        if wmag_m > 0.05:
            fl = Image.new("RGBA", im.size, (0, 0, 0, 0))
            dfl = ImageDraw.Draw(fl)
            u = np.array([wm[0], -wm[1]]) / wmag_m
            ln = min(1.0, wmag_m / 6.0) * 26 * sc + 8 * sc
            for gx in (-1, 0, 1):
                for gy in (-1, 0, 1):
                    c = self.c + np.array([gx, gy]) * (self.s * 0.30)
                    self._arrow(dfl, c - 0.5 * ln * u, c + 0.5 * ln * u, "#9cc3ea", sc * 0.8, alpha=150)
            im = Image.alpha_composite(im, fl)
        d = ImageDraw.Draw(im)
        if xy_past.shape[0] > 1:
            d.line([self._px(p) for p in xy_past], fill=hex_rgb(TRACK_C) + (255,), width=max(1, int(2.2 * sc)))
        # pushes: a cross where the contact was, an arrow along the force (orange; faded once over)
        for pxy, fxy, age in pushes:
            c = np.array(self._px(pxy))
            fm = float(np.hypot(fxy[0], fxy[1]))
            a = 255 if age < 0 else int(110 + 145 * max(0.0, 1.0 - age / PUSH_AFTERGLOW_S))
            r_ = 4 * sc
            d.line([(c[0] - r_, c[1] - r_), (c[0] + r_, c[1] + r_)], fill=hex_rgb(PUSH_C) + (a,), width=max(1, int(2 * sc)))
            d.line([(c[0] - r_, c[1] + r_), (c[0] + r_, c[1] - r_)], fill=hex_rgb(PUSH_C) + (a,), width=max(1, int(2 * sc)))
            if fm > 1e-6:
                u = np.array([fxy[0], -fxy[1]]) / fm
                self._arrow(d, c - u * 22 * sc, c - u * 5 * sc, PUSH_C, sc, alpha=a)
        p = self._px(xy_past[-1])
        has_hold = hold is not None and bool(np.all(np.isfinite(hold)))
        if has_hold:
            # the hold point: a ring (HOLD_RING_M, at least 5 px) + cross, and a thin line to the vehicle
            hp = self._px(hold)
            hr = max(5 * sc, HOLD_RING_M * self.k)
            hc = max(8 * sc, HOLD_CROSS_M * self.k)
            col = hex_rgb(HOLD_C) + (255,)
            d.line([hp, p], fill=col, width=max(1, int(1.4 * sc)))
            d.ellipse([hp[0] - hr, hp[1] - hr, hp[0] + hr, hp[1] + hr], outline=col, width=max(1, int(1.6 * sc)))
            d.line([(hp[0] - hc, hp[1]), (hp[0] + hc, hp[1])], fill=col, width=max(1, int(1.2 * sc)))
            d.line([(hp[0], hp[1] - hc), (hp[0], hp[1] + hc)], fill=col, width=max(1, int(1.2 * sc)))
        if est is not None and bool(np.all(np.isfinite(est))):
            ep = self._px(est)
            er = 4 * sc
            d.ellipse([ep[0] - er, ep[1] - er, ep[0] + er, ep[1] + er], outline=hex_rgb(EST_C) + (255,),
                      width=max(1, int(1.6 * sc)))
            d.text((8 * sc, self.H - self.fh - 16 * sc), tr("est_short", L), fill=hex_rgb(EST_C) + (255,),
                   font=font(int(10 * sc), lang=L))
        rr = 5 * sc
        d.ellipse([p[0] - rr, p[1] - rr, p[0] + rr, p[1] + rr], fill=hex_rgb(BODY_PARTS[4][1]) + (255,),
                  outline=(20, 20, 20, 255), width=max(1, int(1.2 * sc)))
        dist = float(np.hypot(*xy_past[-1]))
        f_ft = font(int(13 * sc), mono=True, bold=True, lang=L)
        txt = tr("drift_val", L, d=dist)
        d.text((10 * sc, self.H - 22 * sc), txt, fill=(20, 20, 20, 255), font=f_ft)
        if has_hold and hold_d is not None and np.isfinite(hold_d):
            ht = tr("hold_pt_short", L, d=float(hold_d))
            x_h = 10 * sc + d.textlength(txt, font=f_ft) + 10 * sc
            f_h = f_ft if x_h + d.textlength(ht, font=f_ft) < self.s - 62 * sc else font(int(11 * sc), mono=True, bold=True, lang=L)
            d.text((x_h, self.H - 22 * sc + (0 if f_h is f_ft else 2 * sc)), ht, fill=hex_rgb("#1f7fb8") + (255,), font=f_h)
        # wind (header right): arrow in a small box, steady blue then gust red, stacked; length 3.2 px per m/s
        box_c = np.array([self.s - 26 * sc, self.hh / 2.0])
        kk = 2.6 * sc
        flip = np.array([1.0, -1.0])
        steady = wind[:2] - (gust[:2] if gust_active else 0.0)
        p0 = box_c - 0.5 * kk * wind[:2] * flip
        p1 = p0 + kk * steady * flip
        d.ellipse([box_c[0] - 20 * sc, box_c[1] - 20 * sc, box_c[0] + 20 * sc, box_c[1] + 20 * sc],
                  outline=(200, 200, 200, 255), width=1)
        self._arrow(d, p0, p1, WIND_C, sc)
        if gust_active:
            self._arrow(d, p1, p1 + kk * gust[:2] * flip, GUST_C, sc)
        wm = float(np.hypot(wind[0], wind[1]))
        ft = font(int(12 * sc), mono=True, bold=True, lang=L)
        wtxt = tr("wind_val", L, w=wm)
        x_t = self.s - 52 * sc - d.textlength(wtxt, font=ft)
        d.text((x_t, 8 * sc), wtxt, fill=hex_rgb(WIND_C) + (255,), font=ft)
        if gust_active:
            gm = float(np.hypot(gust[0], gust[1]))
            d.text((x_t, 24 * sc), tr("gust_val", L, g=gm), fill=hex_rgb(GUST_C) + (255,),
                   font=font(int(11 * sc), mono=True, bold=True, lang=L))
        return im

    @staticmethod
    def _arrow(d: Any, p0: np.ndarray, p1: np.ndarray, col: str, sc: float, alpha: int = 255) -> None:
        v = np.asarray(p1, dtype=float) - np.asarray(p0, dtype=float)
        L = float(np.hypot(*v))
        if L < 1.0:
            return
        u = v / L
        nrm = np.array([-u[1], u[0]])
        hl = min(8 * sc, 0.6 * L)
        p0 = np.asarray(p0, dtype=float)
        p1 = np.asarray(p1, dtype=float)
        d.line([tuple(p0), tuple(p1 - u * hl * 0.6)], fill=hex_rgb(col) + (int(alpha),), width=max(1, int(2.5 * sc)))
        d.polygon([tuple(p1), tuple(p1 - u * hl + nrm * hl * 0.55), tuple(p1 - u * hl - nrm * hl * 0.55)],
                  fill=hex_rgb(col) + (int(alpha),))


class WindCompass:
    """Compass rose of the HUD (top view, x right, y up like the top-view inset): rings in m/s, the current mean
    wind as an arrow from the centre (direction = where the air goes, length = speed), the gust on top in red,
    and the tip of the mean-wind vector over the last ``WIND_TRACE_S`` seconds as a fading trace - a wind whose
    direction swings back and forth draws an arc."""

    def __init__(self, size_px: int, w_max_mps: float, scale: float, lang: str = "en", smoothed: bool = False):
        from PIL import Image, ImageDraw
        self.lang = lang
        self.sc = sc = scale
        self.W = int(size_px)
        self.th = int(36 * sc)                    # title band
        self.fh = int(20 * sc)                    # footer band
        self.H = self.W + self.th + self.fh
        self.Rw = nice_step(max(2.0, 1.05 * float(w_max_mps)))
        self.R = self.W / 2.0 - 22 * sc
        self.c = np.array([self.W / 2.0, self.th + self.W / 2.0])
        L = lang
        bg = Image.new("RGBA", (self.W, self.H), (0, 0, 0, 0))
        d = ImageDraw.Draw(bg)
        d.rounded_rectangle([0, 0, self.W - 1, self.H - 1], radius=int(8 * sc), fill=(14, 20, 30, 178))
        fs = font(int(10 * sc), lang=L)
        cx, cy = self.c
        step = nice_step(self.Rw / 3.0)
        r = step
        while r <= self.Rw + 1e-9:
            rr = r / self.Rw * self.R
            d.ellipse([cx - rr, cy - rr, cx + rr, cy + rr], outline=(120, 135, 155, 200), width=1)
            d.text((cx + rr * 0.72 + 2 * sc, cy + rr * 0.68), f"{r:g}", fill=(150, 165, 185, 255), font=fs)
            r += step
        for a in range(0, 360, 30):
            ar = math.radians(a)
            u = np.array([math.cos(ar), -math.sin(ar)])
            r0 = self.R * (0.9 if a % 90 else 0.82)
            d.line([tuple(self.c + u * r0), tuple(self.c + u * self.R)], fill=(170, 182, 198, 255), width=max(1, int(sc)))
        R = self.R
        for lab, x, y in (("0° +x", None, cy - 15 * sc), ("90° +y", cx + 4 * sc, cy - R + 1 * sc),
                          ("180°", cx - R + 3 * sc, cy - 15 * sc), ("270°", cx + 4 * sc, cy + R - 15 * sc)):
            if x is None:
                x = cx + R - d.textlength(lab, font=fs) - 3 * sc
            d.text((x, y), lab, fill=(200, 208, 220, 255), font=fs)
        d.text((10 * sc, 5 * sc), tr("compass_title", L), fill=TONES["muted"] + (255,), font=font(int(12 * sc), bold=True, lang=L))
        d.text((10 * sc, 20 * sc), tr("compass_trace", L) + "   [m/s]", fill=(150, 160, 175, 255), font=fs)
        self.smoothed = smoothed
        self.bg = bg

    def _p(self, w: np.ndarray) -> np.ndarray:
        return self.c + np.array([float(w[0]), -float(w[1])]) / self.Rw * self.R

    def frame(self, wind_mean: np.ndarray, gust: np.ndarray, gust_active: bool, trace: np.ndarray) -> Any:
        """``trace``: (M, 2+) mean-wind vectors of the last WIND_TRACE_S seconds, oldest first."""
        from PIL import ImageDraw
        im = self.bg.copy()
        d = ImageDraw.Draw(im)
        sc = self.sc
        m = trace.shape[0]
        if m > 1:
            pts = [self._p(w) for w in trace]
            for i in range(1, m):
                a = int(40 + 200 * i / (m - 1))
                d.line([tuple(pts[i - 1]), tuple(pts[i])], fill=(120, 190, 255, a), width=max(1, int(2 * sc)))
        wm = np.asarray(wind_mean, dtype=float)
        sp = float(np.hypot(wm[0], wm[1]))
        if sp > 0.05:
            tip = self._p(wm)
            DriftInset._arrow(d, self.c, tip, "#6fb3ff", sc * 1.2)
            if gust_active:
                DriftInset._arrow(d, tip, tip + np.array([gust[0], -gust[1]]) / self.Rw * self.R, GUST_C, sc * 1.2)
            txt = f"{sp:4.1f} m/s  → {wind_heading_deg(wm):3.0f}°" + ("  (" + tr("compass_smoothed", self.lang) + ")"
                                                                       if self.smoothed else "")
        else:
            txt = tr("calm", self.lang)
        d.ellipse([self.c[0] - 3 * sc, self.c[1] - 3 * sc, self.c[0] + 3 * sc, self.c[1] + 3 * sc], fill=(230, 235, 240, 255))
        ft = font(int(12 * sc), mono=True, bold=True, lang=self.lang)
        d.text((10 * sc, self.H - self.fh - 2 * sc), txt, fill=(150, 200, 255, 255), font=ft)
        return im


# --------------------------------------------------------------------------- #
# 3-D scene (PyVista, off-screen)
# --------------------------------------------------------------------------- #
def _load_stl(name: str, warnings: list[str], stl_dir: str | None = None) -> Any:
    import pyvista as pv
    path = os.path.join(STL_DIR if stl_dir is None else stl_dir, f"{name}.stl")
    if os.path.exists(path):
        m = pv.read(path)
        return m.scale(1e-3, inplace=False)
    warnings.append(f"{_rel(path)} not found: drawn as a simple stand-in")
    return None


def _standin(name: str, geom: VehicleGeom) -> Any:
    import pyvista as pv
    if name == "launch_pad":
        return pv.Cylinder(center=(0, 0, -0.0025), direction=(0, 0, 1), radius=0.095, height=0.005)
    if name == "vane":
        return pv.Box(bounds=(0.013, 0.043, -0.0015, 0.0015, 0.0146, 0.0466))
    if name == "nose_cone":
        return pv.Cone(center=(0, 0, 0.78), direction=(0, 0, 1), height=0.155, radius=0.06, resolution=48)
    spans = {"propulsion_module": (0.0, 0.19), "intake_module": (0.19, 0.35), "avionics_module": (0.35, 0.53),
             "payload_module": (0.53, 0.70), "top_cap": (0.459, 0.475)}
    z0, z1 = spans.get(name, (0.0, 0.1))
    return pv.Cylinder(center=(0, 0, 0.5 * (z0 + z1)), direction=(0, 0, 1), radius=0.06, height=z1 - z0, resolution=48)


def gps_pod_paths(stl_dir: str | None = None) -> list[str]:
    """The GPS antenna pod STL(s) of the CAD export: ``GPS_POD_NAMES`` first (all of them that exist: a pod and its
    lid), else every ``*gps*.stl`` in the directory (sorted); [] when none."""
    d = STL_DIR if stl_dir is None else stl_dir
    named = [os.path.join(d, f"{n}.stl") for n in GPS_POD_NAMES if os.path.exists(os.path.join(d, f"{n}.stl"))]
    if named:
        return named
    try:
        return sorted(os.path.join(d, f) for f in os.listdir(d) if f.lower().endswith(".stl") and "gps" in f.lower())
    except OSError:
        return []


def gps_pod_standin() -> Any:
    """The labelled stand-in of the pod (vehicle frame, m): a box at ``GPS_POD_STANDIN`` (option B of the v0.8 CAD
    study), radial from r0 to r1, tangential +-half_w, z0..z1, rotated to theta."""
    import pyvista as pv
    g = GPS_POD_STANDIN
    box = pv.Box(bounds=(g["r0_m"], g["r1_m"], -g["half_w_m"], g["half_w_m"], g["z0_m"], g["z1_m"]))
    return box.transform(homogeneous(rot_z(math.radians(g["theta_deg"]))), inplace=False)


def load_gps_pod(warnings: list[str]) -> tuple[list[Any], str]:
    """(meshes in metres, source): the CAD STL(s) when present, else the stand-in box (with a warning)."""
    import pyvista as pv
    paths = gps_pod_paths()
    if paths:
        return [pv.read(p).scale(1e-3, inplace=False) for p in paths], ", ".join(_rel(p) for p in paths)
    warnings.append(f"GPS pod STL not found in {_rel(STL_DIR)} (gps_pod.stl / *gps*.stl): drawn as a stand-in box at "
                    "the option-B place of the v0.8 CAD study")
    return [gps_pod_standin()], "stand-in (v0.8 CAD study option B)"


# --------------------------------------------------------------------------- #
# v0.9 (v6): the mesh set of the hardware configuration, top-cap items, the tether
# --------------------------------------------------------------------------- #
def hardware_stl_dir(hw: Hardware, warnings: list[str] | None = None) -> str:
    """The CAD STL directory of ``hw``: legacy / rocket ``STL_DIR`` (read at call time: tests patch it); vtol
    ``stl_vtol`` / ``stl_vtol_tether`` / ``stl_vtol_hybrid`` next to it (a missing power variant -> ``stl_vtol`` with a
    warning: the outside of the vehicle is the same, only the tether parts inside differ)."""
    if not hw.vtol:
        return STL_DIR
    root = os.path.dirname(STL_DIR)
    d = os.path.join(root, VTOL_STL_DIRS[hw.power])
    if not os.path.isdir(d) and hw.power != "battery":
        alt = os.path.join(root, VTOL_STL_DIRS["battery"])
        if warnings is not None:
            warnings.append(f"{_rel(d)} not found: the VTOL {hw.power} vehicle drawn from {_rel(alt)}")
        return alt
    return d


def body_part_list(hw: Hardware) -> tuple[tuple[str, str], ...]:
    """(name, schematic colour) of the printed parts drawn as one rigid body, in drawing order (the first four give the
    body radius).  Legacy: exactly BODY_PARTS (the pre-v0.9 drawing)."""
    if hw.legacy:
        return BODY_PARTS
    if hw.vtol:
        return VTOL_BODY_PARTS + V09_BODY_EXTRAS
    return BODY_PARTS + V09_BODY_EXTRAS


def stl_fin_angle(vane_mesh: Any) -> float:
    """The fin angle of the CAD's single vane / servo-cover STLs (both are exported for fin 1): the vane centroid's
    azimuth (rad)."""
    p = np.asarray(vane_mesh.points, dtype=float)
    return math.atan2(float(np.mean(p[:, 1])), float(np.mean(p[:, 0])))


def fin_part_mesh(hw: Hardware, geom: VehicleGeom, theta_stl: float, warnings: list[str]) -> Any:
    """The direct-drive servo covers of all fins as one mesh (m): ``servo_cover.stl`` (fin 1) rotated from the vane STL's
    fin angle to every vane theta (v0.9: the fins sit at the vane angles).  None for the legacy set or when the STL is
    missing (a warning; the covers are a small detail, no stand-in)."""
    if hw.legacy:
        return None
    path = os.path.join(hardware_stl_dir(hw), f"{FIN_PART[0]}.stl")
    if not os.path.exists(path):
        warnings.append(f"{_rel(path)} not found: servo covers not drawn")
        return None
    import pyvista as pv
    m = pv.read(path).scale(1e-3, inplace=False)
    out = None
    for th in geom.vane_thetas_rad:
        mi = m.transform(homogeneous(rot_z(th - theta_stl)), inplace=False)
        out = mi if out is None else out + mi
    return out


def _box(lo: Sequence[float], hi: Sequence[float]) -> Any:
    import pyvista as pv
    return pv.Box(bounds=(lo[0], hi[0], lo[1], hi[1], lo[2], hi[2]))


def _top_cap_params() -> dict[str, Any]:
    try:
        with open(PARAMS_PATH, "r", encoding="utf-8") as f:
            return json.load(f)["configs"]["vtol"]["top_cap"]
    except (OSError, KeyError, ValueError, TypeError):
        return {}


def _logic_collar_xy_r(hw: Hardware) -> tuple[float, float, float] | None:
    """Switch-collar round (2026-11-05, params configs.vtol.logic_power.collar): (x, y, outer radius) in m of the protective
    collar printed with the top cap in the builds the logic power applies to, else None."""
    try:
        with open(PARAMS_PATH, "r", encoding="utf-8") as f:
            lp = json.load(f)["configs"]["vtol"].get("logic_power") or {}
    except (OSError, KeyError, ValueError, TypeError):
        return None
    co, sw = lp.get("collar"), lp.get("switch") or {}
    if not co or hw.power not in (lp.get("applies_to_power_sources") or []):
        return None
    a = math.radians(float(sw.get("theta_deg", 0.0)))
    r = float(sw.get("r", 0.0)) * 1e-3
    return r * math.cos(a), r * math.sin(a), (0.5 * float(co["inner_d"]) + float(co["wall_t"])) * 1e-3


def top_items(hw: Hardware, top_z_m: float) -> list[tuple[str, Any]]:
    """[(material key, mesh in m, body frame)] of the non-printed items on the VTOL top cap, at the places of params
    ``configs.vtol.top_cap`` (sizes there are vendor data / ASSUMED): GPS ground plane (copper), patch (ceramic) and board
    (PCB) taped on the plate (tape_t), the arming toggle (nut + lever, body ASSUMED), and for tether / hybrid the M5 eye
    bolt (shank + eye ring, eye OD / height ASSUMED) and the spring boot above it (to ``tether_boot_top_b``).  [] for
    rocket / legacy."""
    if not hw.vtol:
        return []
    import pyvista as pv
    tc = _top_cap_params()
    g = tc.get("gps", {})
    out: list[tuple[str, Any]] = []
    mm = 1e-3
    tape = float(g.get("tape_t_ASSUMED", 1.0))
    z0 = top_z_m + tape * mm

    def polar(th_deg: float, r_mm: float) -> tuple[float, float]:
        a = math.radians(th_deg)
        return r_mm * mm * math.cos(a), r_mm * mm * math.sin(a)
    if g:
        px, py = polar(float(g.get("patch_center_theta_deg", 0.0)), float(g.get("patch_center_r", 33.0)))
        s, t_gp = 0.5 * float(g.get("ground_plane_side", 40.0)) * mm, float(g.get("ground_plane_t", 0.8)) * mm
        out.append(("copper", _box((px - s, py - s, z0), (px + s, py + s, z0 + t_gp))))
        pl_, pw_, ph_ = (float(x) * mm for x in g.get("patch_lwh", (25.5, 25.5, 8.8)))
        out.append(("patch", _box((px - pl_ / 2, py - pw_ / 2, z0 + t_gp), (px + pl_ / 2, py + pw_ / 2, z0 + t_gp + ph_))))
        th_b = float(g.get("board_center_theta_deg", 270.0))
        bx, by = polar(th_b, float(g.get("board_center_r", 35.0)))
        bl, bw, bh = (float(x) * mm for x in g.get("board_lwh", (36.0, 25.8, 4.0)))
        board = _box((-bl / 2, -bw / 2, z0), (bl / 2, bw / 2, z0 + bh))
        # long side tangential: local x along e_theta = (-sin, cos)
        out.append(("pcb", board.transform(homogeneous(rot_z(math.radians(th_b) + 0.5 * math.pi), (bx, by, 0.0)),
                                           inplace=False)))
    sw = tc.get("switch_hole", {})
    if sw:
        sx, sy = polar(float(sw.get("theta_deg", 90.0)), float(sw.get("r", 40.0)))
        out.append(("steel", pv.Cylinder(center=(sx, sy, top_z_m + 0.001), direction=(0, 0, 1), radius=0.0046, height=0.002,
                                         resolution=6)))
        lever = _segment_cylinder((sx, sy, top_z_m + 0.002), (sx + 0.0025, sy, top_z_m + 0.012), 0.001, res=10)
        out.append(("steel", lever))
    if hw.tethered:
        te = tc.get("tether", {})
        eye_od = float(te.get("eye_od_ASSUMED", 20.0)) * mm
        eye_h = float(te.get("eye_h_ASSUMED", 25.0)) * mm
        wire = 0.005
        R = 0.5 * eye_od - 0.5 * wire
        zc = top_z_m + eye_h - 0.5 * eye_od
        out.append(("steel", pv.Cylinder(center=(0, 0, top_z_m + 0.0015), direction=(0, 0, 1), radius=0.0046,
                                         height=0.003, resolution=6)))          # M5 nut / collar on the plate
        out.append(("steel", _segment_cylinder((0, 0, top_z_m), (0, 0, zc - R), 0.0025, res=16)))
        ring = pv.ParametricTorus(ringradius=R, crosssectionradius=0.5 * wire, u_res=48, v_res=12, w_res=2)
        ring = ring.transform(homogeneous(rot_x(0.5 * math.pi), (0.0, 0.0, zc)), inplace=False)
        out.append(("steel", ring))
        boot_d = float(te.get("boot_d", 12.0)) * mm
        z_b0 = top_z_m + eye_h + 0.002
        z_b1 = float(tether_boot_top_b(hw, top_z_m)[2])
        n_turn = (z_b1 - z_b0) / 0.003
        a = np.linspace(0.0, 2.0 * math.pi * n_turn, int(n_turn * 16) + 2)
        hel = np.column_stack([0.5 * boot_d * np.cos(a), 0.5 * boot_d * np.sin(a), np.linspace(z_b0, z_b1, a.size)])
        out.append(("boot", _lines_polydata(hel).tube(radius=0.0007, n_sides=6)))
    return out


INTERIOR_PARTS = {          # CAD report placeholder -> material (seen through the windows; colours = typical parts)
    "edf_unit": "edf", "battery": "lipo", "buffer_pack": "lipo", "esc": "esc", "carrier": "pcb_g", "fcu": "pcb_g",
    "nav": "pcb_g", "com": "pcb_g", "gns": "pcb_g", "radio_log": "elec", "misc_electronics": "elec",
    "tether_cap_bank": "elec", "tether_bus_clamp": "alu", "tether_xt90": "xt90", "hybrid_diode_board": "pcb_g",
}   # (the rocket's CanSat / chute / spring sit in closed tubes: not drawn; a box would poke out of the nose's taper)


def hardware_report_path(hw: Hardware) -> str:
    """The CadQuery report of ``hw`` (``cad/exports/cadquery_report[_vtol[_<power>]].json``, next to the STL root)."""
    suffix = "" if not hw.vtol else ("_vtol" if hw.power == "battery" else f"_vtol_{hw.power}")
    return os.path.join(os.path.dirname(os.path.dirname(STL_DIR)), f"cadquery_report{suffix}.json")


def interior_items(hw: Hardware, warnings: list[str] | None = None) -> list[tuple[str, Any]]:
    """[(material key, mesh m, body frame)] of the non-printed parts inside the body, seen through the intake / bay
    windows: the CadQuery report's placeholder envelopes (``INTERIOR_PARTS``; boxes at their bounding boxes - the CAD's own
    'outer box' model; the EDF a housing cylinder + rotor face + spinner).  [] for a legacy log or without the report."""
    if hw.legacy:
        return []
    path = hardware_report_path(hw)
    try:
        with open(path, "r", encoding="utf-8") as f:
            parts = json.load(f)["parts"]
    except (OSError, KeyError, ValueError, TypeError):
        if warnings is not None:
            warnings.append(f"{_rel(path)} not readable: no interior parts drawn")
        return []
    import pyvista as pv
    out: list[tuple[str, Any]] = []
    for name, mat in INTERIOR_PARTS.items():
        p = parts.get(name)
        if not p or "bbox_mm" not in p:
            continue
        lo, hi = (np.asarray(v, dtype=float) * 1e-3 for v in p["bbox_mm"])
        if name == "edf_unit":
            # only the inlet end (top 30 mm): the envelope's radius holds for the housing, not for the tail below it
            # (it would stick out of the narrower duct section of the propulsion module)
            r = 0.5 * min(hi[0] - lo[0], hi[1] - lo[1])
            c = 0.5 * (lo + hi)
            out.append(("edf", pv.Cylinder(center=(c[0], c[1], hi[2] - 0.015), direction=(0, 0, 1), radius=r * 0.985,
                                           height=0.03, resolution=64, capping=False)))
            out.append(("rotor", pv.Disc(center=(c[0], c[1], hi[2] - 0.012), inner=0.012, outer=r * 0.97, normal=(0, 0, 1),
                                         c_res=64)))
            out.append(("spinner", pv.Cone(center=(c[0], c[1], hi[2] - 0.012 + 0.009), direction=(0, 0, 1), height=0.018,
                                           radius=0.014, resolution=32)))
            continue
        out.append((mat, _box(lo, hi)))
    return out


def tether_boot_top_b(hw: Hardware, top_z_m: float) -> np.ndarray:
    """Body-frame top of the spring boot (where the free tether starts): the axis at plate top + boot_len - the CAD's
    'boot top z' of its keep-out check (Layout.cap_tether); the drawn spring starts above the eye."""
    te = _top_cap_params().get("tether", {})
    return np.array([0.0, 0.0, top_z_m + float(te.get("boot_len", 100.0)) * 1e-3])


# ---- tether shape (pure) --------------------------------------------------------------------------------------------
def _catenary_x0(a: np.ndarray, D: float, h: float) -> np.ndarray:
    """Vertex abscissa of z = a cosh((x - x0)/a) + c through (0, 0) and (D, h)."""
    s = 2.0 * a * np.sinh(np.minimum(D / (2.0 * a), 300.0))
    return D / 2.0 - a * np.arcsinh(h / s)


def catenary_param(D: float, h: float, tension_n: float, w_n_per_m: float) -> float | None:
    """Catenary parameter a = H / w (m) of the cable from the vehicle end (0, 0) to the anchor (D, h) (D > 0 horizontal,
    h up) whose tension at the vehicle end is ``tension_n``: the taut branch (the larger a; T(a) grows ~linearly there);
    a target below the smallest feasible tension gives the a of that minimum.  None: draw a straight line (D tiny or the
    tension so large that a exceeds 1 km)."""
    if D < 1e-4 or w_n_per_m <= 0.0:
        return None
    a = np.geomspace(1e-3, 1e3, 400)
    x0 = _catenary_x0(a, D, h)
    with np.errstate(over="ignore", invalid="ignore"):
        T = w_n_per_m * a * np.cosh(np.minimum(np.abs(x0 / a), 300.0))
    T = np.where(np.isfinite(T), T, np.inf)
    i_min = int(np.argmin(T))
    tgt = float(tension_n)
    if tgt <= T[i_min]:
        return float(a[i_min])
    if tgt >= T[-1]:
        return None
    j = i_min + int(np.searchsorted(T[i_min:], tgt))        # T increasing on the taut branch
    lo, hi = a[j - 1], a[j]
    for _ in range(40):                                     # bisection on the taut branch
        mid = math.sqrt(lo * hi)
        xm = float(_catenary_x0(np.array([mid]), D, h)[0])
        Tm = w_n_per_m * mid * math.cosh(min(abs(xm / mid), 300.0))
        lo, hi = (mid, hi) if Tm < tgt else (lo, mid)
    return float(math.sqrt(lo * hi))


def resample_polyline(P: np.ndarray, n: int) -> np.ndarray:
    """``n`` points evenly spaced in arc length along the polyline P."""
    P = np.asarray(P, dtype=float)
    seg = np.linalg.norm(np.diff(P, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    if s[-1] <= 1e-12:
        return np.repeat(P[:1], n, axis=0)
    si = np.linspace(0.0, s[-1], n)
    return np.column_stack([np.interp(si, s, P[:, j]) for j in range(3)])


def tether_curve(p_vehicle: Sequence[float], anchor: Sequence[float], tension_n: float, w_n_per_m: float = TETHER_W_N_PER_M,
                 n: int = 48) -> np.ndarray:
    """(n, 3) points of the hanging tether from the vehicle end to the anchor, even in arc length: the catenary in the
    vertical plane through both points with ``tension_n`` at the vehicle end (``catenary_param``), straight when that
    is None."""
    P, A = np.asarray(p_vehicle, dtype=float), np.asarray(anchor, dtype=float)
    dxy = A[:2] - P[:2]
    D = float(np.hypot(*dxy))
    h = float(A[2] - P[2])
    a = catenary_param(D, h, max(float(tension_n), 0.0), w_n_per_m)
    if a is None:
        return resample_polyline(np.array([P, A]), n)
    e = np.array([dxy[0] / D, dxy[1] / D, 0.0])
    x = np.linspace(0.0, D, 400)
    x0 = float(_catenary_x0(np.array([a]), D, h)[0])
    z = a * (np.cosh((x - x0) / a) - math.cosh(x0 / a))
    pts = P[None, :] + x[:, None] * e[None, :] + z[:, None] * np.array([0.0, 0.0, 1.0])
    pts[-1] = A
    return resample_polyline(pts, n)


def polyline_length(P: np.ndarray) -> float:
    return float(np.sum(np.linalg.norm(np.diff(np.asarray(P, dtype=float), axis=0), axis=1)))


@dataclass
class FreeTether:
    """A disconnected tether hanging from the anchor: node positions (n_t, m, 3) at times ``t`` (s) - a deterministic
    chain simulation (position-based: Verlet step, gravity, linear air drag 0.8 /s, inextensible segments by alternating
    even / odd projection, ground contact with friction) from the drawn shape at the disconnect, node 0 pinned at the
    anchor, the free end starting with the vehicle's velocity (linear along the cable).  Stops early once at rest."""
    t: np.ndarray
    nodes: np.ndarray

    def at(self, tq: float) -> np.ndarray:
        if tq <= self.t[0]:
            return self.nodes[0]
        if tq >= self.t[-1]:
            return self.nodes[-1]
        i = int(np.searchsorted(self.t, tq, side="right")) - 1
        s = (tq - self.t[i]) / (self.t[i + 1] - self.t[i])
        return (1.0 - s) * self.nodes[i] + s * self.nodes[i + 1]


def simulate_free_tether(shape: np.ndarray, v_end: Sequence[float], t0: float, t1: float, ground_z: float,
                         hz: float = 300.0, iters: int = 10, store_hz: float = 100.0, r_m: float = 0.006,
                         max_s: float = 10.0) -> FreeTether:
    """``shape`` (m, 3) from the anchor (node 0) to the free end; simulate from t0 to min(t1, t0 + max_s)."""
    x = resample_polyline(shape, max(16, min(64, int(polyline_length(shape) / 0.06) + 1)))
    m = x.shape[0]
    L = polyline_length(x)
    seg = L / (m - 1)
    s = np.linspace(0.0, 1.0, m)[:, None]
    dt = 1.0 / hz
    v = s * np.asarray(v_end, dtype=float)[None, :]
    xp = x - v * dt
    A = x[0].copy()
    w = np.ones(m)
    w[0] = 0.0
    g = np.array([0.0, 0.0, -9.81]) * dt * dt
    idx = [np.arange(0, m - 1, 2), np.arange(1, m - 1, 2)]
    zg = ground_z + r_m
    n_steps = int(math.ceil(max(0.0, min(t1, t0 + max_s) - t0) * hz))
    every = max(1, int(round(hz / store_hz)))
    ts, xs = [t0], [x.copy()]
    rest = 0
    for k in range(1, n_steps + 1):
        vel = (x - xp) * (1.0 - 0.8 * dt)
        xp = x.copy()
        x = x + vel + g
        x[0] = A
        for _ in range(iters):
            for i in idx:
                d = x[i + 1] - x[i]
                ln = np.linalg.norm(d, axis=1)
                c = ((ln - seg) / np.maximum(ln, 1e-9))[:, None] * d
                ws = w[i] + w[i + 1]
                x[i] += (w[i] / ws)[:, None] * c
                x[i + 1] -= (w[i + 1] / ws)[:, None] * c
            low = x[:, 2] < zg
            x[low, 2] = zg
        on = x[:, 2] <= zg + 1e-6
        xp[on, :2] = x[on, :2] - 0.3 * (x[on, :2] - xp[on, :2])          # ground friction: 70 % of the slide per step
        if k % every == 0 or k == n_steps:
            ts.append(t0 + k * dt)
            xs.append(x.copy())
            rest = rest + 1 if float(np.max(np.linalg.norm(x - xp, axis=1))) * hz < 2e-3 else 0     # < 2 mm/s
            if rest >= 50:
                break
    return FreeTether(np.asarray(ts), np.asarray(xs))


def twisted_pair(P: np.ndarray, offset_m: float = 0.0045, pitch_m: float = 0.20) -> tuple[np.ndarray, np.ndarray]:
    """Two polylines twisted about P (the red / black 10 AWG conductors of the tether bundle)."""
    P = np.asarray(P, dtype=float)
    tng = np.gradient(P, axis=0)
    tng /= np.maximum(np.linalg.norm(tng, axis=1, keepdims=True), 1e-12)
    ref = np.where(np.abs(tng[:, 2:3]) < 0.9, np.array([[0.0, 0.0, 1.0]]), np.array([[1.0, 0.0, 0.0]]))
    b = np.cross(tng, ref)
    b /= np.maximum(np.linalg.norm(b, axis=1, keepdims=True), 1e-12)
    nn = np.cross(tng, b)
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(P, axis=0), axis=1))])
    ph = 2.0 * math.pi * s / pitch_m
    o = offset_m * (np.cos(ph)[:, None] * b + np.sin(ph)[:, None] * nn)
    return P + o, P - o


def boom_stand_geometry(anchor: Sequence[float], ground_z: float, u_dir: Sequence[float],
                        boom_len_m: float = BOOM_LEN_M) -> list[tuple[str, Any]]:
    """[(material key, mesh)] of the overhead boom stand, world m: a light-stand tripod (three legs, sandbags on the
    feet) at ``boom_len_m`` from the anchor along ``u_dir`` (horizontal), its column up to 0.10 m above the anchor, a
    boom over the anchor with a counterweight bag behind the column, a pulley block whose wheel bottom is the anchor,
    the cable along the boom and down the column to a supply box (+ an E-stop) on the ground.  Sizes ASSUMED (a 3.5-4 m
    light stand + boom, docs/유선전원_검토.md)."""
    import pyvista as pv
    A = np.asarray(anchor, dtype=float)
    u = np.array([u_dir[0], u_dir[1], 0.0], dtype=float)
    u /= max(float(np.linalg.norm(u)), 1e-9)
    v = np.array([-u[1], u[0], 0.0])
    R3 = np.column_stack([u, v, [0.0, 0.0, 1.0]])
    gz = float(ground_z)
    S = np.array([A[0], A[1], gz]) + boom_len_m * u                  # column foot
    z_top = max(A[2] + 0.10, gz + 1.0)
    out: list[tuple[str, Any]] = []
    hub = 0.62
    brk = min(2.0, 0.55 * (z_top - gz))                            # lower / upper column section
    out.append(("stand", _segment_cylinder(S + [0, 0, hub - 0.04], S + [0, 0, brk + 0.05], 0.0175, res=20)))
    out.append(("stand", _segment_cylinder(S + [0, 0, brk], [S[0], S[1], z_top + 0.03], 0.0140, res=20)))
    out.append(("stand", _segment_cylinder(S + [0, 0, brk - 0.02], S + [0, 0, brk + 0.06], 0.024, res=20)))   # lock
    out.append(("stand", _segment_cylinder(S + [0, 0, hub - 0.05], S + [0, 0, hub + 0.03], 0.027, res=20)))
    phi0 = math.atan2(-u[1], -u[0])                               # one leg under the boom
    for i in range(3):
        ph = phi0 + i * 2.0 * math.pi / 3.0
        d = np.array([math.cos(ph), math.sin(ph), 0.0])
        top = S + [0, 0, hub] + 0.025 * d
        foot = S + 0.62 * d + [0, 0, 0.012]
        out.append(("stand", _segment_cylinder(top, foot, 0.011, res=12)))
        mid = top + 0.55 * (foot - top)
        out.append(("stand", _segment_cylinder(mid, S + [0, 0, 0.33] + 0.02 * d, 0.0065, res=10)))   # brace
        out.append(("rubber", pv.Cylinder(center=tuple(foot - [0, 0, 0.006]), direction=(0, 0, 1), radius=0.016,
                                          height=0.012, resolution=16)))
        bag = pv.ParametricEllipsoid(0.17, 0.085, 0.055, u_res=24, v_res=16, w_res=2)
        bag = bag.transform(homogeneous(rot_z(ph + 0.5 * math.pi), S + 0.50 * d + [0, 0, 0.05]), inplace=False)
        out.append(("sandbag", bag))
    # boom: from behind the column (counterweight) to just past the anchor
    b0 = np.array([S[0], S[1], z_top]) + 0.55 * u
    b1 = np.array([A[0], A[1], z_top]) - 0.06 * u
    out.append(("boom", _segment_cylinder(b0, b1, 0.015, res=20)))
    out.append(("stand", _obox(np.array([S[0], S[1], z_top - 0.01]), R3, (0.035, 0.035, 0.04))))      # boom clamp
    cw = pv.ParametricEllipsoid(0.09, 0.07, 0.13, u_res=24, v_res=16, w_res=2)
    out.append(("sandbag", cw.translate(tuple(b0 - 0.04 * u - [0, 0, 0.16]), inplace=False)))
    out.append(("boom", _segment_cylinder(b0 - 0.04 * u, b0 - 0.04 * u - [0, 0, 0.04], 0.003, res=8)))
    # pulley block: side plates from the boom down to the wheel, wheel (plane = u, z) with its bottom at the anchor
    wr = 0.03
    wc = A + np.array([0.0, 0.0, wr])
    h_pl = 0.5 * (z_top - (wc[2] - 0.012))
    for sgn in (-1.0, 1.0):
        out.append(("pulley", _obox(wc + sgn * 0.009 * v + [0, 0, h_pl - 0.012], R3, (0.014, 0.0015, h_pl))))
    out.append(("pulley", pv.Cylinder(center=tuple(wc), direction=tuple(v), radius=wr, height=0.012, resolution=36)))
    out.append(("stand", _segment_cylinder(wc - 0.012 * v, wc + 0.012 * v, 0.004, res=10)))
    # the supply behind the stand (away from the vehicle and its jet: the legs sit at 60 / 180 / 300 deg from u)
    psu_c = S + 0.95 * u
    side = 0.035 * u
    path = [wc + [0, 0, wr], np.array([A[0], A[1], z_top - 0.024]) + 0.05 * u,
            np.array([S[0], S[1], z_top - 0.024]) - 0.06 * u, np.array([S[0], S[1], z_top - 0.08]) + side,
            S + side + [0, 0, 0.30], S + 1.4 * side + [0, 0, 0.012], psu_c - 0.18 * u + [0, 0, 0.012],
            psu_c - 0.14 * u + [0, 0, 0.032]]
    out.append(("cable_k", _lines_polydata(resample_polyline(np.array(path), 160)).tube(radius=0.006, n_sides=10)))
    # supply (RSP-1500-24 class, 280 x 127 x 64 mm) and an E-stop box beside it
    out.append(("psu", _obox(psu_c + [0, 0, 0.033], R3, (0.14, 0.0635, 0.032))))
    es = psu_c + 0.22 * v
    out.append(("estop", _obox(es + [0, 0, 0.0375], R3, (0.04, 0.04, 0.0375))))
    out.append(("button", pv.Cylinder(center=tuple(es + [0, 0, 0.086]), direction=(0, 0, 1), radius=0.02, height=0.022,
                                      resolution=24)))
    return out


BOOM_LENS_M = (BOOM_LEN_M, 2.0, 2.5)    # ASSUMED: a longer boom when no 1.5 m place is clear of the person / track


def choose_stand_dir(anchor_xy: Sequence[float], cam_az_deg: Sequence[float], person_xy: np.ndarray | None = None,
                     vehicle_xy: np.ndarray | None = None, boom_lens_m: Sequence[float] = BOOM_LENS_M,
                     prefer_deg: float = 40.0, step_deg: float = 15.0) -> tuple[np.ndarray, float]:
    """(unit horizontal direction (3,) from the anchor to the boom stand's column, boom length m).  For each boom length
    in turn, candidates every ``step_deg``; a candidate is valid when the column stays >= 1.4 m from every person
    position (tripod 0.62 m + a person + margin), the supply box (0.95 m further out) >= 0.8 m from them, and the column
    >= 0.9 m from the vehicle's ground track; among the valid ones the best mean (over the camera azimuths) alignment
    with 'behind the vehicle, ``prefer_deg`` to the right' wins (the column neither hides the vehicle nor stands in front
    of it).  No valid place at any length: the place farthest from the person / track."""
    A = np.asarray(anchor_xy, dtype=float)[:2]
    az = np.radians(np.asarray(cam_az_deg, dtype=float).reshape(-1))
    if az.size > 64:
        az = az[np.linspace(0, az.size - 1, 64).astype(int)]
    pref = az - math.radians(prefer_deg)            # behind (look direction) turned toward the screen right (-angle)
    best_any = None
    for blen in boom_lens_m:
        best = None
        for ang in np.radians(np.arange(0.0, 360.0, step_deg)):
            u = np.array([math.cos(ang), math.sin(ang)])
            S = A + blen * u
            P = S + 0.95 * u
            d_p = d_s = d_v = math.inf
            if person_xy is not None and len(person_xy):
                q = np.asarray(person_xy, dtype=float)[:, :2]
                d_p = float(np.min(np.hypot(*(q - S).T)))
                d_s = float(np.min(np.hypot(*(q - P).T)))
            if vehicle_xy is not None and len(vehicle_xy):
                q = np.asarray(vehicle_xy, dtype=float)[:, :2]
                d_v = float(np.min(np.hypot(*(q - S).T)))
            score = float(np.mean(np.cos(ang - pref)))
            margin = min(d_p - 1.4, d_s - 0.8, d_v - 0.9)
            if margin >= 0.0 and (best is None or score > best[0] + 1e-9):
                best = (score, u)
            if best_any is None or margin > best_any[0] + 1e-9:
                best_any = (margin, u, blen)
        if best is not None:
            return np.array([best[1][0], best[1][1], 0.0]), float(blen)
    u = best_any[1]
    return np.array([u[0], u[1], 0.0]), float(best_any[2])


def _obox(center: Sequence[float], R: np.ndarray, half: Sequence[float]) -> Any:
    """An oriented box: half extents ``half`` along the columns of R, centred at ``center``."""
    hx, hy, hz = half
    return _box((-hx, -hy, -hz), (hx, hy, hz)).transform(homogeneous(np.asarray(R, dtype=float), center), inplace=False)


def hold_marker_mesh(ring_r: float = HOLD_RING_M, cross_m: float = HOLD_CROSS_M, width: float = 0.022,
                     n: int = 96) -> Any:
    """The hold-point mark at the origin (z = 0, translated per frame): a thin annulus of radius ``ring_r`` and a
    cross-hair of 8 ticks along +-x / +-y (inside the ring from 0.06 m, outside it to ``cross_m``; nothing overlaps,
    so a translucent mark has no double-blended spots), all ``width`` wide, as flat quads."""
    import pyvista as pv
    a = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    r0, r1 = ring_r - 0.5 * width, ring_r + 0.5 * width
    pts = [np.column_stack([r0 * np.cos(a), r0 * np.sin(a), np.zeros(n)]),
           np.column_stack([r1 * np.cos(a), r1 * np.sin(a), np.zeros(n)])]
    faces: list[int] = []
    for i in range(n):
        j = (i + 1) % n
        faces += [4, i, j, n + j, n + i]
    base = 2 * n
    h = 0.5 * width
    for ang in (0.0, 0.5 * math.pi, math.pi, 1.5 * math.pi):
        u = np.array([math.cos(ang), math.sin(ang), 0.0])
        v = np.array([-u[1], u[0], 0.0])
        for (s0, s1) in ((0.06, r0 - 0.03), (r1 + 0.03, cross_m)):
            if s1 <= s0:
                continue
            pts.append(np.array([u * s0 - v * h, u * s1 - v * h, u * s1 + v * h, u * s0 + v * h]))
            faces += [4, base, base + 1, base + 2, base + 3]
            base += 4
    return pv.PolyData(np.vstack(pts), faces=np.array(faces))


def _plume_mesh(n: int = 40) -> Any:
    """Open truncated cone (topology fixed, points rewritten per frame)."""
    import pyvista as pv
    a = np.linspace(0.0, 2.0 * math.pi, n, endpoint=False)
    pts = np.zeros((2 * n, 3))
    pts[:n, 0], pts[:n, 1] = np.cos(a), np.sin(a)
    pts[n:, 0], pts[n:, 1], pts[n:, 2] = np.cos(a), np.sin(a), -1.0
    faces = []
    for i in range(n):
        j = (i + 1) % n
        faces += [4, i, j, n + j, n + i]
    return pv.PolyData(pts, np.array(faces)), a


GROUND_TILE_M = 10.0


def ground_texture(px: int = 1024) -> Any:
    """A 10 m x 10 m ground tile: 1 m lines, 5 m lines, the 10 m border (shared with the neighbour tile).
    Mipmapped + anisotropic filtering, so the far field fades to the base colour instead of aliasing."""
    import pyvista as pv
    img = np.empty((px, px, 3), dtype=np.uint8)
    img[:] = hex_rgb(GROUND)
    rng = np.random.default_rng(7)                        # fixed seed: identical texture every run
    img = np.clip(img.astype(int) + rng.integers(-5, 6, size=(px, px, 1)), 0, 255).astype(np.uint8)
    per_m = px / GROUND_TILE_M
    for m in range(0, int(GROUND_TILE_M) + 1):
        c = int(round(m * per_m))
        if m % 10 == 0:
            w, col = 5, hex_rgb(GRID_MAJOR)
        elif m % 5 == 0:
            w, col = 4, hex_rgb(GRID_MAJOR)
        else:
            w, col = 2, hex_rgb(GRID_MINOR)
        lo, hi = max(0, c - w // 2), min(px, c + (w - w // 2))
        img[:, lo:hi] = col
        img[lo:hi, :] = col
    tex = pv.Texture(img)
    tex.repeat = True
    tex.mipmap = True
    tex.interpolate = True
    if hasattr(tex, "SetMaximumAnisotropicFiltering"):
        tex.SetMaximumAnisotropicFiltering(16)
    return tex


def _lines_polydata(pts: np.ndarray, dashed: bool = False) -> Any:
    import pyvista as pv
    pts = np.asarray(pts, dtype=float)
    if pts.shape[0] < 2:
        pts = np.vstack([pts, pts]) if pts.shape[0] == 1 else np.zeros((2, 3))
    if dashed:
        m = pts.shape[0] // 2 * 2
        segs = np.arange(m).reshape(-1, 2)
        cells = np.hstack([np.full((segs.shape[0], 1), 2), segs]).ravel()
        return pv.PolyData(pts[:m], lines=cells)
    cells = np.concatenate([[pts.shape[0]], np.arange(pts.shape[0])])
    return pv.PolyData(pts, lines=cells)


def _segment_cylinder(p0: Sequence[float], p1: Sequence[float], r: float, res: int = 16) -> Any:
    import pyvista as pv
    p0, p1 = np.asarray(p0, dtype=float), np.asarray(p1, dtype=float)
    v = p1 - p0
    return pv.Cylinder(center=tuple(0.5 * (p0 + p1)), direction=tuple(v), radius=r, height=float(np.linalg.norm(v)),
                       resolution=res)


def figure_meshes() -> dict[str, Any]:
    """The 1.7 m reference person (figure frame: x toward the vehicle, z up, origin between the feet), two poses
    sharing the head: 'idle' (arms down) and 'push' (both hands at FIGURE_HANDS, where the pole is held).  A simple
    scale reference, not an anthropometric model."""
    import pyvista as pv
    H = FIGURE_HEIGHT_M
    head_r = 0.105
    head_c = np.array([0.0, 0.0, H - head_r])

    def torso() -> Any:
        tor = pv.Cylinder(center=(0.0, 0.0, 1.13), direction=(0, 0, 1), radius=0.17, height=0.58, resolution=24)
        tor.points[:, 0] *= 0.62
        neck = _segment_cylinder((0, 0, 1.41), (0, 0, head_c[2] - 0.08), 0.05)
        return tor + neck

    sh = 0.21
    idle = torso()
    for s in (-1.0, 1.0):
        idle = idle + _segment_cylinder((0.0, s * 0.10, 0.0), (0.0, s * 0.10, 0.86), 0.065)
        idle = idle + _segment_cylinder((0.0, s * sh, 1.39), (0.02, s * 0.25, 0.83), 0.045)
        idle = idle + pv.Sphere(radius=0.05, center=(0.02, s * 0.25, 0.80))
    push = torso()
    hx, _, hz = FIGURE_HANDS
    for s, xf in ((-1.0, 0.13), (1.0, -0.15)):            # a stride: one foot forward
        push = push + _segment_cylinder((xf, s * 0.10, 0.0), (0.0, s * 0.10, 0.86), 0.065)
        push = push + _segment_cylinder((0.0, s * sh, 1.39), (hx, s * 0.07, hz), 0.045)
        push = push + pv.Sphere(radius=0.05, center=(hx, s * 0.07, hz))
    head = pv.Sphere(radius=head_r, center=tuple(head_c), theta_resolution=24, phi_resolution=24)
    return {"idle": idle, "push": push, "head": head}


@dataclass
class FigureTrack:
    """Where the reference person stands per frame (feet, world), which way it faces (yaw, rad, figure x axis),
    whether it is pushing, and the hand point (the pole's outer end)."""

    feet: np.ndarray
    yaw: np.ndarray
    pushing: np.ndarray
    hands: np.ndarray


def _yaw_to(src_xy: np.ndarray, dst_xy: np.ndarray, fallback: float = 0.0) -> float:
    v = np.asarray(dst_xy, dtype=float) - np.asarray(src_xy, dtype=float)
    return math.atan2(float(v[1]), float(v[0])) if float(np.hypot(*v)) > 1e-6 else fallback


def figure_track(t: np.ndarray, veh_pos: np.ndarray, push_active: np.ndarray | None, push_force: np.ndarray | None,
                 contact: np.ndarray | None, ground_z: float, idle_xy: Sequence[float], walk_s: float = 2.0) -> FigureTrack:
    """Pure.  While a push is active the person stands behind the contact point, on the far side of the force:
    feet = contact_xy - u (POLE_REACH_M + FIGURE_HANDS.x), u = the horizontal force direction (held from the last
    pushing frame when the force is nearly vertical).  Before the first push it waits at the first push position,
    between pushes it holds the last one and walks (straight, linear in time) to the next during the ``walk_s``
    before that push starts; with no push at all it stands at ``idle_xy``.  It faces the vehicle unless pushing
    (then it faces along the force)."""
    n = t.shape[0]
    feet = np.zeros((n, 3))
    feet[:, 2] = ground_z
    yaw = np.zeros(n)
    act = np.zeros(n, dtype=bool) if push_active is None else np.asarray(push_active, dtype=bool)
    reach = POLE_REACH_M + float(FIGURE_HANDS[0])
    if not np.any(act):
        feet[:, :2] = np.asarray(idle_xy, dtype=float)
        for k in range(n):
            yaw[k] = _yaw_to(feet[k, :2], veh_pos[k, :2])
    else:
        u_last = None
        anchor = np.full((n, 2), np.nan)
        for k in np.flatnonzero(act):
            fh = np.asarray(push_force[k, :2], dtype=float)
            fm = float(np.linalg.norm(push_force[k]))
            if fm > 1e-9 and float(np.hypot(*fh)) > 0.2 * fm:
                u_last = fh / float(np.hypot(*fh))
            if u_last is None:
                u_last = np.asarray(contact[k, :2], dtype=float) - np.asarray(idle_xy, dtype=float)
                nn = float(np.hypot(*u_last))
                u_last = u_last / nn if nn > 1e-9 else np.array([1.0, 0.0])
            anchor[k] = np.asarray(contact[k, :2], dtype=float) - u_last * reach
            yaw[k] = math.atan2(float(u_last[1]), float(u_last[0]))
        idx = np.flatnonzero(act)
        starts = idx[np.concatenate([[True], np.diff(idx) > 1])]
        ends = idx[np.concatenate([np.diff(idx) > 1, [True]])]
        for k in range(n):
            if act[k]:
                feet[k, :2] = anchor[k]
                continue
            prev_end = ends[ends < k]
            next_start = starts[starts > k]
            if prev_end.size == 0:
                feet[k, :2] = anchor[next_start[0]]
            elif next_start.size == 0:
                feet[k, :2] = anchor[prev_end[-1]]
            else:
                a_pos, b_pos = anchor[prev_end[-1]], anchor[next_start[0]]
                tb = float(t[next_start[0]])
                ta = max(float(t[prev_end[-1]]), tb - walk_s)
                s = 0.0 if t[k] <= ta else min(1.0, (float(t[k]) - ta) / max(tb - ta, 1e-9))
                feet[k, :2] = (1.0 - s) * a_pos + s * b_pos
            yaw[k] = _yaw_to(feet[k, :2], veh_pos[k, :2], yaw[k - 1] if k else 0.0)
    hands = np.zeros((n, 3))
    for k in range(n):
        c, s_ = math.cos(yaw[k]), math.sin(yaw[k])
        hands[k] = feet[k] + np.array([c * FIGURE_HANDS[0], s_ * FIGURE_HANDS[0], FIGURE_HANDS[2]])
    return FigureTrack(feet=feet, yaw=yaw, pushing=act.copy(), hands=hands)


def figure_points(tr_: FigureTrack) -> np.ndarray:
    """(N, 4, 3) feet / head on both shoulders' sides: what the camera must frame to show the person."""
    n = tr_.feet.shape[0]
    side = np.column_stack([-np.sin(tr_.yaw), np.cos(tr_.yaw), np.zeros(n)]) * 0.25
    up = np.array([0.0, 0.0, FIGURE_HEIGHT_M])
    return np.stack([tr_.feet + side, tr_.feet - side, tr_.feet + up + side, tr_.feet + up - side], axis=1)


def figure_occludes(fig_pts: np.ndarray, veh_pts: np.ndarray, cam: CamView, width: int, height: int,
                    veh_radius_m: float = 0.07) -> bool:
    """Pure: would the person (its screen box from ``fig_pts``) cover the vehicle (its screen box from ``veh_pts``,
    grown by the body radius)?  Only when the person is nearer the camera than the vehicle."""
    pf, zf = project(fig_pts, cam, width, height)
    pv_, zv = project(veh_pts, cam, width, height)
    if float(zf.min()) <= 0.0 or float(zv.min()) <= 0.0:
        return False
    if float(np.mean(zf)) >= float(np.mean(zv)):
        return False
    th = math.tan(math.radians(cam.view_angle_deg) / 2.0)
    grow = veh_radius_m / (float(np.mean(zv)) * th) * 0.5 * height
    fx0, fy0 = pf.min(axis=0)
    fx1, fy1 = pf.max(axis=0)
    vx0, vy0 = pv_.min(axis=0) - grow
    vx1, vy1 = pv_.max(axis=0) + grow
    return bool(fx0 < vx1 and vx0 < fx1 and fy0 < vy1 and vy0 < fy1)


# --------------------------------------------------------------------------- #
# the realistic person's behaviour over time (pure): push blend in / out, walking between push positions
# --------------------------------------------------------------------------- #
PERSON_BLEND_S = 0.3                    # pose transitions: arms up to the pole before a push, down after it
PERSON_WALK_STRIDE_M = 1.15             # one gait cycle (two steps), as tools/human_figure.WALK_STRIDE_M


@dataclass
class PersonTimeline:
    """Push intervals of the log (t0, t1, first / last active sample) and, on the video grid, the walking weight,
    the distance walked (gait phase) and the walking direction (world, unit) of the placement track."""

    pushes: list[tuple[float, float, int, int]]
    t: np.ndarray
    walk_w: np.ndarray
    dist: np.ndarray
    vdir: np.ndarray


def person_timeline(log: "SimLog", t_grid: np.ndarray, fig_grid: FigureTrack | None) -> PersonTimeline:
    """Pure.  Walking = the feet of ``figure_track`` moving faster than 0.15 m/s (full weight from 0.6 m/s), smoothed
    over PERSON_BLEND_S; while pushing the caller suppresses it (the feet then follow the contact point)."""
    pushes: list[tuple[float, float, int, int]] = []
    if log.push_active is not None and np.any(log.push_active):
        idx = np.flatnonzero(log.push_active)
        st = idx[np.concatenate([[True], np.diff(idx) > 1])]
        en = idx[np.concatenate([np.diff(idx) > 1, [True]])]
        pushes = [(float(log.t[a]), float(log.t[b]), int(a), int(b)) for a, b in zip(st, en)]
    t = np.asarray(t_grid, dtype=float)
    n = t.shape[0]
    if fig_grid is None or n < 3:
        return PersonTimeline(pushes, t, np.zeros(n), np.zeros(n), np.tile([1.0, 0.0], (n, 1)))
    xy = fig_grid.feet[:, :2]
    v = np.gradient(xy, t, axis=0)
    sp = np.hypot(v[:, 0], v[:, 1])
    raw = np.clip((sp - 0.15) / 0.45, 0.0, 1.0)
    raw = raw * raw * (3.0 - 2.0 * raw)
    dt = float(np.median(np.diff(t))) if n > 1 else 1.0
    half = max(1, int(round(0.5 * PERSON_BLEND_S / max(dt, 1e-6))))
    ker = np.ones(2 * half + 1) / (2 * half + 1)
    walk_w = np.convolve(np.pad(raw, half, mode="edge"), ker, mode="valid")
    dist = np.concatenate([[0.0], np.cumsum(np.hypot(*np.diff(xy, axis=0).T))])
    vdir = np.tile([1.0, 0.0], (n, 1))
    last = np.array([1.0, 0.0])
    for k in range(n):
        if sp[k] > 0.05:
            last = v[k] / sp[k]
        vdir[k] = last
    return PersonTimeline(pushes, t, walk_w, dist, vdir)


def person_push_weight(pushes: Sequence[tuple[float, float, int, int]], t: float,
                       blend_s: float = PERSON_BLEND_S) -> tuple[float, int | None]:
    """Pure.  (weight 0..1, the log sample whose pole geometry to use when the frame itself is not pushing) at time t:
    1 during a push, smoothstep up over the ``blend_s`` before it and down over the ``blend_s`` after it."""
    best, ref = 0.0, None
    for t0, t1, j0, j1 in pushes:
        if t0 <= t <= t1:
            return 1.0, (j0 if t - t0 < t1 - t else j1)
        if t0 - blend_s < t < t0:
            x = (t - (t0 - blend_s)) / blend_s
            w, j = x * x * (3.0 - 2.0 * x), j0
        elif t1 < t < t1 + blend_s:
            x = 1.0 - (t - t1) / blend_s
            w, j = x * x * (3.0 - 2.0 * x), j1
        else:
            continue
        if w > best:
            best, ref = w, j
    return best, ref


def visual_contact(contact: np.ndarray, force: np.ndarray, pos: np.ndarray, R: np.ndarray, body_r: float) -> np.ndarray:
    """Where the pad is drawn: the logged contact point, moved out to the body surface (against the force,
    perpendicular to the body axis) when it lies inside the body (e.g. a contact given on the axis)."""
    c = np.asarray(contact, dtype=float)
    ax_ = R[:, 2]
    rel = c - pos
    radial = rel - ax_ * float(rel @ ax_)
    if float(np.linalg.norm(radial)) >= 0.9 * body_r:
        return c
    f = np.asarray(force, dtype=float)
    fp = f - ax_ * float(f @ ax_)
    nf = float(np.linalg.norm(fp))
    if nf < 1e-9:
        return c
    return c - radial - fp / nf * body_r


def _align_z(v: np.ndarray) -> np.ndarray:
    """Rotation matrix taking +z to the unit vector v."""
    z = np.array([0.0, 0.0, 1.0])
    v = np.asarray(v, dtype=float) / max(float(np.linalg.norm(v)), 1e-12)
    c = float(z @ v)
    if c > 1.0 - 1e-12:
        return np.eye(3)
    if c < -1.0 + 1e-12:
        return np.diag([1.0, -1.0, -1.0])
    a = np.cross(z, v)
    s = float(np.linalg.norm(a))
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]]) / s
    ang = math.atan2(s, c)
    return np.eye(3) + math.sin(ang) * K + (1.0 - math.cos(ang)) * (K @ K)


def vtk_text_prop(prop: Any, lang: str, size: int, bold: bool = False, color: str = "#111111",
                  bg: str | None = None, bg_opacity: float = 0.0) -> None:
    """Configure a vtkTextProperty: Korean through VTK_FONT_FILE (Malgun Gothic) for lang 'ko', else Arial."""
    import vtk
    path = korean_font_path(bold) if lang == "ko" else None
    if path:
        prop.SetFontFamily(vtk.VTK_FONT_FILE)
        prop.SetFontFile(path)
    else:
        prop.SetFontFamilyToArial()
        prop.SetBold(bool(bold))
    prop.SetFontSize(int(size))
    r, g, b = hex_rgb(color)
    prop.SetColor(r / 255.0, g / 255.0, b / 255.0)
    if bg is not None:
        r, g, b = hex_rgb(bg)
        prop.SetBackgroundColor(r / 255.0, g / 255.0, b / 255.0)
        prop.SetBackgroundOpacity(float(bg_opacity))
    prop.SetJustificationToCentered()
    prop.SetVerticalJustificationToBottom()


def _arrow_mesh(start: np.ndarray, vec: np.ndarray, shaft: float) -> Any:
    """Arrow from ``start`` along ``vec`` (its length) with an absolute shaft radius; the head is
    6 shaft radii long and 2.6 wide, at most half the arrow."""
    import pyvista as pv
    L = float(np.linalg.norm(vec))
    if L < 1e-6:
        return None
    shaft = min(shaft, L / 14.0)                    # short arrows stay arrow-shaped
    return pv.Arrow(start=tuple(start), direction=tuple(vec / L), scale=L, tip_length=min(0.5, 6.0 * shaft / L),
                    tip_radius=2.6 * shaft / L, shaft_radius=shaft / L, tip_resolution=24, shaft_resolution=16)


# v0.9 non-printed items: (realistic linear albedo, metallic, roughness, schematic colour)
HW_MAT: dict[str, tuple[str, float, float, str]] = {
    "pcb": ("#0f2f73", 0.0, 0.45, "#1c3f8f"), "patch": ("#b5a983", 0.0, 0.5, "#cbbf9f"),
    "copper": ("#b06a32", 1.0, 0.35, "#c47a3c"), "steel": ("#c0c4c9", 1.0, 0.30, "#b9bec5"),
    "boot": ("#c0c4c9", 1.0, 0.35, "#9aa0a8"), "stand": ("#18191b", 0.5, 0.50, "#30343a"),
    "boom": ("#c4c8cd", 0.9, 0.35, "#a7adb5"), "rubber": ("#121214", 0.0, 0.90, "#222226"),
    "sandbag": ("#3a3420", 0.0, 0.95, "#8a7f5c"), "pulley": ("#cfd2d6", 1.0, 0.30, "#b9bec5"),
    "cable_k": ("#0e0e10", 0.0, 0.55, "#202024"), "cable_r": ("#8e1515", 0.0, 0.50, "#c0392b"),
    "bungee": ("#2a2a2e", 0.0, 0.80, "#4a4a52"), "psu": ("#a9aeb4", 0.8, 0.45, "#a9aeb4"),
    "estop": ("#d8b40c", 0.0, 0.60, "#e8c21a"), "button": ("#a50f0f", 0.0, 0.40, "#c62828"),
    "edf": ("#2a2c30", 0.6, 0.45, "#6f747b"), "rotor": ("#101113", 0.0, 0.6, "#26282c"),
    "spinner": ("#c8ccd1", 1.0, 0.25, "#c8ccd1"), "lipo": ("#34363b", 0.0, 0.6, "#c86f1c"),
    "esc": ("#16467f", 0.0, 0.5, "#1d5fa8"), "pcb_g": ("#0b3f24", 0.0, 0.4, "#0f5a33"),
    "elec": ("#18191c", 0.0, 0.55, "#6a4c93"), "alu": ("#a8adb3", 0.8, 0.4, "#a8adb3"),
    "xt90": ("#c9a200", 0.0, 0.5, "#e2b400"), "chute": ("#b8441a", 0.0, 0.9, "#d9531e"),
}


def _add_hw_mesh(pl: Any, mesh: Any, key: str, real: bool, **kw: Any) -> Any:
    col, met, rough, sch = HW_MAT[key]
    if real:
        return pl.add_mesh(mesh, color=col, pbr=True, metallic=met, roughness=rough, smooth_shading=True, **kw)
    return pl.add_mesh(mesh, color=sch, smooth_shading=True, specular=0.3, specular_power=20, **kw)


class TetherView:
    """The tether of a tether / hybrid log in a ``Scene``: the static boom stand (``boom_stand_geometry``, placed
    ``BOOM_LEN_M`` from the anchor, 40 deg right of straight behind the vehicle as seen from the first camera azimuth, with
    its own projected shadow) and per frame the tether - connected: the twisted red / black pair from the eye bolt up the
    boot and along ``tether_curve`` (the logged tension at the boot top) to the anchor, the bungee beside its first
    BUNGEE_LEN_M + T / BUNGEE_K_N_PER_M; disconnected: the ``FreeTether`` chain of that interval."""

    def __init__(self, scene: "Scene", log: SimLog, cam_azimuth_deg: float, warnings: list[str]):
        import pyvista as pv
        self.scene, self.log = scene, log
        pl, real = scene.pl, scene.real
        self.assumed: list[str] = []
        self.assumed_ko: list[str] = []
        te = _top_cap_params().get("tether", {})
        top = scene.top_z_m
        eye_h = float(te.get("eye_h_ASSUMED", 25.0)) * 1e-3
        attach_cad = np.array([0.0, 0.0, top + eye_h - 0.005])           # inside the top of the eye ring
        if log.tether_attach_b is not None:
            self.attach_b = np.asarray(log.tether_attach_b, dtype=float)
            off = float(np.linalg.norm(self.attach_b - attach_cad))
            if off > 0.03:
                warnings.append(f"tether_attach_body_m is {off * 1e3:.0f} mm from the CAD eye bolt "
                                f"({attach_cad[2] * 1e3:.0f} mm above the exit plane): drawn where the log says")
        else:
            self.attach_b = attach_cad
            self.assumed.append("attach point = the CAD eye bolt")
            self.assumed_ko.append("고정점 = CAD 아이볼트")
        boot_top_cad = tether_boot_top_b(log.hw, top)
        self.boot_top_b = self.attach_b + (boot_top_cad - attach_cad)
        if log.tether_anchor is not None:
            self.anchor = np.asarray(log.tether_anchor, dtype=float)
        else:
            self.anchor = np.asarray(log.pos[0], dtype=float) + np.array([0.0, 0.0, TETHER_DEFAULT_ANCHOR_H_M])
            self.assumed.append(f"anchor {TETHER_DEFAULT_ANCHOR_H_M:g} m above the first position")
            self.assumed_ko.append(f"도르래 = 시작 위치 위 {TETHER_DEFAULT_ANCHOR_H_M:g} m")
        if log.tether_tension is None:
            self.assumed.append(f"tension {TETHER_DEFAULT_TENSION_N:g} N")
            self.assumed_ko.append(f"장력 {TETHER_DEFAULT_TENSION_N:g} N")
        self.w = float(log.meta.get("tether_w_n_per_m", TETHER_W_N_PER_M))
        if self.assumed:
            warnings.append("tether keys missing -> drawn with ASSUMED " + ", ".join(self.assumed))
        az_all = scene.cam_az_all if scene.cam_az_all is not None else [float(cam_azimuth_deg)]
        self.u, self.boom_len = choose_stand_dir(self.anchor[:2], az_all, scene.person_xy, scene.grid.pos[:, :2])
        gz = log.ground_z
        self.static = boom_stand_geometry(self.anchor, gz, self.u, self.boom_len)
        self.static_actors = [_add_hw_mesh(pl, m, key, real) for key, m in self.static]
        if real:
            RS, wi = scene.RS, scene.world.info
            tris = np.concatenate([RS.mesh_triangles(m, 400) for _, m in self.static], axis=0)
            self.shadow = RS.ShadowDecal(pl, gz + RS.DECAL_Z["shadow"], strength=wi.shadow_strength, n=384)
            self.shadow.update(tris, scene.light_dir, wi.shadow_blur_base_m + 0.5 * wi.shadow_blur_per_m * (self.anchor[2] - gz))
        self.mesh_r, self.mesh_k, self.mesh_b = (pv.Sphere(radius=1e-5) for _ in range(3))
        self.act_r = _add_hw_mesh(pl, self.mesh_r, "cable_r", real)
        self.act_k = _add_hw_mesh(pl, self.mesh_k, "cable_k", real)
        self.act_b = _add_hw_mesh(pl, self.mesh_b, "bungee", real)
        self.free: list[tuple[float, float, FreeTether]] = []
        self._precompute_free()

    # -- shapes ------------------------------------------------------------------------------------------------------
    def tension_at(self, T: float | None) -> float:
        return TETHER_DEFAULT_TENSION_N if T is None or not np.isfinite(T) else max(float(T), 0.0)

    def connected_shape(self, M: np.ndarray, T: float) -> tuple[np.ndarray, int]:
        """(points from the eye to the anchor, index of the boot top) for body pose M and tension T."""
        P = M[:3, :3] @ self.attach_b + M[:3, 3]
        B = M[:3, :3] @ self.boot_top_b + M[:3, 3]
        boot = np.linspace(P, B, 6)
        cur = tether_curve(B, self.anchor, T, self.w, 56)
        return np.vstack([boot, cur[1:]]), 5

    def _precompute_free(self) -> None:
        log = self.log
        c = log.tether_connected
        if c is None or bool(np.all(c)):
            return
        n = log.n
        i = 0
        while i < n:
            if c[i]:
                i += 1
                continue
            j = i
            while j + 1 < n and not c[j + 1]:
                j += 1
            if i == 0:
                P0 = log.pos[0] + quat_to_matrix(log.q[0]) @ self.attach_b
                shape = np.array([self.anchor, P0])
                v_end, t0 = np.zeros(3), float(log.t[0])
            else:
                M = body_matrix(log.pos[i - 1], log.q[i - 1])
                Tt = None if log.tether_tension is None else float(log.tether_tension[i - 1])
                pts, _ = self.connected_shape(M, self.tension_at(Tt))
                shape, v_end, t0 = pts[::-1], np.asarray(log.vel[i - 1], dtype=float), float(log.t[i - 1])
            t1 = float(log.t[j + 1]) if j + 1 < n else float(log.t[-1])
            self.free.append((t0, t1, simulate_free_tether(shape, v_end, t0, t1, log.ground_z)))
            i = j + 1

    def shape_at(self, k: int, M: np.ndarray) -> tuple[np.ndarray, bool, float]:
        """(polyline from the vehicle / free end to the anchor, connected, tension) of frame k."""
        fr = self.scene.fr
        t = float(fr.t[k])
        con = True if fr.tether_connected is None else bool(fr.tether_connected[k])
        T = self.tension_at(None if fr.tether_tension is None else float(fr.tether_tension[k]))
        if con:
            pts, _ = self.connected_shape(M, T)
            return pts, True, T
        for t0, t1, ft in self.free:
            if t0 - 1e-9 <= t <= t1 + 1e-9:
                return ft.at(t)[::-1], False, 0.0
        ft = self.free[-1][2] if self.free else None
        if ft is not None:
            return ft.at(t)[::-1], False, 0.0
        pts, _ = self.connected_shape(M, T)
        return pts, False, 0.0

    def update(self, k: int, M: np.ndarray) -> None:
        pts, con, T = self.shape_at(k, M)
        a, b = twisted_pair(pts)
        self.mesh_r.copy_from(_lines_polydata(a).tube(radius=0.0029, n_sides=8))
        self.mesh_k.copy_from(_lines_polydata(b).tube(radius=0.0029, n_sides=8))
        # bungee: beside the bundle from its vehicle end for BUNGEE_LEN + T / k of arc length
        lb = BUNGEE_LEN_M + T / BUNGEE_K_N_PER_M
        s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(pts, axis=0), axis=1))])
        m = max(2, int(np.searchsorted(s, lb)) + 1)
        seg = resample_polyline(pts[:m], 24)
        tng = np.gradient(seg, axis=0)
        side = np.cross(tng, np.array([0.0, 0.0, 1.0]))
        nrm = np.linalg.norm(side, axis=1, keepdims=True)
        side = np.where(nrm > 1e-9, side / np.maximum(nrm, 1e-12), np.array([[1.0, 0.0, 0.0]]))
        bow = np.sin(np.linspace(0.0, math.pi, seg.shape[0]))[:, None]
        self.mesh_b.copy_from(_lines_polydata(seg + side * (0.010 + 0.012 * bow)).tube(radius=0.0045, n_sides=8))

    def summary(self) -> dict[str, Any]:
        log = self.log
        out: dict[str, Any] = {"anchor_m": [round(float(x), 4) for x in self.anchor],
                               "attach_body_m": [round(float(x), 4) for x in self.attach_b], "w_n_per_m": self.w,
                               "stand_dir": [round(float(x), 4) for x in self.u[:2]], "boom_len_m": self.boom_len,
                               "assumed": self.assumed}
        if log.tether_tension is not None and np.any(np.isfinite(log.tether_tension)):
            out["tension_max_n"] = round(float(np.nanmax(log.tether_tension)), 3)
        out["disconnects"] = [{"t0": round(t0, 3), "t1": round(t1, 3), "sim_s": round(float(ft.t[-1] - ft.t[0]), 3)}
                              for t0, t1, ft in self.free]
        return out


class Scene:
    """The main chase view and the body-fixed vane close-up, both off-screen PyVista plotters."""

    def __init__(self, log: SimLog, fr: Frames, grid: Frames, geom: VehicleGeom, main_size: tuple[int, int],
                 inset_px: int, cam_azimuth_deg: float, warnings: list[str], aa: str = "msaa", lang: str = "en",
                 with_figure: bool = False, with_push: bool = False, style: str = "schematic", hud: str = "full",
                 profile: str = "field", sun_az_deg: float | None = None, asset_dir: str | None = ASSET_DIR,
                 with_gps_pod: bool = False, person_xy: np.ndarray | None = None,
                 cam_az_all: Sequence[float] | None = None):
        import pyvista as pv
        pv.OFF_SCREEN = True
        self.person_xy, self.cam_az_all = person_xy, cam_az_all        # v0.9: where the boom stand must not stand
        if style not in STYLES:
            raise ValueError("style must be " + " | ".join(STYLES))
        self.lang = lang
        self.style, self.hud, self.profile = style, hud, profile
        self.real = style == "realistic"
        self.log, self.fr, self.grid, self.geom = log, fr, grid, geom
        self.pl = pv.Plotter(off_screen=True, window_size=main_size, lighting=None if self.real else "light kit")
        pl = self.pl
        if not self.real:
            pl.set_background(SKY_BOTTOM, top=SKY_TOP)
        if aa == "msaa":
            pl.enable_anti_aliasing("msaa", multi_samples=4 if not self.real else 8)
        elif aa == "fxaa":
            pl.enable_anti_aliasing("fxaa")
        RS = _render_scene() if self.real else None
        self.RS = RS
        # vehicle (v6: the mesh set of the run's hardware configuration; a legacy log: BODY_PARTS from STL_DIR as before)
        hw = log.hw
        self.hw = hw
        stl_dir = None if hw.legacy else hardware_stl_dir(hw, warnings)
        self.stl_dir = STL_DIR if stl_dir is None else stl_dir
        self.body_actors = []
        tops = []
        extras = dict(V09_BODY_EXTRAS)
        for name, col in body_part_list(hw):
            if name in extras and not os.path.exists(os.path.join(self.stl_dir, f"{name}.stl")):
                warnings.append(f"{_rel(os.path.join(self.stl_dir, name + '.stl'))} not found: not drawn (small cover)")
                continue
            m = _load_stl(name, warnings, stl_dir)
            m = _standin(name, geom) if m is None else m
            tops.append(m.bounds[5])
            if self.real:
                self.body_actors.append(pl.add_mesh(m, color=RS.PLA_MATTE.get(name, (RS.PLA_BLACK,))[0], pbr=True,
                                                    metallic=0.0, roughness=RS.PLA_ROUGHNESS, smooth_shading=True,
                                                    split_sharp_edges=True))
                self.body_actors[-1].GetProperty().SetBaseIOR(RS.PLA_BASE_IOR)
            else:
                self.body_actors.append(pl.add_mesh(m, color=col, smooth_shading=True, split_sharp_edges=True,
                                                    specular=0.25, specular_power=20))
        self.length_m = float(max(tops)) if tops else 0.86
        self.top_z_m = self.length_m
        if hw.vtol and self.body_actors:
            top_cap = [a for a, (nm, _) in zip(self.body_actors, body_part_list(hw)) if nm == "top_cap"]
            self.top_z_m = float(top_cap[0].mapper.dataset.bounds[5]) if top_cap else self.length_m
            col = _logic_collar_xy_r(hw) if top_cap else None
            if col is not None:
                # switch-collar round (2026-11-05): the collar stands 17 mm over the plate - the GPS parts sit on the
                # PLATE top (the cap's highest point outside the collar), the body length keeps the collar rim
                pts = np.asarray(top_cap[0].mapper.dataset.points)
                out = np.hypot(pts[:, 0] - col[0], pts[:, 1] - col[1]) > col[2] + 5e-4
                if out.any():
                    self.top_z_m = float(pts[out, 2].max())
        # v0.8: the GPS antenna pod (printed in the same PLA Matte Black), moving with the body; v0.9 VTOL: the GPS sits
        # on the top cap (``top_items``), no pod
        self.pod_actors: list[Any] = []
        self.pod_source = None
        if with_gps_pod and hw.vtol:
            self.pod_source = "top cap (params configs.vtol.top_cap.gps)"
        elif with_gps_pod:
            meshes, self.pod_source = load_gps_pod(warnings)
            for m in meshes:
                if self.real:
                    a = pl.add_mesh(m, color=RS.PLA_MATTE["gps_pod"][0], pbr=True, metallic=0.0, roughness=RS.PLA_ROUGHNESS,
                                    smooth_shading=True, split_sharp_edges=True)
                    a.GetProperty().SetBaseIOR(RS.PLA_BASE_IOR)
                else:
                    a = pl.add_mesh(m, color="#3a3f47", smooth_shading=True, split_sharp_edges=True, specular=0.25,
                                    specular_power=20)
                self.pod_actors.append(a)
        vane = _load_stl(VANE_PART, warnings, stl_dir)
        vane = _standin(VANE_PART, geom) if vane is None else vane
        cx, cy = float(np.mean(vane.points[:, 0])), float(np.mean(vane.points[:, 1]))
        self.theta_stl = math.atan2(cy, cx)
        self.vane_mesh = vane
        if self.real:
            self.vane_actors = [pl.add_mesh(vane, color=RS.PLA_MATTE["vane"][0], pbr=True, metallic=0.0, roughness=RS.PLA_ROUGHNESS,
                                            smooth_shading=False) for _ in geom.vane_thetas_rad]
            for va in self.vane_actors:
                va.GetProperty().SetBaseIOR(RS.PLA_BASE_IOR)
        else:
            self.vane_actors = [pl.add_mesh(vane, color=VANE_COLORS[i % 4], smooth_shading=False) for i in range(len(geom.vane_thetas_rad))]
        # v0.9: the direct-drive servo covers on the fin roots (part of the rigid body) and the non-printed top-cap items
        self.hw_actors: list[Any] = []
        self.hw_meshes: list[tuple[str, Any]] = []
        fins = fin_part_mesh(hw, geom, self.theta_stl, warnings)
        if fins is not None:
            if self.real:
                self.body_actors.append(pl.add_mesh(fins, color=RS.PLA_BLACK, pbr=True, metallic=0.0,
                                                    roughness=RS.PLA_ROUGHNESS, smooth_shading=True, split_sharp_edges=True))
                self.body_actors[-1].GetProperty().SetBaseIOR(RS.PLA_BASE_IOR)
            else:
                self.body_actors.append(pl.add_mesh(fins, color=FIN_PART[1], smooth_shading=True, split_sharp_edges=True,
                                                    specular=0.25, specular_power=20))
        for key, m in top_items(hw, self.top_z_m):
            self.hw_meshes.append((key, m))
            self.hw_actors.append(_add_hw_mesh(pl, m, key, self.real))
        self.interior = interior_items(hw, warnings)          # inside the body: no shadow of their own
        for key, m in self.interior:
            self.hw_actors.append(_add_hw_mesh(pl, m, key, self.real))
        # pad + ground (v0.9 VTOL: the pad only when the log's ground lies below the pad top, i.e. the sim modelled it)
        padm = None
        if not hw.vtol or log.ground_z < -0.005:
            padm = _load_stl(PAD_PART[0], warnings)
            padm = _standin(PAD_PART[0], geom) if padm is None else padm
        self.pad_mesh = padm
        self.ring_radii = self._ring_radii(grid)
        if self.real:
            # painted lines only near the pad: far rings alias into dashes at grazing angles (the drift inset and the
            # HUD carry the large distances; the 2 m mowing stripes / the hall's court lines give the scale)
            r_lim = max(5.0, 1.15 * max(self.ring_radii[-1] / 1.15, 0.0))
            self.ring_radii = [r for r in (1.0, 2.0, 3.0, 5.0, 10.0, 20.0) if r <= r_lim]
            if profile == "hall":
                self.ring_radii = [r for r in self.ring_radii if r <= 3.0]
        if self.real:
            if padm is not None:
                pl.add_mesh(padm, color=RS.PLA_MATTE["launch_pad"][0], pbr=True, metallic=0.0, roughness=0.8,
                            smooth_shading=True, split_sharp_edges=True)
            xy = grid.pos[:, :2]
            r_max = float(np.max(np.hypot(xy[:, 0], xy[:, 1]))) if grid.n else 0.0
            self.world = RS.World(pl, profile, log.ground_z, r_max + 40.0,
                                  float(sun_az_deg if sun_az_deg is not None else cam_azimuth_deg + 130.0), asset_dir,
                                  warnings, self.ring_radii, lang)
            self.ground_half = max(1200.0, 4.0 * (r_max + 40.0))
            wi = self.world.info
            self.light_dir = wi.light_dir
            gz = log.ground_z
            # body-frame triangles (decimated) of everything that casts the moving shadow
            tri = [RS.mesh_triangles(a.mapper.dataset, 900) for a in self.body_actors]
            tri += [RS.mesh_triangles(a.mapper.dataset, 200) for a in self.pod_actors]
            tri += [RS.mesh_triangles(m, 120) for _, m in self.hw_meshes]
            self.body_tris_b = np.concatenate(tri, axis=0)
            self.vane_tris_l = RS.mesh_triangles(vane, 60)
            zs = gz + RS.DECAL_Z["shadow"]
            self.shadow_pad = RS.ShadowDecal(pl, zs, strength=wi.shadow_strength, n=128)
            self.shadow_fig = RS.ShadowDecal(pl, zs, strength=wi.shadow_strength, n=192)
            self.shadow_veh = RS.ShadowDecal(pl, zs, strength=wi.shadow_strength)
            if padm is not None:
                self.shadow_pad.update(RS.mesh_triangles(padm, 500), self.light_dir, wi.shadow_blur_base_m + 0.01)
            else:
                self.shadow_pad.actor.visibility = False
            self.jet = RS.Jet(pl, geom.duct_r_m)
            self.gfx = RS.GroundEffect(pl, gz, indoor=profile == "hall")
        else:
            if padm is not None:
                pl.add_mesh(padm, color=PAD_PART[1], smooth_shading=True, split_sharp_edges=True)
            self._build_ground(grid, log.ground_z)
        # v0.9: the tether (boom stand, pulley, catenary / free cable) on a tether / hybrid log
        self.tether = TetherView(self, log, cam_azimuth_deg, warnings) if hw.tethered else None
        # dynamic items
        self.plume_outer, self.plume_a = _plume_mesh()
        self.plume_inner, _ = _plume_mesh()
        self.plume_outer_actor = pl.add_mesh(self.plume_outer, color=PLUME_C, opacity=0.22, smooth_shading=True,
                                             show_edges=False, lighting=False)
        self.plume_inner_actor = pl.add_mesh(self.plume_inner, color="#fff1d6", opacity=0.38, smooth_shading=True,
                                             lighting=False)
        self.splash = pv.Disc(center=(0, 0, 0), inner=0.0, outer=1.0, normal=(0, 0, 1), c_res=40)
        self.splash_actor = pl.add_mesh(self.splash, color=PLUME_C, opacity=0.25, lighting=False)
        self.shadow = pv.Disc(center=(0, 0, 0), inner=0.0, outer=0.13, normal=(0, 0, 1), c_res=40)
        self.shadow_actor = pl.add_mesh(self.shadow, color="#20262e", opacity=0.35, lighting=False)
        if self.real:
            for a in (self.plume_outer_actor, self.plume_inner_actor, self.splash_actor, self.shadow_actor):
                a.visibility = False
        info_on = hud != "none"                       # 3-D information graphics (trail, track, plumb, wind arrows)
        self.info_on = info_on
        self.trail = _lines_polydata(np.zeros((2, 3)))
        self.trail_actor = pl.add_mesh(self.trail, color=TRACK_C, line_width=2.0 if not self.real else 1.6,
                                       opacity=0.8 if not self.real else 0.55, lighting=False)
        self.track = _lines_polydata(np.zeros((2, 3)))
        self.track_actor = pl.add_mesh(self.track, color=TRACK_C, line_width=3.0 if not self.real else 2.0,
                                       opacity=1.0 if not self.real else 0.6, lighting=False)
        self.plumb = _lines_polydata(np.zeros((2, 3)), dashed=True)
        self.plumb_actor = pl.add_mesh(self.plumb, color="#3b3f46", line_width=1.5, opacity=0.7 if not self.real else 0.45,
                                       lighting=False)
        self.wind_mesh, self.gust_mesh = pv.Sphere(radius=1e-4), pv.Sphere(radius=1e-4)
        self.wind_actor = pl.add_mesh(self.wind_mesh, color=WIND_C, smooth_shading=True)
        self.gust_actor = pl.add_mesh(self.gust_mesh, color=GUST_C, smooth_shading=True)
        # v0.8: the hold-point mark on the ground (between the painted rings and the shadows) and the horizontal-error
        # line from it to the ground point below the CG (above the shadows: it is information, not scenery)
        self.hold_actor = self.hold_line_actor = None
        if log.has_hold:
            self.hold_z = log.ground_z + 0.010
            self.hold_actor = pl.add_mesh(hold_marker_mesh(), color=HOLD_C, opacity=0.78 if self.real else 0.9,
                                          lighting=False)
            self.hold_line = _lines_polydata(np.zeros((2, 3)))
            self.hold_line_actor = pl.add_mesh(self.hold_line, color=HOLD_C, line_width=2.2 if self.real else 2.0,
                                               opacity=0.9, lighting=False)
            self.hold_actor.visibility = self.hold_line_actor.visibility = False
        # trail / ground track from the video-rate grid up to the frame time, then the frame's own point
        self.cg_off = np.array([0.0, 0.0, 0.45 * self.length_m])
        self.track_z = log.ground_z + 0.012
        self.grid_trail = grid.pos + np.einsum("nij,j->ni", grid.R, self.cg_off)
        # body radius for the push pad (98th percentile of the printed modules' radial extent)
        rads = []
        for a in self.body_actors[:4]:
            p = np.asarray(a.mapper.dataset.points, dtype=float)
            rads.append(np.hypot(p[:, 0], p[:, 1]))
        self.body_r = float(np.percentile(np.concatenate(rads), 98)) if rads else 0.06
        self.with_figure, self.with_push = with_figure, with_push
        self.mannequin = None
        self.person = None                            # realistic: the rigged 1.75 m human (tools/human_figure.py)
        self._person_tl: PersonTimeline | None = None
        if with_figure:
            if self.real:
                HF = _human_figure()
                try:
                    self.person = HF.HumanActor(pl)
                except (OSError, KeyError, ValueError) as e:     # npz missing / unreadable: the old capsule figure
                    warnings.append(f"rigged person unavailable ({HF.HUMAN_NPZ}: {e}); drawing the capsule mannequin")
                    self.mannequin = RS.Mannequin(pl)
            else:
                fm = figure_meshes()
                self.fig_idle = pl.add_mesh(fm["idle"], color=FIGURE_C, smooth_shading=True, specular=0.1)
                self.fig_push = pl.add_mesh(fm["push"], color=FIGURE_C, smooth_shading=True, specular=0.1)
                self.fig_head = pl.add_mesh(fm["head"], color="#d9b391", smooth_shading=True)
            self.fig_label = self._billboard(tr("person_real" if self.person is not None else "person", lang), 15,
                                             "#1f2933", bg="#ffffff", bg_opacity=0.75)
        if with_push:
            pole_kw: dict[str, Any] = (dict(pbr=True, metallic=0.85, roughness=0.35) if self.real else {})
            self.pole = pl.add_mesh(pv.Cylinder(center=(0, 0, 0.5), direction=(0, 0, 1), radius=1.0, height=1.0,
                                                resolution=20), color="#b9bcc0" if self.real else POLE_C,
                                    smooth_shading=True, **pole_kw)
            pad_kw: dict[str, Any] = (dict(pbr=True, metallic=0.0, roughness=0.95) if self.real else {})
            self.pole_pad = pl.add_mesh(pv.Sphere(radius=0.045, theta_resolution=24, phi_resolution=24), color=PAD_FOAM_C,
                                        smooth_shading=True, **pad_kw)
            self.push_mesh = pv.Sphere(radius=1e-4)
            self.push_actor = pl.add_mesh(self.push_mesh, color=PUSH_C, smooth_shading=True)
            self.glow = pl.add_mesh(pv.Sphere(radius=0.05, theta_resolution=24, phi_resolution=24), color=PUSH_C,
                                    opacity=0.6, lighting=False)
            self.push_label = self._billboard("", 16, "#ffffff", bold=True, bg=PUSH_C, bg_opacity=0.92)
            for a in (self.pole, self.pole_pad, self.push_actor, self.glow):
                a.visibility = False
            self.push_label.SetVisibility(False)
        self.ghosts: list[GhostVehicle] = []
        self.ghost_k: int | None = None               # the ghosts' frame index while motion-blur sub-frames are drawn
        self.inset = VaneInset(geom, self.vane_mesh, self.theta_stl, inset_px, cam_azimuth_deg, warnings)

    def _billboard(self, text: str, size: int, color: str, bold: bool = True, bg: str | None = None,
                   bg_opacity: float = 0.0) -> Any:
        """A 3-D anchored, screen-facing text (VTK font-file text property for Korean)."""
        import vtk
        a = vtk.vtkBillboardTextActor3D()
        a.SetInput(text)
        vtk_text_prop(a.GetTextProperty(), self.lang, int(round(size * self.pl.window_size[1] / 1080.0 * 1.35)), bold,
                      color, bg, bg_opacity)
        self.pl.renderer.AddActor(a)
        return a

    def update_push_figure(self, k: int, fig: FigureTrack | None, fig_opacity: float,
                           glow: tuple[np.ndarray, float] | None, fig_label: bool = True) -> None:
        """The person (pose, place, translucent when it would cover the vehicle), the pole + pad + force arrow + label
        while a push is active, and the fading 'pushed here' marker (world point, age s) after one."""
        fr = self.fr
        if self.with_figure and fig is not None and (self.person is not None or self.mannequin is not None):
            self._pose_mannequin(k, fig, fig_opacity)
            top = (self.person.top_world() + np.array([0.0, 0.0, 0.14]) if self.person is not None
                   else fig.feet[k] + np.array([0.0, 0.0, FIGURE_HEIGHT_M + 0.12]))
            self.fig_label.SetPosition(*top)
            self.fig_label.SetVisibility(bool(fig_label) and self.hud == "full")
        elif self.with_figure and fig is not None:
            c, s_ = math.cos(fig.yaw[k]), math.sin(fig.yaw[k])
            M = homogeneous(np.array([[c, -s_, 0.0], [s_, c, 0.0], [0.0, 0.0, 1.0]]), fig.feet[k])
            pushing = bool(fig.pushing[k])
            for a, vis in ((self.fig_idle, not pushing), (self.fig_push, pushing), (self.fig_head, True)):
                a.user_matrix = M
                a.visibility = vis
                a.prop.opacity = float(fig_opacity)
            self.fig_label.SetPosition(*(fig.feet[k] + np.array([0.0, 0.0, FIGURE_HEIGHT_M + 0.08])))
            self.fig_label.SetVisibility(bool(fig_label))
        if not self.with_push:
            return
        act = fr.push_active is not None and bool(fr.push_active[k])
        if act:
            C, Cpad, Hd, u, fm = self._pole(k, fig)
            Hb = Hd - (Cpad - Hd) / max(float(np.linalg.norm(Cpad - Hd)), 1e-9) * (POLE_BUTT_M if self.real else 0.0)
            v = Cpad - Hb
            Lp = float(np.linalg.norm(v))
            if Lp > 1e-3:
                Mp = homogeneous(_align_z(v) @ np.diag([0.012, 0.012, Lp]), Hb)    # thinner than the arrow shaft
                self.pole.user_matrix = Mp
                self.pole.visibility = True
            self.pole_pad.user_matrix = homogeneous(None, Cpad)
            self.pole_pad.visibility = True
            la = max(0.12, PUSH_ARROW_M_PER_N * fm)
            start = C - u * (la + 0.06)
            show_info = self.hud != "none"
            if show_info:
                self._set_mesh(self.push_actor, self.push_mesh, _arrow_mesh(start, u * la, 0.016))
            else:
                self.push_actor.visibility = False
            self.push_label.SetInput(tr("push_label", self.lang, f=fm))
            self.push_label.SetPosition(*(start + np.array([0.0, 0.0, 0.10])))
            self.push_label.SetVisibility(show_info)
        else:
            for a in (self.pole, self.pole_pad, self.push_actor):
                a.visibility = False
            self.push_label.SetVisibility(False)
            # the rigged person raises / lowers the pole over PERSON_BLEND_S around a push: keep the pole in its hands,
            # the pad backed off the body by (1 - weight) x POLE_APPROACH_M (no arrow / label: no force yet)
            tl = self._person_tl
            if self.person is not None and fig is not None and tl is not None:
                pw, jref = person_push_weight(tl.pushes, float(fr.t[k]))
                if pw > 0.0 and jref is not None:
                    Hd = fig.hands[k]
                    Cpad0 = self._pad_at(int(jref))
                    w = (Cpad0 - Hd) / max(float(np.linalg.norm(Cpad0 - Hd)), 1e-9)
                    Cpad = Cpad0 - w * (1.0 - pw) * POLE_APPROACH_M
                    Hb = Hd - w * POLE_BUTT_M
                    Lp = float(np.linalg.norm(Cpad - Hb))
                    if Lp > 1e-3:
                        self.pole.user_matrix = homogeneous(_align_z(Cpad - Hb) @ np.diag([0.012, 0.012, Lp]), Hb)
                        self.pole.visibility = True
                        self.pole_pad.user_matrix = homogeneous(None, Cpad)
                        self.pole_pad.visibility = True
        if glow is not None and not act and self.hud != "none":
            p, age = glow
            self.glow.user_matrix = homogeneous(None, p)
            self.glow.prop.opacity = float(0.65 * max(0.0, 1.0 - age / PUSH_AFTERGLOW_S))
            self.glow.visibility = age < PUSH_AFTERGLOW_S
        else:
            self.glow.visibility = False

    def _pole(self, k: int, fig: FigureTrack | None) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray, float]:
        """(contact on the body surface, pad centre, the pole's held end, force direction, |F|) of an active push."""
        fr = self.fr
        F = fr.push_force[k]
        fm = float(np.linalg.norm(F))
        C = visual_contact(fr.push_point[k], F, fr.pos[k], fr.R[k], self.body_r)
        u = F / fm if fm > 1e-9 else np.array([1.0, 0.0, 0.0])
        Cpad = C - u * 0.045                                          # pad centre: the foam touches the body at C
        Hd = fig.hands[k] if (fig is not None and self.with_figure) else Cpad - u * POLE_REACH_M
        return C, Cpad, Hd, u, fm

    def set_person_track(self, t_grid: np.ndarray, fig_grid: FigureTrack | None) -> None:
        """The realistic person's timeline (push blends, walking) from the log and the placement track on the grid."""
        self._person_tl = person_timeline(self.log, t_grid, fig_grid)

    def _pad_at(self, j: int) -> np.ndarray:
        """Pad centre of the pole at log sample j (the same construction as ``_pole`` / the 'pushed here' marker)."""
        log = self.log
        F = log.push_force[j]
        fm = float(np.linalg.norm(F))
        C = visual_contact(log.push_point[j], F, log.pos[j], quat_to_matrix(log.q[j]), self.body_r)
        u = F / fm if fm > 1e-9 else np.array([1.0, 0.0, 0.0])
        return C - u * 0.045

    def _pose_person(self, k: int, fig: FigureTrack, opacity: float) -> None:
        """The rigged person: idle (weight on one leg, slow sway) watching the vehicle (head look-at on its CG),
        walking between push positions (gait from the placement track), and pushing - both hands gripping the pole
        (IK onto POLE_GRIP_M along it; the pole passes through each fist's grip cavity), front foot forward, leaning
        in.  The push pose blends in over PERSON_BLEND_S before the push (arms come up to where the pole will be) and
        out after it.  Shadow: the posed 2.5k-triangle shadow mesh through the same projector as before."""
        HF = _human_figure()
        fr = self.fr
        t = float(fr.t[k])
        tl = self._person_tl
        act = self.with_push and fr.push_active is not None and bool(fr.push_active[k])
        pw, jref = (1.0, None) if act else (person_push_weight(tl.pushes, t) if (tl and self.with_push) else (0.0, None))
        grips = wdir = None
        if pw > 0.0:
            Hd = fig.hands[k]
            Cpad = self._pole(k, fig)[1] if act else self._pad_at(int(jref))
            wdir = (Cpad - Hd) / max(float(np.linalg.norm(Cpad - Hd)), 1e-9)
            grips = (Hd + wdir * POLE_GRIP_M[0], Hd + wdir * POLE_GRIP_M[1])
        walk_w, phase, wfig = 0.0, 0.0, None
        if tl is not None and tl.t.shape[0] > 1:
            walk_w = float(np.interp(t, tl.t, tl.walk_w))
            phase = 2.0 * math.pi * float(np.interp(t, tl.t, tl.dist)) / PERSON_WALK_STRIDE_M
            vx, vy = float(np.interp(t, tl.t, tl.vdir[:, 0])), float(np.interp(t, tl.t, tl.vdir[:, 1]))
            c, s_ = math.cos(float(fig.yaw[k])), math.sin(float(fig.yaw[k]))
            wfig = np.array([c * vx + s_ * vy, -s_ * vx + c * vy])         # world -> figure frame
        look = fr.pos[k] + fr.R[k] @ self.cg_off
        inp = HF.PersonInput(feet=fig.feet[k], yaw=float(fig.yaw[k]), t=t, look_at=look, push_w=pw, grips=grips,
                             pole_dir=wdir, walk_w=walk_w, walk_phase=phase, walk_dir=wfig)
        self.person.update(inp, opacity=opacity)
        wi = self.world.info
        self.shadow_fig.update(self.person.triangles_world(), self.light_dir, wi.shadow_blur_base_m + 0.01)

    def _pose_mannequin(self, k: int, fig: FigureTrack, opacity: float) -> None:
        """Realistic person: relaxed, or (pushing) leaning in with both hands on the pole (two-bone arm IK: the rear
        hand POLE_GRIP_M[0] and the front hand POLE_GRIP_M[1] along the pole from its held end).  With the rigged
        human available (the default) this is ``_pose_person``; the capsule mannequin is the fallback."""
        if self.person is not None:
            self._pose_person(k, fig, opacity)
            return
        fr = self.fr
        act = self.with_push and fr.push_active is not None and bool(fr.push_active[k])
        hands = None
        if act:
            _, Cpad, Hd, _, _ = self._pole(k, fig)
            w = (Cpad - Hd) / max(float(np.linalg.norm(Cpad - Hd)), 1e-9)
            hands = [Hd + w * POLE_GRIP_M[0], Hd + w * POLE_GRIP_M[1]]
        self.mannequin.pose(fig.feet[k], float(fig.yaw[k]), hands, 10.0 if act else 0.0, 0.30 if act else 0.0,
                            opacity=opacity)
        wi = self.world.info
        self.shadow_fig.update(self.mannequin.triangles_world(), self.light_dir, wi.shadow_blur_base_m + 0.01)

    def past(self, k: int) -> np.ndarray:
        """Trail points (CG-ish) up to frame k: grid samples before its time, then the frame itself."""
        j = int(np.searchsorted(self.grid.t, self.fr.t[k] - 1e-9, side="left"))
        cur = self.fr.pos[k] + self.fr.R[k] @ self.cg_off
        return np.vstack([self.grid_trail[:j], cur[None, :]])

    @staticmethod
    def _ring_radii(fr: Frames) -> list[float]:
        """Distance rings around the pad: 1 / 2 / 3 m below the step, then nice steps to 1.15 x the largest drift."""
        xy = fr.pos[:, :2]
        r_max = float(np.max(np.hypot(xy[:, 0], xy[:, 1]))) if fr.n else 0.0
        r_lim = max(5.0, 1.15 * r_max)
        stepr = nice_step(r_lim / 5.0)
        return [r for r in (1.0, 2.0, 3.0) if r < stepr] + [float(x) for x in np.arange(stepr, r_lim + stepr, stepr)]

    def _build_ground(self, fr: Frames, gz: float) -> None:
        import pyvista as pv
        xy = fr.pos[:, :2]
        r_max = float(np.max(np.hypot(xy[:, 0], xy[:, 1]))) if fr.n else 0.0
        half = 10.0 * math.ceil(nice_step(max(60.0, 2.0 * r_max + 40.0)) / 10.0)
        self.ground_half = half
        plane = pv.Plane(center=(0, 0, gz), direction=(0, 0, 1), i_size=2 * half, j_size=2 * half, i_resolution=1, j_resolution=1)
        tc = np.asarray(plane.active_texture_coordinates, dtype=float)
        plane.active_texture_coordinates = tc * (2.0 * half / GROUND_TILE_M)       # one texture tile per 10 m, lines on the metres
        self.pl.add_mesh(plane, texture=ground_texture(), ambient=0.45, diffuse=0.6, specular=0.0)
        r_lim = max(5.0, 1.15 * r_max)
        stepr = nice_step(r_lim / 5.0)
        radii = [r for r in (1.0, 2.0, 3.0) if r < stepr] + list(np.arange(stepr, r_lim + stepr, stepr))
        a = np.linspace(0.0, 2.0 * math.pi, 181)
        self.ring_radii = radii
        for r in radii:
            pts = np.column_stack([r * np.cos(a), r * np.sin(a), np.full(a.size, gz + 0.01)])
            self.pl.add_mesh(_lines_polydata(pts), color=RING, line_width=1.6, opacity=0.85, lighting=False)

    def add_ring_labels(self, azimuth_deg: float) -> None:
        """Ring labels on the side nearest the camera."""
        a = math.radians(azimuth_deg) + math.pi
        pts = np.array([[r * math.cos(a), r * math.sin(a), self.log.ground_z + 0.02] for r in self.ring_radii])
        kw: dict[str, Any] = {}
        ff = korean_font_path(False) if self.lang == "ko" else None
        if ff:
            kw["font_file"] = ff                  # the same face as the overlays (VTK font-file text property)
        self.pl.add_point_labels(pts, [f"{r:g} m" for r in self.ring_radii], font_size=13, text_color=RING,
                                 shape_opacity=0.0, show_points=False, always_visible=True, bold=False, **kw)

    def update(self, k: int, cam: CamView, plume_scale: float, view_px: tuple[int, int] = (1280, 1080),
               avoid_px: Sequence[tuple[int, int, int, int]] = ()) -> None:
        fr, g = self.fr, self.geom
        M = body_matrix(fr.pos[k], fr.q[k])
        for a in self.body_actors + self.pod_actors + self.hw_actors:
            a.user_matrix = M
        for i, a in enumerate(self.vane_actors):
            a.user_matrix = M @ vane_local_matrix(g.vane_thetas_rad[i], float(fr.delta[k, i]), g.hinge_z_m, self.theta_stl)
        if self.tether is not None:
            self.tether.update(k, M)
        # plume: length ~ thrust, cut at the ground along the body axis
        T = max(0.0, float(fr.thrust[k]))
        L = 1.1 * T / plume_scale
        axis_w = -fr.R[k][:, 2]
        hit = None
        if axis_w[2] < -0.05:
            to_ground = (self.log.ground_z - fr.pos[k, 2]) / axis_w[2]
            if 0.0 < to_ground < L:
                L, hit = to_ground, fr.pos[k] + axis_w * to_ground
        vis = bool(L > 0.01)
        if self.real:
            self._update_real(k, M, T, L, hit, plume_scale, cam)
        for mesh, actor, r0, spread, frac in ((self.plume_outer, self.plume_outer_actor, g.duct_r_m, 0.22, 1.0),
                                              (self.plume_inner, self.plume_inner_actor, 0.75 * g.duct_r_m, 0.08, 0.7)):
            if self.real:
                break
            if vis:
                Lf = L * frac
                n = self.plume_a.size
                pts = np.empty((2 * n, 3))
                pts[:n, 0], pts[:n, 1], pts[:n, 2] = r0 * np.cos(self.plume_a), r0 * np.sin(self.plume_a), -0.002
                r1 = r0 + spread * Lf
                pts[n:, 0], pts[n:, 1], pts[n:, 2] = r1 * np.cos(self.plume_a), r1 * np.sin(self.plume_a), -Lf
                mesh.points = pts
                actor.user_matrix = M
            actor.visibility = bool(vis)
        if hit is not None and vis and not self.real:
            rs = g.duct_r_m + 0.22 * L + 0.1 * T / plume_scale
            self.splash.points = self._disc_points(self.splash, hit[0], hit[1], self.log.ground_z + 0.015, rs)
            self.splash_actor.visibility = True
        else:
            self.splash_actor.visibility = False
        # shadow (fades with height)
        hgt = float(fr.pos[k, 2] - self.log.ground_z)
        self.shadow.points = self._disc_points(self.shadow, fr.pos[k, 0], fr.pos[k, 1], self.log.ground_z + 0.008, 0.13)
        self.shadow_actor.prop.opacity = float(0.4 * np.clip(1.0 - hgt / 40.0, 0.15, 1.0))
        # trail (3-D), ground track, plumb line
        tr = self.past(k)
        self.trail.copy_from(_lines_polydata(tr))
        self.track.copy_from(_lines_polydata(np.column_stack([tr[:, 0], tr[:, 1], np.full(tr.shape[0], self.track_z)])))
        top = fr.pos[k]
        nd = max(2, int(hgt / 0.25) // 2 * 2)
        self.plumb.copy_from(_lines_polydata(np.linspace(top, [top[0], top[1], self.log.ground_z + 0.01], nd), dashed=True))
        self.plumb_actor.visibility = bool(hgt > 0.4) and self.info_on
        self.trail_actor.visibility = self.track_actor.visibility = self.info_on
        if self.real:
            self.shadow_actor.visibility = False
        self._update_hold(k)
        gk = self.ghost_k if self.ghost_k is not None else k
        for gh in self.ghosts:
            gh.update(min(gk, gh.fr.n - 1))
        # wind / gust arrows (glyphs: screen size ~ constant per m/s; the HUD carries the numbers)
        dcam = float(np.linalg.norm(cam.position - cam.focal))
        kk = 0.015 * dcam
        w = fr.wind[k]
        steady = fr.wind_steady[k]
        gust = w - steady if fr.gust_active[k] else np.zeros(3)
        center = fr.pos[k] + fr.R[k] @ np.array([0.0, 0.0, 0.55 * self.length_m])
        wh = np.array([w[0], w[1], 0.0])
        wn = float(np.linalg.norm(wh))
        if wn > 0.05 and self.info_on:
            u = wh / wn
            gap = 0.35 + 0.02 * dcam
            span = kk * (np.array([steady[0], steady[1], 0.0]) + np.array([gust[0], gust[1], 0.0]))
            p0 = center - u * gap - span                     # upwind of the vehicle, pointing at it
            pts, _ = project(np.array([p0, center - u * gap]), cam, view_px[0], view_px[1])
            if any(x0 <= px <= x1 and y0 <= py <= y1 for px, py in pts for x0, y0, x1, y1 in avoid_px):
                p0 = center + u * gap                         # would sit under an overlay: draw it downwind
            shaft = 0.0022 * dcam
            am = _arrow_mesh(p0, kk * np.array([steady[0], steady[1], 0.0]), shaft)
            self._set_mesh(self.wind_actor, self.wind_mesh, am)
            gm = _arrow_mesh(p0 + kk * np.array([steady[0], steady[1], 0.0]), kk * np.array([gust[0], gust[1], 0.0]), shaft * 1.15) \
                if fr.gust_active[k] else None
            self._set_mesh(self.gust_actor, self.gust_mesh, gm)
        else:
            self._set_mesh(self.wind_actor, self.wind_mesh, None)
            self._set_mesh(self.gust_actor, self.gust_mesh, None)
        pl = self.pl
        pl.camera.position = tuple(cam.position)
        pl.camera.focal_point = tuple(cam.focal)
        pl.camera.up = tuple(cam.up)
        pl.camera.view_angle = cam.view_angle_deg
        if self.real:
            # the ground / walls do not count in the bounds (a 2.4 km field would push the near plane out): the near
            # plane follows the distance to the vehicle and the camera's height, the far plane is past the ground
            d_v = float(np.linalg.norm(cam.position - fr.pos[k]))
            h_c = max(float(cam.position[2] - self.log.ground_z), 0.05)
            near = float(np.clip(min(0.2 * d_v, 0.5 * h_c), 0.03, 2.0))
            pl.camera.clipping_range = (near, 4.0 * self.ground_half)
        else:
            pl.renderer.ResetCameraClippingRange()

    def _update_real(self, k: int, M: np.ndarray, T: float, L: float, hit: np.ndarray | None, plume_scale: float,
                     cam: CamView) -> None:
        """Realistic jet, ground effect and the projected sun shadow of the vehicle (formulas in render_scene)."""
        fr, g, RS = self.fr, self.geom, self.RS
        t = float(fr.t[k])
        self.jet.update(M, L, t, T > 1e-3)
        axis_w = -fr.R[k][:, 2]
        h_exit = float(fr.pos[k, 2] - self.log.ground_z)
        if axis_w[2] < -0.2:
            h_exit = h_exit / max(-axis_w[2], 1e-3)              # along the jet
            imp = fr.pos[k] + axis_w * h_exit
        else:
            imp = fr.pos[k]
        I = RS.ground_effect_strength(T / max(plume_scale, 1e-9), h_exit) if axis_w[2] < -0.2 else 0.0
        dcam = max(float(np.linalg.norm(cam.position - imp)), 0.1)
        px_per_m = self.pl.window_size[1] / (2.0 * dcam * math.tan(math.radians(cam.view_angle_deg) / 2.0))
        self.gfx.update(imp[:2], I, t, px_per_m)
        # vehicle shadow: body + vanes, projected along the light; a soft contact blob near the ground
        wi = self.world.info
        tris = [self.body_tris_b @ M[:3, :3].T + M[:3, 3]]
        for i in range(len(g.vane_thetas_rad)):
            Mv = M @ vane_local_matrix(g.vane_thetas_rad[i], float(fr.delta[k, i]), g.hinge_z_m, self.theta_stl)
            tris.append(self.vane_tris_l @ Mv[:3, :3].T + Mv[:3, 3])
        hgt = float(fr.pos[k, 2] - self.log.ground_z)
        blur = wi.shadow_blur_base_m + wi.shadow_blur_per_m * max(hgt, 0.0)
        blobs = []
        if hgt < 0.8:
            blobs.append((float(fr.pos[k, 0]), float(fr.pos[k, 1]), 0.22 + 0.3 * hgt, 0.7 * (1.0 - hgt / 0.8)))
        self.shadow_veh.update(np.concatenate(tris, axis=0), self.light_dir, blur, blobs)

    def _update_hold(self, k: int) -> None:
        """The hold-point mark at the frame's hold point and the error line to the ground point below the CG (hidden
        with --hud none and while there is no hold point yet)."""
        if self.hold_actor is None:
            return
        fr = self.fr
        hx = fr.hold_xy[k] if fr.hold_xy is not None else None
        show = bool(self.info_on and hx is not None and np.all(np.isfinite(hx)))
        if show:
            self.hold_actor.user_matrix = homogeneous(None, (float(hx[0]), float(hx[1]), self.hold_z))
            c = fr.cg_xy[k]
            zl = self.log.ground_z + 0.014
            self.hold_line.copy_from(_lines_polydata(np.array([[hx[0], hx[1], zl], [c[0], c[1], zl]])))
        self.hold_actor.visibility = self.hold_line_actor.visibility = show

    def add_ghost(self, frames: "Frames", label: str = "", color: str = "#cfd5dd", opacity: float = 0.42,
                  label_color: str = "#3d434b", label_size: int = 15, label_dz: float = 0.12,
                  hardware: "Hardware | None" = None, warnings: list[str] | None = None) -> "GhostVehicle":
        """A translucent second vehicle in this scene (``GhostVehicle``), posed from ``frames`` (same frame index).
        ``hardware`` (v6): the ghost log's configuration - when it differs from this scene's, the ghost draws its own
        mesh set (else it copies this scene's meshes, as before)."""
        g = GhostVehicle(self, frames, label, color, opacity, label_color, label_size, label_dz, hardware, warnings)
        self.ghosts.append(g)
        return g

    @staticmethod
    def _disc_points(mesh: Any, x: float, y: float, z: float, r: float) -> np.ndarray:
        base = getattr(mesh, "_unit_pts", None)
        if base is None:
            p = np.asarray(mesh.points, dtype=float).copy()
            rr = np.max(np.hypot(p[:, 0], p[:, 1]))
            base = p / (rr if rr > 0 else 1.0)
            mesh._unit_pts = base
        pts = base * r
        pts[:, 0] += x
        pts[:, 1] += y
        pts[:, 2] = z
        return pts

    @staticmethod
    def _set_mesh(actor: Any, target: Any, mesh: Any) -> None:
        """Replace the geometry of ``target`` (the dataset given to add_mesh) in place, or hide the actor."""
        if mesh is None:
            actor.visibility = False
            return
        target.copy_from(mesh)
        actor.visibility = True

    def render(self) -> np.ndarray:
        self.pl.render()                     # off-screen: screenshot() alone returns the previous image
        return self.pl.screenshot(return_img=True)

    def close(self) -> None:
        self.pl.close()
        self.inset.close()


class GhostVehicle:
    """A translucent second vehicle in a ``Scene`` (the "before" vehicle of a shared before / after scene): the printed
    modules and the vanes of the scene's own meshes (the GPS pod is not drawn: the ghost is the v0.7.1 build), light grey
    at ``opacity``, posed per frame index from its own ``frames`` (vanes at their logged angles), with its ground track
    (thin, grey) and a label billboard above it.  It casts no shadow (a visual aid, not scenery)."""

    def __init__(self, scene: Scene, frames: "Frames", label: str = "", color: str = "#cfd5dd", opacity: float = 0.42,
                 label_color: str = "#3d434b", label_size: int = 15, label_dz: float = 0.12,
                 hardware: "Hardware | None" = None, warnings: list[str] | None = None):
        self.scene, self.fr = scene, frames
        pl = scene.pl
        kw: dict[str, Any] = (dict(pbr=True, metallic=0.0, roughness=0.55) if scene.real else dict(specular=0.2))
        body_meshes = [a.mapper.dataset.copy() for a in scene.body_actors]
        vane_mesh, self.theta_stl, self.length_m = scene.vane_mesh, scene.theta_stl, scene.length_m
        if hardware is not None and hardware != scene.hw:
            w = warnings if warnings is not None else []
            d = None if hardware.legacy else hardware_stl_dir(hardware, w)
            body_meshes, tops = [], []
            for name, _ in body_part_list(hardware):
                p = os.path.join(STL_DIR if d is None else d, f"{name}.stl")
                if name in dict(V09_BODY_EXTRAS) and not os.path.exists(p):
                    continue
                m = _load_stl(name, w, d)
                m = _standin(name, scene.geom) if m is None else m
                body_meshes.append(m)
                tops.append(m.bounds[5])
            vm = _load_stl(VANE_PART, w, d)
            vane_mesh = _standin(VANE_PART, scene.geom) if vm is None else vm
            self.theta_stl = stl_fin_angle(vane_mesh)
            fins = fin_part_mesh(hardware, scene.geom, self.theta_stl, w)
            if fins is not None:
                body_meshes.append(fins)
            self.length_m = float(max(tops)) if tops else scene.length_m
        self.body = [pl.add_mesh(m, color=color, opacity=float(opacity), smooth_shading=True, **kw) for m in body_meshes]
        self.vanes = [pl.add_mesh(vane_mesh.copy(), color=color, opacity=float(opacity), smooth_shading=False, **kw)
                      for _ in scene.geom.vane_thetas_rad]
        self.track = _lines_polydata(np.zeros((2, 3)))
        self.track_actor = pl.add_mesh(self.track, color="#8a9099", line_width=1.8, opacity=0.75, lighting=False)
        self.label = scene._billboard(label, label_size, label_color, bg="#ffffff", bg_opacity=0.72) if label else None
        self.cg_off = scene.cg_off
        self.label_dz = float(label_dz)                  # label height above the nose tip (m, body axis)

    def points(self, k: int) -> np.ndarray:
        """(3, 3) exit plane centre, mid body, nose tip of frame k (world): for the camera / off-screen checks."""
        fr = self.fr
        body = np.outer([0.0, 0.5 * self.length_m, self.length_m], [0.0, 0.0, 1.0])
        return fr.pos[k] + body @ fr.R[k].T

    def update(self, k: int) -> None:
        sc, fr, g = self.scene, self.fr, self.scene.geom
        M = body_matrix(fr.pos[k], fr.q[k])
        for a in self.body:
            a.user_matrix = M
        for i, a in enumerate(self.vanes):
            a.user_matrix = M @ vane_local_matrix(g.vane_thetas_rad[i], float(fr.delta[k, i]), g.hinge_z_m, self.theta_stl)
        past = fr.pos[:k + 1] + np.einsum("nij,j->ni", fr.R[:k + 1], self.cg_off)
        if past.shape[0] > 600:                                   # a light polyline (the track is a thin cue)
            past = np.vstack([past[:: int(math.ceil(past.shape[0] / 600))], past[-1:]])
        self.track.copy_from(_lines_polydata(np.column_stack([past[:, 0], past[:, 1], np.full(past.shape[0], sc.track_z)])))
        self.track_actor.visibility = sc.info_on
        if self.label is not None:
            top = fr.pos[k] + fr.R[k] @ np.array([0.0, 0.0, self.length_m + self.label_dz])
            self.label.SetPosition(*top)
            self.label.SetVisibility(sc.hud != "none")


def vane_forces_world(R: np.ndarray, delta: np.ndarray, thetas: Sequence[float], thrust_frac: float,
                      delta_ref: float) -> np.ndarray:
    """Relative side forces of the vanes on the vehicle in the world frame (SPEC 8.2 / 8.3:
    F_i ~ T delta_i along e_theta_i), normalised so that delta_ref at thrust_frac = 1 gives 1."""
    f = np.array([float(d) / delta_ref * max(0.0, thrust_frac) * e_theta(th) for d, th in zip(delta, thetas)])
    return f @ np.asarray(R).T


class VaneInset:
    """Close-up of the vane section with the vehicle's attitude, seen from the main camera's azimuth and
    from below (the duct translucent, neutral vanes as ghosts), with the side force of each vane on the
    vehicle (coloured arrows under the exit plane) and their sum (black)."""

    ELEV_DEG = 28.0          # camera below the exit plane, looking up
    DIST_M = 0.23
    FOCUS_Z = 0.026

    def __init__(self, geom: VehicleGeom, vane_mesh: Any, theta_stl: float, px: int, cam_azimuth_deg: float,
                 warnings: list[str]):
        import pyvista as pv
        self.geom = geom
        self.theta_stl = theta_stl
        self.px = int(px)
        self.az = cam_azimuth_deg
        self.pl = pv.Plotter(off_screen=True, window_size=(self.px, self.px), lighting="light kit")
        pl = self.pl
        pl.enable_anti_aliasing("msaa", multi_samples=4)
        pl.set_background("#f4f6f8", top="#dde5ee")
        r = geom.duct_r_m
        self.static_actors = []
        duct = pv.Cylinder(center=(0, 0, 0.032), direction=(0, 0, 1), radius=r + 0.0012, height=0.064, resolution=72,
                           capping=False)
        self.static_actors.append(pl.add_mesh(duct, color="#7b8794", opacity=0.15, smooth_shading=True))
        a = np.linspace(0.0, 2.0 * math.pi, 121)
        ring0 = np.column_stack([r * np.cos(a), r * np.sin(a), np.zeros(a.size)])
        ring1 = np.column_stack([r * np.cos(a), r * np.sin(a), np.full(a.size, 0.064)])
        self.static_actors.append(pl.add_mesh(_lines_polydata(ring0), color="#374151", line_width=2.0, lighting=False))
        self.static_actors.append(pl.add_mesh(_lines_polydata(ring1), color="#9aa3ad", line_width=1.2, lighting=False))
        hub = pv.Cylinder(center=(0, 0, 0.052), direction=(0, 0, 1), radius=0.013, height=0.024, resolution=36)
        self.static_actors.append(pl.add_mesh(hub, color="#9aa3ad", smooth_shading=True))
        self.ghosts = []
        for th in geom.vane_thetas_rad:
            ghost = vane_mesh.transform(vane_local_matrix(th, 0.0, geom.hinge_z_m, theta_stl), inplace=False)
            self.ghosts.append(pl.add_mesh(ghost, color="#ffffff", opacity=0.35, style="wireframe", line_width=0.6,
                                           lighting=False))
        self.vane_actors = [pl.add_mesh(vane_mesh, color=VANE_COLORS[i % 4], smooth_shading=False, specular=0.2)
                            for i in range(len(geom.vane_thetas_rad))]
        self.force_meshes = [pv.Sphere(radius=1e-5) for _ in geom.vane_thetas_rad]
        self.force_actors = [pl.add_mesh(m, color=VANE_COLORS[i % 4], smooth_shading=True)
                             for i, m in enumerate(self.force_meshes)]
        self.net_mesh = pv.Sphere(radius=1e-5)
        self.net_actor = pl.add_mesh(self.net_mesh, color="#111111", smooth_shading=True)
        self.cam = CamView(np.zeros(3), np.zeros(3), np.array([0.0, 0.0, 1.0]), 30.0)
        self.label_pts = np.zeros((0, 3))
        self.net_tip = None

    def update(self, q: np.ndarray, delta: np.ndarray, thrust_frac: float) -> None:
        g = self.geom
        R = quat_to_matrix(q)
        M = homogeneous(R)
        for a in self.static_actors + self.ghosts:
            a.user_matrix = M
        for i, a in enumerate(self.vane_actors):
            a.user_matrix = M @ vane_local_matrix(g.vane_thetas_rad[i], float(delta[i]), g.hinge_z_m, self.theta_stl)
        F = vane_forces_world(R, delta, g.vane_thetas_rad, thrust_frac, g.delta_cmd_max_rad)
        scale_m = 0.06                       # arrow length of delta_ref (max command) at full thrust
        z_ar = -0.014                        # arrows drawn just under the exit plane, below their vane
        for i, a in enumerate(self.force_actors):
            th = g.vane_thetas_rad[i]
            start = R @ np.array([g.vane_force_r_m * math.cos(th), g.vane_force_r_m * math.sin(th), z_ar])
            v = scale_m * F[i]
            Scene._set_mesh(a, self.force_meshes[i], _arrow_mesh(start, v, 0.0016) if np.linalg.norm(v) > 0.003 else None)
        net = 0.5 * scale_m * F.sum(axis=0)                # the sum at half scale (it can reach ~2.8 x)
        c0 = R @ np.array([0.0, 0.0, z_ar - 0.012])
        self.net_tip = c0 + net if np.linalg.norm(net) > 0.003 else None
        Scene._set_mesh(self.net_actor, self.net_mesh, _arrow_mesh(c0, net, 0.0024) if self.net_tip is not None else None)
        # camera: the main view's azimuth, from below, centred on the vane section
        e = math.radians(self.ELEV_DEG)
        az = math.radians(self.az)
        f = np.array([math.cos(e) * math.cos(az), math.cos(e) * math.sin(az), math.sin(e)])   # looking up
        foc = R @ np.array([0.0, 0.0, self.FOCUS_Z])
        r_ = np.cross(f, [0.0, 0.0, 1.0])
        r_ /= np.linalg.norm(r_)
        up = np.cross(r_, f)
        self.cam = CamView(foc - self.DIST_M * f, foc, up, 30.0)
        pl = self.pl
        pl.camera.position = tuple(self.cam.position)
        pl.camera.focal_point = tuple(self.cam.focal)
        pl.camera.up = tuple(up)
        pl.camera.view_angle = 30.0
        pl.renderer.ResetCameraClippingRange()
        self.label_pts = np.array([R @ np.array([0.058 * math.cos(th), 0.058 * math.sin(th), 0.040])
                                   for th in g.vane_thetas_rad])

    def labels_px(self) -> list[tuple[str, float, float]]:
        """Vane numbers (and the net arrow tip) projected into the inset image."""
        px, z = project(self.label_pts, self.cam, self.px, self.px)
        out = [(str(i + 1), float(p[0]), float(p[1])) for i, (p, zz) in enumerate(zip(px, z)) if zz > 0]
        if self.net_tip is not None:
            p, zz = project(self.net_tip[None, :], self.cam, self.px, self.px)
            if zz[0] > 0:
                out.append(("net", float(p[0, 0]), float(p[0, 1])))
        return out

    def render(self) -> np.ndarray:
        self.pl.render()                     # off-screen: screenshot() alone returns the previous image
        return self.pl.screenshot(return_img=True)

    def close(self) -> None:
        self.pl.close()


# --------------------------------------------------------------------------- #
# composition
# --------------------------------------------------------------------------- #
@dataclass
class Layout:
    W: int
    H: int
    main_w: int
    panel_w: int
    scale: float
    inset_px: int
    drift_px: int

    @classmethod
    def for_size(cls, W: int, H: int, full: bool = False) -> "Layout":
        """Left 2/3 the 3-D view, right 1/3 the side panel; ``full``: the 3-D view fills the frame (--hud minimal/none)."""
        W, H = int(W) // 2 * 2, int(H) // 2 * 2
        main_w = W if full else int(round(W * 2.0 / 3.0)) // 2 * 2
        s = H / 1080.0
        return cls(W, H, main_w, W - main_w, s, int(round(290 * s)), int(round(290 * s)))


def summarize(log: SimLog, t0: float, t1: float) -> dict[str, Any]:
    sel = (log.t >= t0 - 1e-9) & (log.t <= t1 + 1e-9)
    t = log.t[sel]
    pos = log.pos[sel]
    hor = np.hypot(pos[:, 0], pos[:, 1])
    powered = log.state[sel] == STATE_POWERED
    u = log.throttle[sel]
    dt = np.diff(np.append(t, t[-1])) if t.size else np.zeros(0)
    sat = powered & (u >= log.u_max - 1e-3)
    out: dict[str, Any] = {
        "t0": float(t[0]) if t.size else t0, "t1": float(t[-1]) if t.size else t1,
        "max_height_m": float(np.max(pos[:, 2])) if t.size else float("nan"),
        "final_drift_m": float(hor[-1]) if t.size else float("nan"),
        "max_drift_m": float(np.max(hor)) if t.size else float("nan"),
        "max_tilt_deg": float(np.max(log.tilt_deg[sel])) if t.size else float("nan"),
        "powered_s": float(np.sum(dt[powered])) if t.size else 0.0,
        "throttle_saturated_s": float(np.sum(dt[sat])) if t.size else 0.0,
        "final_state": log.state_name(int(log.state[sel][-1])) if t.size else "?",
        "final_state_code": int(log.state[sel][-1]) if t.size else None,
        "final_phase": log.phase_name(int(log.phase[sel][-1])) if t.size and log.has_phase else None,
        "final_phase_code": int(log.phase[sel][-1]) if t.size and log.has_phase else None,
        "final_speed_mps": float(np.linalg.norm(log.vel[sel][-1])) if t.size else float("nan"),
        "commands": [(float(ct), cn) for ct, cn in zip(log.cmd_t, log.cmd_name) if t0 - 1e-9 <= ct <= t1 + 1e-9],
        "events": [(float(et), ex) for et, ex in zip(log.event_t, log.event_text) if t0 - 1e-9 <= et <= t1 + 1e-9],
    }
    return out


def station_keeping(log: SimLog, t0: float | None = None, t1: float | None = None) -> dict[str, Any] | None:
    """Station keeping read from a log with a hold point (None without one), over [t0, t1]: the distance of the true
    CG from the hold point over the HOLD samples (RMS, max, the HOLD time), at the first touchdown ('touchdown #1'
    event), the GPS-usable fraction and the time without GPS while POWERED, and the RMS error of the NAV position
    estimate (|pos_est_xy - CG_xy|) while POWERED.  Definitions as ``sim/vtol_scenarios`` / the v0.8 study: truth CG
    against the hold point."""
    if not log.has_hold:
        return None
    t = log.t
    a = float(t[0]) if t0 is None else float(t0)
    b = float(t[-1]) if t1 is None else float(t1)
    sel = (t >= a - 1e-9) & (t <= b + 1e-9)
    hd = log.hold_dist()
    hold = sel & (log.phase == 3) & np.isfinite(hd)
    dt = np.diff(np.append(t, t[-1]))
    out: dict[str, Any] = {"definition": "|CG_xy - hold_xy| (truth CG, meta.z_cg_m) over the HOLD samples",
                           "hold_s": float(np.sum(dt[hold]))}
    fin = np.flatnonzero(np.isfinite(log.hold_xy[:, 0]))
    out["hold_point_xy"] = [float(x) for x in log.hold_xy[fin[0]]] if fin.size else None
    out["t_hold_point_s"] = float(t[fin[0]]) if fin.size else None
    out["hold_rms_m"] = float(np.sqrt(np.mean(hd[hold] ** 2))) if np.any(hold) else None
    out["hold_max_m"] = float(np.max(hd[hold])) if np.any(hold) else None
    out["touchdown_from_hold_m"] = None
    for et, ex in zip(log.event_t, log.event_text):
        if a - 1e-9 <= et <= b + 1e-9 and str(ex).startswith("touchdown #1"):
            v = float(np.interp(et, t, np.where(np.isfinite(hd), hd, np.nan)))
            out["touchdown_from_hold_m"] = v if math.isfinite(v) else None
            break
    pw = sel & (log.state == STATE_POWERED)
    if log.gps_ok is not None and np.any(pw):
        out["gps_ok_frac_powered"] = float(np.sum(dt[pw & log.gps_ok]) / max(np.sum(dt[pw]), 1e-12))
        out["gps_lost_s_powered"] = float(np.sum(dt[pw & ~log.gps_ok]))
    if log.pos_est is not None and np.any(pw):
        # absolute: includes the GPS's own offset (common to the hold point - it does not move the vehicle)
        e = np.hypot(*(log.pos_est[:, :2] - log.cg_xy()).T)
        ok = pw & np.isfinite(e)
        out["nav_pos_err_rms_m"] = float(np.sqrt(np.mean(e[ok] ** 2))) if np.any(ok) else None
        # relative to the hold point: the drift of that error since the capture (what does move the vehicle)
        er = log.est_rel_xy()
        if er is not None:
            ed = np.hypot(*(er - log.cg_xy()).T)
            okd = hold & np.isfinite(ed)
            out["nav_err_drift_rms_m"] = float(np.sqrt(np.mean(ed[okd] ** 2))) if np.any(okd) else None
    out["criterion_max_m"] = log.station_keeping_limit()
    return out


def push_row_text(p: PushInfo, lang: str, t_now: float | None = None) -> str:
    """One line of the push table; ``t_now`` (a frame time) hides what is not known yet at that time."""
    if t_now is not None and t_now < p.window_end and (p.recovery_s is None or t_now < p.t1 + p.recovery_s):
        rec = tr("rec_pending", lang)
    elif p.recovery_s is None:
        rec = tr("rec_none", lang)
    elif p.recovery_s == 0.0:
        rec = tr("rec_never_out", lang)
    else:
        rec = tr("rec_ok", lang, r=p.recovery_s)
    peak = p.peak_tilt_deg
    moved = p.moved_m
    return tr("push_row", lang, i=p.i, f=p.f_peak_n, d=p.duration_s, p=peak, rec=rec, m=moved)


def gps_caveats(lang: str) -> list[str]:
    """The GPS statements every v0.8 card carries (physical limits; numbers: GPS_QUOTE_* / GPS_WANDER_M, sources there)."""
    if lang == "ko":
        return [f"GPS 위치 유지 (u-blox NEO-6M 모델): GPS 위치 자체가 약 {GPS_WANDER_M:g} m 범위에서 천천히 흔들림 (모델 가정) - "
                f"실제 모듈을 측정하기 전에는 제자리 유지를 '약 {GPS_QUOTE_RMS_M:g} m RMS, 최대 {GPS_QUOTE_MAX_M:g} m'로 표기",
                "GPS는 하늘이 열린 실외에서만 작동: 실내에서는 위치 유지 불가 (자세만 유지하고 바람에 밀림)"]
    return [f"GPS position hold (u-blox NEO-6M model): the GPS position itself wanders ~{GPS_WANDER_M:g} m (model "
            f"assumption) - station keeping is quoted as ~{GPS_QUOTE_RMS_M:g} m RMS / {GPS_QUOTE_MAX_M:g} m max until a "
            "real module is measured",
            "GPS needs open sky: no position hold indoors (attitude only, the vehicle drifts with the air)"]


def station_keeping_lines(sk: dict[str, Any] | None, lang: str) -> list[str]:
    """End-card lines of ``station_keeping`` (values read from the log)."""
    if not sk:
        return []
    ko = lang == "ko"

    def f(v: Any, fmt: str = ".2f") -> str:
        return "--" if v is None else format(float(v), fmt)
    out = []
    if ko:
        line = f"제자리 유지 (참값, 유지점 기준, 호버링 {f(sk.get('hold_s'), '.1f')} s): RMS {f(sk.get('hold_rms_m'))} m, " \
               f"최대 {f(sk.get('hold_max_m'))} m"
        if sk.get("criterion_max_m") is not None:
            line += f"  (기준 최대 {sk['criterion_max_m']:g} m)"
        if sk.get("touchdown_from_hold_m") is not None:
            line += f"  ·  첫 착지 지점: 유지점에서 {f(sk['touchdown_from_hold_m'])} m"
        out.append(line)
        if "gps_ok_frac_powered" in sk:
            g = f"GPS 사용 가능: 동력 비행 시간의 {100.0 * sk['gps_ok_frac_powered']:.0f} % (사용 불가 {sk['gps_lost_s_powered']:.1f} s)"
            if sk.get("nav_err_drift_rms_m") is not None:
                g += f"  ·  GPS·항법 오차의 흔들림 (유지점 설정 후) RMS {f(sk['nav_err_drift_rms_m'])} m"
            out.append(g)
    else:
        line = f"station keeping (truth, from the hold point, HOLD {f(sk.get('hold_s'), '.1f')} s): RMS " \
               f"{f(sk.get('hold_rms_m'))} m, max {f(sk.get('hold_max_m'))} m"
        if sk.get("criterion_max_m") is not None:
            line += f"  (criterion max {sk['criterion_max_m']:g} m)"
        if sk.get("touchdown_from_hold_m") is not None:
            line += f"  -  first touchdown {f(sk['touchdown_from_hold_m'])} m from the hold point"
        out.append(line)
        if "gps_ok_frac_powered" in sk:
            g = f"GPS usable {100.0 * sk['gps_ok_frac_powered']:.0f} % of the powered time (not usable {sk['gps_lost_s_powered']:.1f} s)"
            if sk.get("nav_err_drift_rms_m") is not None:
                g += f"  -  drift of the GPS / NAV error since the hold point was set: RMS {f(sk['nav_err_drift_rms_m'])} m"
            out.append(g)
    return out


def notes_ko_lines(notes: str, mode: str, gps: bool = False) -> list[str]:
    """Korean statements of the model caveats recognised in the (English) ``meta.notes``; ``gps``: the vehicle holds
    position with GPS (no 'no position sensor' statement)."""
    s = str(notes)
    low = s.lower()
    out: list[str] = []
    if "development log" in low:
        out.append("렌더러 개발용 로그 (VTOL 시뮬레이션이 아님)")
    if "test pattern" in low or "kinematic" in low:
        out.append("렌더러 테스트 패턴: 운동학 곡선이며 물리 계산이 아님")
    if "ground-effect" in low or "ground effect" in low:
        out.append("지면 효과 모델 없음 (지면 근처에서도 자유 공기 중 추력으로 계산)")
    if "position" in low and ("sensing" in low or "sensor" in low) and not gps:
        out.append("수평 위치·속도 센서 없음: 바람과 자세 추정 오차만큼 기체가 옆으로 밀려남 (이동 거리로 표시)")
    if "tip-over" in low or "tip over" in low:
        out.append("전복 모델 없음 (착지 후 넘어지는 것은 계산하지 않음)")
    if "no bounce" in low or "rigid surface" in low:
        out.append("지면 접촉은 단단한 바닥, 튕김 없음")
    if "battery-voltage" in low or "battery voltage" in low:
        out.append("추력 모델에 배터리 전압 영향 없음 (별도 표기한 경우 제외)")
    if "wind uniform" in low:
        out.append("바람은 높이에 관계없이 균일")
    if mode == "vtol" and not gps and not any("센서" in x for x in out):
        out.append("수평 위치 센서 없음: 바람에 의한 이동은 보정되지 않음")
    return out


class Renderer:
    def __init__(self, log: SimLog, t0: float, t1: float, fps: float, speed: float, size: tuple[int, int],
                 framing: str = "all", azimuth_deg: float | None = None, elevation_deg: float | None = None,
                 aa: str = "msaa", tq: np.ndarray | None = None, lang: str = "ko", figure: str = "auto",
                 style: str = "schematic", hud: str = "full", camera: str = "chase", motion_blur: int = 1,
                 scene: str = "auto", asset_dir: str | None = ASSET_DIR, gps_pod: str = "auto"):
        if style not in STYLES:
            raise ValueError("style must be " + " | ".join(STYLES))
        if hud not in HUDS:
            raise ValueError("hud must be " + " | ".join(HUDS))
        if camera not in CAMERAS:
            raise ValueError("camera must be " + " | ".join(CAMERAS))
        if gps_pod not in GPS_POD_MODES:
            raise ValueError("gps_pod must be " + " | ".join(GPS_POD_MODES))
        self.style, self.hud, self.camera = style, hud, camera
        self.motion_blur = max(1, int(motion_blur))
        self.profile = scene_profile(log.meta, scene, gps=log.has_gps)
        self.with_gps_pod = gps_pod == "on" or (gps_pod == "auto" and log.has_gps)
        if lang not in LANGS:
            raise ValueError("lang must be " + " | ".join(LANGS))
        if figure not in ("auto", "on", "off"):
            raise ValueError("figure must be auto | on | off")
        if framing not in FRAMINGS:
            raise ValueError("framing must be " + " | ".join(FRAMINGS))
        self.lang = lang
        self.log = log
        self.t0, self.t1, self.fps, self.speed = t0, t1, fps, speed
        self.framing = framing
        self.lay = Layout.for_size(*size, full=hud != "full")
        self.geom = VehicleGeom.from_params()
        self.warnings = list(log.warnings)
        if log.has_gps and self.profile == "hall":
            self.warnings.append("indoor hall scenery with GPS position hold: GPS needs open sky - no position hold "
                                 "indoors (params electronics.gps.accuracy_note); the hall is drawn because --scene hall")
        if lang == "ko" and korean_font_path() is None:
            self.warnings.append("lang ko: no Korean font found (Malgun Gothic / NanumGothic / Noto Sans CJK KR): "
                                 "Hangul will not render")
        # the video grid; preview / --at frames are evaluated on the same camera plan and trail
        self.frame_dt = speed / fps
        self.grid_t = frame_times(t0, t1, fps, speed)
        self.grid = resample(log, self.grid_t, window=self.frame_dt)
        self.tq = self.grid_t if tq is None else np.asarray(tq, dtype=float)
        self.fr = self.grid if tq is None else resample(log, self.tq, window=self.frame_dt)
        self.pushes = [p for p in find_pushes(log) if p.t1 >= t0 - 1e-9 and p.t0 <= t1 + 1e-9]
        self.wind_plot = wind_dir_varies(log)
        if azimuth_deg is not None:
            self.az_grid = np.full(self.grid.n, float(azimuth_deg))
        elif self.pushes:
            # each push seen from the side (it runs across the screen, the person stands beside the vehicle)
            self.az_grid = azimuth_track(self.grid.t, [(p.t0, p.t1, p.dir_deg) for p in self.pushes])
        else:
            self.az_grid = np.full(self.grid.n, auto_azimuth(log))
        self.az = float(self.az_grid[0]) % 360.0
        cam_kw: dict[str, Any] = dict(DEMO_CAMERA) if framing == "demo" else {}
        self.elevation = float(elevation_deg if elevation_deg is not None else cam_kw.pop("elevation_deg", 12.0))
        cam_kw.pop("elevation_deg", None)
        self.with_figure = figure == "on" or (figure == "auto" and (bool(self.pushes) or framing == "demo"))
        self.fig_grid = self.fig = None
        if self.with_figure:
            f_, r_, _ = camera_basis(self.az, self.elevation)
            fh = f_[:2] / max(float(np.hypot(*f_[:2])), 1e-9)
            rh = r_[:2] / max(float(np.hypot(*r_[:2])), 1e-9)
            idle_xy = FIGURE_IDLE_OFFSET[0] * rh + FIGURE_IDLE_OFFSET[1] * fh
            g = self.grid
            self.fig_grid = figure_track(g.t, g.pos, g.push_active, g.push_force, g.push_point, log.ground_z, idle_xy)
            if tq is None:
                self.fig = self.fig_grid
            else:
                fg = self.fig_grid
                yaw = np.unwrap(fg.yaw)
                self.fig = FigureTrack(feet=interp_linear(g.t, fg.feet, self.tq),
                                       yaw=np.interp(self.tq, g.t, yaw),
                                       pushing=(self.fr.push_active.copy() if self.fr.push_active is not None
                                                else np.zeros(self.fr.n, dtype=bool)),
                                       hands=interp_linear(g.t, fg.hands, self.tq))
        self.n_gps_rows = int(log.has_hold) + int(log.gps_ok is not None or log.gps_sats is not None
                                                  or log.gps_fix is not None)
        # v0.9 power / tether rows (which rows exist depends on the keys only, not on the values)
        self.n_power_rows = len(power_hud_rows(frame_power_sample(self.fr, 0, log), lang)) if self.fr.n else 0
        self.scene = Scene(log, self.fr, self.grid, self.geom, (self.lay.main_w, self.lay.H), self.lay.inset_px,
                           self.az, self.warnings, aa=aa, lang=lang, with_figure=self.with_figure,
                           with_push=bool(self.pushes), style=style, hud=hud, profile=self.profile,
                           asset_dir=asset_dir, with_gps_pod=self.with_gps_pod,
                           person_xy=self.fig_grid.feet[:, :2] if self.fig_grid is not None else None,
                           cam_az_all=self.az_grid)
        if self.with_figure:
            self.scene.set_person_track(self.grid.t, self.fig_grid)      # the rigged person's push blends / walking
        if hud != "none":
            self.scene.add_ring_labels(self.az)
        self.push_glow = self._push_glow_points(self.scene.body_r)
        sel = (log.t >= t0 - 1e-9) & (log.t <= t1 + 1e-9)
        self.drift = DriftInset(log.pos[sel, :2] if np.any(sel) else log.pos[:, :2], self.lay.drift_px, self.az,
                                self.lay.scale, lang=lang)
        w_all = log.wind[sel] if np.any(sel) else log.wind
        self.show_compass = hud == "full" and bool(np.max(np.hypot(w_all[:, 0], w_all[:, 1])) > 0.05)  # hidden when calm
        self.compass = WindCompass(int(round(200 * self.lay.scale)), float(np.max(np.hypot(w_all[:, 0], w_all[:, 1]))),
                                   self.lay.scale, lang=lang, smoothed=log.wind_mean_source != "log")
        extra = figure_points(self.fig_grid) if self.with_figure else None
        if framing == "all" and self.scene.tether is not None:
            # v0.9 tether: 'all' also keeps the rig in view (the pulley above the anchor and the stand's foot)
            tv = self.scene.tether
            rig = np.array([tv.anchor + np.array([0.0, 0.0, 0.15]),
                            np.array([*(tv.anchor[:2] + tv.boom_len * tv.u[:2]), log.ground_z])])
            rig_n = np.repeat(rig[None], self.grid.n, axis=0)
            extra = rig_n if extra is None else np.concatenate([extra, rig_n], axis=1)
        self.plan_grid = self._plan(framing, extra, cam_kw)
        plan = self.plan_grid
        self.cam = plan if tq is None else interp_plan(plan, self.grid_t, self.tq)
        if self.motion_blur > 1:
            # sub-frames over half a frame interval (a 180 deg shutter) around each frame time
            offs = (np.arange(self.motion_blur) + 0.5) / self.motion_blur - 0.5
            tsub = np.clip((self.tq[:, None] + 0.5 * self.frame_dt * offs[None, :]).ravel(), float(log.t[0]), float(log.t[-1]))
            self.fr_sub = resample(log, tsub, window=self.frame_dt)
            self.cam_sub = interp_plan(plan, self.grid_t, tsub)
        self.panel = None
        if hud == "full":
            self.panel = PlotPanel(log, t0, t1, self.lay.panel_w, self.lay.H, self.lay.scale,
                                   bottom_reserve_px=self.lay.H - self.inset_rects_px()["drift"][1] + int(10 * self.lay.scale)
                                   + self.push_table_h(), lang=lang, pushes=self.pushes, wind_dir_plot=self.wind_plot)
        self.summary = summarize(log, t0, t1)
        self.summary["gps"] = log.has_gps
        self.summary["gps_pod"] = self.scene.pod_source
        self.summary["station_keeping"] = station_keeping(log, t0, t1)
        self.summary["pushes"] = [{"i": p.i, "t0": p.t0, "t1": p.t1, "f_peak_n": p.f_peak_n, "impulse_ns": p.impulse_ns,
                                   "dir_deg": p.dir_deg, "peak_tilt_deg": p.peak_tilt_deg,
                                   "baseline_tilt_deg": p.baseline_tilt_deg, "recovery_s": p.recovery_s,
                                   "moved_m": p.moved_m} for p in self.pushes]

    def _plan(self, framing: str, extra: np.ndarray | None, cam_kw: dict[str, Any]) -> "CameraPlan | FreeCamPlan":
        """The camera plan on the video grid: spectator (fixed eye-height position, pan / tilt / zoom) or the chase
        camera (realistic: a 38 deg view and a calmer follow)."""
        log, aspect = self.log, self.lay.main_w / self.lay.H
        if self.camera == "spectator":
            return plan_spectator(self.grid, log.ground_z, self.fps / self.speed, self.scene.length_m,
                                  float(self.az_grid[0]), **SPECTATOR)
        kw = dict(cam_kw)
        if self.style == "realistic":
            kw.setdefault("view_angle_deg", 38.0)
            kw["follow_sigma_s"] = max(float(kw.get("follow_sigma_s", 0.12)), 0.3)
            kw["smooth_s"] = max(float(kw.get("smooth_s", 0.6)), 1.0)
        return plan_camera(self.grid, log.ground_z, self.fps / self.speed, aspect, self.scene.length_m, framing,
                           self.az_grid, self.elevation, avoid=self.overlay_rects_ndc(), extra_pts=extra, **kw)

    def _push_glow_points(self, body_r: float) -> list[tuple[float, np.ndarray]]:
        """(push end time, drawn contact point at the last active sample) for the 'pushed here' marker."""
        log = self.log
        out: list[tuple[float, np.ndarray]] = []
        for p in self.pushes:
            j = max(0, int(np.searchsorted(log.t, p.t1, side="left")) - 1)
            while j > 0 and not log.push_active[j]:
                j -= 1
            R = quat_to_matrix(log.q[j])
            out.append((p.t1, visual_contact(log.push_point[j], log.push_force[j], log.pos[j], R, body_r)))
        return out

    # overlay geometry (pixels, y down) shared by the composer and the camera planner
    TICKER_LINES = 4
    PUSH_TABLE_ROWS = 4

    def push_table_h(self) -> int:
        if not self.pushes:
            return 0
        s = self.lay.scale
        return int((28 + 19 * min(len(self.pushes), self.PUSH_TABLE_ROWS) + 10) * s)

    def compass_rect_px(self) -> tuple[int, int, int, int]:
        lay, s = self.lay, self.lay.scale
        y0 = int((84 if self.standin_tag() is not None else 50) * s)
        x1 = lay.main_w - int(16 * s)
        return (x1 - self.compass.W, y0, x1, y0 + self.compass.H)

    def overlay_rects_px(self) -> dict[str, tuple[int, int, int, int]]:
        """Boxes drawn over the 3-D view (the camera keeps the vehicle out of them)."""
        lay, s = self.lay, self.lay.scale
        m = int(16 * s)
        if self.hud == "none":
            return {"tag": (lay.main_w - int(420 * s), 0, lay.main_w, int(44 * s))}
        # v0.8 rows (hold point, GPS) and v0.9 rows (tether, supply, buffer, battery): the box grows with them
        extra = getattr(self, "n_gps_rows", 0) + getattr(self, "n_power_rows", 0)
        if self.hud == "minimal":
            return {"banner": (0, 0, lay.main_w, int(44 * s)),
                    "hud": (m, lay.H - m - int((150 + 20 * extra) * s), m + int(380 * s), lay.H - m)}
        th = int((self.TICKER_LINES * 19 + 30) * s)
        return {
            "banner": (0, 0, lay.main_w, int(44 * s)),
            "hud": (m, int(52 * s), m + int(430 * s), int(52 * s) + int((280 + 22 * extra) * s)),
            **({"compass": self.compass_rect_px()} if self.show_compass else {}),
            "ticker": (m, lay.H - m - th, lay.main_w - m, lay.H - m),
        }

    def inset_rects_px(self) -> dict[str, tuple[int, int, int, int]]:
        """The two insets side by side at the bottom of the side panel (canvas pixels)."""
        lay, s = self.lay, self.lay.scale
        hb, fb = int(40 * s), int(44 * s)
        gap = (lay.panel_w - lay.drift_px - lay.inset_px) // 3
        h_v = lay.inset_px + hb + fb
        h = max(h_v, self.drift.H)
        y0 = lay.H - int(14 * s) - h
        x_d = lay.main_w + gap
        x_v = x_d + lay.drift_px + gap
        return {"drift": (x_d, y0 + (h - self.drift.H), x_d + lay.drift_px, lay.H - int(14 * s)),
                "vanes": (x_v, y0 + (h - h_v), x_v + lay.inset_px, lay.H - int(14 * s))}

    def overlay_rects_ndc(self) -> list[NDC]:
        W, H = float(self.lay.main_w), float(self.lay.H)
        return [(x0 / W * 2.0 - 1.0, x1 / W * 2.0 - 1.0, 1.0 - y1 / H * 2.0, 1.0 - y0 / H * 2.0)
                for x0, y0, x1, y1 in self.overlay_rects_px().values()]

    # ---- one frame --------------------------------------------------------------------
    def figure_opacity(self, k: int, cam: CamView) -> float:
        if not self.with_figure:
            return 1.0
        fp = figure_points(FigureTrack(self.fig.feet[k:k + 1], self.fig.yaw[k:k + 1], self.fig.pushing[k:k + 1],
                                       self.fig.hands[k:k + 1]))[0]
        vp = self._veh_pts()[k]
        return 0.3 if figure_occludes(fp, vp, cam, self.lay.main_w, self.lay.H, self.scene.body_r) else 1.0

    def figure_label_visible(self, k: int, cam: CamView) -> bool:
        """The 'person 1.7 m' label only while the head is in the 3-D view and not under an overlay box."""
        if not self.with_figure:
            return False
        head = self.fig.feet[k] + np.array([0.0, 0.0, FIGURE_HEIGHT_M + 0.08])
        px, z = project(head[None, :], cam, self.lay.main_w, self.lay.H)
        if float(z[0]) <= 0.0:
            return False
        x, y = float(px[0, 0]), float(px[0, 1])
        if not (0.0 <= x <= self.lay.main_w and 0.0 <= y <= self.lay.H):
            return False
        return not any(x0 <= x <= x1 and y0 <= y <= y1 for x0, y0, x1, y1 in self.overlay_rects_px().values())

    def _veh_pts(self) -> np.ndarray:
        """vehicle_points of every rendered frame (computed once)."""
        vp = getattr(self, "_vp_cache", None)
        if vp is None:
            vp = self._vp_cache = vehicle_points(self.fr, self.scene.length_m)
        return vp

    def _glow(self, t: float) -> tuple[np.ndarray, float] | None:
        best = None
        for t_end, p in self.push_glow:
            if t_end <= t + 1e-9 and t - t_end < PUSH_AFTERGLOW_S:
                best = (p, t - t_end)
        return best

    def _render_main(self, k: int, cam: CamView) -> np.ndarray:
        """The 3-D view of frame k; with --motion-blur N the mean of N sub-frame renders over half a frame interval
        (the person / pole are posed at the frame time)."""
        lay, fr, log = self.lay, self.fr, self.log
        rects = list(self.overlay_rects_px().values())
        if self.with_figure or self.pushes:
            self.scene.update_push_figure(k, self.fig, self.figure_opacity(k, cam), self._glow(float(fr.t[k])),
                                          self.figure_label_visible(k, cam))
        if self.motion_blur <= 1:
            self.scene.update(k, cam, log.t_ref_n, (lay.main_w, lay.H), rects)
            return self.scene.render()
        acc = None
        n = self.motion_blur
        try:
            self.scene.fr = self.fr_sub
            self.scene.ghost_k = k
            for i in range(n):
                j = k * n + i
                self.scene.update(j, self.cam_sub.view(j), log.t_ref_n, (lay.main_w, lay.H), rects)
                img = self.scene.render().astype(np.float32)
                acc = img if acc is None else acc + img
        finally:
            self.scene.fr = self.fr
            self.scene.ghost_k = None
        return np.clip(acc / n + 0.5, 0, 255).astype(np.uint8)

    def frame(self, k: int) -> np.ndarray:
        from PIL import Image, ImageDraw
        lay, fr, log = self.lay, self.fr, self.log
        cam = self.cam.view(k)
        main = self._render_main(k, cam)
        if self.hud != "full":
            return self._frame_cinematic(k, cam, main)
        self.scene.inset.az = float(self.cam.azimuth_deg[k])          # the close-up turns with the main camera
        self.scene.inset.update(fr.q[k], fr.delta[k], float(fr.thrust[k]) / log.t_ref_n)
        vimg = self.scene.inset.render()
        canvas = Image.new("RGB", (lay.W, lay.H), SURFACE)
        canvas.paste(Image.fromarray(main), (0, 0))
        canvas.paste(Image.fromarray(self.panel.frame(float(fr.t[k]))), (lay.main_w, 0))
        ov = Image.new("RGBA", (lay.W, lay.H), (0, 0, 0, 0))
        d = ImageDraw.Draw(ov)
        s = lay.scale
        self._draw_marker(d, k, cam)
        self._draw_hud(d, k)
        self._draw_banner(d)
        if self.show_compass:
            self._draw_compass(ov, k)
        # insets (side panel)
        ir = self.inset_rects_px()
        hd = fr.hold_dist()
        dimg = self.drift.frame(self._xy_past(k), fr.wind[k], fr.wind[k] - fr.wind_steady[k], bool(fr.gust_active[k]),
                                wind_mean=self._wind_mean(k), pushes=self._push_markers(float(fr.t[k])),
                                cam_az=float(self.cam.azimuth_deg[k]),
                                hold=fr.hold_xy[k] if fr.hold_xy is not None else None,
                                hold_d=float(hd[k]) if hd is not None else None,
                                est=fr.est_rel[k] if fr.est_rel is not None else None)
        ov.alpha_composite(dimg, (ir["drift"][0], ir["drift"][1]))
        hb, fb = int(40 * s), int(44 * s)
        vx, vy = ir["vanes"][0], ir["vanes"][1]
        d.rounded_rectangle([vx, vy, vx + lay.inset_px - 1, vy + lay.inset_px + hb + fb - 1], radius=int(10 * s),
                            fill=(252, 252, 251, 255), outline=(120, 120, 120, 255), width=max(1, int(s)))
        ov.paste(Image.fromarray(vimg).convert("RGBA"), (vx, vy + hb))
        self._draw_vane_text(d, k, vx, vy, hb)
        self._draw_push_table(d, k)
        self._draw_events(d, k)
        out = Image.alpha_composite(canvas.convert("RGBA"), ov).convert("RGB")
        return np.asarray(out)

    def _frame_cinematic(self, k: int, cam: CamView, main: np.ndarray) -> np.ndarray:
        """--hud minimal: the 3-D view full frame with the title, the simulation tag and a compact HUD (time, phase,
        height, tilt, throttle, wind, push); --hud none: only the simulation tag."""
        from PIL import Image, ImageDraw
        lay, s, L = self.lay, self.lay.scale, self.lang
        canvas = Image.fromarray(main).convert("RGBA")
        ov = Image.new("RGBA", (lay.W, lay.H), (0, 0, 0, 0))
        d = ImageDraw.Draw(ov)
        if self.hud == "minimal":
            self._draw_marker(d, k, cam)
            self._draw_banner(d)
            fr, log = self.fr, self.log
            keep = {tr(x, L) for x in ("TIME", "PHASE", "STATE", "ALT", "TILT", "THR", "HOLD", "GPS", "WIND", "GUST",
                                       "PUSH", "TETHER", "SUPPLY", "BUFFER", "BATT")}
            push = fr.push_force[k] if (fr.push_active is not None and bool(fr.push_active[k])) else None
            sample = {"t": float(fr.t[k]), "speed": self.speed, "mode": log.mode,
                      "phase": log.phase_name(int(fr.phase[k]), L) if log.has_phase or log.mode == "vtol" else None,
                      "state": log.state_name(int(fr.state[k]), L), "state_code": int(fr.state[k]),
                      "alt": float(fr.pos[k, 2]), "h_target": float("nan"), "h_ref": float("nan"),
                      "vz": float(fr.vel[k, 2]), "tilt": float(fr.tilt_deg[k]), "throttle": float(fr.throttle[k]),
                      "u_max": log.u_max, "wind": fr.wind[k], "gust": fr.wind[k] - fr.wind_steady[k],
                      "gust_active": bool(fr.gust_active[k]), "last_cmd": None, "push": push,
                      **frame_gps_sample(fr, k, log), **frame_power_sample(fr, k, log)}
            rows = [r for r in hud_lines(sample, L) if r[0] in keep]
            if log.has_phase or log.mode == "vtol":
                rows = [r for r in rows if r[0] != tr("STATE", L)]
            fl, fv = self._f(14 * s, mono=True, bold=True), self._f(14 * s, mono=True)
            lh = int(20 * s)
            lab_w = int((70 if L == "en" else 84) * s)
            w = lab_w + int(max(d.textlength(v, font=fv) for _, v, _ in rows)) + int(22 * s)
            h = lh * len(rows) + int(14 * s)
            x0 = int(16 * s)
            y0 = lay.H - int(16 * s) - h
            d.rounded_rectangle([x0, y0, x0 + w, y0 + h], radius=int(8 * s), fill=(14, 20, 30, 150))
            for i, (lab, val, tone) in enumerate(rows):
                y = y0 + int(7 * s) + i * lh
                d.text((x0 + int(11 * s), y), lab, fill=TONES["muted"] + (255,), font=fl)
                d.text((x0 + lab_w, y), val, fill=TONES.get(tone, TONES["normal"]) + (255,), font=fv)
        else:
            self._draw_sim_tag(d)
        return np.asarray(Image.alpha_composite(canvas, ov).convert("RGB"))

    def _draw_sim_tag(self, d: Any) -> None:
        """The 'SIMULATION - not flight footage' tag (top right of the 3-D view), drawn in every HUD mode."""
        s, lay, L = self.lay.scale, self.lay, self.lang
        tag = tr("sim_tag", L)
        ft = self._f(14 * s, bold=True, text=tag)
        tw = int(d.textlength(tag, font=ft))
        xr = lay.main_w - int(16 * s)
        d.rounded_rectangle([xr - tw - int(24 * s), int(10 * s), xr, int(40 * s)], radius=int(6 * s), fill=(200, 40, 40, 200))
        d.text((xr - tw - int(12 * s), int(14 * s if L == "ko" else 16 * s)), tag, fill=(255, 255, 255, 255), font=ft)
        if self.standin_tag() is not None:
            w2 = self.standin_tag()
            tw2 = int(d.textlength(w2, font=ft))
            d.rounded_rectangle([xr - tw2 - int(24 * s), int(46 * s), xr, int(76 * s)], radius=int(6 * s),
                                fill=(230, 150, 20, 220))
            d.text((xr - tw2 - int(12 * s), int(50 * s if L == "ko" else 52 * s)), w2, fill=(20, 20, 20, 255), font=ft)

    def standin_tag(self) -> str | None:
        """The orange on-screen tag of stand-in geometry: missing vehicle / pad / vane STLs, else (v0.8) only the GPS
        pod's STL missing; None when every drawn part comes from the CAD export."""
        if any("drawn as a simple stand-in" in w for w in self.warnings):
            return tr("standin", self.lang)
        if any(w.startswith("GPS pod STL not found") for w in self.warnings):
            return tr("standin_pod", self.lang)
        return None

    def _wind_mean(self, k: int) -> np.ndarray:
        return self.fr.wind_mean[k] if self.fr.wind_mean is not None else self.fr.wind[k]

    def _push_markers(self, t: float) -> list[tuple[np.ndarray, np.ndarray, float]]:
        out = []
        for p in self.pushes:
            if p.t0 > t + 1e-9:
                continue
            j = int(np.clip(np.searchsorted(self.log.t, p.t0), 0, self.log.n - 1))
            fpk = self.log.push_force[j:max(j + 1, int(np.searchsorted(self.log.t, p.t1)))]
            F = fpk[int(np.argmax(np.linalg.norm(fpk, axis=1)))]
            out.append((p.point[:2], F[:2], -1.0 if t <= p.t1 + 1e-9 else t - p.t1))
        return out

    def _xy_past(self, k: int) -> np.ndarray:
        j = int(np.searchsorted(self.grid.t, self.fr.t[k] - 1e-9, side="left"))
        return np.vstack([self.grid.pos[:j, :2], self.fr.pos[k, :2][None, :]])

    def _draw_marker(self, d: Any, k: int, cam: CamView) -> None:
        """Screen-space brackets around the vehicle when it is small on screen."""
        lay = self.lay
        vp = self._veh_pts()[k]
        px, z = project(vp, cam, lay.main_w, lay.H)
        if float(z.min()) <= 0.0:
            return
        hpx = float(np.hypot(*(px[2] - px[0])))
        if hpx >= 70 * lay.scale:
            return
        c = px.mean(axis=0)
        r = max(18 * lay.scale, 0.75 * hpx)
        col = (20, 20, 20, 230)
        w = max(1, int(2 * lay.scale))
        L = 0.45 * r
        for sx in (-1, 1):
            for sy in (-1, 1):
                x0, y0 = c[0] + sx * r, c[1] + sy * r
                d.line([(x0, y0), (x0 - sx * L, y0)], fill=col, width=w)
                d.line([(x0, y0), (x0, y0 - sy * L)], fill=col, width=w)

    def _f(self, size: float, mono: bool = False, bold: bool = False, text: str = "") -> Any:
        return font(int(size), mono=mono, bold=bold, text=text, lang=self.lang)

    def _draw_hud(self, d: Any, k: int) -> None:
        fr, log, s, L = self.fr, self.log, self.lay.scale, self.lang
        t = float(fr.t[k])
        push = None
        if fr.push_active is not None and bool(fr.push_active[k]):
            push = fr.push_force[k]
        sample = {
            "t": t, "speed": self.speed, "mode": log.mode,
            "phase": log.phase_name(int(fr.phase[k]), L) if log.has_phase or log.mode == "vtol" else None,
            "state": log.state_name(int(fr.state[k]), L), "state_code": int(fr.state[k]), "alt": float(fr.pos[k, 2]),
            "h_target": float(fr.h_target[k]), "h_ref": float(fr.h_ref[k]), "vz": float(fr.vel[k, 2]),
            "tilt": float(fr.tilt_deg[k]), "throttle": float(fr.throttle[k]), "u_max": log.u_max, "wind": fr.wind[k],
            "gust": fr.wind[k] - fr.wind_steady[k], "gust_active": bool(fr.gust_active[k]),
            "last_cmd": last_command(log.cmd_t, log.cmd_name, t), "push": push,
            **frame_gps_sample(fr, k, log), **frame_power_sample(fr, k, log),
        }
        rows = hud_lines(sample, L)
        fl, fv = self._f(15 * s, mono=True, bold=True), self._f(15 * s, mono=True)
        lh = int(22 * s)
        x0, y0 = int(16 * s), int(52 * s)
        lab_w = int((84 if L == "en" else 92) * s)
        w = lab_w + int(max(d.textlength(v, font=fv) for _, v, _ in rows)) + int(24 * s)
        w = max(w, int(300 * s))
        h = lh * len(rows) + int(16 * s)
        d.rounded_rectangle([x0, y0, x0 + w, y0 + h], radius=int(8 * s), fill=(14, 20, 30, 178))
        for i, (lab, val, tone) in enumerate(rows):
            y = y0 + int(8 * s) + i * lh
            d.text((x0 + int(12 * s), y), lab, fill=TONES["muted"] + (255,), font=fl)
            d.text((x0 + lab_w, y), val, fill=TONES.get(tone, TONES["normal"]) + (255,), font=fv)

    def _draw_compass(self, ov: Any, k: int) -> None:
        g = self.grid
        t = float(self.fr.t[k])
        i0 = int(np.searchsorted(g.t, t - WIND_TRACE_S - 1e-9, side="left"))
        i1 = int(np.searchsorted(g.t, t - 1e-9, side="left"))
        gw = g.wind_mean if g.wind_mean is not None else g.wind
        trace = np.vstack([gw[i0:i1, :2], self._wind_mean(k)[None, :2]])
        img = self.compass.frame(self._wind_mean(k), self.fr.wind[k] - self.fr.wind_steady[k], bool(self.fr.gust_active[k]),
                                 trace)
        x0, y0, _, _ = self.compass_rect_px()
        ov.alpha_composite(img, (x0, y0))

    def _draw_banner(self, d: Any) -> None:
        log, s, lay, L = self.log, self.lay.scale, self.lay, self.lang
        m = log.meta
        title = str((m.get("title_ko") if L == "ko" else None) or m.get("title") or m.get("scenario")
                    or os.path.basename(log.source))
        sc = str(m.get("scenario", ""))
        full = (f"{sc}  |  {title}" if sc and sc not in title else title)
        f = self._f(16 * s, bold=True, text=full)
        max_w = lay.main_w - int(400 * s)
        txt = full
        cut = len(full)
        while d.textlength(txt, font=f) > max_w and cut > 8:
            cut -= 2
            txt = full[:cut].rstrip() + "..."
        d.rounded_rectangle([int(16 * s), int(10 * s), int(16 * s) + int(d.textlength(txt, font=f)) + int(24 * s), int(40 * s)],
                            radius=int(6 * s), fill=(14, 20, 30, 178))
        d.text((int(28 * s), int(15 * s)), txt, fill=(240, 242, 245, 255), font=f)
        self._draw_sim_tag(d)

    def _draw_vane_text(self, d: Any, k: int, vx: int, vy: int, hb: int) -> None:
        s, L = self.lay.scale, self.lang
        d.text((vx + int(10 * s), vy + int(6 * s)), tr("vanes_title", L), fill=(20, 20, 20, 255), font=self._f(12 * s, bold=True))
        d.text((vx + int(10 * s), vy + int(22 * s)), tr("vanes_sub", L), fill=(90, 90, 90, 255), font=self._f(11 * s))
        fl = font(int(14 * s), bold=True)
        for lab, x, y in self.scene.inset.labels_px():
            if 0 <= x < self.lay.inset_px and 0 <= y < self.lay.inset_px:
                shown = "합력" if (lab == "net" and L == "ko") else lab
                fl_ = self._f(14 * s, bold=True) if shown != lab else fl
                tw = d.textlength(shown, font=fl_)
                col = (17, 17, 17, 255) if lab == "net" else hex_rgb(VANE_COLORS[(int(lab) - 1) % 4]) + (255,)
                d.text((vx + x - tw / 2, vy + hb + y - 9 * s), shown, fill=col, font=fl_, stroke_width=max(1, int(2 * s)),
                       stroke_fill=(255, 255, 255, 230))
        dd = np.degrees(self.fr.delta[k])
        y0 = vy + hb + self.lay.inset_px + int(5 * s)
        unit = "°" if L == "ko" else " deg"
        for i in range(min(4, dd.size)):
            col = hex_rgb(VANE_COLORS[i]) + (255,)
            x = vx + int(10 * s) + (i % 2) * int(140 * s)
            y = y0 + (i // 2) * int(18 * s)
            d.text((x, y), f"d{i + 1} {dd[i]:+6.2f}{unit}", fill=col, font=self._f(13 * s, mono=True, bold=True))

    def _draw_push_table(self, d: Any, k: int) -> None:
        """Per-push response (side panel, between the plots and the insets): pushes started so far, newest last."""
        if not self.pushes:
            return
        s, L, lay = self.lay.scale, self.lang, self.lay
        t = float(self.fr.t[k])
        band = float((self.log.meta.get("criteria") or {}).get("gust_band_deg", 2.0))
        h = self.push_table_h()
        y1 = self.inset_rects_px()["drift"][1] - int(8 * s)
        y0 = y1 - h
        x0, x1 = lay.main_w + int(14 * s), lay.W - int(14 * s)
        d.rounded_rectangle([x0, y0, x1, y1], radius=int(8 * s), fill=(255, 247, 237, 255), outline=hex_rgb(PUSH_C) + (255,),
                            width=max(1, int(1.5 * s)))
        d.text((x0 + int(10 * s), y0 + int(6 * s)), tr("push_table", L, b=band), fill=hex_rgb(PUSH_C) + (255,),
               font=self._f(12 * s, bold=True))
        started = [p for p in self.pushes if p.t0 <= t + 1e-9][-self.PUSH_TABLE_ROWS:]
        fr_ = self._f(13 * s, mono=True)
        maxw = (x1 - x0) - int(20 * s)
        for i, p in enumerate(started):
            line = push_row_text(p, L, t)
            while d.textlength(line, font=fr_) > maxw and len(line) > 12:
                line = line[:-4] + "..."
            active = p.t0 <= t <= p.t1 + 1e-9
            col = hex_rgb(PUSH_C) if active else (40, 40, 40)
            d.text((x0 + int(10 * s), y0 + int(28 * s) + i * int(19 * s)), line, fill=col + (255,), font=fr_)

    def _draw_events(self, d: Any, k: int) -> None:
        """Ticker at the bottom of the 3-D view: the last events and ground commands up to this frame (Korean: a short
        reading in front of the logged English text)."""
        s, log, L = self.lay.scale, self.log, self.lang
        t = float(self.fr.t[k])
        x0, y0, x1, y1 = self.overlay_rects_px()["ticker"]
        d.rounded_rectangle([x0, y0, x1, y1], radius=int(8 * s), fill=(14, 20, 30, 170))
        d.text((x0 + int(12 * s), y0 + int(6 * s)), tr("events", L), fill=TONES["muted"] + (255,), font=self._f(12 * s, bold=True))
        evs = [(et, ex, "ev") for et, ex in recent_events(log.event_t, log.event_text, t, 8)]
        # ground commands, unless the event list already reports that command at that time
        for ct, cn in zip(log.cmd_t, log.cmd_name):
            if ct > t + 1e-9:
                break
            dup = any(abs(et - ct) <= 0.05 and cn in ex for et, ex in zip(log.event_t, log.event_text))
            if not dup:
                evs.append((float(ct), tr("ground_cmd", L, c=cmd_label(cn, L)), "cmd"))
        for p in self.pushes:                                    # derived from push_active (renderer)
            if p.t0 <= t + 1e-9:
                evs.append((p.t0, tr("push_ev", L, i=p.i, f=p.f_peak_n), "push"))
        evs.sort(key=lambda e: e[0])
        evs = evs[-self.TICKER_LINES:]
        f = self._f(14 * s, mono=True)
        fg = self._f(14 * s, bold=True)
        maxw = (x1 - x0) - int(24 * s)
        for i, (et, ex, kind) in enumerate(evs):
            age = t - et
            if kind == "cmd":
                col = TONES["accent"]
            elif kind == "push":
                col = (255, 160, 90)
            elif "ABORT" in ex or "FAULT" in ex:
                col = TONES["alert"]
            else:
                col = TONES["normal"] if age < 3.0 else TONES["muted"]
            y = y0 + int(26 * s) + i * int(19 * s)
            x = x0 + int(12 * s)
            head = f"{et:7.2f} s  "
            d.text((x, y), head, fill=col + (255,), font=f)
            x += int(d.textlength(head, font=f))
            gl = event_gloss_ko(ex) if (L == "ko" and kind == "ev") else ""
            if gl:
                d.text((x, y), gl, fill=col + (255,), font=fg)
                x += int(d.textlength(gl + "   ", font=fg))
                col2 = TONES["muted"]
            else:
                col2 = col
            line = ex
            room = x0 + maxw - x
            while d.textlength(line, font=f) > room and len(line) > 6:
                line = line[:-4] + "..."
            if room > 30 * s:
                d.text((x, y), line, fill=col2 + (255,), font=f)

    # ---- cards ------------------------------------------------------------------------
    def _card(self, lines: list[tuple[str, int, tuple[int, int, int], bool]], bg: tuple[int, int, int] = (16, 24, 38),
              behind: np.ndarray | None = None) -> np.ndarray:
        """Text card; ``behind`` (a rendered frame) shows through at 12 %."""
        from PIL import Image, ImageDraw
        lay = self.lay
        if behind is not None and behind.shape[:2] == (lay.H, lay.W):
            base = (0.12 * behind.astype(np.float32) + 0.88 * np.array(bg, dtype=np.float32)).astype(np.uint8)
            im = Image.fromarray(base)
        else:
            im = Image.new("RGB", (lay.W, lay.H), bg)
        d = ImageDraw.Draw(im)
        y = int(lay.H * 0.10)
        x = int(lay.W * 0.08)
        max_w = lay.W - 2 * x
        for txt, size, col, bold in lines:
            f = self._f(size * lay.scale, bold=bold, text=txt)
            for ln in wrap_px(d, txt, f, max_w):
                if y > lay.H - int(40 * lay.scale):
                    break
                d.text((x, y), ln, fill=col, font=f)
                y += int(size * lay.scale * 1.45)
        return np.asarray(im)

    def title_card(self, behind: np.ndarray | None = None) -> np.ndarray:
        log, m, L = self.log, self.log.meta, self.lang
        Ls: list[tuple[str, int, tuple[int, int, int], bool]] = []
        if L == "ko":
            Ls.append(("CanSat EDF 기체 - 비행 시뮬레이션", 26, (150, 190, 255), True))
            Ls.append((str(m.get("title_ko") or m.get("title") or m.get("scenario") or "시뮬레이션 로그"), 40,
                       (255, 255, 255), True))
            mode = tr(f"mode_{log.mode}", L) if f"mode_{log.mode}" in TXT else log.mode
            Ls.append((f"시나리오 {m.get('scenario', '?')}   모드 {mode}   시드 {m.get('seed', '?')}   "
                       f"params sha256 {str(m.get('params_sha256', '?'))[:12]}", 20, (200, 208, 220), False))
            Ls.append((f"로그 {os.path.basename(log.source)}  sha256 {log.sha256[:12]}   시뮬레이션 시간 {self.t0:.2f} - "
                       f"{self.t1:.2f} s, x{self.speed:g} 배속, {self.fps:g} fps", 18, (160, 170, 185), False))
            if not log.hw.legacy:
                Ls.append((f"기체: {log.hw.label(L)}   형상 {_rel(self.scene.stl_dir).replace(os.sep, '/')}", 20,
                           (150, 215, 255), False))
                if self.scene.tether is not None and self.scene.tether.assumed:
                    Ls.append(("테더 키 없음 - 가정값으로 그림: " + ", ".join(self.scene.tether.assumed_ko), 17,
                               (255, 205, 120), False))
            Ls.append(("", 14, (0, 0, 0), False))
            ko_notes = m.get("notes_ko")
            items = wrap(str(ko_notes), 70)[:10] if ko_notes else ["· " + x for x in notes_ko_lines(m.get("notes", ""), log.mode,
                                                                                                     gps=log.has_gps)]
            if log.has_gps:
                items = items + ["· " + x for x in gps_caveats("ko")]
            if items:
                Ls.append(("모델의 한계와 가정", 21, (255, 205, 120), True))
            for ln in items:
                Ls.append((ln, 20, (225, 230, 238), False))
            if m.get("notes"):
                Ls.append(("", 8, (0, 0, 0), False))
                for ln in wrap("notes (log): " + str(m.get("notes", "")), 150)[:5]:
                    Ls.append((ln, 14, (150, 160, 175), False))
            Ls.append(("", 14, (0, 0, 0), False))
            Ls.append(("시뮬레이션 - 실제 비행 영상 아님. 화면의 모든 값은 시뮬레이션 로그에서 읽은 값입니다.", 20,
                       (255, 140, 130), True))
            return self._card(Ls, behind=behind)
        Ls.append(("CanSat EDF vehicle - flight simulation", 26, (150, 190, 255), True))
        Ls.append((str(m.get("title") or m.get("scenario") or "simulation log"), 40, (255, 255, 255), True))
        info = (f"scenario {m.get('scenario', '?')}   mode {log.mode}   seed {m.get('seed', '?')}   "
                f"params sha256 {str(m.get('params_sha256', '?'))[:12]}")
        Ls.append((info, 20, (200, 208, 220), False))
        Ls.append((f"log {os.path.basename(log.source)}  sha256 {log.sha256[:12]}   sim t {self.t0:.2f} - {self.t1:.2f} s, "
                   f"x{self.speed:g} speed, {self.fps:g} fps", 18, (160, 170, 185), False))
        if not log.hw.legacy:
            Ls.append((f"vehicle: {log.hw.label(L)}   meshes {_rel(self.scene.stl_dir).replace(os.sep, '/')}", 20,
                       (150, 215, 255), False))
            if self.scene.tether is not None and self.scene.tether.assumed:
                Ls.append(("tether keys missing - drawn with ASSUMED " + ", ".join(self.scene.tether.assumed), 17,
                           (255, 205, 120), False))
        Ls.append(("", 14, (0, 0, 0), False))
        for ln in wrap(str(m.get("notes", "")), 110)[:12]:
            Ls.append((ln, 19, (225, 230, 238), False))
        if log.has_gps:
            for ln in gps_caveats("en"):
                Ls.append(("- " + ln, 19, (255, 205, 120), False))
        Ls.append(("", 14, (0, 0, 0), False))
        Ls.append(("SIMULATION - not flight footage. Everything shown is read from the log.", 20, (255, 140, 130), True))
        return self._card(Ls, behind=behind)

    def end_card(self, behind: np.ndarray | None = None) -> np.ndarray:
        log, sm, L = self.log, self.summary, self.lang
        Ls: list[tuple[str, int, tuple[int, int, int], bool]] = []
        sat_col = (255, 205, 120) if sm["throttle_saturated_s"] > 0 else (220, 228, 238)
        pct = (100.0 * sm["throttle_saturated_s"] / sm["powered_s"]) if sm["powered_s"] > 0 else 0.0
        if L == "ko":
            Ls.append(("요약 (렌더링 구간의 시뮬레이션 참값)", 30, (255, 255, 255), True))
            Ls.append((f"시간 {sm['t0']:.2f} - {sm['t1']:.2f} s     최고 높이 {sm['max_height_m']:.2f} m     "
                       f"최대 기울기 {sm['max_tilt_deg']:.1f}°", 22, (220, 228, 238), False))
            Ls.append((f"패드로부터 수평 이동: 최종 {sm['final_drift_m']:.2f} m, 최대 {sm['max_drift_m']:.2f} m", 22,
                       (220, 228, 238), False))
            Ls.append((f"동력 비행 {sm['powered_s']:.1f} s, 스로틀 최대(u_max) {sm['throttle_saturated_s']:.1f} s "
                       f"(동력 시간의 {pct:.0f} %)", 22, sat_col, False))
            st = log.state_name(sm["final_state_code"], L) if sm.get("final_state_code") is not None else sm["final_state"]
            fin = f"최종 상태 {st}"
            if sm.get("final_phase_code") is not None:
                fin += f", 단계 {log.phase_name(sm['final_phase_code'], L)}"
            Ls.append((fin + f", 속도 {sm['final_speed_mps']:.2f} m/s", 22, (220, 228, 238), False))
            if log.has_gps:
                for ln in station_keeping_lines(sm.get("station_keeping"), L):
                    Ls.append((ln, 20, (150, 215, 255), False))
                Ls.append((gps_caveats(L)[0] + ".  " + gps_caveats(L)[1] + ".", 17, (255, 205, 120), False))
            elif log.mode == "vtol":
                Ls.append(("수평 위치 센서가 없음: 기체는 제자리를 유지할 수 없고, 바람에 의한 이동은 보정되지 않은 채 "
                           "그대로 보입니다.", 19, (255, 205, 120), False))
            for p in self.pushes:
                Ls.append(("밀기 " + push_row_text(p, L), 19, (255, 170, 110), False))
            for ln in self.tether_lines(L):
                Ls.append((ln, 19, (150, 215, 255), False))
            Ls.append(("", 10, (0, 0, 0), False))
            if sm["commands"]:
                Ls.append(("지상 명령: " + ", ".join(f"{cmd_label(n, L)} @ {t:.1f} s" for t, n in sm["commands"][:12]), 18,
                           (160, 200, 255), False))
            Ls.append(("이벤트:", 18, (200, 208, 220), True))
            for t, e in sm["events"][-8:]:
                g = event_gloss_ko(e)
                Ls.append((f"  {t:8.2f} s  " + (f"{g}  |  " if g else "") + e[:110], 16, (200, 208, 220), False))
            Ls.append(("시뮬레이션 - 실제 비행 영상 아님.", 18, (255, 140, 130), True))
            return self._card(Ls, behind=behind)
        Ls.append(("Summary (sim truth over the rendered interval)", 30, (255, 255, 255), True))
        Ls.append((f"t {sm['t0']:.2f} - {sm['t1']:.2f} s     max height {sm['max_height_m']:.2f} m     "
                   f"max tilt {sm['max_tilt_deg']:.1f} deg", 22, (220, 228, 238), False))
        Ls.append((f"horizontal drift from the pad: final {sm['final_drift_m']:.2f} m, max {sm['max_drift_m']:.2f} m", 22,
                   (220, 228, 238), False))
        Ls.append((f"powered {sm['powered_s']:.1f} s, throttle at u_max {sm['throttle_saturated_s']:.1f} s "
                   f"({pct:.0f} % of powered time)", 22, sat_col, False))
        fin = f"final state {sm['final_state']}" + (f", phase {sm['final_phase']}" if sm["final_phase"] else "")
        Ls.append((fin + f", speed {sm['final_speed_mps']:.2f} m/s", 22, (220, 228, 238), False))
        if log.has_gps:
            for ln in station_keeping_lines(sm.get("station_keeping"), L):
                Ls.append((ln, 20, (150, 215, 255), False))
            Ls.append((gps_caveats(L)[0] + ".  " + gps_caveats(L)[1] + ".", 17, (255, 205, 120), False))
        elif log.mode == "vtol":
            Ls.append(("No horizontal position sensing: the vehicle cannot hold position, wind drift is shown, not corrected.",
                       19, (255, 205, 120), False))
        for p in self.pushes:
            Ls.append(("push " + push_row_text(p, L), 19, (255, 170, 110), False))
        for ln in self.tether_lines(L):
            Ls.append((ln, 19, (150, 215, 255), False))
        Ls.append(("", 10, (0, 0, 0), False))
        if sm["commands"]:
            Ls.append(("ground commands: " + ", ".join(f"{n} @ {t:.1f} s" for t, n in sm["commands"][:14]), 18,
                       (160, 200, 255), False))
        Ls.append(("events:", 18, (200, 208, 220), True))
        for t, e in sm["events"][-10:]:
            Ls.append((f"  {t:8.2f} s  {e}"[:150], 16, (200, 208, 220), False))
        notes = str(log.meta.get("notes", ""))
        if notes:
            Ls.append(("", 8, (0, 0, 0), False))
            for ln in wrap("notes: " + notes, 120)[:5]:
                Ls.append((ln, 16, (170, 180, 195), False))
        Ls.append(("SIMULATION - not flight footage.", 18, (255, 140, 130), True))
        return self._card(Ls, behind=behind)

    def tether_lines(self, L: str) -> list[str]:
        """End-card lines of a tether log (none otherwise): the tension range and the disconnections, from the log."""
        tv = self.scene.tether
        if tv is None:
            return []
        log, out = self.log, []
        sel = (log.t >= self.t0 - 1e-9) & (log.t <= self.t1 + 1e-9)
        if log.tether_tension is not None and np.any(np.isfinite(log.tether_tension[sel])):
            tt = log.tether_tension[sel]
            out.append((f"테더 장력 {np.nanmin(tt):.1f} - {np.nanmax(tt):.1f} N (기체 쪽)" if L == "ko" else
                        f"tether tension {np.nanmin(tt):.1f} - {np.nanmax(tt):.1f} N (vehicle end)"))
        for t0, _, _ in tv.free:
            if self.t0 - 1e-9 <= t0 <= self.t1 + 1e-9:
                out.append(f"테더 분리 {t0:.2f} s (떨어지는 줄은 그림용 사슬 계산)" if L == "ko" else
                           f"tether disconnected at {t0:.2f} s (the falling cable: an illustrative chain model)")
        return out

    def poster_index(self, poster_t: float | None) -> int:
        fr = self.fr
        if poster_t is None:
            pt = self.log.meta.get("poster_t")
            poster_t = float(pt) if pt is not None else None
        if poster_t is None:
            # the strongest push (person, pole and arrow on screen), a gust (wind + correction on screen), else the
            # middle of the first HOLD, else the frame where the vanes work hardest while powered, else the highest
            # powered frame
            if self.pushes:
                p = max(self.pushes, key=lambda q: q.f_peak_n)
                return int(np.clip(np.searchsorted(fr.t, 0.5 * (p.t0 + p.t1)), 0, fr.n - 1))
            if np.any(fr.gust_active):
                idx = np.flatnonzero(fr.gust_active)
                return int(idx[idx.size // 2])
            hold = np.flatnonzero(fr.phase == 3)
            if hold.size:
                end = hold[0] + int(np.argmax(np.diff(np.append(hold, hold[-1] + 2)) > 1))
                return int((hold[0] + end) // 2)
            powered = np.flatnonzero(fr.state == STATE_POWERED)
            if powered.size:
                # net side force of the vanes (pitch / yaw correction, not the symmetric roll trim)
                E = np.array([e_theta(th) for th in self.geom.vane_thetas_rad])          # (4, 3)
                work = np.linalg.norm(fr.delta[powered][:, :E.shape[0]] @ E, axis=1)
                if float(work.max()) > math.radians(1.0):
                    return int(powered[int(np.argmax(work))])
                return int(powered[int(np.argmax(fr.pos[powered, 2]))])
            return fr.n // 2
        return int(np.clip(np.searchsorted(fr.t, poster_t), 0, fr.n - 1))

    def close(self) -> None:
        self.scene.close()


# --------------------------------------------------------------------------- #
# output
# --------------------------------------------------------------------------- #
def write_mp4(path: str, frames: Callable[[], Any], size: tuple[int, int], fps: float, crf: int = 20) -> int:
    """H.264 / yuv420p via the imageio-ffmpeg binary; returns the number of frames written."""
    import imageio_ffmpeg
    os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
    w = imageio_ffmpeg.write_frames(path, size, fps=fps, codec="libx264", pix_fmt_in="rgb24", pix_fmt_out="yuv420p",
                                    quality=None, bitrate=None, macro_block_size=1,
                                    output_params=["-crf", str(crf), "-preset", "medium", "-movflags", "+faststart"])
    w.send(None)
    n = 0
    try:
        for img in frames():
            w.send(np.ascontiguousarray(img, dtype=np.uint8))
            n += 1
    finally:
        w.close()
    return n


def parse_size(s: str) -> tuple[int, int]:
    a, b = s.lower().split("x")
    W, H = int(a), int(b)
    if W < 320 or H < 180:
        raise ValueError("size too small (min 320x180)")
    return W // 2 * 2, H // 2 * 2


def render(log_path: str, out: str, fps: float = 30.0, size: tuple[int, int] = (1920, 1080), speed: float = 1.0,
           start: float | None = None, end: float | None = None, preview: int = 0, at: Sequence[float] | None = None,
           title_s: float = 3.0, end_s: float = 4.0, framing: str = "all", azimuth: float | None = None,
           elevation: float | None = None, poster_t: float | None = None, crf: int = 20, aa: str = "msaa",
           quiet: bool = False, lang: str = "ko", figure: str = "auto", style: str = "realistic", hud: str = "full",
           camera: str = "chase", motion_blur: int = 1, scene: str = "auto", asset_dir: str | None = ASSET_DIR,
           gps_pod: str = "auto") -> dict[str, Any]:
    """Render the video (or preview PNGs); returns the sidecar dict (also written as <out>.json).
    ``elevation`` None: 12 deg, or the demo preset's 8 deg with ``framing='demo'``.  ``style`` realistic (default:
    PBR, procedural sky + IBL, sun shadow, field / hall scenery, soft jet, mannequin) or schematic (the v2 look)."""
    t_wall0 = time.perf_counter()
    log = load_log(log_path)
    t0 = float(log.t[0]) if start is None else max(float(log.t[0]), float(start))
    t1 = float(log.t[-1]) if end is None else min(float(log.t[-1]), float(end))
    if t1 <= t0:
        raise ValueError(f"empty interval [{t0}, {t1}]")
    stem, ext = os.path.splitext(out)
    info: dict[str, Any] = {"log": os.path.abspath(log_path), "log_sha256": log.sha256, "meta": log.meta,
                            "fps": fps, "speed": speed, "size": list(size), "t0": t0, "t1": t1, "framing": framing,
                            "lang": lang, "style": style, "hud": hud, "camera": camera, "motion_blur": int(motion_blur)}
    tq = None
    if at:
        tq = np.clip(np.asarray(sorted(at), dtype=float), t0, t1)
    elif preview:
        tq = np.linspace(t0, t1, int(preview))
    rnd = Renderer(log, t0, t1, fps, speed, size, framing, azimuth, elevation, aa=aa, tq=tq, lang=lang, figure=figure,
                   style=style, hud=hud, camera=camera, motion_blur=motion_blur, scene=scene, asset_dir=asset_dir,
                   gps_pod=gps_pod)
    t_setup = time.perf_counter() - t_wall0
    info.update({"camera_azimuth_deg": rnd.az, "camera_azimuth_range_deg": [round(float(rnd.az_grid.min()), 2),
                                                                            round(float(rnd.az_grid.max()), 2)],
                 "camera_elevation_deg": rnd.elevation, "setup_s": round(t_setup, 3),
                 "figure": rnd.with_figure, "wind_dir_plot": rnd.wind_plot, "wind_mean_source": log.wind_mean_source,
                 "scene": rnd.profile, "gps_pod": rnd.scene.pod_source, "summary": rnd.summary})
    if not log.hw.legacy:                         # v6 keys only for a log that names its hardware (legacy sidecars unchanged)
        info["hardware"] = {"config": log.hw.config, "power_source": log.hw.power, "source": log.hw.source,
                            "stl_dir": _rel(rnd.scene.stl_dir), "pad": rnd.scene.pad_mesh is not None,
                            "top_items": [k for k, _ in rnd.scene.hw_meshes],
                            "interior": [k for k, _ in rnd.scene.interior], "report": _rel(hardware_report_path(log.hw))}
        if rnd.scene.tether is not None:
            info["tether"] = rnd.scene.tether.summary()
    files: list[str] = []
    try:
        if tq is not None:
            outdir = stem + "_frames"
            os.makedirs(outdir, exist_ok=True)
            from PIL import Image
            t_r = time.perf_counter()
            for k in range(rnd.fr.n):
                p = os.path.join(outdir, f"frame_{k:03d}_t{rnd.fr.t[k]:08.3f}.png")
                Image.fromarray(rnd.frame(k)).save(p)
                files.append(os.path.abspath(p))
            dt = time.perf_counter() - t_r
            info["frames"] = rnd.fr.n
            info["render_s"] = round(dt, 3)
            info["s_per_frame"] = round(dt / max(1, rnd.fr.n), 4)
        else:
            from PIL import Image
            n_title = int(round(title_s * fps))
            n_end = int(round(end_s * fps))
            n_body = rnd.fr.n
            t_r = time.perf_counter()
            timing = {"body_s": 0.0}

            k_p = rnd.poster_index(poster_t)
            poster = rnd.frame(k_p)                     # every frame is a pure function of k: order does not matter
            last: dict[str, np.ndarray] = {}

            def gen():
                if n_title:
                    tc = rnd.title_card(behind=poster)
                    for _ in range(n_title):
                        yield tc
                tb = time.perf_counter()
                for k in range(n_body):
                    img = rnd.frame(k)
                    if not quiet and (k % max(1, int(fps * 5)) == 0):
                        el = time.perf_counter() - tb
                        print(f"  frame {k}/{n_body}  t = {rnd.fr.t[k]:.2f} s  {el / max(1, k):.3f} s/frame", flush=True)
                    last["img"] = img
                    yield img
                timing["body_s"] = time.perf_counter() - tb
                if n_end:
                    ec = rnd.end_card(behind=last.get("img"))
                    for _ in range(n_end):
                        yield ec
            n = write_mp4(out, gen, (rnd.lay.W, rnd.lay.H), fps, crf)
            dt = time.perf_counter() - t_r
            files.append(os.path.abspath(out))
            pim = Image.fromarray(poster)
            pp = stem + "_poster.png"
            pim.save(pp)
            files.append(os.path.abspath(pp))
            info.update({"frames": n, "body_frames": n_body, "title_frames": n_title, "end_frames": n_end,
                         "video_s": round(n / fps, 3), "render_s": round(dt, 3),
                         "s_per_frame": round(timing["body_s"] / max(1, n_body), 4), "poster_t": float(rnd.fr.t[k_p])})
    finally:
        rnd.close()
    info["warnings"] = rnd.warnings
    info["files"] = files
    info["wall_s"] = round(time.perf_counter() - t_wall0, 3)
    side = stem + ("_frames.json" if tq is not None else ".json")
    with open(side, "w", encoding="utf-8") as f:
        json.dump(info, f, indent=1, default=lambda o: o.tolist() if isinstance(o, np.ndarray) else str(o))
    info["sidecar"] = side
    return info


# --------------------------------------------------------------------------- #
# development log from an existing (ballistic, v0.6.3) scenario
# --------------------------------------------------------------------------- #
def export_dev_log(scenario: str, out_path: str, sim_root: str = PROJECT_ROOT, t_end: float | None = None) -> dict[str, Any]:
    """Run S1 or S2 of ``sim/run_scenarios.py`` with ``sim.sim6dof.FlightSim`` and write the contract keys.

    Mapping: pos = the logged CG minus R(q) (0, 0, z_cg) (after the nose ejection the post-ejection
    CG, as the sim switches it); phase 0 (NONE), h_target / h_ref NaN (no VTOL controller);
    gust_active from the scenario's gust window; ground_z = -(pad plate + legs + feet) (the ground the
    pad stands on; the ballistic sim lands on the pad-top level, z = 0, so a landed vehicle sits
    0.11 m above that ground - noted in meta.notes); cmd_t = the first transmission of each ground
    command record, event_t / event_text = ``SimResult.events``."""
    if sim_root not in sys.path:
        sys.path.insert(0, sim_root)
    from dataclasses import replace
    from sim import run_scenarios as rs
    from sim.controller import gains_from_control
    from sim.sim6dof import FlightSim, SensorConfig
    from sim.vehicle import load_vehicle

    veh = load_vehicle()
    p = veh.params
    gains = gains_from_control(p["control"])
    gpath = os.path.join(sim_root, "sim", "out", "gains.json")
    g_src = "params.control"
    if os.path.exists(gpath):
        with open(gpath, "r", encoding="utf-8") as f:
            gj = json.load(f)
        gains.update({k: v for k, v in gj.items() if k in gains and k not in ("loop_hz", "imu_hz", "baro_hz")})
        g_src = "sim/out/gains.json"
    sensors = SensorConfig()
    if scenario == "S1":
        sc = rs.scenario_s1(sensors, gains=gains)
    elif scenario == "S2":
        sc = rs.scenario_s2(veh, sensors)
    else:
        raise ValueError("dev scenarios: S1 | S2")
    if t_end is not None:
        sc = replace(sc, t_end=float(t_end))
    res = FlightSim(veh, sc, gains).run()
    L = res.log
    n = L["t"].shape[0]
    R = quat_to_matrix(L["q"])
    z_full = float(veh.z_cg_m)
    z_post = float(veh.post_ejection.com_m[2])
    t_ej = next((t for t, e in res.events if e.startswith("ejection")), None)
    zc = np.where(L["t"] >= (t_ej if t_ej is not None else np.inf), z_post, z_full)
    pos = L["pos"] - R[:, :, 2] * zc[:, None]
    t_launch = 0.0 if sc.bypass_state_machine else (res.link or {}).get("t_launch")
    gust = np.zeros(n, dtype=bool)
    if sc.gust_mps and t_launch is not None:
        tg = t_launch + sc.gust_t0_after_launch_s
        gust = (L["t"] >= tg) & (L["t"] < tg + sc.gust_duration_s)
    recs = (res.link or {}).get("records", [])
    cmds = sorted(((r.get("t_first_tx") if r.get("t_first_tx") is not None else r["t_queued"]), r["name"]) for r in recs)
    lp = p["launch_pad"]
    ground_z = -(float(lp["plate_t"]) + float(lp["leg_h"]) + float(lp["foot_t"])) * 1e-3
    with open(os.path.join(sim_root, "params.json"), "rb") as f:
        psha = hashlib.sha256(f.read()).hexdigest()
    notes = (f"DEVELOPMENT LOG for the renderer, not a VTOL run: v0.6.3 ballistic scenario {sc.name} "
             f"(sim.sim6dof.FlightSim, sim/run_scenarios.scenario_{sc.name.lower()}, gains {g_src}), mapped onto the "
             f"v0.7 log contract: phase NONE and no altitude target (no VTOL controller in this sim). ")
    if scenario == "S2":
        notes += ("The ballistic sim lands on the pad-top level (z = 0) while the ground is drawn where the pad's feet "
                  "stand (-0.11 m), so the landed vehicle floats 0.11 m; the nose ejection and parachute are not drawn.")
    else:
        notes += "S1 bypasses the supervisor: fixed throttle from 20 m, 10 deg initial tilt, no ground link."
    meta = {"scenario": f"dev_{sc.name}", "title": sc.title, "seed": sc.seed, "mode": "ballistic",
            "params_sha256": psha, "notes": notes, "pos_ref": "exit_plane", "z_cg_m": z_full, "t_max_n": veh.t_max_n,
            "limits": {"u_max": 1.0, "tilt_abort_deg": float(p["mission"]["tilt_abort_deg"]),
                       "hover_throttle": float(veh.hover_throttle)},
            "source": f"tools/sim_video.py --export-dev {scenario}", "sim_runtime_s": round(res.runtime_s, 2),
            "t_liftoff": res.t_liftoff, "t_touchdown": res.t_touchdown}
    data = {
        "t": L["t"], "pos": pos, "vel": L["vel"], "q": L["q"], "delta": L["delta"], "throttle": L["throttle"],
        "thrust": L["thrust"], "state": L["state"].astype(np.int8), "phase": np.zeros(n, dtype=np.int8),
        "h_target": np.full(n, np.nan), "h_ref": np.full(n, np.nan), "h_est": L["h_est"], "tilt_deg": L["tilt_deg"],
        "wind": L["wind"], "gust_active": gust, "ground_z": np.array(ground_z),
        "event_t": np.array([t for t, _ in res.events], dtype=float), "event_text": np.array([e for _, e in res.events], dtype=str),
        "cmd_t": np.array([t for t, _ in cmds], dtype=float), "cmd_name": np.array([c for _, c in cmds], dtype=str),
        "meta": np.array(json.dumps(meta)),
    }
    if sc.bypass_state_machine:
        data["h_est"] = np.full(n, np.nan)       # bypass: the altitude filter is not exercised
    os.makedirs(os.path.dirname(os.path.abspath(out_path)), exist_ok=True)
    np.savez_compressed(out_path, **data)
    return {"path": os.path.abspath(out_path), "samples": n, "t_end": float(L["t"][-1]), "runtime_s": res.runtime_s,
            "events": res.events, "meta": meta}


# --------------------------------------------------------------------------- #
# CLI
# --------------------------------------------------------------------------- #
def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    ap.add_argument("--log", help="log .npz (contract in the module docstring)")
    ap.add_argument("--out", help="output .mp4 (poster: <stem>_poster.png, sidecar: <stem>.json)")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--size", default="1920x1080")
    ap.add_argument("--speed", type=float, default=1.0, help="sim seconds per video second")
    ap.add_argument("--start", type=float, default=None, help="sim time of the first frame")
    ap.add_argument("--end", type=float, default=None, help="sim time of the last frame")
    ap.add_argument("--preview", type=int, default=0, help="render N evenly spaced PNG frames instead of the video")
    ap.add_argument("--at", default="", help="comma-separated sim times: PNG frames at these times instead of the video")
    ap.add_argument("--title-s", type=float, default=3.0)
    ap.add_argument("--end-s", type=float, default=4.0)
    ap.add_argument("--framing", choices=FRAMINGS, default="all",
                    help="chase camera keeps vehicle + ground + pad (all), vehicle + ground, or the vehicle only; "
                         "demo = a close, calm camera for a 1-2 m hover with the person in view")
    ap.add_argument("--azimuth", type=float, default=None,
                    help="camera look direction, deg from +x (default: across the strongest push, else across the drift)")
    ap.add_argument("--elevation", type=float, default=None, help="camera pitch down, deg (default 12, demo 8)")
    ap.add_argument("--lang", choices=LANGS, default="ko", help="overlay language (ko: Korean, en: English)")
    ap.add_argument("--figure", choices=("auto", "on", "off"), default="auto",
                    help="1.7 m reference person: auto = when the log has pushes or with --framing demo")
    ap.add_argument("--style", choices=STYLES, default="realistic",
                    help="realistic (PBR, sky + image-based lighting, sun shadow, field / sports-hall scenery, soft jet, "
                         "mannequin) or schematic (the v2 look)")
    ap.add_argument("--hud", choices=HUDS, default="full",
                    help="full = HUD + side panel + insets; minimal = full-frame 3-D view with a compact HUD; "
                         "none = clean 3-D view (the simulation tag stays)")
    ap.add_argument("--camera", choices=CAMERAS, default="chase",
                    help="chase (planned camera, --framing) or spectator (eye height 1.6 m, pans / zooms, slight shake)")
    ap.add_argument("--motion-blur", type=int, default=1, help="N > 1: average N sub-frames of the 3-D view (N x slower)")
    ap.add_argument("--scene", choices=SCENES, default="auto",
                    help="realistic scenery: auto (meta.profile 'demo' -> indoor hall, else outdoor field), field, hall")
    ap.add_argument("--assets", default=ASSET_DIR, help="cache directory of the generated sky / texture images")
    ap.add_argument("--gps-pod", choices=GPS_POD_MODES, default="auto",
                    help="v0.8 GPS antenna pod: auto = when the log carries GPS keys or meta.gps_on, on, off")
    ap.add_argument("--poster-t", type=float, default=None)
    ap.add_argument("--crf", type=int, default=20)
    ap.add_argument("--aa", choices=("msaa", "fxaa", "none"), default="msaa")
    ap.add_argument("--export-dev", choices=("S1", "S2"), default=None,
                    help="write a development log from an existing ballistic scenario to --log first")
    ap.add_argument("--dev-t-end", type=float, default=None)
    ap.add_argument("--sim-root", default=PROJECT_ROOT, help="source tree for --export-dev (default: this project)")
    a = ap.parse_args(argv)
    if a.export_dev:
        a.log = a.log or os.path.join(OUT_DIR, f"dev_{a.export_dev}.npz")
        r = export_dev_log(a.export_dev, a.log, a.sim_root, a.dev_t_end)
        print(f"dev log {r['path']}: {r['samples']} samples to t = {r['t_end']:.2f} s (sim {r['runtime_s']:.1f} s wall)")
        for t, e in r["events"]:
            print(f"    t={t:7.3f}  {e}")
        if not a.out:
            return 0
    if not a.log or not a.out:
        ap.error("--log and --out are required (or --export-dev)")
    at = [float(x) for x in a.at.split(",") if x.strip()] if a.at else None
    info = render(a.log, a.out, a.fps, parse_size(a.size), a.speed, a.start, a.end, a.preview, at, a.title_s, a.end_s,
                  a.framing, a.azimuth, a.elevation, a.poster_t, a.crf, a.aa, lang=a.lang, figure=a.figure,
                  style=a.style, hud=a.hud, camera=a.camera, motion_blur=a.motion_blur, scene=a.scene,
                  asset_dir=a.assets or None, gps_pod=a.gps_pod)
    for w in info["warnings"]:
        print("warning:", w)
    print(f"frames {info.get('frames')}  render {info.get('render_s')} s  ({info.get('s_per_frame')} s/frame)  "
          f"setup {info['setup_s']} s")
    for fpath in info["files"][:3]:
        print("  ", fpath)
    if len(info["files"]) > 3:
        print(f"   ... {len(info['files'])} files")
    print("  sidecar", info["sidecar"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
