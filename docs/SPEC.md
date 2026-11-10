# CanSat EDF Rocket — Engineering Specification (source of truth)

Version 0.6.3 (params.json v0.6.3, 2026-10-15; this document brought to v0.6.3 on 2026-10-16). Language: English for
precision; user-facing docs are Korean. **CAD sections brought to params v0.9.0 on 2026-10-27** (§3.4, §3.5, §3.8,
§3.9, §4, §5.1, §6, §7, §12, change log); the v0.7 / v0.8 GPS / GNS sections are documented in their own Korean notes
and are not repeated here. **Simulation / control / tether sections brought to v0.9 on 2026-10-28** (§13.2, change log):
the VTOL flights fly the VTOL hardware configuration; §8 / §9 still describe the rocket (ballistic) mission, whose
controller and gains are unchanged. **Firmware brought to v0.9 on 2026-10-29** (§10.6, change log): four ATmega128A
(NAV / FCU / COM / GNS), the NAV timing closure (P1, interrupt-paced nlink, push floor 890 µs), the nlink exchange
watchdog W1, per-configuration vehicle constants G1, the power telemetry RP1 (TELEM v4 / v5), the closed-loop 4-core
co-simulation S1 and the per-configuration crosscheck X1; all 22 acceptance criteria PASS in simulation, nothing is
measured on hardware yet. Korean summary: `docs/v0.9_펌웨어_결과.md`.

**v0.9 (2026-10-27, BUDGET preset; KAIST keeps the v0.8 linkage)**: direct-drive vanes - each MG90S sits in a
fin-root fairing with its output axis on the vane hinge axis and turns the vane shaft through a printed coupler; the
fins moved onto the vane angles (0/90/180/270); pods, servo arms, pushrods and clevises are gone (§3.9, `vane_drive`).
The motor leads keep their 5.5 mm bullets and run through a 22 mm propulsion slot, an open 22 × 11 mm intake groove
under a printed cover and a 22 × 11 mm bulkhead hole to joints in the avionics bay (§5.1, `motor_leads`). Printed-part
masses come from slicing each part with its print class (`printer.mass_model`, §6, §12). Checks C28 and C29 are new;
C5, C20, C23 have direct-drive / open-groove branches (§7).

**v0.9 CAD-2 (2026-10-27, user decisions 2 / 4 / 5)**: a hardware configuration switch `mission.config` (`rocket` |
`vtol`, default `rocket` this round) and the lightweight **VTOL configuration** (`configs.vtol`, §13): no CanSat / chute /
payload module / nose; a top cap with the GPS and the tether hardpoint; ONE avionics carrier PCB instead of the four
boards; a shorter windowed avionics module; a 1.6 mm duct, 1.2 mm fairing walls, fin lightening holes, an EDF section
8 mm shorter and a 1.2 mm intake bulkhead; power sources `power.source` = battery | tether | hybrid.  VTOL battery
1636.2 g, battery-less tether 1388.3 g, hybrid 1509.3 g (rocket 2591.6 g, unchanged).

**v0.9 fix round (2026-10-28, independent geometry / mass / render reviews; §13.1)**: both configurations - a 6 mm slit
in the lip-pocket wall and three cable-tie anchors on the 0° strut (the motor leads can be installed and stay 3.5 mm
outside the EDF throat; bulkhead hole 20 × 11 at r 42-53), a horn-socket coupler locked to a +v D-flat by an M2 set
screw (axial retention, no D-flat play, no coupler screws), brass hinge bushings, an EVA pad on the servo; the 22 mm slot /
groove slivers removed.  VTOL - solid fins again (span 76.8, footprint 246.8), tray plane x 25 → 14.6 (lateral CG), the
gland at 180° / GPS patch at 0°, a D32 × 2 load washer under the eye bolt, the tether kit as boxes / solids.  New check
C33 (lateral CG trim).  VTOL battery 1647.2 g, tether 1399.3 g, hybrid 1520.3 g, rocket 2592.6 g.
**Superseded by the v0.9 lead-exit round (2026-10-29, incl. its independent review; §3.7 lead exit, §13.1):** VTOL battery
1655.9 g, tether 1408.0 g, hybrid 1529.0 g, rocket 2601.2 g (flight 2611.2) - the targets 1650 / 1400 g are missed by 5.9 /
8.0 g; lateral CG 0.80 / 2.28 / 1.73 mm (rocket 1.67); C33 with the motor-lead drag in the jet (E): battery 5.2 % WARN,
tether 10.9 % FAIL, hybrid 8.4 % WARN, rocket 4.6 % PASS.
**wire09c clock mount (2026-11-03, user 2026-10-30: "make the motor-lead passage generous, I do not want to measure"; §3.7c; before-state archived in `history/v0.9_wire_round2`):** the EDF is clocked to ONE lead passage at 0 deg that leaves the body through a blister (outer r 76.7, 45 deg V keel), 27 keyways round the circle (no tab screws), floor pads 3.0 mm; the whole `motor_leads.generosity_box` fits (factory sleeve kept, any exit azimuth, lead O4.3, bend R 7-15, TE 0-3, protrusion 30-36, rim gap 0-2): C29 grid 648 envelopes x module 0.0000 mm^3, C34 coverage 100 % (worst residual 6.77 deg). VTOL battery 1652.9 g, tether 1405.0 g, hybrid 1526.0 g, rocket 2598.4 g (targets 1650 / 1400 missed by 2.9 / 5.0 g); C33 tether FAIL 13.8 %, hybrid FAIL 11.0 %, battery WARN 7.5 %, rocket WARN 5.6 % (worst azimuth, worst Cd). **Fix round (2026-11-04, independent review):** keyways re-planned as a seat chain so every sleeve tooth carries the design motor torque (1.01 N m) on ONE tab in tangential bending with a factor >= 5 (the 1.73 mm tooth beside the 193.55 deg hover-seat keyway and a 1.30 mm sliver beside the passage had 0.64 / 0.37): 25 keyways, min tooth 5.30 mm (measured, C35 factor 5.9), coverage worst residual 6.50 deg; the turned 12 x 5 ribbon is a generosity-box section (C29 grid 864 envelopes x module 0.0000 mm^3; drag area 325.0 mm^2, 6.0 %); the leads are bent up BEFORE the EDF is lowered. VTOL battery 1653.6 g, tether 1405.7 g, hybrid 1526.7 g, rocket 2599.1 g (targets missed by 3.6 / 5.7 g); C33 tether FAIL 14.0 %, hybrid FAIL 11.2 %, battery WARN 7.7 %, rocket WARN 5.7 %. The paragraphs below are the earlier states.

**Superseded by the photo round (user photos 2026-10-29, `docs/실물사진/EDF_모터선_출구_2026-10-29_1.jpg` / `_2.jpg`; §3.7;
round 1 archived in `history/v0.9_wire_round1`):** VTOL battery 1655.1 g, tether 1407.2 g, hybrid 1528.2 g, rocket 2600.3 g
(flight 2610.3) - the targets 1650 / 1400 g are still missed by 5.1 / 7.2 g; lateral CG 0.80 / 2.28 / 1.73 mm (rocket
1.67); C33 with the lead-bundle drag (E): battery 4.4 % PASS, tether 9.9 % WARN (Cd band 9.5-10.4 %), hybrid 7.6 % WARN,
rocket 4.1 % PASS.
**Fix round of the photo round (independent reviews 2026-10-29; §3.7):** the factory heat-shrink is replaced by a 14 ± 1 mm
re-sleeve (assembly spec) followed by a triangle -> row transition (7.45 mm, E) before the bend; notch, printed parts (53
STL hashes) and masses unchanged. Lead drag area 242 mm² (D/T 2.39 %, 1.91-2.86 %); C33 is judged on the worst end of the
Cd band: battery 4.7 % (worst 5.2 %) WARN, tether 10.3 % (worst 10.8 %) **FAIL**, hybrid 7.9 % WARN, rocket 4.3 % PASS;
C29 WARNs while the lead exit angle is not measured (route tolerance ±9.6° VTOL / ±12.7° rocket, E, vs ±70° photo
uncertainty).

**v0.6 (2026-10-15, v0.6.0-v0.6.3)**: the flight computer runs on three ATmega128A - NAV (sensors, estimator),
FCU (every flight decision, outputs) and COM (LoRa link) - joined by the ilink UART and the nlink SPI (§10.1, §10.4,
§10.5); the vane-servo leads have a designed route and end on the FCU board (§3.8); the intake's windowed band is
3 mm thick with radial-sector windows whose lower ends are 45° gables, printed from a Bambu project with support
blockers (§5.1, §12); the COM board and a radio/log box hang under the payload bulkhead and the FCU board lies on the
intake bulkhead (§5.2, §5.3, §6, checks C22-C25). Evidence for each step: change log at the end.

**v0.3: the default preset is `BUDGET_90mm_6S`** (vehicle parts budget 200-300k KRW,
CanSat excluded): Hobbyocean 90 mm 6S EDF, 100 A ESC, one 6S 2200 pack, 1.2 mm
walls, international-class CanSat (Ø66 x 115 mm, 350 g) in a 150 mm envelope.
The v0.2 design (KAIST Ø110 x 200 mm / 1 kg, 12S) is kept as the alt preset
`KAIST_90mm_12S` and reproduces the v0.2 layout exactly. Unless a section says
otherwise, the numbers in parentheses below are the v0.2 / `KAIST_90mm_12S`
values; `python cad/layout.py` prints the default preset.

**v0.2 changes** (all driven by tool results, see the change log at the end):
ATmega128 flight computer (control loop 250 Hz); vane span 13–43 mm; hinge
boss Ø12 to r 52; wider servo pods; spigot shoulder rings and propulsion deck
made explicit; nose wall = true 2 mm normal offset; 140 m target / 170 m
geofence; 1.7 m vehicle chute; avionics module 215 mm with a 50 × 50 mm flight
computer; the 66 mm / 70 mm-EDF preset was removed (T/W 0.93).

**Numbers in parentheses are illustrative values for params v0.2.** The
authoritative value is always the named `Layout` attribute; run
`python cad/layout.py` to print the current table.

This document defines **what** is built. `params.json` holds every primitive
number; `cad/layout.py` derives every station/radius from it. Code that builds
geometry (Fusion 360 script, CadQuery twin), analyses it (sizing, checks) or
flies it (simulation, firmware) must read those two sources and never hard-code a
dimension that they provide. When this spec and the code disagree, the code is
wrong. When this spec and `layout.py` disagree, raise it — do not silently pick one.

---

## 0. Mission and concept

* Vehicle: single electric ducted fan (EDF) "rocket" with four exhaust control
  vanes (thrust-vector control, TVC), vertical take-off from a launch pad,
  climbs to a target altitude, cuts the motor, coasts to apogee, releases the
  nose cone and ejects a CanSat (its own parachute), recovers under its own
  parachute stowed in the nose cone.
* Payload (default preset `BUDGET_90mm_6S`, since v0.3): an international-class CanSat, can Ø66 mm × 115 mm,
  ≤ 350 g, in a Ø110 mm × 150 mm envelope (`payload.standard`, `can_d`, `can_h`, `mass_g`, `envelope_d`,
  `envelope_h`). The KAIST CanSat competition "rocket interior" envelope (cylinder Ø110 mm × 200 mm, up to
  1.0 kg) is the v0.2 design, kept as `alt_presets.KAIST_90mm_12S`. v0.2 history: `alt_presets` was then empty -
  the former 66 mm / 70 mm-EDF preset could not lift itself (T/W 0.93).
* Manufacturing: all structure FDM-printed on a Bambu Lab P2S (256³ mm bed) in
  Bambu PLA Matte Ivory White (`printer.*`, colour `printer.color` / `color_hex` #FFFFFF, code 11100; user 2026-11-04,
  was PLA Matte Black / Charcoal since 2026-10-20); the launch pad keeps the black (`printer.color_exceptions`). Livery
  (`livery`, user 2026-11-04, BUDGET rocket and VTOL): the word YUCHAN in Bambu PLA Matte Mandarin Orange (#F99963, code
  11300) as a flush 2-colour inlay, one outer wall line (0.42 mm) deep, on both sides of the avionics module (azimuth
  67 / 293 deg, reading along the axis); the inlay is its own printed part `avionics_logo` (polygons
  `cad/livery/yuchan_logo_polygons.json`, placed by `Layout._livery`), the avionics module is the white body with the
  pocket, printed together as one 2-filament object (AMS; `detect_thin_wall` on so the one-line orange is ONE pass, review F1); check C36. Every printed part fits the bed in its print
  orientation; `layout.py` raises if not.
* Control: a flight computer of three ATmega128A (16 MHz, 5 V, `flight_computer.*`; v0.6.3, §10.1) running
  custom C firmware that implements the attitude-stabilisation and supervisor logic of §8 (§10). Every flight
  computation runs on an ATmega128A (user rule 2026-10-15). Attitude control only: no position guidance and
  no GPS. ArduPilot is not an option on an ATmega128.

## 1. Coordinate system, units, naming

* Right-handed, **+Z up along the thrust axis**, **Z = 0 at the exhaust duct exit
  plane**, origin on the axis. +X points toward vane 1 (θ = 0°). θ is measured
  from +X toward +Y. Radii `r`, angles `θ` in degrees in `params.json`.
* Lengths mm, masses g, angles deg in `params.json`/`layout.py`. **Fusion 360
  API lengths are cm** — every mm value is divided by 10 before it reaches the
  API; angles in radians. The CadQuery twin works in mm.
* Component (part) names are the keys of `Layout.printed_parts` plus the
  placeholder names in §6. Use them verbatim for Fusion components, STL file
  names and report keys.
* Material density for all printed parts: `printer.density_g_cm3` (1.31).
* Everything is built in **global vehicle coordinates**; a component's local
  origin coincides with the vehicle origin (identity transform). Do not
  translate occurrences.

## 2. Body tube and module joints

* Body OD `body_od` = 120, wall `body_wall` = 2 → ID `body_id` = 116.
* Modules stack bottom→top: `propulsion_module`, `intake_module`,
  `avionics_module`, `payload_module`, `nose_cone`. Z ranges:
  `Layout.module_z_ranges()`.
* **Spigot joint**: the *lower* module ends in a male spigot ring of OD
  `spigot_od` (= ID − 0.4), ID `spigot_id`, height `spigot_h` = 15; the *upper*
  module's plain tube (ID `body_id`) slides over it and sits on the lower
  module's shoulder (the shoulder is the annulus between `spigot_od` and
  `body_od` at the spigot's bottom Z). Each joint is fixed with
  `body.joint_screw_count` = 4 radial M3 screws at θ = `Layout.joint_thetas`
  (45°, 135°, 225°, 315°), at Z = spigot bottom + `joint_screw_dz` (7.5):
  through-hole Ø`joint_hole_outer_d` (3.2) in the upper tube, hole
  Ø`joint_hole_spigot_d` (2.5, self-tapping M3) in the spigot. Both holes are
  radial (axis through the vehicle axis).
* **Shoulder ring** (v0.2, made explicit): `spigot_od`/2 (57.8) is 0.2 mm
  smaller than the tube bore radius (58), so a spigot standing on a plain tube
  would share no face with it. Every lower module therefore carries a ring of
  thickness `body.wall` (2) directly under the spigot, Z ∈ [spigot bottom − 2,
  spigot bottom], from r = `spigot_od`/2 − `body.spigot_wall`/2 (56.8, mid-wall
  of the spigot) out to `body_r`. Its top face outside `spigot_od` is the seat
  of the upper module. The propulsion module uses the deck of §3.2 instead.
* Exception: the nose/payload joint is held by the latch (§5.4), not screws, but
  the geometry is the same spigot/skirt lap.

## 3. Propulsion module (`propulsion_module`, one printed body, Z 0 → `z_prop_top`)

Printed upright (exit plane on the bed). Height `prop_module_h` (215).

3.1 **Exhaust duct / vane section**, Z ∈ [0, `z_vane_section_top`] (0–90):
tube ID `duct_id` (90), OD `vane_section_od` (98).

3.2 **EDF section**, Z ∈ [`z_vane_section_top`, `z_prop_spigot_top`]:
* Outer **cone shell**: revolve of a trapezoid — outer radius linear from
  `cone_r_bottom` (= `vane_section_r`, 49) at Z = `cone_z0` (90) to `cone_r_top`
  (= `body_r`, 60) at Z = `cone_z1` (= `z_prop_spigot_bottom`, 200); wall
  `cone_wall` (2) measured radially inward.
* Inner **sleeve**: tube bore Ø`edf_bore_d` (94), OD `sleeve_od` (98), Z ∈
  [`z_sleeve_bottom` (= `z_edf_shoulder`, 108), `z_prop_spigot_bottom`].
* **Shoulder ring**: Z ∈ [`z_vane_section_top`, `z_edf_shoulder`] solid between
  the duct bore (Ø`duct_id`) and the cone shell outer surface. The step from
  Ø94 to Ø90 at `z_edf_shoulder` is the EDF housing's bottom stop
  (`edf.bottom_clearance` = 1 mm below the housing).
* **Spigot ring**: Z ∈ [`z_prop_spigot_bottom`, `z_prop_spigot_top`] (200–215),
  OD `spigot_od`, ID `edf_bore_d` (solid annulus; the EDF lip rests on its top
  face). Four Ø2.5 radial holes at `joint_thetas`, Z = 207.5; v0.5d (both presets): blind, depth
  `prop_joint_hole_depth` (7) from the spigot OD, so an over-long joint screw bottoms in plastic instead of
  reaching the EDF housing (use M3 × 8).
* **Deck** (v0.2): solid annulus Z ∈ [`z_prop_spigot_bottom` − `body.wall`,
  `z_prop_spigot_bottom`] (198–200) from r = `edf_bore_r` (47) out to the cone
  outer surface (following the cone, no lip). It ties the spigot ring to the
  cone shell and is the seat of the intake skirt; it prints as an 11 mm radial
  bridge between the sleeve and the cone.
* **Webs**: `propulsion_module.web_count` (4) radial plates, thickness `web_t`
  (2), at `web_thetas` (45°+k·90°), spanning from the sleeve OD to the cone
  inner surface, Z ∈ [`z_sleeve_bottom`, `z_prop_spigot_bottom`].

3.3 **Hinge bosses**: 4 cylinders Ø`hinge_boss_od` (12) with axis radial at
θ = `vane_thetas`, Z = `vane_hinge_z` (37), from r = `duct_r` (inside the wall)
to r = `hinge_boss_r_out` (52), joined to the duct wall; bore Ø`hinge_bore_d`
(5.4) through boss and wall (r 45 → 52). (v0.2: Ø14 / r 55 collided with the
MG90S mounting ear by 1.95 mm³; Ø12 / r 52 leaves ≈ 0.7 mm.)

3.4 **Servo pods** (4, for the vanes; **only `vane_drive.mode` = `pushrod`**, the KAIST preset - v0.9 BUDGET uses the
fin-root fairings of §3.9): a rectangular cup, open at the top (+Z), at θ = `pod_thetas` (= vane θ + 24°):
* In the pod's local frame (radial axis u, tangential axis v, vertical Z): the
  outer box spans u ∈ [`pod_r_in` (46), `pod_r_out` (66.5)], v ∈ ±`pod_w`/2
  (±16.5), Z ∈ [`pod_z0` (18.5), `pod_rim_z` (38.5)]. The box is unioned with the
  duct. The duct bore is re-cut only if `pod_r_in` < `duct_r` (never with v0.2
  values; a re-cut that removes nothing is an error in Fusion).
* Cavity: u ∈ [`pod_cavity_r_in` (50), `pod_cavity_r_out` (63.5)], v ∈
  ±`pod_cavity_w`/2 (±11.8), Z ∈ [`pod_floor_top_z` (20.5), `pod_rim_z`] — open
  at the top. (v0.2: cavity 23.6 and side wall 4.7 so both walls beside each
  Ø1.8 ear hole are ≥ 1.2 mm = `printer.min_feature`.)
* Underside: the box's bottom face is chamfered at `pod.underside_chamfer_deg`
  (45°) from the duct wall outward (so the overhang prints without support) —
  implement as a wedge cut: remove the region below the line from
  (u = `vane_section_r`, Z = `pod_z0` − (`pod_r_out` − `vane_section_r`)) to
  (u = `pod_r_out`, Z = `pod_z0`). If this wedge reaches below Z = 0, clamp at
  Z = 0.
* Servo ear screw holes: two Ø`servo.ear_hole_d` (1.8) holes, 6 mm deep, from the
  rim top face downward, at v = ±`servo.ear_hole_pitch`/2 (±14) about the servo
  body centre, u = `servo_axis_r` (56.75). (The MG90S shaft is offset
  `servo.shaft_offset_along_l` (5.5) from the body centre; the shaft is placed
  at v = `servo_shaft_tangential_offset` (−5.5, toward the vane horn) and the
  body centre at v = 0.)

3.5 **Fins** (4): flat plates, thickness `fin_t` (3), in the plane containing the
vehicle axis at θ = `fin_thetas` (`fin.theta0_deg` + k·90°: 45° with the pushrod linkage; v0.9 direct drive 0°, ON
the vane angles, so each fin carries its vane's servo fairing - the fins are also the landing legs, planform and
`fin_t` unchanged; the launch pad's legs follow, `launch_pad.leg_theta0_deg`). Planform in that plane
(r horizontal, Z vertical):
root at r = `fin_root_r` (48, overlapping 1 mm into the duct wall) from
Z = `fin_te_z` (0) to Z = `fin_root_le_z` (90); leading edge straight from
(`fin_root_r`, `fin_root_le_z`) to (`fin_tip_r` (119), `fin_tip_le_z` (40));
tip from (`fin_tip_r`, `fin_tip_le_z`) to (`fin_tip_r`, 0); trailing edge along
Z = 0 back to the root. Union with the module.

3.6 **EDF seat check** (not geometry, `edf.mount` = `lip_clamp`, KAIST preset): with the lip
resting on the spigot top face at Z = `z_edf_lip_bottom`, the housing bottom is at
`z_edf_housing_bottom` and the shoulder at `z_edf_shoulder`.

3.7 **Tab mount (v0.5, `edf.mount` = `tabs`, default preset).** The Hobbyocean 90 mm EDF is held by
its two moulded mounting tabs, as the vendor intends; dimensions are read from the vendor drawing
(`edf.drawing_source`) and every value below is a Layout attribute (numbers: default preset).
* EDF placement: bell-lip front face at `z_edf_lip_top` (202.0); the bell (Ø`edf.lip_od` 112, depth
  `edf.lip_depth` 11) occupies [`z_edf_lip_bottom` (191.0), `z_edf_lip_top`], `lip_gap_below` (1) above
  the spigot top; housing Ø`edf.housing_od` down to `z_edf_housing_bottom` (120.9, 81.1 from the lip
  front); motor Ø`edf.motor_d` protrudes `edf.motor_protrusion_rear` (30) below it.
* Tabs: `tab_count` (2) radial plates at `tab_thetas` (90°, 270°; v0.5d - the leads then leave at 0°, on the
  ESC side of the avionics tray), r ∈ [`tab_r_in` (49), `tab_r_out`
  (57.1)], thickness `tab_t` (3.0, ASSUMED), Z ∈ [`tab_z0` (134.0), `tab_z1` (176.3)]; three slots
  `tab_slot_w` × `tab_slot_len_radial` (3.5 × 5.8) centred at r = `tab_slot_r` (52), Z = `tab_slot_zs`.
* **Keyways** (cut, per tab): box tangential ±`keyway_w`/2 (4.0), r ∈ [`keyway_r_in`, `keyway_r_out`
  (57.6)], Z ∈ [`keyway_z0` (= `tab_z0`), `keyway_z1`] — open at the top so the EDF drops in from
  above; the keyway floor is the tab's axial seat.
* **Screw lugs** (solid, per tab, unioned): angular sector [θ_tab − hs·`block_theta_far` (13.1°),
  θ_tab + hs·`block_theta_head` (39.3°)] (hs = `tab_screw_head_side`, +1 → head side at +θ), r ∈
  [`block_r_in` (= `sleeve_r`), outer skin], one Z range per entry of `boss_z_ranges` ((132.8, 149.4),
  (161.0, 175.0)): each screw ± (`d_cbore`/2 + `block_margin_z`); the lowest lug reaches `floor_t` (1.2)
  below `keyway_z0` so the lug forms the keyway floor, the highest merges with the deck.
* **Tab screws** (`tab_screw_size` × `tab_screw_len`, M3 × 20, one per used slot at Z = `tab_screw_zs`
  (144.2, 166.2)): tangential axis P(t) = `tab_screw_r`·ê_r(θ_tab) + `tab_screw_head_side`·t·ê_θ(θ_tab),
  Z = const (`Layout.tab_screw_point`). Cut per screw: clearance hole Ø`tab_screw_d_clear` (3.2) for
  t ∈ [`tab_screw_t_keyway`, `tab_screw_t_head`]; counterbore Ø`tab_screw_d_cbore` (6.4) for
  t ∈ [`tab_screw_t_head` (14), `tab_screw_t_cbore_end` (35.91)] — the end is where the counterbore's
  edge nearest the vehicle axis (r = `tab_screw_r` − d/2) leaves the skin, + 1 mm, so the button head
  enters straight along the axis; tap hole Ø`tab_screw_d_tap` (2.5) for t ∈ [`tab_screw_t_tap_end` (−7),
  −`tab_screw_t_keyway`]. Thread engagement `tab_screw_engagement` (4.0 mm) beyond the tab; the thinnest
  wall from any hole segment to the EDF bore is `tab_hole_wall_to_bore` (0.69 mm, clearance hole at the
  keyway face).
* **Outer skin**: chamfer + linear cone from `cone_z0` to `cone_z1` (= `z_cyl0` = `keyway_z0` −
  `cyl_margin_below_tab`, 131.0), then a cylinder r = `body_r` from `cone_z1` to `z_prop_spigot_bottom`
  (`Layout.outer_skin_profile`, `cone_outer_radius`). Webs follow it.
* **Liner**: the shoulder ring keeps its minimum height (`z_edf_shoulder` = `z_vane_section_top` +
  `shoulder_ring_min_h`, 108); a thin duct tube r ∈ [`duct_r`, `liner_r_out` (= `duct_r` + `liner_wall`,
  46.6)] over [`z_edf_shoulder`, `z_liner_top`] (= `z_edf_housing_bottom` − `edf.bottom_clearance`, 119.9)
  plus a top ring r ∈ [`duct_r`, `sleeve_r`] over [`z_liner_ring_bottom`, `z_liner_top`] continue the Ø90
  duct up to the housing (closed cavity behind the tube).
* **Wire slot** (cut): the motor leads leave the housing 90° from the tabs (`edf.wire_exit`); two boxes
  at θ = `wire_theta` (0°), tangential ±`wire_slot_w`/2 (14), r from `wire_slot_r_in`: (a) r ≤
  `wire_slot_lower_r_out` (57.78, corners keep `skin_min_wall` to the skin) for Z ∈ [`wire_slot_z0` (139.7),
  `z_prop_spigot_bottom`] through sleeve and deck; (b) r ≤ `wire_slot_r_out` (58.7) for Z ∈
  [`z_prop_spigot_bottom`, `wire_slot_z1`] through the spigot ring (open at the top and to the skirt).
* **Lead exit (v0.9 lead-exit round, user 2026-10-29; `edf.wire_exit.motor_exit` / `stator_vane`, `motor_leads.lower_route`).**
  The leads leave the motor's side at its lowest end (motor bottom `ml_z_motor_bottom` = `z_edf_housing_bottom` −
  `edf.motor_protrusion_rear`, 90.9; exit centre `ml_z_exit` 93.9, ASSUMED 3 mm above it), bend up along the motor can
  (`ml_u_climb` 24.85), bend out along the trailing edge of the EDF's own stator vane at `wire_theta` (ASSUMED flush with the
  housing's rear rim; run `ml_z_run` 117.74), leave the EDF under the rear rim and bend up into the slot beside the housing
  (`ml_u_slot` 53.254), all bends R `bend_r_centre_ASSUMED` 7.2. The slot box (a) keeps its start `wire_slot_z0`
  (= `z_edf_lip_top` − `wire_slot.wide_z0_from_lip_front`, 139.7); below it the **lead notch / lower slot** (cut) is the (u, z)
  polygon `ml_low_cut_uz` at `wire_theta`, `wire_slot.low_w` (12.4) wide: from `duct_r` − 0.5 (opens the bore under the rim,
  through the liner) out to the leads' envelope + `low_cut_margin` with a 45° chamfer along the outer bend, up 0.5 into box
  (a). Its outer corner keeps `cone_wall` + `skin_min_wall` to the cone skin (`ml_u_slot` is the largest radius that does).
  The pre-2026-10-29 housing-side exit (`edf.wire_exit.superseded_2026_10_29`) loads only from params without `motor_exit`.
  Review fix (2026-10-29): no cable tie round the stator vane (it would loop over the vane's leading edge next to the rotor);
  `lower_route.restraint` = `rtv_bond_stator_te` - the leads are bonded with RTV to the stator trailing edge over
  `ml_bond_u` (32.05-44.5), the gap lead -> trailing edge `ml_stator_te_gap` (1.36 / 2.24) must stay <= `bond_gap_max_ASSUMED`
  (one OD) or Layout raises; free spans exit -> bond 32.1 / 31.2 mm <= `max_free_span_jet_ASSUMED` 40 (no basis in the jet,
  ASSUMED). The leads lie side by side only (stacking would bend the inner strand at R - OD). Jet blockage (E) 270 mm²
  projected (5.0 %), drag area 309 mm² (5.7 %, run + exit bend outside the hub); `ml_jet_drag` (Cd 1.0, 0.8-1.2 ASSUMED)
  drives a C33 moment term and the T/W-1.3 thrust derate in `sizing/vtol_sizing.py` (D/T 3.0 %, 2.4-3.7 %). The route fits
  bend radii up to 9.0 (VTOL) / 9.5 mm (rocket) only. RTV + chafe tape 1.0 g ASSUMED are lumped into `motor_leads`.
  **Photo round (user photos 2026-10-29, supersedes the climb above; `edf.wire_exit.motor_exit.level` = `front_ring`,
  `motor_leads.lower_route.bundle`; round 1 in `edf.wire_exit.superseded_2026_10_29_round1`).** The photos (EDF from the
  exhaust side) show the stator vanes at the housing's rear rim, the motor can protruding behind it, and the three leads
  leaving the motor side at its front ring (stator / rim level) as ONE heat-shrink bundle along one stator vane, out under
  the rear rim; single leads after the shrink; the housing's side slot is unused. Drawing data: rim = lip front − 81.1
  (rocket 120.9, VTOL 112.9), motor rear end = lip front − 111.1 (90.9 / 82.9; overall_len 116.4 − spinner 5.0 = 111.4),
  front ring = rim .. rim − `front_ring_w_ASSUMED` 5. Route: bundle O`bundle.od_ASSUMED` 8.4 straight from the motor side
  (r 17.65) at `ml_z_exit` = `ml_z_run` = rim − `gap_to_te_ASSUMED` 0.4 − 4.2 (116.3 / 108.3) behind the vane's trailing
  edge and under the rim, the shrink ending at `ml_u_shrink_end` = r_motor + `shrink_len_from_motor_ASSUMED` 28 = 45.65;
  single leads then ONE bend R 7.2 up into the slot at `ml_u_slot` 52.953 / 52.169 (bend start − shrink end
  `ml_shrink_bend_margin` +0.10 / −0.68 mm: on the VTOL the bundle takes the first 5.4° of the bend; its envelope is
  checked there). The notch polygon's bottom sits `low_cut_margin` under the bundle (rocket z 111.40, VTOL 103.40, ~3-3.8 mm
  deeper than round 1) and its chamfer outside every envelope. RTV bond u 20.0-44.5, free spans 2.35 / 0 mm, TE gap 0.40.
  Jet: one bundle from the hub radius 20 to `duct_r` 45: 210 mm² (3.9 %), D/T 2.07 % (1.66-2.48 %, Cd 0.8-1.2). RTV /
  seal / tape 1.7 g + shrink 0.3 g ASSUMED lumped into `motor_leads` (50.98 g VTOL). Tolerances (Layout, M): VTOL shrink
  length ≤ 28.72 mm, bend R ≤ 7.91, bundle OD ≤ 10.16, lead OD ≤ 3.81, gap ≤ 3.03, TE inset ≤ 3.2; rocket ≤ 30.00 / 9.71 /
  11.6 / 3.87 / 3.6 / 3.2.
  **Fix round (independent reviews 2026-10-29, supersedes the shrink / tolerance numbers above).** The photos show the
  factory sleeve stepped (two layers), ending at about the housing's outer surface (~31 mm from the can) with the leads
  leaving it stacked along the motor axis. In the bend each lead needs radius R, so the leads must lie side by side across
  the slot first: triangle -> row moves each lead by od / √3 = 2.08 mm, an S-bend at R 7.2 needs `ml_transition_len`
  7.45 mm (E); the bundle envelope runs to `ml_u_bundle_env_end` = shrink end + transition, the row envelope from the
  shrink end (their union over the transition, `motor_lead_lower_legs`). A 28-31 mm factory sleeve then does not fit
  (Layout raises, rim corner), so the assembly spec is: slit off the factory sleeve (only if it does not cover the
  winding-to-lead joints - else stop) and re-sleeve the first `shrink_len_from_motor_ASSUMED` = 14 ± 1 mm as a round
  triangle (8.4 target). Shrink end r 31.65, transition end 39.10 (inside `duct_r`: the bundle never reaches the module),
  bend start − (shrink + transition) `ml_shrink_bend_margin` +6.65 / +5.87 mm (C29 WARNs if < 0). Exit-angle tolerance
  `ml_exit_angle_tol_deg` (E): the straight single leads (14.10 / 13.32 mm) steer an exit-angle error sideways into the notch
  together with the transition: ±12.7° rocket / ±9.6° VTOL (18 mm sleeve: ±3.3 / ±2.4); C29 WARNs while
  `edf.wire_exit.motor_exit.theta_measured` is false and the tolerance is below `theta_uncertainty_deg_ASSUMED` 70 (the
  two photo readings: about −90° or 22-47° from a tab). Jet: bundle 8.4 × (31.65 − 20) + row 10.8 × (45 − 31.65) = 242 mm²
  (4.5 %), D/T 2.39 % (1.91-2.86 %); Cd 0.8-1.2 kept for both (short bodies, aspect ratio ≤ 2: a flat plate is ~1.2 up to
  AR 5, Hoerner; 2.0 is the 2D value). C8 now quantifies the lead envelope in the exhaust path (2440 mm³, allowed); C33 is
  judged on the worst end of the Cd band; `sizing/vtol_strength.py` adds the net section through the notch (outer cone
  shell only: factor 69). Tolerances (Layout, M): shrink ≤ 19.87 VTOL / 20.65 rocket without WARN (builds ≤ 21.3 / 22.5),
  bend R ≤ 8.00 / 9.83, bundle OD ≤ 11.6, lead OD ≤ 3.81 / 3.87, gap ≤ 3.6, TE inset ≤ 3.2.

**3.7c wire09c clock mount (`propulsion_module.tab_mount` = `clock_keyways`, the default; `screws` keeps 3.7 / 3.7b).**
* Lead passage (`lead_passage`, `Layout._clock_fan`): in plan the union of the radial rectangles u in [`duct_r` - 3, `lp_r_wall`],
  |v| <= `lp_hw` turned over theta_P +- `lp_E_design` (theta_P = `lead_passage.theta_deg`, 0; `lp_E_design` = `clock_keyways.E_cov_deg`
  7.5 + the tab play 0.54 = 8.04 deg); `lp_hw` = the largest tangential envelope half of `motor_leads.generosity_box` (+
  `wire_clearance`) + `fan_margin` = 7.2; `lp_r_wall` = motor_d/2 + max sleeve 35 + max bend R 15 + the stacked O4.3 leads 6.85 +
  `bend_tol` 1.0 = 75.5. Exact outline `layout.clock_fan_outline` (two side lines, the corner arc sampled inside). Cut from the fan
  floor (lowest envelope of every corner - `floor_margin`: VTOL 96.8, rocket 104.8) through the module top.
* Blister: the outline grown by `wall` 1.2 minus the EDF bore disc, from the fan floor - wall to `blister_top_gap` under the intake;
  V keel under its floor: (v, z) triangles (45 deg) extruded radially in `keel_step` 2 mm steps, each no wider than the outline
  over its step (`lp_keel`); the keel rests on fin 1. Frontal area outside the body 619 mm^2 (`lp_blister_frontal_mm2`, used by
  sizing.py's drag reference).
* Keyways (`Layout._clock_seat`, computed): `kw` 4.0 wide, r `keyway_r_in`..`keyway_r_out`, floor `lp_z_floor` = `tab_z0` -
  `edf.axial_retention.floor_drop` 2.4, open at the top; the walls off the webs (+`web_margin`), the servo-column guide walls
  (+`col_clear`) and the joint holes. Fix round (2026-11-04, review F1): every sleeve tooth (sleeve_r - edf_bore_r = 1.6 deep,
  free from the keyway floor to the deck bottom, 42.15 mm) carries the C35 design torque max(1, 1.5 P / omega) = 1.01 N m on
  ONE tab, fixed-fixed, uniform, factor >= `tooth_factor_min` 5 vs `printer.bending_mpa` -> w_req 4.86 mm, planned >= w_req +
  `tooth_w_margin` 0.2 = 5.06 (also the tooth between a keyway and the passage; webs / walls not credited). Planner: a seat
  chain on the mod-180 circle (`plan_step_deg` 0.25 grid), a seat = a keyway pair or one keyway + a tab hovering in the
  passage, consecutive seats >= the tooth spacing (10.43 deg) and <= 2 `E_cov_deg` - `plan_gap_margin_deg` (14 deg), fewest
  seats, then the smallest largest gap: 25 keyways (24.75 .. 333.50 deg), 25 seats (3 hovering), largest seat gap 13.0 deg,
  worst residual 6.50 deg. C35 measures every tooth on the solid (min 5.30 mm, factor 5.9). Floor pad under every keyway: (r, z) polygon `ck_pad_rz`, `pad_t`
  3.0, kw + 2 `pad_margin` wide, 45 deg corbel. Servo-column guide walls: a U prism (web rule) per column whose channel is cut back
  to the cavity (`col_wall` 1.2).
* Axial retention (`edf.axial_retention`, A): lip on the intake shelf (up, `lip_stop_gap` 0.5), EVA foam pads (3.5 mm) under the tabs
  on the keyway floors (down). No tab screws / lugs / 22 mm slot / lead notch.
* Leads (`Layout._motor_lead_clock`): every corner's path (`clock_lead_path`: bundle, single leads under the rim, bend R, vertical
  leg at the wall, steer to `u_top` 64.5 at `z_intake_bottom` - 2, flat in the groove); mass / route from the longest corner;
  drag area of the worst corner 325.0 mm^2 (6.0 %; the turned 12 x 5 ribbon, fix round F5). Passages downstream are checked with the thickest lead (O4.3); the intake
  bulkhead hole is r 41-53.2.

3.8 **Vane-servo lead route (v0.6, `edf.mount` = `tabs` and `servo_leads` in params; `Layout.servo_leads`).**
Before v0.6 the four MG90S leads had no path: the pod cavity (23.6 for a 23.0 case) left 0.3 mm for a lead that
needs ≥ 1.3, the shoulder ring is solid with 2.4 mm between the duct bore and the outer surface at its bottom, the
EDF section skin was closed and the motor-lead path is sized for three 12 AWG leads. The route: out of the pod,
outside up beside the fin, through the shoulder ring / cone skin into the closed annulus between the sleeve and the
skin, one straight column per lead to the deck, round the spigot ring under the intake skirt to `sl_gather_theta`
(180°), past the EDF bell in a second intake channel (§5.1), along the strut at 180° and up through the intake
bulkhead servo hole (−x side) onto the FCU board, which drives the servos and lies on that bulkhead (v0.6.3, §5.2;
in v0.6.0-v0.6.2 the leads ran on, across under the tray, to the flight computer on the tray's +x face). Every value
is a Layout attribute `sl_*` (numbers: default preset); the SPEC 7 check is C23.
* **Ribbon envelope** `servo_leads.ribbon_w_ASSUMED` × `ribbon_t_ASSUMED` (3.6 × 1.3: 26 AWG 3-core flat silicone
  servo wire; measure the wire bought). Every passage is ≥ the ribbon (one lead) or a 2 × 2 stack (7.2 × 2.6,
  gathered leads) + 2 × `min_clearance` (0.4) in both directions, and every wall the route leaves is ≥
  `printer.min_feature` (layout raises otherwise; the walls are listed in C10).
* **Lead exit** `servo_leads.lead_exit_ASSUMED` = `far_end`: the MG90S lead leaves the case end face opposite the
  output shaft, near the bottom, i.e. at v = `sl_side` · body_l/2 (+v, away from the vane horn and pushrod at −v).
  Verify on the unit before printing; `shaft_end` has no route (layout raises).
* **Pod lead window** (cut, every vane pod): u ∈ [`sl_win_u0`, `sl_win_u1`] (52.85–57.45, centred on
  `servo_axis_r`), v from the cavity out through the far end wall (11.8 → 16.5, over-travel 1 mm each side), Z ∈
  [`sl_win_z0` (= `pod_floor_top_z`, 20.5), `sl_win_z1` (27.5)]: 5.0 mm below the ear-screw hole, 4.45 mm of end
  wall beside it.
* **Outside run** (not enclosed; taped to the duct and fin root): up beside the pod's far face to the crossing band
  Z ∈ [`sl_cross_z0` (88.40 = fin leading edge at r = `sl_entry_u0`, + `crossing_above_fin_le`), `sl_cross_z1`
  (= + ribbon width, 92.00)], over the fin line (above its leading edge, below the shoulder-ring chamfer) to the
  column at `sl_col_thetas` = the fin on the lead side + `sl_side` · `column_theta_from_fin_deg` (53°, 143°, 233°,
  323°). The splice to the extension (`servo_lead_kept` 60 mm of the MG90S lead, `splice_len` 20) lies on this
  outside run.
* **Entry slot** (cut, per lead): vertical box at `sl_col_thetas`, r ∈ [`sl_entry_u0` (= `sleeve_r` +
  `entry.gap_to_sleeve`, 52.55), `sl_entry_u1` (54.95)], tangential ±`entry.w`/2 (4.6), Z from `sl_entry_z0`
  (= `z_vane_section_top`; no material at r ≥ `sl_entry_u0` there) to `sl_entry_z_open` (112.50: the skin's inner
  surface passes `sl_entry_u1`, `Layout.outer_skin_z_at_radius`) + 1 mm over-travel into the annulus. It opens the
  chamfer / cone from Z `sl_entry_z_mouth` (95.15); it stays ≥ 1.2 mm from the webs, below the screw lugs and off
  the sleeve.
* **Column**: inside the annulus each lead runs straight from the slot centre (r 53.75, Z `sl_entry_z_open`) to the
  deck-notch centre (r 57.10, deck bottom) - a straight line, so a pull wire threads it.
* **Deck notch** (cut, per lead, tangential ±`deck_notch.w`/2 (4.6)): (a) r ∈ [`sl_notch_u0` (= spigot_od/2 −
  `deck_notch.radial`, 55.60), spigot_od/2] through the deck - the deck outside its corners (`sl_deck_seat` 1.35)
  stays the intake skirt's seat; (b) r ∈ [`sl_notch_u0`, spigot_od/2 + 1] up the spigot ring from
  `z_prop_spigot_bottom` into the rebate (open to the skirt).
* **Rebate** (cut): annular sector r ∈ [`sl_rebate_r0` (= spigot_od/2 − `rebate.depth`, 55.20), spigot_od/2 + 1],
  Z ∈ [`sl_rebate_z0` (= `z_prop_spigot_top` − `rebate.h`, 185.0), spigot top + 1], θ ∈ [`sl_rebate_th0`,
  `sl_rebate_th1`] (50.63–325.37: from the outermost column on each side to `sl_gather_theta`, never across the
  motor wire slot). Two ribbons per side lie in it flat (width along Z). It is enclosed by the intake skirt and the
  pocket-wall ledge once the intake is on and open from outside before that (the leads are laid in by hand). It
  crosses the keyways `sl_rebate_over_tab` (8.65) above the tab tops and opens their outer wall there on purpose
  (C21 probes the keyway wall up to `sl_rebate_z0`); its floor is `sl_rebate_floor_over_joint` (1.25) above the
  joint holes; the spigot ring inside it is `sl_rebate_ring` (5.45).
* **Mass**: lumped part `servo_leads` (§6): the wire beyond the splice along `Layout.servo_lead_polyline`, + `slack`
  (50) and a board connector (`fc_connector_mass_g` 0.5) per lead at the board end, at `mass_g_per_m_ASSUMED`
  (10 g/m, weigh the wire). v0.6.3 (`Layout.has_fcu`): the last three legs, in the bay, are an estimate - up from the
  bulkhead servo hole to the FCU's top face, across to the point of its footprint nearest the hole, on to the middle
  of its top face (the servo header's position on the board is not known) → `sl_lead_mass_g` (21.5 g) at
  `sl_lead_com` (−24.2, −0.6, 245.9). Route length per lead `sl_route_len` (534, 444, 460, 549 mm), extension to
  cut `sl_ext_len` (524, 434, 450, 539 mm) - CAD report, C23 detail and `python cad/layout.py`. (v0.6 servo-lead
  session, to the tray-mounted flight computer: 27.4 g at (−4.3, −0.5, 291.5), extensions 672, 582, 598, 687 mm.) The MG90S
  lead cut off at the pod (servo placeholder mass unchanged) is ignored (conservative for the margin).
* **Why this route** (2026-10-15): (1) joining the motor-lead path - its 14 × 6 channel already carries 3 × 12 AWG
  (C20 proxy 12 × 5), the 2 × 2 servo stack needs +2.6 mm of depth in a closed, verified channel, and the signal
  leads would run ~160 mm alongside the 100 A PWM phase leads; (2) an external raceway - the fins block the
  circumference below Z 90, so four conduits up the vane section plus one along the intake into the bay (≈ 9 × 4.5
  with 1.2 walls) add ≈ 10 g of PLA near Z 190 (ΔCG ≈ −0.8 mm on top of the leads' −0.9 mm, −0.014 cal: 0.496 cal
  on the v0.5d vehicle), drag, and three module joints to bridge; (3) this internal route removes 8.0 g of PLA low
  in the propulsion module (CAD: −6110 mm³; intake −46 mm³) against the leads' 27.4 g at Z 291.5: measured on the
  CadQuery report with and without `servo_leads`, ΔCG −0.27 mm and Δ static margin −0.002 cal (v0.5d vehicle 0.510
  → 0.508; with the v0.6 COM board 0.520 → 0.518 cal, CG 377.8 → 377.5 mm) - C13 stays WARN. (These are the v0.6
  numbers; the v0.6.3 leads are 21.5 g at Z 245.9 and the whole v0.6.3 vehicle has 0.51 cal, CG Z 376.3 - C13.)
* **v0.9 (direct drive) - the C23 pod route is replaced**: each servo's lead leaves the case's upper end into the
  connector bay over the case (`dd_conn_*`: a 26 × 8 × 4.5 mm connector pair, ASSUMED), then runs inside the fin root
  in a closed tunnel Ø`sl_tunnel_d` (4.8) along a printed pipe Ø`sl_pipe_d` (7.2, wall 1.2) from the bay down-inward to
  the column's entry slot (`sl_pipe_points`; the pipe rises 33° from vertical, prints without support; a web pipe ties
  it to the fin plate, `sl_web_points`), then up the existing column / deck notch / rebate / servo channel to the FCU as
  in v0.6. The pipe walls to the duct bore, the hinge bore and the cover taps are reported in `sl_pipe_walls` (min 4.9
  mm). Extension lengths `sl_ext_len` 589 / 539 / 450 / 499 mm, lump `servo_leads` 26.8 g. The lead is unplugged at
  the bay connector to take the servo out (§3.9).

3.9 **Direct-drive vanes (v0.9, `vane_drive.mode` = `direct`, `Layout._direct_drive`).** In each fin's frame (u radial
along the vane hinge axis, v = +e_θ, Z) the MG90S output axis lies ON the hinge axis (Z = `vane_hinge_z`). Radially
outward from the hinge boss (`hinge_boss_r_out` 52): gap `boss_gap` (1), the printed `vane_coupler` - hub
Ø`dd_hub_d` (10) u [53, 61] and flange Ø`dd_flange_d` (12) u [61, 63], D-bore Ø`dd_bore_d` (5.2 = shaft + 0.2) with its
flat `dd_bore_flat` (1.6) below the axis through hub and flange - then the stock horn trimmed to a Ø12 × 2 disc (u [63,
65], screwed 2 × M2 × 4 or glued to the flange), the 5 mm spline, the case u [70, 92.5] with the ears at u [72, 74.5].
The vane shaft now ends `shaft_end_gap` (0.5) short of the flange (`vane_shaft_r_out` 60.5) and its D-flat starts
0.5 mm before the hub (`vane_d_flat_r_in` 52.5): 7.5 mm engagement. **Load path**: the hinge boss bore takes the vane's
side and axial loads; the coupler carries torque only. **Alignment**: the case floats `servo_float` (0.3) on every
side in its pocket (foam pad under the cover) so a pocket printed up to `print_tolerance_ASSUMED` (0.2) off the hinge
axis cannot side-load the coupler (C28). **Hinge moment** (`vane_drive.hinge_moment`): N (q S cl_α δ at thrust_peak and
the commanded limit) × centre-of-pressure offset up to 0.1 c (ASSUMED) + bearing friction μ 0.3 (ASSUMED) × N × shaft
r = 0.119 kgf·cm against the MG90S 1.8 kgf·cm (×15.2, side load 2.95 N).
* **Fairing** (part of `propulsion_module`, `fair_*`): a box u [`fair_u`] = [46.4, 97.4], v ±`fair_hv` (8.15), Z
  [`fair_z`] = [24.35, 67.45] around the pocket, with a 45° keel under it to the fin plate (`fair_keel_vz`, down to Z
  17.4) so it prints upright without support. **Pocket** (`dd_pocket_*`): u [52.7, 92.8], v ±6.55, Z [30.7, 65.85] -
  case + float, the coupler / horn zone, a floor slot u [71.7, 74.8], Z [25.95, 30.7] for the lower ear tab and the
  connector bay. The pocket is open on the service side (+v) below the lintel (Z ≤ `dd_open_z1` 64.25): the ceiling
  bridges the pocket width onto the 1.6 mm lintel and the lintel bridges the 40 mm opening - no support. Walls:
  `fair_walls` (inner wall to the duct 5.3, side 1.6, floor 1.6, top 1.6, outer block 4.6). The hinge bore (Ø5.4) runs
  from the duct through the fairing's inner wall into the pocket (cut AFTER the fins: the fin plate lies on the bore).
* **Servo cover** (printed `servo_cover_k`, class C): a 1.2 mm plate on the service face, u `cov_u` [47.4, 97.4], Z
  `cov_z` [24.35, 67.45], two M2 × 8 self-tapping screws along −v into Ø1.6 × 6 tap holes (`cov_screws_uz`: beside
  the hinge boss below the lead pipe, and in the outer block), Ø2.2 clearance holes; C5 checks the tap material
  (`cov_tap_walls`, min 1.5 mm) and a Ø6 × 60 screwdriver path along +v.
* **Removal**: two screws, cover off, unplug the bay connector, slide the servo (with coupler and horn on its spline)
  out along −v under the lintel - the vane stays in its bore; C5 sweeps the case, ears, horn, coupler and connector
  half out of the pocket.
* Mass: printed coupler 0.87 g and cover 3.42 g each (slicer), placeholders `servo_vane_k` (case box) and
  `servo_horn_k` (1 g), lump `vane_drive_hardware` 4 g (ASSUMED: 4 × (2 cover + 2 coupler screws + horn screw)).

## 4. Vanes, horns, linkage (printed `vane` ×4, `vane_horn` ×4; placeholders: servo, arm, pushrod)

**v0.9**: §4.2-§4.5 (horn, servo in its pod, arm, pushrod) apply to `vane_drive.mode` = `pushrod` only (KAIST preset).
With `direct` (BUDGET) the vane of §4.1 keeps its plate, edge rounds and D-flat, its shaft ends at
`vane_shaft_r_out` = `dd_u_hub1` − 0.5 and its D-flat starts at `vane_d_flat_r_in` = `dd_u_hub0` − 0.5; the printed
parts are `vane_coupler_k` and `servo_cover_k` instead of `vane_horn_k` (§3.9).

4.1 **Vane** (`vane`): flat plate, chord `vane_chord` (32) along Z, span from
r = `vane_r_in` (13) to r = `vane_r_out` (43), thickness `vane_t` (3)
(v0.2: at ±25° the old 8–44.5 span hit the duct wall at the tip and the
neighbouring vane at the root; the inner 13 mm lies in the EDF tail-cone wake
anyway),
tangential; leading edge (top) at Z = `vane_le_z` (46.6), trailing edge at
`vane_te_z` (14.6); hinge axis radial at Z = `vane_hinge_z` (37), θ = vane θ.
Leading and trailing edges rounded with radius `vane.edge_radius` (1.4, full
round = half the thickness approximately). Integral shaft: cylinder
Ø`vane_shaft_d` (5) on the hinge axis from r = `vane_shaft_r_in` (20) to
r = `vane_shaft_r_out` (59), unioned to the plate. D-flat on the shaft for
r ≥ `vane.d_flat_r_in` (55.5): remove a slab `vane.d_flat_depth` (1.0) deep on
the −Z side. Print orientation: lying flat (plate on the bed, shaft along the
bed) — the STL is exported in vehicle coordinates; the slicer rotates it.

4.2 **Horn** (`vane_horn`): plate thickness `vane.horn_t` (3) radial, r ∈
[`horn_r_in` (55.5), `horn_r_out` (58.5)], width `vane.horn_w` (8) tangential,
extending from Z = `horn_tail_z` (32) to Z = `horn_tip_z` (49) along the vane's
θ. D-bore (Ø5.4 with a matching flat) at the hinge axis — the flat sits
(`vane_shaft_d`/2 − `d_flat_depth`) + (`hinge_bore_d` − `vane_shaft_d`)/2 =
1.7 mm below the axis, i.e. the same 0.2 mm clearance as the round part;
Ø`vane.horn_pin_hole_d`
(1.6) radial hole at Z = `horn_tip_z` − 2 for the pushrod clevis.

4.3 **Servo placeholder** (per pod and the release servo, §5.4): box
`servo.body_l` × `servo.body_w` × `servo.case_top_from_bottom` (23 × 12.5 ×
22.5), bottom at `servo_bottom_z` (20.5), long side tangential, centred on the
pod θ with the body centre at v = 0, u = `servo_axis_r`; ears: plate
`servo.ear_span` × `servo.body_w` × `servo.ear_t` (32.5 × 12.5 × 2.5) with its
underside at `pod_rim_z`; output boss Ø`servo.boss_d` (6) from the case top up
to `servo_arm_z_bottom` at v = `servo_shaft_tangential_offset`.

4.4 **Servo arm placeholder**: `pod.arm_len` × `pod.arm_w` × `pod.arm_t`
(14 × 5 × 2) from the servo axis pointing radially outward (+u), Z ∈
[`servo_arm_z_bottom` (48), `servo_arm_z_top` (50)].

4.5 **Pushrod placeholder**: cylinder Ø`pod.pushrod_d` (1.5) from the arm tip
(u = `servo_arm_tip_r`, v = `servo_shaft_tangential_offset`, in the pod frame)
to the horn tip (r = (`horn_r_in`+`horn_r_out`)/2, θ = vane θ, Z = `horn_tip_z`
− 2). Report its length in the CAD report (`pushrod_len_mm`, 25.8). Fusion
builds it along +X from the origin and places it with a free-move transform
(angled-and-offset construction planes were rejected by Fusion 2705).

4.6 Kinematics assumption (used by sim/firmware): 1:1 linkage, vane deflection
δ = servo angle. Mechanical limit `vane.max_deflection_mech_deg` (25),
commanded limit `vane.max_deflection_cmd_deg` (20).

## 5. Intake, avionics, payload modules and nose

5.1 **Intake module** (`intake_module`, printed flipped (the spigot on the bed), Z `z_intake_bottom`
(200) → `z_intake_top` (315)):
* Skirt Z ∈ [200, 215]: tube OD `body_od`, ID `body_id`; 4 Ø3.2 radial holes
  at `joint_thetas`, Z = 207.5 (aligned with the propulsion spigot holes).
* Lip pocket Z ∈ [`z_intake_skirt_top` (215), `z_lip_pocket_top` (219)]: OD
  `body_od`, ID `lip_pocket_id` (110.5 KAIST lip clamp; 113.5 tab mount).
* Shelf Z ∈ [219, `z_shelf_top` (222)]: OD `body_od`, ID `shelf_id` (104). The
  EDF lip (Ø110 × 4) is sandwiched between the propulsion spigot top face and
  this shelf; thrust is carried lip → shelf → intake tube → 4 screws → spigot.
* **Windowed band** (v0.6.1, both presets; numbers in this bullet and the next two: default preset) Z ∈
  [`z_shelf_top`, `z_intake_bulkhead_bottom`] (204.9–330.9): `intake_module.window_wall_t` (3) thick, thickened
  inward - r ∈ [`r_band_in` = `body_r` − `window_wall_t`, `body_r`] (57.0–60.0), outer surface = body OD. Windows:
  `strut_count` (4) radial sectors `window_sectors` centred on `window_thetas` (strut θ + 45°), each
  `window_angle_deg` = 360°/`strut_count` − `strut_angle_deg` (`strut_w`/`body_r` rad = 17.19°) = 72.81° wide,
  Z ∈ [`z_window_bottom`, `z_window_top`] (209.9–326.9, `window_h` 117). Every strut therefore keeps `strut_w` (18) at `body_r` with radial
  side faces: inner face `strut_w_in` (17.10), horizontal section 52.65 mm². Struts at `strut_thetas` (tabs preset
  0°, 90°, 180°, 270°; KAIST 45°+k·90°). The layout raises when the band is thinner than `body.wall`, the shelf no
  longer stands out past the band by `printer.min_feature` (ledge `shelf_ledge` 5.00), a strut's inner face is below
  min_feature, the inlet ratio is short, or a lead's shelf notch / channel no longer opens the step-in through the
  band or its bulkhead hole / bundle reaches the band (`band_step_in`, `band_clear_*`: both bulkhead lead holes end
  1.00 mm inside the band face, the 2 × 2 servo bundle 2.58 mm). Before v0.6.1: the 1.2 mm tube wall, two orthogonal
  rectangular through-cuts of width `window_cut_w`, struts 14 mm - they flexed while printing and broke when the
  supports were removed (user, 2026-10-15; `intake_module.window_wall_note`).
* **Window lower end** (v0.6.2, both presets, `intake_module.window_bottom_end`): only the lower end of each window -
  its ceiling when the intake prints flipped - is a truncated gable. In the window's own frame (u along the window
  centre, y′ tangential, z) the boundary is z = `z_window_bottom` + max(0, |y′| − `bridge_w`/2)·tan(`slope_deg`):
  flat over `bridge_w` (20, bridged in print), then two planar faces rising at `slope_deg` (45°) to the struts
  (`gable_h_inner` / `gable_h_outer` 23.83 / 25.61 at the strut faces). The upper end (the print floor) stays square.
  Both generators cut each window as its radial sector ∩ the prism `window_gable_profile_yz` (extruded along u); the
  band keeps the gussets under the boundary (8315 mm³ for the four windows, 10.9 g solid). On the curved band
  surfaces the lower edge rises at cos(φ)·tan(`slope_deg`), flattest at the strut sides: `gable_edge_slope_min_deg`
  38.83° (layout raises below `GABLE_MIN_EDGE_SLOPE_DEG` 35°; the slicer's support threshold is 30°; `slope_deg`
  outside 30–60° is rejected). Lowering the windows instead was rejected (user, 2026-10-15); the static-margin cost
  given for it, ~0.02 cal per −10 mm, is NOT VERIFIED - no report on disk measures it.
* **Inlet area** (C9): `inlet_area_ratio` = the minimum open area across the band (gussets included, sampled over
  r ∈ [`r_band_in`, `body_r`]) / fan annulus ≥ `intake_module.min_inlet_area_ratio` (2.0): 31 323 mm² at r 57.00 →
  5.97 (outer surface 6.23; fan annulus 5247 mm²).
* Bulkhead Z ∈ [297, `z_intake_bulkhead_top` (300)]: full disc Ø`body_od`
  with a central Ø`cable_hole_d` (20) hole and `intake_module.vent_hole_count`
  (4) Ø`vent_hole_d` (8) holes at r = `vent_hole_r` (40), θ =
  `vent_hole_thetas` (45°+k·90°).
* Spigot Z ∈ [300, 315]: OD `spigot_od`, ID `spigot_id`; 4 Ø2.5 radial holes at
  `joint_thetas`, Z = 307.5.
* v0.5 (`edf.mount` = `tabs`): the lip pocket spans [`z_intake_skirt_top`, `z_lip_pocket_top` =
  `z_edf_lip_top` + `lip_stop_gap`] so the shelf is only a back-up thrust stop 0.5 mm above the lip
  face (pocket ID `lip_pocket_id` 113.5: 0.75 mm radial play, the tabs do not centre the bell); struts at
  `strut_thetas` (0°, 90°, 180°, 270°) so a strut sits over the wire channel.
  **Wire channel** at θ = `wire_theta` (0°): void tangential ±`wire_ch_w`/2 (14), r ∈ [`wire_ch_r0` (57.95),
  `wire_ch_r1` (63.95)], Z ∈ [`wire_ch_z0` (178.85), `wire_ch_z1` (208.5 = `z_lip_pocket_top` + `gate_h`)],
  then the **floor wedge** (solid, unioned after the cuts): the (r, z) triangle `wire_ch_floor_rz`
  ((58.8, 178.85), (64.45, 178.85), (64.45, 184.5)) extruded over the channel width, so the floor rises at 45°
  from the skirt's inner face and reaches `wire_gate_lo_z0` (184.0 = skirt top − `gate_h`) at `wire_ch_r1` —
  the intake prints flipped, and a flat floor became a ceiling the slicer filled with tree supports inside
  the closed channel. The channel opens the skirt so the leads step out of the propulsion wire slot within
  [`wire_gate_lo_z0`, `z_intake_skirt_top`], removes the pocket wall outside r = `wire_ch_r0` (a 1.2 mm wall
  stays around the lip) and the ring below the windows; plus a **shelf notch** (void, same tangential width,
  r ∈ [`wire_notch_r0` (51.5), `wire_ch_r0` + 0.5], Z ∈ [`wire_notch_z0` (= `z_lip_pocket_top`),
  `wire_ch_z1`]) so the leads step back in above the lip face; closed outside by a **bulge** (solid,
  unioned before the cuts) tangential ±`wire_bulge_w`/2, r ∈ [`body_r` − 0.6, `wire_bulge_r_out` (65.15)],
  Z ∈ [`wire_bulge_z0` (177.65), `wire_bulge_z1` (209.7)] (bottom and top closed). Only bare leads pass
  the 6 mm gates: 4 mm bullet connectors are soldered after threading (guide step 5).
  Below the lip the leads run in the propulsion wire slot (radial overlap with the channel), above the
  shelf inside the tube along the strut at `wire_theta` (0°), then through the **bulkhead wire hole** (θ = `wire_theta`,
  tangential ±`wire_ch_w`/2, r ∈ [`wire_hole_r0` (48), `wire_hole_r1` (56)]).
* v0.6 **servo channel** (`Layout.servo_leads`, §3.8) at θ = `sl_gather_theta` (180°, on a strut; ≥ 10 mm of arc
  from the motor bulge): the motor-lead channel's pattern with its own width and depth, wall and `gate_h` from
  `intake_module.wire_channel`. **Bulge** (solid, unioned before the cuts) tangential ±`sl_bulge_w`/2 (11.4), r ∈
  [`body_r` − 0.6, `sl_bulge_r_out` (62.75)], Z ∈ [`sl_bulge_z0` (181.05), `sl_bulge_z1` (209.7)]; **channel** (void)
  ±`sl_ch_w`/2 (9.0), r ∈ [`sl_ch_r0` (= `wire_ch_r0`, 57.95), `sl_ch_r1` (61.55)], Z ∈ [`sl_ch_z0` (182.25),
  `sl_ch_z1` (208.5 = `z_lip_pocket_top` + `gate_h`)]; **floor wedge** (solid, unioned after the cuts) the triangle
  `sl_ch_floor_rz` ((58.8, 182.25), (62.05, 182.25), (62.05, 185.5)) over the channel width - it reaches
  `sl_gate_lo_z0` (= `sl_rebate_z0`, 185.0) at `sl_ch_r1`; **shelf notch** (void) ±`sl_ch_w`/2, r ∈ [`sl_notch_r0`
  (51.5), `sl_ch_r0` + 0.5], Z ∈ [`z_lip_pocket_top`, `sl_ch_z1`]; **bulkhead servo hole** ±`sl_ch_w`/2, r ∈
  [`sl_hole_r0`, `sl_hole_r1`] (50–56). The leads step out of the propulsion rebate within [`sl_gate_lo_z0`,
  `z_intake_skirt_top`] (5 mm for the 3.6 mm ribbon), climb the channel outside the 1.2 mm pocket wall as a 2 × 2
  stack (7.2 × 2.6), step in above the lip face through the notch and run along the strut at 180° (cable ties) to
  the hole, then up onto the FCU board (§5.2). Like the motor leads they pass the gates bare: the board connectors
  are fitted after threading.

**5.1 v0.9 - open motor-lead groove (`intake_module.wire_channel.open`, decision 3).** The closed v0.5d channel (bulge,
floor wedge, 6 mm leads stripped of their connectors) is replaced so the three 12 AWG leads (OD 3.6 ASSUMED) pass
with their 5.5 mm bullets (sleeve OD 8 ASSUMED) one at a time:
* Propulsion wire slot `wire_slot_w` 14 → 22 mm (both boxes, §3.7). The EDF lead exit (`edf.wire_exit`, θ −90° from a
  tab, Z 47.4-60.3 from the lip front) is READ FROM THE VENDOR DRAWING - ASSUMED until measured on the unit.
* Intake groove at `wire_theta`: open to the outside, r [`wire_ch_r0`, `wire_ch_r1`] = [57.95, 68.95] (11 deep), 22
  wide, Z from the intake bottom to the shelf top (`wire_groove_z1` 204.9); 4 mm side walls outside `body_r`
  (`wire_side_wall`) from the intake bottom to `wire_ch_z1` 208.5, a 45° roof wedge over them (`wire_roof_rz`, to Z
  219.25, 9.9 mm under the gable window boundary at its edge, `wire_bulge_window_clear`), two pairs of cover-screw pads
  (Z 179 / 204.5). The gate where the leads step in above the lip is `wire_gate_w` 14 wide (Z [202.5, 208.5]); it ends
  3.1 mm inside the strut (`wire_gate_in_strut`) - 22 mm put the lip-pocket wall's top edge under the gate beside the
  strut and failed the window support check (measured).
* Printed `wire_cover` (class B, 1.6 mm): closes the groove outside r1 over Z [175, 209.7], 4 × M2 × 8 self-tapping
  into Ø1.6 × 6 taps in the pads (`wire_cov_walls` ≥ 1.2).
* Bulkhead wire hole 22 × 11 (r `bulkhead_hole_r` [41, 52]); the joints sit in the avionics bay in
  `ml_joint_box` (u [42, 54.5], v ±12.5, Z from 2 mm above the bulkhead, 44 mm tall), clear of every placeholder by ≥ 2
  mm (`ml_joint_clearance`, min ESC 7.0).
* Lead route (`ml_route_uz`, `motor_lead_polyline`): exit → slot → groove → gate → up the 0° strut → bulkhead hole →
  joint zone: 218 mm + 60 slack = 278 mm per lead; lump `motor_leads` 42.9 g (37 g/m ASSUMED + 3 bullet pairs 4 g
  ASSUMED) at `ml_com`. `ml_sections` lists, per passage, the width / height needed for the three leads side by side and
  for one bullet with 1 mm clearance against what is built.

5.2 **Avionics module** (`avionics_module`, Z `z_avionics_bottom` (300) →
`z_avionics_top` (515)):
* Tube OD/ID `body_od`/`body_id` for Z ∈ [300, `z_avionics_spigot_bottom`
  (500)] with the §2 shoulder ring at Z ∈ [498, 500], spigot Z ∈ [500, 515]
  (OD `spigot_od`, ID `spigot_id`, Ø2.5 holes at Z = 507.5). Ø3.2 holes at
  Z = 307.5 (over the intake spigot).
* Tray rails: two pairs of ribs on the inner wall at (x = `tray_x` ±
  (`rail_gap`/2 + `rail_w`/2), y = ±`rail_y`) — i.e. at each of the two wall
  points where the plane x = `tray_x` (25) meets the bore, two ribs
  `rail_w` (2) wide (in x) × `rail_depth` (5) deep (radially inward), Z ∈
  [`z_rail_bottom` (318), `z_rail_top` (498)], leaving a slot of width
  `rail_gap` (3.4) centred on x = `tray_x` for the 3 mm tray. The ribs are
  embedded into the wall and **clipped at `body_r`** (nothing may protrude
  outside the tube; v0.2 fix).
* Vent slots: `avionics_module.vent_slot_count` (4) rectangular through-cuts
  (radial) `vent_slot_w` (8) wide × `vent_slot_h` (40) tall at θ =
  `vent_slot_thetas` (60°, 120°, 240°, 300°), Z ∈ [`z_vent_slot_bottom` (430),
  +40].
* **Tray** (`avionics_tray`, separate printed plate): x ∈ [`tray_x` −
  `tray_t`/2, `tray_x` + `tray_t`/2] (23.5–26.5), y ∈ ±`tray_w`/2 (±50), Z ∈
  [`z_tray_bottom` (320), `z_tray_top` (495)]. Features: board holes (v0.6.3: the NAV board; the params keys keep
  their v0.5 names) — 4 × Ø`fc_hole_d` (3) on a `fc_hole_pattern` (44) square centred at (y = 0,
  Z = `nav_center_z`, the former `fc_center_z` (465)); battery strap slots — 4 slots `strap_slot_w` ×
  `strap_slot_h` (20 × 3) at y = ±40, Z = `battery_z[0]` + 25 and
  `battery_z[1]` − 25; ESC strap slots — 2 slots at y = ±35, Z = mid-ESC. Each slot is `strap_slot_w`
  along Z by `strap_slot_h` along y (a horizontal strap round the Z-long battery; v0.5d - the Fusion script
  had them 90° turned).
  Printed flat.
* Placeholders (+x face of the tray): ESC box `esc_x/y/z` (Z 330–430), **NAV board** `nav_x/y/z` (v0.6.3,
  `electronics.nav`, formerly the flight computer `fc`: same box at the same spot, 50 × 50 × 15, Z 440–490;
  ATmega128A minimum-system board with the GY-521 MPU-6050 and barometer modules on a 4-ch level shifter, so the
  IMU stays where the v0.5 FC had it; barometer part: BMP280 module, user decision 2026-11-04, instead of the GY-63
  MS5611 - since the 2026-11-10 control round the default NAV driver is the BMP280's, `hal/bmp280.*`; the MS5611 driver is a
  build option, env `nav_ms5611`). (−x face): battery box `battery_x/y/z`, misc-electronics box `misc_x/y/z`
  (default preset: 20 (x) × 40 (y) × 30 (z), Z 362.9–392.9, below the battery because
  `avionics_module.battery_above_misc` (v0.3); 5 V logic UBEC, buzzer, arming switch, wiring - v0.6.2: the LoRa
  module, OpenLog and radio level shifter moved to the radio/log box of §5.3, `electronics.misc.mass_g` 60 → 40).
* **FCU board** (v0.6.3, `electronics.fcu`, both presets; numbers: default preset): 50 × 50 × 15 lying flat on
  the intake bulkhead's top face, inside the intake spigot bore below the tray, centred on the axis with its edges
  along x / y, on `foam_t` (3) double-sided foam tape plus a cable tie through two bulkhead vent holes
  (`fcu_x/y/z`: Z 335.9–350.9). The tie runs 45°/225° and 135°/315° cross the board (`fcu_tie_pairs`); all four
  vent holes stay uncovered (0.64 mm in plan). The board covers the centre cable hole - nothing is routed through
  it (motor leads: bulkhead wire hole, servo leads: servo hole); bulkhead under the board 2186 mm² (87.4 % of the
  footprint) for the tape. The layout raises below 1 mm to the spigot bore or below
  `payload_module.com_mount.min_clear_to_tray` (2) to the tray, the tray placeholders, the rail ribs, the tray stops
  and, in plan, the motor-lead and servo-lead bulkhead holes (measured: tray 2.00, ESC 12.04, misc 12.00, motor-lead
  hole 23.0, servo-lead hole 25.0, spigot bore 21.64), and when no tie run crosses the board. The intake-avionics
  joint screws (Z 340.4, within the board's height) have 21.64 mm of radial room in the spigot bore before they
  reach it. The vane-servo leads end on it (§3.8). Mass 20 g ASSUMED (weigh it). Check C25.
* v0.5d **tray stop** (both presets): in each rail slot a block between the two ribs of the pair, same radial
  extent as the ribs, Z ∈ [`z_tray_stop_bottom` (= `z_tray_bottom` − `tray_stop_h`), `z_tray_bottom`]; the tray
  rests on it at its design height (before v0.5d nothing held it and it slid 20 mm down onto the intake
  bulkhead, lowering the CG by ~4.7 mm and closing the space the motor leads use).


5.3 **Payload module** (`payload_module`, Z `z_payload_bottom` (500) →
`z_payload_top` (751)):
* Tube OD/ID `body_od`/`body_id` Z ∈ [500, `z_payload_spigot_bottom` (736)]
  with the §2 shoulder ring at Z ∈ [734, 736]; Ø3.2 holes at Z = 507.5; top
  spigot Z ∈ [736, 751] (no screw holes — the nose skirt is latched).
* Bulkhead Z ∈ [`z_payload_bulkhead_bottom` (515), `z_payload_bulkhead_top`
  (518)]: disc filling the bore, with a central spring-seat boss
  Ø`spring_seat_boss_d` (30) × `payload_module.spring_seat_boss_h` (8) on top,
  and `bulkhead_hole_count` (4) Ø`bulkhead_hole_d` (6) holes at r =
  `bulkhead_hole_r` (40), θ = 45°+k·90°.
* Spring placeholder: tube OD `spring_od` (40), wall = `spring_wire_d` (1.6),
  Z ∈ [`z_spring_bottom` (518), `z_spring_top` (532)].
* **Piston** (`piston`, printed): disc Ø`piston_d` (115) × `piston_t` (4),
  Z ∈ [`z_piston_bottom` (532), `z_piston_top` (536)], with a recess
  Ø`piston_recess_d` (41) × `piston_recess_depth` (1.5) on its underside.
* Payload envelope (check volume, not a part): cylinder Ø`envelope_d` (110),
  Z ∈ [`z_envelope_bottom` (536), `z_envelope_top` (736)]. CanSat placeholder:
  can Ø`can_d` (83) Z ∈ [`z_can_bottom`, `z_can_top`] plus chute block Ø`can_d`
  up to `z_chute_pack_top` (see `payload.chute_pack_h`); total mass
  `payload.mass_g`.
* **Release servo pod**: same cup as §3.4 but on the body tube: u ∈
  [`release_pod_r_in` (58.5), `release_pod_r_out` (76.5)], cavity u ∈
  [`release_pod_cavity_r_in` (61), `release_pod_cavity_r_out` (74.5)], Z ∈
  [`release_pod_z0` (711), `release_pod_rim_z` (731)], θ = `release_pod_theta`
  (12°), with the 45° underside chamfer and the two ear holes. Servo
  placeholder with bottom at `release_servo_bottom_z` (713), axis at u =
  `release_servo_axis_r` (67.75), v = `servo_shaft_tangential_offset`.
* **Latch arm** (`latch_arm`, printed): `latch_arm_len` × `latch_arm_w` ×
  `payload_module.latch_arm_t` (10 × 6 × 3) with a Ø6 hub at the servo axis,
  Z ∈ [`z_latch_arm_bottom` (740.5), `z_latch_arm_top` (743.5)]. Latched
  position: the arm points from the servo axis toward the lug centre
  (r = (`lug_r_in`+`lug_r_out`)/2, θ = `latch_lug_theta`); its tip stays
  outside the nose skirt (a 14 mm arm reached r ≈ 54 and cut into it).
  Released: rotated `mission.deploy_servo_travel_deg` (90°) away from the lug.
* **COM-board mount** (v0.6, `payload_module.com_mount`, both presets; numbers: default preset): four bosses
  Ø`boss_d` (7) × `boss_h` (3) on a `pattern` (44) square centred on the axis, edges along x / y (`com_boss_xy`
  ±22), grow down from the bulkhead underside (Z [`z_com_boss_tip`, `z_com_boss_top`] = 529.9–532.9). Each has a
  blind Ø`hole_d` (2.5) hole `hole_depth` (3.8) deep from its tip (`z_com_hole_end` 533.7): the bulkhead keeps
  `com_hole_skin` (1.2 = min_feature) over the hole, so an M3 × 5 self-tapping screw driven from below through the
  1.6 mm board (3.4 mm of thread) cannot reach the spring side; layout raises when the hole would open. The COM
  board box (`electronics.com.dims_lwh` 50 × 50 × 15, components hanging down - no straight pin headers on that
  side) hangs from the boss tips into the avionics spigot bore (`com_board_x/y/z`, Z 514.9–529.9). The layout raises
  when it leaves < 1 mm to the spigot bore, < `min_clear_to_tray` (2) to the tray, battery, ESC, NAV, misc, FCU or a
  rail rib, a boss < min_feature from a bulkhead hole, or the board covers a bulkhead hole in plan (measured: tray
  2.00, battery 5.00, NAV 7.07, spigot bore 21.64, boss edge → hole 2.39, board → hole 1.64). The bosses print with
  short supports inside the skirt (§12). Mass 20 g ASSUMED (weigh it). Check C22.
* **Radio/log box** (v0.6.2, `electronics.radio_log`, both presets): the LoRa module + antenna, the OpenLog +
  microSD, the second BSS138 4-ch shifter (radio signals) and the second AMS1117-3.3 (radio 3.3 V), moved from the
  tray's misc box into one box `dims_lwh` (30 × 20 × 15) centred at r `r_center` (40) on θ `theta_deg` (−90°),
  hanging from the payload-bulkhead underside beside the COM board (`radio_log_x/y/z`: Z 517.9–532.9), held by
  double-sided foam tape and a cable tie; the antenna hangs vertically along the avionics wall, away from the ESC and
  the motor leads. Reason (params note, user 2026-10-15): it raises the CG for static margin; the margin change of
  this move alone is not reported separately (NOT VERIFIED). The layout raises below 1 mm to the spigot bore, below
  `min_clear_to_tray` (2) to the COM board, the COM bosses (plan), every joint-screw axis (Ø`joint_hole_outer_d`,
  a screw of any length), the tray, its placeholders and the rail ribs, and when it covers a bulkhead hole
  (measured: spigot bore 4.80, COM board 5.00, COM bosses 7.13, joint-screw axes 9.01, nearest rail 7.09). Mass
  20 g ASSUMED (weigh it). Check C24.

5.4 **Nose cone** (`nose_cone`, printed base-down):
* Skirt Z ∈ [`z_nose_skirt_bottom` (736), `z_nose_skirt_top` (751)]: tube OD
  `body_od`, ID `nose_skirt_id` (116.0) — slides over the payload spigot.
* **Latch lug** on the skirt exterior at θ = `latch_lug_theta` (0°): block
  r ∈ [`lug_r_in` (60), `lug_r_out` (66)], tangential width `lug_w` (10),
  Z ∈ [`z_lug_bottom` (736), `z_lug_top` (740)]. The latch arm sits
  `payload_module.latch_arm_clearance_z` (0.5) above it; the spring load is
  axial on the arm (no torque on the servo).
* Body: revolve of the Haack profile `Layout.nose_profile_points()` (outer
  radius `nose_outer_radius(x)`, x from the base at `z_nose_base` (751) to the
  tip at `z_nose_tip` (941)), then shelled to a **constant wall `nose_wall` (2)
  measured along the surface normal**, base face open. Fusion: ShellFeature
  removing the base face. CadQuery: subtract the revolve of the normal-offset
  inner profile (`nose_inner_points`), followed to x = L − 8 and then closed to
  the axis so the last 2 mm of the tip stay solid (v0.3: 2 mm matches the Fusion shell within 0.1 %); its first point is pinned to
  (`body_r` − wall, `z_nose_base`) because a 7 nm numerical sliver against the
  skirt bore made the STL non-watertight. (v0.1's radial offset r_out − wall
  gave a wall thinner than 2 mm on the taper.) Tip blunting is not applied.
* Anchor bar: `nose.anchor_bar_w` × `anchor_bar_h` (8 × 4) spanning the bore as
  a chord through the axis along X at Z = `z_anchor_bar` (755), unioned to the
  wall; a Ø`nose.tether_eye_d` (4) vertical hole at its centre (tether eye)
  leaves 2 mm of material each side.
* Chute pack placeholder: cylinder Ø`chute_pack_d` (80) × `chute_pack_h` (70)
  Z ∈ [`z_chute_pack_bottom` (765), `z_chute_pack_top` (835)]; mass =
  `recovery.chute_mass_g` + `lines_swivel_mass_g` + `shock_cord_mass_g`.
  Cavity volume `nose_cavity_volume_L` (0.90) ≥ `recovery.chute_pack_volume_L_required` (0.7).

5.5 **Launch pad** (`launch_pad`, separate, printed plate-down): annular plate
OD `pad_plate_od` (190), ID `pad_plate_id` (102), thickness `pad_plate_t` (5),
Z ∈ [−5, 0] (the fins' trailing edges rest on its top face at Z = 0, supported
from r 51 to 95); `launch_pad.leg_count` (4) square legs `leg_size` (20) at
r = `launch_pad.leg_r` (80), θ = `pad_leg_thetas` (45°+k·90°, under the fins),
Z ∈ [−5 − `pad_leg_h`, −5]; feet `foot_size`² × `foot_t` under each leg;
fin-locator ribs: for each fin θ, two ribs `fin_locator_rib_w` ×
`fin_locator_rib_h` × `fin_locator_rib_len` (3 × 4 × 36) on the plate top from
r = `fin_locator_rib_r_in` (55), offset tangentially ±(`fin_locator_gap`/2 +
`rib_w`/2) from the fin plane. (v0.2: the P2S profile reserves a
wrap-detection zone at x 153–216, y 235–256 mm, and Bambu Studio refuses parts
that come within ≈ 10 mm of it; a 240 mm plate cannot be sliced, 190 mm can.)

## 6. Placeholders, masses, mass properties

Non-printed items are modelled as simple solids ("placeholders") in their own
components so that the CAD mass properties are the vehicle's. Each placeholder
gets density = mass / volume. Printed parts use `printer.density_g_cm3`.

| component | geometry | mass (g) |
|---|---|---|
| `edf_unit` | lip disc Ø`edf.lip_od` × `edf_lip_t` at Z [`z_edf_lip_bottom`, `z_edf_lip_top`]; housing ring OD `edf.housing_od`, ID `edf.housing_id`, Z [`z_edf_housing_bottom`, `z_edf_lip_bottom`]; hub cylinder Ø`edf.hub_d_ASSUMED` from `z_edf_housing_bottom` to `z_edf_housing_bottom` + 70. v0.5 tabs: the lip is a ring ID `edf.housing_id` (envelope of the bell, Ø`edf.lip_od` × `edf.lip_depth`), plus the motor cylinder Ø`edf.motor_d` for `edf.motor_protrusion_rear` below the housing and the two tab plates (§3.7) with their three slots | `edf.mass_g` |
| `battery` | box `battery_x/y/z` | `battery.mass_g` |
| `esc` | box `esc_x/y/z` | `electronics.esc.mass_g` |
| `nav` (v0.6.3; `fc` until v0.6.2) | box `nav_x/y/z`: the NAV board on the tray's +x face above the ESC, the former FC box (§5.2) | `electronics.nav.mass_g` |
| `fcu` (v0.6.3) | box `fcu_x/y/z`: flat on the intake bulkhead top on `electronics.fcu.foam_t` foam (§5.2) | `electronics.fcu.mass_g` (ASSUMED) |
| `com` (v0.6) | box `com_board_x/y/z`: hangs from the four `com_mount` boss tips under the payload bulkhead (§5.3) | `electronics.com.mass_g` (ASSUMED) |
| `radio_log` (v0.6.2) | box `radio_log_x/y/z`: centred at `r_center` on `theta_deg`, top = payload bulkhead underside (§5.3) | `electronics.radio_log.mass_g` (ASSUMED) |
| `misc_electronics` | box `misc_x/y/z` on the tray's −x face: Z [`z_tray_bottom` + 10, + length] below the battery when `avionics_module.battery_above_misc` (default preset), else Z `battery_z[1]` + 8 upward (§5.2) | `electronics.misc.mass_g` |
| `servo_vane_1..4`, `servo_release` | §4.3 | `servo.mass_g` each |
| `servo_arm_1..4`, `pushrod_1..4` | §4.4, §4.5 | 1 g each (reported, negligible) |
| `cansat_payload` | can + chute block (§5.3) | `payload.mass_g` |
| `spring` | §5.3 | 0.5 × `hardware_mass_g.spring_piston_hardware` |
| `chute_pack` | §5.4 | chute + lines + cord |
| lumped (no body) `joint_screws_inserts` | point mass at the vehicle axis, Z = mean of the four joint Z's | `hardware_mass_g.joint_screws_inserts` |
| lumped (no body) `pushrods_horns_clevises` | point at Z = `vane_hinge_z` | `hardware_mass_g.pushrods_horns_clevises` |
| lumped (no body) `servo_leads` (v0.6, only with `Layout.servo_leads`) | point mass at `sl_lead_com` (§3.8, off the axis) | `sl_lead_mass_g` |
| `servo_horn_1..4` (v0.9 direct) | Ø`dd_horn_d` × `vane_drive.horn.t` disc on the hinge axis, u [`dd_u_flange1`, `dd_u_horn1`] (§3.9) | `vane_drive.horn.mass_g` (1) |
| lumped (no body) `vane_drive_hardware` (v0.9 direct, replaces `pushrods_horns_clevises`) | point on the axis at the fairing's mid Z | `vane_drive.hardware_g_ASSUMED` (4) |
| lumped (no body) `motor_leads` (v0.9, `motor_leads` in params) | point at `ml_com` (§5.1) | `ml_mass_g` (42.9, ASSUMED wire / bullet masses) |

**Printed masses (v0.9, `printer.mass_model`)**: mode `slicer_part_g` (BUDGET) - each printed part's report mass is the
grams Bambu Studio extrudes for it (G-code part extrusion, supports / brim excluded) when sliced with its print class
(`printer.part_class` → `printer.print_classes` A-E: walls, top / bottom layers, infill; docs/print_mass_calibration.json),
from the table `printer.mass_model.table` written by `cad/cadquery/make_v09_print_files.py`. A table entry counts only
when its `solid_volume_mm3` equals the part's solid volume (1e-6 relative); otherwise the class's
`effective_density_fallback_g_cm3` is used and the report warns. The effective density (mass / solid volume) also scales
the inertia; volume, COM and bbox are the solid's. Mode `solid` (KAIST) keeps `printer.density_g_cm3` and the v0.8
report entries bit for bit. `extra.mass_model` lists per part the model, solid mass and class.

**Report** (`cad/exports/<generator>_report.json`, identical schema for both
generators):

```json
{
  "generator": "fusion360" | "cadquery",
  "generator_version": "...",
  "preset": "KAIST_90mm_12S",
  "params_sha256": "...",            // sha256 of params.json bytes
  "parts": {
    "<name>": {"kind": "printed" | "placeholder" | "lumped",
               "volume_mm3": 0.0, "density_g_cm3": 0.0, "mass_g": 0.0,
               "bbox_mm": [[xmin, ymin, zmin], [xmax, ymax, zmax]],
               "com_mm": [x, y, z],
               "inertia_about_com_kg_m2": [[Ixx, Ixy, Ixz], [Ixy, Iyy, Iyz], [Ixz, Iyz, Izz]]}
  },
  "totals": {"mass_g": 0.0, "printed_mass_g": 0.0, "com_mm": [x, y, z],
             "inertia_about_com_kg_m2": [[...]],
             "post_ejection": {"mass_g": 0.0, "com_mm": [...], "inertia_about_com_kg_m2": [[...]]}},
  "derived": {"pushrod_len_mm": 0.0, "nose_cavity_volume_L": 0.0, "footprint_d_mm": 0.0,
              "vehicle_h_mm": 0.0, "inlet_area_ratio": 0.0},
  "warnings": ["..."]
}
```

`post_ejection` excludes `cansat_payload`, `nose_cone`, `chute_pack` (the chute
is deployed; its mass hangs on the cord — exclude for inertia, include for the
descent mass separately as `descent_mass_g`).

Fusion mass-property recipe (lengths cm internally): for each body,
`pp = body.getPhysicalProperties(adsk.fusion.CalculationAccuracy.HighCalculationAccuracy)`;
`V_mm3 = pp.volume * 1000`; `com_mm = pp.centerOfMass * 10`;
`ok, xx, yy, zz, xy, yz, xz = pp.getXYZMomentsOfInertia()` returns kg·cm²
about the **world origin** using the body's *current* material density
`pp.density` (kg/cm³). Scale by `(rho_g_cm3 / 1000) / pp.density`, convert to
kg·m² (× 1e-4), then move to the body's COM with the parallel-axis theorem,
then sum to the total COM. Fusion's sign convention for the products of
inertia must be verified on a test cube before trusting off-diagonal terms;
only the diagonal terms are used downstream.

## 7. Checks (CadQuery `check_geometry.py`; Fusion run compared against it)

Each check reports PASS/FAIL/WARN (N/A for a tab check on a lip_clamp preset) with the measured number.
Checks that need positive evidence (overlap ≥ x, material present) treat an uncomputable boolean as 0.
All geometric "no intersection" checks use boolean intersection volume < 1 mm³; a boolean that OCC
cannot evaluate counts as a violation (no evidence is never PASS).

| id | check |
|---|---|
| C1 | every printed part's bbox fits 256−2·`usable_margin` in XY and 256−`usable_margin` in Z in its print orientation (tubes/propulsion/nose upright; vanes/tray/piston/horn/arm flat; pad plate-down) |
| C2 | payload envelope cylinder does not intersect any printed body |
| C3 | `edf_unit` does not intersect `propulsion_module` or `intake_module`; and the EDF cannot pass upward: the shelf annulus overlaps the lip in plan (ID `shelf_id` < `edf.lip_od`) |
| C4 | each vane at δ = 0 and δ = ±`max_deflection_mech_deg` (rotated about its hinge axis) does not intersect `propulsion_module` or another vane; vane shaft passes through the bore (shaft ∩ module = 0) |
| C5 | horns, servo placeholders, arms and pushrods do not intersect `propulsion_module`, fins or each other (pushrod ∩ pod = 0) |
| C6 | adjacent modules do not intersect (spigot/skirt lap clearance); nose skirt ∩ payload module = 0 |
| C7 | battery/esc/nav/misc (v0.6.3: `fc` is `nav`) ∩ (tube ∪ tray) = 0; tray ∩ tube = 0; tray lies within the rail slot (tray bbox x within `tray_x` ± `rail_gap`/2); v0.5d: in each rail slot a 0.1 mm slab under the tray footprint is ≥ 95 % avionics_module material whose top is `z_tray_bottom` (± 0.05) - the tray stands on the stop (a missing stop is FAIL) |
| C8 | exhaust path: cylinder Ø(`duct_id` − 2) from Z = 0 to `z_edf_housing_bottom` intersects nothing but vanes/shafts and (tabs) the `edf_unit` motor/hub (∩ `propulsion_module` = 0) |
| C9 | `inlet_area_ratio` ≥ `min_inlet_area_ratio`. v0.6.1: judged on the MINIMUM opening of the built band (thin cylindrical probe shells across it) and the Layout minimum at `r_band_in`; solid evidence: every window clear through (inset probe ∩ intake < 0.01 mm³), every strut's mid-height section ≥ 0.95 × the design sector and `window_wall_t` deep, the band `window_wall_t` thick and solid in the margin rings. v0.6.2: the windows are probed clear above the lower boundary only; the gussets under it are solid (void < 0.01 mm³), the bridge is flat at `z_window_bottom`, and every lower-boundary edge on the outer and inner band surface (BRep edges) is flat within the bridge or ≥ `GABLE_MIN_EDGE_SLOPE_DEG` (35°) from horizontal, spanning the full gable height on both sides of every window (printability evidence) |
| C10 | minimum designed wall/feature ≥ `printer.min_feature` (list every wall parameter used). Listed since v0.6: the servo-route walls (pod window roof and end wall, entry slot → web, deck notch → joint hole, deck outside the notch corners, rebate floor over the joint holes, spigot ring in the rebate, servo bulge walls / floor / roof), the COM boss wall beside its hole ((`boss_d` − `hole_d`)/2) and the bulkhead skin over the blind hole; v0.6.1: `window_wall_t`, the strut inner face `strut_w_in` and the shelf ledge past the band; v0.6.2: the gussets (no new minimum feature: full band thickness, their thin edge lies on the ring). Derived features below min_feature are WARN, not FAIL |
| C11 | nose cavity volume (solid minus shelled volume, above the anchor bar) ≥ `chute_pack_volume_L_required`; `chute_pack` ∩ `nose_cone` = 0 |
| C12 | total mass vs `edf.thrust_nominal_N`: T/W ≥ 1.30 → PASS, 1.15–1.30 → WARN, < 1.15 → FAIL |
| C13 | CG height above `vane_hinge_z` ≥ 250 mm; Barrowman CP (fins + nose) below CG for the coast configuration (static margin ≥ 1.0 body diameters → PASS, ≥ 0.5 → WARN) |
| C14 | every placeholder lies inside the body bore of its module (bbox check); includes `nav`, `fcu`, `com` and `radio_log` |
| C15 | latch arm in the latched position overlaps the lug in XY projection by ≥ 4 mm and does not intersect lug/skirt; released position has zero XY overlap |
| C16 | joint screw holes: for each joint and each θ, a thin probe cylinder Ø2 along the hole axis intersects neither the outer tube nor the spigot (holes exist and align); v0.5d: the propulsion holes are blind - the probe starts 0.2 mm past the hole bottom and the floor from the EDF bore to the hole bottom is ≥ 95 % solid (a through hole when `prop_joint_hole_depth` is set is FAIL) |
| C17 | every preset builds (`meta.preset` plus every key of `alt_presets`) without exceptions |
| C18 | (tabs) each tab slides in: the tab swept from `tab_z0` up through `keyway_z1` ∩ `propulsion_module` = 0, and the seated tab ∩ (`propulsion_module` ∪ `intake_module`) = 0; the keyway floor touches the tab (gap < 0.05 mm) and carries it: module material in a 0.1 mm slab under the tab footprint ≥ 0.95 × `tab_t` × (`tab_r_out` − `edf_bore_r`) |
| C19 | (tabs) every tab screw (the Ø6.0 tunnel probe must also cross where the skin is, ∩ skin annulus > 1 mm³, i.e. the tunnel really reaches the outside): a Ø3.0 probe along the axis for t ∈ [`tab_screw_t_keyway`, `tab_screw_t_head`] and a Ø6.0 probe for t ∈ [`tab_screw_t_head` + 0.2, `tab_screw_t_cbore_end` + 2] ∩ `propulsion_module` < 0.01 mm³ for the tunnel (a clearance, not the 1 mm³ tolerance) and < 1 mm³ for the hole (clear hole, and the button head enters straight from outside); a Ø2.0 probe for t ∈ [`tab_screw_t_tap_end`+0.2, −`tab_screw_t_keyway`] ∩ module = 0 and a Ø3.2 probe there ∩ module ≥ 0.95 × π/4 (3.2² − 2.5²) × length (material all round to self-tap); the axis passes through the tab slot: a Ø3.0 probe for \|t\| ≤ `tab_t`/2 ∩ `edf_unit` = 0 AND a probe of Ø(`tab_slot_w` + 1.0) there ∩ `edf_unit` > 0 (the slot is really where the screw goes) |
| C20 | (tabs) wire path: a 12 (tangential) × 5 (radial) mm proxy bundle with its inner face at r = (`edf_bore_r` + `sleeve_r`)/2, swept from Z = `wire_slot_z0` up the propulsion wire slot, stepping out within [`wire_gate_lo_z0`, `z_intake_skirt_top`] to r ≥ `wire_ch_r0`, up past the lip, stepping in within [`wire_notch_z0`, `wire_ch_z1`] and through the bulkhead wire hole, ∩ (`propulsion_module` ∪ `intake_module` ∪ `edf_unit`) = 0; and the channel is closed outside: the bulge's outer wall, side walls, floor wedge (triangle `wire_ch_floor_rz` over the channel width) and roof are solid (missing < 0.5 mm³ each) and the intake's outer 0.3 mm over [`z_intake_bottom`, `z_window_bottom`] has no opening except the joint holes and the channel behind the bulge (v0.6: and the servo channel behind the servo bulge, §5.1). v0.6.1: the upper gate is taken to the band's inner face, and the leg that runs up beside the strut keeps a BRep gap > 0 to the windowed band |
| C21 | (tabs) outer skin and slivers, measured on the solid with radial probes: material between each keyway (centre and corners) and the skin outer surface ≥ 0.8 mm over the keyway height (v0.6 with `Layout.servo_leads`: up to `sl_rebate_z0` - above it the servo-lead rebate opens the keyway's outer wall on purpose, `sl_rebate_over_tab` above the tab tops); skin thickness beside the wire slot ≥ `skin_min_wall` and the skin sector across the slot width (+1°) over [`wire_slot_z0`, `z_prop_spigot_bottom`] intact (void < 1 mm³); the screw lugs lie inside the cylindrical section (`block_z0` ≥ `z_cyl0`) |
| C22 | (v0.6, `electronics.com` + `payload_module.com_mount`) COM board under the payload bulkhead, on the solids: BRep clearances (uncomputable → 0 → FAIL) ≥ `com_mount.min_clear_to_tray` to the tray, battery, ESC, NAV, misc and the rail ribs, ≥ 1 mm to the avionics module and radially to the spigot ID; seated (com ∩ payload_module < 1 mm³ and the lowest payload material over the board is the boss tip, \|gap\| < 0.05); each boss ring and the floor above each blind hole ≥ 95 % material; a probe Ø(`hole_d` − 0.1) in each blind hole ∩ payload < 0.01 mm³; the bulkhead hole columns stay open. N/A without the COM board |
| C23 | (tabs, `Layout.servo_leads`, v0.6) vane-servo lead route (§3.8, §5.1), ribbon proxies `sl_ribbon_w` × `sl_ribbon_t` placed 0.2 mm off the walls they follow: per lead W through the pod window (flat, from the case end face), O up outside the pod's far face to `sl_cross_z1`, X the crossing corridor (annular sector at [`sl_cross_z0`, `sl_cross_z1`] from O's corners to the column's slot edge), S up the entry slot to `sl_entry_z_open`, C the straight column (parallelogram to the deck-notch centre at the deck bottom), N up the deck notch into the rebate band; gathered: R− / R+ two-ribbon layers along the rebate from the outermost column on each side to `sl_gather_theta`, then the 2 × 2 bundle - B steps out within [`sl_gate_lo_z0`, `z_intake_skirt_top`], U up the servo channel, D steps in through the shelf notch, E up the strut through the bulkhead servo hole. Every proxy ∩ (`propulsion_module` ∪ `intake_module` ∪ `edf_unit`) < 1 mm³ and W/O/X ∩ (servos, arms, pushrods, horns) < 1 mm³; the servo bulge's outer wall, side walls, floor wedge and roof, the deck outside every notch (the skirt seat) and the sleeve at every column are solid (missing < 0.5 mm³ each). N/A without the route. v0.6.1: leg E keeps a BRep gap ≥ `servo_leads.min_clearance` to the windowed band (2.58 mm) |
| C24 | (v0.6.2, `electronics.radio_log`) radio/log box under the payload bulkhead, on the solids: BRep clearances ≥ `com_mount.min_clear_to_tray` to the tray, battery, ESC, NAV, misc, COM board, rail ribs, the payload material below the bulkhead (COM bosses, skirt) and every joint-screw axis proxy (Ø`joint_hole_outer_d` from the axis to `body_r`: a screw of any length), ≥ 1 mm to the avionics module; seated (no overlap with the payload module, whose lowest material over the box is the bulkhead underside = the box top); the bulkhead holes stay open. A separate row from C22, gated on its own params key. N/A without the box |
| C5 (v0.9 direct) | `_c5_direct`, on the solids per fin: (a) servo (case + ears + output boss), horn and coupler ∩ (`propulsion_module` ∪ cover) < 1 mm³ and the servo keeps ≥ `servo_float` − 0.01 from the module (it floats); (b) the lower ear tab captured in its floor slot (material both radial sides and under it ≥ 95 %); (c) slide-out: servo + horn + coupler + connector half swept along −v past the fairing face meet no module material; (d) cover seated on the service face (overlap < tol, BRep gap < 0.01) and covering the opening; (e) each cover screw: clearance hole and tap hole clear, tap material ≥ 95 %, a Ø6 × 60 screwdriver along +v clear of every other component; (f) the connector bay holds the connector clear of module and servo |
| C20 (v0.9 open) | `_c20_open`: the three leads side by side (grown by `motor_leads.wire_clearance`) ∩ (`propulsion_module` ∪ `intake_module` ∪ `edf_unit` ∪ `wire_cover`) = 0 along the whole path; the groove is OPEN with the cover off, its side walls and roof wedge solid, the cover seated (no overlap, gap < 0.01) with clear tap / cover holes and tap material; leg E keeps a BRep gap > 0 to the windowed band; no other opening in the intake's outer skin |
| C23 (v0.9 direct) | as v0.6 from the column up; near the servo the legs are K (connector bay) and P (the tunnel inside the lead pipe) instead of W / O / X, checked against the module and the servo / horn / coupler; the pipe wall around the middle half of each tunnel is solid |
| C28 | (v0.9 direct) coupler alignment and torque: (a) the horn disc's COM on the hinge axis (offset < 0.01 mm); (b) `servo_float` ≥ `print_tolerance_ASSUMED`; (c) coupler and horn rotated with the vane at 0 / ±`max_deflection_mech_deg` clear the module and the servo case by ≥ `servo_float` − 0.01 (BRep) and the coupler clears the vane shaft at every deflection; (d) shaft engagement ≥ half the hub, shaft end short of the flange; (e) servo torque ≥ 2 × the worst hinge moment (`vane_hinge_moment_kgcm`, ASSUMED CP offset / friction). N/A with the pushrod linkage |
| C29 | (v0.9 `motor_leads`) 5.5 mm bullets single file: a bullet proxy Ø(`bullet.d_ASSUMED` + 2 × `bullet_clearance`) × `len` with the other two leads beside it passes (i) straight up through the intake module alone (skirt bore, lip pocket, gate, plenum beside the 0° strut, bulkhead hole) and (ii) up the groove with the cover on, beside the EDF and the spigot - every proxy ∩ material < 0.01 mm³; (iii) the joint zone `ml_joint_box` in the avionics bay clears the avionics module, the intake and every bay component by ≥ `com_mount.min_clear_to_tray` (BRep). N/A without `motor_leads` . wire09c clock mount: (vi) every `generosity_box` corner x e in {-E, 0, +E}: grown envelope x module / intake < 0.01 mm^3, drop-in sweep x module < 0.01, raw envelope x EDF (TE inset 0 / 3, protrusion 30 / 36, nominal + hover seats) < 0.01, hovering tab -> leads >= 0.5 mm, negative controls (e + 6 deg, wall + 3 mm) must collide; WARN when TE inset + rim gap > `bond_gap_max_ASSUMED` (RTV bond optional) |
| C34 | (clock mount) coverage: from the keyways and hover seats every exit azimuth 0-360 deg (0.01 deg) is within `E_cov_deg` of theta_P (worst residual <= E_cov, no gaps); the EDF at every seat x module < 1 mm^3; min sleeve tooth >= `sleeve_min`. N/A without the clock mount |
| C35 | (clock mount) axial retention (A / E): foam compression in [min, max] for every stack case (+-`stack_tol_ASSUMED`), keyway engagement after the largest lift >= 30 mm, the shelf over the lip on the intake solid (notches excluded) and the lip gap clear, shelf bending (peak thrust) and keyway-tooth bearing (full-power torque on one tab) factors >= 5. N/A without the clock mount |
| C36 | (livery, user 2026-11-04, `check_livery.py`) the YUCHAN inlay: (a) the logo footprint grown by `livery.check.clearance_min_mm` (3 mm), built as a thin shell under the outer skin, lies in the PLAIN module's wall on every copy (< 0.05 mm³ outside material: no window, vent slot, hole, joint-screw hole or part end within 3 mm), and (review F6, 2026-11-04) per copy the BRep distance from the inlay to every void of the skin band (the copy's sector of r [r_in, body_r] minus the plain module, past both ends) >= 3 mm with NO volume tolerance (the containment tolerance let ~0.15 mm of encroachment pass), plus the BRep distance to each known feature; (b) on the placed polygons min orange stroke >= `min_stroke_mm` (1.6), min white gap >= `min_white_gap_mm` (1.2) (largest disc whose opening / closing removes only corner slivers, k = 0.25), smallest island >= `min_island_mm2` (3); (c) depth <= `printer.line_w` (one outer line) and wall - depth >= `livery.white_behind_min_mm` (0.78, the two white lines), measured depth = inlay volume / outer area; (d) V(white body) + V(inlay) = V(plain module), white x inlay = 0, inlay inside the plain module, inlay outer area = polygon area (0.5 %); (e) information (F7): orange chords / white gaps below one line width at the layer mid-planes. The 2-colour print file is checked on its G-code (`make_print_files.inlay_print_check`, review F1): orange path <= 1.15 x the single pass (inlay outer area / layer height) and orange grams within 10 % of the solid. N/A without a livery (KAIST) |
| C37 | (2026-11-05 logic power, user decisions 1-4, `configs.vtol.logic_power`; VTOL battery build only, N/A elsewhere) the carrier PCB rev 2c's 1S logic pack at spot A (on the 6S pack's -x face) on the solids: no overlap with the 6S pack, BRep >= 1 mm to the tube wall and >= `com_mount.min_clear_to_tray` to every other flight solid, >= `logic_power.clearance.min_to_leads_plugs` (PCB review MF2-10) to the servo-lead bundle, the GPS lead, the carrier plug boxes and the switch lead, the lengthened lower 6S strap wraps it; reported (E): the strap / tie loops against the leads |
| C38 | (2026-11-05 logic power) the MTS-102 switch through the top cap: hole open, no overlap with the cap, body / lugs >= `printer.min_feature` from the cap and >= `min_clear_to_tray` from everything below the plate and from the cap joint screws at their limit `logic_power.switch.cap_joint_screw_max_len` (M3 x 6; the any-length proxy is reported), the nut bears on the plate (flats - hole >= 2 x `nut_min_bearing_w_ASSUMED`), the plate strength rule of `sizing/vtol_strength.py` untouched, lead keep-outs to J15 / J14 clear, cap removal slack |
| C39 | (2026-11-05 logic power) the lever is reachable with the vehicle closed (finger circle free, inside the body edge, below the eye bolt when fitted); without a collar WARN while the lever top is the vehicle top (the highest other flight solid of the build, measured on the solids); with the switch collar (user decision 2026-11-05, `logic_power.collar`) the lever's swept envelope (`rocket_geometry.logic_lever_positions`, every sampled throw angle ±`lever_throw_deg_ASSUMED`) must sit under the collar rim by ≥ `rim_margin_min`, inside the convex hull of the rim tops in plan by ≥ `plan_margin_min` (no plane resting on the rim reaches it) and ≥ `lever_clear_min` from the cap - PASS only then, else FAIL; tip-over / catch-line / GPS-sky numbers reported. C30 measures the vehicle top on the solids (the printed cap with its collar) and names the collar and the lever inside it (`logic_switch` by name; every other logic_* solid counts, review fix 2026-11-05) as its only non-tether exceptions |
| C40 | (2026-11-05 logic power) the named mass switch `logic_power.in_flight_mass_model` is honoured by the report (false: no logic part in `parts` / `totals`, the top cap's mass entry is the cap without the switch hole and without the switch collar) and the pending full-mass numbers (mass, CG, lateral offset, C33 trim) are reported |
| C41 | (2026-11-05 switch collar, user decision '상단 캡과 한 몸으로 출력하는 보호 칼라를 달아줘', `configs.vtol.logic_power.collar`; VTOL battery build only, N/A elsewhere) the protective collar printed with the top cap: (a) FDM wall - a whole number of `printer.nozzle_d` lines, ≥ `min_feature`, all perimeters with the cap's class (2 × wall loops × nozzle ≥ wall), the wall measured by radial strip probes on both arcs and the base ring = `wall_t`, bore and both finger notches (U slots: width `notch_w`, round bottom of radius `notch_w`/2, lowest point `notch_bottom_h`; review fix 2026-11-05) open; (b) at each end position a fingertip probe (sphere Ø`finger_tip_d_ASSUMED` level with the outer corner of the lever's tip end + the finger behind it, `rocket_geometry.logic_finger_probes`) touches the lever (≤ 0.05 mm, no overlap) and meets no solid; review fix (2026-11-05): along the PUSH (the probe kept on the lever's outboard tip corner from the end position to `finger_path_past_centre_deg` past centre, every `finger_path_step_deg`, both sides) the probe stays ≥ `finger_path_margin_min` from the cap with its collar (BRep; Layout refuses a collar short of it and a bore inside the Ø`finger_clear_d` circle); (c) GPS sky (E): the share of the sky above the receiver's elevation mask (`gps_sky.receiver_elev_mask_deg_ASSUMED`) the collar shades seen from the patch centre ≤ `gps_sky.max_blocked_sky_above_mask_pct`, its top elevation reported; (d) rim side load (E): the vehicle with the logic power × `side_load.dyn_ASSUMED` × `mu_ASSUMED` on one arc's rim, arc (the governing height along the U notch, × `kt_notch_ASSUMED`) and ring (plate) cantilever, interlayer allowed (`interlayer_fraction_of_tensile_ASSUMED` × `printer.tensile_mpa`) / stress ≥ `factor_min`; plate root (E, review fix 2026-11-05): rigid-boss annular plate clamped at the farthest support, the side-load moment and an inverted set-down (lever-rule share × vehicle × dyn + μ × it on the rim), `printer.bending_mpa` / stress ≥ `side_load.plate_root.factor_min`; (e) BRep ≥ `min_feature` to the GPS solids, ≥ 1 mm to the eye bolt when fitted, every Layout collar clearance held, the collar adds the Layout volume to the cap |
| C25 | (v0.6.3, `electronics.fcu`) FCU board on the intake bulkhead, on the solids: BRep clearances ≥ `com_mount.min_clear_to_tray` to the tray, its placeholders, the rail ribs + tray stops and the lead bundles that rise through the bulkhead wire / servo holes (C20 / C23 leg E continued up to the tray bottom), ≥ 1 mm to the avionics module and the intake spigot and radially to the spigot ID; seated (no overlap with the intake; the highest intake material under the footprint is the bulkhead top, `foam_t` below the board; the bulkhead under the footprint ≥ 50 % solid and equal to `Layout.fcu_seat_fraction`); both lead holes open through the bulkhead and free of the board up to the tray bottom; at least one pair of reachable vent holes whose cable-tie run crosses the board (a Ø1 rod between the hole centres at the board's mid height meets it); the centre cable hole reported. N/A without the FCU |
| X3 | (cross-generator, `tools/compare_stl_slices.py` + report diff) the Fusion and CadQuery exports of every printed part agree within 0.5 % in volume and slice by slice; vehicle mass, CG and inertia agree within 0.1 % |

## 8. Flight control system

*(v0.9: §8 is the rocket / ballistic controller, unchanged - `sim/out/gains.json`. The VTOL configuration flies these laws
plus `control.vtol_overrides`, the vertical / position controllers and the power rules of §13.2.)*

8.1 **Frames**: body frame b: x_b toward vane 1, y_b toward vane 2, z_b up
(thrust). World frame w: z up. Attitude quaternion q (w→b). IMU on the NAV board (v0.6.3; the FC before),
aligned with b (firmware has a mounting-rotation parameter).

8.2 **Actuators**: vane deflections δ₁..δ₄ (rad). **Positive δ_i = side force
on the vehicle along +ê_θ** of that vane, ê_θ(θ) = (−sin θ, cos θ, 0). The
vane turns the jet toward its trailing edge and the vehicle feels the reaction,
so positive δ_i moves the trailing edge toward **−ê_θ**. (v0.1 wrote "trailing
edge toward +ê_θ, i.e. force along +ê_θ", which contradicts itself; the sim and
firmware have always used the force convention, and the servo direction is
calibrated on the bench by this definition — `firmware/README.md`.) Throttle
u ∈ [0,1]; release servo (latched / released).

8.3 **Vane force model**: exhaust velocity
V_e = √(T / (ρ A_e)), A_e = π/4 · (`duct_id`² − (`edf.exit_hub_fraction`·`duct_id`)²)
(mm² → m²), dynamic pressure q_e = ½ ρ V_e². Per-vane side force
F_i = q_e · S_v · C_L(δ_i), S_v = `vane_area_mm2`·1e-6,
C_L(δ) = `vane.cl_alpha_per_rad` · δ for |δ| ≤ stall (`vane.stall_deg`), flat
(C_L(stall)) beyond. Force applied at r_v = `vane.force_centroid_r` (28 mm),
Z = `vane_hinge_z`, direction ê_θ(θ_i). The force also deflects the jet: the
thrust vector tilts by the same lateral force (included in the sim as the vane
force acting on the vehicle; no extra turning-loss term).

8.4 **Moments** (lever arm l_v = z_CG − `vane_hinge_z`, CG from the CAD report):
with δ_x ≡ ½(δ₁ − δ₃), δ_y ≡ ½(δ₂ − δ₄), δ_z ≡ ¼(δ₁+δ₂+δ₃+δ₄):
M_x = 2 l_v q_e S_v C_Lα δ_x, M_y = 2 l_v q_e S_v C_Lα δ_y, M_z = 4 r_v q_e S_v C_Lα δ_z.
(Derivation: vane 1 at +x with force +y gives moment (l_v F, 0, r_v F); vane 3
at −x with force −y gives (l_v F, 0, −r_v F) etc.) Residual fan swirl torque
τ_swirl = `edf.residual_swirl_torque_fraction_ASSUMED` · P/ω (sign per
`edf.rotation`) is a disturbance in the sim and must be trimmed by δ_z.

8.5 **Mixer** (inverse): given desired moments (M_x, M_y, M_z) and a thrust
estimate T̂ = max(T_model(u), `control.thrust_estimate_floor_fraction`·T_max):
δ_x = M_x / (2 l_v q̂_e S_v C_Lα), δ_y = M_y / (2 l_v q̂_e S_v C_Lα),
δ_z = `control.roll_priority_scale` · M_z / (4 r_v q̂_e S_v C_Lα);
δ₁ = δ_x + δ_z, δ₂ = δ_y + δ_z, δ₃ = −δ_x + δ_z, δ₄ = −δ_y + δ_z;
if any |δ_i| > δ_max (`max_deflection_cmd_deg`), first shrink δ_z, then scale
(δ_x, δ_y) uniformly so the largest |δ_i| = δ_max. Servo PWM:
`pwm_i = pwm_center_us + sign_i · δ_i[deg] · pwm_us_per_deg + trim_i`.

8.6 **Attitude controller** (runs at `control.loop_hz`, dt = 1/250 s = 4 ms on
the FCU ATmega128A, on the NAV's estimate of that tick (§10.5); servo and ESC commands are latched by 50 Hz PWM frames):
* Setpoint q_sp: upright (z_b = z_w) with yaw hold at the arming heading
  (`control.yaw_hold`).
* Error: q_e = q⁻¹ ⊗ q_sp (body frame); if q_e.w < 0 negate; rotation-vector
  error e = 2·(q_e.x, q_e.y, q_e.z) (small-angle; exact: 2·atan2(|v|, w)·v/|v|).
* Rate setpoint: ω_sp,xy = clamp(`att_kp_xy` · e_xy, ±`rate_sp_max_xy_rad_s`),
  ω_sp,z = clamp(`att_kp_z` · e_z, ±`rate_sp_max_z_rad_s`).
* Rate PID per axis (gains `rate_kp/ki/kd_*`, integrator clamp `rate_i_max_*`,
  D on measured gyro with a first-order LPF at `d_term_lpf_hz`, gyro LPF
  `gyro_lpf_hz`): M = Kp·(ω_sp − ω) + Ki·∫(ω_sp − ω)dt − Kd·d(ω_f)/dt
  [N·m]. Integrator frozen while any vane is saturated or while T̂ < floor.
* Controller output passes through the mixer (8.5).
* Gains in `params.json` are initial values; the simulation tunes them and
  writes `sim/out/gains.json`; `tools/gen_gains_header.py` turns it into
  `firmware/atmega128_fc/src/gains.h`; the firmware never hard-codes a number
  that lives in `gains.json`.

8.7 **Thrust model** (sim + firmware estimate): T = T_max · u^`edf.throttle_exponent`
with a first-order lag `edf.thrust_time_constant_s`; T_max =
`edf.thrust_nominal_N`; electrical power P = `edf.power_nominal_W`·u³ (for
energy accounting only).

8.8 **Onboard supervisor (v0.4) — the only autonomous logic on the vehicle.**
User requirement (2026-10-13): the vehicle computer does attitude control only; flight
commands come from the ground over LoRa (§10.3). The vehicle therefore runs: the
attitude estimator (§9 `pad_gyro`) and the altitude / v_z filter (for telemetry and the
interlocks only) - on the NAV since v0.6.3 -, the attitude controller and mixer (§8.5–8.6),
a command executor and the safety interlocks below - on the FCU, which makes every flight
decision (§10.1; the COM only carries the commands and the replies). It has **no mission
sequencing**: launch, throttle, motor cut and nose release are ground commands; the ground
station software may automate them (assist mode). The v0.3 onboard mission state machine is
superseded. The supervisor itself (`sim/onboard.py`, `core/onboard.c`) did not change in v0.6:
the chip split only changes where its inputs come from (§10.4, §10.5).

States (`sim/onboard.py` is the reference, the firmware mirrors it):
```
SAFE      motor 1000 us, vanes centred, release latched          (boot state)
ARMED     on the pad, motor off, vanes centred, attitude estimator aligning
POWERED   throttle target > 0, attitude control on, throttle slewed by throttle_ramp_s (0 -> 1)
UNPOWERED throttle 0 after POWERED; attitude control on for coast_control_hold_s, then vanes centred
RELEASED  release servo open; motor off until power cycle; vanes centred
ABORT     motor off now; release after abort_release_delay_s, then RELEASED
```
Commands (valid frame, this `vehicle_id`, SEQ newer than the last executed one):

| command | accepted in | effect |
|---|---|---|
| ARM | SAFE | -> ARMED if arm switch closed, sensors healthy and aligned, tilt < arm_tilt_max_deg, V_cell >= min_cell_v, link alive |
| DISARM | ARMED | -> SAFE (once POWERED use CUT / KILL) |
| LAUNCH(nonce) | ARMED | nonce must equal the nonce in the latest telemetry; throttle target = climb_throttle -> POWERED |
| THROTTLE(u, nonce) | ARMED (nonce required), POWERED, UNPOWERED | throttle target = clamp(u, 0, climb_throttle); u > 0 -> POWERED, u = 0 -> UNPOWERED |
| CUT | POWERED | throttle target 0 -> UNPOWERED |
| RELEASE | ARMED, POWERED, UNPOWERED | throttle 0 now, open the release servo after release_after_cut_s -> RELEASED |
| KILL | any except SAFE | -> ABORT (from ARMED: -> SAFE) |

Commands that do not apply are ignored and reported as rejected in the telemetry.

Safety interlocks — cannot be disabled by any command (they exist because LoRa is slow
and can drop out; without them a lost link leaves a powered vehicle climbing until the
battery is empty):

| # | condition | in | action |
|---|---|---|---|
| I1 | no valid ground frame for link_loss_abort_s (3.0 s) | POWERED, UNPOWERED | ABORT |
| I1p | no valid ground frame for link_loss_disarm_s (2.0 s) | ARMED | SAFE |
| I2 | tilt > tilt_abort_deg (45) | POWERED | ABORT |
| I3 | spin / gyro-saturation supervisor (tilt_rate_abort_dps, spin_rate_abort_dps, rate_abort_persist_s) | POWERED | ABORT |
| I4 | apogee limiter: h + coast_drag_factor v_z^2/(2g) >= apogee_limit_m (150) | POWERED | CUT only (keeps the legal ceiling whatever the link latency) |
| I5 | h > geofence_altitude_m (160) | POWERED, UNPOWERED | ABORT |
| I6 | fall-safe: v_z <= fall_deploy_vz_mps (-8) for fall_deploy_confirm_s (0.3 s), not released | POWERED, UNPOWERED | RELEASE |
| I7 | POWERED longer than max_motor_time_s (25) | POWERED | CUT |
| I8 | V_cell < min_cell_v | POWERED | ABORT |
| I9 | IMU failure > 0.1 s or baro failure > 0.5 s (v0.6.3: no valid NAV frame counts as both; a NAV that rebooted in flight has no valid attitude, so its frames count as an IMU failure - §10.5) | ARMED / flight | SAFE / ABORT |
| I10 | watchdog or brown-out reset in flight | boot | SAFE with the release opened (existing) |

The arm switch opening while armed acts as KILL. Attitude control is active in POWERED
and for coast_control_hold_s of UNPOWERED (fan swirl decay, v0.2 finding). The link interlocks I1p / I1 also
cover a dead COM or a cut ilink wire (no IL_CMD reaches the supervisor: the same as a lost radio link, §10.4).

**Ground station** (`tools/ground_station.py`, reference logic `sim/ground_station_logic.py`):
it is the link master and polls at `link.poll_hz`; every poll carries the operator's
current command, repeated until the telemetry's ack_seq confirms it. It shows and logs the
telemetry. **Assist mode** (default on, the operator can override) holds the former onboard
mission rules: auto-CUT when P + max(dP/dt, 0)·T_tel/2 >= target_altitude_m, where
P = h + v_z·t_link + coast_drag_factor·v_z²/(2g) is the predicted apogee using the measured link
delay t_link, dP/dt is measured between consecutive POWERED telemetry and T_tel = max(1/poll_hz, t_link)
is the telemetry interval. At 2 Hz P grows by about 11 m per telemetry on the v0.6.3 vehicle (`sim_report.md`: S2
dP/dt 21.6 m/s × T_tel 0.5 s; the v0.4 text and the module docstring give ~14–15 m), so deciding on P alone
overshoots the target by up to a whole step and let the onboard limiter I4 cut first on the fast
SX127x link; the half-interval lead centres the cut on the target. Auto-RELEASE when the downlinked v_z, predicted forward to
the moment the servo will open, reaches apogee_detect_vz:
v_z − g·(t_link + release_after_cut_s + T_tel/2) <= apogee_detect_vz, T_tel = max(1/poll_hz, t_link)
(or coast_timeout_s after the first UNPOWERED telemetry). Both rules are evaluated on each new telemetry.

**v0.6.3: a reply timeout is a decision point too.** When a request goes unanswered, the CUT rule and the
predictive RELEASE rule are evaluated at the timeout on the latest telemetry extrapolated by its age
a = now − its arrival time: CUT on P + dP/dt·a (+ the same half-interval lead), RELEASE on v_z − g·(a + t_link +
release_after_cut_s + T_tel/2), the coast timeout on coast time + a. Nothing is extrapolated before two POWERED
telemetry frames gave a dP/dt, nor for the literal release rule. Why (module docstring of
`sim/ground_station_logic.py`): without it one lost frame near the cut moves the decision ~dP/dt·T_tel ≈ 11 m past
the centred point - more than the 10 m between target_altitude_m and the I4 limit; in practice mode, 100 seeds per
radio at 5 % loss, I4 cut first in 15 (lora_uart) / 19 (sx127x) flights without the rule and 10 / 6 with it, nearly
all of the rest having lost the CUT exchange itself (apogee 147.6–149.5 m in every I4 flight). Those practice runs
are not saved in a report on disk.

Results (v0.6.3 vehicle 2586 g, `sim/out/sim_report.md` 2026-10-15): base S2 cuts by CUT at 122.3 m, apogee
144.8 m, opens the nose 0.12 s after apogee at −1.2 m/s (sx127x, S2s: 0.00 s, +1.1 m/s); 10 more S2 seeds
(301–310) PASS 10/10, release − apogee −0.14…+0.16 s, |v_z| ≤ 1.6 m/s, one of them cut by I4; the 20-seed estimator
and link Monte Carlo passes 19/20 without the touchdown term (seed 103 opens 0.51 s after apogee at −5.01 m/s, just
past |v_z| < 5; CUT 19 ×, I4 1 ×, apogee ≤ 148.3 m); S8 (30 % loss) PASS in 10/10 link seeds, 5 of them cut by
I4. History: v0.4 found that the literal "v_z <= 0 for apogee_confirm_s" rule opened the nose 1.2 s after apogee at
−11 m/s on the 2.4 kbps UART link; on the v0.5 vehicle the base S2 seed lost the CUT and the first RELEASE frames
and opened 1.24 s after apogee at −12.0 m/s (one lost frame costs ~0.5 s on that link). coast_drag_factor = 1.20
(v0.4): after the cut the fan still pushes for ~τ = 0.15 s and drag costs less than 15 %, so 0.85 under-predicted
the coast by ~7 m and let the apogee reach 157 m; re-measured on the v0.6.3 S7 (I4 is the only cut) the factor that
predicts the apogee is 1.142 (estimate) / 1.147 (truth), so 1.20 stays.

**Powered-flight supervisor** (spin / gyro saturation) of v0.2 is interlock I3.

## 9. Simulation (`sim/`)

*(v0.9: the VTOL plant terms - VTOL vehicle, vane play, rotor gyro / spool reaction, power and tether models, ground
effect, the evaluation harness and its seed sets - are in §13.2. `run_scenarios.py --plant v09|v081` flies the ballistic
S1-S8 on either plant.)*

6-DOF rigid body, quaternion attitude, RK4 physics at 1 kHz, controller at
`control.loop_hz` (250 Hz) with the measured computation delay (`sim/fc_timing.py`): v0.6.3 the sample of tick k
reaches the output registers `FC_COMPUTE_DELAY_S_MEASURED` = 4.71 ms later (sub-tick resolution) - the three-chip
co-simulation's maximum from the NAV's MPU-6050 burst start to the FCU's last vane OCR write, 4.709 ms when the sim
was run, rounded up to 10 µs; the current `cosim_report.json` measures max 4.658 ms (lora_uart) / 4.655 ms
(sx127x), p50 4.24 ms in flight, so the sim stays on the conservative side. The co-simulation judges this path against
these 4.71 ms (the delay the sim, the gain tuning and the Monte Carlo fly with), not against one 4 ms tick
(`run_cosim.py`; the count of ticks over 4 ms is reported for information). v0.6.2 and earlier: exactly one control
tick, 4.0 ms, by construction. The NAV's pad / flight switch (estimator alignment, altitude held at 0) follows the
FCU state of the last nlink exchange, 2 control ticks behind the supervisor (`SensorConfig.nav_state_lag_ticks` = 2;
v0.5: 1; §10.5). Equivalent actuator delay at the baseline: compute 4.7 + wait to the frame edge 1.3 + pulse 1.5 +
half-frame hold 10.0 = 17.5 ms, servo lag τ 70 ms on top.
Servo and ESC commands are latched only at the 50 Hz PWM frame edges
(zero-order hold, per-timer phase), then first-order `servo.time_constant_s` +
rate limit `servo.rate_limit_deg_s` + saturation. Sensors and estimator
(`sim/estimator.py`, default "pad_gyro" = what the firmware flies): MPU-6050
gyro (0.005 °/s/√Hz, ±2000 °/s, 16.4 LSB per °/s, slowly drifting bias up to
0.5 °/s, DLPF 2.8 ms); gyro bias and accelerometer levelling only on the pad,
gyro-only quaternion integration after launch (an accelerometer under thrust
reads the thrust axis, not gravity); BMP280 baro 0.44 m RMS with 0.055 m steps at 50 Hz and
0.1 s latency (2026-11-10 control round, `sim/sim6dof.py` `BARO_MODELS`: the noise is 2.5 x the datasheet figure - E, the
datasheet value itself A, not measured on the part; the altitude-filter crossover went 0.5 -> 0.4 Hz with it; the MS5611
model, 0.25 m RMS, stays selectable). Truth + noise and a Mahony filter remain as options. Linear loop
margins with the frame hold, delay and servo lag are computed by
`sim/loop_margins.py`; the tuned gains must keep phase margin ≥ 30° and gain
margin ≥ 2 on both axes. Thrust lag 8.7. The powered-flight spin/saturation
supervisor of §8.8 is on in every scenario.
Aero: drag ½ρV²·`body_cd`·A_ref (A_ref = π `body_r`²) along −V; normal force
from body (`body_cn_alpha_per_rad`) + fins (Barrowman C_Nα) acting at the
Barrowman CP; gravity; wind (steady `wind_mps_default` + gust
`gust_mps_default` for `gust_duration_s`). Mass properties from
`cad/exports/cadquery_report.json` (fallback: `fusion360_report.json`; if
neither exists, the estimate in `sizing/sizing_report.json`). Scenarios:
S1 hover-stabilise from 10° initial tilt (fixed thrust = weight), S2 nominal
flight to `target_altitude_m` with wind + gust, S3 flight to 250 m
(competition), S4 servo failure of one vane at t = 2 s (expect ABORT), S5
gain sensitivity ±50 %; since v0.4 also S6 total link loss 4 s after launch (expect I1 ABORT), S7 assist off
(expect the I4 cut and the I6 release), S8 30 % packet loss (no unsafe outcome) with a 10-seed link Monte Carlo,
and the informational S2s (sx127x), S2t (truth + 0.5° estimator), S2b (worst gyro-bias drift), S4x (S4 with I3
off). Outputs: `sim/out/*.png` plots, `sim/out/sim_report.md`
with metrics (max tilt, settling time, apogee, lateral drift, max vane
deflection, energy used, time in saturation), `sim/out/gains.json`.
v0.6.3 run (2026-10-15, params sha256 3f31c0a1…; re-run 2026-10-16 on params sha256 0a5c261b… with the same gains
and the same metrics, CadQuery mass properties): gains retuned on S1 for the 4.71 ms delay and the 2-tick NAV lag
(137 evaluations; `gen_gains_header.py` wrote `GAINS_PARAMS_STALE` 0 against the current params.json); linear margins pitch/yaw PM 40.6° (GM × 6.20), roll PM 30.1° (GM × 3.43), 6-DOF gain margins 4.97 /
3.08; S1, S2, S4, S5, S6, S7, S8 PASS, S3 INFO (would pass the S2 criteria at its own limits).

## 10. Firmware (`firmware/`)

v0.2: the flight computer is an **ATmega128A** (16 MHz, 5 V, 4 KB SRAM, no
FPU; `flight_computer.*` in params.json holds the pin maps, timers and baud
rates). ArduPilot does not run on it, and the Teensy design of v0.1 is dropped.
v0.6: it is split over several ATmega128A (§10.1).

### 10.1 Architecture (v0.6.3: three ATmega128A)

User rule (2026-10-15, `flight_computer.architecture`): every flight computation runs on an ATmega128A; any number
of chips may be used. v0.6.0 moved the LoRa radio to a second chip (COM); v0.6.3 moved the sensors, the estimator
and the altitude filter to a third (NAV). All three are ATmega128A minimum-system boards (16 MHz crystal, 5 V, the
same fuses), built from one source tree (`platformio.ini`: `-D FC_ROLE_NAV / FC_ROLE_FCU / FC_ROLE_COM`), powered
from the 5 V logic UBEC with a common ground (no level shifter on the inter-chip links).

| chip | board, place | runs | links |
|---|---|---|---|
| NAV | `electronics.nav`, tray +x face (the v0.5 FC spot, §5.2) | MPU-6050 + barometer on I2C 400 kHz (level shifter; part and default driver: BMP280 since 2026-11-10, MS5611 driver = build option `nav_ms5611`), pad alignment / gyro-only attitude estimator, tilt, altitude filter, sensor health. No flight decisions | nlink SPI master (§10.5) |
| FCU | `electronics.fcu`, flat on the intake bulkhead (§5.2) | onboard supervisor (commands + I1–I10, §8.8), attitude controller + mixer, the 6 PWM outputs, battery ADC, arm switch, LED / buzzer, OpenLog log. Every flight decision | nlink SPI slave; ilink USART1 (§10.4); log USART0 TX |
| COM | `electronics.com`, under the payload bulkhead (§5.3) | LoRa radio driver (§10.3), uplink decoding → IL_CMD, TELEM built from the FCU's IL_SNAP, link status. No flight decisions | ilink USART0; radio USART1 (lora_uart) or SPI (sx127x) |

**Loop timing.** The NAV's Timer2 paces the vehicle: at the end of each NAV tick the 72-byte nlink exchange runs,
and the FCU's tick starts when its last byte has arrived (frame-triggered); without a frame the FCU falls back to its
own Timer2 (§10.5). The COM runs its own 250 Hz tick and has no hard deadline (bounded main-loop passes). Why the
NAV: the v0.6 FCU (sensors + estimator + supervisor + controller + outputs in one 4 ms chain) measured 2971 µs
time-critical and 3308.5 µs = 82.7 % of the tick with every interrupt (`simbench_report.json` `fcu_simbench.v06_reference`
`crit_plus_all_isr_us`; limit 80 %). `core/nlink_proto.h` quotes 78.8 % "with its interrupts" and ~83–86 % in the
worst composition; no report on disk holds the 78.8 % figure, so what it covers is NOT VERIFIED. v0.6.3 (limit 80 % of the tick): FCU
1944 µs (48.6 %) in the single-chip simbench and at most 2578 µs (64.4 %) in the three-chip co-simulation with every
interrupt and a busy log port (worst: the NAV-reset fault run); NAV ≤ 2435 µs (60.9 %) as an upper bound with the
whole MPU-6050 burst.

**Failure semantics** (the supervisor is unchanged; each fault lands on an interlock it already had):
* dead or resetting COM, or a cut ilink wire: no IL_CMD reaches the supervisor - a lost radio link, I1p on the pad
  (2 s → SAFE), I1 in flight (3 s → ABORT, release 0.5 s later; co-simulation `fault_cut`: ABORT 750 ticks after
  the last IL_CMD, RELEASED 125 ticks later). A COM reset whose link gap stays below I1 changes no state and the
  link resumes (`fault_reset`).
* dead FCU: no IL_SNAP, so the COM transmits nothing and the ground sees reply timeouts; a hung FCU loop is reset by
  its watchdog into a SAFE boot that opens the release if it happened in flight (I10).
* dead or resetting NAV, or a cut nlink wire: no valid navigation frame, the FCU runs on its Timer2 with imu_ok =
  baro_ok = false → I9 after 25 ticks (0.1 s): ABORT in flight, SAFE on the pad. A NAV that rebooted in flight
  delivers frames again but without EST_VALID (that needs a completed pad alignment), so imu_ok stays false → I9
  ABORT (`fault_nav_hold` / `fault_nav_pulse`: ABORT 25 ticks after the last valid frame, RELEASED 125 ticks later).
* corrupted bytes: refused by the CRC on every link and counted; a babbling COM is bounded (it can flood the FCU's
  RX ring but not stretch its time-critical part past the limit; without an IL_CMD the supervisor sees a lost link).
* chips built from different `params_gen.h`: `FC_PARAMS_TAG` travels in the COM's IL_STAT and in the FCU frame of
  the nlink; a mismatch is logged and shown on the NAV LED, no flight decision reads it. (v0.9 VTOL builds: a NAV or
  COM mismatch also holds the supervisor's arm-switch input open on the ground, so ON is refused; a GNS of another
  configuration is not detected - §10.6.)

**Programming** (ISP pins shared with the links): FCU - unplug the OpenLog (PE0/PE1) and hold the NAV in reset or
unplug the nlink connector (FCU PB1 = ISP SCK); COM - unplug the ilink connector (PE0/PE1).

### 10.2 Firmware build, hardware, verification

* `firmware/atmega128_fc/` — bare-metal C99 with avr-libc, built by avr-gcc
  through PlatformIO (`platform = atmelavr`; envs `nav`, `fcu` (+ bench builds), `com`, `com_lora_uart`,
  `com_sx127x`, the simbench and co-simulation builds). `src/core/` is portable C99 with
  no AVR headers (quaternion, PID, mixer, attitude controller, attitude estimator, altitude filter,
  thrust model, the v0.4 onboard supervisor, the radio link protocol, the ilink and nlink protocols, the log
  records) mirroring `sim/controller.py`, `sim/estimator.py`, `sim/onboard.py`, `sim/link_proto.py`,
  `sim/ilink_proto.py` and `sim/nlink_proto.py` in 32-bit float; `src/hal/` holds the AVR drivers including the
  two radio backends (§10.3), `ilink_uart` and `nlink_spi`; `src/nav/` is the NAV's 250 Hz loop, `src/main.c` the
  FCU's frame-triggered loop with the link work in bounded background steps, `src/com/` the COM.
* Hardware: NAV - MPU-6050 (GY-521) + barometer on I2C 400 kHz through a BSS138 level shifter (the
  ATmega128's TWI V_IH at 5 V is 3.5 V, above a 3.3 V sensor's output). Barometer part: BMP280 module (TYE-BMP29,
  Devicemart no=15442588, 3.3 V, I2C 0x77 with CSB and SDO tied to 3.3 V; carrier PCB rev 2c J2; user decision
  2026-11-04, instead of the GY-63 MS5611). Since the 2026-11-10 control round the NAV builds the BMP280 driver by default
  (`hal/bmp280.*`, `core/bmp280_math.*`, selected in `hal/baro.h` from `flight_computer.nav.baro_sensor`: forced mode,
  pressure x4 / temperature x1 oversampling, IIR off - datasheet values, A, not measured on the part); the MS5611 driver
  (`hal/ms5611.*`, `core/ms5611_math.*`) is a build option (env `nav_ms5611`). COM - since 2026-11-06 the default radio is the ESP-12E bridge of §10.7 on USART1 (vehicle_radio = esp_bridge); the fallback is the LoRa module of §10.3 on USART1
  (lora_uart) or SPI (sx127x) through a second BSS138 module, with its own 3.3 V regulator (both in the radio/log
  box, §5.3). FCU - OpenLog serial SD logger on USART0 TX (57 600 baud, shared with the ISP pins); 6 hardware PWM
  outputs on Timer1 (OC1A/B/C = vanes 1–3) and Timer3 (OC3A = vane 4, OC3B = ESC, OC3C = release servo), Fast PWM
  mode 14, TOP 39 999, prescaler 8 → 50 Hz frames with 0.5 µs resolution; battery on ADC0. Fuses (every chip):
  M103C and JTAGEN unprogrammed, BOD 4.0 V. Pins: `flight_computer.pins` (FCU), `.nav.pins`, `.com.pins`;
  `src/board.h`.
* Attitude estimate (NAV): gyro bias and accelerometer levelling on the pad only,
  gyro-only quaternion integration after launch (an accelerometer under
  thrust reads the thrust axis, not gravity).
* Parameters: one `src/params_gen.h` for all three chips from `params.json` + Layout + the CAD report
  (`tools/gen_firmware_params.py`, which checks each chip's pin table on its own), gains `src/gains.h` from
  `sim/out/gains.json` (`tools/gen_gains_header.py`); never typed by hand.
* Verification: `host_test/test_core.c` compiled for the host with
  `python -m ziglang cc -std=c99` against the byte vectors of the three link references;
  `tools/crosscheck_controller.py` feeds identical vectors to the Python references (float64) and the C core
  (float32) and bounds the difference - controller, supervisor (506 232 ticks, 0 mismatches), radio protocol,
  ilink, nlink, estimator, altitude filter, log round trip: PASS (`crosscheck_report*.json`); simavr benches of
  each chip (`run_simbench.py` FCU, `run_nav_simbench.py`, `run_com_simbench.py`: all checks PASS) and the
  three-chip co-simulation (`tools/run_cosim.py`, `tools/cosim/README.md`: NAV, FCU and COM in lockstep, wired as
  on the board, both radio backends, normal flights and faults: 43 / 43 criteria PASS in `cosim_report.json`).
  Flight builds: FCU 27 956 B flash / 2417 B static RAM, NAV 17 332 / 771, COM 10 280 / 928 (lora_uart), 10 756 /
  929 (sx127x); every chip keeps ≥ 1024 B of stack free.

### 10.3 LoRa ground link (v0.4)

The command link is the user's LoRa module; the SBUS receiver is dropped. The ground
station is the master: the vehicle transmits only as a reply to a valid uplink frame, so
uplink and downlink never collide on the half-duplex radio. Since v0.6 the radio is on the COM chip: it decodes the
uplink, forwards every accepted CMD frame to the FCU as an IL_CMD and transmits only a TELEM built from the FCU's
IL_SNAP that answers an IL_CMD it forwarded less than 80 ms earlier (reply gate, §10.4); the frames below are
unchanged.

Frame (both directions, little-endian): `0xA5 0x5A | LEN | VEH | TYPE | SEQ | PAYLOAD | CRC16`.
LEN counts VEH..PAYLOAD; CRC16-CCITT (poly 0x1021, init 0xFFFF) over LEN..PAYLOAD;
VEH = `flight_computer.link.vehicle_id`. Frames with a wrong CRC, length or vehicle id are
dropped silently.

* Uplink `TYPE 0x10` CMD, payload 5 bytes: `cmd u8` (0 NONE/heartbeat, 1 ARM, 2 DISARM,
  3 LAUNCH, 4 THROTTLE, 5 CUT, 6 RELEASE, 7 KILL), `arg u16` (throttle x1000), `nonce u16`.
* Downlink `TYPE 0x20` TELEM: state u8, flags u8 (arm switch, sensors ok, aligned, link ok,
  released, control active, rejected, interlock), reason u8 (the latest interlock code once any
  interlock has fired - latched with the interlock flag until power-up, a later rejection only sets
  the rejected flag; before that the last reject code),
  ack_seq u8, nonce u16 (new random value after every executed LAUNCH/THROTTLE-from-ARMED),
  t_ms u32, h i16 (0.1 m), v_z i16 (cm/s), roll / pitch / yaw i16 (0.01 deg), tilt u16
  (0.01 deg), throttle u16 (x1000), V_cell u16 (mV), loop_max u16 (us).

Radio backends (compile-time `flight_computer.link.radio`, env `com`; `com_lora_uart` and `com_sx127x` always build
both). **2026-11-06: superseded for the backend choice by §10.7** - the COM backend is `flight_computer.link.vehicle_radio`
(default `esp_bridge`, the ESP8266 Wi-Fi bridge); `link.radio` stays the link model (`lora_uart`) and the two LoRa
backends below stay buildable as the fallback:
* `lora_uart` (default): Ebyte E22 / E220 / E32-class transparent UART module, M0 = M1 = 0
  (normal mode), COM USART1 at `link.uart_baud` 8N1. Channel, air rate and address are set
  beforehand with the vendor's configuration tool; the MCU only moves bytes.
* `sx127x`: an SX1276-based 900 MHz module on SPI, e.g. HopeRF RFM95W or Ai-Thinker Ra-01H.
  Ra-01 / Ra-02 (SX1278) are 433 MHz parts and do not fit the 920 MHz plan; the generator refuses
  a frequency outside the band in `link.frequency_note`. LoRa mode, `link.sx127x` frequency, SF,
  bandwidth, coding rate, sync word, explicit header, payload CRC on. SPI runs at F_CPU/32 =
  500 kHz because MOSI / SCK / NSS pass through BSS138 channels (keep it at or below 1 MHz);
  the module needs its own 3.3 V regulator rated >= 150 mA for transmit.

Level shifting: every MCU -> module line goes through a BSS138 channel to 3.3 V;
module -> MCU lines go direct (3.3 V >= the AVR's 3.0 V input-high threshold at 5 V).
Frequency: in Korea use a KC-certified module in the 920 MHz band; whether a 433 MHz
module may be used was not verified — check 전파누리 (rra.go.kr) first.
Latency budget (`sim/link_model.py`, reported by the generator and in `sim/out/sim_report.md`): uplink + on-vehicle
reply delay + downlink at `link.poll_hz`; the sim's link model uses the same numbers plus a packet-loss rate. On the
vehicle (v0.6.3, measured in the three-chip co-simulation, normal flight):
* `COM_FORWARD_S` = 0.6 ms: uplink frame complete at the COM's radio driver → IL_CMD queued at the FCU (max 0.30 ms
  lora_uart / 0.52 ms sx127x, rounded up); the supervisor takes it at the FCU's first tick after that (0 ticks late
  in every normal run).
* `FC_REPLY_DELAY_S` = 14 ms: uplink at the COM → TELEM handed to the vehicle radio (current `cosim_report.json`:
  max 12.5 / 13.2 ms, p50 10.7 / 9.4 ms; the spread is the wait for the FCU tick). One constant, the worst normal
  case rounded up, so the model's mean and worst round trip are equal. It replaces v0.5's `FC_REPLY_DELAY_TICKS` =
  16 control ticks (64 ms + up to one tick). Not modelled: a burst of uplinks queued at the COM (up to 38.7 ms in the
  current report); the timeout's 68 ms allowance below covers it.
* `FC_REPLY_ALLOWANCE_S` = 68 ms (= v0.5's 17 ticks): the latest reply the ground waits for. The ground's reply
  timeout is built on it, not on the 14 ms: timeout = (uplink + 68 ms + downlink) × `REPLY_TIMEOUT_FACTOR_ASSUMED`
  (1.0) + `REPLY_TIMEOUT_MARGIN_S_ASSUMED` (40 ms) = 531.0 ms (lora_uart) / 292.4 ms (sx127x), unchanged from
  v0.6.2. The COM's 80 ms reply window was derived from it (§10.4); shrink the timeout only together with that window,
  or a legitimate late reply can collide with the ground's next request (since v0.6.3 that can be an assist CUT /
  RELEASE sent at the timeout itself, §8.8).

Round trip 437.0 ms (lora_uart, 2.4 kbps, assumed Ebyte overheads) / 198.4 ms (sx127x SF7 / 125 kHz) → maximum
request rate 2.29 / 5.04 Hz, so `link.poll_hz` = 2 Hz is achievable on both. Consecutive lost uplink frames tolerated
before I1p (2.0 s) / I1 (3.0 s): 2 / 4 (sim report). (v0.5 single chip: 491 ms worst round trip, maximum 2.04 Hz.)
Measure the real modules before flight - every Ebyte / SX127x overhead in the model is `_ASSUMED`.

### 10.4 ilink: FCU ↔ COM (v0.6)

Normative references: `sim/ilink_proto.py` (byte-exact, the decoder rules), `src/core/ilink_proto.h`,
`src/com/com_link.h` (pacing, reply gate), `src/hal/ilink_uart.h`. Summary:

* Wire: FCU USART1 (PD2 RXD1 / PD3 TXD1) ↔ COM USART0 (PE0 RXD0 / PE1 TXD0), `flight_computer.ilink.baud` =
  250 000 8N1, U2X 0 (UBRR 3, 0 % error at 16 MHz) - one byte per 40 µs. The FCU's TXD1 reaches the COM's PE0 (its
  ISP PDI) through `series_resistor_ohm` (1 kΩ). RX pull-ups on, so an absent peer leaves the line idle-high. Byte
  rings behind hand-written ISRs (RX 51 cycles, TX 47 cycles a byte: they also run during the nlink exchange).
* Frame: `0xC5 0x5C | LEN | TYPE | PAYLOAD[LEN] | CRC16 lo | hi`, CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, the
  radio link's CRC) over LEN, TYPE, payload; little-endian, float = IEEE-754 binary32. Each TYPE has one fixed LEN:
  - `IL_CMD` 0x01, COM → FCU, 8 B (14 B frame): seq, cmd, arg, nonce, rssi_dbm, snr_q4 - one uplink CMD frame the
    COM's decoder accepted (vehicle id, length, CRC checked), in arrival order; the FCU hands it to the supervisor as
    the v0.5 single chip handled a frame it decoded itself. The COM does not interpret the command.
  - `IL_SNAP` 0x02, FCU → COM, 49 B (55 B frame): the supervisor's reply snapshot of the tick that processed the
    uplink (reply_seq, state, flags, reason, ack_seq, nonce, throttle, tick, h, v_z, tilt, V_cell, loop_max, q[4]).
    The COM computes roll / pitch / yaw from q, quantises and sends TELEM with SEQ = reply_seq; it never transmits
    anything not built from a SNAP.
  - `IL_STAT` 0x03, COM → FCU, 36 B (42 B frame): the COM's counters, reset cause, radio status, params tag. The FCU
    logs it ('C' record); nothing in the supervisor depends on it.
* Decoder (both chips, v0.6 phase 3): one byte at a time with bounded work and no replay - hunting for 0xC5, then
  sync, LEN, TYPE checked in the header, payload taken with an incremental CRC, CRC low byte checked on arrival; a
  rejected candidate's bytes are discarded and only the rejecting byte is looked at once more, so a frame hidden
  inside a rejected candidate is lost (the ground's retry covers a lost IL_CMD). At most one frame per input byte.
  Cost on the FCU ≤ 91 cycles for any byte (+ ≤ 4 for the direction filter) - the replaying decoder of phase 2 cost
  up to 6316 cycles for one byte. v0.6.3 direction filter: the FCU's decoder accepts CMD and STAT, the COM's only
  SNAP; a LEN or TYPE that cannot travel in that direction is rejected in the header, so a SNAP-like header in
  garbage cannot swallow the bytes behind it.
* FCU timing: in its time-critical part the FCU decodes only the ilink bytes in which an IL_CMD can still complete
  (`il_decoder_cmd_need`), the rest in the background and while it waits for the next nlink frame. Co-simulation:
  every IL_CMD reached the supervisor of the first FCU tick after its last byte (0 ticks late, 1240 / 1200 IL_CMD).
* STAT pacing (COM, `com_link.h` D4): an IL_STAT is queued right behind an IL_CMD the COM has just queued, at most
  once per 1 / `stat_hz` (1 s); on its own divider only after 1.5 / `stat_hz` without an IL_CMD; never in front of an
  IL_CMD (an IL_CMD behind a 42-byte STAT would reach the supervisor one tick late). Gap ≥ 1 s, ≤ ~2.5 s when the
  uplink stops. Boot STAT (`IL_STAT_BOOT`, R3): not before `COM_STAT_BOOT_MS` = 100 ms after the COM's reset, so an
  FCU powered up with it is listening (the v0.6.2 COM sent it at 1.8 ms and the co-simulation lost its first 10
  bytes in front of the FCU's receiver enable; an FCU leaving reset up to ~99.5 ms after the COM still gets it whole).
* Reply gate (COM, D5): a SNAP becomes a TELEM only if its reply_seq is the seq of an IL_CMD the COM queued less
  than `COM_REPLY_WINDOW_MS` = 80 ms (20 COM ticks) ago whose slot is still open; accepting it closes that slot and
  every older one. The pending TELEM goes to the radio only while the same window is open, otherwise it is dropped.
  Stray, duplicate and late SNAPs are refused and counted. 80 ms lies above the 68 ms the ground's timeout grants
  (§10.3) and inside the two bounds the COM must meet - reply on the ground before its timeout (h + radio start ≤
  108 ms) and off the air before the ground's next request (≤ 175.2 ms lora_uart / 168.0 ms sx127x);
  `run_com_simbench.py` re-derives the bounds from `sim/link_model.py` on every run (margins 27.8 / 23.1 ms to the
  first, 38.9 / 83.1 ms to the second with the worst radio start).
* Fault behaviour: §10.1 - a dead COM or a cut wire is a lost radio link (I1p / I1); a dead FCU means no SNAP and no
  TELEM. A babbling COM (continuous bytes or a stream of CMD headers) is bounded: in the co-simulation the FCU's
  time-critical part stays ≤ 80 % of the tick, no navigation frame or flight record is lost, no state changes and the
  link resumes. Evidence: `crosscheck_report_ilink.json` (3000 streams, 750 317 bytes: Python = C on every frame,
  counter and need value), `com_simbench_report.json` (both backends PASS), `cosim_report.json`.

### 10.5 nlink: NAV → FCU navigation frame (v0.6.3)

(v0.9: the exchange is interrupt-paced, the VTOL push floor is 890 µs and the exchange watchdog W1 bounds it - §10.6.2,
§10.6.3. VTOL builds use NL_TYPE_NAV4 / NL_TYPE_FCU4 since v0.8.1 (13-byte FCU frame, `src/core/nlink_proto.h`); the
wire, the 72-byte exchange, the FCU semantics, the 4.6 ms frame timeout and the 58-cycle interrupt limit below hold.)

Normative references: `sim/nlink_proto.py` (byte-exact, `SpiSlaveModel` for the slave's byte alignment),
`src/core/nlink_proto.h` (the contract in its header comment), `src/hal/nlink_spi.h`. Summary:

* Wire: SPI, NAV = master (PB0 SS output, PB1 SCK, PB2 MOSI, PB3 MISO input with pull-up), FCU = slave (PB0 SS with
  pull-up, PB3 MISO output, one interrupt per byte); mode 0, MSB first, `flight_computer.nlink.spi_clock_hz` = 2 MHz
  (F_CPU / 8), `byte_gap_us` = 4 µs between bytes so the FCU's slave interrupt can load its next return byte. The
  NAV makes SS an output (high) before it enables the SPI as master.
* Exchange: once per NAV tick, as soon as that tick's navigation state is computed, the NAV pulls SS low, exchanges
  exactly 72 bytes full duplex and releases SS (~630 µs; co-simulation SS low 634–642 µs).
  - NAV → FCU: `0xD5 0x5D | 0x01 | SEQ | PAYLOAD[66] | CRC16 lo | hi`, SEQ = the NAV tick counter's low byte, CRC-16/
    CCITT-FALSE over TYPE, SEQ, payload. Payload: q[4], bias-corrected rates[3], specific force[3], h, v_z,
    h_baro_rel, tilt (float32); imu_fail, baro_fails, twi_errors, loop_us_max (u16); flags (IMU_OK, BARO_OK, ALIGNED,
    GYRO_SAT, EST_VALID, BARO_NEW, BOOT); reset_cause (NAV MCUCSR at boot + IMU / baro init failure + params-tag
    mismatch bits - reported and logged, no flight decision reads them).
  - FCU → NAV, in the same exchange: `0xD5 0x5D | 0x02 | SEQ | state | fflags | params_tag | 0 | CRC16` (10 bytes),
    then 62 × 0x00. SEQ = the last NAV frame the FCU accepted, state = the supervisor state after the FCU's latest
    tick, fflags = BENCH (bench_vanes build), RECOVERY (FCU booted by I10).
  - A frame is valid iff the sync pair, the TYPE and the CRC are right; nothing else is looked at (bytes 10–71 of
    the FCU half are ignored). There is no streaming decoder: the driver delivers whole exchanges, and the FCU slave
    discards a partial exchange (SS high with bytes pending: a lost byte, an extra SCK edge, an SS glitch, a NAV reset
    mid-frame), so an error costs the exchange it happened in and at most one more.
* Timing rule: the FCU's tick starts when the last byte of a complete exchange has arrived. If none arrives within
  `frame_timeout_ms` = 4.6 ms of the previous tick start, the FCU runs the tick on its own Timer2 with the navigation
  data invalid, and every 4 ms after that until frames return (co-simulation `fault_nav_hold`: first fallback tick
  4615 µs after the last frame tick, then 4.004–4.041 ms apart; the gate allows 4.6 ms ± 60 µs for the first and
  4 ms − 20 / + 60 µs after that, because the idle wait finishes the ilink byte it is decoding and the RX interrupts
  queued at that instant before the fallback tick starts - `run_cosim.py`). Why 4.6 ms (params `nlink.note`): the NAV's frame time
  moves by up to ~0.45 ms at the pad → flight switch (co-simulation: largest tick-to-tick increase 435.9 / 450.4 µs
  against the 600 µs the timeout leaves over the tick), and a fallback tick that starts later than ~4.6 ms leaves the
  FCU's background no time and drops flight records (co-simulation A/B in the params note: 4.6 ms 0 dropped, 4.7 ms 1,
  5.0 ms 2–4; only the 4.6 ms run is in the current report), so 4.6 ms keeps ~0.15 ms of margin on both sides.
  `tools/gen_firmware_params.py` requires the timeout to lie between one and two control ticks.
* Semantics on the FCU (the supervisor is unchanged): imu_ok = a valid frame started this tick ∧ IMU_OK ∧ (EST_VALID ∨
  the state is not a flying one); baro_ok = valid ∧ BARO_OK; aligned = ALIGNED; gyro_saturated = GYRO_SAT; h, v_z,
  tilt, rates and q from the frame (the newest valid one is held while none comes). No valid frame (timeout, CRC,
  length, sync) → imu_ok = baro_ok = false for that tick: a dead NAV or a cut wire is an IMU + baro failure, I9 after
  25 ticks, exactly like a dead MPU-6050 on the v0.5 single chip.
* Semantics on the NAV (from the state of the last valid FCU frame; SAFE before the first one after a NAV boot, so
  EST_VALID can only be earned on the pad): the altitude filter is held at the pad (h = v_z = 0, ground reference
  low-pass filtered) while that state is SAFE or ARMED; the estimator aligns on the pad while SAFE or ARMED and BENCH
  is clear, gyro-only otherwise. Because the FCU's bytes go out while it still waits for this tick's frame, the NAV
  acts on a state one FCU tick old: its pad / flight switch lags the supervisor by 2 control ticks (v0.5: 1). At
  LAUNCH the throttle ramps from 0 over 0.5 s, so the vehicle has not moved when the estimator leaves the pad
  alignment; the 6-DOF sim flies this lag (§9).
* FCU interrupt budget around the exchange: 58 cycles is the measured hard limit (`nlink_spi.h`: a competing
  interrupt of 58 cycles kept every byte on time, 82 cycles made some return bytes late; FCU simbench check
  `exchange_isrs_le_58_cycles`, longest measured: ilink RX 57, ilink UDRE 47, log UDRE 46); ~50 cycles is the design
  target of `nlink_spi.h` for every FCU interrupt or cli section (ilink RX 51 typical, log UDRE 46: at the edge). A slave interrupt later than the byte gap leaves the previous byte in the shift
  register: in reply bytes 1–9 the NAV's CRC refuses that FCU frame (it keeps the last valid one), in bytes 10–71 it
  costs nothing. Co-simulation, 150 000 exchanges per backend in normal flight: no navigation frame lost, no FCU reply
  refused after the first, reply bytes late: none (lora_uart) / one, byte ≥ 10 (sx127x); no wrong data accepted in any
  run; the FCU's background ends ≥ 226 µs before the next exchange.
* Fault behaviour: §10.1 (dead NAV / cut wire → I9; NAV reboot in flight → no EST_VALID → I9 ABORT). Damaged
  exchanges (bit flips, dropped / extra bytes, noise): every one refused by the FCU and the NAV, one FCU tick without
  a valid frame per damaged exchange, tick spacing ≤ the frame timeout + 60 µs (the gate in `run_cosim.py`; a dropped
  byte turns the tick into a fallback tick that starts a little after the timeout: max 4620 / 4627 µs in `fault_nl_drop`),
  no state change, frames resume
  (`fault_nl_flip / drop / extra / noise`). Evidence: `crosscheck_report_nlink.json` (5000 + 5000 encodings, 20 000
  decode buffers, PASS), `nav_simbench_report.json` (all checks PASS), `cosim_report.json`.

### 10.6 v0.9 firmware round (2026-10-29; four ATmega128A) - supersedes §10.1-§10.5 where they differ

Governing implementation spec `fw09/spec/SPEC_firmware_v09.md`; acceptance record `fw09/impl/integ/ACCEPTANCE.md`;
final-run evidence `fw09/impl/fix/runs/final/` (`fw09/` = the round's scratch folder
`C:\Users\yc012\AppData\Local\Temp\claude\B--Claude\bed4b475-b8b1-46b7-ab64-c3b11e27401e\scratchpad\fw09\`; copies in
`B:\Video\펌웨어\v0.9\구현\`); Korean summary `docs/v0.9_펌웨어_결과.md`; before state `history/v0.9_pre_fw/` (v0.8.1
firmware code + the v0.9 reference). Every number below is measured in simulation (simavr benches, the 4-core
co-simulation, host tests, crosscheck) unless marked E; **none is measured on hardware**.

**10.6.1 Chips and build configurations (G1).** NAV (MPU-6050 + MS5611, estimator, altitude filter), FCU (supervisor,
vertical / attitude / mixer, outputs, log), COM (LoRa), GNS (GPS receiver, GPS filter, position controller; glink to
the NAV) - architecture, wiring and mass as v0.8.1. The VTOL carrier rev 2c has no 5th chip footprint (the user removed
it 2026-11-05), so the P3 fallback (move NAV work to a 5th ATmega128A; trigger in `SPEC_firmware_v09.md` §1.3) is no
longer on the board: using it needs a board respin (user decision). The NAV firmware reads the bought barometer, a BMP280, by default
since the 2026-11-10 control round (§10.2 hardware; the MS5611 driver is a build option, env `nav_ms5611`). Every VTOL env names its vehicle configuration with `-D FC_VCFG`
(0 battery, 1 tether, 2 hybrid; = the FCU's `PWR_SRC`); `tools/gen_firmware_params.py` emits one constant set per
configuration from `cad/exports/cadquery_report_vtol{,_tether,_hybrid}.json`, the ballistic envs keep the rocket set
(`cadquery_report.json`; `mission.config` stays 'rocket'). The header refuses a VTOL AVR build without `FC_VCFG`, a VTOL
image with the rocket constants and an FCU whose `PWR_SRC` differs. Configuration-dependent macros only: `FC_PARAMS_TAG`,
`FC_VEHICLE_NAME`, `MIX_L_V_M`, `FC_HOVER_THROTTLE`, `VEH_*`, `VC_U_HOVER`, `VC_INV_U_HOVER`, `VC_TO_SEARCH_F`, `POS_TW`
(`fw09/impl/D/G1_DESIGN.md`).

| configuration | NAV / FCU / GNS / COM envs | `MIX_L_V_M` | `FC_HOVER_THROTTLE` | `POS_TW` | `FC_PARAMS_TAG` |
|---|---|---|---|---|---|
| before G1 (every VTOL env) | - | 0.351006 | 0.9160 | 1.1918 | 0x95 |
| VTOL battery (field / demo / no GPS) | `nav` / `fcu`, `fcu_demo`, `fcu_nogps` / `gns`, `gns_demo` / `com` (= `com_lora_uart`), `com_sx127x` | 0.210045 | 0.7301 | 1.8759 | 0x09 |
| VTOL tether | `nav_wired` / `fcu_wired` / `gns_wired` / `com_wired` | 0.184700 | 0.6729 | 2.2083 | 0x67 |
| VTOL hybrid | `nav_wired_buffer` / `fcu_wired_buffer` / `gns_wired_buffer` / `com_wired_buffer` | 0.197319 | 0.7014 | 2.0325 | 0x5D |
| rocket (ballistic) | `nav_ballistic` / `fcu_ballistic` / - / `com_ballistic` | 0.351006 | 0.9160 | 1.1918 | 0x76 |

(`fw09/impl/D/runs/cfg_macro_compare.txt`, profile field.) Before G1 the C commanded 210 / 351 = 0.60 of the reference's
vane deflection for the same moment and a hover feed-forward 25-36 % too high (E, mixer formula; `SPEC_firmware_v09.md`
§3.2). The tether / hybrid FCUs fly without GPS (`ONB_NO_GPS`); `gns_wired*` keep a fitted GNS consistent with the tag.

**Mixed image sets.** The FCU (VTOL builds, `src/main.c`) latches a flag when a valid NAV frame carries
NAV_RC_TAG_MISMATCH (0x20: the NAV saw an FCU of another tag) or an IL_STAT carries another `params_tag`; while it is set
and the vehicle is SAFE or ARMED (motor off) the supervisor's arm-switch input is held open - ON is refused with
`LP_REJ_ARM_SWITCH`, the TELEM ARM_SWITCH flag shows the chain open, a spool-up in progress ends in SAFE; in flight it
changes nothing. The supervisor (`core/onboard.c`) and the reference are unchanged (for a matched set the flag is constant
false). Co-simulation `cl_mixed_images` (tether FCU + battery NAV / GNS / COM): SAFE for the whole 52 s, 100 ON
answered and refused, the NAV tag flag in 12961 / 12962 frames; each latch path alone also refuses. **A GNS of another
configuration is not detected** (glink frames carry no tag; adding one is a change of `sim/glink_proto.py` - a decision):
check the GNS image hash against `FINAL_HASHES.txt` before the first flight of a vehicle.

**10.6.2 nlink timing (P1 items I1-I10, B1, T1).** The NAV master clocks the 72-byte exchange from its Timer3 compare-B
interrupt, one byte per interrupt, 140 cycles apart (`NL_PACE` 107 + 33), TCNT3 = 0 at each exchange start; the 72
interrupts cost ~4.0k cycles (250 µs) of NAV CPU instead of the ~10k-cycle busy-wait (`src/hal/nlink_spi.c`). SS low:
618.44-620.56 µs single-chip bench, ~632 µs on the chip, 636-641 µs in the 4-core co-simulation (params
`flight_computer.nlink.exchange_watchdog_note`). VTOL push floor `NAV_PUSH_MIN_US_VTOL` 1310 → 890 µs (`src/nav/nav.h`;
the NAV bench re-derives it and checks nav.h: 890 = 890), FCU bench `SB_NAV_PUSH_US_VTOL` 1560 → 1140 µs; any change to
the NAV code before the push must re-derive it. Largest tick-to-tick frame step 211.2 µs (inside the 600 µs the 4.6 ms
frame timeout leaves). Also: Q30 / Q32 fixed-point quaternion integration (`src/core/qfix.h` `fx_mulhi`; same series,
Newton window and float fallback as `sim/quat.py integrate`), R(q)^T e_z kept with q, the CRC by a 256-entry flash table,
`lp_quantize` byte shifts, a Q30 asin series for the tilt below 30°, `LATE_WALL` ×115 >> 6, `arm_gates` inlined, FCU slave
reply rows `s_tx[2][16]`. Result: NAV mandatory work (tick → FCU reply, incl. the MS5611 TWI) max 2782.2 µs (69.6 %), mean
2499.9 µs; time-critical (tick → frame delivered) 1759.2 µs (44.0 %); 0 of 1780 bench flight ticks into the next IMU lead;
v0.8.1: 4099.2 µs (102.5 %) (`history/v0.9_pre_fw/README.md`). C == reference: VTOL NAV estimator max |dq| 7.82e-6
(bound 1e-4); AVR `fx_mulhi` vs C 200 000 pairs, 0 mismatches.

**10.6.3 W1 exchange watchdog (NAV flight and co-simulation builds, `NLINK_HW`).** While an exchange is open,
`nlink_spi_exchange_wait` reads TCNT3 (atomically - its TEMP register is shared with the compare ISR) and TOV3; at TCNT3
≥ `NLINK_WD_COUNTS` (16 000 = 1000 µs, params `flight_computer.nlink.exchange_watchdog_us`) or TOV3 set it closes the
exchange with interrupts off and only if it is still open: compare-B interrupt off, SS high, the Timer1 stamp, byte 0 of
the buffer cleared (a partial FCU frame is refused on its sync byte), busy = 0, one trip counted
(`nlink_spi_stats()->resyncs`). The latest close is max(start + 1000 µs, the loop's step-7 wait). Generator and `#error`
checks: 1000 µs > 1.5 × the exchange (945 µs) and < the tick − the push floor (3110 µs). Boot check: after every
driver's init Timer3 must still be the nlink's (`TCCR3A == 0`, `TCCR3B == CS30`, no Timer3 interrupt) - otherwise the
NAV starts no exchange (the FCU sees no NAV: I9 on the pad) and its LED blinks at ~16 Hz; the other Timer3 drivers
(`hal/pwm.c`, `hal/gps_uart.c`, the glink bench wire) are refused at compile time in a flight NAV build. Single-chip bench
builds (`NLINK_HW 0`) have no watchdog. The FCU sees a cut exchange as a resync or a missing frame: one frame lost, its
existing timeout / fallback tick. Co-simulation fault `nl_lose_cmp` (the harness clears OCIE3B at byte 30): 1 trip,
1 timeout + 1 resync, no abort; the same fault on the tree without W1: I9 ABORT (the designed contrast); 0 trips in every
normal and transient run; cost max 0.0 µs NAV tick → vane, +0.9 µs frame delivered (open loop, W1 tree − no-W1 tree;
limit 5 µs). The trip count is not carried in the nlink (protocol change, decision); on hardware it shows indirectly in
the FCU's resync / timeout counters.

**10.6.4 RP1 power telemetry (reference protocol change, `sim/link_proto.py`, `sim/ilink_proto.py`, `sim/power_link.py`,
`sim/link_model.py`, `sim/sim6dof.py`, `sim/ground_station_logic.py`).** The tether / hybrid builds send TELEM v4 (TYPE 0x23,
41 B = TELEM v2 + 2 B) / TELEM v5 (0x24, 47 B = TELEM v3 + 2 B); the 2 B power tail = `v_aux` (tether or buffer voltage)
and `pwr_flags`; `LP_LEN_MAX` 40 → 42, largest frame 47 B (`src/core/link_proto.h`); ilink `IL_SNAP2` 55 → 57 B,
`IL_SNAP3` 62 → 65 B (`pw_vaux`, `pw_flags`, `spare3`; `src/core/ilink_proto.h`); reason codes R_W_LOST 0x30, R_W_SAG 0x31,
R_W_BUF 0x32, R_W_D2 0x33, REJ_POWER 0x58 now in `sim/link_proto.py`; ground bridge LINK_LEN_MAX 42, 47-byte frames
(`bridge_config_gen.h` e6f18bab). Battery and ballistic images are unchanged by RP1 (`LP_RP1 0` in the ballistic
builds). TUNE 7400 pairs (`fw09/impl/C/tune_pairs.md`): 130 / 130 battery runs digest-identical; tether / hybrid 0
outcome flips in 26 runs; ground round trip +7.2 / +10.8 ms (sx127x / lora_uart, by design); 57 / 57 named verdicts
unchanged. Link model (E, measured inputs): lora_uart TELEM5 2.014 Hz against the 2 Hz poll (thin), sx127x 4.439 Hz
(`fw09/impl/fix/runs/final/rtt_numbers.txt`). Comparison videos 29 / 30 (W_supply_sag, H_buffer_low) and the headline
31 in `B:\Video\영상\제어변경_비교\v0.9\04_펌웨어라운드_RP1\`; Korean change log `docs/제어_변경_이력.md` ("v0.9 펌웨어
라운드") and `B:\Video\제어변경_이력\v0.9_제어_변경이력.md` §10.

**10.6.5 Other firmware changes.** The v0.9 control port (`C_port_notes.md` §1-4, 6, 7: att_err_mode 2, per-axis
anti-windup, rotor reaction feed-forward, yaw-trim reserve, two-slope ceiling, k_T search / learning, take-off reference
hold / approach, LAND rebase, touchdown release, keep_vel; power monitor + rules W_ARM / W_SAG / W_LOST / W_BUF / W_D2,
ADC chain; envs `fcu_wired`, `fcu_wired_buffer`). Background reserves re-derived from the co-simulation job lengths:
`BG_RESERVE_US_VTOL` 290, `REC_RESERVE_US_VTOL` 220, COM text 500 µs; text records start only on a frame-seen tick
(acceptance fix F1). ADC interrupt (`__vector_21`, power builds) rewritten 79 / 85 → 45 / 56 cycles (≤ 58, now checked as
an exchange ISR and counted in crit + ISR); the flight ADC chain runs in the new simavr test
`firmware/atmega128_fc/tools/adc_chain_test/`. T2: VTOL boot / decode_log text "RECOVERY (I10): reset in flight ->
ABORT, motor off and locked until power cycle (VTOL: no recovery device)"; RV1 (the sim's "chute" event text) not adopted.

**10.6.6 Tools (S1, X1).** `firmware/atmega128_fc/tools/run_cosim4.py` + `firmware/atmega128_fc/tools/cosim/`
(`cosim4.c`, `plant4.c`, `twi_models.c`): the four flight-code images in lockstep with a plant (vertical point mass + small-angle attitude, ESC /
vane PWM in, ground contact), the MPU-6050 / MS5611 TWI register models, a synthetic GPS from the plant and the COM's
synthetic ground; simavr corrections for the Timer0 preload (IMU lead 497.4-498.1 µs), the TWI START / SLA timing and the
watchdog-reset crash (`firmware/atmega128_fc/tools/cosim/README.md`); 24 runs (normal field / demo / no GPS / wired / wired_buffer, GPS loss
3 s / 20 s, GNS hold / pulse, nav_pulse, nl_flip / drop / extra / noise, cut, reset, nl_lose_cmp, babble, mixed images,
open-loop W1 pair). The plant is validated in the vertical channel only (bench-plant replay h rms 0.026 m); its attitude /
horizontal / wind model is ASSUMED and not validated, so HOLD figures are timing evidence, not flight-dynamics evidence.
`firmware/atmega128_fc/tools/run_all_benches.py`: GNS → NAV → FCU → COM → co-sim → FCU re-judge → ballistic hashes.
Crosscheck (X1): the plant check per configuration (rocket / battery / tether / hybrid), the 6-DOF replays against the
parameters the 6-DOF ran with, `--campaign-set` replay of recorded campaign runs; host test per configuration
(`tools/build_host_test.py --configs --qfix`).

**10.6.7 Acceptance (all 22 rows of `SPEC_firmware_v09.md` §7 PASS; `fw09/impl/integ/ACCEPTANCE.md` §4).**

| quantity | limit | v0.8.1 firmware | v0.9 firmware |
|---|---|---|---|
| NAV mandatory work, bench | ≤ 3200 µs (accept ≤ 3000) | 4099.2 µs | 2782.2 µs |
| NAV tick → vane, co-sim | ≤ 4000 µs (accept ≤ 3800) | 4321.5 µs (open loop) | 3578.4 µs worst (babble), 3375.0 µs normal (closed loop, TWI, HOLD) |
| IMU sample → vane, co-sim | ≤ 4710 µs (accept ≤ 4500) | 4817.5 µs (open loop) | 4076.1 µs worst, 3872.8 µs normal |
| NAV ticks into the IMU lead | 0 | 1389 / 1390 | 0 / 12960 |
| MS5611 in flight | 50 Hz | 0 (BARO-lost LAND 9.9 s) | 50 Hz, no BARO-lost |
| FCU crit + all ISRs, bench | ≤ 3200 µs | - | 3089.4 (fcu) / 2761.4 (wired) / 2790.1 (wired_buffer) / 2167.0 (ballistic) µs |
| stack free (FCU battery build) | ≥ 1024 B | 1029 B | 1215 B (NAV 2678, GNS 1266, COM 2659 B) |
| ilink commands unanswered in HOLD | 0 | 18 / 77 (bench) | 0 (co-sim HOLD 13.1-20.0 s; bench 0 / 77) |
| GNS cycle margin | > 10000 µs | 14466.4 µs | 15001.4 µs |
| glink margin (L = 3) | ≥ 2000 µs | 2791.7 µs | 2548.4 µs min over all runs |
| C == reference | PASS | - | crosscheck PASS 401.7 s; host test 8 × 29 699 checks, 0 failures |

Sources: v0.8.1 bench `history/v0.9_pre_fw/README.md` §4, v0.8.1 co-sim `fw09/p1/README.md`, glink v0.8.1
`SPEC_firmware_v09.md` §2; v0.9 `fw09/impl/fix/runs/final/` (`all_benches_*.log`, `rows_summary.txt`, `crosscheck.log`,
`hosttest.log`). Ballistic images (declared header-only change, code = v0.8.1): nav eb91c396, fcu ef8f9ced, com
e8043085; 75 PlatformIO envs + 3 ground-bridge envs = 78 images, hashes in `fw09/impl/accept/FINAL_HASHES.txt`. The
single-chip FCU bench still reports its self-loaded timing checks as FAIL (the bench's own ~1 ms per tick); the co-sim
judges them (re-judge PASS). Held-out check FRESH3 (seed 10500, run once, `fw09/final/FRESH3_REPORT.md`): (a) C replay
PASS (175 runs, 1 607 410 FCU stack ticks, 0 mismatching; NAV max |dq| 2.22e-5), (b) named 39 / 57 = FRESH2, PASS,
(c) campaign counts inside FRESH2 ± 2 binomial σ FAIL in 5 of 8 groups (sampling, plus the campaign round's push-window
change after FRESH2) - a spec-owner decision.

**10.6.8 Open.** GNS configuration mixing undetected (above); FRESH3 (c); `tools/crosscheck_controller.py` omits `u_cap`
in the vc_margins rebuild of a W_SAG replay (fix in `fw09/final/runs/replay_tool_fix.patch`, not yet applied); no
hardware timing - the measurements of `SPEC_firmware_v09.md` §11 (logic analyser on the nlink, NAV GPIO stamps, IMU →
vane scope, stack paint, real TWI / IMU lead, weights and thrust ≥ 2.6 kgf at 22.2 V, rotor sign, ADC two-point
calibration) come before the first powered flight, as a checklist in `docs/v0.9_펌웨어_결과.md` §7; no GPIO-stamp
measurement build exists yet; `FC_COMPUTE_DELAY_S_MEASURED` stays 4.71 ms until then; the `gains.h` / `vtol_gains.json`
STALE build warnings (not re-tuned); 5 pre-existing `sim/test_vtol_scenarios.py` failures.

### 10.7 Vehicle radio = ESP8266 Wi-Fi bridge (ESP-NOW), firmware round 2026-11-06

User decisions 2026-11-05: Wi-Fi (ESP8266) for the vehicle link where the range allows ("가능하면 wifi방식을 채택"), the
rocket uses the ESP too; range plan E / A in `pcb/vtol_carrier/spec/carrier_spec.json` `radio_decision_2026_11_05`.
Korean round report: `docs/v0.9_펌웨어_결과.md` §11; wiring and build: `firmware/README.md` §0.10,
`firmware/esp_bridge/README.md`. Labels: M measured (simulation / host), E estimated, A assumed.

* **Selection.** `flight_computer.link.vehicle_radio` (`esp_bridge` | `lora_uart` | `sx127x`) is the COM backend of the
  plain envs (`com`, `com_wired`, `com_wired_buffer`, `com_ballistic`; `params_gen.h` `FC_LINK_RADIO`,
  `FC_RADIO_ESP_BRIDGE` = 3); `com_esp_bridge`, `com_lora_uart`, `com_sx127x` (and their `_ballistic` twins) force one
  each, so all three always compile. `flight_computer.link.radio` keeps its meaning as the **link model** of
  `sim/link_model.py` (sim timing, the ground's reply timeout, the generator's link budget): it stays `lora_uart`, the
  pessimistic case for the ESP bridge, because the model has no Wi-Fi entry and an ESP-NOW model changes the assist /
  link-loss timing (control-relevant, next round). The four-core co-simulation keeps its lora_uart COMs.
* **Bridge.** Both ends run one sketch, `firmware/esp_bridge` (roles `vehicle_esp12e` / `ground_esp07s`): a
  byte-transparent UART <-> ESP-NOW bridge. The frame protocol of §10.3 (sync, LEN, CRC, nonce, TELEM v1-v5, the
  command set) is unchanged; the COM and the ground station see the bytes they saw over a transparent LoRa module.
  UART -> air: a complete frame at the buffer start goes out as one packet at once, other bytes after
  `idle_flush_bytes` (3) byte times of silence, an incomplete frame after the longest frame + that gap, <= 250 B per
  packet (`src/bridge_core.h`, the ground_bridge rule). ESP-NOW unicast between two locally administered MACs set by
  the sketch (`link.esp_bridge.mac_*`), fixed channel, 802.11b PHY, TX power per end, 1 Mbps (ESP-NOW default, A).
  Compiled on this PC with arduino-cli and the core esp8266:esp8266 3.1.2 (`firmware/esp_bridge/build_arduino_cli.py`,
  2026-11-06: 5 role / flag variants exit 0, no warnings in the bridge source, deterministic .bin, M); the PlatformIO
  environments are not built (platform espressif8266 not installed). Run-time behaviour on hardware (1 Mbps, the
  rate-limit call, TX power, range) is A - nothing flashed. The portable part is host-tested (zig c++).
* **COM backend `hal/esp_bridge.c`.** USART1 at `link.esp_bridge.uart_baud` = 38400 8N1, U2X 0 (UBRR 25, +0.16 %;
  the ESP's 80 MHz divider +0.016 %; limit `uart_max_error` 0.2 %). The ESP's RST is PC6 (`com.pins.lora_reset`, a
  BSS138 channel, non-inverting), used **open-drain**: driven low to hold the ESP in reset, released to an input (Hi-Z,
  the channel's pull-ups raise RST), never driven high - a manual RST-GND short (carrier J3 5-6, the rocket's flash
  header) cannot short a driven pin (review F1). Only on a **cold COM boot** (PORF, or no reset flag): held low from
  `radio_pins_safe`, released by `radio_init` after `reset_hold_ms` (2 ms); the RX interrupt drops every byte for `boot_discard_ms` (400 ms = `ESP_BOOT_TICKS` 100 COM ticks) after the release (the ESP8266
  ROM's 74880-baud boot log), then all bytes go to the link decoder, which skips non-frames. After a COM watchdog /
  brown-out / external reset the ESP is left running and no window opens (review F2: a COM restart costs the link what
  it cost with the Ebyte module, co-simulation row 18; a hung ESP needs its own watchdog or a power cycle, A). No busy line:
  `radio_send` hands the reply to the UART at once (the ground's one-request-at-a-time polling paces the frames); no
  RSSI; IL_STAT shows "not sx127x" (protocol unchanged). The reply window D5 (§10.4) holds with radio start 0 against
  the lora_uart model's timeout (checked by `run_com_simbench.py`).
* **Wiring.** VTOL carrier rev 2c: ESP-12E U7 (Q4 COM PD3 -> 1 kOhm -> RXD0, Q11 TXD0 -> COM PD2, Q5 COM PC6 -> RST,
  straps, J3 flash header, J26 COM reset hold). Rocket COM board (hand-wired): the same nets through the comms box's
  BSS138 board (ch1 TX, ch2 RST, ch3 ESP TX -> COM RX, ch4 the OpenLog line), ESP-12E on the box's AMS1117 3.3 V
  (`firmware/README.md` §0.10). Open for the user: the rocket's range (apogee 140-150 m + drift) and the box fit.
  **Amends §10.3:** the ESP's TXD0 -> COM RXD1 also goes through a BSS138 channel (direct 3.3 V drive leaves <= 0.3 V
  over V_IH 3.0 V, carrier rev-1 review E2); the LoRa fallback wiring keeps the §10.3 rule (module -> MCU direct).
* **Ground.** `tools/ground_station.py --link-hw esp_bridge` (default from `vehicle_radio`): serial at 38400 with DTR /
  RTS released (an ESP board's auto-reset); `--link-hw lora_uart` / `sx127x` keep the LoRa hardware at
  `link.uart_baud` 9600 (Ebyte module / `firmware/ground_bridge`).
* **Not in this round (control-relevant):** an ESP-NOW link model; the logic-battery telemetry (FCU PF3,
  `flight_computer.logic_battery_monitor`; TELEM v1-v5 have no free field and a longer battery-build reply changes the
  sim's link timing); any onboard low-battery action; the BMP280 driver; tether `bec_min_v`. (Done since, in the 2026-11-10
  control round: the ESP-NOW link model and the BMP280 driver with its sim barometer model.)

## 11. Sizing (`sizing/sizing.py`)

Reads params (+ preset) and, when present, the CAD report. Computes: mass
budget table (structure from CAD or formula estimate), T/W at nominal and peak
thrust, hover throttle, exhaust velocity, vane authority (N·m, rad/s²), roll
authority vs swirl torque, climb energy and peak current for the mission,
battery C-rate check, parachute size for `target_descent_mps`, CG/CP static
margin, Korean regulatory class by mass. Writes `sizing/sizing_report.md` and
`sizing/sizing_report.json`.

## 12. Print and assembly rules (summarised; full guide in Korean docs)

* Print-ready STLs with the orientation applied are generated by
  `tools/make_print_files.py` into `cad/exports/print/`, which also slices
  every part with Bambu Studio's CLI (P2S 0.4 nozzle, `0.20mm Standard @BBL
  P2S` + 5 walls, 5 top/bottom layers, 15 % gyroid, auto brim; Bambu PLA
  Matte) and writes `print_plan.md` with the real print time and filament per
  part; the support listed per part is what the slicer wrote into the G-code header. Tree supports (`tree(auto)`)
  for the payload module (bulkhead 15 mm above the bed; v0.6: the four COM-board bosses under it), the nose (anchor
  bar) and the vanes (the Ø5 shaft stands 1 mm proud of the plate); the intake module (printed flipped) uses normal
  snug supports (`normal(auto)`, `snug`, `support_expansion` −0.5 mm) under the bulkhead inside the spigot, the
  shelf ledge, the spigot step and the lead bulges - none inside the windows (next bullet); the launch pad uses normal
  (grid) supports for the gap under the plate. Plan of 2026-10-15: 30 h 21 min and 1272 g PLA for the set, 24 h 50 min
  and 1070 g without the launch pad.
* v0.6.2 intake print file: the intake is printed from a Bambu Studio project, `cad/exports/print/intake_module.3mf`
  (the oriented part + one support-blocker volume per window opening, taken from the Layout, + Bambu's own resolved
  settings), not from an STL: with the 45° window gables (§5.1) the windows need no support, but the slicer still put
  supports inside every window until they were blocked. `make_print_files.py` checks the sliced G-code (window support
  gate): every support extrusion is mapped back to the design frame through the slicer's own placement, the
  registration is proven on the band ring and on every strut side face, and support is counted where its bead can
  overlap a window opening (the whole sector up to the strut faces and the band's full radial extent, grown by half
  the support line width). Anything but PASS is recorded in the plan, makes `--check` report STALE and exits 3.
  Plan of 2026-10-15: PASS, 0 mm of support in each of the four windows; the bare part sliced as a control: FAIL,
  58 229 mm of support in the windows. Bridge-length / no-support-bridge settings were measured and do not help. A
  part has one print file: writing its STL removes an older `<part>.3mf` and vice versa.
* Print-file freshness (`make_print_files.py --check`): v0.6.1 - the Fusion exports are used only when
  `fusion360_report.json` was written from the current params.json by the repository's script version, with no
  failed component and every part exported; v0.6.2 - also for the preset being sliced and with no unanswered Fusion
  point-containment evidence for a required feature (unless `--allow-unanswered`); v0.6.3 (geometry-aware) -
  `print_plan.json` records the sha256 of every source STL as it was read for slicing, and the print files are
  current when (1) the Fusion run on disk is valid for the preset, (2) every sliced part's source STL still has the
  recorded sha256, (3) every print file of the plan is on disk and (4) the intake window gate passed. A params.json
  change that changes no printed STL therefore leaves the print set current once the Fusion script has been run on
  it; until then the STLs on disk are not known to be the current design's (exit 2). `--adopt` gives a pre-v0.6.3
  plan its source hashes part by part, only where the print file is proven to be the current source STL oriented.
* M3 screws self-tap into 2.5 mm holes in PLA; use M3 heat-set inserts if a
  joint is opened often.
* v0.5 tab screws: each counterbore leaves the cylindrical skin obliquely, so the skin around the
  exit thins to a feather edge (< 0.6 mm) on the side away from the axis; the slicer drops it, which
  only widens the opening. The intake wire bulge's flat roof prints as an overhang when the intake
  is printed flipped (supported like the rest of that part); the channel floor is a 45° wedge so no
  support grows inside the closed channel - check the channel is clear before assembly. Never heat PLA above 50 °C (HDT 52 °C) — the ESC and
  EDF motor are cooled by intake air; the battery bay has vent slots.
* v0.6 servo leads (§3.8): the pod windows (4.6 mm bridge), entry slots (vertical) and deck notches print without
  support in the upright propulsion module; the rebate is an open step. The servo bulge prints like the wire bulge
  (roof overhang under the existing supports, 45° floor wedge inside). Threading: a pull wire down each straight
  column from the deck notch to the entry slot; leads bare through every gate, the connectors for the FCU board
  fitted last (v0.6.3: the leads end on the FCU on the intake bulkhead; extensions 524 / 434 / 450 / 539 mm, §3.8).
  Keep the servo leads apart from the motor phase leads and the ESC (twist each lead's three wires; the route keeps
  them on the opposite side of the vehicle up to the bulkhead).
* v0.6 boards: the COM board is screwed from below into the four payload-bulkhead bosses with M3 × 5 self-tapping
  screws (a longer screw is not allowed: the blind hole leaves 1.2 mm of bulkhead), components hanging down, no
  straight pin headers on that side; the radio/log box hangs beside it on double-sided foam tape and a cable tie,
  the antenna vertical along the avionics wall; the FCU board lies on 3 mm double-sided foam tape on the intake
  bulkhead, held by a cable tie through two opposite vent holes (45°/225° or 135°/315°). The ilink runs from the FCU
  (intake bulkhead) to the COM (payload bulkhead) with the 1 kΩ series resistor in the FCU → COM line; the nlink runs
  from the FCU up to the NAV on the tray. Weigh the FCU and COM boards and the radio/log box: their masses (20 g
  each) are ASSUMED in params.json.
* Bench-test the EDF thrust before any flight; the sizing uses vendor numbers.

**v0.9 print set (provisional, `cad/exports/print/provisional_v09_cadquery/`, `make_v09_print_files.py`)**: every
printed part is sliced from the CadQuery twin's STL with its print class (P2S 0.4, 0.20 mm Standard, Bambu PLA Matte;
classes A load path 4 walls / 4+4 layers, B shell 3 / 4+4, C flat plate 3 / 4+4, D small parts 5 / 5+5, E ground
equipment 3 / 4+4, 15 % gyroid). Until a Fusion 360 GUI run of params v0.9 exists this set replaces
`cad/exports/print` for the masses and the budget; `tools/make_print_files.py` stays the official path.
* `propulsion_module`: upright, exit on the bed, **turned 45° on the plate** - with the fins on the axes it spans
  246.8 mm and the P2S refuses it axis-aligned (wrap-detection zone, measured "plate is empty"); diagonal 176.2 mm.
  **No support**: tree(auto) was refused by the CLI and normal(auto) also filled the closed servo pockets and duct
  cavities (+73 g, measured); the fairing keels are 45°, the pocket ceilings bridge onto the lintel. 350.1 g, 8 h 02
  (v0.9 CAD-1; after the 2026-10-29 lead notch: 349.0 g, 7 h 58 - the print plan is the current value).
* `intake_module`: flipped as v0.6.2 with the support blockers, window support check PASS with the 14 mm gate (the
  22 mm gate FAILED it); 129.6 g, 4 h 51.
* New parts: `vane_coupler` ×4 upright, flange on the bed, D-bore vertical (0.87 g); `servo_cover` ×4 and
  `wire_cover` flat (3.42 g, 2.23 g). `vane_horn` is gone in BUDGET.
* Totals (14 part types, quantities applied): 31 h 43 min, filament 1298 g, parts 1086 g, flight parts 944 g (v0.9
  CAD-1; current print plan `cad/exports/print/provisional_v09_cadquery/print_plan_provisional.md`, 2026-10-29: 31 h 50 min,
  1298 g, parts 1085 g, flight parts 944 g; the
  launch pad 141 g excluded). Assembly: servo + horn + coupler slide into the pocket from the service side under the
  lintel, foam pad, cover with 2 × M2 × 8; motor leads with bullets laid into the open groove, cover with 4 × M2 × 8.

## 13. VTOL configuration (v0.9 CAD-2, `mission.config` = `vtol`)

**Switch.** `mission.config` selects the HARDWARE configuration (orthogonal to `mission.mode`, the flight program):
`rocket` = the v0.9 CAD-1 stack (ballistic launch + CanSat deployment; `mission.mode` `ballistic` needs it) and `vtol` =
the airframe below. `layout.load_params(path, preset, config, power)` deep-merges `configs.vtol` over the preset (BUDGET
only - `Layout` raises for a pushrod / lip-clamp preset) and sets `power.source` (`battery` | `tether` | `hybrid`; the
tether report's `wired` / `wired_buffer` load as `tether` / `hybrid`; a non-battery source on the rocket raises). The
rocket dict is exactly the CAD-1 one (configs / power never leak; `test_vtol_config.py`), so every rocket artifact stays
bit-identical (Layout scalars, STL sha256 unchanged). Reports of the VTOL configuration carry the suffix `_vtol`
(`_vtol_tether`, `_vtol_hybrid`): `build_rocket.py`, `check_geometry.py`, `make_v09_print_files.py`, `replay_x3.py`,
`fusion_mock_run.py` take `--config` / `--power`; the Fusion script reads `cad/fusion360/config.txt` (or
`CANSAT_CONFIG` / `CANSAT_POWER`) and writes `fusion_vtol*/` and `fusion360_report_vtol*.json`. The default stays
`rocket` this round: flipping it changes every default-layout consumer (sim, firmware headers, pinned rocket tests) and
belongs to the sim / firmware round with the VTOL gain re-tune.

**Stack (z, mm, `python cad/layout.py --config=vtol`).** Propulsion 0..182 (EDF section 92: `edf_section_min_h`, the
shortest the liner ring allows), intake 167..339.1 (bulkhead 1.2 mm at 322.9..324.1, two vent holes at 90 / 270°),
avionics 324.1..474.1 (`tube_h` 150; tray 344.1..456.1, windows 343.1..455.1), top cap skirt from 459.1, plate
474.1..475.3, GPS patch top 485.9; `vehicle_h` (cap top) 475.3 mm (rocket 856.3), footprint Ø245.2 (duct OD 93.2: fin
tips at 122.6).

**Top cap (`top_cap`, `Layout._top_cap`, `rocket_geometry.top_cap`, Fusion `build_top_cap`).** Skirt over the avionics
spigot (screwed at the joint thetas like the payload joint), 1.2 mm plate with a flat top (prints top face down), four
1.6 × 12 ribs at 22.5° + k·90° and a Ø16 hub under it joined by a 2 × 3 mm rim ring at r 54.5..56.5. Holes: M5
hardpoint (axis), conductor gland 2 × Ø5.8 at 25 mm / 0°, GPS lead Ø8 at 37 mm / 230°, arming switch Ø6.5 at 40 mm /
90°. On top (taped): the GPS patch on a 40 mm ground plane at 33 mm / 180° and the GPS board at 35 mm / 270°
(open sky: the patch top is the vehicle's highest point). The GPS part is the NEO-7M module (SZH-NT07, Devicemart
no=10895400, user decision 2026-11-04); the CAD board box is still the GY-NEO6MV2 size from params (36 × 25.8 × 4 mm);
the NEO-7M board size is not checked (A). Load path eye bolt → hub → ribs → ring → plate annulus over
the spigot (shear only) → skirt → joint screws. Tether keep-outs (tether report): the free cable from the top of the
≥ 100 mm boot is ≥ 200 mm from every intake window (263 mm); the boot base is 167.5 mm (reported). `cap_clearance`
holds every hole / rib / bay-part / GPS clearance and raises when short.

**Avionics bay.** The carrier PCB (`electronics.carrier`, 76 × 70 × 15, 50 g ASSUMED: 4 × ATmega128A NAV / FCU / COM /
GNS, sensor modules (GY-521, BMP280), shifters, LDO, ESP-12E, OpenLog; rev 2c has no 5th chip footprint - the user
removed it 2026-11-05; the `carrier.model` text in params was aligned with rev 2c in the 2026-11-10
control round) takes the v0.6.3 FCU spot
on the intake bulkhead with every FCU rule (`_fcu`, C25: bore 5.34 mm, tray 2.00, motor-lead hole 3.00, joint zone
4, vent holes 90 / 270 reachable for the tie). Tray 112 mm from bulkhead + 20: battery (-x) and ESC (+x) at the tray
bottom + 2, misc box (25 g) above the ESC (`misc_side` +x). No NAV / COM / GNS / radio box / GPS entry / vent slots.
14 hexagonal windows (w 16, 45° pointed ends, at 0 ± 22k, 180 ± 22k, 84, 106, 254, 276°; struts 6.53 mm at the bore,
7.9° from the rail ribs) and two 24 × 100 mm tray holes. Power sources: `tether` removes the pack and adds the
tether kit (lumped: hardpoint 15 g at the cap, lead + XT90-S 41.1 g, cap bank + clamp 45 g at the ESC); `hybrid` adds
the 650 mAh buffer box in the battery slot and the diode board 21 g.

**Propulsion lightening.** Duct wall 2.4 → 1.6 (`vane_section_od` 93.2), fairing wall / floor / top 1.6 → 1.2, one
pentagon hole per fin outboard of the fairing (`fin.hole`, u 101..118.6, z 10..50.2, 45° gable, frame ≥ 3.6 mm).

**Checks (VTOL rows).** C2, C11, C13, C15, C22, C24, C26, C27, X2 N/A (rocket-only, reason in the row); C6 / C7 / C14
/ C25 / C29 have VTOL branches; new C30 (top cap: no overlap, holes open, ribs / hub / ring material, Layout clearances,
tether path, GPS on top), C31 (windows open, struts / rail ribs material, tray holes open), C32 (fin holes open, frame
material, tip-over 26.3° toward a fin / 19.2° between fins; WARN < 15°). Result: 0 FAIL, 1 WARN (C10, the pre-existing
0.69 mm tab-hole wall).

**Masses and sizing.** Printed parts from the VTOL slicer table (`provisional_v09_cadquery_vtol`, class settings):
propulsion 293.7, intake 122.0, avionics 58.7, tray 14.6, top cap 35.6 g. `sizing/vtol_sizing.py` writes
`sizing_report_vtol*.json/.md` and `tw_table_v09.json/.md` (T/W and hover throttle over T_max 2.0 / 2.3 / 2.6 / 3.09 kgf
for vtol-battery / vtol-hybrid / vtol-tether / rocket; P1 / P2 power; vane authority VTOL vs rocket; tip-over; battery
options as estimates). `sizing/vtol_strength.py` (hand checks, `docs/vtol_strength_checks.json`) and
`sizing/vtol_savings.py` (grams per step, `docs/vtol_savings.json`) document every saving.

### 13.1 v0.9 fix round (2026-10-28) - supersedes the CAD-2 numbers above where they differ

**Motor leads (both configurations; `intake_module.wire_channel.lip_slit / tie_anchors / bulkhead_hole_*`,
`motor_leads.plenum`).** The 1.2 mm lip-pocket wall was a closed ring between the interior (where the bullets are threaded
with the intake held above the EDF) and the groove (where the leads end): a `lip_slit.w` = 6 mm slit at `wire_theta` from
the skirt top to the pocket top lets the bare leads move across (`Layout.wire_lip_slit_w`, `rocket_geometry.intake_lip_slit`).
Between the gate and the bulkhead the leads run at `Layout.ml_u_in` = hole r1 − wire_clearance − od/2 (50.8 mm, 3.5 mm
outside the EDF throat r 45.5) along the 0° strut, tied to `tie_anchors` (block 12 wide on the strut's inner face from
`ml_anchor_u` 53.0, 45° roof toward the bulkhead, a 1.4 × 3 mm tie tunnel; count = fewest for free spans ≤
`plenum.max_free_span` 40: three, 31.4 mm).  Bulkhead hole 20 × 11 at r 42-53 (`bulkhead_hole_w`; corners 3.07 mm inside
the band - the sliced intake passes the window-315 support probe).  Below the skirt top the groove is cut from the skirt
bore's chord (`wire_skirt_cut_r0`), the propulsion slot's inner face from the EDF bore's chord at the slot edge
(`wire_slot_r_in` 48.02): no slivers.  C29 adds (iv) the slit passes a bare lead and the wall ring is open, (v) the
throat clearance, the free spans, the anchors' tunnels and material.

**Direct drive (`vane_drive.coupler.socket / set_screw`, `vane_drive.horn`, `bushing`, `foam_pad`, `flat_side`).** Radial
stack: hub `dd_u_hub0..hub1` (53..61, D-bore with the flat on +v up to `dd_u_flat_end` = shaft end + 0.3, round beyond),
`horn_axial_gap` 2.2 to the horn arms (`dd_u_arm0` 63.2..`dd_u_horn1` 65.0, arms 8.8 × 5.5 × 1.8 ASSUMED, round hub Ø7 to
`dd_u_collar1` 68, screw head Ø4 × 1.3 on the arm face), socket (flange Ø12) from the hub end to 1.5 past the arm plane
(66.5) with a slot 5.55 wide across it (open along v) and a Ø7.2 hub relief; case top unchanged at 70.  An M2 × 3 set
screw at `dd_set_screw_u` (mid engagement) through the hub onto the +v flat locks coupler and shaft: the vane + coupler
stop inward after `dd_inward_stop` 0.3 mm (hub on the pocket's inner wall) and outward at the duct wall (2.0 mm,
measured), where neither the shaft end nor the socket floor reaches the horn (shaft end → screw head 2.4 mm).  The
shaft end gap is 1.5.  The hinge bore is the bushing OD (6.0); brass 6 × 0.45 tube (ID 5.1 ASSUMED) from the duct wall to
the boss face, `hinge_bushing_k` placeholders (0.47 g).  EVA pad 3 mm between case and cover (compression 0.27..0.50).
Estimated lost motion: 2 atan(slot_fit / arm_len) = 0.65° + gear 1° ASSUMED.  C5 adds the removal order (the coupler
sweep is blocked by the installed shaft; clear with the vane pushed in by the engagement + 1 mm) and the set-screw key
path; C28 adds retention (measured inward / outward stops, horn untouched), foam, bushing, set-screw thread.

**VTOL configuration.** `configs.vtol.fin` = span 76.8, no hole (the hole's bottom strip carried ~90 % of the lateral load:
~1.5× instead of the reported 4.0×).  Tray plane x 14.6, tray 107.5 wide, holes |y| 4-33; side windows 90/112/248/270°
(rails at ±75.6°, 3.4° clear).  Top cap: gland at 180° (x −25, over the battery slot), GPS patch at 0°, `top_cap.hardpoint`
= M5 × 16 eye bolt (collar on the cap top), D32 × 2 aluminium washer in a flush Ø32.5 × 2 recess in the hub / rib bottoms,
M5 jam nut (bolt end 459.3, nut bottom 459.4; ≥ 3 mm over every bay box, flight pack 4.2).  Tether kit:
`power.tether_kit_ASSUMED.boxes` (cap bank, bus clamp, XT90-S; hybrid + diode board) on the tray's −x side, placeholders
(`Layout.tether_boxes`, cap bank 34.5 mm from the ESC's input end), the eye bolt / washer / nut as placeholders, the boot
and the 10 AWG pair lumped (`tether_lead_polyline`).  C33 (both configurations): standing pitch trim W·r / (2 N l) at
hover ≤ 5 % PASS, ≤ 10 % WARN: rocket 2.8 %, VTOL battery 1.8 %, tether 7.0 % (WARN, no pack on the −x side), hybrid
4.9 %.  C32 for solid fins: plate material, tip-over with the CG offset.

**Masses (slicer).** VTOL battery 1647.2 g (propulsion 304.5, intake 122.6, avionics 58.6, tray 14.2, top cap 35.2 g),
tether 1399.3, hybrid 1520.3; rocket 2592.6 (propulsion 350.0, intake 130.2).  T/W 1.3 at 2.14 / 1.98 / 1.82 / 3.38 kgf.
*Lead-exit round 2026-10-29 (supersedes):* battery 1655.9, tether 1408.0, hybrid 1529.0, rocket 2601.2 g (propulsion
303.7 / 349.1); T/W 1.3 at 2.15 / 1.99 / 1.83 / 3.39 kgf on mass, 2.22 / 2.05 / 1.89 / 3.50 kgf with the motor-lead drag (E);
C33 2.4 / 7.8 / 5.6 / 3.0 % on the CG alone, 5.2 % WARN / 10.9 % FAIL / 8.4 % WARN / 4.6 % PASS with the lead drag.
*Photo round 2026-10-29 (supersedes):* battery 1655.1, tether 1407.2, hybrid 1528.2, rocket 2600.3 g (propulsion
303.7 / 349.0, motor leads 51.0 g); T/W 1.3 at 2.15 / 1.99 / 1.83 / 3.39 kgf on mass, 2.20 / 2.03 / 1.87 / 3.47 kgf with
the lead-bundle drag (E); C33 2.4 / 7.8 / 5.5 / 3.0 % on the CG alone, 4.4 % PASS / 9.9 % WARN / 7.6 % WARN / 4.1 % PASS
with the bundle drag.
*Fix round 2026-10-29 (supersedes the drag numbers):* masses unchanged; T/W 1.3 at 2.20 / 2.04 / 1.87 / 3.48 kgf with the
lead drag (242 mm², D/T 2.39 %, E); C33 4.7 % (worst Cd 5.2 %) WARN / 10.3 % (10.8 %) FAIL / 7.9 % (8.4 %) WARN / 4.3 %
(4.6 %) PASS, judged on the worst end of the Cd band. The hover power / supply-limit rows of `sizing/tw_table_v09.md` are
mass only (the drag adds ~3.7 % hover power, E); a bench thrust of this EDF as built already includes the drag (the
derate applies to the proxy table only).
`sizing/vtol_sizing.py` adds the end-of-pack voltage condition (3.7 V/cell loaded: T ∝ V^1.61, 0.87×) and the
supply-limited T/W of one RSP-1500-24 (25.2 V / 60 A, loop 59.5 mΩ: 21.6 V, 1298 W) with the hybrid crossover (40.3 A).
`massprops.precise_volume` (adaptive GProp, ε 1e-7) is used for the STL check and the savings; report entries keep the
default integration (bit-identical history) and list the difference in `extra.printed_volume_precise`.

### 13.2 v0.9 simulation / control / tether round (2026-10-28; Python reference + sim only)

Korean summary with every table: `docs/v0.9_시뮬레이션_제어_결과.md`. The firmware (C core, generated headers,
crosscheck vectors) is ported in the next round from `C_port_notes.md` (copy `sim/out/v09_fix_round/C_port_notes.md`).
Thrust is NOT measured: the sim flies T_max 2.14-3.09 kgf at 22.2 V (PROXY envelope, `edf.thrust_v09`).

**Plant (`sim/sim6dof.py`, `sim/vehicle.py`, `sim/edf.py`, `sim/tether.py`).** `mission.vtol.hardware_config` = `vtol`:
every VTOL flight flies the VTOL configuration (no chute / CanSat / payload chain) with the mass properties of
`cadquery_report_vtol{,_tether,_hybrid}.json` (battery 1647.2 g, CG 247.0 mm, Ixx 0.0351 kg m², l_v 210.0 mm; tether
1399.3 g / 221.7 / 0.0290 / 184.7; hybrid 1520.3 g / 234.3 / 0.0319 / 197.3; lateral CG 0.61 / 2.06 / 1.52 mm;
after the lead-exit round 2026-10-29: 1655.9 g / 246.3 / 0.0351, 1408.0 g / 221.0 / 0.0289, 1529.0 g / 233.6 / 0.0318,
lateral CG 0.80 / 2.28 / 1.73 mm; photo round 2026-10-29: 1655.1 g / 246.4 / 0.0351, 1407.2 g / 221.1 / 0.0289,
1528.2 g / 233.7 / 0.0318, lateral CG 0.80 / 2.28 / 1.73 mm - read from the reports automatically; the sim does not model
the lead drag yet).
Direct-drive vanes 1:1 with a lost-motion operator (`vane.direct_drive_play`: 0.65° + gear 1° ASSUMED, sampled 0.3-1.0
+ 0.5-2.0°). Rotor gyroscopic and spool-up reaction torques (`edf.thrust_v09.rotor_inertia_kg_m2_ASSUMED` 4.5e-5,
ω_ref = no-load KV × 22.2 V, `edf.rotation` CW ASSUMED). Thrust ∝ V^1.61 (end of pack ×0.87). Electrical plant
battery / tether / hybrid (`power.electrical_plant_ASSUMED`; supply CC / latch, ideal-diode hybrid bus, events unplug /
cut / trip / sag) and the tether rig (`power.tether_plant_ASSUMED`; catenary, catch line, snag). Fix round (independent
physics review): vane-force spread ×0.72-1.05 with the yaw lever, residual-swirl SIGN drawn ± (p 0.5), ground effect
T × (1 + a_T e), vane q × (1 − a_q e), e = exp(−4h/D) (ASSUMED shape), catch-line devices SRL / auto-locking belay (line-
speed lock, free pay-out; bungee damper on the stretch rate), slack feed ≤ 0.25 m, per-channel ADC gain error,
regeneration with brake chopper and supply OVP. `run_scenarios.py --plant v081` turns the play / rotor terms off.

**Control (VTOL build only; the ballistic `gains.json` is unchanged).** `control.vtol_overrides`: rate gains scaled by
the inertia ratio (rate_kp/ki/kd_xy 0.5243 / 0.1712 / 0.01642 = ×0.266; rate_kp/ki_z 0.0661 / 0.0841 = ×0.647;
pitch / roll PM 18.3° → 40.4°, GM 1.60 → 6.01 at K_m 1); `att_err_mode` 2 (reduced attitude error e_xy = 2(wx − yz,
wy + xz), yaw separate - v0.8.2 candidate D, always on); `aw_per_axis` 1 (per-axis conditional anti-windup); rotor
reaction feed-forward (`ff_rotor_h_nms` 0.15169, τ 0.15 s, `ff_rotor_react` 1; the gyro feed-forward `ff_rotor_gyro` 0);
`mix_yaw_trim_reserve` 1 (the mixer keeps the yaw-integrator trim when pitch / roll saturate). Vertical
(`sim/vertical_ctrl.py`, `sim/out/vtol_gains.json`): vz_kp 0.05, vz_ki 0.008, alt_kp 1.0 (vertical PM −36.7…−42.8° →
24.6…32.8° with the baro thrust-compensation coupling); `mission.vtol.takeoff_v09` two-slope ceiling (u 0.5 in 0.5 s,
then 0.3/s), take-off thrust search 0.08/s, k_T learning from the lift-off (τ 0.4 s, lift-off latch f_z > 1.1 g), the
reference held until the lift-off; reference approach (step ≤ |d| >> 8, ≥ 0.2 m/s); LAND reference rebase to max(ref, h);
touchdown thrust release (0.2/s to −0.12, ASSUMED) while floating in the flare band; `thrust_learning.k_min` 0.55.
Position: `touchdown_policy` `keep_vel` (v_sp 0 in the flare band). Not adopted (TUNE worse): thrust-normalised yaw
integrator, V16, mixer yaw reserve in degrees, reference governor, cap_margin, NAV-lagged throttle.

**Power rules (`sim/onboard.py` PowerMonitor, `mission.vtol.power_rules_ASSUMED`; battery build bit-identical).** Q4 integer
filters (0.0275 V per count, 32 ms IIR). W_ARM: ON refused outside V_tether 24.6-25.6 V (hybrid: buffer 3.72-3.92 V/cell).
W_SAG (tether): < 20.0 V for 0.3 s → warning, frozen thrust ceiling `u_cap`, UP refused; 1.0 s → LAND. W_LOST (hybrid):
tether < 20.0 V or 0.5 V below the buffer for 40 ms → LAND on the buffer. W_BUF: buffer < 3.4 V/cell 0.2 s or < 3.6 V/cell
2 s → LAND. W_D2: buffer charging in flight (> 3.95 V/cell 0.5 s) → LAND. SAFE (motor off) only on the pad (take-off
check not passed AND support signature AND no lift-off latch); otherwise LAND. Tethered limits: hover 1.5, alt_min 1.0,
alt_max 2.0 (UP refused), ceiling 3.2, geofence 3.6 m; flight 180 s − 10 s reserve, hard cutoff 200 s. Reason codes
R_W_LOST 0x30, R_W_SAG 0x31, R_W_BUF 0x32, R_W_D2 0x33, REJ_POWER 0x58 (`sim/power_link.py`); TELEM v4 / v5 = v2 / v3 +
2 B (v_aux, pwr_flags). Procedure: `docs/유선_시험비행_절차.md` (pulley ≥ 4.5 m, SRL / auto-locking belay, slack ≤ 0.25 m,
ADC two-point calibration, hover current ≤ 0.7 × supply limit, no pushes in the hybrid demo).

**Evaluation.** Harness groups field_battery 120, field_swirl 80, field_swirl_neg 40, thrust_low 60, thrust_stress 30,
demo_push 60, tether 40, hybrid 40 + 57 named scenarios; seeds TUNE 7400 (tuning only), EVAL 8300 (one run per round;
the fix round's re-run counts as tuning), FRESH 9200 (reviewer, consumed), FRESH2 9700 (fix-round verification,
consumed), campaign 1090. New VTOL criteria: touchdown ground speed ≤ 0.3 m/s, footprint tip margin > 0, climb overshoot
(motor start → HOLD + 3 s) ≤ 0.5 m (tether / hybrid 0.3 m), not hanging on the catch line, cable loop clear of the intake.
Results (pass / abort): FRESH2, same hardened plant, control before → after the fix round: field 5/26 → 22/17 of 120,
field_swirl 2/29 → 9/19, field_swirl_neg 0/30 → 2/22, tether 1/14 → 6/4, demo_push 0/57 → 0/56, hybrid 0/33 → 0/33; named
29 → 39 of 57. Campaign 1090: field 500 pass 89, abort 63 (53 with negative swirl); demo 5-15 N pushes 282/300 abort;
tether 14 pass / 28 abort of 150; hybrid 129 abort of 150. Thrust sweep (same 30 samples): pass 0/30 up to 2.30 kgf, 2 at
2.46, 4 at 2.62 (overshoot p50 0.38 m), 12 at 3.09; V15 stopped 20/30 on the pad at 1.50 kgf, 4/30 ran 54-60 s on the pad
without V15. Bench gate proposed: T_max ≥ 2.6 kgf at 22.2 V (2.46 kgf minimum = T/W 1.3 at the end of the pack).

**FCU cost (estimates; firmware round measures on simavr).** Against the v0.8.1 measured paths: tilted (worst) about
−143 µs, level about −118 µs, tether builds +~20 µs; static RAM +4 B (battery build 1 B left), wired_buffer build +12 B
(7 B to free). Nothing added to the NAV (102.5 % in the v0.8.1 profile; timing closure is the firmware round's first item).

Evidence state (2026-10-28, params.json sha256 ae0b99e4…): pytest (whole project) 1097 passed, 58 failed, 15 errors,
8 skipped, 2 xfailed. 67 of the failures / errors are `tools/test_gen_firmware_params.py`: the generator refuses
`control.vtol_overrides.att_err_mode` (no `fc_vtol_gains` field; also aw_per_axis, ff_rotor_*, mix_yaw_trim_reserve) -
the firmware round adds the C fields and regenerates. The others: `bridge_config_gen.h` STALE, the pre-existing
build-flags test, and four VTOL scenario tests open since the plant / control rounds (push yaw rate on the light vehicle,
k_T 1.40 vs 1.25 ± 0.06 for the ×1.25 plant, the weak-plant motor-time bound of the v0.8.1 ramp, push return).
**Stale until regenerated**: `params_gen.h`, `gains.h`, `bridge_config_gen.h`, the crosscheck vectors, the CAD / sizing
reports' params_sha256 (geometry unchanged).

## Change log

**v0.2 (2026-10-12)** — every change came from a tool result, not a guess:

| change | evidence |
|---|---|
| vane span 8–44.5 → 13–43 | CadQuery C4/X1: at ±25° the tip corner left the Ø90 duct and adjacent vane roots overlapped 17 mm³ |
| hinge boss Ø14 / r 55 → Ø12 / r 52 | C5: MG90S ear ∩ boss 1.95 mm³ |
| pod cavity 24 → 23.6, side wall 3 → 4.7 | C10: wall beside the Ø1.8 ear hole was 0.10 mm |
| latch arm 14 → 10, anchor bar 4 → 8 wide | C15: arm cut 9 mm³ into the nose skirt; C10: 0 mm wall beside the tether eye |
| spigot shoulder rings + propulsion deck explicit | Fusion dropped the spigot ring (self-overlapping profile) and the two generators differed by 1.4–5 % until both modelled the ring |
| rails clipped at `body_r` | Fusion rail boxes protruded ≈ 5 mm outside the avionics tube |
| nose wall = normal offset | Fusion ShellFeature vs the radial approximation differed by 2.6 % |
| climb throttle 0.9 → 1.0, target 150 → 140 m, geofence 220 → 170 m | T/W 1.28: 0.9 throttle gave 0.3 m/s² climb; full throttle overshoots to ≈ 148 m |
| coast_control_hold_s = 0.5 s | sim: 98 °/s yaw spin-up at cut-off without it, 30 °/s with it |
| vehicle chute 1.6 → 1.7 m | sizing: required Ø 1.60–1.71 m depending on the tethered mass |
| ATmega128 flight computer, loop 250 Hz, 50 × 50 mm board, avionics 205 → 215 mm | user requirement (2026-10-12) |
| launch pad Ø240 → Ø190 | Bambu Studio CLI refused the 240 mm plate (P2S wrap-detection zone) |
| 66 mm / 70 mm-EDF preset removed | T/W 0.93 |

**v0.3 (2026-10-13)** — budget redesign (user requirement: 200-300k KRW for the vehicle, CanSat excluded):

| change | evidence |
|---|---|
| default preset `BUDGET_90mm_6S`, full design kept as `KAIST_90mm_12S` | `tools/make_budget_preset.py`; the full preset reproduces the v0.2 layout (checked attribute by attribute) |
| Hobbyocean 90 mm 6S EDF + 100 A ESC + one 6S 2200 | `docs/budget/price_research_2026-10-13.json`: best thrust per KRW (proxy 3.09 kg @ 22.2 V for 83,400 KRW); 70 mm 6S units could not lift the airframe |
| 350 g / Ø66 x 115 mm CanSat, envelope 150 mm | `tools/budget_trade.py`: with ~3 kg thrust the light airframe carries 300-400 g; KAIST 650-1000 g payloads are out of reach in this budget |
| walls 1.2 mm, duct 2.4 mm, fins 2.4 mm, tray 2 mm, piston 2.4 mm, EDF section min 100 mm | printed mass 1448 -> 948 g (CAD) |
| 45 deg cone chamfer (`Layout.cone_outer_radius`, `shoulder_ring_profile`) | the Ø98 housing's sleeve (r 51.35) poked through a cone that started at the 2.4 mm duct (r 50.07 at the shoulder) |
| fins 76 mm span / 60 mm tip chord, battery above misc electronics, nose 140 mm, intake windows 117 mm | static margin 0.14 -> 0.52 cal (CAD C13) |
| chute 1.4 m (DIY ripstop), battery divider 47k/4.7k for 6S | sizing: required Ø 1.32 m; ADC uses 89 % of the 2.56 V range at 25.2 V |
| CadQuery nose solid tip 8 -> 2 mm, Fusion sleeve starts at the EDF shoulder | Fusion vs CadQuery nose -1.13 % -> -0.04 %; the old Fusion sleeve left an unsupported ring outside the chamfer |

**v0.4 (2026-10-13)** — user requirements: command link over the user's LoRa module; only attitude control on the vehicle.

| change | reason |
|---|---|
| onboard mission state machine replaced by the onboard supervisor of §8.8 (commands + interlocks I1-I10) | user requirement; interlocks kept because LoRa is slow and can drop out |
| launch / cut / release automation moved to the ground station assist mode | same |
| apogee limiter at 150 m onboard, geofence 170 -> 160 m | ground cut arrives 0.3-0.6 s late; the limiter keeps the legal 150 m ceiling |
| SBUS receiver and inverter removed, LoRa link §10.3 (UART and SX127x backends) | the user already owns a LoRa module; its type is not known yet |
| coast_drag_factor 0.85 -> 1.20; predictive auto-RELEASE; poll_hz 4 -> 2; link_loss_disarm_s 1 -> 2 s, link_loss_abort_s 2 -> 3 s | v0.4 sim: apogee 152-158 m with 0.85, nose opening 1.2 s late with the literal rule, and the 2.4 kbps UART link's 0.43 s round trip cannot poll at 4 Hz (one lost poll disarmed the pad) |
| assist auto-CUT decides half a telemetry interval ahead (P + max(dP/dt, 0)·T_tel/2) | at 2 Hz P jumps ~14 m per telemetry; on SX127x the onboard I4 limiter cut before the assist |
| telemetry reason: a latched interlock code is no longer overwritten by a later rejection | ground-stage practice flights: 3/19 operators saw only "rejected: state" after I4 / I6 had acted |
| sim link model: vehicle reply 1 -> 16 control ticks | measured in simavr: 3-5 ticks on the pad, up to 16 in powered flight |
| SX127x examples: 900 MHz RFM95W / Ra-01H instead of Ra-01 / Ra-02 | Ra-01 / Ra-02 are 433 MHz parts |

**v0.5 (2026-10-13)** — user request: mount the EDF on its own screw tabs (option 2 after the fit review against the vendor drawing).

| change | reason |
|---|---|
| `edf.mount` = `tabs`: keyways, two screw bosses with M3 × 20 tangential screws, keyway floor as the axial seat (§3.7) | the vendor drawing shows two tabs 114.2 mm across; the v0.4 model had none, so the real EDF would not enter the Ø99.5 sleeve |
| EDF section outer skin cylindrical above `z_cyl0` (131.0); cone 95.2–131.0 | the tabs reach r 57.1, outside the old cone skin |
| bell lip modelled from the drawing (depth 11, front face 81.1 above the housing rear); lip floats in a 12.5 mm pocket, shelf = back-up stop | v0.4 assumed a flat 4 mm lip and 85 mm length; the real bell would have stood 6 mm proud and kept the intake module from closing |
| motor-lead path: wire slot in the propulsion module, channel + bulge past the lip in the intake module, bulkhead wire hole; intake struts turned 45° (one over the channel) | there was no passage from the EDF to the ESC (0.75 mm gap between housing and sleeve) |
| thin liner tube (1.6 wall + top ring) under the housing instead of a taller solid shoulder ring | keeps the jet step-free without ~30 mm of solid ring; a solid liner (22 g, low) pushed the static margin below 0.5 cal |
| v0.5 review fixes: counterbore to `tab_screw_t_cbore_end`, lugs per screw forming the keyway floor, clearance hole 3.2, wire slot split in two boxes, 6 mm wire gates with a shelf notch and a closed bulge, window_margin_below 4 → 5 | independent reviewers measured: head blocked by a skin brow, tab floor 4.8 mm² on the sleeve end, 0.59 mm hole wall, skin gouged to 0.88 mm, 1.4 mm wire gates (C20 FAIL), open bulge underside |
| KAIST preset keeps `lip_clamp` (v0.4 geometry, identical layout) | no drawing of the JP Hobby unit |
| v0.5d final-review fixes: channel floor 45° wedge, bulge down to 177.65, lip pocket ID 113.5, channel depth 6, tabs at 90°/270° with the leads at 0° (ESC side), bare-lead threading; tray stop and blind propulsion joint holes (both presets) | independent assembly review: bullets cannot pass the 6 mm gates, tree supports filled the closed channel, the tray slid 20 mm down (CG −4.7 mm), joint screws could reach the EDF, 0.25 mm bell play |

**v0.6 — vane-servo lead route (2026-10-15)** — found while fitting the real EDF: the four MG90S leads (≈ 25 cm) had
no path from the vane pods (Z 18–48) to the flight computer (Z ≈ 458–508).

| change | reason |
|---|---|
| `servo_leads` params; pod lead windows, outside run, four entry slots through the shoulder ring / cone skin, straight columns in the annulus, deck notches, spigot-ring rebate to 180°, intake servo channel + bulge + floor wedge + shelf notch at 180°, bulkhead servo hole (§3.8, §5.1; tabs preset only, KAIST unchanged) | no designed path: 0.3 mm beside the case for the lead, solid 2.4 mm shoulder ring, closed skin, motor path sized for 3 × 12 AWG |
| route chosen over joining the motor path and over an external raceway | §3.8 "why": the motor channel has no room and would put the signal leads beside the 100 A phase leads; a raceway costs ≈ 10 g low (0.496 cal on the v0.5d vehicle), drag and three joints; the internal route removes 8.0 g low: Δ margin −0.002 cal (0.520 → 0.518 with the COM board) |
| lumped part `servo_leads` (27.4 g at Z 291.5, 10 g/m ASSUMED) in both generators, sizing and sim (v0.6.3: 21.5 g at Z 245.9, the leads end on the FCU - see v0.6.3) | the 2.4 m of extension wire is heavier than the modelled arms and pushrods; C13 must see it |
| C23 (route proxies, bulge closure, deck seat, sleeve); C20 allows the servo channel opening; C21 keyway probe up to `sl_rebate_z0` | SPEC 7: evidence for every new passage; the rebate opens the keyway's outer wall 8.65 mm above the tab tops on purpose |

**v0.6.0 — two ATmega128A: FCU + COM (2026-10-15)** — user rule (2026-10-15, params `flight_computer.architecture`):
every flight computation runs on an ATmega128A, with any number of chips.

| change | evidence |
|---|---|
| a second ATmega128A (COM) takes the LoRa radio, the uplink decoding and the TELEM building; every flight decision stays on the FCU, which in v0.6.0 still read the sensors (§10.1) | user rule; a dead COM maps onto the existing link interlocks I1p / I1 (co-simulation `fault_cut`: ABORT 750 ticks after the last IL_CMD) |
| ilink (§10.4): 250 kbaud UART, fixed-length CMD / SNAP / STAT frames with the radio link's CRC; streaming decoder with bounded work per byte and no replay | the replaying decoder cost up to 6316 cycles for one byte (~1821 per byte on sync spam) on the FCU (`ilink_proto.h`); now ≤ 91 cycles; Python = C on every frame and counter (`crosscheck_report_ilink.json`, 3000 streams, PASS) |
| COM: IL_STAT paced behind an IL_CMD (D4); 80 ms reply gate (D5) | an IL_CMD queued behind a 42-byte STAT reached the supervisor one tick late; the gate keeps every TELEM inside the ground's half-duplex window (`com_simbench_report.json`: PASS on both backends, margins 27.8 / 23.1 ms) |
| COM board under the payload bulkhead on four bosses with blind M3 holes, placeholder `com` (§5.3, §6), check C22 | C22 PASS (tray 2.00 mm, battery 5.00 mm, spigot bore 21.64 mm; bosses solid, holes clear, 1.2 mm skin); static margin 0.510 → 0.520 cal with the board (§3.8) |

**v0.6.1 — intake band (2026-10-15)**

| change | evidence |
|---|---|
| windowed band 1.2 → `window_wall_t` 3 mm, thickened inward; windows as radial sectors; struts 14 → 18 mm with radial side faces; both presets (§5.1) | user: the 1.2 mm struts flexed while printing and broke when the supports were removed (params `window_wall_note`); C9 PASS (on the v0.6.3 solids) - windows clear, strut sections 52.65 mm² (≥ 0.95 × design), band 3.00 mm; inlet ratio of the square-ended sectors at `r_band_in` 6.46 (`inlet_area_ratio_square`, ≥ 2); Fusion vs CadQuery intake ΔV +0.03 % |
| C9 judges the minimum opening of the built band with solid evidence; C10 lists the band; C20 / C23 measure the leg beside the strut to the band | gaps 2.17 mm (motor leads) / 2.58 mm (servo bundle, ≥ 0.4) |
| print files only from a valid Fusion run (same params.json, script version, every part exported) (§12) | `make_print_files.py` docstring, v0.6.1 |

**v0.6.2 — window gables, intake print gate, radio/log box (2026-10-15)**

| change | evidence |
|---|---|
| lower end of every window = truncated gable, 45°, 20 mm bridge; upper end square; both presets (§5.1) | the flipped intake print still put supports inside every window (verifier finding on v0.6.1, `make_print_files.py` docstring); user decision: no support inside the windows, so nothing to rip off the struts (`layout.py` `_window_gable`, params `window_bottom_end.note`); C9 PASS - gussets solid, bridge flat, all 16 sloped lower edges ≥ 38.83° (limit 35°); the gussets take the inlet ratio from 6.46 to 5.97 (≥ 2) |
| intake printed from the Bambu project `intake_module.3mf` with a support blocker per window, checked on the sliced G-code; the Fusion run must be for the preset sliced and leave no point containment unanswered (§12) | `print_plan.md`: window support gate PASS, 0 mm of support in each window (bare part: 58 229 mm, FAIL) |
| radio/log box (LoRa + antenna, OpenLog, 2nd level shifter, 2nd AMS1117) under the payload bulkhead; `electronics.misc` 60 → 40 g (§5.3, §6), check C24 | user 2026-10-15 (params note: raises the CG for static margin, antenna away from the ESC and the motor leads); C24 PASS (COM board 5.00 mm, joint-screw axes 9.01 mm, spigot bore 4.80 mm) |
| window height kept | user: lowering the windows was rejected; the ~0.02 cal per −10 mm quoted for it is NOT VERIFIED (no report on disk) |

**v0.6.3 — third ATmega128A: NAV (2026-10-15)** — CAD checks (params sha256 0a5c261b…, 2026-10-16; the same result on
3f31c0a1… on 2026-10-15): default preset 22 PASS, 3 WARN (C10 derived features, C12 T/W 1.195, C13 0.51 cal), 0 FAIL;
KAIST 18 PASS, 2 WARN, 5 N/A.

| change | evidence |
|---|---|
| NAV chip: MPU-6050 + MS5611, estimator, tilt, altitude filter; nlink SPI, one 72-byte frame per tick pushed by the NAV; the FCU's tick starts on the frame, Timer2 fallback (§10.1, §10.5) | the v0.6 FCU's time-critical part measured 3308.5 µs = 82.7 % of the tick with every interrupt (`simbench_report.json` `v06_reference`; `nlink_proto.h` quotes 78.8 % and ~83–86 % worst case - the 78.8 % is in no report on disk); v0.6.3: FCU ≤ 64.4 %, NAV ≤ 60.9 % (limit 80 %); three-chip co-simulation, both radio backends, normal flights and faults: 43 / 43 criteria PASS (`cosim_report.json`) |
| `flight_computer.nlink.frame_timeout_ms` = 4.6 | params `nlink.note`: A/B in the co-simulation 4.6 / 4.7 / 5.0 ms → 0 / 1 / 2–4 dropped flight records (not in the current report, which holds the 4.6 ms run); largest frame-ready increase 450 µs < the 600 µs margin; `nav_simbench_report.json` re-run on 4.6 ms (2026-10-16: FCU timeout margin 600 µs, frame-ready increase 271 µs, all checks PASS) |
| imu_ok = valid frame ∧ IMU_OK ∧ (EST_VALID ∨ not flying); a NAV reboot in flight → I9 ABORT (§10.5, §8.8 I9) | co-simulation `fault_nav_hold` / `fault_nav_pulse`: ABORT (I9) 25 ticks after the last valid frame, RELEASED 125 ticks later; FCU simbench check `nav_reset_in_flight_i9_abort` |
| boards: NAV at the v0.5 FC spot (`fc` → `nav`), FCU flat on the intake bulkhead on foam tape + cable tie (§5.2, §6), check C25; the servo leads end on the FCU - 21.5 g at Z 245.9, extensions 524 / 434 / 450 / 539 mm (§3.8) | C25 PASS (tray 2.00 mm, ESC 12.04 mm, rising lead bundles 24.50 / 26.70 mm, tie runs 45°/225° and 135°/315°); C23 PASS; no printed part changed: `make_print_files.py --adopt` proved the 10 recorded print files to be the v0.6.3 Fusion STLs oriented (launch_pad not recorded: no print file under its plan name); X3 PASS (vehicle 2585.8 / 2585.7 g, CG Z 376.29 / 376.28 mm) |
| sim: compute delay = the measured 4.71 ms (was one tick), NAV pad / flight switch 2 ticks late; link model `FC_REPLY_DELAY_S` 14 ms + `COM_FORWARD_S` 0.6 ms (was 16 ticks), ground timeout from `FC_REPLY_ALLOWANCE_S` 68 ms (unchanged 531 / 292 ms); gains retuned (§9, §10.3) | co-simulation: sample → last vane register max 4.66 ms, p50 4.24 ms in flight; uplink → TELEM ≤ 13.2 ms; `sim_report.md`: S1, S2, S4–S8 PASS, S3 INFO, PM 40.6° / 30.1°, GM 6.20 / 3.43, round trip 437.0 / 198.4 ms |
| ground assist decides CUT / RELEASE at a reply timeout on extrapolated telemetry (§8.8) | `sim/ground_station_logic.py` docstring: practice mode, 100 seeds per radio at 5 % loss, I4 cut first in 15 / 19 flights without the rule, 10 / 6 with it (runs not saved); `sim_report.md`: S2 Monte Carlo 19 / 20, S8 link seeds 10 / 10 |

Evidence state (2026-10-16): every artifact derived from params.json except the Fusion 360 run was regenerated on
params.json sha256 0a5c261b… (`FC_PARAMS_TAG` 0x6E): the CadQuery reports and checks (both presets), the sizing
reports, the sim run and `gains.json` (the same gain values and metrics as the 3f31c0a1… run), `params_gen.h`,
`gains.h`, the ground-bridge header, the budget, every firmware build, the host test (20 173 checks, 0 failures),
`crosscheck_controller.py` (every section PASS), the FCU / NAV / COM simbenches (PASS) and the co-simulation (43 / 43
PASS); pytest 491 passed, 3 skipped, 0 failed of 494 (the 3 skips are CadQuery twin tests for a lip-clamp preset).
`gen_firmware_params.py --check`, `gen_gains_header.py --check` and `gen_bridge_config.py --check` report up to date.
Pending: the Fusion run (`fusion360_report.json`, its STL / STEP exports, `crosscheck_fusion_vs_cadquery.txt`, X3) is
from params.json 3f31c0a1… and the print plan from a07c805b…; it will be re-run on the current params.json, and until
then `make_print_files.py --check` reports STALE (exit 2; launch_pad.stl is also not in `cad/exports/print`).
Which outputs record the params.json hash: the CadQuery reports and checks, the sizing reports, `sim/out/metrics.json`
and `sim_report.md`, `gains.json` (`gains.h` records the sha256 of `gains.json` and sets `GAINS_PARAMS_STALE` from
it), `params_gen.h`, the ground-bridge header, `fusion360_report.json` and the print plan. The budget outputs and the
simbench, co-simulation and crosscheck reports record none: they belong to a params.json by build order (run after the
headers were generated from it) and by `FC_PARAMS_TAG` (the co-simulation's COM-reset runs log "params tag 0x6E match").

**v0.9 (2026-10-27, CAD; user decisions 1 / 3 / 4 of 2026-10-27, mass calibration)** — BUDGET preset only, KAIST keeps
the pushrod linkage, 45° fins and the solid mass model (its report entries unchanged):

| change | evidence |
|---|---|
| direct-drive vanes: MG90S in fin-root fairings on the hinge axis, printed D-bore coupler, fins 45° → 0° (on the vane axes), pods / arms / pushrods / clevises removed, launch-pad legs 45° → 0° (§3.9) | C5 (direct) PASS: servo-module min gap 0.300 mm = float, ear slot, slide-out, cover seat, screws + screwdriver, connector; C28 PASS: axis offset 0.0000 mm, float 0.3 ≥ tolerance 0.2, coupler gap 0.300 mm, hinge moment 0.119 kgf·cm vs 1.8 (×15.2); C4 PASS (shafts pass the bore, vanes at 0 / ±25°); C1 PASS (bed extent 0.794 with the 45° plate turn) |
| hinge bores cut after the fins in the Fusion script | X3 replay before the fix: propulsion replay − twin 315.5 mm³ = 4 × 78.9 mm³ of fin plate in the bores (u 46.4-52.7, Z 34.3-39.7); after: 0.0000 / 0.0000 mm³, slices PASS |
| servo leads through tunnels in printed pipes inside the fin roots (§3.8), replacing the pod route | C23 (direct) PASS; pipe walls `sl_pipe_walls` min 4.9 mm; lump 26.8 g |
| motor leads with 5.5 mm bullets: slot 14 → 22 mm, open 22 × 11 intake groove + printed cover, 14 mm gate, 22 × 11 bulkhead hole, joints in the avionics bay (§5.1) | C20 (open) PASS, C29 PASS (bullet proxies ∩ material 0.0000 mm³ straight up through the intake and up the groove with the cover on; joint zone clear); window support check PASS (the 22 mm gate FAILED it: the lip-pocket wall's top edge hung under the gate beside the strut) |
| `edf.wire_exit` keys renamed `_ASSUMED` + `measure_note` | the exit angle / Z range are read from the vendor drawing, not measured - measure before printing the propulsion module |
| printed masses from the slicer (`printer.mass_model` 'slicer_part_g', §6, §12) | all 13 flight part types sliced with their class (table `slicer_part_masses.json`, every entry volume-matched); printed 944.4 g (solid model of the same geometry 1081.3 g); vehicle 2591.6 g (same geometry solid 2728.5 g; v0.8.1 solid 2663.8 g) |
| twin test pins re-pinned (hardware changed, not a regression): `V09_SOLID_SHA256` solid entries | propulsion 437.71 → 443.83 g solid, intake 132.98 → 135.90 g, vane 4.27 → 4.30 g, launch pad rotated (356.80 g), new coupler 0.88 g / cover 3.38 g / wire cover 2.19 g solid, `vane_horn` ×4 removed; the coupler pin re-made once more when its D-bore was taken through the flange (707.5 → 670.7 mm³) |
| C12 / C13 | C12 WARN T/W 1.192 (30.3 N / 2592 g); C13 WARN 0.61 cal (v0.8: 0.56); C10 WARN 0.69 mm tab hole (unchanged) |

T/W over the EDF thrust (decision 4; CAD mass 2591.6 g, sizing flight mass 2601.6 g; VTOL = flight mass − CanSat,
chute, payload module, piston, spring, release servo, latch = 1931.6 g, ESTIMATE - that configuration is not designed):

| thrust kgf | rocket T/W (flight) | hover throttle | VTOL T/W | hover throttle | mass for T/W 1.3 |
|---|---|---|---|---|---|
| 2.00 | 0.769 | > 1 (cannot hover) | 1.035 | 0.983 | 1538 g |
| 2.30 | 0.884 | > 1 (cannot hover) | 1.191 | 0.916 | 1769 g |
| 2.60 | 0.999 | 1.000 | 1.346 | 0.862 | 2000 g |
| 3.09 | 1.188 | 0.918 | 1.600 | 0.791 | 2377 g |

Hover throttle = (W / T)^(1 / `edf.throttle_exponent` 2). 3.09 kgf is the PROXY Powerfun table value; the Hobbyocean
unit publishes no thrust - bench-test it.

Evidence state (2026-10-27, params.json sha256 35e81886…): CadQuery reports, checks (BUDGET 0 FAIL / 3 WARN, KAIST 0
FAIL / 2 WARN), STL / STEP, the provisional print set and slicer table, sizing reports (both presets), the budget, the
X3 replay of the Fusion script for the changed parts (intake: the four gable-window volumes are not modelled by the replay,
replay ∖ twin is exactly them and twin ∖ replay 0 mm³; vane: the mock run records no LE / TE fillet, 50.5 mm³). pytest:
1083 passed, 3 failed, 3 errors, 5 skipped, 2 xfailed; the 3 errors (the archived v0.7.1 report checked against the
v0.9 part list) were fixed and pass on re-run; the failures are the two params-derived headers (`params_gen.h`,
`bridge_config_gen.h`: STALE) and `test_build_flags_select_the_v08_build_and_the_v071_fallback`, which expects the v0.8
FCU position loop although the default chain is the v0.8.1 GNS board (flight_computer.glink, not changed by v0.9).
**Stale until regenerated**: `params_gen.h` and `bridge_config_gen.h` (the firmware round regenerates them),
`sim/out/vtol_gains.json` (tuned on another params.json), the Fusion GUI run and the official `cad/exports/print` set,
the sim campaign outputs.

**v0.9 CAD-2 (2026-10-27; user decisions 2 / 4 / 5, architect mass target VTOL battery <= 1650 g, tether <= 1400 g)** —
the VTOL configuration (§13). The rocket configuration is unchanged (Layout scalars identical, the 14 rocket STL sha256
identical, 2591.6 g).

| change | evidence |
|---|---|
| `mission.config` (rocket / vtol, default rocket), `configs.vtol`, `power` (battery / tether / hybrid), `battery_options_ASSUMED` in params; `load_params(path, preset, config, power)`, `config_suffix` | `test_vtol_config.py` (rocket dict and Layout identical with / without the switch, aliases, refusals: rocket + tether, KAIST + vtol) |
| VTOL stack: top cap, carrier PCB at the FCU spot, 150 mm windowed avionics module, 112 mm tray with holes, misc above the ESC, no payload chain / nose / boards / GPS pod / launch pad | C30 / C31 / C25 / C7 / C14 PASS; VTOL checks 0 FAIL / 1 WARN (C10, pre-existing) for battery, tether and hybrid |
| propulsion: duct 1.6 mm, fairing walls 1.2 mm, fin lightening holes, EDF section 92 mm; intake bulkhead 1.2 mm + vent holes at 90 / 270 | C3, C4, C5, C8, C18-C21, C23, C28, C29, C32 PASS; X3 replay of the Fusion script 0.0000 mm³ both ways for propulsion, avionics, top cap, covers, wire cover and the placeholders (tray: through_cut not modelled by the replay) |
| slicer masses (VTOL table `provisional_v09_cadquery_vtol`) | propulsion 293.69, intake 122.00, avionics 58.68, tray 14.58, top cap 35.60 g; 9 types, 16 h 58 min, filament 640 g, parts 558 g; re-slice reproduces every value |
| masses | VTOL battery 1636.2 g, tether 1388.3 g, hybrid 1509.3 g; per-step savings `docs/vtol_savings.json` (geometry ~194 g measured, carrier + misc 91.6 g ASSUMED, payload chain 670 g) |
| sizing | `sizing_report_vtol*.json`, `tw_table_v09.md`: T/W 1.3 at 2.13 / 1.96 / 1.80 kgf (battery / hybrid / tether), rocket 3.38 kgf; pitch angular acceleration at hover 15.6 vs 10.8 rad/s² (rocket); tip-over 26.3° / 19.2° (rocket 17.6° / 12.7°) |
| hand checks `docs/vtol_strength_checks.json` (ASSUMED load factors) | duct factor >= 76, fin hole >= 3.75, cap ribs 3.1 at 100 N / 1.03 at 300 N (bungee mandatory), windows net 179 mm², tray 35, bulkhead 1.7 MPa |
| tools: `build_rocket.py`, `check_geometry.py`, `make_v09_print_files.py` (also refuses a report without STL files - a `--no-export` report used to slice nothing and empty the table), `replay_x3.py`, `fusion_mock_run.py`, `fusion_api_lint.py` (+ the VTOL layouts), Fusion script 0.9.1 (`config.txt`; first-instance STL export of couplers / covers), `budget_bom_vtol.py`, `vtol_sizing.py`, `vtol_strength.py`, `vtol_savings.py` | pytest: 1086 passed, 3 failed (the two stale headers + the pre-existing v0.8 build-flags test), 5 skipped, 2 xfailed; new tests 17 passed; twin + print-file tests 144 passed, 3 skipped; Fusion mock rocket / VTOL 0 failed; API lint 0 findings |

**Stale until regenerated**: `params_gen.h` / `bridge_config_gen.h` (params changed; the firmware round), the sim (it
does not read the VTOL configuration yet), the Fusion GUI run.

**v0.9 fix round (2026-10-28; independent geometry / mass / render reviews of CAD-1 + CAD-2)** — §13.1.  Hardware changed
in both configurations, so the BUDGET pins changed (re-pinned with reasons in `test_cadquery_twin.V09_SOLID_SHA256`);
KAIST is unchanged (4396.8 g, its v0.4 / v0.6.2 pins hold).

| change | evidence |
|---|---|
| lip-pocket slit 6 mm, 3 strut tie anchors, bulkhead hole 20 × 11 at r 42-53, groove / slot sliver cuts | C29 (iv) slit probe 0 mm³, wall ring 98.3 % (open; the reviewer's STL ring test: 0.983 at 6 levels, was 1.0); (v) leads 3.5 mm outside the throat, free span 31.4 ≤ 40; C9 / C20 / C25 PASS; sliced intake: window-315 support probe PASS; STL thickness survey: no slot / groove slivers left |
| coupler socket + M2 set screw on a +v D-flat, cut double-arm horn, shaft end gap 1.5, brass bushings, EVA pad | C28: inward stop 0.3 (found), outward 2.00 mm, horn untouched at the stop, arms engaged 1.8 / 1.8, foam 0.27-0.50, tilt 0.82°; C5: removal sweep clear with the vane pushed in 7.5 mm, key path clear; all-pairs interference 0 mm³ (55 / 42 / 49 / 52 parts) |
| VTOL: solid fins (span 76.8), tray plane 14.6 / 107.5 wide / holes to 33, side windows moved, gland 180° / GPS 0°, load washer + jam nut, tether boxes | C32 (fin plate solid, tip-over with the CG offset), C33 (trim 1.8 / 7.0 WARN / 4.9 %), C30 (stack 4.2 mm over the pack), C14, C25; hand checks: solid fin ×8.5 (hole ~×1.5 reviewer), washer ×4 at 300 N |
| Fusion script 0.9.2 mirrors every change | replay X3 0.0000 mm³ for propulsion, couplers, horns, bushings, covers, top cap, avionics module, tether boxes / eye bolt / washer / nut; intake outside the four gable-window volumes 0.0000 (slit, anchors, tunnels, hole inside); mock runs 0 failed (4 configurations), lint 0 |
| masses (slicer) | rocket 2591.6 → 2592.6 g; VTOL battery 1636.2 → 1647.2, tether 1388.3 → 1399.3, hybrid 1509.3 → 1520.3 (targets 1650 / 1400 met by 2.8 / 0.7 g); superseded 2026-10-29 (lead-exit round): 1655.9 / 1408.0 / 1529.0, rocket 2601.2 - targets missed by 5.9 / 8.0 g; photo round 2026-10-29: 1655.1 / 1407.2 / 1528.2, rocket 2600.3 - targets missed by 5.1 / 7.2 g |
| sizing: voltage condition, supply-limited T/W, hybrid crossover; precise volumes for the STL check and the savings | `tw_table_v09.md`; duct / fairing savings 23.1 / 11.1 → 21.4 / 8.3 g |

pytest (whole project, params sha256 7a473189…): 1102 passed, 5 failed, 5 skipped, 2 xfailed; two of the failures were
test expectations of the old geometry (the wall radius beside the slit; the sim estimate's linkage without the
bushings), fixed and re-run (2 passed); the remaining three are the params-derived headers `params_gen.h` /
`bridge_config_gen.h` (STALE, the firmware round regenerates them) and the pre-existing
`test_build_flags_select_the_v08_build_and_the_v071_fallback` (v0.8.1 GNS chain, not touched).

**v0.9 sim / control / tether round (2026-10-28; Python reference + sim only; §13.2)** — the VTOL flights fly the VTOL
hardware (user decision). params.json sha256 7a473189… → b2f89251… (plant) → fd9fdb47… (control) → f3edab43… (tether) →
ae0b99e4… (fix round). Every number below comes from a run on disk (scratch `v09sim/`, `sim/out/v09_*`,
`sim/out/vtol/campaign_v09_*`); before / after pairs fly the same plant and samples.

| change | evidence |
|---|---|
| plant: VTOL vehicle from the CAD reports, direct-drive play, rotor gyro / spool reaction, thrust envelope + voltage, battery / tether / hybrid power, tether rig, touchdown ground speed ≤ 0.3 m/s + tip margin (not a control change) | old controller on the new plant, EVAL 8300: field 10 pass / 79 abort of 120 (all I3), swirl 0 / 69, demo 0 / 59, tether 0 / 29, hybrid 0 / 40; rotor-off what-if (TUNE) 23 → 16 aborts. Ballistic: S5 PASS → FAIL, servo NOT ADEQUATE (roll 6-DOF GM 3.36 → 1.76), peak yaw after cut-off 99 → 1388 °/s; ballistic pins re-pinned on the v0.9 rocket (plant changed) |
| C1-C7: rate gains × inertia ratio, candidate D, per-axis anti-windup, rotor feed-forwards, two-slope take-off + thrust search (k_min 0.55), vz_kp / vz_ki / alt_kp 0.05 / 0.008 / 1.0, touchdown `keep_vel` | loop model: pitch PM 18.3 → 40.4°, vertical −36.7…−42.8 → 24.6…32.8°. EVAL 8300 (once): field 10/79 → 42/0, swirl 0/69 → 18/1, thrust_low 5/33 → 4/0, tether 0/29 → 1/9, demo 0/59 → 0/57, hybrid 0/40 → 0/40. Reviewer FRESH 9200: field 6/73 → 20/1 (the EVAL 42 passes were a lucky sample) |
| T1-T6: power rules W_ARM / W_SAG / W_LOST / W_BUF / W_D2, tethered limits, ground-station PWR line + Korean alarms, procedure `docs/유선_시험비행_절차.md` | battery and ballistic logs bit-identical (5 runs); 0 power-rule triggers in healthy tether / hybrid flights (TUNE 26, EVAL 80, FRESH 74); W_LOST LAND 0.04 s after the loss; `sim/test_power_rules.py` 36 + 1 tests |
| independent reviews (physics, control-safety) | high: vane effectiveness never spread (×0.76 → field aborts 0 → 6 on TUNE), catch-line handler law, power rules stopped the motor in the air before the take-off check (6/6 falls), W_LOST landings caught by the locked belay (6/10), yaw robustness to the swirl SIGN (−1.5: 12/24 aborts, +1.5: 0/24); medium: push envelope ~3 N, ground effect, cable loop at the intake, ADC gain error, regeneration, EVAL vs FRESH, overshoot |
| fix round plant P1-P9 (vane-force spread, swirl sign, ground effect, SRL / autolock belay, slack ≤ 0.25 m, ADC gain, regeneration / OVP, new criteria climb_overshoot 0.5 / 0.3 m, not_hanging, cable_clear_of_intake, touchdown selection) - tightening, applied to before and after alike | knobs off: 5 battery / ballistic runs bit-identical |
| fix round control F1-F10: SAFE only on the pad, att_err_mode 2, gyro FF off, mixer yaw-trim reserve, k_T learning from the lift-off, reference hold, reference approach, touchdown release, LAND rebase, reaction FF as a gain | FRESH2 9700 (held out): field 5/26 → 22/17, swirl 2/29 → 9/19, swirl_neg 0/30 → 2/22, tether 1/14 → 6/4, named 29 → 39 / 57; overshoot p50 0.68 → 0.41 m (thrust_stress regressed −0.07 → 1.06 m); power-rule scenarios 20 → 22 / 27; floating LANDED 37/99 → 0/99 (TUNE). Re-pin of `V071_VTOL_DIGESTS` (intended control change) with the reason in the test |
| campaign 1090 (1880 runs, once) + sampler fix (demo / hybrid pushes now in HOLD) | field 500: 89 pass, 63 abort (I3 36, I2 27; 53 negative swirl); demo 5-15 N 282/300 abort, 1-3 N 12/100; tether 14/150 pass, max true height 3.35 m; thrust sweep 0/30 pass ≤ 2.30 kgf, 12/30 at 3.09; swirl sweep −2.0: 25/30 abort |
| renderer contract v6 (VTOL meshes, tether stand / catenary, power HUD rows); v0.8.1 baseline archive `history/v0.8.1` + `run_baseline.py`; 28 comparison videos + Korean change log | renderer tests 136 passed, legacy frames byte-identical; archive: 16/16 campaign rows, 6/6 named, 7/7 pins identical |

Open (owner in brackets): negative residual swirl (control + bench), demo push rule (user), take-off overshoot / landing
bounce / touchdown ground speed (control), V15 gap at T/W ≤ 1 (supervisor), snag → CG-height bridle and landing feet ≥ 45 mm
(CAD), ballistic re-tune for the vane play (if the rocket mission flies), the C port + headers + crosscheck vectors + 4-core
co-simulation + NAV timing closure (firmware round).

**v0.9 firmware round (2026-10-29; §10.6)** — four ATmega128A, no hardware / wiring / mass change; the reference changed
only by RP1 (link protocol). Before state `history/v0.9_pre_fw/` (v0.8.1 firmware code, v0.9 reference); params.json
ae0b99e4… → ae9669a7… (only the `flight_computer.pins` power pins and `flight_computer.nlink.exchange_watchdog_us` /
note); headers params_gen.h 66a12ac1, gains.h 15d1d043 (unchanged), bridge_config_gen.h e6f18bab, `--check` up to date.
Evidence `fw09/impl/integ/ACCEPTANCE.md`, `fw09/impl/fix/runs/final/`, `fw09/final/FRESH3_REPORT.md` (`fw09/` = the
round's scratch folder, copies in `B:\Video\펌웨어\v0.9\구현\`).

| change | evidence |
|---|---|
| P1: fixed-point quaternion, interrupt-paced nlink, push floor 1310 → 890 µs, table CRC, small items I6-I10, bench wire B1 | NAV mandatory work 4099.2 → 2782.2 µs (bench); co-sim NAV tick → vane 4321.5 → 3578.4 µs, IMU → vane 4817.5 → 4076.1 µs, IMU-lead ticks 1389 / 1390 → 0 / 12960, MS5611 in flight 0 → 50 Hz; NAV q max 7.82e-6 vs the reference |
| v0.9 control port (FCU / GNS / COM) + power rules + ADC chain + fcu_wired / fcu_wired_buffer | crosscheck PASS 401.7 s: supervisor 982 451 ticks, FCU stack replays 276 451 ticks, power rules 273 000 ticks, 0 mismatches |
| W1 exchange watchdog 1000 µs | nl_lose_cmp: 1 trip, no abort (without W1: I9 ABORT); cost 0.0 / +0.9 µs |
| G1 per-configuration constants (`FC_VCFG`), tags 0x09 / 0x67 / 0x5D, new nav / gns / com wired envs | l_v 351.0 → 210.0 / 184.7 / 197.3 mm, hover throttle 0.916 → 0.730 / 0.673 / 0.701 (`fw09/impl/D/runs/cfg_macro_compare.txt`); per-configuration plant check PASS |
| RP1 TELEM v4 / v5, LEN_MAX 42, SNAP2 57 / SNAP3 65 B | TUNE 7400: battery 130 / 130 digest-identical, tether / hybrid 0 flips, named 57 / 57; videos 29 / 30 |
| S1 closed-loop 4-core co-simulation, X1 crosscheck per configuration, T2 VTOL text | 24 co-sim runs, every row PASS; host test 8 × 29 699 checks 0 failures |
| review fixes: mixed NAV / COM image set refused on the ground (F1), ADC ISR 79 / 85 → 45 / 56 cycles + adc_chain_test (F2), judge additions SYS-1 / -2 / -5 / -6 | cl_mixed_images SAFE 52 s, 100 ON refused; all 22 rows PASS on the final state |
| FRESH3 10500 (once) | (a) PASS, (b) PASS 39 / 57, (c) FAIL 5 / 8 groups (decision) |

pytest (whole project): step 0 1167 passed / 6 failed; final 1216 passed / 5 failed (the pre-existing
`sim/test_vtol_scenarios.py` ones), 8 skipped, 2 xfailed. Open: §10.6.8.

**2026-11-06 firmware round (§10.7; no control change)** — the vehicle radio is the ESP8266 Wi-Fi bridge (ESP-NOW), VTOL and
rocket (`flight_computer.link.vehicle_radio` = `esp_bridge`; `link.radio` stays the lora_uart link model; LoRa backends kept
as the fallback); COM backend `hal/esp_bridge.c` (USART1 38400 U2X 0, ESP reset on PC6 - open-drain, cold COM boot only -
400 ms boot-log drop, no busy line);
bridge sketch `firmware/esp_bridge` (compiled with arduino-cli + core esp8266:esp8266 3.1.2, all 5 variants, M; not flashed); ground station
`--link-hw`; NEO-7M MON-VER `00070000` -> u-blox 7 (report only); headers regenerated, every image rebuilt (vehicle
constants unchanged), CAD reports re-made for the params hash only. Acceptance: 20 of 22 PASS, row 19 not re-judged, row
21 (ballistic hashes nav c1757ef8 / fcu 6c5b873c / com efc294a6, after the independent-review fixes; full values in
`firmware/atmega128_fc/HEX_HASHES_2026-11-06.txt`) awaiting the user's re-declaration. Stopped (control
procedure): the logic-battery telemetry (no free TELEM field; a longer battery-build reply changes the sim's link timing).
Korean report: `docs/v0.9_펌웨어_결과.md` §11.
