# VTOL avionics carrier - rev 3 plan (JLCPCB turnkey), settled after review

Plan date 2026-11-10. Revision: 3 (plan settled after the review of 2026-11-10 - nothing built, routed or checked by a tool yet).

Generated from `B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier/spec/rev3_plan.json` (sha256 `415d1e20a0903103d6bb4b48253ffc7054cfd3d2371c008706b906fe5c1eda15`) by `C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/final/make_md_final.py`. The JSON is the source; this file adds no fact.

Labels: F = file / vendor page / datasheet (path:line or URL), M = measured here (command given), E = estimate, A = assumption. No evidence = NOT_VERIFIED.

## 1. Status

- Ready to implement: **yes** - reviewers raised 0 blockers; 32 findings (11 major, 21 minor) are settled, 0 refuted; consistency check `PASS` (8 checks, section 20).
- Board items 175: 140 keep, 11 replace, 10 keep-dnp, 1 remove, 13 add. Changed parts (replace + add + remove): 25.
- JLC BOM: 40 lines (34 SMT, 6 THT), 157 parts per board, 39 THT joints per board. Every line has a JLC record of 2026-11-10 (20:56..21:41 KST) that satisfies I7.
- Nothing was built, placed, routed or checked by KiCad in this phase. 'Ready to implement' means the plan is consistent and sourced; it does not mean the board is verified. What stays NOT_VERIFIED is in section 17.
- JLC's acceptance of the board (parts near the edge, THT fitting, order type) can only be seen after the user's upload; the plan forbids paying before JLC's DFM result and quote were read (section 15).

## 2. Interpretation and invariants

- I1: U1..U4 ATmega128A-AU keep their footprints and are NOT in the main JLC BOM / CPL (user solders them); an optional second BOM / CPL pair that adds C6510 is produced.
- I2: sensors = GY-521 (J1), BMP280 TYE-BMP29 (J2), NEO-7M SZH-NT07 (lead to J7). The user fits them; the mating connectors on the carrier are JLC-assembled; the GY-521 is soldered into plated holes with its own pin strip.
- I3: every other populated part is in the JLCPCB assembly library, in stock and assembled by JLC in the same order (no devicemart part on the board, no hand-solder-later list, no LCSC-only part). The Pololu S9V11F5S6CMA is replaced by an on-board converter; THT connectors are soldered by JLC; the ESP is the JLC library part ESP-12F (pin-compatible with the ESP-12E).
- I4: OpenLog stays a plug-in module on the JLC-assembled socket J5 (recommendation); anything that needs a firmware change is a user decision.
- I5: off-board items (battery, switch + lead, GPS lead, shunts, buzzer, modules, MCUs) are on a separate purchase list, not in the JLC BOM.
- I6: only what the request forces plus provable defects is changed.
- I7: orderable = min(stockCount, canPresaleNumber) >= max(20 x qty + lossNumber, leastNumber), isBuyComponent = 1, no noBuyReason, assemblyMode smtWeld or manualWeld; Basic preferred over Extended when electrically equal; stock >= 1000 preferred (below = THIN, reported).

- (a) every ATmega128A pin of U1..U4 keeps its rev 2c net (checked: plan pin map == rev2c_mcu_pinmap.json).
- (b) board outline 76 x 70 mm, 1.6 mm, no mounting holes (foam tape + one cable tie, tie band x -3..+3), all connector positions and orientations, component height limit 13.4 mm above the board top, ESP antenna keep-out: unchanged.
- (c) every JLC-assembled part is on the TOP side; nothing on the bottom.
- (d) 4 layers: F.Cu / In1 GND / In2 +5V / B.Cu.
- (e) symbols and footprints only from the on-disk KiCad 10 libraries; pin numbers / pads from the library file and the datasheet; a mismatch is an error. Consequence in this plan: U8 uses a generic 6-pin symbol and U9 a pin-identical substitute symbol (both flagged).
- (f) no existing check, tolerance or design rule is loosened (track 0.2, clearance 0.13, via 0.3 / 0.6, edge 0.3, hole-hole 0.5 mm); the XTAL1 length deviation (5.96 mm in rev 2c) must not get worse.
- (g) derived artifacts stay deterministic (same spec -> byte-identical KiCad / BOM / CPL / gerbers); renders and JLC stock numbers are evidence, not deterministic artifacts.

## 3. Changes against rev 2c

140 parts are unchanged (133 fitted by JLC, 5 fitted by user, 2 fitted by none). The rows below are every other board item. The tag says why the item changes.

| Ref | Action | rev 2c | rev 3 (value / MPN / JLC code) | Fitted by | Tag | Reason |
|---|---|---|---|---|---|---|
| C45 | replace | 22u 10V tantalum / C7183 | 22u 10V tantalum (TAJA226K010RNJ, +-10 %) / TAJA226K010RNJ / C11366 | JLC | optional sourcing (the rev 2c part passes I7 today with less stock; revertible in D3 / D12) | same Kyocera AVX TAJ series / case A / 10 V / ESR 3 Ohm; stock 403792 instead of 2264 (I7 prefers >= 1000 with depth) |
| J2 | replace | 1x6 2.54 mm right-angle socket (module lies flat) / C55629861 | 1x6 2.54 mm right-angle socket (module lies flat) / HC-PM254-8.5H-1x6PW / C22436147 | JLC | optional sourcing (the rev 2c part passes I7 today with less stock; revertible in D3 / D12) | 1x6 right-angle socket whose manufacturer drawing was read against the KiCad footprint (review E-10); thin stock -> hard order-day gate with alternates C55629861 (rev 2c part, no datasheet, 3.2 mm tails) then C54876735 (user decisions D3 / D12). |
| J3 | keep-dnp | 2x3 2.54 mm vertical: ESP-12E programming header (3.3 V logic ONLY) / C65114 | 2x3 2.54 mm vertical: ESP-12E programming header (3.3 V logic ONLY) | none | unchanged (rev 2c footprint without a part) | programming / reset-hold header inside the cable-tie band: stays bare plated holes exactly as rev 2c (no fitted part is allowed in the band). Access for programming is user work (soldered headers or a pogo adapter) - review S1 / OM-3, user decision D6; option B = JLC-fitted headers outside the band. |
| J5 | replace | 1x6 2.54 mm right-angle socket: OpenLog lies FLAT beside it (outline reserved) / C55629861 | 1x6 2.54 mm right-angle socket: OpenLog lies FLAT beside it (outline reserved) / HC-PM254-8.5H-1x6PW / C22436147 | JLC | optional sourcing (the rev 2c part passes I7 today with less stock; revertible in D3 / D12) | 1x6 right-angle socket whose manufacturer drawing was read against the KiCad footprint (review E-10); thin stock -> hard order-day gate with alternates C55629861 (rev 2c part, no datasheet, 3.2 mm tails) then C54876735 (user decisions D3 / D12). |
| J6 | remove | 1x5 2.54 mm: Pololu S9V11F5S6CMA 5 V step-up/step-down module (VOUT, GND, VIN, EN, PG) / no=15855364 | - | - | forced (I3: the devicemart Pololu module leaves the board) | I3: the Pololu S9V11F5S6CMA (devicemart) and its 1x5 header are removed; the 5 V rail is made on the board by U8 / L5 and the cut-off by U9. Nets +5V, GND and VBAT_LOGIC_SW keep all other members; the no-connect entries J6.4 (EN) and J6.5 (PG) disappear. The refdes J6 stays unused. |
| J8 | replace | 1x3 2.54 mm right-angle header (S, +, -) / C55629817 | 1x3 2.54 mm right-angle header (S, +, -) / HX PZ2.54-1x3P WZ / C32713262 | JLC | optional sourcing (the rev 2c part passes I7 today with less stock; revertible in D3 / D12) | rev 2c listed this THT header for hand soldering (C55629817); rev 3 uses a JLC-stocked right-angle 1x3 header that JLC solders (THT, manualWeld). Footprint, position and orientation unchanged. |
| J9 | replace | 1x3 2.54 mm right-angle header (S, +, -) / C55629817 | 1x3 2.54 mm right-angle header (S, +, -) / HX PZ2.54-1x3P WZ / C32713262 | JLC | optional sourcing (the rev 2c part passes I7 today with less stock; revertible in D3 / D12) | rev 2c listed this THT header for hand soldering (C55629817); rev 3 uses a JLC-stocked right-angle 1x3 header that JLC solders (THT, manualWeld). Footprint, position and orientation unchanged. |
| J10 | replace | 1x3 2.54 mm right-angle header (S, +, -) / C55629817 | 1x3 2.54 mm right-angle header (S, +, -) / HX PZ2.54-1x3P WZ / C32713262 | JLC | optional sourcing (the rev 2c part passes I7 today with less stock; revertible in D3 / D12) | rev 2c listed this THT header for hand soldering (C55629817); rev 3 uses a JLC-stocked right-angle 1x3 header that JLC solders (THT, manualWeld). Footprint, position and orientation unchanged. |
| J11 | replace | 1x3 2.54 mm right-angle header (S, +, -) / C55629817 | 1x3 2.54 mm right-angle header (S, +, -) / HX PZ2.54-1x3P WZ / C32713262 | JLC | optional sourcing (the rev 2c part passes I7 today with less stock; revertible in D3 / D12) | rev 2c listed this THT header for hand soldering (C55629817); rev 3 uses a JLC-stocked right-angle 1x3 header that JLC solders (THT, manualWeld). Footprint, position and orientation unchanged. |
| J12 | replace | 1x3 2.54 mm right-angle header (S, +, -) / C55629817 | 1x3 2.54 mm right-angle header (S, +, -) / HX PZ2.54-1x3P WZ / C32713262 | JLC | optional sourcing (the rev 2c part passes I7 today with less stock; revertible in D3 / D12) | rev 2c listed this THT header for hand soldering (C55629817); rev 3 uses a JLC-stocked right-angle 1x3 header that JLC solders (THT, manualWeld). Footprint, position and orientation unchanged. |
| J16 | replace | 1x2 right-angle header / C55629816 | 1x2 right-angle header / HX PZ2.54-1x2P WZ / C32713261 | JLC | optional sourcing (the rev 2c part passes I7 today with less stock; revertible in D3 / D12) | as J8: JLC-assembled right-angle 1x2 header instead of the hand-solder part C55629816 |
| J18 | replace | 1x4 right-angle header / C55629818 | 1x4 right-angle header / HX PZ2.54-1x4P WZ / C32713263 | JLC | optional sourcing (the rev 2c part passes I7 today with less stock; revertible in D3 / D12) | as J8: JLC-assembled right-angle 1x4 header instead of the hand-solder part C55629818 |
| J19 | keep-dnp | AVR ISP 2x3 (USBasp) / C65114 | AVR ISP 2x3 (USBasp) | none | unchanged (rev 2c footprint without a part) | programming / reset-hold header inside the cable-tie band: stays bare plated holes exactly as rev 2c (no fitted part is allowed in the band). Access for programming is user work (soldered headers or a pogo adapter) - review S1 / OM-3, user decision D6; option B = JLC-fitted headers outside the band. |
| J20 | keep-dnp | AVR ISP 2x3 (USBasp) / C65114 | AVR ISP 2x3 (USBasp) | none | unchanged (rev 2c footprint without a part) | programming / reset-hold header inside the cable-tie band: stays bare plated holes exactly as rev 2c (no fitted part is allowed in the band). Access for programming is user work (soldered headers or a pogo adapter) - review S1 / OM-3, user decision D6; option B = JLC-fitted headers outside the band. |
| J21 | keep-dnp | AVR ISP 2x3 (USBasp) / C65114 | AVR ISP 2x3 (USBasp) | none | unchanged (rev 2c footprint without a part) | programming / reset-hold header inside the cable-tie band: stays bare plated holes exactly as rev 2c (no fitted part is allowed in the band). Access for programming is user work (soldered headers or a pogo adapter) - review S1 / OM-3, user decision D6; option B = JLC-fitted headers outside the band. |
| J22 | keep-dnp | AVR ISP 2x3 (USBasp) / C65114 | AVR ISP 2x3 (USBasp) | none | unchanged (rev 2c footprint without a part) | programming / reset-hold header inside the cable-tie band: stays bare plated holes exactly as rev 2c (no fitted part is allowed in the band). Access for programming is user work (soldered headers or a pogo adapter) - review S1 / OM-3, user decision D6; option B = JLC-fitted headers outside the band. |
| J24 | keep-dnp | 2-pin header + shunt / C55629809 | 2-pin header + shunt | none | unchanged (rev 2c footprint without a part) | programming / reset-hold header inside the cable-tie band: stays bare plated holes exactly as rev 2c (no fitted part is allowed in the band). Access for programming is user work (soldered headers or a pogo adapter) - review S1 / OM-3, user decision D6; option B = JLC-fitted headers outside the band. |
| J25 | keep-dnp | 2-pin header + shunt / C55629809 | 2-pin header + shunt | none | unchanged (rev 2c footprint without a part) | programming / reset-hold header inside the cable-tie band: stays bare plated holes exactly as rev 2c (no fitted part is allowed in the band). Access for programming is user work (soldered headers or a pogo adapter) - review S1 / OM-3, user decision D6; option B = JLC-fitted headers outside the band. |
| J26 | keep-dnp | 2-pin header + shunt / C55629809 | 2-pin header + shunt | none | unchanged (rev 2c footprint without a part) | programming / reset-hold header inside the cable-tie band: stays bare plated holes exactly as rev 2c (no fitted part is allowed in the band). Access for programming is user work (soldered headers or a pogo adapter) - review S1 / OM-3, user decision D6; option B = JLC-fitted headers outside the band. |
| Q3 | keep-dnp | BSS138 / C40912 | BSS138 | none | unchanged (rev 2c footprint without a part) | optional (DNP) in rev 2c and unchanged: footprint only, in neither BOM nor CPL |
| R47 | keep-dnp | 10k / C25804 | 10k | none | unchanged (rev 2c footprint without a part) | optional (DNP) in rev 2c and unchanged: footprint only, in neither BOM nor CPL |
| U7 | replace | ESP-12E (ESP8266EX + 4 MB SPI flash, PCB antenna) / C89297 | ESP-12F (ESP8266MOD, 4 MB flash, PCB antenna) / ESP-12F(ESP8266MOD) / C82891 | JLC | sourcing (I7: ESP-12E C89297 is Standard-only and thin; I3 allows the pin-compatible ESP-12F) | C89297 ESP-12E is 'Standard PCBA only' (componentProductType 2) and thin (1575); the pin-compatible Ai-Thinker ESP-12F C82891 is Economic+Standard with 13x the stock. Same footprint RF_Module:ESP-12E, same 22 pin numbers (library symbol ESP-12F extends ESP-12E). I3 allows the 12F. |
| U8 | add | - | TPS61023DRLR / TPS61023DRLR / C919459 | JLC | forced (I3: on-board 5 V converter replaces the module) | I3: the devicemart boost module is replaced by an on-board converter of JLC parts. GENERIC SYMBOL: no TPS61023 symbol exists in the KiCad 10 libraries (1695 symbols scanned, 0 pin-identical); the 6-pin generic symbol keeps invariant (e) but makes ERC blind for U8 - the pad->net table check in checks_to_add replaces it. |
| U9 | add | - | TLV3012BIDBVR / TLV3012BIDBVR / C20345924 | JLC | keeps a rev 2c function (battery cut-off was inside the Pololu module); can be dropped in D2 | Keeps the rev 2c under-voltage cut-off (it was inside the Pololu module). Review E-1 / E-2 / E-8: the TLV3011A limits came from a superseded datasheet, its open-drain output made EN a 701 kOhm node and it has no power-on reset. TLV3012B (SBOS300C limits) is push-pull, drives EN directly and holds its output low for 1.9 ms after power-up. SUBSTITUTE SYMBOL: no TLV3012 symbol on disk; Comparator:MCP65R41 has the same pin numbers and functions (1 push-pull out, 2 V-, 3 +, 4 -, 5 Vref, 6 V+); the Value field and the BOM carry TLV3012BIDBVR / C20345924. |
| L5 | add | - | 1.0 uH FXL0420-1R0-MA1 / FXL0420-1R0-MA1 / C53426685 | JLC | forced (I3: on-board 5 V converter replaces the module) | TPS61023 needs 0.37..2.9 uH effective; 1.0 uH is the datasheet's design value. Review E-3: the -MA1 variant (Isat 6.5 A / 9.0 A against 5.6 A / 7.0 A, same DCR 27 mOhm, same body and land pattern) keeps the typical current-limit peak of 5.1 A below saturation. C167203 (-M) stays as drop-in alternate. |
| C57 | add | - | 22u / CL31A226KAHNNNE / C12891 | JLC | forced (I3: on-board 5 V converter replaces the module) | second output capacitor for the 1 MHz boost (output ripple and effective capacitance under DC bias); same line as C41-C43 |
| C58 | add | - | 1n 50V X7R / CL10B102KB8NNNC / C1588 | JLC | forced (I3: on-board 5 V converter replaces the module) | TI loop-stability recommendation: zero at about 1 kHz with R63 (1061 Hz here) |
| C59 | add | - | 100n / CC0603KRX7R9BB104 / C14663 | JLC | keeps a rev 2c function (battery cut-off was inside the Pololu module); can be dropped in D2 | comparator supply bypass (datasheet application information) |
| C60 | add | - | 100n / CC0603KRX7R9BB104 / C14663 | JLC | keeps a rev 2c function (battery cut-off was inside the Pololu module); can be dropped in D2 | keeps load steps from tripping the cut-off |
| R63 | add | - | 150k 1% / 0603WAF1503T5E / C22807 | JLC | forced (I3: on-board 5 V converter replaces the module) | Vout = 0.595 V x (1 + 150k / 20k) = 5.06 V nominal |
| R64 | add | - | 20k 1% / 0603WAF2002T5E / C4184 | JLC | forced (I3: on-board 5 V converter replaces the module) | see R63 |
| R65 | add | - | 470k 1% / 0603WAF4703T5E / C23178 | JLC | keeps a rev 2c function (battery cut-off was inside the Pololu module); can be dropped in D2 | UVLO divider upper resistor (VBAT_LOGIC_SW -> UVLO_SENSE); value from the worst-case search with TLV3012B limits (review E-1): on 3.4438..3.7334 V, off 3.005..3.2532 V. |
| R66 | add | - | 270k 1% / 0603WAF2703T5E / C22965 | JLC | keeps a rev 2c function (battery cut-off was inside the Pololu module); can be dropped in D2 | UVLO divider lower resistor (UVLO_SENSE -> GND); value from the worst-case search with TLV3012B limits (review E-1): on 3.4438..3.7334 V, off 3.005..3.2532 V. |
| R67 | add | - | 3.3M 1% / 0603WAF3304T5E / C26096 | JLC | keeps a rev 2c function (battery cut-off was inside the Pololu module); can be dropped in D2 | UVLO hysteresis resistor (comparator output -> UVLO_SENSE); value from the worst-case search with TLV3012B limits (review E-1): on 3.4438..3.7334 V, off 3.005..3.2532 V. |
| R68 | add | - | 1M 1% / 0603WAF1004T5E / C22935 | JLC | keeps a rev 2c function (battery cut-off was inside the Pololu module); can be dropped in D2 | review E-2 / E-8: with the push-pull comparator the 1M resistor is no longer a pull-up; as a pull-down it defines EN when the comparator output is not yet driven. 0 uA with EN low, about 3.7 uA at 3.7 V in the on state. |

Conditional items (not in the default design):

| Item | Condition | Default |
|---|---|---|
| H1 / H2 tooling holes (Mechanical:MountingHole, MountingHole:ToolingHole_1.152mm) | only for an Economic order with 'Added by Customer' and only if the legal-position search after placement and routing proves two opposite-corner positions (check; D10). M C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/verify/order_mech_work/measure_board.json section D: free grid points footprint-free / also track-free: -x,+y 462 / 128; +x,+y 55 / 5; -x,-y 0 / 0; +x,-y 201 / 0. | not in the design |
| JLC-fitted programming headers outside the tie band (C32713283 x 5, C32713268 or C41417359 x 3) | D6 option B: only with the user's yes and a found / not-found placement report | not in the design (bare holes as rev 2c) |
| U1..U4 = C6510 in the BOM / CPL | D4: user chooses the with_mcu file pair | not in the main BOM / CPL |

## 4. Nets

| New net | Members | Kind |
|---|---|---|
| BOOST_SW | L5.2, U8.5 | switch node (1 MHz, keep the copper minimal) |
| BOOST_FB | C58.2, R63.2, R64.1, U8.1 | feedback, 0.595 V |
| BOOST_EN | R67.2, R68.1, U8.2, U9.1 | enable = UVLO comparator output (push-pull, low impedance; 1M pull-down R68) |
| UVLO_SENSE | C60.1, R65.2, R66.1, R67.1, U9.3 | battery divider tap (163 kOhm Thevenin, 100 nF filter) |
| UVLO_REF | U9.4, U9.5 | 1.242 V reference tied to IN- |

- No rev 2c net disappears: +5V, GND and VBAT_LOGIC_SW lose only their J6 members.
- +5V is now driven by U8 pin 6 through a passive (generic-symbol) pin: keep the rev 2c PWR_FLAG on +5V.
- VBAT_LOGIC_SW now feeds a power-input pin (U9 pin 6, symbol MCP65R41): add a PWR_FLAG on VBAT_LOGIC_SW or ERC reports 'power input not driven'.
- Removed nets: none. Removed no-connect flags: J6.4, J6.5.

## 5. Power block (all numbers E from datasheet limits unless marked)

- winner: A (TPS61023DRLR synchronous boost + TLV3012B push-pull under-voltage lock-out) with grafts
- numbers_source: C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/design/plan_work/verify_power.json
- command: cd /b/Claude && .venv/Scripts/python.exe -X utf8 C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/design/plan_work/verify_power.py
- vout_V:
  - nominal_V: 5.0575
  - min_V: 4.8439
  - max_V_pwm: 5.2774
  - typ_V_pfm: 5.1085
  - max_V_pfm_E: 5.3302
  - fb_leak_error_mV: 3.0
  - divider_uA: 29.8
  - ffz_Hz_with_1nF: 1061.0
- iout_capability_at_IC_input_3p0V_ILIM_min:
  - D: 0.4661
  - dIL_A: 1.398
  - Iout_CL_A: 1.815
  - Vin_at_IC_V: 3.0
  - ilim: ILIM min 2.7 A
  - L: L nom
  - margin_vs_1p5x_peak: 1.55
  - margin_vs_peak_783mA: 2.32
- valley_limit_needed_for_1p5x_peak_at_3p0V_A: 1.5
- source_path:
  - path_resistance_ohm:
    - VBAT_LOGIC_SW trace rev 2c (E): 0.08
    - Q10 AO3401A Rds(on) at Vgs -2.5..-3 V (A, datasheet max 85 mOhm at -2.5 V): 0.085
    - switch + 3 connector pairs (A): 0.05
    - cell internal resistance (A): 0.06
  - sum_ohm: 0.275
  - cases:
    - peak at 3.0 V pack:
      - Vbat_V: 3.0
      - Iout_A: 0.783
      - Iin_A: 1.746
      - V_at_IC_V: 2.52
      - IL_peak_A: 2.441
      - IL_valley_A: 1.051
    - typ at 3.0 V pack:
      - Vbat_V: 3.0
      - Iout_A: 0.273
      - Iin_A: 0.538
      - V_at_IC_V: 2.852
      - IL_peak_A: 1.24
      - IL_valley_A: -0.164
    - peak at 3.3 V pack:
      - Vbat_V: 3.3
      - Iout_A: 0.783
      - Iin_A: 1.528
      - V_at_IC_V: 2.88
      - IL_peak_A: 2.23
      - IL_valley_A: 0.826
    - peak at 3.7 V pack:
      - Vbat_V: 3.7
      - Iout_A: 0.783
      - Iin_A: 1.318
      - V_at_IC_V: 3.337
      - IL_peak_A: 1.996
      - IL_valley_A: 0.641
- uvlo:
  - comparator: TLV3012BIDBVR (C20345924), push-pull, reference 1.223 / 1.242 / 1.260 V, drift 100 ppm/C x 40 K, offset +-9 mV, internal hysteresis 2 / 6 / 8 mV (F C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/verify/electrical_work/tlv3011.txt, SBOS300C)
  - resistors:
    - R65: 470k
    - R66: 270k
    - R67: 3.3M
    - tolerance: 1 % each (corner analysis, 256 corners)
  - V_on_nominal: 3.5895
  - V_off_nominal: 3.1269
  - V_on_worst: [3.4438, 3.7334]
  - V_off_worst: [3.005, 3.2532]
  - hysteresis_worst: [0.4256, 0.4941]
  - limits_A:
    - v_off_min: 3.0
    - v_on_max: 3.75
  - limits_met: True
  - rebound_V_at_path_resistance:
    - ohm_E: 0.275
    - typ_load: 0.1388
    - peak_load: 0.3981
  - hysteresis_min_minus_rebound_peak_V: 0.0275
  - path_resistance_accept_limit_ohm: 0.294
  - pack_rest_voltage_below_which_sustained_load_trips_worst_case_V:
    - peak_load: 3.621
    - typ_load: 3.381
  - sense_thevenin_kohm: 163.0
  - sense_filter_tau_ms_100nF: 16.3
  - power_on_reset: output low for 1.9 ms after V+ rises (F C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/verify/electrical_work/tlv3011.txt:479); shorter than the sense filter
- drain_after_cutoff_uA:
  - uvlo_divider_at_3V0: 4.17
  - R68_pulldown_EN_low: 0.0
  - TLV3012B_Iq_typ_max: [2.4, 3.6]
  - TPS61023_ISD_typ_max_25C: [0.1, 0.2]
  - new_block_total_typ: 6.67
  - new_block_total_max: 7.97
  - existing_R15_R16_47k_47k_divider_if_ADC_pin_high_Z: 34.0
  - existing_R15_into_unpowered_U2_PF3_clamp_worst: 61.7
  - on_state_divider_at_3V7: 5.0
  - on_state_R68_at_3V7: 3.7
- startup:
  - capacitance_uF:
    - +5V MLCC nominal (2x22u + 5x10u + 8x100n): 94.8
    - +3V3 behind U6 (22u + 100u tantalum + 10u + 100n): 132.1
  - total_uF: 226.9
  - precharge_time_ms_to_3p6V_at_350mA_typ: 2.33
  - precharge_time_ms_at_200mA_min: 4.08
  - cout_effective_window_uF: [4, 1000]
- inductor:
  - part: FXL0420-1R0-MA1 (C53426685)
  - Isat_typ_max_A: [6.5, 9.0]
  - Irms_typ_max_A: [5.4, 6.0]
  - DCR_mohm: 27
  - IL_peak_in_current_limit_typ_A: 5.098
  - source: F C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/datasheets/FXL0420_C167203/FXL0420_C167203.txt:279
  - note: ILIM has no datasheet maximum; valley 3.7 A typ + ripple = 5.1 A stays below Isat 6.5 A (first plan: 5.6 A, review E-3)
- loss_estimate_peak_W:
  - conduction_IC: 0.124
  - inductor_DCR: 0.059
  - IC_temp_rise_C_at_142p7_CperW_incl_50mW_switching_E: 24.9
- load_assumption: +5V load 273 mA typical / 783 mA peak (E: rev 2c power budget with the ESP-12F at 500 mA, F C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/datasheets/C82891_text/C82891.txt:122); 1.5 x peak = 1.175 A
- comparison:
  - {"proposal": "A", "converter": "TPS61023DRLR C919459 boost, 1 MHz, SOT-563, L 1.0 uH 2.0 mm high", "scores_1to5": {"i_margin_3V0": 5, "ii_cell_protection": 4, "iii_stock_and_library_type": 4, "iv_kicad_library": 2, "v_layout_and_area": 4, "vi_inspect_rework": 4}, "sum": 23}
  - {"proposal": "B", "converter": "TPS63061DSCR C2876326 buck-boost, 2.4 MHz, WSON-10-EP 3x3", "scores_1to5": {"i_margin_3V0": 3, "ii_cell_protection": 4, "iii_stock_and_library_type": 3, "iv_kicad_library": 4, "v_layout_and_area": 3, "vi_inspect_rework": 2}, "sum": 19}
  - {"proposal": "C", "converter": "MT3608 C84817 async boost + SS34 + 4.7 uH 5x5x4 mm + load-switch FET", "scores_1to5": {"i_margin_3V0": 1, "ii_cell_protection": 3, "iii_stock_and_library_type": 4, "iv_kicad_library": 4, "v_layout_and_area": 1, "vi_inspect_rework": 5}, "sum": 18}
- grafts:
  - from B: a pin-identical Comparator library symbol for U9 instead of a generic connector symbol (now Comparator:MCP65R41, push-pull)
  - from C: widen the battery path (VBAT_LOGIC_RAW / VBAT_LOGIC / VBAT_LOGIC_SW as pours or >= 2.0 mm) to <= 0.035 Ohm (rev 2c trace about 0.08 Ohm, E); all leaded / visible packages as a selection criterion
  - from the BOM re-check: C45 -> C11366 and U7 -> C82891
- rejected:
  - B's 10 MOhm hysteresis network: leakage-sensitive on a board that is not cleaned by the user
  - moving R15 (FCU_LBAT_ADC divider) behind a switch: firmware / calibration untouched in rev 3 (D13)
  - TLV3011AIDBVR C2870632 (first plan): limits from a superseded datasheet, open-drain output made EN a 701 kOhm node, no power-on reset (review E-1, E-2, E-8)
  - V_on maximum <= 3.70 V: no E24 combination keeps the worst-case hysteresis above the rebound at peak load; the limit is 3.75 V (50 mV under the pre-flight rule)
- review_numbers_source: C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/final/uvlo_tlv3012b.json
- review_command: cd /b/Claude && .venv/Scripts/python.exe -X utf8 C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/final/uvlo_tlv3012b.py 470e3 270e3 3.3e6
- en_drive: EN is driven by the push-pull output: high = VBAT - VOH (VOH 90 typ / 200 max mV at 5 mA, F C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/verify/electrical_work/tlv3011.txt:292; at the microamp load here below 10 mV, E), low = VOL; source impedance <= 40 Ohm (E from VOH max); R68 1M pull-down defines EN while U9 is unpowered

## 6. JLC BOM (snapshot, not a promise)

I7 per line: available = min(stock, pre-sale quantity) >= max(20 x quantity + loss, minimum order), buyable, assembled by JLC, assembly library. 'Time' is the fetch time of the record (KST) on 2026-11-10; the record file is in the JSON (`evidence_file`).

| # | Comment | Designators | Footprint | JLC code | MPN | Library | Process | Qty | Stock | Available | Need | Time | I7 |
|---|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 100n 16V X7R 0402 | C1, C2, C4, C9, C10, C12, C17, C18, C20, C25, C26, C28, C54 | C_0402_1005Metric | C1525 | CL05B104KO5NNNC | Basic | SMT | 13 | 32109226 | 24064886 | 275 | 20:56 | ORDERABLE |
| 2 | 10u 25V X5R 0805 | C3, C11, C19, C27, C44, C53, C56 | C_0805_2012Metric | C15850 | CL21A106KAYNNNE | Basic | SMT | 7 | 8530242 | 6731916 | 150 | 20:56 | ORDERABLE |
| 3 | 100n 50V X7R 0603 | C5, C8, C13, C16, C21, C24, C29, C32, C46, C48, C49, C50, C51, C55, C59, C60 | C_0603_1608Metric | C14663 | CC0603KRX7R9BB104 | Basic | SMT | 16 | 36656358 | 26279573 | 335 | 20:56 | ORDERABLE |
| 4 | 27p 50V C0G 0402 | C6, C7, C14, C15, C22, C23, C30, C31 | C_0402_1005Metric | C1557 | 0402CG270J500NT | Extended (Preferred) | SMT | 8 | 360854 | 260171 | 170 | 20:56 | ORDERABLE |
| 5 | 22u 25V X5R 1206 | C41, C42, C43, C57 | C_1206_3216Metric | C12891 | CL31A226KAHNNNE | Basic | SMT | 4 | 1977813 | 1569820 | 84 | 20:56 | ORDERABLE |
| 6 | 22u 10V tantalum A | C45 | CP_EIA-3216-18_Kemet-A | C11366 | TAJA226K010RNJ | Extended | SMT | 1 | 403792 | 395341 | 22 | 20:56 | ORDERABLE |
| 7 | 220u 10V electrolytic D6.3xL5.4 | C47 | CP_Elec_6.3x5.4 | C2833309 | RVT220UF10V67RV0037 | Extended | SMT | 1 | 192389 | 184316 | 23 | 20:56 | ORDERABLE |
| 8 | 100u 10V tantalum B | C52 | CP_EIA-3528-21_Kemet-B | C7196 | TAJB107M010RNJ | Extended | SMT | 1 | 93450 | 90022 | 20 | 20:56 | ORDERABLE |
| 9 | 1n 50V X7R 0603 | C58 | C_0603_1608Metric | C1588 | CL10B102KB8NNNC | Basic | SMT | 1 | 5712854 | 4447683 | 30 | 20:56 | ORDERABLE |
| 10 | LED red 0603 | D1, D2, D3, D4 | LED_0603_1608Metric | C2286 | KT-0603R | Basic | SMT | 4 | 6279546 | 5611421 | 90 | 20:56 | ORDERABLE |
| 11 | B5819W 1A 40V Schottky SOD-123 | D6 | D_SOD-123 | C8598 | B5819W SL | Basic | SMT | 1 | 725265 | 596424 | 28 | 20:56 | ORDERABLE |
| 12 | JST-GH BM04B-GHS-TBT 4P vertical | J7 | JST_GH_BM04B-GHS-TBT_1x04-1MP_P1.25mm_Vertical | C161692 | BM04B-GHS-TBT(LF)(SN) | Extended | SMT | 1 | 1877 | 1090 | 20 | 21:41 | ORDERABLE |
| 13 | 10uH 0805 multilayer inductor | L1, L2, L3, L4 | L_0805_2012Metric | C1046 | SDFL2012S100KTF | Basic | SMT | 4 | 190908 | 166746 | 88 | 20:56 | ORDERABLE |
| 14 | 1.0uH FXL0420-1R0-MA1 | L5 | L_Changjiang_FXL0420 | C53426685 | FXL0420-1R0-MA1 | Extended | SMT | 1 | 1065 | 1065 | 22 | 21:41 | ORDERABLE |
| 15 | BSS138 | Q1, Q2, Q4, Q5, Q6, Q7, Q8, Q11 | SOT-23 | C40912 | BSS138-7-F | Extended | SMT | 8 | 399691 | 398690 | 165 | 20:56 | ORDERABLE |
| 16 | MMBT3904 | Q9 | SOT-23 | C20526 | MMBT3904(RANGE:100-300) | Basic | SMT | 1 | 4325651 | 4207844 | 25 | 20:56 | ORDERABLE |
| 17 | AO3401A | Q10 | SOT-23 | C15127 | AO3401A | Basic | SMT | 1 | 1646471 | 1514696 | 24 | 20:56 | ORDERABLE |
| 18 | 10k 1% | R1, R3, R5, R7, R11, R14, R23, R24, R25, R26, R27, R29, R31, R38, R40, R41, R43, R45, R46, R48, R49, R50, R51, R52, R53, R55, R56, R57, R58, R59, R60, R61, R62 | R_0603_1608Metric | C25804 | 0603WAF1002T5E | Basic | SMT | 33 | 45566676 | 35161809 | 670 | 20:56 | ORDERABLE |
| 19 | 1k 1% | R2, R4, R6, R8, R12, R13, R28, R30, R54 | R_0603_1608Metric | C21190 | 0603WAF1001T5E | Basic | SMT | 9 | 25423222 | 18819229 | 190 | 20:56 | ORDERABLE |
| 20 | 47k 1% | R15, R16, R32, R34, R36 | R_0603_1608Metric | C25819 | 0603WAF4702T5E | Basic | SMT | 5 | 3000073 | 2340211 | 110 | 20:56 | ORDERABLE |
| 21 | 220R 1% | R17, R18, R19, R20, R21 | R_0603_1608Metric | C22962 | 0603WAF2200T5E | Basic | SMT | 5 | 2696901 | 2284098 | 110 | 20:56 | ORDERABLE |
| 22 | 4.7k 1% | R33, R35, R37, R42, R44 | R_0603_1608Metric | C23162 | 0603WAF4701T5E | Basic | SMT | 5 | 15290785 | 12048281 | 110 | 20:56 | ORDERABLE |
| 23 | 100k 1% | R39 | R_0603_1608Metric | C25803 | 0603WAF1003T5E | Basic | SMT | 1 | 15576301 | 13221898 | 30 | 20:56 | ORDERABLE |
| 24 | 150k 1% | R63 | R_0603_1608Metric | C22807 | 0603WAF1503T5E | Basic | SMT | 1 | 702670 | 568740 | 30 | 20:56 | ORDERABLE |
| 25 | 20k 1% | R64 | R_0603_1608Metric | C4184 | 0603WAF2002T5E | Basic | SMT | 1 | 5298488 | 4350163 | 30 | 20:56 | ORDERABLE |
| 26 | 470k 1% | R65 | R_0603_1608Metric | C23178 | 0603WAF4703T5E | Basic | SMT | 1 | 895208 | 733918 | 30 | 21:32 | ORDERABLE |
| 27 | 270k 1% | R66 | R_0603_1608Metric | C22965 | 0603WAF2703T5E | Extended (Preferred) | SMT | 1 | 222414 | 176246 | 30 | 21:32 | ORDERABLE |
| 28 | 3.3M 1% | R67 | R_0603_1608Metric | C26096 | 0603WAF3304T5E | Extended | SMT | 1 | 222671 | 182642 | 30 | 21:32 | ORDERABLE |
| 29 | 1M 1% | R68 | R_0603_1608Metric | C22935 | 0603WAF1004T5E | Basic | SMT | 1 | 4171558 | 3382549 | 30 | 20:57 | ORDERABLE |
| 30 | AMS1117-3.3 | U6 | SOT-223-3_TabPin2 | C6186 | AMS1117-3.3 | Basic | SMT | 1 | 1543207 | 1409446 | 20 | 20:57 | ORDERABLE |
| 31 | ESP-12F | U7 | ESP-12E | C82891 | ESP-12F(ESP8266MOD) | Extended | SMT | 1 | 31195 | 30239 | 20 | 20:57 | ORDERABLE |
| 32 | TPS61023DRLR | U8 | SOT-563 | C919459 | TPS61023DRLR | Extended | SMT | 1 | 31489 | 30635 | 21 | 20:57 | ORDERABLE |
| 33 | TLV3012BIDBVR | U9 | SOT-23-6 | C20345924 | TLV3012BIDBVR | Extended | SMT | 1 | 1531 | 1161 | 20 | 21:41 | ORDERABLE |
| 34 | 16MHz 18pF 3225 | Y1, Y2, Y3, Y4 | Crystal_SMD_3225-4Pin_3.2x2.5mm | C70562 | X322516MRB4SI | Extended | SMT | 4 | 27902 | 27762 | 84 | 20:57 | ORDERABLE |
| 35 | socket 1x6 right-angle 2.54 | J2, J5 | PinSocket_1x06_P2.54mm_Horizontal | C22436147 | HC-PM254-8.5H-1x6PW | Extended | THT | 2 | 553 | 543 | 40 | 21:41 | ORDERABLE (thin) |
| 36 | pin header 1x3 right-angle 2.54 | J8, J9, J10, J11, J12 | PinHeader_1x03_P2.54mm_Horizontal | C32713262 | HX PZ2.54-1x3P WZ | Extended | THT | 5 | 30851 | 30685 | 100 | 20:57 | ORDERABLE |
| 37 | Molex 22035025 SPOX 2P vertical | J14 | Molex_SPOX_5267-02A_1x02_P2.50mm_Vertical | C5364642 | 22035025 | Extended | THT | 1 | 5877 | 5456 | 21 | 20:57 | ORDERABLE |
| 38 | JST B2B-PH-K-S 2P vertical | J15, J17 | JST_PH_B2B-PH-K_1x02_P2.00mm_Vertical | C131337 | B2B-PH-K-S(LF)(SN) | Extended | THT | 2 | 304854 | 189946 | 41 | 20:57 | ORDERABLE |
| 39 | pin header 1x2 right-angle 2.54 | J16 | PinHeader_1x02_P2.54mm_Horizontal | C32713261 | HX PZ2.54-1x2P WZ | Extended | THT | 1 | 55654 | 55427 | 20 | 20:57 | ORDERABLE |
| 40 | pin header 1x4 right-angle 2.54 | J18 | PinHeader_1x04_P2.54mm_Horizontal | C32713263 | HX PZ2.54-1x4P WZ | Extended | THT | 1 | 27550 | 27335 | 20 | 20:57 | ORDERABLE |

Fee-bearing Extended SMT types (11): C11366, C2833309, C7196, C161692, C53426685, C40912, C26096, C82891, C919459, C20345924, C70562. Basic lines 21, Preferred Extended lines 2. Components about 10.51 USD per board (E, tier-1 prices).

## 7. Alternates and the order-day gate

On the day of ordering every BOM line and alternate is fetched again (>= 1.5 s between requests, cache under C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/jlc_cache) and I7 is recomputed. A failing line is replaced only by its named alternate in the listed order; any other replacement is a design change and needs a new check and the user. Nothing is ordered by the tool: the user uploads and pays.

| For | Alternate (in this order) | Stock | Pre-sale | Time | I7 today | Note |
|---|---|---|---|---|---|---|
| J2, J5 (C22436147) | 1. C55629861 SH-PM254R-1x6P-H85 | 985 | 985 | 21:41 | ORDERABLE | same KiCad footprint assumed for both (no datasheet for C55629861: NOT_VERIFIED); C55629861 tails 3.2 mm -> protrusion about 1.6 mm, flush-cut |
| J2, J5 (C22436147) | 2. C54876735 PM2.54-1x6PWZ-H8.5 | 961 | 595 | 21:06 | ORDERABLE | same KiCad footprint assumed for both (no datasheet for C55629861: NOT_VERIFIED); C55629861 tails 3.2 mm -> protrusion about 1.6 mm, flush-cut |
| L5 (C53426685) | 1. C167203 FXL0420-1R0-M | 107845 | 106839 | 20:56 | ORDERABLE | same series and land pattern, Isat 5.6 A (first plan's part), stock 107845 |
| J7 (C161692) | 1. C52037397 GH1.25-4PLT | 1883 | 1847 | 21:07 | ORDERABLE | other makers' GH 1.25 mm 4P vertical; footprint check required before use |
| J7 (C161692) | 2. C2829262 GH1.25-4PLTPZ | 57256 | 56193 | 21:07 | ORDERABLE | other makers' GH 1.25 mm 4P vertical; footprint check required before use |
| U8 (C919459) | 1. C1852149 TPS61023DRLT | 550 | 531 | 21:07 | ORDERABLE | same MPN, other reel; stock 550 (thin) |
| J8..J12, J16, J18 (THT right-angle headers) if JLC refuses hand soldering | 1. C46061676 HX PZ2.54-1x3P WT | 20527 | 20110 | 21:41 | ORDERABLE | SMD right-angle headers, reflow; footprint not chosen (needs an on-disk KiCad footprint and the user's yes) |
| J8..J12, J16, J18 (THT right-angle headers) if JLC refuses hand soldering | 2. C46061675 HX PZ2.54-1x2P WT | 19488 | 19065 | 21:41 | ORDERABLE | SMD right-angle headers, reflow; footprint not chosen (needs an on-disk KiCad footprint and the user's yes) |
| J8..J12, J16, J18 (THT right-angle headers) if JLC refuses hand soldering | 3. C46061677 HX PZ2.54-1x4P WT | 13982 | 13030 | 21:41 | ORDERABLE | SMD right-angle headers, reflow; footprint not chosen (needs an on-disk KiCad footprint and the user's yes) |
| J15, J17 (C131337) | 1. C160352 B2B-PH-SM4-TB(LF)(SN) | 88732 | 66284 | 19:36 | ORDERABLE | SMD JST-PH; new footprint (D7) |
| U9 (C20345924) | none | - | - | - | - | no second source with the same limits: TLV3011B is not stocked; a stock-out stops the order |

Parts behind user decisions (not on the board by default):

| Option | Part | Stock | Pre-sale | Time | I7 today |
|---|---|---|---|---|---|
| D4 MCUs fitted by JLC | C6510 ATMEGA128A-AU | 2956 | 2846 | 20:57 | ORDERABLE |
| D6 option B headers | C32713283 HX PZ2.54-2x3P TP | 36386 | 33586 | 19:41 | ORDERABLE |
| D6 option B headers | C32713268 HX PZ2.54-1x2P ZZ | 183285 | 178093 | 20:04 | ORDERABLE |
| D6 option B headers | C41417359 HX PZ2.54-1x2P TP-YQ | 46608 | 46151 | 20:04 | ORDERABLE |
| D3 revert to rev 2c part numbers | C7183 TAJA226M010RNJ | 2264 | 1913 | 19:32 | ORDERABLE |
| D3 revert to rev 2c part numbers | C55629817 SH-PZ254R-1x3P | 2000 | 2000 | 21:07 | ORDERABLE |
| D3 revert to rev 2c part numbers | C55629816 SH-PZ254R-1x2P | 1750 | 1750 | 19:32 | ORDERABLE |
| D3 revert to rev 2c part numbers | C55629818 SH-PZ254R-1x4P | 1898 | 1898 | 19:32 | ORDERABLE |
| D3 revert to rev 2c part numbers | C55629861 SH-PM254R-1x6P-H85 | 985 | 985 | 21:41 | ORDERABLE |
| D8 other level-shifter FET | C7420339 BSS138 | 691851 | 567173 | 21:07 | ORDERABLE |

## 8. Parts the user fits

| Ref | Part | How | On the board |
|---|---|---|---|
| U1, U2, U3, U4 | ATmega128A-AU (TQFP-64, 0.8 mm pitch) | user solders 4 chips, 256 joints (default, D4); or JLC fits C6510 with the with_mcu file pair | footprint only |
| J1 | GY-521 MPU-6050 module (SZH-EK007, devicemart 1247052) | solder the module's own 1x8 pin strip into the plated holes (8 joints), module flat on a 2 mm foam / nylon support | 8 plated holes, no part |
| J2 | BMP280 module TYE-BMP29 (devicemart 15442588) | plug its right-angle 6-pin header into socket J2 (if the delivered module has a straight header, solder a right-angle strip first - header type NOT_VERIFIED) | socket J2 (JLC) |
| J7 | NEO-7M GPS module SZH-NT07 (devicemart 10895400) | sits on the vehicle top cap; a 4-wire JST-GH lead goes to J7 | JST-GH J7 (JLC) |
| J5 | SparkFun OpenLog DEV-13712 (logger, treated like a sensor module: D5) | user solders a right-angle 1x6 pin header onto the OpenLog (6 joints on the module) and plugs it into socket J5; fit NOT_VERIFIED (D5, D15) | socket J5 (JLC) |
| J3, J19, J20, J21, J22, J24, J25, J26 | programming access (not MCU, not sensor): 5 x 2x3 and 3 x 1x2 positions | D6 default A: bare plated holes; the user solders 5 x 2x3 + 3 x 1x2 headers (36 joints) and fits 3 shunts, or uses a pogo-pin adapter (contact NOT_VERIFIED). Option B: fitted by JLC | plated holes only (default) |

## 9. Off-board purchase list (not in the JLC BOM)

- 4 x ATmega128A-AU (unless JLC fits them, D4) - domestic distributor or LCSC C6510
- GY-521 MPU-6050 module; BMP280 TYE-BMP29 module (+ a right-angle 1x6 pin strip if its header is straight) - devicemart (as in rev 2c)
- NEO-7M SZH-NT07 (devicemart) + GPS lead: a ready-made 4-wire JST-GH 1.25 mm lead (domestic shop), or 2 x GHR-04V-S housings + 8 x SSHL-002T-P0.2 contacts from LCSC (C160418, C189897; crimp tool needed)
- SparkFun OpenLog DEV-13712 + microSD card + one right-angle 1x6 2.54 mm pin header (any 2.54 mm right-angle strip cut to 6 pins from a domestic shop; LCSC C55629820)
- logic battery 1S Li-Po DTP105565P 3.7 V 4200 mAh (Molex 5264-type 2-pin plug - mating NOT_VERIFIED against the pack in hand)
- power switch MTS-102 + lead with JST PHR-2 housing and SPH-002T-P0.5S contacts (LCSC C157955, C111515, or a ready-made PH 2.0 mm 2-pin lead from a domestic shop)
- arm switch + 2-pin 2.54 mm Dupont lead (J16); buzzer + JST PH 2-pin lead (J17) - domestic shop
- only if headers are soldered for programming (D6 A): 5 x 2x3 and 3 x 1x2 2.54 mm pin headers + 3 shunts (domestic shop; LCSC C65114, C55629809, C100114); otherwise a 2x3 pogo-pin ISP adapter
- USBasp programmer; USB-serial adapter with 3.3 V logic for the ESP-12F; 3 mm foam tape and one cable tie
- JLC does not ship loose parts with a PCBA order: the C-numbers above are LCSC order codes. Stock and price of these loose items were not checked today: NOT_VERIFIED.

## 10. JLC order

### Default: Standard PCBA

- pcba_type: Standard
- assembly_side: Top only
- layers: 4
- thickness_mm: 1.6
- stackup: JLC standard 4-layer stack-up (no impedance control)
- pcb_qty: 5
- pcba_qty: 2 (default, D11) or 5; JLC minimum 2
- confirm_parts_placement: Yes, 0.45 USD (CPL rotations of polarised parts are NOT_VERIFIED)
- bake: ask for baking of moisture-sensitive parts (ESP-12F C82891, LEDs C2286) in the order remark; fee NOT_VERIFIED (D14)
- surface_finish: ENIG (D16; surcharge NOT_VERIFIED)
- edge_rails: 5 mm process edges with tooling holes and fiducials, added by JLC (F C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/scout/jlc_rules.md:172, :176); the two rail sides are chosen from the edge-distance table (implementation check); boards arrive with the rails attached (F :326)
- tooling_holes: on JLC's rails; none in the design (H1 / H2 of the first plan are dropped, review OM-5)
- reflow_peak: 240 +-5 C (F jlc_rules.md)
- why: M C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/verify/order_mech_work/measure_board.json: 27 part bodies are closer than 2.5 mm to the outline (T&C 2.3, F C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/scout/jlc_rules.md:30) and two are inside the 0.3 mm Economic rule (U7 0.15 mm, J15 0.29 mm; F C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/scout/jlc_rules.md:164). The connector positions are fixed (invariant b). All BOM lines have componentProductType 0 (usable in both order types); board 76 x 70 mm meets the 70 x 70 mm minimum.
- acceptance: NOT_VERIFIED until the user uploads the files and reads JLC's DFM result and quote (before paying). Gerber, BOM and CPL are identical for both order types.
- fees_usd_E:
  - 2_boards:
    - setup: 25.56
    - stencil: 8.21
    - feeder_types: 34
    - feeder_loading: 52.02
    - tht_hand_soldering: 4.86
    - confirm_parts_placement: 0.45
    - components_tier1_prices: 21.02
    - sum_without_pcb_shipping_tax: 112.12
  - 5_boards:
    - setup: 25.56
    - stencil: 8.21
    - feeder_types: 34
    - feeder_loading: 52.02
    - tht_hand_soldering: 6.78
    - confirm_parts_placement: 0.45
    - components_tier1_prices: 52.55
    - sum_without_pcb_shipping_tax: 145.57
- standard_only_fees_applicability_NOT_VERIFIED:
  - handling_fee_min: 14.93
  - two_fixtures_1_to_29_pcs: 16.42
  - packing_E: 0.5
  - sum: 31.85
  - source: F C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/scout/jlc_rules.md:147-148
- sum_with_standard_only_fees_E:
  - 2_boards: 143.97
  - 5_boards: 177.42

### Alternative: Economic PCBA

- pcba_type: Economic
- assembly_side: Top only
- layers: 4
- thickness_mm: 1.6
- stackup: JLC standard 4-layer stack-up (no impedance control)
- pcb_qty: 5
- pcba_qty: 2 (default, D11) or 5; JLC minimum 2
- confirm_parts_placement: Yes, 0.45 USD (CPL rotations of polarised parts are NOT_VERIFIED)
- bake: ask for baking of moisture-sensitive parts (ESP-12F C82891, LEDs C2286) in the order remark; fee NOT_VERIFIED (D14)
- surface_finish: ENIG (D16; surcharge NOT_VERIFIED)
- when: only if JLC's DFM result accepts the parts near the edge (U7 0.15 mm, J15 0.29 mm) and THT hand soldering on an Economic order, seen by the user before paying; otherwise Standard
- tooling_holes: 'Added by JLCPCB' (JLC may drill 1.152 mm holes where copper is, F C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/scout/jlc_rules.md:173); 'Added by Customer' only if the legal-position search proves two positions (D10)
- reflow_peak: 255 +-5 C (F jlc_rules.md) - reflow rating table required (check, review OM-9)
- fees_usd_E:
  - 2_boards:
    - setup: 8.18
    - stencil: 1.53
    - feeder_types: 11
    - feeder_loading: 33.77
    - tht_hand_soldering: 4.86
    - confirm_parts_placement: 0.45
    - components_tier1_prices: 21.02
    - sum_without_pcb_shipping_tax: 69.81
  - 5_boards:
    - setup: 8.18
    - stencil: 1.53
    - feeder_types: 11
    - feeder_loading: 33.77
    - tht_hand_soldering: 6.78
    - confirm_parts_placement: 0.45
    - components_tier1_prices: 52.55
    - sum_without_pcb_shipping_tax: 103.26

### Files and fee basis

- Files to upload: gerbers zip (4 copper layers + drill); jlcpcb_bom.csv (Comment,Designator,Footprint,LCSC Part #); jlcpcb_cpl.csv (Designator,Mid X,Mid Y,Layer,Rotation)
- Optional: jlcpcb_bom_with_mcu.csv / jlcpcb_cpl_with_mcu.csv add U1..U4 = C6510 (ATMEGA128A-AU, Extended, 4 chips per board, stock 2956 / presale 2846 at 20:57, I7 ORDERABLE; recommended in D4)
- Fee basis: F C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/scout/jlc_rules.md (fee table read 2026-11-10); the order-page option names and the final quote are NOT_VERIFIED (no Gerber was uploaded, nothing was put in a cart)

## 11. Renders

| Id | Board file | What |
|---|---|---|
| a1-a3 | render_as_delivered.kicad_pcb | as delivered by JLC: top, 3/4 front, bottom (models of every user-fitted part hidden: U1-U4, modules, GPS plug) |
| b1-b4 | render_completed.kicad_pcb | completed by the user: top, 3/4 front, 3/4 back, side elevation with the 13.4 mm limit line |
| c1-c5 | render_completed.kicad_pcb | sensor close-ups: GY-521 on J1, BMP280 in J2, OpenLog in J5 (microSD side), ESP-12F with antenna keep-out, one MCU with the pin-1 mark. The OpenLog picture shows an assumed fit (A); the real fit is decided by the fit check, not by this render. |
| d1-d2 | render_completed.kicad_pcb + PyVista vehicle scene | NEO-7M: JST-GH plug and lead at J7; module on the top cap with the lead down to the carrier |
| e1-e4 | composed (PIL + malgun.ttf, 1920 x 1080, Korean captions) | 'JLC에서 받은 상태' vs '직접 장착한 뒤'; one slide per sensor with pin table and who solders what; GPS lead slide; overview with call-outs |
| models | - | CadQuery coloured STEP for GY-521, BMP280, OpenLog, NEO-7M, GH plug, J14 (no library model) -> ${KIPRJMOD}/3d/*.step -> kicad-cli pcb render. Module dimensions are A until measured with calipers (D15); captions must say so. NEO-7M outline in hand is the NEO-6M one (A). Interpreter: B:/0)AI_Cad/CanSat_EDF_Rocket/.venv/Scripts/python.exe -B (cadquery 2.8.0, pyvista 0.49.1, M import test; the worktree has no .venv); scripts and outputs stay in the worktree. |
| spike | - | first implementation step: build the d2 scene once end-to-end before any other render work (review OM-6) |

## 12. Placement notes

- Power block goes into the former Pololu corner (+x / -y; rev 2c module body x 24.52..37.22, y -34.17..-18.87, header strip x 24.1..37.5, y -34.5..-31.1). The 2.0 mm under-module height limit there disappears with the module.
- Area: 13 new parts need about 95 mm2 of courtyard (E: U8 4.6, L5 26.8, U9 13.9, C57 10.6, 6 R 25.9, 3 C0603 13.0) against 45.6 mm2 freed by the J6 strip; board courtyard occupancy was 87.6 %. U6, C43, C44, C45 and C41 / C42 must be re-placed inside the corner. Whether it fits is NOT_VERIFIED until place.py runs; the L5 courtyard (5.7 x 4.7 mm) does not fit the 3.4 mm strip by itself.
- Hot loop first: U8 pin 6 (VOUT) -> C57 / C43 -> U8 pin 4 (GND) as small as the courtyards allow, both capacitors within 2 mm of the IC, GND vias to In1 directly at the capacitor pads.
- L5 next to U8 pins 3 / 5; C41 / C42 (input capacitors, re-used) at the L5 / VIN end; BOOST_SW copper only as large as the two pads need.
- R63 / R64 / C58 at U8 pin 1, on the side away from L5 and BOOST_SW.
- U9, R65..R68, C59, C60 as one cluster near U8 pin 2; UVLO_SENSE is a 163 kOhm node: short, not parallel to BOOST_SW, GND on both sides. BOOST_EN is driven by the comparator output (low impedance).
- Keep the block at least as far from the NAV crystal Y1 / U1 XTAL pins as the rev 2c Pololu pins were; the distance to J7 (GPS lead, about 64 mm) and to the ESP antenna keep-out (about 67 mm) does not shrink (E, corner-to-corner).
- No connector, MCU, crystal, module socket or ESP footprint moves (ANCHORS unchanged). U7 keeps footprint RF_Module:ESP-12E and its position.
- No tooling holes in the default design (Standard order: holes on JLC's rails). H1 / H2 only under the condition in conditional_items - never by loosening a rule.
- Heights: all new parts <= 2.0 mm (L5 2.0 mm F datasheet; SOT-563 0.6, SOT-23-6 1.45 nominal E) - add FP_HEIGHT entries; the 13.4 mm limit is unaffected.
- Silkscreen: unambiguous polarity / pin-1 marks on F.SilkS for C45, C52 (tantalum: reversed = fire hazard), C47, D1-D4, D6, Q1-Q11, U6, U7, U8, U9, J7, U1-U4; refdes of U8 / U9 / L5 readable after assembly.
- All parts stay on the top side; the bottom stays free for the 3 mm foam tape.
- Edge distances are reported, not changed: the connectors and U7 keep their rev 2c positions (invariant b); the edge-distance table names the two board sides for JLC's rails.
- Programming-access report for D6: search positions for 5 SMD 2x3 headers and 3 1x2 headers outside the tie band without moving an anchored part; report found / not found - do not apply without the user's yes.
- OpenLog over J5: keep-out for the module body and the microSD card overhang from calliper numbers; the parts under it today are R45, R43, R2, C6, R44, R42, R39, U1 (1.2 mm), C48, R29, Q2 (1.1 mm) (M C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/verify/order_mech_work/measure_board.json).

## 13. Routing notes

- Any footprint change invalidates the Freerouting session replay: a fresh route is needed (rev 2c: 20-35 min per attempt, 4-10 attempts, E).
- VBAT_LOGIC_RAW / VBAT_LOGIC / VBAT_LOGIC_SW from J14 through Q10 and J15 to L5: pour or >= 2.0 mm, target <= 0.035 Ohm (rev 2c about 0.08 Ohm, E). At 3.0 V and peak load the path carries about 1.75 A average (E) - the Pololu module saw the same current, so this is an improvement, not a new requirement.
- BOOST_SW: F.Cu only, no via, as short as the pads allow.
- +5V: from the C57 / C43 pads into the In2 +5V plane with >= 3 vias; U8 pin 4 and the capacitor grounds into In1 GND with >= 3 vias; keep In1 solid under the block.
- BOOST_FB sensed at the output capacitor, routed away from BOOST_SW / L5.
- No signal trace under L5 on F.Cu.
- Design rules, net classes and the XTAL checks are used unchanged; if the corner cannot be routed, the answer is a different placement, not a relaxed rule.

## 14. Checks to add in the implementation phase

- fit-set equality: set(BOM designators) == set(CPL designators) == {parts with fit == jlc}; U1-U4, J1, J3, J19-J22, J24-J26, Q3, R47, JP1, NT1 in neither file (and exactly the MCUs added in the with_mcu pair).
- U8 / U9 pad -> net equality against a table transcribed from the datasheets (TPS61023 tps61023.txt:150-158; TLV3012B SBOS300C DBV pin table), run twice: on the exported netlist and on the pads of the .kicad_pcb. ERC gives no evidence for U8 (generic symbol) - the report must say so. Second-reader review of the table.
- MCU pin-map equality: netlist pins of U1..U4 == C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/scout/rev2c_mcu_pinmap.json (0 differences).
- JLC source gate (hard): on the release day and again on the order day every BOM line and every named alternate is re-fetched; the newest record per code is used; a line that fails I7 stops the release (no warning-only mode); the snapshot time is printed per line; THIN lines (< 1000) are listed.
- library gate: every symbol / footprint in the spec exists on disk and every used pin number is a pad (probe_library.py extended with Comparator:MCP65R41 and the other new ids).
- ERC 0 errors with PWR_FLAG on +5V and VBAT_LOGIC_SW; DRC 0 violations / 0 unconnected with the rev 2c rule file unchanged (hash compared).
- height table: every footprint has an FP_HEIGHT entry (L5 = 2.0 mm, datasheet maximum); max height <= 13.4 mm; under-module limits for the parts that stay under a module.
- THT lead protrusion table (bottom side) against the 1.5 mm foam-tape rule: J15 / J17 1.8 mm nominal -> reported as a user flush-cut, not silently accepted; J14 from the Molex drawing.
- polarity marks: every polarised footprint has an F.SilkS mark that is not covered by the part body.
- CPL rotation table: each polarised part's rotation is listed with its source; without a verified source the line is NOT_VERIFIED and the order uses 'Confirm Parts Placement'.
- determinism: two builds from the same spec give byte-identical .kicad_sch / .kicad_pcb / BOM / CPL / gerbers; STEP files are compared with their timestamp header lines excluded.
- power bring-up record (hardware, user): switch-on and switch-off voltage (limits 3.44..3.73 V and 3.00..3.25 V), +5V at 0.27 A and 0.78 A, +5V and +3V3 minimum during Wi-Fi transmission (3V3 >= 3.0 V, F ESP-12F supply range), EN voltage, a scope capture of +5V / EN at switch-on, and the source resistance pack -> U8 VIN (accept <= 0.29 Ohm). Stays NOT_VERIFIED until measured.
- edge-distance table (review OM-1): body and pad distance of every footprint to the outline, with the lines under 2.5 mm (T&C 2.3) and under 0.3 mm listed for the order note; the two rail sides are chosen from this table. No rule is loosened and no anchored part is moved.
- MCU hand-solder access table (review OM-2): for U1..U4 the fitted parts within 1.0 mm of the pad ring and the parts higher than 2 mm within 5 mm; plus the paste-layer decision for U1..U4 pads (no paste aperture in the default gerbers, apertures only with the with_mcu pair) recorded in the manifest.
- tooling-hole legal-position search (review OM-5; only for an Economic order with 'Added by Customer'): after placement and routing, two opposite corners, clear of copper on all four layers, of keep-outs, module bodies and the tie band; not found -> no holes in the design.
- THT quote gate (review S2, user): JLC's quote must list the 6 THT lines as matched and hand-soldered before anything is paid; until then 'JLC solders the THT connectors' is NOT_VERIFIED.
- reflow rating table (review OM-9): rated reflow peak of each JLC-fitted part from its datasheet against 240 +-5 C (Standard) and 255 +-5 C (Economic): C2833309 (C47), C82891 (ESP-12F), C161692, C131337, C5364642, the pin headers and sockets; no rating in a document -> NOT_VERIFIED in the table.
- BOM / CPL format check (review OM-7) against JLC's sample files: header strings, encoding, unit suffix, sign of Y, THT rows; NOT_VERIFIED until JLC's upload preview shows every line matched (user gate).
- OpenLog / BMP280 fit check (review OM-4, S8): module body, microSD card overhang and insertion path as a keep-out built from calliper dimensions (D15) and the socket seating height from the Hong Cheng drawing; clearance >= 0.5 mm (A) over the tallest part underneath; decided by these numbers, not by a render.
- render tool-chain spike as the first implementation step (review OM-6): build the d2 scene (NEO-7M on the top cap) once with B:/0)AI_Cad/CanSat_EDF_Rocket/.venv/Scripts/python.exe -B on worktree scripts (PYTHONDONTWRITEBYTECODE=1, outputs only under B:/0)AI_Cad/CanSat_EDF_Rocket_pcb_rev3/pcb/vtol_carrier); NEO-7M dimensions labelled A.
- programming-access report (review S1): are there legal positions outside the tie band for 5 SMD 2x3 headers (C32713283) and 3 1x2 headers without moving an anchored part - found / not found, for D6.
- connector drawing check (review E-10): J14 Molex 22035025 hole, pitch and tail length against the KiCad footprint; J15 / J17 / J7 / U7 re-read; a mismatch is an error.
- ORDER_ko.md carries the order-page field table (D9, D10, D11, D14, D16) and the change-scope table (tag per change).

## 15. Before paying and after delivery

- Before paying (order day): re-run the JLC source gate; upload gerbers, BOM and CPL; read JLC's DFM result and quote: all 40 lines matched, the 6 THT lines hand-soldered, no objection to the parts near the edge. If JLC objects, stop and report - do not change the order type or a part without a new check.
- Inspect: tantalum C45 / C52 and electrolytic C47 polarity against the silkscreen, U7 / U8 / U9 / J7 orientation, no solder bridges on U8 (SOT-563, 0.5 mm pitch). Snap off the edge rails (Standard order).
- Bottom side: measure the THT lead protrusion; flush-cut every lead above 1.5 mm (J15 / J17: 4 leads, 1.8 mm nominal) before the foam tape goes on.
- Before any module or MCU is fitted, power J14 from a current-limited bench supply (100 mA limit) with J15 bridged: at 3.3 V the 5 V rail must stay off; raise slowly - it must start between 3.44 and 3.73 V; then check 5 V (4.84..5.33 V) and 3.3 V; lower slowly - it must cut off between 3.00 and 3.25 V. Record the four numbers and a scope capture of +5V and EN at switch-on.
- Load test: 0.27 A and 0.78 A on +5V at 3.4 V and 3.7 V supply (2 A limit): the rail must hold and U8 / L5 stay below about 60 C (E). With the real pack, switch and leads: measure the source resistance from the pack to U8 VIN (voltage drop / input current); accept <= 0.29 Ohm, otherwise the cut-off can cycle at the end of discharge - report before flying.
- Fit U1..U4 (ATmega128A-AU, pin 1 to the mark) unless JLC fitted them (D4): fine tip, flux, inspect every 0.8 mm pitch pin for bridges; the neighbouring parts are within 1 mm of the pads.
- Fit the modules: GY-521 soldered flat on its own 1x8 strip with a 2 mm support; BMP280 plugged into J2; OpenLog (with its own soldered right-angle header) plugged into J5 - check the pin order printed on each module against the board silkscreen first.
- Programming, one supply mode only: the board is powered from the logic pack (above 3.73 V) or a 4.0 V bench supply at J14; the USBasp target-power jumper is open (never 5 V from both). Access as decided in D6 (soldered headers + shunts, or a pogo adapter). Flash the ESP-12F through J3 at 3.3 V logic only.
- During the first Wi-Fi transmission record the minimum of +5V and +3V3.
- Plug the leads: battery J14, switch J15, GPS J7, servos J8..J11 / ESC J12, arm switch J16, buzzer J17. Mount as rev 2c: 3 mm foam tape + one cable tie across x -3..+3.

## 16. User decisions (the plan proceeds with the default)

| Id | Question | Options | Recommended | Why | Default used by the plan | Consequence of the default |
|---|---|---|---|---|---|---|
| D1 | Which 5 V converter replaces the Pololu module? | A: TPS61023 boost + TLV3012B cut-off (13 visible-lead parts) / B: TPS63061 buck-boost (real KiCad symbol, hidden WSON joints, 1.41x guaranteed margin, stock 4699) | A | A delivers 1.81 A at 3.0 V at the IC input (E, datasheet minimum current limit) = 2.32 x the 783 mA peak; every joint can be inspected and reworked. | A | U8 has no library symbol: ERC gives no evidence for U8; a pad-to-net table check replaces it. |
| D2 | Battery cut-off thresholds of the logic pack (1S Li-ion). | keep: on 3.44..3.73 V (nominal 3.59 V), off 3.00..3.25 V (nominal 3.13 V) / other thresholds: new resistor values and a new JLC stock check / no cut-off block: U9, R65..R68, C59, C60 are dropped, EN tied to the switched battery; only the pack's own protection circuit is left (the rev 2c Pololu module had a cut-off) | keep | switch-off never below 3.00 V and switch-on never above 3.73 V (worst case, M uvlo_tlv3012b.py), which is under the pre-flight rule 'pack >= 3.8 V'. | keep | A pack resting below 3.73 V may not start the board. Near the end of discharge a sustained peak load can trip the cut-off when the resting pack voltage is below about 3.62 V (E, worst-case threshold, 0.275 Ohm source resistance). |
| D3 | Three part-number changes are not forced by the request (larger stock or a verified drawing). | keep the plan's parts: C45 = C11366, pin headers J8..J12 / J16 / J18 = C3271326x, J2 / J5 = C22436147 / revert to the rev 2c part numbers: C7183, C55629817 / C55629816 / C55629818, C55629861 (all in the JLC library today, stock 2264 / 2000 / 1750 / 1898 / 985) | keep | C11366 has 403792 in stock against 2264; the HX headers 27550..55654 against 1750..2000; C22436147 is the only 1x6 right-angle socket whose manufacturer drawing was read against the KiCad footprint (review E-10). | keep | none on the board function; the rev 2c numbers stay as named alternates in the order-day gate. |
| D4 | Who fits the four ATmega128A-AU? | user solders them after delivery (as requested) / JLC fits them: upload jlcpcb_bom_with_mcu.csv / jlcpcb_cpl_with_mcu.csv (C6510, stock 2956 / presale 2846 today) | JLC fits them | M (C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/verify/order_mech_work/measure_board.json, section B): 14 / 15 / 20 / 14 already-fitted parts lie within 1.0 mm of the pad rings of U1 / U2 / U3 / U4, the J5 socket is 0.73 mm from one pin row of U2, C47 (5.4 mm high) is 2.5 mm away and the U4 pads are 0.5 mm from the board edge. Hand soldering 4 x 64 pins at 0.8 mm pitch between these parts is possible with a fine tip and flux but slow and easy to bridge (E). | user solders them (the request excludes the MCUs from the JLC BOM) | 256 fine-pitch joints by hand per board in tight clusters; both file pairs are generated so the choice can still be made on the order day. |
| D5 | How is the OpenLog fitted? It is a logger, not a sensor, and it is sold without a header. | plug-in: user solders a right-angle 1x6 header onto the OpenLog (6 joints on the module, none on the carrier) and plugs it into socket J5; no firmware change / on a short 6-wire lead into J5 (no fit question on the board, the module is taped elsewhere) / soldered down or an on-board logger: needs firmware work (not planned) | plug-in | keeps the firmware and the rev 2c connector position; the only user soldering is on the module itself. | plug-in; the OpenLog counts as a user-fitted module like the sensors | Fit is NOT_VERIFIED: stack height, module orientation and the microSD card overhang over R2, C6, Y1, C44, U1, C1, C7, C43 (M measure_board.json section E) are decided by a calliper measurement in the implementation phase; if it does not fit, the 6-wire lead is used. |
| D6 | Programming access: 5 x 2x3 ISP / ESP headers (J3, J19..J22) and 3 x 1x2 reset-hold jumpers (J24..J26) sit in the cable-tie band, where nothing may be fitted. | A: bare plated holes as in rev 2c. After delivery the user either solders 5 x 2x3 + 3 x 1x2 headers (36 joints; the cable tie must then run beside them) or programs through a pogo-pin adapter held on the holes (contact NOT_VERIFIED; a 0.64 mm square pin, 0.905 mm diagonal, does not press-fit in a 1.0 mm hole) / B: JLC fits the headers outside the band: C32713283 (SMD 2x3), C32713268 (THT 1x2) or C41417359 (SMD 1x2), all orderable today. Moves 8 footprints on a board that is 87.6 % occupied: placement NOT_VERIFIED and it changes connector positions (invariant b) | B if the implementation finds legal positions, otherwise A | B is the only option that needs no soldering by the user, which is what the request asks for; A keeps every connector where rev 2c had it. | A | 36 joints or a pogo adapter are user work that is neither MCU nor sensor. The implementation reports whether B is placeable; the board is not changed to B without the user's yes. |
| D7 | Switch and buzzer connectors J15 / J17 (JST-PH). | THT part C131337 soldered by JLC; user flush-cuts 4 leads (1.8 mm nominal against the 1.5 mm foam rule) / SMD part C160352 (stock 88732): new footprint, re-placement | THT | no footprint or position change. | THT | 4 leads to cut per board. |
| D8 | Level-shifter FET (8 per board). | C40912 BSS138-7-F / C7420339 (Preferred Extended, other manufacturer) | C40912 | same part as rev 2c; the saving of one 3.07 USD feeder fee exists only on an Economic order. | C40912 | none. |
| D9 | JLC order type. | Standard PCBA: JLC adds 5 mm edge rails with tooling holes and fiducials; about 112 USD for 2 assembled boards without PCB / shipping / tax (E), up to 144 USD if the handling, fixture and packing fees apply (NOT_VERIFIED) / Economic PCBA: about 70 USD (E); JLC panelises the board itself and routes it out after assembly | Standard | M (C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/verify/order_mech_work/measure_board.json, section A): 27 part bodies are closer than 2.5 mm to the board edge (T&C 2.3) and two are closer than the 0.3 mm that JLC suggests for Economic orders (U7 0.15 mm, J15 0.29 mm; depanel tolerance about 0.2 mm, F). The connector positions are fixed by the vehicle, so the parts cannot move. Edge rails are JLC's own answer for parts near the edge; Standard also solders at 240 C peak instead of 255 C. | Standard | about 42..74 USD more than Economic; boards arrive with the rails attached (snap off; depanelling fee NOT_VERIFIED). Gerber, BOM and CPL are the same files for both types. Acceptance by JLC is NOT_VERIFIED for both types until the user sees JLC's DFM result and quote; do not pay before that. |
| D10 | Tooling holes. | Standard order: on the rails, added by JLC (nothing in the design) / Economic order: 'Added by JLCPCB' (JLC may drill where copper is, F) / Economic order: two 1.152 mm holes in the design, only if the implementation proves two legal opposite-corner positions after placement and routing on all four layers | on the rails (follows D9) | M measure_board.json section D: the -x,-y corner has no free position at all and the +x,-y corner none free of tracks on the rev 2c board. | none in the design; H1 / H2 of the first plan are dropped | none with a Standard order. |
| D11 | Quantities. | 5 PCBs, 2 assembled (JLC minimum) / 5 PCBs, 5 assembled | 2 assembled first | the board was never built; the power block and the fit of the modules are not measured yet. | 5 PCBs / 2 assembled | 5 assembled cost about 146 USD (Standard, E). See D17 for the 3 bare boards. |
| D12 | Module socket J2 / J5 (1x6 right-angle). | C22436147 (stock 553 / presale 543: thin) / C55629861 (rev 2c part, 985, no datasheet, 3.2 mm tails) / C54876735 (961 / presale 595) | C22436147 | only part with a drawing that matches the KiCad footprint; 40 pieces are needed for 20 boards. | C22436147 with a hard stock check on the order day; then C55629861, then C54876735 | if all three are short on the order day the order waits; the user may also pre-order the part into the JLC parts library (user action). |
| D13 | Battery-voltage divider R15 / R16 (47k / 47k) stays as in rev 2c. | keep / change (firmware scale changes: not planned) | keep | no firmware change. | keep | 34 uA from the pack while the switch is on, also after the cut-off, plus 6.7 uA typ / 8.0 uA max of the new block. The user must switch off; the cut-off is not a storage switch. |
| D14 | Ask JLC to bake moisture-sensitive parts (ESP-12F, LEDs)? | yes / no | yes | T&C 5.10 offers it; the fee is NOT_VERIFIED. | yes (order remark) | small fee, unknown. |
| D15 | Module dimensions for the renders and the fit checks. | measure GY-521, BMP280, OpenLog, NEO-7M with callipers / accept assumed outlines labelled A | measure | the OpenLog fit (D5) is decided by these numbers; the NEO-7M outline in hand is the NEO-6M one (A). | assumed outlines, every picture and check labelled A until measured | renders may differ from the real modules. |
| D16 | PCB options on the order page that the plan did not name before. | surface finish ENIG or lead-free HASL / solder mask colour / copper weight, via covering, order-number mark, material | ENIG, green mask, white legend, JLC default 4-layer stack-up and copper, tented vias, order number placed by JLC | ENIG gives flat pads for the 0.8 mm pitch MCU pads and the bare holes that the user solders later; the rest are JLC defaults. | as recommended | ENIG surcharge and the exact option names are NOT_VERIFIED (nothing was uploaded). |
| D17 | Bare spare boards (PCB quantity above the assembled quantity). | accept them as spares for mechanical fit only / assemble all 5 (D11) | accept | T&C 5.7 (Economic) says unassembled boards may have been through reflow and their pads tinned; whether this applies to a Standard order is NOT_VERIFIED. | accept | do not plan to hand-assemble the spares. |

## 17. Open risks and what is NOT_VERIFIED

- Nothing was built or measured: every power-block number is a datasheet calculation (E).
- JLC's acceptance of this board is NOT_VERIFIED: 27 part bodies are closer than 2.5 mm to the edge (U7 0.15 mm, J15 0.29 mm). Standard PCBA with rails is the plan's answer, but only JLC's DFM result and quote after the user's upload can confirm it. Nothing may be paid before that.
- 'JLC solders the 6 THT lines' is NOT_VERIFIED until the quote shows it (review S2). The SMD fallback parts would need new footprints and the user's yes.
- TPS61023 valley current limit is specified only at 3.6 V input; at 3.0 V the design needs 1.50 A (56 % of the 2.7 A minimum) for 1.5 x peak (E).
- Cut-off hysteresis against the voltage rebound: 0.426 V minimum against 0.398 V at peak load and the estimated 0.275 Ohm - margin 28 mV. Above 0.29 Ohm the supply can cycle on and off at the end of discharge under peak load. The source resistance is measured on the first board.
- The cut-off acts on the loaded voltage (filter 16.3 ms): with a sustained peak load it can trip when the resting pack voltage is below 3.62 V (worst-case threshold; 3.38 V at typical load) (E).
- Switch-on threshold up to 3.73 V: a pack resting below that may not start the board.
- Start-up into 226.9 uF with the 1.9 ms power-on reset and the sense filter is NOT_VERIFIED (scope capture on the first board).
- +3V3 headroom: +5V minimum 4.844 V minus the AMS1117 dropout of 1.3 V max leaves 0.244 V above 3.3 V (E); the minimum during Wi-Fi transmission is NOT_VERIFIED.
- Drain after cut-off: 6.7 uA typ / 8.0 uA max from the new block plus 34..62 uA through the existing R15 / R16 divider: the user must switch off, the cut-off is not a storage switch.
- 22 uF 1206 capacitors: DC-bias derating at 5 V not read from a Samsung curve (NOT_VERIFIED); the effective output capacitance must stay >= 4 uF - two 22 uF plus 5 x 10 uF on the rail make this uncritical (E).
- Placement of the power block is NOT_VERIFIED: about 95 mm2 needed against 45.6 mm2 freed, with U6 / C43 / C44 / C45 / C41 / C42 to be re-placed at 87.6 % occupancy. If it does not fit, the plan has no approved fallback (moving a connector breaks invariant (b)).
- U8 uses a generic connector symbol: ERC gives no evidence for its pins; the pad-to-net table check replaces it.
- CPL rotations for SOT-563, SOT-23-6, SOT-23, SOT-223, tantalums, ESP-12F, JST-GH and all THT parts are NOT_VERIFIED (sources conflict); a reversed tantalum is a fire hazard -> 'Confirm Parts Placement' and inspect before power.
- BOM / CPL file format against JLC's parser is NOT_VERIFIED until the upload preview shows every line matched.
- Molex 22035025 (J14): hole 0.85 mm and tail length not verified against a drawing; no 3D model; mating with the pack's plug assumed (A).
- THT lead protrusion: J15 / J17 1.8 mm nominal against the 1.5 mm foam rule (user flush-cuts 4 leads); the socket alternate C55629861 has 3.2 mm tails (about 1.6 mm protrusion).
- Stock moves: thin lines today: C22436147 (543 available, 40 needed). C161692 is falling (1877 / 1090). U9 has no second source with the same limits. The order-day gate is hard.
- Reflow ratings of C47 (C2833309), ESP-12F and the JST / header parts were not found in a document: NOT_VERIFIED; Standard solders at 240 C peak, Economic at 255 C.
- ESP-12F RF behaviour assumed equal to the ESP-12E in the same keep-out (A); the ESP firmware image is the same ESP8266 / 4 MB layout (A).
- BMP280 module header type (right-angle / straight) and the pin order of all three sensor modules are A (check the silkscreen).
- OpenLog fit over J5 is NOT_VERIFIED (stack height, orientation, microSD card overhang); module outlines are A until measured; the NEO-7M outline in hand is the NEO-6M one.
- Programming access: the pogo-adapter contact on bare holes is NOT_VERIFIED; whether option B (JLC-fitted headers outside the band) can be placed is NOT_VERIFIED.
- Hand soldering U1..U4: 14 / 15 / 20 / 14 fitted parts within 1.0 mm of the pad rings (M); difficulty is E.
- AVCC choke C1046 is rated 15 mA (1.15 Ohm): the AVCC current of the ATmega128A was not read against it (NOT_VERIFIED, review S9).
- Fees are estimates from the fee table; whether the Standard-only handling / fixture / packing fees apply, the ENIG surcharge, the bake fee and the depanelling fee are NOT_VERIFIED.
- Render tool chain: the d2 scene was never executed; the interpreter with CadQuery / PyVista is the MAIN checkout's venv (import test only, M).
- Rev 2c documents contain inconsistencies that must not be copied: board.layers = 2 in the spec vs 4 delivered, '2 x 22 pF' vs 27 pF fitted, stale counts in SPEC_ko.md, 160 vs 152 populated, MANIFEST paths into the MAIN checkout, populate: true for the 8 unpopulated programming headers.

## 18. Review findings

Reviewers: electrical (E-1..E-11), sourcing (S1..S10), order / mechanics (OM-1..OM-11). Every finding was reproduced from the cited file, datasheet or tool output before it was settled. 0 findings were refuted. Input plan sha256 `caa709352ce9929f01ed7d8ae636884c27f53bcf252d6bf9328d100d4ae0af93`.

| Id | Severity | Verdict | What changed |
|---|---|---|---|
| E-1 | major | holds - fixed | U9 TLV3011AIDBVR (C2870632) -> TLV3012BIDBVR (C20345924, limits of the current datasheet SBOS300C: reference 1.223..1.260 V, offset +-9 mV); R65 / R66 / R67 = 470k / 270k / 3.3M. Worst case over 256 corners (M C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/final/uvlo_tlv3012b.py): on 3.44..3.73 V (nominal 3.59 V), off 3.00..3.25 V (nominal 3.13 V). |
| E-2 | major | holds - fixed | The comparator is now push-pull and drives EN directly (tens of ohms instead of 701 kOhm); R68 1M is a pull-down that keeps EN low while U9 is unpowered. EN input current no longer sets the level. |
| E-3 | minor | holds - fixed | L5 -> FXL0420-1R0-MA1 (C53426685, Isat 6.5 A / 9.0 A, same land pattern, stock 1065); C167203 stays as verified drop-in alternate. |
| E-4 | minor | holds - fixed | +5V peak 683 -> 783 mA (ESP supply requirement > 500 mA, F C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/datasheets/C82891_text/C82891.txt:122). Capability 1.815 A at 3.0 V = 2.32 x peak, 1.55 x (1.5 x peak). |
| E-5 | minor | holds - check added | Bring-up record now carries the measured source resistance with an accept limit of 0.29 Ohm (hysteresis minimum 0.426 V / input current at peak load). At the estimated 0.275 Ohm the margin is only 28 mV (E). |
| E-6 | minor | holds - fixed | One supply mode for programming: board powered from the logic pack (or 4.0 V bench supply at J14), USBasp target-power jumper open, never 5 V from both. Other modes are NOT_VERIFIED and not described. |
| E-7 | minor | holds - was already disclosed; check sharpened | The plan states that ERC gives no evidence for U8. The pad-to-net table is checked twice: on the exported netlist and on the PCB pads. |
| E-8 | minor | holds - fixed | TLV3012B has a power-on reset (output low for 1.9 ms, F C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/verify/electrical_work/tlv3011.txt:479); C61 is removed; the sense filter (16.3 ms) is slower than the reset. Start into 226.9 uF stays NOT_VERIFIED until a scope capture on the first board. |
| E-9 | minor | holds - check added | +5V minimum 4.844 V minus AMS1117 dropout 1.3 V max leaves 0.244 V above 3.3 V (E). The bring-up record carries the +5V and +3V3 minimum during Wi-Fi transmission. |
| E-10 | minor | holds - check added | J14 Molex 22035025: hole 0.85 mm and tail length stay NOT_VERIFIED; a drawing check against the KiCad footprint is a named implementation check, with J15 / J17 / J7 / U7 re-read. |
| E-11 | minor | holds - fixed | L5 height 2.0 mm (datasheet maximum) is used for the height table; ESP-12F pad numbering note (KiCad 22 pads / datasheet 16 + 6) added; comparator citations point at SBOS300C. |
| S1 | major | holds - moved to a user decision | 'press-in' and 'not a hand-solder-later item' are deleted. Decision D6 states the real work (36 joints or a pogo adapter, contact NOT_VERIFIED) and option B (JLC-fitted headers outside the band, three orderable parts). Shunts are listed only with soldered headers. OpenLog header -> D5. |
| S2 | major | holds - downgraded to NOT_VERIFIED, gate and fallback parts added | 'JLC solders the 6 THT lines' is NOT_VERIFIED until the user sees it in JLC's quote (first runbook step, before paying). Reflow fallback parts fetched today and orderable: C46061676 (1x3), C46061675 (1x2), C46061677 (1x4), C160352 (JST-PH SMD); their footprints are not chosen yet. |
| S3 | major | holds - fixed (gate + alternates); change tagged optional | C22436147 stays (drawing matches the footprint; 553 / 543 at 21:41); hard order-day gate with ordered alternates C55629861 then C54876735; pre-order by the user named; D3 / D12. |
| S4 | minor | holds - fixed | Each loose item names where to buy it (LCSC by the same C-number or a domestic shop); stock of loose items is NOT_VERIFIED. |
| S5 | minor | holds - fixed | C161692 1877 / 1090 at 21:41 (falling); alternates C52037397 and C2829262 named, to be used only after a footprint check. |
| S6 | minor | holds - fixed | Alternates table added: L5 C167203, U8 other reel C1852149 (stock 550: thin), U9 none with the same limits (TLV3011B not stocked) - stated. |
| S7 | minor | holds - fixed | Every BOM line shows its snapshot time; the newest record per code is used; the order-day re-fetch is a hard gate (fails, does not warn). |
| S8 | minor | holds - check added | Socket seating height is NOT_VERIFIED; it is taken from the Hong Cheng drawing in the OpenLog / BMP280 fit check. |
| S9 | minor | holds - stays NOT_VERIFIED, now stated | AVCC choke C1046 (15 mA, 1.15 Ohm): AVCC current NOT_VERIFIED, to be read from the ATmega128A datasheet in the implementation; C2286 is a red LED; brand '--' noted. |
| S10 | minor | holds - fixed | Preferred flag of C5364642 and C32713263: False (reviewer search); both are plain Extended. |
| OM-1 | major | holds - fixed + moved to a user decision | Default order type Economic -> Standard with JLC edge rails (D9), full price including the possible handling / fixture / packing fees; edge-distance table is a named check; acceptance stays NOT_VERIFIED until the user sees JLC's DFM result. |
| OM-2 | major | holds - moved to a user decision | D4 carries the measured access numbers, a recommendation (JLC fits C6510) separate from the default (user solders, as requested); access table and paste-aperture check added; surface finish in D16. |
| OM-3 | major | holds - moved to a user decision | Same change as S1: D5 (OpenLog header) and D6 (programming access) list the user work truthfully. |
| OM-4 | major | holds - check added + moved to a user decision | OpenLog fit check with card-overhang keep-out, decided by calliper numbers and the socket drawing, not by renders; fallback 6-wire lead (D5); render caption c3 no longer shows the fit as given. |
| OM-5 | major | holds - fixed | H1 / H2 are dropped from the default design. Standard: holes on JLC's rails. Economic: 'Added by JLCPCB' unless a legal-position search after routing proves two positions (D10). |
| OM-6 | major | holds - fixed (interpreter measured) + first-step spike | Interpreter named and tested (M: B:/0)AI_Cad/CanSat_EDF_Rocket/.venv/Scripts/python.exe -B imports cadquery 2.8.0, pyvista 0.49.1; run on worktree scripts, nothing written to the MAIN checkout); the d2 scene is the first implementation step; NEO-7M dimensions labelled A; STEP timestamps excluded from the determinism check. |
| OM-7 | minor | holds - check added | BOM / CPL format check against JLC's sample files added; stays NOT_VERIFIED until JLC's upload preview parses the files; 'Confirm Parts Placement' is ordered. |
| OM-8 | minor | holds - fixed | Order-page fields added (D16); Standard minimum 70 x 70 mm is met by 76 x 70 mm; Standard total now shown with and without the handling / fixture / packing fees. |
| OM-9 | minor | holds - check added | Reflow rating table (240 C Standard / 255 C Economic against C47, ESP-12F, JST parts) is a named check; ratings not found in a datasheet stay NOT_VERIFIED; the MSL claim for the LEDs is withdrawn (not found in a document). |
| OM-10 | minor | holds - fixed | 17 decisions, each with question / options / recommendation / reason / default / consequence; the six missing ones are D4, D5, D6, D9, D16, D17. |
| OM-11 | minor | holds - fixed | Every change carries a tag: forced / keeps a rev 2c function / sourcing / optional; the optional ones are revertible in D2, D3, D12. |

## 19. 사용자에게 보고할 것

### 보드에서 바뀌는 것과 이유

- 5 V 전원 모듈(Pololu, 디바이스마트 부품)을 뺐습니다. JLC 부품으로 만든 승압 회로(U8, L5 등 13개)를 보드에 직접 올립니다. 요청하신 대로 JLC가 전부 실장할 수 있게 하기 위해서입니다.
- 배터리 저전압 차단 회로를 넣었습니다. 켜짐 3.44~3.73 V, 꺼짐 3.00~3.25 V입니다(계산값, 실측 전). 예전 모듈이 하던 기능을 유지하려는 것입니다. 검토에서 비교기 사양이 옛 데이터시트 값이라는 지적이 나와 부품을 TLV3012B로 바꾸고 저항값을 다시 계산했습니다.
- Wi-Fi 모듈은 ESP-12E 대신 핀이 같은 ESP-12F를 씁니다. ESP-12E는 재고가 적고 주문 방식이 제한됩니다.
- 서보·스위치·부저 등 스루홀 커넥터 12개도 JLC가 납땜합니다. 그중 9개는 JLC 재고가 더 많은 부품 번호로 바꿨습니다(D3, D12에서 되돌릴 수 있음).
- MCU 4개의 핀 배치와 펌웨어, 보드 크기(76 x 70 mm), 커넥터 위치는 그대로입니다.
- 보드는 아직 만들어진 적이 없습니다. 전원 회로 수치는 모두 데이터시트 계산이며 실측이 아닙니다.

### 사용자가 직접 장착하는 것

- ATmega128A-AU 4개: 직접 납땜(기본). 주변 부품이 1 mm 안에 14~20개씩 있어 작업이 까다롭습니다. JLC가 실장하는 파일도 같이 만듭니다(권장).
- GY-521(IMU): 모듈의 8핀을 보드 구멍에 직접 납땜합니다.
- BMP280: 소켓 J2에 꽂습니다.
- NEO-7M(GPS): 기체 윗뚜껑에 두고 4선 케이블로 J7에 꽂습니다.
- OpenLog: 모듈에 ㄱ자 6핀 헤더를 납땜(6곳)한 뒤 소켓 J5에 꽂습니다. 보드 위 높이가 맞는지는 아직 확인되지 않았습니다. 버니어로 잰 뒤 판단합니다.
- 프로그래밍 단자(2x3 5곳, 1x2 3곳): 기본은 빈 구멍입니다. 헤더 36핀을 직접 납땜하거나 포고핀 어댑터를 써야 합니다. 'JLC가 헤더까지 실장'하는 안(B)은 배치 가능 여부를 구현 단계에서 확인해 보고합니다.
- 스위치·부저 커넥터 J15/J17: 뒷면으로 나온 다리 4개를 1.5 mm 이하로 잘라야 합니다.
- Standard 주문이면 보드 가장자리의 5 mm 레일을 떼어내야 합니다.

### 사용자가 따로 사는 것

- ATmega128A-AU 4개(JLC 실장을 택하면 불필요)
- GY-521, BMP280(TYE-BMP29), NEO-7M(SZH-NT07) 모듈과 GPS용 JST-GH 4선 케이블
- OpenLog, microSD 카드, ㄱ자 6핀 헤더 1개
- 로직 배터리(1S 리튬폴리머 4200 mAh), 전원 스위치와 PH 2핀 케이블, 암 스위치와 2핀 케이블, 부저와 PH 2핀 케이블
- 프로그래밍용: USBasp, 3.3 V USB-시리얼 어댑터, (헤더를 납땜할 경우) 2x3 헤더 5개·1x2 헤더 3개·점퍼 3개 또는 포고핀 어댑터
- 3 mm 폼테이프, 케이블 타이 1개
- 이 낱개 부품들의 오늘 재고와 가격은 확인하지 않았습니다(NOT_VERIFIED). JLC는 낱개 부품을 보내주지 않으므로 LCSC나 국내 쇼핑몰에서 사야 합니다.

### JLC 주문 방식

- JLC 주문 방식: Standard PCBA, 윗면 단면 실장, 4층 1.6 mm, PCB 5장 중 2장 실장(기본).
- 이유: 부품 27개가 보드 끝에서 2.5 mm 안에 있고, 그중 2개(U7 0.15 mm, J15 0.29 mm)는 Economic 방식의 권장 거리 0.3 mm보다 가깝습니다. 커넥터 위치는 기체에 맞춰 고정이라 옮길 수 없습니다. Standard는 JLC가 가장자리 레일을 붙여 줍니다.
- 예상 비용(PCB·배송·세금 제외, 추정): 2장 실장 약 112 USD, 추가 수수료가 붙으면 최대 약 144 USD. Economic은 약 70 USD이지만 JLC가 거절할 수 있습니다.
- 올릴 파일: 거버 zip, jlcpcb_bom.csv, jlcpcb_cpl.csv. MCU까지 맡기면 *_with_mcu 파일 쌍.
- BOM 40줄 모두 오늘 JLC 조립 라이브러리에서 재고와 주문 가능 여부를 확인했습니다. 재고가 적은 줄: 소켓 C22436147(543개). 주문하는 날 다시 확인해야 합니다.
- JLC가 이 보드를 그대로 받아주는지(가장자리 부품, 스루홀 납땜, BOM/CPL 형식)는 아직 확인되지 않았습니다. 파일을 올린 뒤 JLC의 DFM 결과와 견적을 보고 나서 결제하십시오. 도구는 주문하지 않습니다.

### 사용자가 정할 것 (괄호 안은 계획이 쓰는 기본값)

- D1: 5 V 변환 회로 방식 (기본: A안 TPS61023 승압 + TLV3012B 차단)
- D2: 배터리 차단 전압 (기본: 켜짐 3.59 V / 꺼짐 3.13 V(공칭) 유지)
- D3: 요청과 무관하게 바꾼 부품 번호 3건을 유지할지 (기본: 유지(재고가 더 많거나 도면이 확인된 부품))
- D4: MCU를 누가 붙일지 (기본: 직접 납땜(요청대로). 권장은 JLC 실장)
- D5: OpenLog 장착 방법 (기본: 헤더를 납땜해 J5에 꽂기. 안 맞으면 6선 케이블)
- D6: 프로그래밍 단자 (기본: A안 빈 구멍(직접 납땜 36핀 또는 포고핀). B안은 가능 여부 보고 후 결정)
- D7: J15/J17 커넥터 (기본: 스루홀, 다리 4개 직접 절단)
- D8: 레벨시프터 FET (기본: C40912 유지)
- D9: 주문 방식 (기본: Standard(권장과 같음))
- D10: 툴링 홀 (기본: 설계에 넣지 않음. JLC 레일에 생김)
- D11: 수량 (기본: PCB 5장, 실장 2장)
- D12: 모듈 소켓 J2/J5 (기본: C22436147, 주문일 재고 부족 시 C55629861, 그다음 C54876735)
- D13: 배터리 전압 분압 저항 (기본: 그대로. 차단 후에도 약 40 uA가 흐르므로 스위치를 꺼야 함)
- D14: 흡습 부품 베이킹 요청 (기본: 요청함(수수료 미확인))
- D15: 모듈 치수 (기본: 가정값 사용, 그림에 '가정' 표시. 실측을 권장)
- D16: 표면처리·색상 등 (기본: ENIG, 녹색, JLC 기본값(추가 요금 미확인))
- D17: 실장하지 않은 여분 보드 (기본: 기구 확인용으로만 사용)

## 20. Consistency check

Command: `cd /b/Claude && .venv/Scripts/python.exe -X utf8 C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/final/check_final.py`

Result file: `C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/final/check_final_result.json` - overall **PASS** for plan sha256 `415d1e20a0903103d6bb4b48253ffc7054cfd3d2371c008706b906fe5c1eda15`.

| Check | Result | Problems |
|---|---|---|
| C1_one_bom_line_per_jlc_part | PASS | 0 |
| C2_every_bom_line_is_a_jlc_assembly_part_orderable_today | PASS | 0 |
| C3_mcu_pin_map_equals_rev2c | PASS | 0 |
| C4_library_symbols_footprints_on_disk | PASS | 0 |
| C5_nets_consistent | PASS | 0 |
| C6_u8_u9_pad_net_table | PASS | 0 |
| C7_no_devicemart_or_lcsc_only_part_on_the_board | PASS | 0 |
| C8_review_findings_and_user_report_complete | PASS | 0 |

The check proves that the plan is consistent with the rev 2c spec, the on-disk KiCad libraries and today's JLC records. It is not ERC, DRC or a JLC DFM result.

## 21. Implementation files

- V/history/rev2c/ (archive of the rev 2c spec / kicad / out before anything is regenerated)
- V/scripts/build_spec.py (fit attribute, J6 removed, power block, bom_key, PKG_EXPECT, new nets; rev 2c value / role texts that still say ESP-12E or Pololu updated) -> V/spec/carrier_spec.json, V/spec/SPEC_ko.md
- V/scripts/probe_library.py -> V/spec/library_probe.json
- V/scripts/jlc_sources.py (new: loads SCRATCH/jlc_cache records, I7 gate) -> V/spec/parts_sources.json
- V/scripts/common.py, build_schematic.py -> V/kicad/vtol_carrier.kicad_sch (+ PWR_FLAG on VBAT_LOGIC_SW)
- V/scripts/place.py (ANCHORS / MODULES / FP_HEIGHT / TARGET_HINT / PWR_FIRST) -> V/kicad/placement.json
- V/scripts/build_pcb.py, fanout.py, route.py, close_leftovers.py, silk.py -> V/kicad/vtol_carrier.kicad_pcb
- V/scripts/checks_exports.py (BOM + CPL filtered by fit, no --smd-only, THT rotations, rotation override table, optional MCU pair) -> V/out/jlcpcb_bom.csv, jlcpcb_cpl.csv, jlcpcb_bom_with_mcu.csv, jlcpcb_cpl_with_mcu.csv, gerbers zip, drill, ERC / DRC reports, STEP
- V/scripts/board_metrics.py (RA_PLUGS, VERT_PLUGS, supply_pads, MODULE_TOP_A, converter caps) -> metrics
- V/scripts/make_module_models.py (new, CadQuery) -> V/kicad/3d/*.step; V/scripts/render_rev3.py (new) -> V/out/render/*.png; V/scripts/make_presentation.py -> V/out/presentation/*.png
- V/scripts/make_all.py, determinism.py -> V/out/MANIFEST.json
- V/README_ko.md, V/out/ORDER_ko.md (how to order, what to buy separately, what to do after delivery), V/out/off_board_purchase.csv
- pre-flight: cad_envelope.py:13 and route.py:58 point at the MAIN checkout / a fixed JAR - repoint to the worktree / an env override before running anything
- V/scripts/edge_distance.py (new) -> V/out/edge_distance.csv (body / pad distance to the outline; rail sides)
- V/scripts/jlc_gate.py (new; order-day hard gate over BOM lines and alternates, prints the snapshot time per line) -> V/out/jlc_gate_report.json
- V/scripts/fit_checks.py (new; OpenLog / BMP280 keep-outs, MCU hand-solder access table, THT protrusion, reflow rating table) -> V/out/fit_checks.json
- V/out/ORDER_ko.md additionally carries the order-page field table, the change-scope table (tag per change) and the 'before paying' gate (DFM result, 40 matched lines, 6 THT lines hand-soldered)

Scripts of this step: `C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/final/finalize_plan.py`, `C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/final/final_texts.py`, `C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/final/check_final.py`, `C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/final/make_md_final.py`, `C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/final/live_fetch.py`, `C:/Users/yc012/AppData/Local/Temp/claude/133067549dd2ec4c/bed4b475-b8b1-46b7-ab64-c3b11e27401e/scratchpad/pcb_rev3/final/uvlo_tlv3012b.py`
