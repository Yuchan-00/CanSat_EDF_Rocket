# Three-chip co-simulation (v0.6.3 NAV + FCU + COM)

The NAV, the FCU and the COM run together, instruction by instruction, in one libsimavr process (PlatformIO's
`tool-simavr`, the same simulator `run_simbench.py` / `run_nav_simbench.py` / `run_com_simbench.py` use), wired as on
the board:

```
NAV (env nav_cosim)                FCU (env fcu_cosim)                       COM (env com_cosim = lora_uart,
                                                                                  com_cosim_sx127x)
  PB0 SS   ------- nlink ------->    PB0 SS (external level)
  SPI out  ------- MOSI -------->    SPI in (SPIF, SPI_STC)
  SPI in   <------ MISO ---------    shift register (modelled by the harness)
                                     USART1 TX  ------- ilink ------------->  USART0 RX
                                     USART1 RX  <------ ilink --------------  USART0 TX
  USART0 TX -> harness: report      USART0 TX -> harness: report lines        USART1 TX -> harness: TELEM (lora_uart)
  PORTA     -> harness: tick marker  PORTA     -> harness: tick marker         RAM g_cosim_report -> harness
                                     OCRnxH    -> harness: PWM register writes
```

Run everything (builds, probes, harness, all scenarios, report):

```
.venv/Scripts/python.exe firmware/atmega128_fc/tools/run_cosim.py            # --sweep 40 --jobs 12 by default
.venv/Scripts/python.exe firmware/atmega128_fc/tools/run_cosim.py --only faults
```

Output: `firmware/atmega128_fc/host_test/build/cosim_report.json` (PASS / FAIL per criterion, every figure) and one event
file per run in `tools/cosim/build/runs/` (record format in the header of `cosim.c`).  About 8 minutes on 16 threads
(102 runs of 16 simulated seconds, ~45 s each).

## Files

| file | what |
|---|---|
| `cosim.c` | the harness: three ATmega128 cores, lockstep, the nlink (SPI) and ilink (USART) wires, faults, observation |
| `elf_stubs.c` | libelf symbols `sim_elf.o` references; every firmware is loaded from Intel HEX, so they are never called |
| `isr_cost_avr.c`, `isr_cost.c` | probe: cycles simavr charges per interrupt (7; the chip needs 11) and with the harness's correction (11) |
| `spi_probe_avr.c`, `spi_probe.c` | probe: simavr's SPI as master and as slave (byte time, what the slave sends, SS) |
| `../run_cosim.py` | builds, layout probes, probes, scenarios, analysis, criteria, report |

## Timing model (what the numbers mean)

* **Lockstep.** The core that is behind runs the next instruction (ties: FCU, NAV, COM); a no-op cycle timer every 16
  cycles bounds how far a sleeping core can run ahead, so a USART byte reaches the other chip at most 16 cycles (1 us)
  late.  The SPI bytes are scheduled exactly (below).
* **Interrupt response.** simavr charges an interrupt the vector JMP and the RETI but not the chip's 4-cycle interrupt
  response (`isr_cost`: 7 cycles against 11).  The harness adds the 4 cycles to the core's clock whenever an interrupt
  starts running (`isr_resp=4`), so every core - its timers, its own Timer1 figures, the nlink slave's reply-byte
  latency - runs at the chip's interrupt cost; `run_cosim.py` checks with `isr_cost` that the corrected charge is 11.
  (v0.6 phase 3 added the 4 cycles in the analysis instead; the in-core correction also makes the SPI race exact.)
  Not modelled: the 4 extra cycles of an interrupt that wakes a sleeping core (NAV / COM tick start, 0.25 us).
* **nlink (SPI), simavr as measured by `spi_probe`.** The master raises its SPI output 8 x the SCK divider cycles after
  the SPDR write (64 cycles at F_CPU / 8) with the byte it wrote, and takes its SPI input as the received byte.  A
  slave receives whatever its SS pin says, and answers with the byte it just received (its single SPDR byte is
  overwritten), never with the byte it loaded.  The harness therefore models the FCU's slave as the chip has it:
  - a byte starts when the NAV writes SPDR as master; it reaches the FCU only if the FCU's PB0 (SS) is low and its SPI
    is on (else the NAV reads its MISO pull-up, 0xFF);
  - MISO = the FCU's shift register at that instant: its last SPDR write, or - if it wrote none since the previous byte
    ended - the byte received last (the echo of a late slave interrupt); an FCU SPDR write while a byte shifts is a
    write collision (ignored, counted);
  - an FCU cycle timer delivers the NAV's byte to the FCU's SPI input exactly at the byte's end (SPIF + SPI_STC);
  - the FCU's PB0 follows the NAV's PB0 as an external level (`AVR_IOCTL_IOPORT_SET_EXTERNAL`; without it simavr
    re-applies the FCU's own pull-up on every PORTB write); a NAV that does not drive PB0 (reset, boot) leaves it high;
    SS rising aborts a byte in progress at the slave; the shift register keeps its content across SS (assumption);
  - the reply-byte deadline is the master's SPDR write; on the chip the slave's MSB must be on MISO before the first
    SCK edge, ~4 cycles later - the model is conservative by that much.
  Every exchange is recorded (X): the bytes each side sent and received, late reply bytes, write collisions, the
  smallest margin of a reply write and the FCU interrupt that was running at that byte's SPIF.
* **USART.** As in v0.6 phase 3: simavr 1.8 counts a parity bit in every frame and computes the byte time only when
  UBRRnL is written; with `wire_bits=10` the harness keeps every USART of every chip at the byte time its registers
  configure (record W).
* **Tick markers.** `fcu_cosim` writes `(tick << 1) | 1` to PORTA when its tick starts (the frame was noticed, or the
  timeout passed) and `tick << 1` at the end of the time-critical part; `nav_cosim` writes them after its Timer2 wake-up
  and right after SS rises.  The FCU's time-critical part is measured from the frame's last byte (the exchange record)
  to its marker down; the NAV's from its Timer2 compare to SS high.
* **What the co-simulation builds do not exercise**, added by `run_cosim.py` analytically (and said so in the report):
  the NAV's I2C traffic (synthetic sensors: one MS5611 bus script, 11 TWI interrupts, in MS5611 phases 0 and D1; the
  MPU-6050 burst runs in the IMU lead before the tick, outside the measured windows); the IMU lead itself (simavr's
  Timer0 ignores timer.c's TCNT0 preload - measured: its compare fires 7 cycles after Timer2's instead of 496 us before
  - so the lead is placed at FC_IMU_LEAD_COUNTS x FC_T2_PRESCALER before the Timer2 compare, and the Timer0 interrupt and
  the NAV's simbench-only console UDRE are taken out of the NAV's figures); the FCU's flight records on the log port (a
  continuously busy log port's UDRE interrupts are added).  The FCU simbench's 1 Hz report lines (written after the
  background) make ~10 ticks per run start late; those get the run's worst normal detection time instead
  (`simbench_late`).
* **Resets.** simavr's `avr_reset()` crashes in the watchdog module once the firmware has enabled the watchdog; the
  harness resets a core itself the same way minus that module, holds it, and sets MCUCSR.EXTRF on release (COM reset,
  NAV hold / pulse).  The NAV pulse is held for 1 ms + the ATmega128's reset time-out (16K CK + 65 ms typical for
  CKSEL 1111 / SUT 11; the datasheet gives no min / max).
* **Synthetic ground.** The COM firmware carries its own ground (src/com/com_simbench.c).  After a COM reset it starts
  again from SEQ 0, so the FCU treats its NONE polls as repeats; a real ground keeps its SEQ.

## Building the harness by hand

```
SIM=~/.platformio/packages/tool-simavr
python -m ziglang cc -target x86_64-windows-gnu -std=c99 -O2 -I $SIM/include/simavr -I firmware/atmega128_fc/src/com \
    -o cosim.exe firmware/atmega128_fc/tools/cosim/cosim.c firmware/atmega128_fc/tools/cosim/elf_stubs.c \
    $SIM/lib/libsimavr.a -lws2_32
```

The `tool-simavr` package installed on the development PC is the Windows build (its package.json lists
windows_amd64 / windows_arm64).  On another system set `SIMAVR_DIR` to a prefix holding `include/simavr/*.h` and
`lib/libsimavr.a` of simavr 1.8 (not tried yet; if that library was built with libelf, link `-lelf` instead of
`elf_stubs.c`).  `run_cosim.py` re-runs `isr_cost` and `spi_probe` on every run and stops if simavr's SPI does not
behave as the harness assumes.

# Four-core closed-loop co-simulation (v0.9: NAV + GNS + FCU + COM + plant)

`cosim4.c` is the v0.9 acceptance co-simulation (SPEC_firmware_v09 WP-S / S1): the three chips above plus the GNS on the
glink (NAV USART1 <-> GNS USART0), and a plant that closes the loop.  Adopted from the generated
`timing82/profile/cosim4/cosim4.c` (this harness + the GNS core); its header lists every record and argument.

```
.venv/Scripts/python.exe firmware/atmega128_fc/tools/run_cosim4.py                 # every scenario, 2 at a time
.venv/Scripts/python.exe firmware/atmega128_fc/tools/run_cosim4.py --only cl_normal_fcu,cl_nav_pulse
```

Output: `firmware/atmega128_fc/host_test/build/cosim4_report.json` (the co-simulation rows of SPEC section 7 judged
PASS / FAIL with their numbers, every run's analysis, the closed loop's validation, the job lengths for the background
reserves) and one event file per run in `tools/cosim/build/runs4/`.  A 52 s closed-loop run takes ~100 s on one core.

| file | what |
|---|---|
| `cosim4.c` | the harness: four ATmega128 cores, lockstep, nlink / ilink / glink wires, faults, observation, the simavr corrections below |
| `cosim4.h` | what the harness shares with the plant and the sensor models |
| `plant4.c` | the closed-loop plant: the FCU's ESC / vane compare registers -> thrust (EDF lag), vane moments (the image's own mixer model), rigid-body attitude, height, horizontal motion with a constant disturbance, ground contact -> the sensors |
| `twi_models.c` | the NAV's MPU-6050 (0x68) and MS5611 (0x77) register models on its TWI (from timing82's navflight.c), fed by the plant; the TWI bus-time correction |
| `../run_cosim4.py` | builds (images, header probes, harness), scenarios, analysis, the section 7 rows, the report |

Builds (closed loop, 52 s, never flash): `nav_cosim_twi` (nav_cosim with the real TWI drivers: `NAV_SB_TWI`),
`gns_cosim_cl[_demo]` (its synthetic receiver reports the plant's antenna from the harness-written `g_cosim_gps`),
`fcu_cosim_cl[_demo|_nogps|_wired|_wired_buffer]`, `com_cosim_cl` (its synthetic ground flies the field profile:
`COM_SB_FIELD`).  Open loop (15 s, the P1 runs): `nav_cosim`, `gns_cosim`, `fcu_cosim`, `com_cosim`.

The plant is the vehicle the FCU image's own `params_gen.h` describes (a host probe compiled with the env's -D flags
reads it): mass from u_hover and T_max, the mixer's forward model, `VEH_I*`.  It therefore cannot reveal a wrong vehicle
constant in the image (that is the crosscheck's per-configuration plant check, SPEC X1); it is a timing / sequencing
tool that flies the whole profile (take-off, CLIMB, HOLD tilted against a constant disturbance, DOWN / UP, LAND,
LANDED, SAFE) through the real flight code.  Not modelled: drag, ground effect, servo lag, the 50 Hz PWM frame hold,
the rotor's reaction torque, the MPU's DLPF, tether forces.

simavr corrections (v0.9, measured; details in the `cosim4.c` header):

* **Timer0 preload.** simavr restarts a timer from 0 when its clock switches on and drops the TCNT0 the firmware
  preloaded while it was stopped, so the NAV's IMU-lead compare came 7 cycles after the tick instead of 496 us before.
  The harness replays the preload through simavr's own TCNT0 handler when TCCR0 starts the clock: the lead is now
  simulated (measured t2 - t0 = 497.8 us; +1.8 us against the chip's 31 x 256 cycles, simavr's re-basing formula).
* **TWI bus time.** On this simavr a data byte is scheduled 360 cycles after its TWCR write (9 SCL periods at
  400 kHz = 22.5 us: right), but a START and the SLA byte after a first START get 0 cycles.  The harness moves those
  to 1 and 9 SCL periods (`twidbg=N` prints the delays it finds).
* **Watchdog reset.** A core whose own watchdog expires makes simavr swap `avr->run` for a callback whose
  `avr_reset()` crashes in the watchdog module's reset - the deterministic 0xC0000005 of SPEC section 2 row 8.  The
  harness detects the swap after the instruction and resets the core itself (as `core_hold_reset`), MCUCSR.WDRF set.
* **PWM high bytes.** simavr calls a registered I/O write hook instead of storing the byte; the harness's hook on the
  OCRnxH addresses (since v0.6.3, also in `cosim.c`) did not store it, so the FCU's compare registers held only their
  low bytes.  Both harnesses now store it (timing figures unchanged: they use the write's moment).
