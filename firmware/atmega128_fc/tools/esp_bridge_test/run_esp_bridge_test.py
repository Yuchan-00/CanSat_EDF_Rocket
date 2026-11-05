#!/usr/bin/env python3
"""The COM's ESP bridge backend (src/hal/esp_bridge.c, 2026-11-06) in simavr: ``esp_bridge_avr.c`` + the backend built with
PlatformIO's avr-gcc and the -D flags of the env ``com_esp_bridge``, run by ``esp_bridge_sim.c`` (zig cc + PlatformIO's
libsimavr.a, as tools/adc_chain_test) playing the ESP-12E's side of the wire.

Invariants checked, two images:

* cold COM boot (no reset flag, as a power-on): the ESP is held in reset (PC6 driven low) from radio_pins_safe until
  radio_init releases it no earlier than ESP_RESET_HOLD_MS; PC6 is open-drain - released to an input, never driven high
  (review F1: a manual RST-GND short cannot short a driven pin); every byte that reaches USART1 RX inside the boot window
  (ESP_BOOT_TICKS COM ticks after the release - the ESP8266's boot log) is dropped and counted, none reaches radio_read;
  a burst that straddles the window's end loses only its head (a prefix), its tail and the frame after it reach
  radio_read unchanged and in order (review F5); a reply goes out on USART1 at once (no busy line), byte for byte, at
  ESP_UART_BAUD (simavr times the USART from UBRR); a second reply while one is going out is refused; no RX error or
  overflow.
* warm COM boot (MCUCSR = WDRF, a COM watchdog reset; review F2): PC6 is never driven (the ESP keeps running), radio_init
  does not wait the hold time, no boot window - every injected byte reaches radio_read in order.

simavr's USART input takes whole bytes, so a 74880-baud framing error (FE1) cannot be injected (A).

    python firmware/atmega128_fc/tools/esp_bridge_test/run_esp_bridge_test.py    # exit 0 pass, 1 fail, 77 tools missing

Output: ``firmware/atmega128_fc/host_test/build/esp_bridge_test/`` (elf / hex, the simulator, console_<variant>.txt).
"""
from __future__ import annotations

import hashlib
import os
import re
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
FW = os.path.normpath(os.path.join(HERE, "..", ".."))
SRC = os.path.join(FW, "src")
OUT = os.path.join(FW, "host_test", "build", "esp_bridge_test")
ENV = "com_esp_bridge"
GARBAGE_MS, FRAME_MS = 200.0, 600.0           # = esp_bridge_sim.c defaults: inside / after the boot window
GARBAGE = bytes([0x55, 0x00, 0xFF, 0xA5, 0x5A, 0x10, 0x72, 0x6F, 0x6D, 0xA5, 0xA5, 0x5A, 0x05, 0x0D, 0x0A, 0x80])
SPAN = bytes(0x30 + i for i in range(48))     # = esp_bridge_sim.c span[]
SPAN_LEAD_MS = 7.0                            # the straddling burst starts this long before the nominal window end
CMD = bytes([0xA5, 0x5A, 0x08, 0x5A, 0x10, 0x07, 0x01, 0x00, 0x00, 0x34, 0x12, 0xC3, 0x3C])   # = esp_bridge_sim.c cmd
TICK_US = 4000                                # COM_TICK_HZ 250
SIMAVR_BITS_PER_BYTE = 11                     # simavr's USART times an 8N1 byte as 11 bit times (measured by this test)
RESET_WATCHDOG = 0x08                         # hal/watchdog.h


def sh(cmd: list[str], **kw) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace", **kw)


def pio_home() -> str:
    return os.environ.get("PLATFORMIO_CORE_DIR") or os.path.join(os.path.expanduser("~"), ".platformio")


def tool(package: str, name: str) -> str | None:
    exe = os.path.join(pio_home(), "packages", package, "bin", name + (".exe" if os.name == "nt" else ""))
    return exe if os.path.exists(exe) else None


def simavr_dir() -> str | None:
    d = os.environ.get("SIMAVR_DIR") or os.path.join(pio_home(), "packages", "tool-simavr")
    return d if os.path.exists(os.path.join(d, "lib", "libsimavr.a")) else None


def have_zig() -> bool:
    return sh([sys.executable, "-m", "ziglang", "version"]).returncode == 0


def env_defines(env: str) -> list[str]:
    sys.path.insert(0, os.path.join(FW, "tools"))
    import run_cosim4  # noqa: E402  (the PlatformIO env's -D flags, ${sec.key} resolved)
    return run_cosim4.env_defines(FW, env)


def header_int(name: str) -> int:
    with open(os.path.join(SRC, "params_gen.h"), "r", encoding="utf-8") as f:
        m = re.search(rf"^#define\s+{name}\s+\(?(\d+)[uUlL]*", f.read(), re.M)
    if not m:
        raise SystemExit(f"{name} not in params_gen.h")
    return int(m.group(1))


def build_sim() -> str:
    sim = simavr_dir()
    exe = os.path.join(OUT, "esp_bridge_sim" + (".exe" if os.name == "nt" else ""))
    cmd = [sys.executable, "-m", "ziglang", "cc"] + (["-target", "x86_64-windows-gnu"] if os.name == "nt" else [])
    cmd += ["-std=c99", "-O2", "-Wall", "-Wextra", "-I", os.path.join(sim, "include", "simavr"), "-o", exe,
            os.path.join(HERE, "esp_bridge_sim.c"), os.path.join(FW, "tools", "cosim", "elf_stubs.c"),
            os.path.join(sim, "lib", "libsimavr.a")]
    cmd += ["-lws2_32"] if os.name == "nt" else ["-lm"]
    r = sh(cmd)
    if r.returncode != 0:
        raise RuntimeError("esp_bridge_sim build failed: " + (r.stdout + r.stderr)[-3000:])
    return exe


def build_image(gcc: str, objcopy: str, variant: str, reset_cause: int) -> tuple[str | None, str]:
    defs = env_defines(ENV)
    elf, hexf = os.path.join(OUT, f"{ENV}_{variant}.elf"), os.path.join(OUT, f"{ENV}_{variant}.hex")
    r = sh([gcc, "-mmcu=atmega128", "-Os", "-std=gnu99", "-DF_CPU=16000000UL", "-Wall", "-Wextra", "-Werror", *defs,
            f"-DEB_RESET_CAUSE={reset_cause}", "-I", SRC, os.path.join(HERE, "esp_bridge_avr.c"),
            os.path.join(SRC, "hal", "esp_bridge.c"), "-o", elf])
    if r.returncode != 0:
        return None, "avr-gcc failed: " + (r.stdout + r.stderr)[-2000:]
    r = sh([objcopy, "-O", "ihex", "-R", ".eeprom", elf, hexf])
    if r.returncode != 0:
        return None, "avr-objcopy failed: " + r.stderr[-1000:]
    return hexf, ""


def simulate(sim_exe: str, hexf: str, variant: str, span_ms: float) -> tuple[str, int]:
    s = sh([sim_exe, hexf, f"garbage_ms={GARBAGE_MS:g}", f"span_ms={span_ms:g}", f"frame_ms={FRAME_MS:g}", "max_ms=1300"],
           timeout=600)
    con = s.stdout + s.stderr
    with open(os.path.join(OUT, f"console_{variant}.txt"), "w", encoding="utf-8", newline="\n") as f:
        f.write(con)
    return con, s.returncode


def parse(con: str) -> dict:
    return {
        "pins": re.search(r"EB pins ddr6=(\d) port6=(\d)", con),
        "init": re.search(r"EB init t=(\d+) hold_us=(\d+)", con),
        "rx": [(int(t), int(b, 16)) for t, b in re.findall(r"RX t=(\d+) b=([0-9a-f]{2})", con)],
        "u1": [(float(t), int(b, 16)) for t, b in re.findall(r"U1 t_us=([\d.]+) b=([0-9a-f]{2})", con)],
        "pin_ev": [(float(t), int(v)) for t, v in re.findall(r"PIN t_us=([\d.]+) v=(\d)", con)],
        "drvhi": re.findall(r"DRVHI t_us=([\d.]+)", con),
        "inj": [float(t) for t in re.findall(r"INJ t_us=([\d.]+)", con)],
        "tx": re.search(r"TX t=(\d+) q=(\d) q2=(\d)", con),
        "busy": re.search(r"TX busy=(\d)", con),
        "stats": re.search(r"EB stats rx=(\d+) err=(\d+) ovf=(\d+) disc=(\d+) txf=(\d+) ok=(\d+)", con),
        "sim": re.search(r"SIM cycles=(\d+) state=(-?\d+)", con),
    }


def reply_checks(d: dict, con: str, rc: int, ubrr: int, baud: int) -> dict:
    u1, tx, busy, stats, sim = d["u1"], d["tx"], d["busy"], d["stats"], d["sim"]
    frame47 = bytes((0xA5 ^ (i * 37)) & 0xFF for i in range(47))
    bit_us = 16 * (ubrr + 1) / 16.0                     # one bit at UBRR, U2X 0, 16 MHz
    byte_us_sim = SIMAVR_BITS_PER_BYTE * bit_us         # simavr's USART byte time (measured: 11 bit times per 8N1 byte)
    gaps = [b[0] - a[0] for a, b in zip(u1, u1[1:])]
    return {
        "reply queued, second one refused while busy": bool(tx) and tx.group(2) == "1" and tx.group(3) == "0",
        "reply on USART1 byte for byte": bytes(b for _, b in u1) == frame47,
        f"USART1 bit time = UBRR {ubrr} at 16 MHz ({bit_us:.2f} us; {baud} baud within 0.2 %)":
            len(gaps) == 46 and abs(sum(gaps) / len(gaps) - byte_us_sim) < 0.5
            and abs(bit_us / (1e6 / baud) - 1.0) <= 0.002,
        "reply leaves at once (first byte out one byte time after radio_send: no busy-line wait)":
            bool(tx) and bool(u1) and u1[0][0] - int(tx.group(1)) < byte_us_sim + 50.0,
        "transmitter idle 200 ms later, one tx_frame": bool(busy) and busy.group(1) == "0" and bool(stats)
                                                     and stats.group(5) == "1" and stats.group(6) == "1",
        "simulation ended normally": "EB end" in con and bool(sim) and rc == 0,
    }, (round(sum(gaps) / len(gaps), 2) if gaps else None)


def run() -> dict:
    gcc, objcopy = tool("toolchain-atmelavr", "avr-gcc"), tool("toolchain-atmelavr", "avr-objcopy")
    if not gcc or not objcopy or not simavr_dir() or not have_zig():
        return {"status": "NOT RUN", "why": f"avr-gcc {gcc or 'missing'}, avr-objcopy {objcopy or 'missing'}, "
                                            f"libsimavr {simavr_dir() or 'missing'}, zig cc {'ok' if have_zig() else 'missing'}"}
    os.makedirs(OUT, exist_ok=True)
    try:
        sim_exe = build_sim()
    except RuntimeError as e:
        return {"status": "FAIL", "why": str(e)}
    baud, ubrr = header_int("ESP_UART_BAUD"), header_int("ESP_UART_UBRR")
    hold_ms, boot_ticks = header_int("ESP_RESET_HOLD_MS"), header_int("ESP_BOOT_TICKS")
    span_ms = hold_ms + boot_ticks * TICK_US / 1000.0 - SPAN_LEAD_MS
    out: dict = {"env": ENV, "variants": {}}
    checks: dict = {}

    # ---- cold COM boot -------------------------------------------------------------------------------------------
    hexf, why = build_image(gcc, objcopy, "cold", 0)
    if not hexf:
        return {"status": "FAIL", "why": why}
    con, rc = simulate(sim_exe, hexf, "cold", span_ms)
    d = parse(con)
    pins, init, rx, pin_ev, inj, stats = d["pins"], d["init"], d["rx"], d["pin_ev"], d["inj"], d["stats"]
    hold_t = next((t for t, v in pin_ev if v == 0), None)
    release_us = next((t for t, v in pin_ev if v == 1), None)
    # the window counts from the first radio_service after the release: it ends at a tick boundary between
    # release + (boot_ticks - 1) ticks and release + (boot_ticks + 1) ticks
    win_lo = (release_us or 0) + (boot_ticks - 1) * TICK_US
    win_hi = (release_us or 0) + (boot_ticks + 1) * TICK_US
    rx_bytes = bytes(b for _, b in rx)
    k_drop = int(stats.group(4)) - len(GARBAGE) if stats else -1        # span bytes dropped (its head)
    span_tail = SPAN[k_drop:] if 0 <= k_drop <= len(SPAN) else b"?"
    byte_us_sim = SIMAVR_BITS_PER_BYTE * 16 * (ubrr + 1) / 16.0
    c = {
        "radio_pins_safe holds the ESP in reset (PC6 output, low)": bool(pins) and pins.groups() == ("1", "0"),
        f"RST low once, released once, >= ESP_RESET_HOLD_MS = {hold_ms} ms":
            [v for _, v in pin_ev] == [0, 1] and bool(init) and int(init.group(2)) >= hold_ms * 1000
            and hold_t is not None and release_us is not None and release_us - hold_t >= hold_ms * 1000,
        "PC6 never driven high (open-drain: released to an input)": not d["drvhi"],
        "garbage inside the window, the straddling burst across its end, the frame after it":
            len(inj) == 3 and release_us is not None and release_us < inj[0] < win_lo - TICK_US
            and inj[1] < win_lo and inj[1] + len(SPAN) * byte_us_sim > win_hi and inj[2] > win_hi + TICK_US,
        f"every garbage byte ({len(GARBAGE)}) and the burst's head dropped and counted, at least one burst byte through":
            bool(stats) and 1 <= k_drop < len(SPAN),
        "radio_read returned the burst's tail then the frame, unchanged and in order (nothing from inside the window)":
            rx_bytes == span_tail + CMD and all(t > win_lo for t, _ in rx),
        "no RX error / overflow, rx_bytes = burst tail + frame": bool(stats) and stats.group(2) == "0"
                                                                  and stats.group(3) == "0"
                                                                  and int(stats.group(1)) == len(span_tail) + len(CMD),
    }
    rc_checks, byte_mean = reply_checks(d, con, rc, ubrr, baud)
    c.update(rc_checks)
    checks.update({f"cold: {k}": v for k, v in c.items()})
    with open(hexf, "rb") as f:
        sha = hashlib.sha256(f.read()).hexdigest()
    out["variants"]["cold"] = {"release_us": release_us, "window_end_us": [win_lo, win_hi], "injections_us": inj,
                               "span_dropped": k_drop, "rx_bytes": len(rx), "u1_bytes": len(d["u1"]),
                               "u1_byte_us_mean": byte_mean, "hex_sha256": sha}

    # ---- warm COM boot (watchdog reset) --------------------------------------------------------------------------
    hexf, why = build_image(gcc, objcopy, "warm", RESET_WATCHDOG)
    if not hexf:
        return {"status": "FAIL", "why": why}
    con, rc = simulate(sim_exe, hexf, "warm", span_ms)
    d = parse(con)
    pins, init, rx, pin_ev, stats = d["pins"], d["init"], d["rx"], d["pin_ev"], d["stats"]
    rx_bytes = bytes(b for _, b in rx)
    c = {
        "PC6 never driven (pins safe leaves it an input, the RST level never changes)":
            bool(pins) and pins.groups() == ("0", "0") and pin_ev == [] and not d["drvhi"],
        "radio_init does not wait the reset hold (< 1 ms)": bool(init) and int(init.group(2)) < 1000,
        "no boot window: garbage, burst and frame all reach radio_read in order, nothing dropped":
            rx_bytes == GARBAGE + SPAN + CMD and bool(stats) and stats.group(4) == "0",
        "no RX error / overflow": bool(stats) and stats.group(2) == "0" and stats.group(3) == "0"
                                  and int(stats.group(1)) == len(GARBAGE) + len(SPAN) + len(CMD),
    }
    rc_checks, byte_mean_w = reply_checks(d, con, rc, ubrr, baud)
    c.update(rc_checks)
    checks.update({f"warm: {k}": v for k, v in c.items()})
    with open(hexf, "rb") as f:
        sha_w = hashlib.sha256(f.read()).hexdigest()
    out["variants"]["warm"] = {"injections_us": d["inj"], "rx_bytes": len(rx), "u1_byte_us_mean": byte_mean_w,
                               "hex_sha256": sha_w}
    out["checks"] = checks
    out["status"] = "PASS" if all(checks.values()) else "FAIL"
    return out


def main() -> int:
    res = run()
    if res["status"] == "NOT RUN":
        print(f"esp_bridge_test: NOT RUN - {res['why']}")
        return 77
    if "checks" not in res:
        print(f"esp_bridge_test: FAIL - {res['why']}")
        return 1
    for name, v in res["variants"].items():
        print(f"esp_bridge_test {res['env']} {name}: " + ", ".join(f"{k} {val}" for k, val in v.items()
                                                                  if k != "hex_sha256") + f", hex {v['hex_sha256'][:16]}")
    for k, v in res["checks"].items():
        print(f"   {'ok  ' if v else 'FAIL'} {k}")
    print(f"esp_bridge_test: {res['status']} ({len(res['checks'])} checks)")
    return 0 if res["status"] == "PASS" else 1


if __name__ == "__main__":
    sys.exit(main())
