"""Compile the ESP8266 bridge with arduino-cli (esp8266:esp8266 core) for every role / flag variant.

arduino-cli needs a sketch folder <name>/<name>.ino, so src/ is copied into build/sketch/esp_bridge/ (git-ignored) with an
empty .ino shell; the code itself stays in src/main.cpp and src/bridge_core.*.  Each variant's .bin, its size and sha256
go to build/<variant>/ and build/build_results.json.  The config header must be current first
(gen_esp_bridge_config.py --check); this script refuses to build a stale one.

Run:  .venv/Scripts/python.exe firmware/esp_bridge/build_arduino_cli.py [variant ...]
Needs: arduino-cli (PATH, or %LOCALAPPDATA%/Programs/arduino-cli) and the core esp8266:esp8266 (3.1.2 measured here).
"""
from __future__ import annotations

import hashlib
import json
import os
import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
SRC = HERE / "src"
BUILD = HERE / "build"
# ESP-12E and ESP-07S both carry 4 MB flash; the generic board with a 4M2M layout fits both roles.
FQBN = "esp8266:esp8266:generic:eesz=4M2M,baud=115200"
VARIANTS = {
    "vehicle": ["-DBRIDGE_ROLE_VEHICLE"],
    "ground": ["-DBRIDGE_ROLE_GROUND"],
    "vehicle_debug": ["-DBRIDGE_ROLE_VEHICLE", "-DBRIDGE_DEBUG_UART1"],
    "vehicle_ratelimit": ["-DBRIDGE_ROLE_VEHICLE", "-DBRIDGE_USE_RATE_LIMIT_API"],
    "ground_debug_ratelimit": ["-DBRIDGE_ROLE_GROUND", "-DBRIDGE_DEBUG_UART1", "-DBRIDGE_USE_RATE_LIMIT_API"],
}


def find_cli() -> str:
    cli = shutil.which("arduino-cli")
    if cli:
        return cli
    local = Path(os.environ.get("LOCALAPPDATA", "")) / "Programs" / "arduino-cli" / "arduino-cli.exe"
    if local.exists():
        return str(local)
    sys.exit("arduino-cli not found (PATH or %LOCALAPPDATA%/Programs/arduino-cli)")


def main(argv: list[str]) -> int:
    check = subprocess.run([sys.executable, str(HERE / "gen_esp_bridge_config.py"), "--check"],
                           capture_output=True, text=True, encoding="utf-8", errors="replace")
    if check.returncode:
        print(check.stdout + check.stderr)
        return 1
    cli = find_cli()
    names = argv or list(VARIANTS)
    results = {}
    failed = 0
    for name in names:
        sketch = BUILD / "sketch" / "esp_bridge"
        if sketch.exists():
            shutil.rmtree(sketch)
        sketch.mkdir(parents=True)
        for f in SRC.iterdir():
            shutil.copy2(f, sketch / f.name)
        (sketch / "esp_bridge.ino").write_text("// sketch shell: the code is in src/main.cpp\n", encoding="utf-8")
        out = BUILD / name
        out.mkdir(parents=True, exist_ok=True)
        extra = " ".join(VARIANTS[name] + ["-Wall", "-Wextra"])
        r = subprocess.run([cli, "compile", "--fqbn", FQBN, "--warnings", "all", "--output-dir", str(out),
                            "--build-property", f"compiler.cpp.extra_flags={extra}", str(sketch)],
                           capture_output=True, text=True, encoding="utf-8", errors="replace")
        log = r.stdout + r.stderr
        (out / "compile.log").write_text(log, encoding="utf-8")
        ours = [l for l in log.splitlines() if "warning" in l and ("main.cpp" in l or "bridge_core" in l)]
        rec = {"exit": r.returncode, "flags": VARIANTS[name], "warnings_in_bridge_src": ours,
               "memory": [l.strip() for l in log.splitlines() if l.strip().startswith(". ")]}
        bins = sorted(out.glob("esp_bridge.ino.bin"))
        if bins:
            data = bins[0].read_bytes()
            rec.update(bin=bins[0].name, bytes=len(data), sha256=hashlib.sha256(data).hexdigest())
        results[name] = rec
        failed += r.returncode != 0
        print(f"{name}: exit {r.returncode}, {rec.get('bytes')} B, sha256 {rec.get('sha256', '')[:16]}, "
              f"{len(ours)} warnings in the bridge source")
    (BUILD / "build_results.json").write_text(json.dumps({"fqbn": FQBN, "variants": results}, indent=1), encoding="utf-8")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
