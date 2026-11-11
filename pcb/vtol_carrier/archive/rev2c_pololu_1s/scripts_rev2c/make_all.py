"""Regenerate everything from spec/carrier_spec.json (run with the AI_EDA venv python).

  python make_all.py              # schematic -> placement -> unrouted 4-layer board -> replay of the recorded
                                  # Freerouting sessions (kicad/work/ses_replay.json, deterministic) -> finishing
                                  # (power widening, scripted routing of the last open connections close_leftovers.py,
                                  # pad-safe GND stitching, silkscreen, dangling-stub removal) -> checks + outputs
  python make_all.py --reroute    # same, but run Freerouting again (power_first order, In2 = +5V plane, plane fan-out
                                  # first; Freerouting is not deterministic: the new board can differ)
  CARRIER_LAYERS=2 ...            # the 2-layer variant (did not route: README_ko.md section 3)
"""

from __future__ import annotations

import subprocess
import sys

import common

HERE = common.ROOT / "scripts"


def step(args: list[str]) -> None:
    print(">>", " ".join(args), flush=True)
    r = subprocess.run(args, cwd=str(HERE), text=True, encoding="utf-8", errors="replace")
    if r.returncode != 0:
        raise SystemExit(f"step failed: {args}")


def main() -> int:
    py = [sys.executable, "-X", "utf8"]
    step(py + ["build_schematic.py"])
    step(py + ["place.py"])
    step(py + ["build_pcb.py"])
    route = [str(common.KICAD_PY), "route.py"]
    if "--reroute" in sys.argv:
        step(route + ["40", "6"])
    else:
        step(route + ["--reuse-ses"])
    step(py + ["checks_exports.py"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
