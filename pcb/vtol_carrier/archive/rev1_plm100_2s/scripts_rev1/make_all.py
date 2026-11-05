"""Regenerate everything from spec/carrier_spec.json (run with the AI_EDA venv python).

  python make_all.py              # schematic -> placement -> unrouted board -> import the recorded
                                  # Freerouting session (kicad/work/vtol_carrier.ses) -> checks + outputs
  python make_all.py --reroute    # same, but run Freerouting again instead of re-importing the session
                                  # (Freerouting is not deterministic: the new board can differ)
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
        step(route + ["40", "4"])
    else:
        step(route + ["--reuse-ses"])
    step(py + ["checks_exports.py"])
    return 0


if __name__ == "__main__":
    sys.exit(main())
