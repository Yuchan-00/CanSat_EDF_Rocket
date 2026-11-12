"""Read the VTOL CAD layout (read-only) and write the numbers the plug-fit check needs (kicad/work/cad_envelope.json).

Invariant: every number comes from ``cad/layout.py`` (``Layout`` built from params.json, VTOL battery config) - none is
typed here. Run with the AI_EDA venv python (B:/Claude/.venv/Scripts/python.exe); the CAD project is never written.
"""

from __future__ import annotations

import json
import os
import sys
from pathlib import Path

#: the repository this script lives in (<repo>/pcb/vtol_carrier/scripts/cad_envelope.py) - never another checkout;
#: redesign2: $CARRIER_PROJ names the checkout for a scratch copy of the board folder (read-only, as build_spec.py)
CANSAT = Path(os.environ["CARRIER_PROJ"]) if os.environ.get("CARRIER_PROJ") else Path(__file__).resolve().parents[3]
sys.path.insert(0, str(CANSAT / "cad"))
sys.path.insert(0, str(CANSAT))

import layout as L  # noqa: E402

import common  # noqa: E402


def main() -> int:
    p = L.load_params(config="vtol")
    lay = L.Layout(p)
    out = {
        "source": "cad/layout.py Layout(load_params(config='vtol')) - read-only",
        "spigot_bore_r": lay.spigot_id / 2,
        "carrier_z": list(lay.fcu_z),
        "board_top_z": lay.fcu_z[0] + 1.6,
        "tray_bottom_z": lay.z_tray_bottom,
        "joint_thetas_deg": list(lay.joint_thetas),
        "joint_screw_z": lay.z_intake_spigot_bottom + lay.joint_screw_dz,
        "motor_joint_zone": {"u": list(lay.ml_joint_u), "half_width": lay.ml_joint_hv, "z": list(lay.ml_joint_z),
                             "theta_deg": lay.wire_theta, "min_clear": p["motor_leads"]["joint_zone"]["min_clear"]
                             if "motor_leads" in p else 2.0},
        "servo_lead_hole": {"theta_deg": lay.sl_gather_theta, "r": [lay.sl_hole_r0, lay.sl_hole_r1], "w": lay.sl_ch_w},
    }
    path = common.WORK_DIR / "cad_envelope.json"
    path.write_text(json.dumps(out, indent=1) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps(out, indent=1))
    return 0


if __name__ == "__main__":
    sys.exit(main())
