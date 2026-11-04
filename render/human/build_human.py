"""Build the rigged human from the Blender Studio Human Base Meshes bundle (CC0): every step in order.

  1. bl_prepare_body.py   (Blender)  body from the bundle, sculpt folded into the cage, 1.75 m, soles on z = 0
  2. fit_joints.py        (Python)   joint centres from the mesh (face-set boundary rings + cross-sections)
  3. bl_build_rig.py      (Blender)  armature, bone-heat weights + clean-up, LBS export (build/lbs_raw.npz)
  4. make_lbs_npz.py      (Python)   human_lbs.npz for tools/human_figure.py: figure frame, grip, pose presets
  5. bl_finalize_rig.py   (Blender)  human_rigged.blend: IK targets, constraints + drivers, actions, material
  6. bl_render_tests.py   (Blender)  optional (--stills): Eevee test stills of the presets for checking deformation

Run with the project venv:  python render/human/build_human.py [--bundle PATH] [--blender EXE] [--stills]
Defaults: the bundle under render/assets/human_base_meshes/ (git-ignored), Blender 5.2 under %LOCALAPPDATA%.
The outputs are deterministic for a given bundle + Blender version (no RNG / clock in any step).
"""
from __future__ import annotations

import argparse
import os
import subprocess
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(os.path.dirname(HERE))
BUNDLE = os.path.join(ROOT, "render", "assets", "human_base_meshes", "human-base-meshes-bundle-v1.4.1",
                      "human_base_meshes_bundle.blend")
BLENDER = os.path.join(os.environ.get("LOCALAPPDATA", ""), "Programs", "Blender", "blender-5.2.2-windows-x64",
                       "blender.exe")


def run(cmd: list[str]) -> None:
    print("$", " ".join(os.path.basename(c) if i == 0 else c for i, c in enumerate(cmd)), flush=True)
    r = subprocess.run(cmd, cwd=HERE, text=True, encoding="utf-8", errors="replace", capture_output=True,
                       env={**os.environ, "PYTHONIOENCODING": "utf-8"})
    keep = [ln for ln in (r.stdout + r.stderr).splitlines()
            if ln.strip() and "cannot be made relative" not in ln and not ln.startswith(("Fra:", "Blender quit"))]
    print("\n".join(keep[-25:]))
    if r.returncode != 0 or any("Traceback" in ln for ln in keep):
        raise SystemExit(f"step failed: {cmd[-1] if cmd else cmd}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--bundle", default=BUNDLE)
    ap.add_argument("--blender", default=BLENDER)
    ap.add_argument("--build", default=os.path.join(HERE, "build"))
    ap.add_argument("--stills", action="store_true", help="also render the Blender test stills")
    a = ap.parse_args()
    if not os.path.exists(a.bundle):
        raise SystemExit(f"bundle not found: {a.bundle} (download the CC0 Human Base Meshes v1.4.1 from Blender "
                         "Studio and unzip it under render/assets/human_base_meshes/)")
    bl = [a.blender, "--background", "--factory-startup", "--python"]
    py = sys.executable
    run(bl + ["bl_prepare_body.py", "--", a.bundle, a.build, "1.75"])
    run([py, "fit_joints.py", a.build])
    run(bl + ["bl_build_rig.py", "--", a.build])
    run([py, "make_lbs_npz.py", a.build, os.path.join(HERE, "human_lbs.npz")])
    run(bl + ["bl_finalize_rig.py", "--", a.build, os.path.join(HERE, "human_lbs.npz"),
              os.path.join(HERE, "human_rigged.blend")])
    if a.stills:
        run(bl + ["bl_render_tests.py", "--", os.path.join(HERE, "human_rigged.blend"), os.path.join(a.build, "stills")])
    print("done:", os.path.join(HERE, "human_rigged.blend"), os.path.join(HERE, "human_lbs.npz"))


if __name__ == "__main__":
    main()
