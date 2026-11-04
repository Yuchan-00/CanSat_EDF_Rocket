"""Install (or refresh) the CanSatEDFRocket script into Fusion 360's Scripts folder.

Invariant enforced: the installed copy is byte-identical to the repository copy
and carries a ``project_root.txt`` with the absolute project root, so the script
always imports the repository's ``cad/layout.py`` and writes into the repository's
``cad/exports`` regardless of where Fusion loads it from.

Usage: python tools/install_fusion_script.py [--dest <folder>] [--uninstall]
"""
from __future__ import annotations

import argparse
import filecmp
import os
import shutil
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
SRC_DIR = os.path.join(ROOT, "cad", "fusion360", "CanSatEDFRocket")
SCRIPT_NAME = "CanSatEDFRocket"
FILES = (f"{SCRIPT_NAME}.py", f"{SCRIPT_NAME}.manifest")


def default_dest() -> str:
    appdata = os.environ.get("APPDATA")
    if not appdata:
        raise EnvironmentError("APPDATA is not set; pass --dest")
    return os.path.join(appdata, "Autodesk", "Autodesk Fusion 360", "API", "Scripts", SCRIPT_NAME)


def install(dest: str) -> list[str]:
    os.makedirs(dest, exist_ok=True)
    written: list[str] = []
    for name in FILES:
        src = os.path.join(SRC_DIR, name)
        if not os.path.isfile(src):
            raise FileNotFoundError(src)
        dst = os.path.join(dest, name)
        shutil.copy2(src, dst)
        if not filecmp.cmp(src, dst, shallow=False):
            raise RuntimeError(f"copy verification failed for {name}")
        written.append(dst)
    marker = os.path.join(dest, "project_root.txt")
    with open(marker, "w", encoding="utf-8", newline="\n") as f:
        f.write(ROOT + "\n")
    written.append(marker)
    pyc = os.path.join(dest, "__pycache__")
    if os.path.isdir(pyc):
        shutil.rmtree(pyc, ignore_errors=True)
    return written


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--dest", default=None, help="target script folder (default: %%APPDATA%%\\Autodesk\\Autodesk Fusion 360\\API\\Scripts\\CanSatEDFRocket)")
    ap.add_argument("--uninstall", action="store_true")
    args = ap.parse_args(argv)
    dest = args.dest or default_dest()
    if args.uninstall:
        if os.path.isdir(dest):
            shutil.rmtree(dest)
            print(f"removed {dest}")
        else:
            print(f"nothing to remove at {dest}")
        return 0
    written = install(dest)
    print(f"installed {SCRIPT_NAME} -> {dest}")
    for w in written:
        print(f"  {w} ({os.path.getsize(w)} bytes)")
    print(f"project root recorded: {ROOT}")
    print("In Fusion 360: Utilities > Add-Ins > Scripts tab > CanSatEDFRocket > Run")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
