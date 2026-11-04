"""PlatformIO pre-script of the FCU envs (platformio.ini [fcu_common] extra_scripts): pin the onboard supervisor to the
build's mission mode at compile time.

params.json ``mission.mode`` is a build-time choice (v0.7, ``src/fc_mode.h``).  An env that does not pin the mode
itself (``-D FC_MODE_BALLISTIC`` / ``-D FC_MODE_VTOL`` with ``-D ONB_FIXED_MODE=...``) flies the mode of
``src/params_gen.h`` (``MISSION_MODE``); this script adds ``-D ONB_FIXED_MODE=<that mode>`` so ``core/onboard.c``
drops the other mission's supervisor code from the flight firmware (``fc_mode.h`` checks that the two agree).

Invariant: the script only ever adds the define that ``params_gen.h`` already implies; it never chooses a mode.
"""
import os
import re

Import("env")  # noqa: F821  (PlatformIO SCons environment)

_MODES = {"MISSION_MODE_BALLISTIC": "LP_MODE_BALLISTIC", "MISSION_MODE_VTOL": "LP_MODE_VTOL"}


def _pinned(flags) -> bool:
    text = " ".join(str(f) for f in flags)
    return "FC_MODE_BALLISTIC" in text or "FC_MODE_VTOL" in text or "ONB_FIXED_MODE" in text


def _params_mode(project_dir: str) -> str:
    path = os.path.join(project_dir, "src", "params_gen.h")
    with open(path, "r", encoding="utf-8") as f:
        m = re.search(r"^#define\s+MISSION_MODE\s+(MISSION_MODE_\w+)", f.read(), re.M)
    if not m or m.group(1) not in _MODES:
        raise SystemExit(f"pio_mission_mode: {path} has no MISSION_MODE - re-run tools/gen_firmware_params.py")
    return _MODES[m.group(1)]


if not _pinned(env.get("BUILD_FLAGS", [])):  # noqa: F821
    env.Append(CPPDEFINES=[("ONB_FIXED_MODE", _params_mode(env.subst("$PROJECT_DIR")))])  # noqa: F821
