# PlatformIO pre-build script: regenerate src/bridge_config_gen.h from params.json before every build,
# so the bridge can never be flashed with radio settings other than the vehicle's.
# (SCons extra scripts have no __file__; the project directory comes from the environment.)
import os
import subprocess

Import("env")  # noqa: F821  (provided by PlatformIO / SCons)

_gen = os.path.join(env.subst("$PROJECT_DIR"), "gen_bridge_config.py")  # noqa: F821
_r = subprocess.run([env.subst("$PYTHONEXE"), _gen], capture_output=True, text=True,  # noqa: F821
                    encoding="utf-8", errors="replace")
print(_r.stdout.strip())
if _r.returncode != 0:
    print(_r.stderr)
    env.Exit(1)  # noqa: F821
