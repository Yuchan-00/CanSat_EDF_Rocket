"""Static check of the Fusion 360 script against the installed API stubs.

Invariant enforced: every ``adsk.core.X`` / ``adsk.fusion.X`` referenced by the
script is a class, function or enum defined in the stub files of the installed
Fusion build, every ``adsk.core.X.Y`` is a member of X, and every other attribute
name used in the script exists somewhere in core.py/fusion.py (or is an attribute
of the script's own classes, of an imported stdlib module, of the ``Layout``
object or a builtin-type method).  Findings are printed with line numbers and the
exit code is 1 when any finding remains.

Usage: python tools/fusion_api_lint.py [script.py] [--stubs <adsk package dir>]
"""
from __future__ import annotations

import argparse
import ast
import glob
import importlib
import io
import os
import re
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
DEFAULT_SCRIPT = os.path.join(ROOT, "cad", "fusion360", "CanSatEDFRocket", "CanSatEDFRocket.py")
# Names Fusion injects into the ``adsk`` package at run time (not present in the stubs).
RUNTIME_INJECTED = {"terminate", "autoTerminate", "doEvents"}


def find_stub_dir() -> str | None:
    env = os.environ.get("FUSION_ADSK_STUBS")
    if env and os.path.isfile(os.path.join(env, "core.py")):
        return env
    base = os.path.join(os.environ.get("LOCALAPPDATA", ""), "Autodesk", "webdeploy", "production")
    hits = glob.glob(os.path.join(base, "*", "Api", "Python", "packages", "adsk", "core.py"))
    if not hits:
        return None
    hits.sort(key=os.path.getmtime, reverse=True)
    return os.path.dirname(hits[0])


class Stubs:
    """Parsed view of core.py / fusion.py: classes, members, module functions."""

    def __init__(self, stub_dir: str):
        self.dir = stub_dir
        self.classes: dict[str, dict[str, set[str]]] = {"core": {}, "fusion": {}}
        self.functions: dict[str, set[str]] = {"core": set(), "fusion": set()}
        self.all_members: set[str] = set()
        for mod in ("core", "fusion"):
            self._parse(mod, os.path.join(stub_dir, f"{mod}.py"))

    def _parse(self, mod: str, path: str) -> None:
        cls_re = re.compile(r"^class (\w+)\(")
        def_re = re.compile(r"^def (\w+)\(")
        meth_re = re.compile(r"^    def (\w+)\(")
        enum_re = re.compile(r"^    (\w+) = _(?:core|fusion)\.\w+")
        attach_re = re.compile(r"^(\w+)\.(\w+) = (?:property\(|lambda|staticmethod)")
        current = None
        classes = self.classes[mod]
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                m = cls_re.match(line)
                if m:
                    current = m.group(1)
                    classes.setdefault(current, set())
                    continue
                m = def_re.match(line)
                if m:
                    self.functions[mod].add(m.group(1))
                    current = None
                    continue
                if current is not None:
                    m = meth_re.match(line)
                    if m and not m.group(1).startswith("__"):
                        classes[current].add(m.group(1))
                        self.all_members.add(m.group(1))
                        continue
                    m = enum_re.match(line)
                    if m:
                        classes[current].add(m.group(1))
                        self.all_members.add(m.group(1))
                        continue
                m = attach_re.match(line)
                if m:
                    classes.setdefault(m.group(1), set()).add(m.group(2))
                    self.all_members.add(m.group(2))
        for name, members in classes.items():
            self.all_members.add(name)
        # inherited members: resolve simple single inheritance ``class A(B)`` within the same file
        self._resolve_inheritance(mod, path)

    def _resolve_inheritance(self, mod: str, path: str) -> None:
        base_re = re.compile(r"^class (\w+)\((?:adsk\.(?:core|fusion)\.)?(\w+)\)")
        bases: dict[str, str] = {}
        with open(path, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                m = base_re.match(line)
                if m and m.group(2) not in ("object",):
                    bases[m.group(1)] = m.group(2)
        classes = self.classes[mod]
        other = self.classes["core" if mod == "fusion" else "fusion"]
        for cls in list(classes):
            seen = set()
            b = bases.get(cls)
            while b and b not in seen:
                seen.add(b)
                classes[cls] |= classes.get(b, set()) | other.get(b, set())
                b = bases.get(b)

    def has_class(self, mod: str, name: str) -> bool:
        return name in self.classes[mod] or name in self.functions[mod]

    def has_member(self, mod: str, cls: str, name: str) -> bool:
        return name in self.classes.get(mod, {}).get(cls, set())


def dotted_chain(node: ast.AST) -> list[str] | None:
    parts: list[str] = []
    while isinstance(node, ast.Attribute):
        parts.append(node.attr)
        node = node.value
    if isinstance(node, ast.Name):
        parts.append(node.id)
        return list(reversed(parts))
    return None


def builtin_method_names() -> set[str]:
    import datetime
    import hashlib
    names: set[str] = set()
    stdlib_objects = (str, list, dict, set, tuple, float, int, bytes, bool, BaseException, io.TextIOWrapper, type(sys.argv), type(None), object,
                      re.compile(""), hashlib.sha256(), datetime.datetime, datetime.timedelta)
    for t in stdlib_objects:
        names |= {n for n in dir(t) if not n.startswith("__")}
    names |= {"args", "message", "name", "value", "items", "keys", "values"}
    return names


def layout_attribute_names() -> set[str]:
    sys.path.insert(0, os.path.join(ROOT, "cad"))
    import layout  # noqa: E402
    names: set[str] = set()
    # v0.9: the union over the default preset and every alt preset - mode-specific attributes (vane_drive 'pushrod':
    # pod_*, horn_*, servo_arm_*, sl_win_*; 'direct': dd_*, fair_*, cov_*) exist only on the Layout of a preset that uses
    # that mode, and the script reads them only in that mode's branch
    presets = [None] + sorted((layout.load_params().get("alt_presets") or {}).keys())
    lays = [layout.build(pr) for pr in presets]
    # ... plus the default preset with the v0.8 pushrod linkage (its servo-lead pod windows sl_win_*: no preset combines
    # pushrod with the tab-mount servo-lead route since v0.9, but the script keeps that branch)
    p = layout.load_params()
    if (p.get("vane_drive") or {}).get("mode", "pushrod") != "pushrod":
        p["vane_drive"]["mode"] = "pushrod"
        p["fin"]["theta0_deg"] = 45.0
        p["launch_pad"]["leg_theta0_deg"] = 45.0
        p["propulsion_module"]["tab_mount"] = "screws"     # wire09c: the clock mount is planned for the direct-drive columns
        lays.append(layout.Layout(p))
    # wire09c: ... plus the default preset with the v0.5 screw tab mount (lugs, wire slot, lead notch ml_low_cut_*: the
    # script keeps that branch for tab_mount 'screws')
    ps = layout.load_params()
    if (ps.get("propulsion_module") or {}).get("tab_mount", "screws") != "screws":
        ps["propulsion_module"]["tab_mount"] = "screws"
        lays.append(layout.Layout(ps))
    # v0.9 CAD-2: ... plus the VTOL configuration for every power source (top cap, carrier, windows, fin holes, tether
    # items exist only on those Layouts, and the script reads them only in its VTOL branches)
    if (layout.load_params().get("configs") or {}).get("vtol"):
        lays += [layout.build(config="vtol", power=pw) for pw in layout.POWER_SOURCES]
        # v0.9 fix round: ... plus the VTOL layout WITH the CAD-2 fin lightening hole (configs.vtol no longer has fin.hole -
        # the mass reviewer's strength finding - but the script keeps the has_fin_hole branch for params that do)
        pv = layout.load_params(config="vtol", power="battery")
        if not (pv.get("fin") or {}).get("hole"):
            pv["fin"]["hole"] = {"root_gap": 3.6, "tip_strip": 4.0, "bottom_strip": 10.0, "le_margin": 6.0, "gable_deg": 45.0}
            lays.append(layout.Layout(pv))
    for lay in lays:
        names |= set(vars(lay).keys()) | {n for n in dir(lay) if not n.startswith("__")}
    names |= {n for n in dir(layout) if not n.startswith("__")}
    return names


def lint(script_path: str, stubs: Stubs) -> list[str]:
    with open(script_path, "r", encoding="utf-8") as f:
        src = f.read()
    tree = ast.parse(src, filename=script_path)
    findings: list[str] = []
    # own classes: method names and self.<attr> assignments; own module-level names
    own: set[str] = set()
    stdlib_mods: dict[str, object] = {}
    for node in ast.walk(tree):
        if isinstance(node, ast.ClassDef):
            own.add(node.name)
            for b in node.body:
                if isinstance(b, (ast.FunctionDef, ast.AsyncFunctionDef)):
                    own.add(b.name)
                elif isinstance(b, ast.Assign):
                    for t in b.targets:
                        if isinstance(t, ast.Name):
                            own.add(t.id)
                elif isinstance(b, ast.AnnAssign) and isinstance(b.target, ast.Name):
                    own.add(b.target.id)
        elif isinstance(node, ast.FunctionDef):
            own.add(node.name)
        elif isinstance(node, (ast.Assign, ast.AnnAssign)):
            targets = node.targets if isinstance(node, ast.Assign) else [node.target]
            flat: list[ast.AST] = []
            for t in targets:
                flat.extend(t.elts if isinstance(t, (ast.Tuple, ast.List)) else [t])
            for t in flat:
                if isinstance(t, ast.Attribute) and isinstance(t.value, ast.Name) and t.value.id in ("self", "cls"):
                    own.add(t.attr)
        elif isinstance(node, ast.Import):
            for a in node.names:
                if a.name.split(".")[0] != "adsk":
                    try:
                        stdlib_mods[a.asname or a.name.split(".")[0]] = importlib.import_module(a.name.split(".")[0])
                    except Exception:
                        pass
    whitelist = builtin_method_names() | own
    layout_names = layout_attribute_names()
    layout_roots = {"lay", "L", "layout", "self.lay", "cb.lay", "ctx.lay"}
    checked = 0
    for node in ast.walk(tree):
        if not isinstance(node, ast.Attribute):
            continue
        chain = dotted_chain(node)
        line = node.lineno
        if chain and chain[0] == "adsk":
            # only check the full chain once (outermost attribute node)
            if len(chain) >= 2 and chain[1] not in ("core", "fusion"):
                if chain[1] in RUNTIME_INJECTED:
                    print(f"INFO {line}: adsk.{chain[1]} is injected at run time (not in stubs)")
                else:
                    findings.append(f"{line}: adsk.{chain[1]} is not a stub module")
                continue
            if len(chain) >= 3:
                mod, cls = chain[1], chain[2]
                checked += 1
                if not stubs.has_class(mod, cls):
                    findings.append(f"{line}: adsk.{mod}.{cls} not found in {mod}.py")
                    continue
                if len(chain) >= 4:
                    checked += 1
                    if not stubs.has_member(mod, cls, chain[3]):
                        findings.append(f"{line}: adsk.{mod}.{cls}.{chain[3]} is not a member of {cls}")
            continue
        attr = node.attr
        checked += 1
        root = chain[0] if chain else None
        if root in stdlib_mods and len(chain) == 2:
            if not hasattr(stdlib_mods[root], attr):
                findings.append(f"{line}: {root}.{attr} not found in module {root}")
            continue
        if root in stdlib_mods:
            continue  # deeper stdlib chains (os.path.join etc.)
        if chain and ".".join(chain[:-1]) in layout_roots and attr in layout_names:
            continue
        if attr in layout_names and chain and chain[0] in ("L", "lay"):
            continue
        if attr in stubs.all_members or attr in whitelist:
            continue
        if attr in layout_names:
            continue
        findings.append(f"{line}: attribute '{attr}' (on {'.'.join(chain) if chain else '<expr>'}) is not an API member, own attribute, layout attribute or builtin method")
    print(f"checked {checked} attribute references in {os.path.relpath(script_path, ROOT)}")
    return findings


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("script", nargs="?", default=DEFAULT_SCRIPT)
    ap.add_argument("--stubs", default=None, help="directory containing adsk/core.py and fusion.py")
    args = ap.parse_args(argv)
    stub_dir = args.stubs or find_stub_dir()
    if not stub_dir:
        print("ERROR: Fusion API stubs not found (set FUSION_ADSK_STUBS)")
        return 2
    stubs = Stubs(stub_dir)
    print(f"stubs: {stub_dir} ({len(stubs.classes['core'])} core classes, {len(stubs.classes['fusion'])} fusion classes)")
    findings = lint(args.script, stubs)
    for f in findings:
        print("FINDING", f)
    print(f"{len(findings)} finding(s)")
    return 1 if findings else 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
