"""Run the Fusion 360 script's ``run()`` against a fake ``adsk`` package.

Invariant checked: all of OUR Python logic (layout maths, component loops, mass
property arithmetic, report/log writing) executes without an exception when the
Fusion API is replaced by MagicMock-based fakes.  Numeric API reads return 1.0
(MagicMock's ``__float__``), ``modelToSketchSpace`` is an identity with z = 0 so
plane probes pass, and ``getXYZMomentsOfInertia`` returns a 7-tuple.  Every
``.add(...)`` / ``createInput(...)`` / ``addNewComponent(...)`` call returns a NEW
fake (each feature, input, body and component is a distinct object, so the
script's body-identity checks see distinct bodies as in Fusion), and bounding
boxes span +-1e5 mm (they contain every point, so placement checks pass; the
mock never checks geometry).  Exports are redirected to a temporary directory
through ``CANSAT_EXPORT_ROOT``.

Body / volume model (``BodyModel``; no geometry, only bookkeeping that Fusion does):
* a NewBody extrude / revolve creates one body (``feature.bodies``) and lists it in
  its component's ``bRepBodies``; every body starts with ``MOCK_BODY_VOLUME_CM3``;
* a Combine needs a listed target and listed tool bodies; Cut lowers the target's
  ``volume`` by ``MOCK_CUT_FRACTION`` of each tool's volume, Join raises it by the
  tools' volumes; with ``isKeepToolBodies`` False (Fusion's default) the tools are
  consumed (unlisted, ``isValid`` False);
* (v0.6) a Cut extrude / revolve lowers the volume of each listed body in its input's
  ``participantBodies`` by ``MOCK_CUT_FRACTION * MOCK_BODY_VOLUME_CM3`` (``CB.cut_verified``
  falls back to that volume drop because the mock cannot answer ``pointContainment``;
  a Cut feature without participants is not modelled, as before);
* (v0.6.2) an Intersect extrude must name listed ``participantBodies`` (refused otherwise: without them Fusion
  would intersect every body of the component) and keeps ``MOCK_INTERSECT_FRACTION`` of each one's volume (the
  gable window tool; its volume drop is the fallback evidence when ``pointContainment`` is unanswered);
* ``BRepBody.deleteMe`` and ``removeFeatures.add(body)`` unlist a body;
* ``adsk.core.ObjectCollection`` is a real collection.
So ``CB.cut_between``'s volume check, ``CB._alive`` and ``combine_all``'s cutter
guard run on real bookkeeping.

The run must finish, write ``fusion360_report.json`` with the SPEC section 6 keys,
report zero failed components and leave every printed part as one body.  Exit
code 0 on success.

Usage: python tools/fusion_mock_run.py [--keep] [--preset NAME]
"""
from __future__ import annotations

import argparse
import importlib.util
import json
import os
import shutil
import sys
import tempfile
import types
from unittest.mock import MagicMock

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
SCRIPT = os.path.join(ROOT, "cad", "fusion360", "CanSatEDFRocket", "CanSatEDFRocket.py")
BBOX_HALF_CM = 1.0e4   # fake bounding boxes: +-1e5 mm, i.e. they contain every model point
MOCK_BODY_VOLUME_CM3 = 1.0   # every NEW body starts with this volume (what MagicMock's float() gave before)
MOCK_CUT_FRACTION = 0.01     # a Combine/Cut removes this fraction of each tool's volume from the target (10 mm^3)
MOCK_INTERSECT_FRACTION = 0.5   # v0.6.2: an Intersect feature keeps this fraction of each participant body's volume

_MODEL: "BodyModel | None" = None   # set by make_fake_adsk(); used by Fake's special cases


class FakeList:
    """Ordered collection with Fusion's count / item(i) / iteration interface (bRepBodies, feature.bodies,
    adsk.core.ObjectCollection)."""

    def __init__(self, items=None):
        self._items: list = list(items or [])

    @staticmethod
    def create() -> "FakeList":
        """adsk.core.ObjectCollection.create()"""
        return FakeList()

    def add(self, obj) -> bool:
        self._items.append(obj)
        return True

    def item(self, i: int):
        return self._items[i]

    @property
    def count(self) -> int:
        return len(self._items)

    def __len__(self) -> int:
        return len(self._items)

    def __iter__(self):
        return iter(list(self._items))

    def _remove(self, obj) -> bool:
        for i, x in enumerate(self._items):
            if x is obj:
                del self._items[i]
                return True
        return False


def _fake_args(inp) -> tuple:
    """Positional arguments of the createInput(...) call that made ``inp`` (() when unknown)."""
    return inp.__dict__.get("_fake_args", ()) if isinstance(inp, MagicMock) else ()


def _component_of(collection):
    """The component owning a feature collection (``<component>.features.<xxx>Features``), else None."""
    node = collection
    while isinstance(node, MagicMock):
        if node._mock_name == "features":
            return node._mock_new_parent
        node = node._mock_new_parent
    return None


class BodyModel:
    """Bookkeeping of bodies and their volumes (see the module docstring).  Raises on API misuse that Fusion
    would reject (a Combine on a body that is not listed, a tool that is the target)."""

    def __init__(self, fusion_mod):
        fo = fusion_mod.FeatureOperations
        self.NEW, self.CUT, self.JOIN = fo.NewBodyFeatureOperation, fo.CutFeatureOperation, fo.JoinFeatureOperation
        self.INTERSECT = fo.IntersectFeatureOperation
        self.stats = {"bodies_created": 0, "combine_cut": 0, "combine_join": 0, "tools_consumed": 0, "bodies_deleted": 0,
                      "feature_cut": 0, "feature_intersect": 0}

    @staticmethod
    def _listed(body) -> bool:
        owner = body.__dict__.get("_fake_owner") if isinstance(body, MagicMock) else None
        return owner is not None and any(x is body for x in owner)

    @staticmethod
    def _volume(body) -> float:
        return float(body.__dict__["volume"])

    def register(self, comp, body) -> None:
        owner = comp.bRepBodies
        owner.add(body)
        body.__dict__["_fake_owner"] = owner
        body.isValid = True
        body.volume = MOCK_BODY_VOLUME_CM3
        self.stats["bodies_created"] += 1

    def unlist(self, body) -> bool:
        owner = body.__dict__.get("_fake_owner") if isinstance(body, MagicMock) else None
        if owner is None or not owner._remove(body):
            return False
        body.isValid = False
        return True

    def delete(self, obj) -> None:
        if self.unlist(obj):
            self.stats["bodies_deleted"] += 1

    def on_add(self, collection, feat, args: tuple) -> None:
        kind = collection._mock_name
        if kind in ("extrudeFeatures", "revolveFeatures"):
            inp = args[0] if args else None
            if any(a is self.NEW for a in _fake_args(inp)):
                comp = _component_of(collection)
                if comp is None:
                    raise RuntimeError(f"mock: {kind}.add NewBody outside a component")
                body = Fake(name=f"{kind}.body")
                self.register(comp, body)
                feat.bodies = FakeList([body])
            elif any(a is self.CUT for a in _fake_args(inp)):
                # v0.6: a Cut feature removes material from its participant bodies (only those named explicitly)
                parts = inp.__dict__.get("participantBodies") if isinstance(inp, MagicMock) else None
                for b in list(parts or []):
                    if self._listed(b):
                        b.volume = self._volume(b) - MOCK_CUT_FRACTION * MOCK_BODY_VOLUME_CM3
                        if b.volume <= 0.0:
                            raise RuntimeError("mock: volume model exhausted (raise MOCK_BODY_VOLUME_CM3)")
                self.stats["feature_cut"] += 1
            elif any(a is self.INTERSECT for a in _fake_args(inp)):
                # v0.6.2: an Intersect feature keeps the part of each participant body inside the tool shape; Fusion
                # applies it to the participants only, so an Intersect without explicit, listed participants is refused
                # (it would act on every body of the component)
                parts = list(inp.__dict__.get("participantBodies") or []) if isinstance(inp, MagicMock) else []
                if not parts:
                    raise RuntimeError("mock: Intersect feature without participantBodies is not modelled")
                for b in parts:
                    if not self._listed(b):
                        raise RuntimeError("mock: Intersect participant is not a listed body")
                    b.volume = self._volume(b) * MOCK_INTERSECT_FRACTION
                self.stats["feature_intersect"] += 1
        elif kind == "combineFeatures":
            inp = args[0]
            fa = _fake_args(inp)
            if len(fa) < 2:
                raise RuntimeError("mock: combineFeatures.add with an input not made by createInput(target, tools)")
            target, tools = fa[0], list(fa[1])
            op = inp.__dict__.get("operation")
            if not self._listed(target):
                raise RuntimeError("mock: Combine target is not a body of any component")
            for t in tools:
                if t is target or not self._listed(t):
                    raise RuntimeError("mock: Combine tool is the target or not a listed body")
            if op is self.CUT:
                target.volume = self._volume(target) - sum(MOCK_CUT_FRACTION * self._volume(t) for t in tools)
                if target.volume <= 0.0:
                    raise RuntimeError("mock: volume model exhausted (raise MOCK_BODY_VOLUME_CM3)")
                self.stats["combine_cut"] += 1
            elif op is self.JOIN:
                target.volume = self._volume(target) + sum(self._volume(t) for t in tools)
                self.stats["combine_join"] += 1
            else:
                raise RuntimeError(f"mock: Combine operation {op!r} not modelled")
            if not inp.__dict__.get("isKeepToolBodies", False):
                for t in tools:
                    if self.unlist(t):
                        self.stats["tools_consumed"] += 1
        elif kind == "removeFeatures":
            self.delete(args[0])


class FakePoint3D:
    """Real x/y/z so sketch-plane probes and point arithmetic behave numerically."""

    def __init__(self, x: float = 0.0, y: float = 0.0, z: float = 0.0):
        self.x, self.y, self.z = float(x), float(y), float(z)

    @staticmethod
    def create(x: float = 0.0, y: float = 0.0, z: float = 0.0) -> "FakePoint3D":
        return FakePoint3D(x, y, z)

    def asArray(self) -> list[float]:
        return [self.x, self.y, self.z]

    def copy(self) -> "FakePoint3D":
        return FakePoint3D(self.x, self.y, self.z)


class Fake(MagicMock):
    """MagicMock whose children are Fakes and which special-cases a few API calls."""

    def _get_child_mock(self, **kw):
        return Fake(**kw)

    def __getattr__(self, name):
        if name == "modelToSketchSpace":
            return lambda p: FakePoint3D(getattr(p, "x", 0.0), getattr(p, "y", 0.0), 0.0)
        if name == "sketchToModelSpace":
            return lambda p: p
        if name == "getXYZMomentsOfInertia":
            return lambda: (True, 2.0, 2.0, 2.0, 0.1, 0.1, 0.1)
        if name == "centerOfMass":
            return FakePoint3D(1.0, 2.0, 3.0)
        if name in ("minPoint",):
            return FakePoint3D(-BBOX_HALF_CM, -BBOX_HALF_CM, -BBOX_HALF_CM)
        if name in ("maxPoint",):
            return FakePoint3D(BBOX_HALF_CM, BBOX_HALF_CM, BBOX_HALF_CM)
        if name == "add":
            # a fresh object per call: features, bodies, sketches and planes are distinct like in Fusion
            parent = self

            def _add(*a, **k):
                feat = Fake(name=f"{parent._mock_name or 'fake'}.add()")
                if _MODEL is not None:
                    _MODEL.on_add(parent, feat, a)
                return feat
            return _add
        if name == "createInput":
            # a fresh input per call that remembers its arguments (the body model reads target / tools / operation)
            parent = self

            def _create_input(*a, **k):
                inp = Fake(name=f"{parent._mock_name or 'fake'}.input")
                inp.__dict__["_fake_args"] = a
                return inp
            return _create_input
        if name == "addNewComponent":
            # a distinct occurrence (and so a distinct component with its own bRepBodies) per call, as in Fusion
            return lambda *a, **k: Fake(name="occurrence")
        if name == "bRepBodies":
            bodies = FakeList()
            self.__dict__["bRepBodies"] = bodies     # cached: later reads find it without __getattr__
            return bodies
        if name == "deleteMe":
            target = self
            return lambda: (_MODEL.delete(target) if _MODEL is not None else None)
        return super().__getattr__(name)


def make_fake_adsk() -> types.ModuleType:
    adsk = types.ModuleType("adsk")
    core = types.ModuleType("adsk.core")
    fusion = types.ModuleType("adsk.fusion")
    core.Point3D = FakePoint3D
    plane_cls = Fake(name="Plane")
    plane_cls.classType = staticmethod(lambda: "adsk::core::Plane")
    core.Plane = plane_cls

    def core_getattr(name: str):
        obj = Fake(name=f"adsk.core.{name}")
        setattr(core, name, obj)
        return obj

    def fusion_getattr(name: str):
        obj = Fake(name=f"adsk.fusion.{name}")
        setattr(fusion, name, obj)
        return obj

    core.__getattr__ = core_getattr
    fusion.__getattr__ = fusion_getattr
    core.ObjectCollection = FakeList          # a real collection (create / add / item / count)
    adsk.core = core
    adsk.fusion = fusion
    sys.modules["adsk"] = adsk
    sys.modules["adsk.core"] = core
    sys.modules["adsk.fusion"] = fusion
    global _MODEL
    _MODEL = BodyModel(fusion)
    return adsk


REQUIRED_TOP = ("generator", "generator_version", "preset", "params_sha256", "parts", "totals", "derived", "warnings")
REQUIRED_PART = ("kind", "volume_mm3", "density_g_cm3", "mass_g", "bbox_mm", "com_mm", "inertia_about_com_kg_m2")


def main(argv: list[str]) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--keep", action="store_true", help="keep the temporary export directory")
    ap.add_argument("--preset", default=None, help="preset name passed to the script through CANSAT_PRESET")
    ap.add_argument("--config", default=None, help="v0.9 CAD-2 hardware configuration passed through CANSAT_CONFIG")
    ap.add_argument("--power", default=None, help="v0.9 CAD-2 VTOL power source passed through CANSAT_POWER")
    args = ap.parse_args(argv)
    tmp = tempfile.mkdtemp(prefix="cansat_fusion_mock_")
    os.environ["CANSAT_EXPORT_ROOT"] = tmp
    os.environ["CANSAT_PROJECT_ROOT"] = ROOT
    if args.preset:
        os.environ["CANSAT_PRESET"] = args.preset
    if args.config:
        os.environ["CANSAT_CONFIG"] = args.config
    if args.power:
        os.environ["CANSAT_POWER"] = args.power
    cfg = args.config or os.environ.get("CANSAT_CONFIG") or "rocket"
    pw = args.power or os.environ.get("CANSAT_POWER") or "battery"
    sfx = "" if cfg == "rocket" else "_vtol" + ("" if pw == "battery" else f"_{pw}")
    make_fake_adsk()
    spec = importlib.util.spec_from_file_location("CanSatEDFRocket", SCRIPT)
    mod = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    spec.loader.exec_module(mod)
    rc = 0
    try:
        mod.run("mock")
        report_path = os.path.join(tmp, f"fusion360_report{sfx}.json")
        log_path = os.path.join(tmp, f"fusion{sfx}", "run.log")
        if not os.path.isfile(report_path):
            print("FAIL: report not written")
            if os.path.isfile(log_path):
                with open(log_path, "r", encoding="utf-8") as f:
                    print(f.read()[-4000:])
            return 1
        with open(report_path, "r", encoding="utf-8") as f:
            rep = json.load(f)
        missing = [k for k in REQUIRED_TOP if k not in rep]
        bad_parts = {n: [k for k in REQUIRED_PART if k not in p] for n, p in rep["parts"].items() if any(k not in p for k in REQUIRED_PART)}
        kinds = {}
        for p in rep["parts"].values():
            kinds[p["kind"]] = kinds.get(p["kind"], 0) + 1
        failed = rep.get("failed_components", {})
        n_attempted = len(rep.get("timings_s", {}))
        print(f"components attempted: {n_attempted}")
        print(f"parts in report: {len(rep['parts'])} {kinds}")
        print(f"failed components: {len(failed)} {failed}")
        print(f"soft failures: {len(rep.get('soft_failures', []))}")
        print(f"warnings: {len(rep.get('warnings', []))}")
        print(f"preset: {rep.get('preset')} params_sha256: {rep.get('params_sha256', '')[:16]}...")
        print(f"derived: pushrod_len_mm={rep['derived'].get('pushrod_len_mm'):.3f} vehicle_h_mm={rep['derived'].get('vehicle_h_mm')} inlet_area_ratio={rep['derived'].get('inlet_area_ratio'):.3f}")
        print(f"totals.mass_g (mock volumes, not physical): {rep['totals']['mass_g']:.3f}")
        with open(log_path, "r", encoding="utf-8") as f:
            lines = f.readlines()
        n_feat = sum(1 for ln in lines if " FEAT " in ln)
        n_err = sum(1 for ln in lines if " ERROR " in ln)
        print(f"run.log: {len(lines)} lines, {n_feat} FEAT lines, {n_err} ERROR lines")
        assert _MODEL is not None
        print(f"mock body model: {_MODEL.stats}")
        multi = {n: p.get("bodies") for n, p in rep["parts"].items() if p["kind"] == "printed" and p.get("bodies") != 1}
        if multi:
            print(f"FAIL: printed parts not a single body (leftover cutter / unjoined body): {multi}")
            rc = 1
        if missing:
            print(f"FAIL: report missing keys {missing}")
            rc = 1
        if bad_parts:
            print(f"FAIL: parts missing keys {bad_parts}")
            rc = 1
        if failed:
            print("FAIL: components failed")
            for ln in lines:
                if " ERROR " in ln:
                    print("   ", ln.rstrip())
            rc = 1
        if n_attempted == 0:
            print("FAIL: no components attempted")
            rc = 1
        if rc == 0:
            print("OK: mock run finished without component failures")
    finally:
        if args.keep:
            print(f"kept: {tmp}")
        else:
            shutil.rmtree(tmp, ignore_errors=True)
    return rc


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
