"""Livery (user decisions 2026-11-04): white printed parts + the orange YUCHAN inlay on the avionics module.

Invariant under test: the logo polygons are the deterministic generator's output; Layout places them on the module of
BOTH configurations (rocket; vtol battery / tether / hybrid) and not on the KAIST preset; the CadQuery twin builds the
white body (with the 0.42 mm pocket) and the orange inlay so that their union is the plain module; check C36 passes on
the real design and FAILS on deliberately wrong ones (logo moved onto the vent slots / windows, a 0.84 mm inlay, a
stroke below the rule, the logo 0.1 mm too close to the module's end - reviewfix F6) - evidence that the check is
alive.  STEP colours follow params (white, orange, black pad).
"""
from __future__ import annotations

import copy
import json
import os
import subprocess
import sys

import pytest

_HERE = os.path.dirname(os.path.abspath(__file__))
if _HERE not in sys.path:
    sys.path.insert(0, _HERE)

import check_geometry as cg  # noqa: E402
import check_livery as ckl  # noqa: E402
import massprops as mp  # noqa: E402
import rocket_geometry as rg  # noqa: E402
from layout import PARAMS_PATH, Layout, load_params  # noqa: E402

ROOT = os.path.normpath(os.path.join(_HERE, "..", ".."))


def _layout(config: str | None = None, power: str | None = None, edit=None) -> Layout:
    p = load_params(PARAMS_PATH, None, config, power)
    if edit:
        p = copy.deepcopy(p)
        edit(p)
    return Layout(p)


@pytest.fixture(scope="module", params=["rocket", "vtol"])
def livery_build(request):
    L = _layout(request.param)
    white, inlay, plain = rg.avionics_module(L), rg.avionics_logo(L), rg.avionics_module(L, with_livery=False)
    return request.param, L, white, inlay, plain


def test_logo_generator_is_deterministic():
    r = subprocess.run([sys.executable, os.path.join(ROOT, "cad", "livery", "make_logo.py"), "--check"],
                       capture_output=True, text=True, encoding="utf-8", errors="replace", cwd=ROOT, timeout=300)
    assert r.returncode == 0, r.stdout + r.stderr
    assert r.stdout.count("identical") == 2


def test_livery_applies_to_both_configs_and_not_kaist():
    for cfg, power in (("rocket", None), ("vtol", None), ("vtol", "tether"), ("vtol", "hybrid")):
        L = _layout(cfg, power)
        assert L.has_livery, (cfg, power)
        assert L.livery_thetas == [67.0, 293.0]
        assert L.livery_depth == pytest.approx(0.42) and L.livery_r_in == pytest.approx(L.body_r - 0.42)
        assert L.livery_z0 == pytest.approx(L.z_avionics_bottom)
        assert "avionics_logo" in L.printed_parts
    k = Layout(load_params(PARAMS_PATH, "KAIST_90mm_12S"))
    assert not k.has_livery and "avionics_logo" not in k.printed_parts


def test_printer_colour_is_white_and_the_pad_stays_black():
    p = load_params()
    pr = p["printer"]
    assert pr["color"] == "Ivory White" and pr["color_hex"] == "#FFFFFF" and pr["color_code"] == "11100"
    assert pr["color_exceptions"]["launch_pad"]["bambu_name"] == "Charcoal"
    lv = p["livery"]
    assert lv["colours"]["logo"]["hex"] == "#F99963" and lv["colours"]["logo"]["code"] == "11300"
    assert rg.PRINTED_COLOR == (1.0, 1.0, 1.0)
    assert rg.LIVERY_COLOR == pytest.approx((0xF9 / 255, 0x99 / 255, 0x63 / 255))
    parts = rg.build_all(_layout("rocket"))
    assert parts["avionics_module"].color == rg.PRINTED_COLOR and parts["avionics_logo"].color == rg.LIVERY_COLOR
    assert parts["launch_pad"].color == rg.LAUNCH_PAD_COLOR != rg.PRINTED_COLOR
    assert all(p_.color == rg.PRINTED_COLOR for n, p_ in parts.items()
               if p_.kind == "printed" and n not in ("avionics_logo", "launch_pad"))


def test_white_body_plus_inlay_is_the_plain_module(livery_build):
    cfg, L, white, inlay, plain = livery_build
    vp, vw, vi = mp.precise_volume(plain), mp.precise_volume(white), mp.precise_volume(inlay)
    assert vw + vi == pytest.approx(vp, abs=ckl.LIVERY_VOL_TOL_MM3)
    assert inlay.cut(plain).Volume() == pytest.approx(0.0, abs=1e-6)          # the inlay lies in the plain wall
    assert len(inlay.Solids()) == len(L.livery_thetas) * len(L.livery_polys_vz)
    # 0.42 mm x the logo area on both copies (polygon area from the file's own summary)
    area = json.load(open(os.path.join(ROOT, L.livery_file), encoding="utf-8"))["summary_M"]["orange_area_mm2_per_copy"]
    assert vi == pytest.approx(2 * area * L.livery_depth, rel=0.003)
    # the logo sits on the module's outer skin, between its lower end and the spigot step
    bb = inlay.BoundingBox()
    assert bb.zmin > L.z_avionics_bottom + 3.0 - 1e-6 and bb.zmax < L.z_avionics_spigot_bottom - 3.0


def test_c36_passes_on_the_design(livery_build):
    cfg, L, white, inlay, plain = livery_build
    r = ckl.livery_check(L, white, inlay, plain)
    assert r["ok"], r["fails"]
    assert r["clearance_min_reported_mm"] >= 3.0
    assert r["polygons"]["min_stroke_mm"] >= 2.0 and r["polygons"]["smallest_island_mm2"] >= 8.0
    assert all(v["outside_material_mm3"] <= ckl.LIVERY_VOL_TOL_MM3 for v in r["clearance_containment"].values())
    # reviewfix F6: the void-band distance (every void near each copy, no volume tolerance) - the module's lower end
    for th in L.livery_thetas:
        vb = r["clearance_void_band"][f"{th:g}"]
        assert vb["min_mm"] >= 3.0 and vb["void_z_mm"][1] == pytest.approx(L.z_avionics_bottom, abs=0.01)
    # reviewfix F7: per-layer chords are reported (information)
    lc = r["layer_chords"]
    assert lc["layers_with_orange"] == 578 and lc["min_chord_mm"] < lc["line_w_mm"]


@pytest.mark.parametrize("config,theta,dz", [("rocket", 67.0, 8.0), ("vtol", 90.0, 0.0)])
def test_c36_fails_when_the_logo_meets_a_slot_or_window(config, theta, dz):
    """Negative control: the rocket logo raised 8 mm into the vent slot at 60 deg (its bottom is 5.9 mm above the logo
    top); one VTOL copy moved onto a window column (90 deg)."""
    def edit(p):
        p["livery"]["copies_theta_deg"] = [theta, 293.0]
        p["livery"]["z_offset_mm"] = dz
    L = _layout(config, edit=edit)
    r = ckl.livery_check(L, rg.avionics_module(L), rg.avionics_logo(L))
    assert not r["ok"]
    assert any(f.startswith("clearance") for f in r["fails"])
    assert r["clearance_containment"][f"{theta:g}"]["outside_material_mm3"] > 1.0


def test_c36_void_band_fails_a_small_encroachment():
    """reviewfix F6 negative control (independent review 2026-11-04): the logo moved 0.1 mm down (2.95 mm from the
    module's lower end).  The containment boolean alone lets it pass (0.0097 mm^3 outside < its 0.05 mm^3 tolerance);
    the void-band row (BRep distance, no volume tolerance) must FAIL it on both copies."""
    def edit(p):
        p["livery"]["z_offset_mm"] = -0.1
    L = _layout("vtol", edit=edit)
    r = ckl.livery_check(L, rg.avionics_module(L), rg.avionics_logo(L))
    assert not r["ok"]
    for th in L.livery_thetas:
        assert f"clearance < 3 mm at {th:g} deg (void band)" in r["fails"]
        assert r["clearance_void_band"][f"{th:g}"]["min_mm"] == pytest.approx(2.95, abs=0.005)
        assert r["clearance_containment"][f"{th:g}"]["outside_material_mm3"] <= ckl.LIVERY_VOL_TOL_MM3
    assert not any(f.startswith("clearance") and "void band" not in f for f in r["fails"])


def test_c36_fails_on_a_two_line_inlay():
    """Negative control: 0.84 mm (two lines) leaves 0.36 mm of white - the AMS probe's wrong case."""
    def edit(p):
        p["livery"]["inlay_depth_mm"] = 0.84
    L = _layout("vtol", edit=edit)
    r = ckl.livery_check(L, rg.avionics_module(L), rg.avionics_logo(L))
    assert "inlay depth" in r["fails"]


def test_c36_fails_on_a_stroke_rule_above_the_design():
    """Negative control: the same logo judged with a stroke rule above its measured 2.20 mm."""
    def edit(p):
        p["livery"]["check"]["min_stroke_mm"] = 2.5
    L = _layout("vtol", edit=edit)
    r = ckl.livery_check(L, rg.avionics_module(L), rg.avionics_logo(L))
    assert r["fails"] == ["min stroke"]


def test_c36_is_na_for_kaist():
    L = Layout(load_params(PARAMS_PATH, "KAIST_90mm_12S"))
    parts = {"propulsion_module": rg.Part("propulsion_module", "printed", "propulsion_module", rg._box(0, 1, 0, 1, 0, 1),
                                          "propulsion_module")}
    c = cg.Checker(L, parts, {}).c36_livery()
    assert c.status == cg.NA
