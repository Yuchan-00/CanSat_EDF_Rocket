"""v0.9 CAD-2 VTOL sizing (sizing/vtol_sizing.py) and the hand checks (sizing/vtol_strength.py): physics identities and
refusals, independent of the report files where possible."""
from __future__ import annotations

import math
import os
import sys

import pytest

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.normpath(os.path.join(HERE, ".."))
for _p in (HERE, os.path.join(ROOT, "cad")):
    if _p not in sys.path:
        sys.path.insert(0, _p)

import vtol_sizing as vs  # noqa: E402

G = 9.80665


def test_tw_rows_identities():
    rows = vs.tw_rows(1636.2, 2.0)
    assert [r["thrust_kgf"] for r in rows] == [2.0, 2.3, 2.6, 3.09]
    for r in rows:
        assert r["tw"] == pytest.approx(r["thrust_kgf"] / 1.6362)
        assert r["u_hover"] == pytest.approx(math.sqrt(1.0 / r["tw"]))
        assert r["mass_for_tw_1p3_g"] == pytest.approx(r["thrust_kgf"] * 1000.0 / 1.3)
    heavy = vs.tw_rows(2601.6, 2.0)
    assert heavy[0]["u_hover"] is None and not heavy[0]["hover_possible"]


def test_power_model_matches_the_tether_report_hover_point():
    # tether report key_results: VTOL 1650 g battery hover P1 = 883 W (P = 2264 (W / 30.30)^1.5)
    p = vs.power_w(1.650 * G, 30.302548499999997, 2264.0)
    assert p == pytest.approx(883.0, abs=1.0)


def test_pack_current_sag():
    i, v = vs.pack_current(880.0, 6, 0.018)
    assert v == pytest.approx(22.2 - i * 0.018, abs=1e-9) and i * v == pytest.approx(880.0)
    assert math.isnan(vs.pack_current(1e6, 6, 0.018)[0])


def test_tip_over_angles():
    t = vs.tip_over(250.0, 122.6)
    assert t["angle_toward_a_fin_deg"] == pytest.approx(math.degrees(math.atan2(122.6, 250.0)))
    assert t["angle_between_fins_deg"] < t["angle_toward_a_fin_deg"]


def test_load_report_refuses_foreign_params(tmp_path):
    p = tmp_path / "r.json"
    p.write_text('{"params_sha256": "0", "parts": {}}', encoding="utf-8")
    with pytest.raises(ValueError):
        vs.load_report(str(p), None, "1", [])
    with pytest.raises(FileNotFoundError):
        vs.load_report(str(tmp_path / "missing.json"), None, "1", [])
