"""Tests of tools/budget_trade.py and tools/make_budget_preset.py after the v0.6.3 electronics split.

params.json electronics.fc became electronics.nav (the sensor board, same spot) and electronics.fcu is new (v0.6 added
.com and .radio_log, and misc went 60 -> 40 g): the trade study counts every electronics entry with a mass_g and refuses
the old 'fc' key; the budget preset's overrides are the ones params.json holds (``--check`` is read only - no test
writes params.json).

Run: ``.venv/Scripts/python.exe -m pytest tools/test_budget_trade.py -q -p no:cacheprovider``
"""
from __future__ import annotations

import copy
import json
import os
import sys

import pytest

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import budget_trade as bt  # noqa: E402
import make_budget_preset as mbp  # noqa: E402

with open(mbp.PARAMS, "r", encoding="utf-8") as _f:
    P = json.load(_f)


def test_electronics_masses_counts_every_board_and_refuses_fc():
    el = P["electronics"]
    m = bt.electronics_masses(el)
    assert {"esc", "nav", "fcu", "com", "radio_log", "misc"} <= set(m)
    assert m == {k: float(v["mass_g"]) for k, v in el.items() if isinstance(v, dict) and "mass_g" in v}
    assert "release_servo_count" not in m                                  # not a board
    old = copy.deepcopy(el)
    old["fc"] = old.pop("nav")
    with pytest.raises(KeyError, match="renamed it electronics.nav"):
        bt.electronics_masses(old)
    no_nav = {k: v for k, v in el.items() if k != "nav"}
    with pytest.raises(KeyError, match="nav"):
        bt.electronics_masses(no_nav)


def test_evaluate_current_budget_design():
    """The current wall / envelope candidate evaluates (no KeyError 'fc') and its non-printed mass carries the v0.6.3
    boards with params.json's masses."""
    name, over = next((n, o) for n, o in bt.candidates() if n == "wall 1.2 / envelope h 150")
    r = bt.evaluate(name, over)
    npg = r["non_printed_g"]
    for k in ("nav", "fcu", "com", "radio_log", "misc", "esc"):
        assert npg[k] == round(P["electronics"][k]["mass_g"]), k
    assert "fc" not in npg and r["dry_g"] > r["printed_g"]


def test_budget_overrides_match_params_and_check_is_read_only(capsys):
    before = open(mbp.PARAMS, "rb").read()
    assert mbp.check(copy.deepcopy(P)) == 0
    assert "OK" in capsys.readouterr().out
    assert open(mbp.PARAMS, "rb").read() == before
    over = mbp.budget_overrides()
    assert "fc" not in over["electronics"] and over["electronics"]["nav"]["mass_g"] == 40.0
    assert over["electronics"]["misc"]["mass_g"] == bt.BUDGET_COMMON["electronics"]["misc"]["mass_g"] == 40.0


def test_override_diffs_names_every_difference():
    p = copy.deepcopy(P)
    p["electronics"]["misc"]["mass_g"] = 60.0
    del p["electronics"]["nav"]
    diffs = mbp.override_diffs(p, mbp.budget_overrides())
    assert any(d.startswith("electronics.misc.mass_g: budget value 40.0, params.json 60.0") for d in diffs)
    assert "electronics.nav: missing in params.json" in diffs
    assert mbp.check(p) == 1
