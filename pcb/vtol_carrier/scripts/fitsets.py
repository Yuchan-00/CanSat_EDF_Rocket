"""Who fits what (rev 3): the ``fit`` attribute of every spec part and the JLC BOM grouping derived from it.

Invariant this module enforces: the JLC BOM, the CPL and every "fitted by JLC" count are derived ONLY from
``part["fit"]`` (one of FITS) and ``part["bom"]`` in spec/carrier_spec.json - never from a refdes list typed in a
script, never from ``populate``:

* ``jlc``           fitted by JLCPCB, in the BOM and the CPL
* ``jlc_optional``  footprint kept, fitted by the user by default; in the BOM / CPL only of the ``*_with_mcu`` pair
* ``user``          fitted by the user after delivery (e.g. the GY-521 pin strip in J1); in neither file
* ``none``          bare plated holes / DNP / copper-only features (net tie, solder jumper); in neither file

Dependency-free on purpose (json-level dicts only) so build_spec.py, common.py, rev3_checks.py and the export step
import the same functions.
"""
from __future__ import annotations

import re

FITS = ("jlc", "jlc_optional", "user", "none")


def natural(text: str) -> list:
    """Sort key: 'C10' after 'C9'."""
    return [int(t) if t.isdigit() else t for t in re.split(r"(\d+)", text)]


def fit_of(part: dict) -> str:
    """The part's fit; a missing or unknown value is an error, never a default."""
    fit = part.get("fit")
    if fit not in FITS:
        raise ValueError(f"{part.get('refdes')}: fit {fit!r} is not one of {FITS}")
    return fit


def jlc_fitted(part: dict, with_mcu: bool = False) -> bool:
    """True when JLC fits the part in the chosen file pair (main pair, or the ``*_with_mcu`` pair)."""
    fit = fit_of(part)
    return fit == "jlc" or (with_mcu and fit == "jlc_optional")


def populated(part: dict) -> bool:
    """True when the footprint carries a part on the finished vehicle (spec ``populate`` is exactly True).

    False = DNP in the schematic / board (bare holes, optional parts). This is NOT the BOM filter: U1..U4 and J1 are
    populated (by the user) but not fitted by JLC - use jlc_fitted for the BOM / CPL."""
    return part["populate"] is True


def fit_sets(parts: list[dict]) -> dict[str, list[str]]:
    """{fit: [refdes, ...]} with every fit present (possibly empty), refdes in natural order."""
    out: dict[str, list[str]] = {f: [] for f in FITS}
    for p in parts:
        out[fit_of(p)].append(p["refdes"])
    return {f: sorted(refs, key=natural) for f, refs in out.items()}


def bom_groups(parts: list[dict], with_mcu: bool = False) -> list[dict]:
    """JLC BOM lines for the chosen file pair: one line per LCSC code, in the order of the spec's line numbers.

    Each line: {line, jlc_code, comment, footprint (KiCad lib:name), footprint_name (JLC column), designators,
    qty, assembly ('SMT' | 'THT'), mpn, library_type}. Raises when a JLC-fitted part has no code or when one code
    is used with two comments / footprints / assembly processes (JLC needs one line per code)."""
    lines: dict[str, dict] = {}
    for p in parts:
        if not jlc_fitted(p, with_mcu):
            continue
        bom = p.get("bom") or {}
        code = bom.get("jlc_code")
        if not code:
            raise ValueError(f"{p['refdes']}: fitted by JLC but has no jlc_code")
        ln = lines.setdefault(code, {
            "line": bom.get("line"), "jlc_code": code, "comment": bom["comment"], "footprint": p["footprint"],
            "footprint_name": p["footprint"].split(":", 1)[1], "designators": [], "qty": 0,
            "assembly": bom["assembly"], "mpn": bom.get("mpn"), "library_type": bom.get("library_type")})
        for key, val in (("comment", bom["comment"]), ("footprint", p["footprint"]), ("assembly", bom["assembly"])):
            if ln[key] != val:
                raise ValueError(f"{p['refdes']}: {code} used with two different {key}: {ln[key]!r} / {val!r}")
        ln["designators"].append(p["refdes"])
        ln["qty"] += 1
    out = sorted(lines.values(), key=lambda d: (d["line"] is None, d["line"] or 0, natural(d["jlc_code"])))
    for ln in out:
        ln["designators"].sort(key=natural)
    return out


def summary(parts: list[dict], with_mcu: bool = False) -> dict:
    """Counts the reports quote: parts per fit, JLC-fitted parts, BOM lines split SMT / THT."""
    groups = bom_groups(parts, with_mcu)
    sets = fit_sets(parts)
    return {"parts": len(parts), "fit_counts": {f: len(r) for f, r in sets.items()},
            "jlc_fitted_parts": sum(g["qty"] for g in groups), "bom_lines": len(groups),
            "bom_lines_smt": sum(1 for g in groups if g["assembly"] == "SMT"),
            "bom_lines_tht": sum(1 for g in groups if g["assembly"] == "THT"),
            "with_mcu": with_mcu}
