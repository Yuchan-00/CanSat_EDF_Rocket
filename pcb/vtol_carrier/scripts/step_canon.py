"""Canonical entity numbering for the kicad-cli STEP export (redesign2 fix stage).

    python scripts/step_canon.py <file.step> [--check]

Invariant: a pure RENUMBERING. Every DATA entity of the file is kept with its exact parameter text (strings untouched;
only white space outside strings is removed); only the '#n' instance numbers and the order of the entities change.
The new number of an entity depends on the content of the graph only, never on the order kicad-cli wrote it in:
colour refinement (the entity text with references blanked, then repeatedly the colours of the entities it refers to
in order and of the entities that refer to it with the argument position), ranks taken from SORTED signatures, and
individualisation of the first tied class until every entity has its own colour. Why: kicad-cli 10.0.6 'pcb export
step' writes the same entities in a different order on every run (M, 2026-11-11 and redesign2/pipeline/steptest), so
the plan's STEP byte comparison (scripts/determinism.py, NORMALISED, unchanged) could never pass.

Before the file is replaced the result is proved to be the same graph: it is parsed again and every new entity text,
with its references mapped back through the numbering, must equal the old entity text (same count, bijective map).
If that proof fails the export is left as kicad-cli wrote it and the caller gets ok False. The HEADER section
(FILE_NAME with the export time) is copied unchanged.
"""
from __future__ import annotations

import hashlib
import sys
from pathlib import Path


def _strip_ws(body: bytes) -> bytes:
    """remove white space outside quoted strings ('' is an escaped quote inside a string)"""
    out = bytearray()
    q = False
    for ch in body:
        if ch == 0x27:  # '
            q = not q
            out.append(ch)
        elif not q and ch in b" \t\r\n":
            continue
        else:
            out.append(ch)
    return bytes(out)


def _split_entities(data: bytes) -> list[tuple[int, bytes]]:
    """DATA section -> [(instance number, body after '=' without the final ';')]"""
    ents = []
    i, n = 0, len(data)
    while i < n:
        while i < n and data[i] in b" \t\r\n":
            i += 1
        if i >= n:
            break
        if data[i] != 0x23:  # '#'
            raise ValueError(f"unexpected byte at {i}: {data[i:i + 30]!r}")
        j = data.index(b"=", i)
        num = int(data[i + 1:j].strip())
        k = j + 1
        q = False
        while True:
            ch = data[k]
            if ch == 0x27:
                q = not q
            elif ch == 0x3B and not q:  # ';'
                break
            k += 1
        ents.append((num, _strip_ws(data[j + 1:k])))
        i = k + 1
    return ents


def _tokens(body: bytes) -> tuple[list[bytes], list[int]]:
    """split a body into literal pieces and references: body = lit[0] + #ref[0] + lit[1] + ... + lit[-1]"""
    lits, refs = [], []
    cur = bytearray()
    q = False
    i, n = 0, len(body)
    while i < n:
        ch = body[i]
        if ch == 0x27:
            q = not q
            cur.append(ch)
            i += 1
        elif ch == 0x23 and not q:
            j = i + 1
            while j < n and 0x30 <= body[j] <= 0x39:
                j += 1
            lits.append(bytes(cur))
            cur = bytearray()
            refs.append(int(body[i + 1:j]))
            i = j
        else:
            cur.append(ch)
            i += 1
    lits.append(bytes(cur))
    return lits, refs


def _parse(text: bytes):
    head, sep, rest = text.partition(b"DATA;")
    if not sep:
        raise ValueError("no DATA section")
    data, sep2, tail = rest.partition(b"ENDSEC;")
    if not sep2:
        raise ValueError("no ENDSEC after DATA")
    return head + sep, data, sep2 + tail


def _refine(n: int, init: list[int], kids: list[list[int]], parents: list[list[tuple[int, int]]]) -> list[int]:
    col = init
    k = len(set(col))
    while True:
        sig = [(col[v], tuple(col[c] for c in kids[v]), tuple(sorted((col[p], pos) for p, pos in parents[v])))
               for v in range(n)]
        ranks = {s: r for r, s in enumerate(sorted(set(sig)))}
        new = [ranks[s] for s in sig]
        k2 = len(ranks)
        if k2 == k:
            return new
        col, k = new, k2


def canonical(text: bytes) -> tuple[bytes, dict]:
    head, data, tail = _parse(text)
    ents = _split_entities(data)
    nums = [e[0] for e in ents]
    if len(set(nums)) != len(nums):
        raise ValueError("duplicate instance numbers")
    idx = {num: i for i, num in enumerate(nums)}
    lits, kids = [], []
    for _num, body in ents:
        lit, refs = _tokens(body)
        lits.append(lit)
        kids.append([idx[r] for r in refs])
    n = len(ents)
    parents: list[list[tuple[int, int]]] = [[] for _ in range(n)]
    for v in range(n):
        for pos, c in enumerate(kids[v]):
            parents[c].append((v, pos))
    blank = [hashlib.sha256(b"#".join(l)).digest() for l in lits]
    order = {h: r for r, h in enumerate(sorted(set(blank)))}
    col = _refine(n, [order[h] for h in blank], kids, parents)
    individualised = 0
    while len(set(col)) < n:
        cnt: dict[int, int] = {}
        for c in col:
            cnt[c] = cnt.get(c, 0) + 1
        tied = min(c for c, m in cnt.items() if m > 1)
        pick = next(v for v in range(n) if col[v] == tied)  # members of a refined class are taken as interchangeable
        col = [2 * c + (1 if (c == tied and v != pick) else 0) for v, c in enumerate(col)]
        col = _refine(n, col, kids, parents)
        individualised += 1
    new_num = [c + 1 for c in col]
    rows = sorted(range(n), key=lambda v: new_num[v])
    lines = []
    for v in rows:
        lit = lits[v]
        parts = [lit[0]]
        for r, l in zip(kids[v], lit[1:]):
            parts.append(b"#%d" % new_num[r])
            parts.append(l)
        lines.append(b"#%d=%s;" % (new_num[v], b"".join(parts)))
    out = head + b"\n" + b"\n".join(lines) + b"\n" + tail
    return out, {"entities": n, "individualised_classes": individualised, "old_to_new": dict(zip(nums, new_num))}


def prove_same_graph(old: bytes, new: bytes, old_to_new: dict[int, int]) -> str | None:
    """None = the new file is the old graph renumbered by old_to_new; otherwise the first difference"""
    h0, d0, t0 = _parse(old)
    h1, d1, t1 = _parse(new)
    if h0.rstrip() != h1.rstrip() or t0.strip() != t1.strip():
        return "header / trailer differ"
    e0 = dict(_split_entities(d0))
    e1 = dict(_split_entities(d1))
    if len(e0) != len(e1) or len(set(old_to_new.values())) != len(old_to_new) or set(old_to_new) != set(e0):
        return "entity count / numbering not bijective"
    for o, body in e0.items():
        lit0, ref0 = _tokens(body)
        nb = e1.get(old_to_new[o])
        if nb is None:
            return f"#{o} missing"
        lit1, ref1 = _tokens(nb)
        if lit0 != lit1 or [old_to_new[r] for r in ref0] != ref1:
            return f"#{o} differs"
    return None


def canonicalise_file(path: Path) -> dict:
    old = path.read_bytes()
    try:
        new, info = canonical(old)
    except (ValueError, KeyError, IndexError) as exc:
        return {"ok": False, "why": f"parse failed: {exc}", "sha256_kicad_cli": hashlib.sha256(old).hexdigest()}
    bad = prove_same_graph(old, new, info["old_to_new"])
    res = {"ok": bad is None, "entities": info["entities"], "individualised_classes": info["individualised_classes"],
           "sha256_kicad_cli": hashlib.sha256(old).hexdigest(), "proof": bad or "same graph, renumbered (every entity checked)"}
    if bad is None:
        path.write_bytes(new)
        res["sha256_canonical"] = hashlib.sha256(new).hexdigest()
    return res


if __name__ == "__main__":
    import json
    p = Path(sys.argv[1])
    if "--check" in sys.argv:
        new, info = canonical(p.read_bytes())
        print(json.dumps({"entities": info["entities"], "individualised_classes": info["individualised_classes"],
                          "proof": prove_same_graph(p.read_bytes(), new, info["old_to_new"]) or "ok",
                          "sha256_canonical": hashlib.sha256(new).hexdigest()}))
    else:
        print(json.dumps(canonicalise_file(p)))
