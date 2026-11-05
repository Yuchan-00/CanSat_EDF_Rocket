"""Presentation images for the VTOL carrier (deliver step).

Invariant: this script only READS the finished board / schematic exports and writes new
images under out/presentation/. It never touches the manifest-hashed files in out/
(render_*.png, schematic/*.svg, *.pdf), so out/manifest.json stays valid.

Run with KiCad's Python (it has wx and PIL):
  C:/Users/yc012/AppData/Local/Programs/KiCad/10.0/bin/python.exe scripts/make_presentation.py
"""
from __future__ import annotations

import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
OUT = ROOT / "out"
PRES = OUT / "presentation"
PCB = ROOT / "kicad" / "vtol_carrier.kicad_pcb"
SCH_DIR = OUT / "schematic"
K = r"C:\Users\yc012\AppData\Local\Programs\KiCad\10.0\bin\kicad-cli.exe"

RENDERS = {
    "pcb_top_2400.png": ["--side", "top"],
    "pcb_bottom_2400.png": ["--side", "bottom"],
    "pcb_iso_front_2400.png": ["--side", "top", "--rotate", "-45,0,30", "--perspective", "--zoom", "0.9"],
    "pcb_iso_back_2400.png": ["--side", "top", "--rotate", "-50,0,-150", "--perspective", "--zoom", "0.9"],
}

# sheet order for the overview montage (root sheet first)
SHEETS = ["vtol_carrier", "vtol_carrier-Power", "vtol_carrier-FCU", "vtol_carrier-NAV",
          "vtol_carrier-COM", "vtol_carrier-GNS", "vtol_carrier-Level_shift",
          "vtol_carrier-IO_connectors"]


def run(cmd: list[str]) -> subprocess.CompletedProcess:
    return subprocess.run(cmd, capture_output=True, text=True, encoding="utf-8", errors="replace")


def renders() -> dict:
    res = {}
    for name, extra in RENDERS.items():
        r = run([K, "pcb", "render", "-w", "2400", "-h", "1800", "--quality", "high",
                 "--background", "opaque", *extra, "-o", str(PRES / name), str(PCB)])
        res[name] = {"exit": r.returncode, "exists": (PRES / name).exists()}
    return res


EDGE = r"C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe"


def sheets_to_png(width: int = 3000) -> dict:
    """Rasterise each schematic SVG with headless Edge (KiCad's wx has no nanosvg build).

    A tiny HTML wrapper (in a temp dir, outside the repo) shows the local SVG at a fixed
    pixel size on a white page; Edge takes a screenshot of exactly that window size.
    """
    import re
    import tempfile
    res = {}
    tmp = Path(tempfile.mkdtemp(prefix="vtol_sch_"))
    for s in SHEETS:
        svg = SCH_DIR / f"{s}.svg"
        head = svg.read_text(encoding="utf-8", errors="replace")[:2000]
        m = re.search(r'width="([\d.]+)mm" height="([\d.]+)mm"', head)
        w_mm, h_mm = float(m.group(1)), float(m.group(2))
        h = int(round(width * h_mm / w_mm))
        html = tmp / f"{s}.html"
        html.write_text(
            f'<!doctype html><html><body style="margin:0;background:#fff">'
            f'<img src="{svg.as_uri()}" style="display:block;width:{width}px;height:{h}px">'
            f'</body></html>', encoding="utf-8")
        dst = PRES / f"sch_{s.replace('vtol_carrier-', '').replace('vtol_carrier', 'root')}.png"
        r = run([EDGE, "--headless=new", "--disable-gpu", "--hide-scrollbars",
                 f"--user-data-dir={tmp / 'edge_profile'}", "--force-device-scale-factor=1",
                 f"--window-size={width},{h}", f"--screenshot={dst}", html.as_uri()])
        res[dst.name] = {"w": width, "h": h, "exit": r.returncode, "exists": dst.exists()}
    return res


def montage(sheet_pngs: list[Path]) -> dict:
    from PIL import Image, ImageDraw
    tiles = [Image.open(p).convert("RGB") for p in sheet_pngs]
    tw = 1200
    scaled = [t.resize((tw, int(tw * t.height / t.width)), Image.LANCZOS) for t in tiles]
    th = max(t.height for t in scaled)
    cols, gap, label_h = 4, 24, 48
    rows = (len(scaled) + cols - 1) // cols
    W = cols * tw + (cols + 1) * gap
    H = rows * (th + label_h) + (rows + 1) * gap
    canvas = Image.new("RGB", (W, H), (235, 235, 240))
    d = ImageDraw.Draw(canvas)
    from PIL import ImageFont
    font = ImageFont.load_default(size=32)
    for i, (t, p) in enumerate(zip(scaled, sheet_pngs)):
        r, c = divmod(i, cols)
        x = gap + c * (tw + gap)
        y = gap + r * (th + label_h + gap)
        d.text((x + 8, y + 6), p.stem.replace("sch_", "sheet: "), fill=(20, 20, 20), font=font)
        canvas.paste(t, (x, y + label_h))
        d.rectangle([x - 1, y + label_h - 1, x + tw, y + label_h + t.height], outline=(120, 120, 120))
    dst = PRES / "schematic_overview.png"
    canvas.save(dst)
    return {"file": dst.name, "w": W, "h": H, "tiles": len(scaled)}


def main() -> int:
    PRES.mkdir(parents=True, exist_ok=True)
    report = {"renders": renders()}
    report["sheets"] = sheets_to_png()
    pngs = [PRES / f"sch_{s.replace('vtol_carrier-', '').replace('vtol_carrier', 'root')}.png" for s in SHEETS]
    report["montage"] = montage(pngs)
    (PRES / "presentation_log.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8", newline="\n")
    print(json.dumps(report, indent=2))
    ok = all(v["exit"] == 0 and v["exists"] for v in report["renders"].values())
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
