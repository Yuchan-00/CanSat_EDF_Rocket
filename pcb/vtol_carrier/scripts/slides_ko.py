"""Korean slide helper for the rev 3 mounting pictures (plan section 11, views e1-e4).

Invariant this module enforces: a caption, call-out or table that rests on assumed dimensions (tag A in
spec/module_dimensions.json, or an illustrative route) always carries the word '가정' - `caption_text` appends it when
the caller marks the item `assumed` and the text does not already contain it, and a slide that holds any assumed item
gets the footer note that explains the word.  Nothing here measures or verifies anything: it only composes pictures
that other scripts rendered.

Pure PIL (works under KiCad's python and under the CAD venv), 1920 x 1080, Malgun Gothic, no timestamps in the PNG
(same input -> same file).

    from slides_ko import Panel, Table, Callout, compose_slide, annotate
    compose_slide("GY-521 장착", [Panel("out/render/c1_gy521_on_J1.png", "J1 구멍 8개에 핀 스트립을 납땜", assumed=True)],
                  "out/render/e2_gy521.png", table=Table("J1 핀", ["핀", "신호"], [["1", "VCC"], ...]))

Demo (proves the helper on existing pictures):
    python -B -X utf8 scripts/slides_ko.py --demo OUT.png --images A.png B.png
"""
from __future__ import annotations

import argparse
import os
from dataclasses import dataclass, field
from pathlib import Path
from typing import Sequence, Union

from PIL import Image, ImageDraw, ImageFont

W, H = 1920, 1080
ASSUMED_WORD = "가정"
ASSUMED_NOTE = "'가정' 표시 = 문서로 확인하지 못한 치수나 경로로 그린 부분입니다. 실물을 받은 뒤 다시 확인해야 합니다."
FONT_DIRS = (Path("C:/Windows/Fonts"), Path("/usr/share/fonts/truetype/nanum"), Path("/usr/share/fonts/opentype/noto"))
FONT_REGULAR = ("malgun.ttf", "NanumGothic.ttf", "NotoSansCJK-Regular.ttc")
FONT_BOLD = ("malgunbd.ttf", "NanumGothicBold.ttf", "NotoSansCJK-Bold.ttc")

BG = (246, 247, 249)
INK = (24, 28, 36)
MUTED = (88, 96, 110)
TITLE_BG = (22, 38, 66)
TITLE_INK = (255, 255, 255)
ACCENT = (200, 30, 30)            # the colour of every '가정' mark
PANEL_EDGE = (200, 204, 212)
TABLE_HEAD = (226, 231, 240)
BADGE_BG = {"delivered": (30, 110, 60), "completed": (20, 80, 160), None: (70, 76, 90)}

TITLE_H, FOOT_H, MARGIN, GAP = 104, 56, 36, 24

ImageLike = Union[str, os.PathLike, Image.Image]


def font_path(bold: bool = False) -> Path:
    """Malgun Gothic (the font the task names); other Korean fonts only so the helper also runs off this PC."""
    env = os.environ.get("SLIDES_KO_FONT_BOLD" if bold else "SLIDES_KO_FONT")
    if env and Path(env).is_file():
        return Path(env)
    for name in (FONT_BOLD if bold else FONT_REGULAR):
        for d in FONT_DIRS:
            if (d / name).is_file():
                return d / name
    raise FileNotFoundError("no Korean font found (expected C:/Windows/Fonts/malgun.ttf); set SLIDES_KO_FONT")


def font(size: int, bold: bool = False) -> ImageFont.FreeTypeFont:
    return ImageFont.truetype(str(font_path(bold)), size)


def caption_text(text: str, assumed: bool) -> str:
    """The text as it is drawn: an assumed item always contains the word '가정'."""
    text = " ".join(text.split())
    if assumed and ASSUMED_WORD not in text:
        text = f"{text} ({ASSUMED_WORD})"
    return text


@dataclass
class Panel:
    """One picture with its caption.  `badge`: 'delivered' -> 'JLC에서 받은 상태', 'completed' -> '직접 장착한 뒤'."""
    image: ImageLike
    caption: str
    assumed: bool = False
    badge: str | None = None
    crop: tuple[float, float, float, float] | None = None       # fractions (left, top, right, bottom) of the picture


@dataclass
class Table:
    """Small table beside the pictures (pin table, who solders what)."""
    title: str
    header: Sequence[str]
    rows: Sequence[Sequence[str]]
    assumed: bool = False
    note: str | None = None
    col_weights: Sequence[float] | None = None


@dataclass
class Callout:
    """Label with a leader line: `target` is the pixel the line points at, `box` the top-left corner of the label."""
    text: str
    target: tuple[float, float]
    box: tuple[float, float]
    assumed: bool = False


BADGE_TEXT = {"delivered": "JLC에서 받은 상태", "completed": "직접 장착한 뒤"}


def _open(image: ImageLike) -> Image.Image:
    if isinstance(image, Image.Image):
        return image.convert("RGB")
    with Image.open(image) as im:
        return im.convert("RGB")


def wrap(draw: ImageDraw.ImageDraw, text: str, fnt: ImageFont.FreeTypeFont, max_w: float) -> list[str]:
    """Greedy wrap: breaks at spaces, and inside a word only when the word alone is wider than the line."""
    lines: list[str] = []
    for para in text.split("\n"):
        cur = ""
        for word in para.split(" "):
            trial = word if not cur else f"{cur} {word}"
            if draw.textlength(trial, font=fnt) <= max_w:
                cur = trial
                continue
            if cur:
                lines.append(cur)
            cur = ""
            for ch in word:                                   # a single word longer than the line: character wrap
                if draw.textlength(cur + ch, font=fnt) > max_w and cur:
                    lines.append(cur)
                    cur = ch
                else:
                    cur += ch
        lines.append(cur)
    return lines


def fit(img: Image.Image, box_w: int, box_h: int) -> Image.Image:
    """Whole picture inside the box, aspect kept (never cropped, never stretched)."""
    s = min(box_w / img.width, box_h / img.height)
    size = (max(1, round(img.width * s)), max(1, round(img.height * s)))
    return img.resize(size, Image.LANCZOS)


def _line_h(fnt: ImageFont.FreeTypeFont) -> int:
    return int(fnt.size * 1.38)


def _draw_lines(draw: ImageDraw.ImageDraw, xy: tuple[float, float], lines: Sequence[str], fnt: ImageFont.FreeTypeFont,
                fill: tuple[int, int, int], mark: bool = True) -> None:
    """Text lines; the word '가정' is drawn in the accent colour so it cannot be overlooked."""
    x0, y = xy
    for line in lines:
        x = x0
        parts = line.split(ASSUMED_WORD) if mark else [line]
        for k, part in enumerate(parts):
            if k:
                draw.text((x, y), ASSUMED_WORD, font=fnt, fill=ACCENT)
                x += draw.textlength(ASSUMED_WORD, font=fnt)
            draw.text((x, y), part, font=fnt, fill=fill)
            x += draw.textlength(part, font=fnt)
        y += _line_h(fnt)


def _draw_table(img: Image.Image, t: Table, x: int, y: int, w: int, h: int) -> None:
    draw = ImageDraw.Draw(img)
    f_title, f_cell, f_note = font(30, True), font(24), font(21)
    title = caption_text(t.title, t.assumed)
    lines = wrap(draw, title, f_title, w)
    _draw_lines(draw, (x, y), lines, f_title, INK)
    y += len(lines) * _line_h(f_title) + 8
    n = len(t.header)
    weights = list(t.col_weights) if t.col_weights else [1.0] * n
    if len(weights) != n or any(len(r) != n for r in t.rows):
        raise ValueError("Table: header, rows and col_weights must have the same number of columns")
    widths = [w * wt / sum(weights) for wt in weights]
    pad = 8
    for ri, row in enumerate([list(t.header)] + [list(r) for r in t.rows]):
        cells = [wrap(draw, str(c), f_cell, cw - 2 * pad) for c, cw in zip(row, widths)]
        rh = max(len(c) for c in cells) * _line_h(f_cell) + 2 * pad
        if y + rh > y + h and ri > 0 and y + rh > H - FOOT_H - MARGIN:
            raise ValueError(f"Table '{t.title}': does not fit on the slide (row {ri})")
        cx = x
        for c, cw in zip(cells, widths):
            draw.rectangle([cx, y, cx + cw, y + rh], fill=TABLE_HEAD if ri == 0 else (255, 255, 255), outline=PANEL_EDGE)
            _draw_lines(draw, (cx + pad, y + pad), c, f_cell, INK)
            cx += cw
        y += rh
    if t.note:
        y += 10
        _draw_lines(draw, (x, y), wrap(draw, caption_text(t.note, False), f_note, w), f_note, MUTED)


def compose_slide(title: str, panels: Sequence[Panel], out: str | os.PathLike, subtitle: str | None = None,
                  table: Table | None = None, footer: str | None = None, cols: int | None = None,
                  table_w: int = 600) -> Path:
    """One 1920 x 1080 slide: title bar, the pictures in a grid with captions under them, optional table on the
    right, footer.  Returns the path written.  Raises when a caption does not fit instead of cutting it."""
    if not panels:
        raise ValueError("compose_slide: no panels")
    img = Image.new("RGB", (W, H), BG)
    draw = ImageDraw.Draw(img)
    f_title, f_sub, f_cap, f_badge, f_foot = font(46, True), font(26), font(27), font(24, True), font(21)

    draw.rectangle([0, 0, W, TITLE_H], fill=TITLE_BG)
    draw.text((MARGIN, 18 if subtitle else 26), title, font=f_title, fill=TITLE_INK)
    if subtitle:
        tw = draw.textlength(title, font=f_title)
        draw.text((MARGIN + tw + 28, 40), subtitle, font=f_sub, fill=(196, 206, 226))

    any_assumed = any(p.assumed or ASSUMED_WORD in p.caption for p in panels) or bool(table and table.assumed)
    foot = footer or ""
    if any_assumed:
        foot = f"{foot}   {ASSUMED_NOTE}".strip()
    draw.rectangle([0, H - FOOT_H, W, H], fill=(232, 235, 240))
    _draw_lines(draw, (MARGIN, H - FOOT_H + 14), wrap(draw, foot, f_foot, W - 2 * MARGIN)[:1], f_foot, MUTED)

    x0, y0 = MARGIN, TITLE_H + MARGIN
    area_w = W - 2 * MARGIN - ((table_w + GAP) if table else 0)
    area_h = H - FOOT_H - MARGIN - y0
    n = len(panels)
    cols = cols or (n if n <= 3 else (n + 1) // 2)
    rows = (n + cols - 1) // cols
    cell_w = (area_w - GAP * (cols - 1)) // cols
    cell_h = (area_h - GAP * (rows - 1)) // rows
    for i, p in enumerate(panels):
        cx = x0 + (i % cols) * (cell_w + GAP)
        cy = y0 + (i // cols) * (cell_h + GAP)
        cap = wrap(draw, caption_text(p.caption, p.assumed), f_cap, cell_w - 24)
        cap_h = len(cap) * _line_h(f_cap) + 20
        pic_h = cell_h - cap_h
        if pic_h < 120:
            raise ValueError(f"compose_slide: caption of panel {i + 1} leaves no room for the picture")
        draw.rectangle([cx, cy, cx + cell_w, cy + cell_h], fill=(255, 255, 255), outline=PANEL_EDGE)
        pic = _open(p.image)
        if p.crop:
            l, t, r, b = p.crop
            pic = pic.crop((round(l * pic.width), round(t * pic.height), round(r * pic.width), round(b * pic.height)))
        pic = fit(pic, cell_w - 2, pic_h - 2)
        img.paste(pic, (cx + 1 + (cell_w - 2 - pic.width) // 2, cy + 1 + (pic_h - 2 - pic.height) // 2))
        if p.badge:
            text = BADGE_TEXT.get(p.badge, p.badge)
            bw = int(draw.textlength(text, font=f_badge)) + 28
            draw.rectangle([cx + 12, cy + 12, cx + 12 + bw, cy + 12 + 42], fill=BADGE_BG.get(p.badge, BADGE_BG[None]))
            draw.text((cx + 26, cy + 17), text, font=f_badge, fill=(255, 255, 255))
        draw.line([cx, cy + pic_h, cx + cell_w, cy + pic_h], fill=PANEL_EDGE)
        _draw_lines(draw, (cx + 12, cy + pic_h + 10), cap, f_cap, INK)
    if table:
        _draw_table(img, table, x0 + area_w + GAP, y0, table_w, area_h)
    out = Path(out)
    out.parent.mkdir(parents=True, exist_ok=True)
    img.save(out, format="PNG")
    return out


def annotate(image: ImageLike, callouts: Sequence[Callout], out: str | os.PathLike | None = None, size: int = 26,
             banner: str | None = None, banner_assumed: bool = False) -> Image.Image:
    """Call-outs with leader lines on a picture (overview slide, vehicle scene) and an optional banner along the
    bottom.  Returns the image; writes it when `out` is given."""
    img = _open(image)
    draw = ImageDraw.Draw(img)
    fnt = font(size)
    bh, banner_lines = 0, []
    if banner:
        banner_lines = wrap(draw, caption_text(banner, banner_assumed), fnt, img.width - 2 * MARGIN)
        bh = len(banner_lines) * _line_h(fnt) + 20
    y_floor = img.height - bh - 8                                # labels stay above the banner
    items = []
    for c in callouts:
        lines = [" ".join(s.split()) for s in c.text.split("\n") if s.strip()]      # '\n' in the text = line break
        text = " ".join(lines)
        if c.assumed and ASSUMED_WORD not in text:
            lines[-1] = f"{lines[-1]} ({ASSUMED_WORD})"
            text = " ".join(lines)
        bw = max(draw.textlength(s, font=fnt) for s in lines) + 20
        bhh = len(lines) * _line_h(fnt) + 12
        if bw > img.width - 16 or bhh > y_floor - 8:
            raise ValueError(f"call-out does not fit the picture: {text!r}")
        bx = min(max(c.box[0], 8), img.width - 8 - bw)           # the label never leaves the picture
        by = min(max(c.box[1], 8), y_floor - bhh)
        items.append({"c": c, "lines": lines, "text": text, "box": [bx, by, bx + bw, by + bhh]})

    def x_overlap(a: dict, b: dict) -> bool:
        return a["box"][0] < b["box"][2] + 6 and b["box"][0] < a["box"][2] + 6

    order = sorted(items, key=lambda it: it["box"][1])
    for i, it in enumerate(order):                               # push down past the labels above ...
        for prev in order[:i]:
            if x_overlap(it, prev) and it["box"][1] < prev["box"][3] + 8:
                d = prev["box"][3] + 8 - it["box"][1]
                it["box"][1] += d
                it["box"][3] += d
    for i in range(len(order) - 1, -1, -1):                      # ... then back up from the floor
        it = order[i]
        limit = y_floor
        for nxt in order[i + 1:]:
            if x_overlap(it, nxt):
                limit = min(limit, nxt["box"][1] - 8)
        if it["box"][3] > limit:
            d = it["box"][3] - limit
            it["box"][1] -= d
            it["box"][3] -= d
        if it["box"][1] < 8:
            raise ValueError("call-outs do not fit above each other: use fewer or shorter labels")
    for it in items:
        c, box = it["c"], it["box"]
        ax = min(max(c.target[0], box[0]), box[2])           # leader starts at the box point nearest to the target
        ay = min(max(c.target[1], box[1]), box[3])
        draw.line([ax, ay, c.target[0], c.target[1]], fill=(255, 255, 255), width=5)
        draw.line([ax, ay, c.target[0], c.target[1]], fill=INK, width=2)
        r = 6
        draw.ellipse([c.target[0] - r, c.target[1] - r, c.target[0] + r, c.target[1] + r], fill=ACCENT, outline=(255, 255, 255))
    for it in items:                                             # boxes after all leaders: no line crosses a label
        c, box = it["c"], it["box"]
        draw.rectangle(box, fill=(255, 255, 255), outline=ACCENT if (c.assumed or ASSUMED_WORD in it["text"]) else INK, width=2)
        _draw_lines(draw, (box[0] + 10, box[1] + 5), it["lines"], fnt, INK)
    if banner:
        draw.rectangle([0, img.height - bh, img.width, img.height], fill=(255, 255, 255))
        draw.line([0, img.height - bh, img.width, img.height - bh], fill=ACCENT, width=3)
        _draw_lines(draw, (MARGIN, img.height - bh + 10), banner_lines, fnt, INK)
    if out is not None:
        out = Path(out)
        out.parent.mkdir(parents=True, exist_ok=True)
        img.save(out, format="PNG")
    return img


def main(argv: Sequence[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description="Korean slide helper (demo mode only; import it to build the real slides)")
    ap.add_argument("--demo", required=True, help="PNG to write")
    ap.add_argument("--images", nargs="+", required=True, help="one to four pictures")
    args = ap.parse_args(argv)
    badges = ["delivered", "completed", None, None]
    caps = ["JLC가 부품을 실장해서 보내 준 상태의 기판", "센서 모듈을 직접 끼우고 납땜한 뒤의 기판 - 모듈 치수는 판매 페이지 값",
            "센서가 소켓에 꽂힌 모습", "GPS 플러그와 리드선"]
    panels = [Panel(p, caps[i % 4], assumed=(i % 4 != 0), badge=badges[i % 4]) for i, p in enumerate(args.images[:4])]
    table = Table("시험용 표 (helper demo)", ["핀", "신호", "누가 납땜"],
                  [["1", "VCC", "사용자"], ["2", "GND", "사용자"], ["3", "SCL", "사용자"], ["4", "SDA", "사용자"]],
                  assumed=True, note="이 표는 도우미 시험용입니다. 실제 핀 표는 spec 파일에서 만들어야 합니다.",
                  col_weights=[1, 2, 2])
    out = compose_slide("도우미 시험 슬라이드", panels, args.demo, subtitle="slides_ko.py --demo", table=table,
                        footer="rev 2c 시험용 사본으로 만든 그림")
    print(f"wrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
