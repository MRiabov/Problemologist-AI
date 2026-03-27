from __future__ import annotations

import math
import sys
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont
from scene_spec import SCENE

WIDTH = 1600
HEIGHT = 1000

BG_TOP = (250, 251, 253)
BG_BOTTOM = (236, 239, 244)
TEXT = (35, 41, 54)
MUTED = (109, 115, 130)

COLOR_MAP = {
    "aluminum_6061": ((200, 208, 218), (162, 171, 186), (129, 137, 151)),
    "hdpe": ((231, 240, 241), (171, 194, 198), (126, 156, 162)),
    "hardwood": ((168, 120, 71), (123, 82, 47), (97, 64, 37)),
    "silicone_rubber": ((203, 76, 76), (166, 49, 49), (132, 31, 31)),
}

AX = (1.0, 0.46)
AY = (-1.0, 0.46)
AZ = (0.0, -1.0)
SCALE = 420.0
ORIGIN = (804.0, 760.0)

FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"


def load_font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    try:
        return ImageFont.truetype(FONT_PATH, size=size)
    except Exception:
        return ImageFont.load_default()


def lerp(a: int, b: int, t: float) -> int:
    return int(round(a + (b - a) * t))


def clamp(v: float, lo: float, hi: float) -> float:
    return max(lo, min(hi, v))


def project(x: float, y: float, z: float) -> tuple[float, float]:
    return (
        ORIGIN[0] + SCALE * (x * AX[0] + y * AY[0] + z * AZ[0]),
        ORIGIN[1] + SCALE * (x * AX[1] + y * AY[1] + z * AZ[1]),
    )


def lighten(rgb: tuple[int, int, int], amount: float) -> tuple[int, int, int]:
    return tuple(lerp(c, 255, amount) for c in rgb)


def darken(rgb: tuple[int, int, int], amount: float) -> tuple[int, int, int]:
    return tuple(lerp(c, 18, amount) for c in rgb)


def face_colors(
    material_id: str,
) -> tuple[tuple[int, int, int], tuple[int, int, int], tuple[int, int, int]]:
    top, side_a, side_b = COLOR_MAP[material_id]
    return top, side_a, side_b


def draw_poly(draw: ImageDraw.ImageDraw, points, fill, outline=None, width: int = 1):
    draw.polygon(points, fill=fill, outline=outline)
    if outline and width > 1:
        draw.line(points + [points[0]], fill=outline, width=width, joint="curve")


def draw_box(
    draw: ImageDraw.ImageDraw,
    center: tuple[float, float, float],
    size: tuple[float, float, float],
    material_id: str,
    *,
    outline: tuple[int, int, int] | None = None,
    label: str | None = None,
    label_pos: tuple[float, float] | None = None,
    label_fill: tuple[int, int, int] = TEXT,
):
    cx, cy, cz = center
    dx, dy, dz = size
    x0 = cx - dx / 2.0
    x1 = cx + dx / 2.0
    y0 = cy - dy / 2.0
    y1 = cy + dy / 2.0
    z0 = cz
    z1 = cz + dz

    p000 = project(x0, y0, z0)
    p100 = project(x1, y0, z0)
    p110 = project(x1, y1, z0)
    p010 = project(x0, y1, z0)
    p001 = project(x0, y0, z1)
    p101 = project(x1, y0, z1)
    p111 = project(x1, y1, z1)
    p011 = project(x0, y1, z1)

    top, side_a, side_b = face_colors(material_id)

    # soft shadow
    shadow_offset = (18, 20)
    shadow = [
        (x + shadow_offset[0], y + shadow_offset[1])
        for x, y in [p001, p101, p111, p011]
    ]
    draw.polygon(shadow, fill=(0, 0, 0, 32))

    draw_poly(draw, [p000, p010, p011, p001], fill=side_a, outline=outline, width=3)
    draw_poly(draw, [p100, p110, p111, p101], fill=side_b, outline=outline, width=3)
    draw_poly(draw, [p001, p101, p111, p011], fill=top, outline=outline, width=3)

    if label and label_pos:
        font = load_font(28)
        draw.text(label_pos, label, fill=label_fill, font=font, anchor="mm")


def draw_background() -> Image.Image:
    img = Image.new("RGBA", (WIDTH, HEIGHT), BG_TOP)
    px = img.load()
    for y in range(HEIGHT):
        t = y / max(1, HEIGHT - 1)
        row = tuple(lerp(BG_TOP[i], BG_BOTTOM[i], t) for i in range(3))
        for x in range(WIDTH):
            px[x, y] = row + (255,)
    return img


def draw_base_plate(draw: ImageDraw.ImageDraw):
    plate = [
        project(-0.76, -0.30, 0.0),
        project(0.76, -0.30, 0.0),
        project(0.62, 0.30, 0.0),
        project(-0.62, 0.30, 0.0),
    ]
    shadow = [(x + 22, y + 22) for x, y in plate]
    draw.polygon(shadow, fill=(19, 29, 44, 44))
    draw.polygon(plate, fill=(69, 98, 129, 255), outline=(52, 78, 107), width=4)

    overlay = Image.new("RGBA", (WIDTH, HEIGHT), (0, 0, 0, 0))
    overlay_draw = ImageDraw.Draw(overlay)
    mask = Image.new("L", (WIDTH, HEIGHT), 0)
    mask_draw = ImageDraw.Draw(mask)
    mask_draw.polygon(plate, fill=255)
    for i in range(-2, 22):
        x0 = 248 + i * 42
        overlay_draw.line(
            [(x0, 510), (x0 + 210, 830)],
            fill=(255, 255, 255, 16),
            width=2,
        )
    for i in range(-1, 11):
        y = 560 + i * 26
        overlay_draw.line(
            [(140, y), (1410, y)],
            fill=(255, 255, 255, 12),
            width=1,
        )
    img = Image.composite(
        overlay, Image.new("RGBA", (WIDTH, HEIGHT), (0, 0, 0, 0)), mask
    )
    return img


def draw_arrow(draw: ImageDraw.ImageDraw, start, end, bend=0.0):
    sx, sy = start
    ex, ey = end
    mid = ((sx + ex) / 2.0, (sy + ey) / 2.0 - bend)
    points = []
    for t in [i / 24.0 for i in range(25)]:
        x = (1 - t) ** 2 * sx + 2 * (1 - t) * t * mid[0] + t**2 * ex
        y = (1 - t) ** 2 * sy + 2 * (1 - t) * t * mid[1] + t**2 * ey
        points.append((x, y))
    draw.line(points, fill=(43, 108, 183), width=8, joint="curve")

    angle = math.atan2(ey - points[-2][1], ex - points[-2][0])
    size = 16
    tip = (ex, ey)
    left = (
        ex - size * math.cos(angle) + size * 0.55 * math.sin(angle),
        ey - size * math.sin(angle) - size * 0.55 * math.cos(angle),
    )
    right = (
        ex - size * math.cos(angle) - size * 0.55 * math.sin(angle),
        ey - size * math.sin(angle) + size * 0.55 * math.cos(angle),
    )
    draw.polygon([tip, left, right], fill=(43, 108, 183))


def draw_legend(draw: ImageDraw.ImageDraw):
    font = load_font(24)
    bold = load_font(28)
    title = "Compact engineering-coder benchmark"
    draw.text((110, 64), title, fill=TEXT, font=bold)
    draw.text(
        (110, 105),
        "Benchmark schematic derived from a real benchmark scene",
        fill=MUTED,
        font=font,
    )

    legend_items = [
        ("Start payload", (168, 120, 71)),
        ("Goal tray", (171, 194, 198)),
        ("Keep-out blocker", (203, 76, 76)),
        ("Base fixture", (162, 171, 186)),
    ]
    x = 110
    y = 880
    for label, color in legend_items:
        draw.rounded_rectangle(
            [x, y, x + 24, y + 24], radius=6, fill=color, outline=(255, 255, 255)
        )
        draw.text((x + 36, y + 12), label, fill=TEXT, font=font, anchor="lm")
        x += 300


def draw_info_card(draw: ImageDraw.ImageDraw):
    x0, y0, x1, y1 = 1030, 130, 1490, 286
    draw.rounded_rectangle(
        [x0, y0, x1, y1],
        radius=22,
        fill=(255, 255, 255, 214),
        outline=(214, 221, 232),
        width=2,
    )
    title_font = load_font(24)
    body_font = load_font(19)
    draw.text((1058, 156), "compact benchmark envelope", fill=TEXT, font=title_font)
    draw.text(
        (1058, 190), "bounds: (-1, -1, 0) to (1, 1, 2) m", fill=MUTED, font=body_font
    )
    draw.text((1058, 222), "payload starts at x = -0.70 m", fill=MUTED, font=body_font)
    draw.text((1058, 254), "goal tray sits at x = 0.62 m", fill=MUTED, font=body_font)


def draw_goal_zone(draw: ImageDraw.ImageDraw):
    # Goal zone outline around the tray.
    rect = [
        project(0.48, -0.08, 0.04),
        project(0.78, -0.08, 0.04),
        project(0.78, 0.08, 0.04),
        project(0.48, 0.08, 0.04),
    ]
    draw.line(rect + [rect[0]], fill=(79, 165, 122), width=5)
    draw.text((1250, 470), "goal zone", fill=(79, 165, 122), font=load_font(24))


def draw_forbid_zone(draw: ImageDraw.ImageDraw):
    rect = [
        project(-0.04, -0.06, 0.04),
        project(0.14, -0.06, 0.04),
        project(0.14, 0.06, 0.04),
        project(-0.04, 0.06, 0.04),
    ]
    draw.line(rect + [rect[0]], fill=(195, 78, 78), width=5)
    draw.text((738, 356), "forbid zone", fill=(195, 78, 78), font=load_font(24))


def build_image() -> Image.Image:
    canvas = draw_background()
    plate = draw_base_plate(ImageDraw.Draw(canvas, "RGBA"))
    canvas.alpha_composite(plate)
    draw = ImageDraw.Draw(canvas, "RGBA")

    # Fixtures.
    draw_box(
        draw,
        SCENE["support_tower"]["center"],
        SCENE["support_tower"]["size"],
        SCENE["support_tower"]["material_id"],
        label="support tower",
        label_pos=(610, 290),
        label_fill=(66, 75, 88),
    )
    draw_box(
        draw,
        SCENE["forbid_blocker"]["center"],
        SCENE["forbid_blocker"]["size"],
        SCENE["forbid_blocker"]["material_id"],
        outline=(155, 28, 28),
        label="blocker",
        label_pos=(930, 404),
        label_fill=(155, 28, 28),
    )
    draw_box(
        draw,
        SCENE["goal_tray"]["center"],
        SCENE["goal_tray"]["size"],
        SCENE["goal_tray"]["material_id"],
        outline=(96, 131, 136),
        label="goal tray",
        label_pos=(1096, 598),
        label_fill=(59, 103, 110),
    )
    draw_box(
        draw,
        SCENE["payload"]["center"],
        SCENE["payload"]["size"],
        SCENE["payload"]["material_id"],
        outline=(115, 76, 43),
        label="payload",
        label_pos=(450, 420),
        label_fill=(115, 76, 43),
    )

    draw_goal_zone(draw)
    draw_forbid_zone(draw)
    draw_arrow(draw, project(-0.67, 0.0, 0.14), project(0.60, 0.0, 0.12), bend=120.0)

    draw_info_card(draw)
    draw_legend(draw)
    return canvas.convert("RGB")


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: render_figure.py OUTPUT_PATH")
    output_path = Path(sys.argv[1])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    image = build_image()
    image.save(output_path, "PNG")
    print(output_path)


if __name__ == "__main__":
    main()
