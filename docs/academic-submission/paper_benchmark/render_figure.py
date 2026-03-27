from __future__ import annotations

import importlib.util
import math
import sys
from dataclasses import dataclass
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

WIDTH = 1600
HEIGHT = 1000

FONT_PATH = "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf"

BACKGROUND = (246, 248, 251, 255)
TITLE_COLOR = (35, 41, 54, 255)
SUBTITLE_COLOR = (108, 115, 130, 255)

MODEL_COLORS = {
    "aluminum_6061": (96, 109, 128, 255),
    "hdpe": (58, 191, 92, 255),
    "hardwood": (163, 112, 70, 255),
    "silicone_rubber": (194, 75, 75, 255),
}

MODEL_HIDDEN_COLORS = {
    "aluminum_6061": (164, 172, 185, 140),
    "hdpe": (160, 224, 166, 130),
    "hardwood": (207, 181, 154, 130),
    "silicone_rubber": (222, 153, 153, 130),
}

VIEW_UP = (0.0, 0.0, 1.0)
VIEW_OFFSET = (2.8, -2.8, 2.2)
LOOK_AT_Z_BIAS = 0.18
SCENE_BOX = (120, 220, 1480, 848)


@dataclass(frozen=True)
class Segment2D:
    start: tuple[float, float]
    end: tuple[float, float]


@dataclass
class ProjectedPart:
    label: str
    material_id: str
    visible_segments: list[Segment2D]
    hidden_segments: list[Segment2D]
    bounds: tuple[float, float, float, float]
    depth: float


def load_font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    try:
        return ImageFont.truetype(FONT_PATH, size=size)
    except Exception:
        return ImageFont.load_default()


def load_assembly(script_path: Path):
    spec = importlib.util.spec_from_file_location("paper_benchmark_script", script_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load benchmark script: {script_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    if not hasattr(module, "build"):
        raise RuntimeError("paper benchmark script does not define build()")
    return module.build()


def normalize_vector(vector: tuple[float, float, float]) -> tuple[float, float, float]:
    length = math.sqrt(sum(component * component for component in vector))
    if length == 0:
        return (0.0, 0.0, 0.0)
    return tuple(component / length for component in vector)


def project_point_to_screen(
    point: tuple[float, float],
    bounds: tuple[float, float, float, float],
    scene_box: tuple[int, int, int, int] = SCENE_BOX,
    bias_x: float = 0.0,
    bias_y: float = 0.0,
) -> tuple[float, float]:
    min_x, min_y, max_x, max_y = bounds
    left, top, right, bottom = scene_box
    box_w = right - left
    box_h = bottom - top
    content_w = max_x - min_x
    content_h = max_y - min_y
    if content_w <= 0 or content_h <= 0:
        return ((left + right) / 2.0, (top + bottom) / 2.0)
    scale = min(box_w / content_w, box_h / content_h)
    scaled_w = content_w * scale
    scaled_h = content_h * scale
    offset_x = left + (box_w - scaled_w) / 2.0
    offset_y = top + (box_h - scaled_h) / 2.0
    x = offset_x + (point[0] - min_x) * scale + bias_x
    y = offset_y + (max_y - point[1]) * scale + bias_y
    return (x, y)


def draw_dashed_line(
    draw: ImageDraw.ImageDraw,
    start: tuple[float, float],
    end: tuple[float, float],
    *,
    fill: tuple[int, int, int, int],
    width: int,
    dash_length: float = 18.0,
    gap_length: float = 11.0,
) -> None:
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    distance = math.hypot(dx, dy)
    if distance == 0:
        return
    direction_x = dx / distance
    direction_y = dy / distance
    position = 0.0
    while position < distance:
        segment_end = min(distance, position + dash_length)
        p0 = (start[0] + direction_x * position, start[1] + direction_y * position)
        p1 = (
            start[0] + direction_x * segment_end,
            start[1] + direction_y * segment_end,
        )
        draw.line([p0, p1], fill=fill, width=width)
        position += dash_length + gap_length


def draw_segment(
    draw: ImageDraw.ImageDraw,
    segment: Segment2D,
    *,
    fill: tuple[int, int, int, int],
    width: int,
    dashed: bool = False,
) -> None:
    if dashed:
        draw_dashed_line(draw, segment.start, segment.end, fill=fill, width=width)
    else:
        draw.line([segment.start, segment.end], fill=fill, width=width)


def project_assembly(assembly) -> list[ProjectedPart]:
    bbox = assembly.bounding_box()
    look_at = (
        (bbox.min.X + bbox.max.X) / 2.0,
        (bbox.min.Y + bbox.max.Y) / 2.0,
        (bbox.min.Z + bbox.max.Z) / 2.0 + LOOK_AT_Z_BIAS,
    )
    viewport_origin = (
        look_at[0] + VIEW_OFFSET[0],
        look_at[1] + VIEW_OFFSET[1],
        look_at[2] + VIEW_OFFSET[2],
    )
    view_dir = normalize_vector(
        (
            look_at[0] - viewport_origin[0],
            look_at[1] - viewport_origin[1],
            look_at[2] - viewport_origin[2],
        )
    )

    projected_parts: list[ProjectedPart] = []
    for child in getattr(assembly, "children", []):
        label = getattr(child, "label", "part")
        metadata = getattr(child, "metadata", None)
        material_id = getattr(metadata, "material_id", "aluminum_6061")
        visible, hidden = child.project_to_viewport(
            viewport_origin,
            VIEW_UP,
            look_at=look_at,
            focus=7.0,
        )

        vis_segments: list[Segment2D] = []
        hid_segments: list[Segment2D] = []
        all_points: list[tuple[float, float]] = []
        for edge in visible:
            p0 = edge.start_point()
            p1 = edge.end_point()
            start = (p0.X, p0.Y)
            end = (p1.X, p1.Y)
            vis_segments.append(Segment2D(start, end))
            all_points.extend([start, end])
        for edge in hidden:
            p0 = edge.start_point()
            p1 = edge.end_point()
            start = (p0.X, p0.Y)
            end = (p1.X, p1.Y)
            hid_segments.append(Segment2D(start, end))
            all_points.extend([start, end])

        if not all_points:
            continue
        xs = [point[0] for point in all_points]
        ys = [point[1] for point in all_points]
        center = (
            (min(xs) + max(xs)) / 2.0,
            (min(ys) + max(ys)) / 2.0,
            0.0,
        )
        depth = sum((center[i] - viewport_origin[i]) * view_dir[i] for i in range(3))
        projected_parts.append(
            ProjectedPart(
                label=label,
                material_id=material_id,
                visible_segments=vis_segments,
                hidden_segments=hid_segments,
                bounds=(min(xs), min(ys), max(xs), max(ys)),
                depth=depth,
            )
        )

    projected_parts.sort(key=lambda item: item.depth, reverse=True)
    return projected_parts


def render_projection(image_path: Path) -> None:
    script_path = Path(__file__).resolve().with_name("script.py")
    assembly = load_assembly(script_path)
    projected_parts = project_assembly(assembly)

    points: list[tuple[float, float]] = []
    for part in projected_parts:
        for segment in part.visible_segments:
            points.extend([segment.start, segment.end])
        for segment in part.hidden_segments:
            points.extend([segment.start, segment.end])

    if not points:
        raise RuntimeError("projection produced no drawable geometry")

    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    bounds = (min(xs), min(ys), max(xs), max(ys))

    canvas = Image.new("RGBA", (WIDTH, HEIGHT), BACKGROUND)
    draw = ImageDraw.Draw(canvas, "RGBA")
    title_font = load_font(30)
    subtitle_font = load_font(21)
    label_font = load_font(18)
    small_font = load_font(16)
    part_font = load_font(19)

    def transform(point: tuple[float, float]) -> tuple[float, float]:
        return project_point_to_screen(point, bounds)

    # Base technical linework first.
    for part in projected_parts:
        vis_color = MODEL_COLORS.get(part.material_id, MODEL_COLORS["aluminum_6061"])
        hid_color = MODEL_HIDDEN_COLORS.get(
            part.material_id, MODEL_HIDDEN_COLORS["aluminum_6061"]
        )
        for segment in part.visible_segments:
            draw_segment(
                draw,
                Segment2D(transform(segment.start), transform(segment.end)),
                fill=vis_color,
                width=6,
            )
        for segment in part.hidden_segments:
            draw_segment(
                draw,
                Segment2D(transform(segment.start), transform(segment.end)),
                fill=hid_color,
                width=4,
                dashed=True,
            )

    def part_center_screen(part: ProjectedPart) -> tuple[float, float]:
        min_x, min_y, max_x, max_y = part.bounds
        return transform(((min_x + max_x) / 2.0, (min_y + max_y) / 2.0))

    def label(
        text: str,
        xy: tuple[float, float],
        fill: tuple[int, int, int, int],
        *,
        anchor: str = "mm",
    ):
        draw.text(
            xy,
            text,
            fill=fill,
            font=part_font,
            anchor=anchor,
            stroke_width=4,
            stroke_fill=(246, 248, 251, 255),
        )

    part_lookup = {part.label: part for part in projected_parts}
    if "support_tower" in part_lookup:
        cx, cy = part_center_screen(part_lookup["support_tower"])
        label("support tower", (cx - 6, cy - 92), (92, 100, 115, 255))
    if "payload" in part_lookup:
        cx, cy = part_center_screen(part_lookup["payload"])
        label("payload", (cx - 10, cy - 58), (163, 112, 70, 255))
    if "forbid_blocker" in part_lookup:
        cx, cy = part_center_screen(part_lookup["forbid_blocker"])
        label("blocker", (cx + 10, cy - 62), (194, 75, 75, 255))
        label("forbid zone", (cx + 14, cy - 86), (194, 75, 75, 255))
    if "goal_tray" in part_lookup:
        cx, cy = part_center_screen(part_lookup["goal_tray"])
        label("goal tray", (cx + 10, cy - 66), (58, 191, 92, 255))
        label("goal zone", (cx + 220, cy - 72), (58, 191, 92, 255), anchor="lm")

    # Paper-style overlay.
    draw.text(
        (110, 54),
        "Compact engineering-coder benchmark",
        fill=TITLE_COLOR,
        font=title_font,
    )
    draw.text(
        (110, 95),
        "OCP projection with Pillow annotations",
        fill=SUBTITLE_COLOR,
        font=subtitle_font,
    )

    card = (586, 106, 1018, 194)
    draw.rounded_rectangle(
        card, radius=18, fill=(255, 255, 255, 210), outline=(218, 223, 232), width=2
    )
    draw.text((610, 124), "pipeline color schema", fill=TITLE_COLOR, font=label_font)
    schema = [
        ((612, 156, 700, 176), (163, 112, 70, 242), "start"),
        ((730, 156, 838, 176), (56, 112, 196, 242), "transfer"),
        ((860, 156, 944, 176), (58, 191, 92, 242), "goal"),
    ]
    for box, fill, text in schema:
        draw.rounded_rectangle(box, radius=8, fill=fill)
        tx = (box[0] + box[2]) / 2
        ty = (box[1] + box[3]) / 2
        draw.text(
            (tx, ty - 1), text, fill=(255, 255, 255), font=small_font, anchor="mm"
        )

    # Curved transfer arrow.
    points_curve = []
    start = (330, 322)
    control = (720, 470)
    end = (1030, 625)
    for t in [i / 30.0 for i in range(31)]:
        x = (1 - t) ** 2 * start[0] + 2 * (1 - t) * t * control[0] + t**2 * end[0]
        y = (1 - t) ** 2 * start[1] + 2 * (1 - t) * t * control[1] + t**2 * end[1]
        points_curve.append((x, y))
    draw.line(points_curve, fill=(58, 112, 196, 255), width=8)
    tip = points_curve[-1]
    prev = points_curve[-3]
    angle = math.atan2(tip[1] - prev[1], tip[0] - prev[0])
    size = 16
    left = (
        tip[0] - size * math.cos(angle) + size * 0.55 * math.sin(angle),
        tip[1] - size * math.sin(angle) - size * 0.55 * math.cos(angle),
    )
    right = (
        tip[0] - size * math.cos(angle) - size * 0.55 * math.sin(angle),
        tip[1] - size * math.sin(angle) + size * 0.55 * math.cos(angle),
    )
    draw.polygon([tip, left, right], fill=(58, 112, 196, 255))

    pipeline_chips = [
        ((380, 436), "pick", (163, 112, 70, 220), (255, 255, 255)),
        ((720, 555), "transfer", (56, 112, 196, 220), (255, 255, 255)),
        ((1056, 706), "place", (58, 191, 92, 220), (255, 255, 255)),
    ]
    for (cx, cy), text, fill, text_fill in pipeline_chips:
        w = 108 if text != "transfer" else 138
        h = 34
        rect = (cx - w / 2, cy - h / 2, cx + w / 2, cy + h / 2)
        draw.rounded_rectangle(
            rect, radius=12, fill=fill, outline=(255, 255, 255, 120), width=2
        )
        draw.text((cx, cy - 1), text, fill=text_fill, font=small_font, anchor="mm")

    legend_y = 890
    legend_items = [
        ("Start payload", (163, 112, 70)),
        ("Goal tray", (58, 191, 92)),
        ("Keep-out blocker", (194, 75, 75)),
        ("Base fixture", (160, 170, 186)),
    ]
    x = 110
    for text, color in legend_items:
        draw.rounded_rectangle(
            [x, legend_y, x + 24, legend_y + 24], radius=6, fill=color
        )
        draw.text(
            (x + 34, legend_y + 12),
            text,
            fill=TITLE_COLOR,
            font=subtitle_font,
            anchor="lm",
        )
        x += 320

    canvas.convert("RGB").save(image_path)


def main() -> None:
    if len(sys.argv) != 2:
        raise SystemExit("usage: render_figure.py OUTPUT_PATH")

    output_path = Path(sys.argv[1])
    output_path.parent.mkdir(parents=True, exist_ok=True)
    render_projection(output_path)
    print(output_path)


if __name__ == "__main__":
    main()
