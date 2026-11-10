"""Dependency-free overlay rendering for dataset previews."""

from __future__ import annotations

from pathlib import Path
from typing import Any

from .png_codec import read_png, write_png


# Five-by-seven uppercase bitmap font.  Preview text is diagnostic, not part of
# the training image, so a compact built-in font avoids a Pillow dependency.
_FONT = {
    " ": ("00000",) * 7,
    "A": ("01110", "10001", "10001", "11111", "10001", "10001", "10001"),
    "B": ("11110", "10001", "10001", "11110", "10001", "10001", "11110"),
    "C": ("01111", "10000", "10000", "10000", "10000", "10000", "01111"),
    "D": ("11110", "10001", "10001", "10001", "10001", "10001", "11110"),
    "E": ("11111", "10000", "10000", "11110", "10000", "10000", "11111"),
    "F": ("11111", "10000", "10000", "11110", "10000", "10000", "10000"),
    "G": ("01111", "10000", "10000", "10111", "10001", "10001", "01111"),
    "H": ("10001", "10001", "10001", "11111", "10001", "10001", "10001"),
    "I": ("11111", "00100", "00100", "00100", "00100", "00100", "11111"),
    "J": ("00111", "00010", "00010", "00010", "10010", "10010", "01100"),
    "K": ("10001", "10010", "10100", "11000", "10100", "10010", "10001"),
    "L": ("10000", "10000", "10000", "10000", "10000", "10000", "11111"),
    "M": ("10001", "11011", "10101", "10101", "10001", "10001", "10001"),
    "N": ("10001", "11001", "10101", "10011", "10001", "10001", "10001"),
    "O": ("01110", "10001", "10001", "10001", "10001", "10001", "01110"),
    "P": ("11110", "10001", "10001", "11110", "10000", "10000", "10000"),
    "Q": ("01110", "10001", "10001", "10001", "10101", "10010", "01101"),
    "R": ("11110", "10001", "10001", "11110", "10100", "10010", "10001"),
    "S": ("01111", "10000", "10000", "01110", "00001", "00001", "11110"),
    "T": ("11111", "00100", "00100", "00100", "00100", "00100", "00100"),
    "U": ("10001", "10001", "10001", "10001", "10001", "10001", "01110"),
    "V": ("10001", "10001", "10001", "10001", "10001", "01010", "00100"),
    "W": ("10001", "10001", "10001", "10101", "10101", "10101", "01010"),
    "X": ("10001", "10001", "01010", "00100", "01010", "10001", "10001"),
    "Y": ("10001", "10001", "01010", "00100", "00100", "00100", "00100"),
    "Z": ("11111", "00001", "00010", "00100", "01000", "10000", "11111"),
    "0": ("01110", "10001", "10011", "10101", "11001", "10001", "01110"),
    "1": ("00100", "01100", "00100", "00100", "00100", "00100", "01110"),
    "2": ("01110", "10001", "00001", "00010", "00100", "01000", "11111"),
    "3": ("11110", "00001", "00001", "01110", "00001", "00001", "11110"),
    "4": ("00010", "00110", "01010", "10010", "11111", "00010", "00010"),
    "5": ("11111", "10000", "10000", "11110", "00001", "00001", "11110"),
    "6": ("01110", "10000", "10000", "11110", "10001", "10001", "01110"),
    "7": ("11111", "00001", "00010", "00100", "01000", "01000", "01000"),
    "8": ("01110", "10001", "10001", "01110", "10001", "10001", "01110"),
    "9": ("01110", "10001", "10001", "01111", "00001", "00001", "01110"),
    "_": ("00000", "00000", "00000", "00000", "00000", "00000", "11111"),
    ":": ("00000", "00100", "00100", "00000", "00100", "00100", "00000"),
    ".": ("00000", "00000", "00000", "00000", "00000", "00110", "00110"),
    "-": ("00000", "00000", "00000", "11111", "00000", "00000", "00000"),
    "/": ("00001", "00010", "00010", "00100", "01000", "01000", "10000"),
    "=": ("00000", "11111", "00000", "11111", "00000", "00000", "00000"),
    "?": ("01110", "10001", "00001", "00010", "00100", "00000", "00100"),
}


class Canvas:
    def __init__(self, width: int, height: int, rgb: bytes):
        if len(rgb) != width * height * 3:
            raise ValueError("RGB canvas byte count mismatch")
        self.width = width
        self.height = height
        self.data = bytearray(rgb)

    def pixel(self, x: int, y: int, color: tuple[int, int, int]) -> None:
        if 0 <= x < self.width and 0 <= y < self.height:
            offset = (y * self.width + x) * 3
            self.data[offset : offset + 3] = bytes(color)

    def line(
        self,
        start: tuple[float, float],
        end: tuple[float, float],
        color: tuple[int, int, int],
        thickness: int = 2,
    ) -> None:
        x0, y0 = round(start[0]), round(start[1])
        x1, y1 = round(end[0]), round(end[1])
        dx, sx = abs(x1 - x0), 1 if x0 < x1 else -1
        dy, sy = -abs(y1 - y0), 1 if y0 < y1 else -1
        error = dx + dy
        radius = max(0, thickness // 2)
        while True:
            for yy in range(y0 - radius, y0 + radius + 1):
                for xx in range(x0 - radius, x0 + radius + 1):
                    self.pixel(xx, yy, color)
            if x0 == x1 and y0 == y1:
                break
            doubled = 2 * error
            if doubled >= dy:
                error += dy
                x0 += sx
            if doubled <= dx:
                error += dx
                y0 += sy

    def rectangle(
        self,
        box: list[float] | tuple[float, float, float, float],
        color: tuple[int, int, int],
        thickness: int = 2,
    ) -> None:
        x0, y0, x1, y1 = box
        self.line((x0, y0), (x1, y0), color, thickness)
        self.line((x1, y0), (x1, y1), color, thickness)
        self.line((x1, y1), (x0, y1), color, thickness)
        self.line((x0, y1), (x0, y0), color, thickness)

    def fill_rectangle(
        self, box: tuple[int, int, int, int], color: tuple[int, int, int]
    ) -> None:
        x0, y0, x1, y1 = box
        for y in range(max(0, y0), min(self.height, y1)):
            start = (y * self.width + max(0, x0)) * 3
            end_x = min(self.width, x1)
            if end_x > max(0, x0):
                self.data[start : (y * self.width + end_x) * 3] = bytes(color) * (
                    end_x - max(0, x0)
                )

    def circle(
        self, center: tuple[float, float], radius: int, color: tuple[int, int, int]
    ) -> None:
        cx, cy = round(center[0]), round(center[1])
        for y in range(cy - radius, cy + radius + 1):
            for x in range(cx - radius, cx + radius + 1):
                if (x - cx) ** 2 + (y - cy) ** 2 <= radius**2:
                    self.pixel(x, y, color)

    def text(
        self,
        x: int,
        y: int,
        value: str,
        color: tuple[int, int, int] = (255, 255, 255),
        scale: int = 2,
    ) -> None:
        cursor = x
        for character in value.upper():
            glyph = _FONT.get(character, _FONT["?"])
            for row, bits in enumerate(glyph):
                for column, bit in enumerate(bits):
                    if bit == "1":
                        for yy in range(scale):
                            for xx in range(scale):
                                self.pixel(cursor + column * scale + xx, y + row * scale + yy, color)
            cursor += 6 * scale


POINT_COLORS = (
    (255, 60, 60),    # TL
    (60, 255, 60),    # TR
    (70, 120, 255),   # BR
    (255, 230, 40),   # BL
)


def render_overlay(width: int, height: int, rgb: bytes, record: dict[str, Any]) -> bytes:
    canvas = Canvas(width, height, rgb)
    heading = (
        f"FRAME={record.get('frame_index')} TICK={record.get('tick')} "
        f"SIM_TIME={float(record.get('sim_time', 0.0)):.6f}"
    )
    heading_width = min(width, 12 * len(heading) + 16)
    canvas.fill_rectangle((0, 0, heading_width, 22), (0, 0, 0))
    canvas.text(6, 4, heading, scale=2)

    for armor in record.get("armors", []):
        corners = armor["corners_tl_tr_br_bl"]
        for index in range(4):
            canvas.line(tuple(corners[index]), tuple(corners[(index + 1) % 4]), (255, 80, 255), 2)
        canvas.rectangle(armor["bbox_xyxy"], (40, 240, 255), 2)
        for point, color in zip(corners, POINT_COLORS):
            canvas.circle(tuple(point), 5, color)
        label = (
            f"{armor.get('team')} {armor.get('robot')} {armor.get('armor_type')} "
            f"ID={armor.get('armor_id')}"
        )
        label_width = min(width, 12 * len(label) + 8)
        x = max(0, min(width - label_width, round(armor["bbox_xyxy"][0])))
        y = max(24, min(height - 18, round(armor["bbox_xyxy"][1]) - 18))
        canvas.fill_rectangle((x, y, x + label_width, y + 17), (0, 0, 0))
        canvas.text(x + 4, y + 2, label, scale=2)
    return bytes(canvas.data)


def write_preview(image_path: Path, record: dict[str, Any], output_path: Path) -> None:
    width, height, rgb = read_png(image_path)
    if width != record.get("width") or height != record.get("height"):
        raise ValueError("preview image dimensions do not match annotation")
    overlay = render_overlay(width, height, rgb, record)
    write_png(output_path, width, height, overlay)
