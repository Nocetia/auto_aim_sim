"""Small, dependency-free helpers for dataset image I/O.

The decoder intentionally implements only the PNG subset used by the dataset
exporter: 8-bit, non-interlaced true-colour images, with or without alpha.  It
does not apply colour-management, gamma, or channel-order conversions.
"""

from __future__ import annotations

import os
from pathlib import Path
import struct
import sys
import tempfile
from typing import TypeVar
import zlib


PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
_MAX_PNG_DIMENSION = (1 << 31) - 1
_IDAT_CHUNK_SIZE = 1 << 20


class ImageCodecError(ValueError):
    """Base class for invalid image data passed to this module."""


class RawImageError(ImageCodecError):
    """Raised when a raw RGB/RGBA image is malformed."""


class PngCodecError(ImageCodecError):
    """Raised when a PNG is malformed or uses an unsupported PNG feature."""


# Keep the conventional all-caps acronym spelling available to callers too.
PNGCodecError = PngCodecError
# Short spelling used by the dataset validator.
PngError = PngCodecError


__all__ = [
    "ImageCodecError",
    "RawImageError",
    "PngCodecError",
    "PNGCodecError",
    "PngError",
    "raw_to_rgb",
    "write_png",
    "read_png",
]


_ErrorT = TypeVar("_ErrorT", bound=ImageCodecError)


def _validate_dimensions(
    width: int,
    height: int,
    error_type: type[_ErrorT],
    context: str,
) -> int:
    # bool is an int subclass, but accepting True as a width is almost always a
    # caller bug and makes validation less predictable.
    if type(width) is not int or type(height) is not int:
        raise error_type(f"{context} width and height must be integers")
    if width <= 0 or height <= 0:
        raise error_type(
            f"{context} width and height must be positive (got {width}x{height})"
        )
    if width > _MAX_PNG_DIMENSION or height > _MAX_PNG_DIMENSION:
        raise error_type(
            f"{context} dimensions exceed PNG's 2^31-1 limit "
            f"(got {width}x{height})"
        )
    return width * height


def _buffer_bytes(
    value: object,
    name: str,
    error_type: type[_ErrorT],
    expected_length: int,
) -> bytes:
    try:
        view = memoryview(value)
    except (TypeError, ValueError) as exc:
        raise error_type(f"{name} must be a bytes-like object") from exc
    try:
        if view.nbytes != expected_length:
            raise error_type(
                f"{name} length mismatch: expected {expected_length} bytes, "
                f"got {view.nbytes}"
            )
        return view.tobytes()
    finally:
        view.release()


def raw_to_rgb(raw: object, width: int, height: int, encoding: str) -> bytes:
    """Validate an RGB8/RGBA8 frame and return tightly packed RGB bytes.

    Encoding matching is case-insensitive.  RGBA alpha samples are discarded;
    the RGB samples themselves are returned unchanged and in the same order.
    """

    pixel_count = _validate_dimensions(width, height, RawImageError, "raw image")
    if not isinstance(encoding, str):
        raise RawImageError("raw image encoding must be a string")

    normalized = encoding.casefold()
    if normalized == "rgb8":
        channels = 3
    elif normalized == "rgba8":
        channels = 4
    else:
        raise RawImageError(
            f"unsupported raw image encoding {encoding!r}; expected RGB8 or RGBA8"
        )

    expected = pixel_count * channels
    data = _buffer_bytes(
        raw,
        f"{encoding.upper()} data for {width}x{height}",
        RawImageError,
        expected,
    )

    if channels == 3:
        return data

    rgb = bytearray(pixel_count * 3)
    rgb[0::3] = data[0::4]
    rgb[1::3] = data[1::4]
    rgb[2::3] = data[2::4]
    return bytes(rgb)


def _png_chunk(chunk_type: bytes, payload: bytes) -> bytes:
    if len(chunk_type) != 4:
        raise AssertionError("PNG chunk type must contain four bytes")
    if len(payload) > _MAX_PNG_DIMENSION:
        raise PngCodecError(
            f"PNG {chunk_type.decode('ascii', 'replace')} chunk is too large"
        )
    crc = zlib.crc32(chunk_type)
    crc = zlib.crc32(payload, crc) & 0xFFFFFFFF
    return (
        struct.pack(">I", len(payload))
        + chunk_type
        + payload
        + struct.pack(">I", crc)
    )


def _encoded_png(width: int, height: int, rgb: bytes) -> bytes:
    row_bytes = width * 3
    scanline_bytes = row_bytes + 1
    if scanline_bytes > sys.maxsize or height > sys.maxsize // scanline_bytes:
        raise RawImageError(f"RGB image {width}x{height} is too large to encode")

    filtered = bytearray(scanline_bytes * height)
    source_offset = 0
    output_offset = 0
    for _ in range(height):
        # Filter type 0 preserves the stored channel bytes verbatim.
        filtered[output_offset] = 0
        output_offset += 1
        filtered[output_offset : output_offset + row_bytes] = rgb[
            source_offset : source_offset + row_bytes
        ]
        source_offset += row_bytes
        output_offset += row_bytes

    compressed = zlib.compress(filtered)
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
    pieces = [PNG_SIGNATURE, _png_chunk(b"IHDR", ihdr)]
    for offset in range(0, len(compressed), _IDAT_CHUNK_SIZE):
        pieces.append(
            _png_chunk(b"IDAT", compressed[offset : offset + _IDAT_CHUNK_SIZE])
        )
    pieces.append(_png_chunk(b"IEND", b""))
    return b"".join(pieces)


def _path(value: object) -> Path:
    try:
        path = Path(value)  # type: ignore[arg-type]
    except (TypeError, ValueError) as exc:
        raise TypeError("path must be a string or path-like object") from exc
    if not path.name:
        raise ValueError("path must name a file")
    return path


def write_png(path: object, width: int, height: int, rgb: object) -> None:
    """Atomically write a standard 8-bit RGB PNG.

    The temporary file is written and fsynced in the destination directory,
    then installed with :func:`os.replace`, so a failed write cannot leave a
    partially written destination file.
    """

    pixel_count = _validate_dimensions(width, height, RawImageError, "RGB image")
    expected = pixel_count * 3
    data = _buffer_bytes(
        rgb, f"RGB data for {width}x{height}", RawImageError, expected
    )

    target = _path(path)
    encoded = _encoded_png(width, height, data)

    descriptor = -1
    temporary_name: str | None = None
    try:
        descriptor, temporary_name = tempfile.mkstemp(
            prefix=".png-codec-", suffix=".tmp", dir=os.fspath(target.parent)
        )
        with os.fdopen(descriptor, "wb") as output:
            descriptor = -1
            output.write(encoded)
            output.flush()
            os.fsync(output.fileno())
        os.replace(temporary_name, target)
        temporary_name = None
    finally:
        if descriptor >= 0:
            os.close(descriptor)
        if temporary_name is not None:
            try:
                os.unlink(temporary_name)
            except FileNotFoundError:
                pass


def _chunk_label(chunk_type: bytes) -> str:
    return chunk_type.decode("ascii", "replace")


def _valid_chunk_type(chunk_type: bytes) -> bool:
    return len(chunk_type) == 4 and all(
        ord("A") <= value <= ord("Z") or ord("a") <= value <= ord("z")
        for value in chunk_type
    )


def _parse_png(blob: bytes) -> tuple[int, int, int, bytes]:
    if not blob.startswith(PNG_SIGNATURE):
        raise PngCodecError("invalid PNG signature")

    offset = len(PNG_SIGNATURE)
    chunk_index = 0
    seen_ihdr = False
    seen_plte = False
    seen_idat = False
    idat_ended = False
    seen_iend = False
    width = 0
    height = 0
    colour_type = -1
    idat_parts: list[bytes] = []

    while offset < len(blob):
        chunk_index += 1
        if len(blob) - offset < 12:
            raise PngCodecError(
                f"truncated PNG chunk header at byte offset {offset}"
            )

        length = struct.unpack_from(">I", blob, offset)[0]
        if length > _MAX_PNG_DIMENSION:
            raise PngCodecError(
                f"PNG chunk {chunk_index} length exceeds the 2^31-1 limit"
            )
        chunk_type = blob[offset + 4 : offset + 8]
        if not _valid_chunk_type(chunk_type):
            raise PngCodecError(
                f"PNG chunk {chunk_index} has an invalid type code"
            )
        # The third type-code letter is reserved and must be uppercase in PNG.
        if chunk_type[2] & 0x20:
            raise PngCodecError(
                f"PNG chunk {_chunk_label(chunk_type)} has an invalid reserved bit"
            )

        payload_start = offset + 8
        payload_end = payload_start + length
        chunk_end = payload_end + 4
        if chunk_end > len(blob):
            available = len(blob) - payload_start
            if available < length:
                raise PngCodecError(
                    f"truncated {_chunk_label(chunk_type)} data: declared {length} "
                    f"bytes, only {max(0, available)} available"
                )
            raise PngCodecError(
                f"truncated {_chunk_label(chunk_type)} CRC: expected 4 bytes, "
                f"got {len(blob) - payload_end}"
            )

        payload = blob[payload_start:payload_end]
        stored_crc = struct.unpack_from(">I", blob, payload_end)[0]
        computed_crc = zlib.crc32(chunk_type)
        computed_crc = zlib.crc32(payload, computed_crc) & 0xFFFFFFFF
        if stored_crc != computed_crc:
            raise PngCodecError(
                f"CRC mismatch in {_chunk_label(chunk_type)} chunk {chunk_index}: "
                f"stored 0x{stored_crc:08x}, computed 0x{computed_crc:08x}"
            )
        offset = chunk_end

        if chunk_index == 1 and chunk_type != b"IHDR":
            raise PngCodecError("IHDR must be the first PNG chunk")

        if chunk_type == b"IHDR":
            if seen_ihdr or chunk_index != 1:
                raise PngCodecError("PNG must contain exactly one first IHDR chunk")
            if length != 13:
                raise PngCodecError(f"IHDR must be 13 bytes, got {length}")
            (
                width,
                height,
                bit_depth,
                colour_type,
                compression_method,
                filter_method,
                interlace_method,
            ) = struct.unpack(">IIBBBBB", payload)
            _validate_dimensions(width, height, PngCodecError, "PNG")
            if bit_depth != 8:
                raise PngCodecError(
                    f"unsupported PNG bit depth {bit_depth}; only 8-bit is supported"
                )
            if colour_type not in (2, 6):
                raise PngCodecError(
                    f"unsupported PNG colour type {colour_type}; expected RGB (2) "
                    "or RGBA (6)"
                )
            if compression_method != 0:
                raise PngCodecError(
                    f"unsupported PNG compression method {compression_method}"
                )
            if filter_method != 0:
                raise PngCodecError(f"unsupported PNG filter method {filter_method}")
            if interlace_method != 0:
                raise PngCodecError("interlaced PNG images are not supported")
            seen_ihdr = True
            continue

        if not seen_ihdr:
            raise PngCodecError("PNG chunk encountered before IHDR")

        if chunk_type == b"PLTE":
            if seen_plte:
                raise PngCodecError("PNG contains more than one PLTE chunk")
            if seen_idat:
                raise PngCodecError("PLTE must appear before the first IDAT chunk")
            if length == 0 or length % 3 != 0 or length > 256 * 3:
                raise PngCodecError(
                    "PLTE length must contain between 1 and 256 RGB entries"
                )
            seen_plte = True
            continue

        if chunk_type == b"IDAT":
            if idat_ended:
                raise PngCodecError("IDAT chunks must be consecutive")
            seen_idat = True
            idat_parts.append(payload)
            continue

        if seen_idat:
            idat_ended = True

        if chunk_type == b"IEND":
            if length != 0:
                raise PngCodecError(f"IEND must be empty, got {length} data bytes")
            if not seen_idat:
                raise PngCodecError("IEND encountered before any IDAT chunk")
            if offset != len(blob):
                raise PngCodecError(
                    f"unexpected {len(blob) - offset} trailing bytes after IEND"
                )
            seen_iend = True
            break

        # An uppercase first letter denotes a critical chunk.  A decoder must
        # reject critical chunk types it does not understand.
        if not (chunk_type[0] & 0x20):
            raise PngCodecError(
                f"unsupported critical PNG chunk {_chunk_label(chunk_type)}"
            )

    if not seen_iend:
        raise PngCodecError("PNG is missing its IEND chunk")

    return width, height, colour_type, b"".join(idat_parts)


def _decompress_idat(
    compressed: bytes, width: int, height: int, channels: int
) -> bytes:
    row_bytes = width * channels
    scanline_bytes = row_bytes + 1
    if scanline_bytes > sys.maxsize or height > (sys.maxsize - 1) // scanline_bytes:
        raise PngCodecError(f"PNG dimensions {width}x{height} are too large to decode")
    expected = scanline_bytes * height

    decompressor = zlib.decompressobj()
    try:
        decoded = decompressor.decompress(compressed, expected + 1)
        if decompressor.unconsumed_tail:
            raise PngCodecError(
                f"PNG IDAT expands beyond the expected {expected} bytes"
            )
        decoded += decompressor.flush()
    except zlib.error as exc:
        raise PngCodecError(f"invalid PNG IDAT zlib stream: {exc}") from exc

    if not decompressor.eof:
        raise PngCodecError("truncated PNG IDAT zlib stream")
    if decompressor.unused_data:
        raise PngCodecError("unexpected data after the PNG IDAT zlib stream")
    if len(decoded) != expected:
        qualifier = "too much" if len(decoded) > expected else "too little"
        raise PngCodecError(
            f"PNG IDAT produced {qualifier} scanline data: expected {expected} "
            f"bytes, got {len(decoded)}"
        )
    return decoded


def _paeth(left: int, above: int, upper_left: int) -> int:
    estimate = left + above - upper_left
    left_distance = abs(estimate - left)
    above_distance = abs(estimate - above)
    upper_left_distance = abs(estimate - upper_left)
    if left_distance <= above_distance and left_distance <= upper_left_distance:
        return left
    if above_distance <= upper_left_distance:
        return above
    return upper_left


def _unfilter(
    decoded: bytes, width: int, height: int, channels: int
) -> bytes:
    row_bytes = width * channels
    previous = bytearray(row_bytes)
    rgb = bytearray(width * height * 3)
    input_offset = 0
    output_offset = 0

    for row_index in range(height):
        filter_type = decoded[input_offset]
        input_offset += 1
        row = bytearray(decoded[input_offset : input_offset + row_bytes])
        input_offset += row_bytes

        if filter_type == 0:
            pass
        elif filter_type == 1:
            for index in range(row_bytes):
                left = row[index - channels] if index >= channels else 0
                row[index] = (row[index] + left) & 0xFF
        elif filter_type == 2:
            for index in range(row_bytes):
                row[index] = (row[index] + previous[index]) & 0xFF
        elif filter_type == 3:
            for index in range(row_bytes):
                left = row[index - channels] if index >= channels else 0
                row[index] = (row[index] + ((left + previous[index]) >> 1)) & 0xFF
        elif filter_type == 4:
            for index in range(row_bytes):
                left = row[index - channels] if index >= channels else 0
                above = previous[index]
                upper_left = previous[index - channels] if index >= channels else 0
                row[index] = (
                    row[index] + _paeth(left, above, upper_left)
                ) & 0xFF
        else:
            raise PngCodecError(
                f"invalid PNG filter type {filter_type} on row {row_index}"
            )

        if channels == 3:
            rgb[output_offset : output_offset + row_bytes] = row
            output_offset += row_bytes
        else:
            for source_offset in range(0, row_bytes, 4):
                rgb[output_offset] = row[source_offset]
                rgb[output_offset + 1] = row[source_offset + 1]
                rgb[output_offset + 2] = row[source_offset + 2]
                output_offset += 3
        previous = row

    return bytes(rgb)


def read_png(path: object) -> tuple[int, int, bytes]:
    """Read a validated 8-bit RGB/RGBA PNG and return RGB samples.

    All chunk CRCs are checked.  RGBA alpha is discarded after PNG filters are
    reversed; no colour, gamma, or channel-order transformation is performed.
    """

    source = _path(path)
    blob = source.read_bytes()
    width, height, colour_type, compressed = _parse_png(blob)
    channels = 3 if colour_type == 2 else 4
    decoded = _decompress_idat(compressed, width, height, channels)
    return width, height, _unfilter(decoded, width, height, channels)
