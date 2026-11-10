"""Canonical armor annotations and geometry validation.

The simulator already assigns semantic corner identities.  This module never
sorts points by x/y; it validates and preserves TL, TR, BR, BL verbatim.
"""

from __future__ import annotations

import math
from typing import Any, Iterable


SCHEMA_VERSION = 1
ROBOT_NUMBERS = {
    "hero": 1,
    "infantry_3": 3,
    "infantry_4": 4,
    "infantry_5": 5,
    "sentry": 7,
}
ROBOT_ARMOR_TYPES = {
    "hero": "large",
    "infantry_3": "small",
    "infantry_4": "small",
    "infantry_5": "small",
    "sentry": "small",
}


class AnnotationError(ValueError):
    """Raised when a canonical annotation cannot be constructed safely."""


def _finite_number(value: Any, field: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise AnnotationError(f"{field} must be a number")
    number = float(value)
    if not math.isfinite(number):
        raise AnnotationError(f"{field} must be finite")
    return number


def _integer(value: Any, field: str, minimum: int | None = None) -> int:
    number = _finite_number(value, field)
    if number != math.floor(number):
        raise AnnotationError(f"{field} must be an integer")
    result = int(number)
    if minimum is not None and result < minimum:
        raise AnnotationError(f"{field} must be >= {minimum}")
    return result


def _numeric_array(value: Any, length: int, field: str) -> list[float]:
    if not isinstance(value, list) or len(value) != length:
        raise AnnotationError(f"{field} must contain exactly {length} numbers")
    return [_finite_number(item, f"{field}[{index}]") for index, item in enumerate(value)]


def signed_polygon_area(corners: Iterable[Iterable[float]]) -> float:
    points = [tuple(point) for point in corners]
    return 0.5 * sum(
        points[index][0] * points[(index + 1) % len(points)][1]
        - points[(index + 1) % len(points)][0] * points[index][1]
        for index in range(len(points))
    )


def _orientation(a: tuple[float, float], b: tuple[float, float], c: tuple[float, float]) -> float:
    return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])


def _on_segment(a: tuple[float, float], b: tuple[float, float], p: tuple[float, float]) -> bool:
    epsilon = 1e-9
    return (
        min(a[0], b[0]) - epsilon <= p[0] <= max(a[0], b[0]) + epsilon
        and min(a[1], b[1]) - epsilon <= p[1] <= max(a[1], b[1]) + epsilon
        and abs(_orientation(a, b, p)) <= epsilon
    )


def _segments_intersect(
    a: tuple[float, float],
    b: tuple[float, float],
    c: tuple[float, float],
    d: tuple[float, float],
) -> bool:
    epsilon = 1e-9
    ab_c = _orientation(a, b, c)
    ab_d = _orientation(a, b, d)
    cd_a = _orientation(c, d, a)
    cd_b = _orientation(c, d, b)
    if ((ab_c > epsilon and ab_d < -epsilon) or (ab_c < -epsilon and ab_d > epsilon)) and (
        (cd_a > epsilon and cd_b < -epsilon) or (cd_a < -epsilon and cd_b > epsilon)
    ):
        return True
    return (
        (abs(ab_c) <= epsilon and _on_segment(a, b, c))
        or (abs(ab_d) <= epsilon and _on_segment(a, b, d))
        or (abs(cd_a) <= epsilon and _on_segment(c, d, a))
        or (abs(cd_b) <= epsilon and _on_segment(c, d, b))
    )


def validate_corners(
    corners: Any,
    width: int,
    height: int,
    *,
    minimum_area: float = 1.0,
) -> dict[str, Any]:
    """Validate TL/TR/BR/BL points without reordering them.

    Positive signed area is the expected clockwise winding in image
    coordinates (x right, y down).  It is the only safe generic ordering
    check under perspective; x/y sorting would destroy semantic identities.
    """

    if not isinstance(corners, list) or len(corners) != 4:
        raise AnnotationError("corners must contain exactly four TL/TR/BR/BL points")
    if width <= 0 or height <= 0:
        raise AnnotationError("image dimensions must be positive")
    if not math.isfinite(minimum_area) or minimum_area <= 0:
        raise AnnotationError("minimum_area must be finite and positive")

    points: list[tuple[float, float]] = []
    for index, point in enumerate(corners):
        if not isinstance(point, (list, tuple)) or len(point) != 2:
            raise AnnotationError(f"corners[{index}] must contain u,v")
        points.append(
            (
                _finite_number(point[0], f"corners[{index}][0]"),
                _finite_number(point[1], f"corners[{index}][1]"),
            )
        )

    if _segments_intersect(points[0], points[1], points[2], points[3]) or _segments_intersect(
        points[1], points[2], points[3], points[0]
    ):
        raise AnnotationError("TL/TR/BR/BL polygon self-intersects")

    signed_area = signed_polygon_area(points)
    if signed_area <= minimum_area:
        if signed_area < -minimum_area:
            raise AnnotationError("corners have reversed winding; expected TL/TR/BR/BL")
        raise AnnotationError(f"polygon area must be greater than {minimum_area:g} px^2")

    xs = [point[0] for point in points]
    ys = [point[1] for point in points]
    bbox = [min(xs), min(ys), max(xs), max(ys)]
    if bbox[2] <= bbox[0] or bbox[3] <= bbox[1]:
        raise AnnotationError("corner bbox must have positive width and height")

    truncated = any(
        point[0] < 0.0 or point[0] >= width or point[1] < 0.0 or point[1] >= height
        for point in points
    )
    clipped = [
        min(max(bbox[0], 0.0), float(width)),
        min(max(bbox[1], 0.0), float(height)),
        min(max(bbox[2], 0.0), float(width)),
        min(max(bbox[3], 0.0), float(height)),
    ]
    center = [sum(xs) / 4.0, sum(ys) / 4.0]
    return {
        "corners": [[point[0], point[1]] for point in points],
        "bbox_xyxy": bbox,
        "bbox_clipped_xyxy": clipped,
        "center_uv": center,
        "polygon_area_px2": signed_area,
        "truncated": truncated,
        "winding": "clockwise_image",
    }


def armor_type_for_robot(robot: str) -> str:
    try:
        return ROBOT_ARMOR_TYPES[robot]
    except KeyError as error:
        raise AnnotationError(f"unsupported appearance.robot: {robot!r}") from error


def canonical_record(
    frame: dict[str, Any],
    image_path: str,
    episode_id: str,
    frame_index: int,
    *,
    minimum_area: float = 1.0,
    scene: dict[str, Any] | None = None,
) -> dict[str, Any]:
    """Create one lossless, model-independent record from frame.json."""

    if frame.get("protocol") != "rm_aim_sim/2" or frame.get("type") != "frame":
        raise AnnotationError("not an rm_aim_sim/2 frame packet")
    publisher_id = frame.get("publisher_id")
    session_id = frame.get("session_id")
    request_id = frame.get("request_id")
    if not all(isinstance(value, str) and value for value in (publisher_id, session_id, request_id)):
        raise AnnotationError("publisher_id, session_id and request_id must be non-empty strings")

    tick = _integer(frame.get("tick"), "tick", 0)
    sim_time = _finite_number(frame.get("sim_time"), "sim_time")
    if sim_time < 0:
        raise AnnotationError("sim_time must be nonnegative")
    camera = frame.get("camera")
    if not isinstance(camera, dict):
        raise AnnotationError("camera must be an object")
    width = _integer(camera.get("width"), "camera.width", 1)
    height = _integer(camera.get("height"), "camera.height", 1)
    camera_timestamp = _finite_number(camera.get("timestamp"), "camera.timestamp")
    camera_tick = _integer(camera.get("tick"), "camera.tick", 0)
    if abs(camera_timestamp - sim_time) > 1e-6 or camera_tick != tick:
        raise AnnotationError("camera timestamp/tick must match the frame snapshot")
    _numeric_array(camera.get("K"), 9, "camera.K")
    _numeric_array(camera.get("distortion"), 5, "camera.distortion")
    _numeric_array(camera.get("R_camera2gimbal"), 9, "camera.R_camera2gimbal")
    _numeric_array(camera.get("t_camera2gimbal"), 3, "camera.t_camera2gimbal")
    _numeric_array(camera.get("origin"), 3, "camera.origin")
    _finite_number(camera.get("yaw"), "camera.yaw")
    _finite_number(camera.get("pitch"), "camera.pitch")
    _finite_number(camera.get("brightness_gain"), "camera.brightness_gain")
    appearance = frame.get("appearance")
    if not isinstance(appearance, dict):
        raise AnnotationError("appearance must be an object")
    robot = appearance.get("robot")
    team = appearance.get("team")
    if robot not in ROBOT_NUMBERS:
        raise AnnotationError(f"unsupported appearance.robot: {robot!r}")
    if team not in ("red", "blue"):
        raise AnnotationError("appearance.team must be red or blue")
    source_armors = camera.get("armors")
    if not isinstance(source_armors, list) or len(source_armors) > 4:
        raise AnnotationError("camera.armors must be an array of at most four entries")

    armors: list[dict[str, Any]] = []
    seen_ids: set[int] = set()
    armor_type = armor_type_for_robot(robot)
    for armor_index, source in enumerate(source_armors):
        if not isinstance(source, dict):
            raise AnnotationError(f"camera.armors[{armor_index}] must be an object")
        armor_id = _integer(source.get("id"), f"camera.armors[{armor_index}].id", 0)
        if armor_id > 3:
            raise AnnotationError("camera armor id must be in 0..3")
        if armor_id in seen_ids:
            raise AnnotationError("camera.armors contains a duplicate physical id")
        seen_ids.add(armor_id)
        geometry = validate_corners(source.get("corners"), width, height, minimum_area=minimum_area)
        armors.append(
            {
                "armor_id": armor_id,
                "team": team,
                "robot": robot,
                "robot_number": ROBOT_NUMBERS[robot],
                "armor_type": armor_type,
                "corners_tl_tr_br_bl": geometry["corners"],
                "bbox_xyxy": geometry["bbox_xyxy"],
                "bbox_clipped_xyxy": geometry["bbox_clipped_xyxy"],
                "center_uv": geometry["center_uv"],
                "polygon_area_px2": geometry["polygon_area_px2"],
                "truncated": geometry["truncated"],
                # No depth/raycast test exists.  Null is intentional: false
                # would incorrectly claim that the target is unobstructed/easy.
                "difficult": None,
                "occlusion": None,
                "visibility": "geometric_projection_only",
                "winding": geometry["winding"],
                "field_provenance": {
                    "armor_id": "frame.camera.armors[].id",
                    "team": "frame.appearance.team",
                    "robot": "frame.appearance.robot",
                    "robot_number": "derived_from_frame.appearance.robot",
                    "armor_type": "derived_from_frame.appearance.robot_preset",
                    "corners_tl_tr_br_bl": "frame.camera.armors[].corners",
                    "bbox_center_area_truncated": "derived_from_corners_and_image_bounds",
                    "difficult_occlusion": "unavailable_in_protocol",
                },
            }
        )

    gimbal = frame.get("gimbal")
    if not isinstance(gimbal, dict):
        raise AnnotationError("gimbal must be an object")
    _numeric_array(gimbal.get("origin"), 3, "gimbal.origin")
    _numeric_array(gimbal.get("quaternion_wxyz"), 4, "gimbal.quaternion_wxyz")
    source_encoding = frame.get("encoding")
    if not isinstance(source_encoding, str) or source_encoding.lower() not in ("rgb8", "rgba8"):
        raise AnnotationError("encoding must be rgb8 or rgba8")

    camera_record = {
        "K": camera.get("K"),
        "distortion": camera.get("distortion"),
        "R_camera2gimbal": camera.get("R_camera2gimbal"),
        "t_camera2gimbal": camera.get("t_camera2gimbal"),
        "gimbal_quaternion_wxyz": gimbal.get("quaternion_wxyz"),
        "origin_world": camera.get("origin"),
        "yaw": camera.get("yaw"),
        "pitch": camera.get("pitch"),
        "brightness_gain": camera.get("brightness_gain"),
        "timestamp": camera.get("timestamp"),
        "tick": camera.get("tick"),
    }
    return {
        "schema_version": SCHEMA_VERSION,
        "episode_id": episode_id,
        "frame_index": frame_index,
        "image": image_path,
        "request_id": request_id,
        "publisher_id": publisher_id,
        "session_id": session_id,
        "tick": tick,
        "sim_time": sim_time,
        "width": width,
        "height": height,
        "encoding": source_encoding.upper(),
        "source_encoding": source_encoding,
        "camera": camera_record,
        "appearance": appearance,
        "health": frame.get("health"),
        "gimbal": gimbal,
        "weapon": frame.get("weapon"),
        "paused": frame.get("paused"),
        "ipc": frame.get("ipc"),
        "capture_timing": frame.get("capture_timing"),
        "scene": scene or {},
        "armors": armors,
        "frame_local_metadata": {
            "appearance": appearance,
            "health": frame.get("health"),
            "gimbal": gimbal,
            "weapon": frame.get("weapon"),
            "stats": frame.get("stats"),
            "ipc": frame.get("ipc"),
            "paused": frame.get("paused"),
        },
        "annotation_semantics": {
            "corner_order": "TL_TR_BR_BL",
            "coordinates": "pixel_u_right_v_down",
            "visibility": "front_facing_lit_and_all_four_keypoints_in_frame; no pixel occlusion test",
            "armor_type": "derived preset; actual lightbar dimensions are not carried by frame.json",
        },
    }


def check_canonical_geometry(record: dict[str, Any], *, minimum_area: float = 1.0) -> list[str]:
    """Return geometry/schema errors for a canonical record."""

    errors: list[str] = []
    try:
        width = _integer(record.get("width"), "width", 1)
        height = _integer(record.get("height"), "height", 1)
    except AnnotationError as error:
        return [str(error)]
    try:
        tick = _integer(record.get("tick"), "tick", 0)
        sim_time = _finite_number(record.get("sim_time"), "sim_time")
        camera = record.get("camera")
        if not isinstance(camera, dict):
            raise AnnotationError("camera must be an object")
        _numeric_array(camera.get("K"), 9, "camera.K")
        _numeric_array(camera.get("distortion"), 5, "camera.distortion")
        _numeric_array(camera.get("R_camera2gimbal"), 9, "camera.R_camera2gimbal")
        _numeric_array(camera.get("t_camera2gimbal"), 3, "camera.t_camera2gimbal")
        _numeric_array(camera.get("gimbal_quaternion_wxyz"), 4, "camera.gimbal_quaternion_wxyz")
        if abs(_finite_number(camera.get("timestamp"), "camera.timestamp") - sim_time) > 1e-6:
            raise AnnotationError("camera.timestamp differs from sim_time")
        if _integer(camera.get("tick"), "camera.tick", 0) != tick:
            raise AnnotationError("camera.tick differs from tick")
        if record.get("source_encoding") not in ("rgb8", "rgba8"):
            raise AnnotationError("source_encoding must be rgb8 or rgba8")
        if record.get("encoding") != str(record.get("source_encoding")).upper():
            raise AnnotationError("encoding must preserve normalized source_encoding")
    except AnnotationError as error:
        errors.append(str(error))
    armors = record.get("armors")
    if not isinstance(armors, list):
        return ["armors must be an array"]
    appearance = record.get("appearance")
    record_robot = appearance.get("robot") if isinstance(appearance, dict) else None
    record_team = appearance.get("team") if isinstance(appearance, dict) else None
    seen_ids: set[int] = set()
    for index, armor in enumerate(armors):
        prefix = f"armors[{index}]"
        if not isinstance(armor, dict):
            errors.append(f"{prefix} must be an object")
            continue
        try:
            armor_id = _integer(armor.get("armor_id"), f"{prefix}.armor_id", 0)
            if armor_id > 3 or armor_id in seen_ids:
                raise AnnotationError(f"{prefix}.armor_id must be unique in 0..3")
            seen_ids.add(armor_id)
            geometry = validate_corners(
                armor.get("corners_tl_tr_br_bl"), width, height, minimum_area=minimum_area
            )
            for field in ("bbox_xyxy", "bbox_clipped_xyxy", "center_uv"):
                actual = armor.get(field)
                expected = geometry[field]
                if not isinstance(actual, list) or len(actual) != len(expected) or any(
                    not isinstance(value, (int, float))
                    or isinstance(value, bool)
                    or not math.isfinite(float(value))
                    or abs(float(value) - reference) > 1e-6
                    for value, reference in zip(actual, expected)
                ):
                    errors.append(f"{prefix}.{field} is inconsistent with corners")
            area = _finite_number(armor.get("polygon_area_px2"), f"{prefix}.polygon_area_px2")
            if abs(area - geometry["polygon_area_px2"]) > 1e-6:
                errors.append(f"{prefix}.polygon_area_px2 is inconsistent with corners")
            if armor.get("truncated") is not geometry["truncated"]:
                errors.append(f"{prefix}.truncated is inconsistent with image bounds")
            expected_type = armor_type_for_robot(armor.get("robot"))
            if armor.get("armor_type") != expected_type:
                errors.append(f"{prefix}.armor_type is inconsistent with robot")
            if armor.get("robot_number") != ROBOT_NUMBERS.get(armor.get("robot")):
                errors.append(f"{prefix}.robot_number is inconsistent with robot")
            if armor.get("team") not in ("red", "blue"):
                errors.append(f"{prefix}.team must be red or blue")
            if armor.get("robot") != record_robot or armor.get("team") != record_team:
                errors.append(f"{prefix} class fields disagree with frame appearance")
        except (AnnotationError, TypeError) as error:
            errors.append(f"{prefix}: {error}")
    return errors
