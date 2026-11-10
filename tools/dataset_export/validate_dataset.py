#!/usr/bin/env python3
"""Validate an exported rm_aim_sim canonical dataset."""

from __future__ import annotations

import argparse
from collections import Counter
import hashlib
import json
import math
from pathlib import Path
import sys
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from tools.dataset_export.annotations import check_canonical_geometry  # noqa: E402
from tools.dataset_export.png_codec import PngError, read_png  # noqa: E402


EXPECTED_WIDTH = 1440
EXPECTED_HEIGHT = 1080


def _reject_constant(value: str) -> None:
    raise ValueError(f"non-finite JSON number {value}")


def strict_loads(serialized: str) -> Any:
    return json.loads(serialized, parse_constant=_reject_constant)


def load_json(path: Path) -> Any:
    return strict_loads(path.read_text(encoding="utf-8"))


def load_jsonl(path: Path) -> tuple[list[dict[str, Any]], list[str]]:
    records: list[dict[str, Any]] = []
    errors: list[str] = []
    try:
        stream = path.open("r", encoding="utf-8")
    except OSError as error:
        return [], [f"{path}: {error}"]
    with stream:
        for line_number, line in enumerate(stream, 1):
            if not line.strip():
                errors.append(f"{path}:{line_number}: blank JSONL line")
                continue
            try:
                value = strict_loads(line)
                if not isinstance(value, dict):
                    raise ValueError("record is not an object")
                records.append(value)
            except (json.JSONDecodeError, ValueError) as error:
                errors.append(f"{path}:{line_number}: {error}")
    return records, errors


def safe_dataset_path(root: Path, relative: Any) -> tuple[Path | None, str | None]:
    if not isinstance(relative, str) or not relative:
        return None, "path must be a non-empty string"
    candidate = Path(relative)
    if candidate.is_absolute():
        return None, "absolute paths are forbidden"
    resolved = (root / candidate).resolve(strict=False)
    try:
        resolved.relative_to(root)
    except ValueError:
        return None, "path escapes dataset root"
    return resolved, None


def _write_report(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + "\n", encoding="utf-8"
    )
    temporary.replace(path)


def validate_split(root: Path, known_episodes: set[str]) -> list[str]:
    split_path = root / "split.json"
    if not split_path.exists():
        return []
    errors: list[str] = []
    try:
        split = load_json(split_path)
    except (OSError, json.JSONDecodeError, ValueError) as error:
        return [f"split.json: {error}"]
    if not isinstance(split, dict) or not isinstance(split.get("splits"), dict):
        return ["split.json must contain a splits object"]
    ownership: dict[str, str] = {}
    for split_name, payload in split["splits"].items():
        episodes = payload.get("episodes") if isinstance(payload, dict) else None
        if not isinstance(episodes, list):
            errors.append(f"split {split_name!r} episodes must be an array")
            continue
        for episode in episodes:
            if not isinstance(episode, str):
                errors.append(f"split {split_name!r} has a non-string episode")
                continue
            if episode in ownership:
                errors.append(
                    f"episode leakage: {episode!r} appears in {ownership[episode]!r} and {split_name!r}"
                )
            ownership[episode] = split_name
    missing = known_episodes - set(ownership)
    unknown = set(ownership) - known_episodes
    if missing:
        errors.append(f"split.json omits episodes: {sorted(missing)}")
    if unknown:
        errors.append(f"split.json references unknown episodes: {sorted(unknown)}")
    return errors


def validate_dataset(root: Path, *, minimum_empty_ratio: float = 0.0) -> dict[str, Any]:
    root = root.expanduser().resolve()
    errors: list[str] = []
    warnings: list[str] = []
    statistics: Counter[str] = Counter()
    if not root.is_dir():
        return {
            "valid": False,
            "errors": [f"dataset root does not exist: {root}"],
            "warnings": [],
            "statistics": {},
        }
    try:
        manifest = load_json(root / "dataset.json")
    except (OSError, json.JSONDecodeError, ValueError) as error:
        manifest = {}
        errors.append(f"dataset.json: {error}")
    try:
        classes = load_json(root / "classes.json")
        if not isinstance(classes, dict):
            errors.append("classes.json root must be an object")
    except (OSError, json.JSONDecodeError, ValueError) as error:
        errors.append(f"classes.json: {error}")

    canonical_path = root / "annotations" / "canonical.jsonl"
    records, jsonl_errors = load_jsonl(canonical_path)
    errors.extend(jsonl_errors)
    request_ids: set[tuple[str, str]] = set()
    ticks: set[tuple[str, str, int]] = set()
    image_paths: set[str] = set()
    image_digests: dict[str, str] = {}
    record_keys: set[tuple[str, int]] = set()
    known_episodes: set[str] = set()

    for record_number, record in enumerate(records, 1):
        prefix = f"canonical line {record_number}"
        statistics["frames"] += 1
        episode = record.get("episode_id")
        frame_index = record.get("frame_index")
        if not isinstance(episode, str) or not episode:
            errors.append(f"{prefix}: episode_id must be a non-empty string")
        elif not isinstance(frame_index, int) or isinstance(frame_index, bool) or frame_index < 0:
            errors.append(f"{prefix}: frame_index must be a nonnegative integer")
        else:
            key = (episode, frame_index)
            if key in record_keys:
                errors.append(f"{prefix}: duplicate episode/frame index {key}")
            record_keys.add(key)
            known_episodes.add(episode)

        request_key = (str(record.get("publisher_id")), str(record.get("request_id")))
        if request_key in request_ids:
            errors.append(f"{prefix}: duplicate publisher/request_id {request_key}")
        request_ids.add(request_key)
        tick = record.get("tick")
        tick_key = (str(record.get("publisher_id")), str(record.get("session_id")), tick)
        if not isinstance(tick, int) or isinstance(tick, bool):
            errors.append(f"{prefix}: tick must be an integer")
        elif tick_key in ticks:
            errors.append(f"{prefix}: duplicate publisher/session/tick {tick_key}")
        else:
            ticks.add(tick_key)
        sim_time = record.get("sim_time")
        if (
            not isinstance(sim_time, (int, float))
            or isinstance(sim_time, bool)
            or not math.isfinite(float(sim_time))
        ):
            errors.append(f"{prefix}: sim_time must be finite")

        geometry_errors = check_canonical_geometry(record)
        errors.extend(f"{prefix}: {message}" for message in geometry_errors)
        armors = record.get("armors")
        if isinstance(armors, list):
            statistics["instances"] += len(armors)
            if armors:
                statistics["armor_frames"] += 1
            else:
                statistics["empty_frames"] += 1

        relative_image = record.get("image")
        image_path, path_error = safe_dataset_path(root, relative_image)
        if path_error:
            errors.append(f"{prefix}: image {path_error}")
            continue
        if image_path is None:
            errors.append(f"{prefix}: image path could not be resolved")
            continue
        if relative_image in image_paths:
            errors.append(f"{prefix}: duplicate image path {relative_image!r}")
        image_paths.add(relative_image)
        if not image_path.is_file():
            errors.append(f"{prefix}: image does not exist: {relative_image}")
            continue
        try:
            width, height, rgb = read_png(image_path)
        except (OSError, PngError, ValueError) as error:
            errors.append(f"{prefix}: image cannot be decoded: {error}")
            continue
        if (width, height) != (record.get("width"), record.get("height")):
            errors.append(f"{prefix}: PNG dimensions disagree with annotation")
        if (width, height) != (EXPECTED_WIDTH, EXPECTED_HEIGHT):
            errors.append(
                f"{prefix}: exporter v1 requires {EXPECTED_WIDTH}x{EXPECTED_HEIGHT}, got {width}x{height}"
            )
        digest = hashlib.sha256(rgb).hexdigest()
        if digest in image_digests:
            errors.append(
                f"{prefix}: duplicate decoded image content also used by {image_digests[digest]!r}"
            )
        else:
            image_digests[digest] = str(relative_image)

    actual_images = {
        str(path.relative_to(root))
        for path in root.glob("episodes/*/images/*.png")
        if path.is_file()
    }
    missing_annotations = actual_images - image_paths
    missing_images = image_paths - actual_images
    if missing_annotations:
        errors.append(f"images without canonical records: {sorted(missing_annotations)}")
    if missing_images:
        errors.append(f"canonical image paths not in images directories: {sorted(missing_images)}")
    if len(actual_images) != len(records):
        errors.append(
            f"manifest/image count mismatch: {len(records)} canonical records, {len(actual_images)} images"
        )

    for episode in known_episodes:
        episode_root = root / "episodes" / episode
        episode_annotations, episode_errors = load_jsonl(episode_root / "annotations.jsonl")
        errors.extend(episode_errors)
        episode_frames, frame_errors = load_jsonl(episode_root / "frames.jsonl")
        errors.extend(frame_errors)
        expected_count = sum(1 for record in records if record.get("episode_id") == episode)
        if len(episode_annotations) != expected_count:
            errors.append(
                f"episode {episode!r}: {len(episode_annotations)} annotation rows, expected {expected_count}"
            )
        if len(episode_frames) != expected_count:
            errors.append(f"episode {episode!r}: {len(episode_frames)} frame rows, expected {expected_count}")
        episode_keys = {
            (record.get("episode_id"), record.get("frame_index")) for record in episode_annotations
        }
        canonical_keys = {
            (record.get("episode_id"), record.get("frame_index"))
            for record in records
            if record.get("episode_id") == episode
        }
        if episode_keys != canonical_keys:
            errors.append(f"episode {episode!r}: annotations.jsonl keys disagree with canonical JSONL")
        canonical_by_key = {
            (record.get("episode_id"), record.get("frame_index")): record
            for record in records
            if record.get("episode_id") == episode
        }
        for annotation in episode_annotations:
            key = (annotation.get("episode_id"), annotation.get("frame_index"))
            if key in canonical_by_key and annotation != canonical_by_key[key]:
                errors.append(f"episode {episode!r}: annotation row {key} differs from canonical JSONL")
        seen_frame_keys: set[tuple[Any, Any]] = set()
        for frame_row in episode_frames:
            key = (frame_row.get("episode_id"), frame_row.get("frame_index"))
            if key in seen_frame_keys:
                errors.append(f"episode {episode!r}: duplicate frames.jsonl key {key}")
                continue
            seen_frame_keys.add(key)
            canonical = canonical_by_key.get(key)
            metadata = frame_row.get("frame_metadata")
            if canonical is None:
                errors.append(f"episode {episode!r}: frames.jsonl key {key} has no canonical row")
                continue
            if not isinstance(metadata, dict):
                errors.append(f"episode {episode!r}: frames.jsonl key {key} lacks frame_metadata")
                continue
            if frame_row.get("image") != canonical.get("image"):
                errors.append(f"episode {episode!r}: frames.jsonl image differs at {key}")
            for field in ("request_id", "publisher_id", "session_id", "tick", "sim_time"):
                if metadata.get(field) != canonical.get(field):
                    errors.append(
                        f"episode {episode!r}: frame_metadata.{field} differs from canonical at {key}"
                    )
            camera = metadata.get("camera")
            if not isinstance(camera, dict):
                errors.append(f"episode {episode!r}: frame_metadata.camera missing at {key}")
                continue
            if camera.get("width") != canonical.get("width") or camera.get("height") != canonical.get(
                "height"
            ):
                errors.append(f"episode {episode!r}: frame dimensions differ from canonical at {key}")
            source_encoding = metadata.get("encoding")
            if source_encoding != canonical.get("source_encoding"):
                errors.append(f"episode {episode!r}: frame encoding differs from canonical at {key}")
            if camera.get("tick") != metadata.get("tick") or camera.get("timestamp") != metadata.get(
                "sim_time"
            ):
                errors.append(f"episode {episode!r}: frame camera snapshot is not synchronous at {key}")

    if isinstance(manifest, dict):
        totals = manifest.get("totals")
        if not isinstance(totals, dict):
            errors.append("dataset.json totals must be an object")
        else:
            if totals.get("frames") != len(records):
                errors.append("dataset.json total frame count disagrees with canonical JSONL")
            if totals.get("instances") != statistics["instances"]:
                errors.append("dataset.json total instance count disagrees with canonical JSONL")

    errors.extend(validate_split(root, known_episodes))
    empty_ratio = statistics["empty_frames"] / statistics["frames"] if statistics["frames"] else 0.0
    statistics["image_files"] = len(actual_images)
    statistics_dict = dict(statistics)
    statistics_dict["empty_ratio"] = empty_ratio
    if statistics["frames"] == 0:
        errors.append("dataset contains no frames")
    if empty_ratio + 1e-12 < minimum_empty_ratio:
        errors.append(
            f"empty-frame ratio {empty_ratio:.6f} is below required {minimum_empty_ratio:.6f}"
        )
    if statistics["empty_frames"] == 0:
        warnings.append("dataset contains no empty scenes")
    if not (root / "split.json").exists():
        warnings.append("split.json is absent; run split_dataset.py after collecting multiple episodes")
    return {
        "valid": not errors,
        "errors": errors,
        "warnings": warnings,
        "statistics": statistics_dict,
    }


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument(
        "--min-empty-ratio", type=float, default=0.0, help="fail below this required empty-scene ratio"
    )
    parser.add_argument("--no-write-report", action="store_true")
    args = parser.parse_args()
    if not math.isfinite(args.min_empty_ratio) or not 0 <= args.min_empty_ratio <= 1:
        parser.error("--min-empty-ratio must be finite and in [0,1]")
    return args


def main() -> int:
    args = parse_arguments()
    result = validate_dataset(args.dataset, minimum_empty_ratio=args.min_empty_ratio)
    if not args.no_write_report and args.dataset.is_dir():
        _write_report(args.dataset / "reports" / "validation.json", result)
    print(json.dumps(result, ensure_ascii=False, allow_nan=False, indent=2))
    return 0 if result["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
