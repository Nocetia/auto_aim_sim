#!/usr/bin/env python3
"""Export synchronized raw frames and canonical armor labels from rm_aim_sim."""

from __future__ import annotations

import argparse
from collections import Counter
from configparser import ConfigParser
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import subprocess
import sys
import time
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from tools.dataset_export.annotations import AnnotationError, canonical_record  # noqa: E402
from tools.dataset_export.dataset_protocol import (  # noqa: E402
    ContextChangedError,
    ContinuityError,
    FrameProtocolError,
    StreamContinuity,
    StrictFrameClient,
)
from tools.dataset_export.png_codec import read_png, raw_to_rgb, write_png  # noqa: E402
from tools.dataset_export.preview_render import write_preview  # noqa: E402


EXPECTED_WIDTH = 1440
EXPECTED_HEIGHT = 1080


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat().replace("+00:00", "Z")


def atomic_json(path: Path, value: Any) -> None:
    temporary = path.with_name(path.name + ".tmp")
    temporary.write_text(
        json.dumps(value, ensure_ascii=False, allow_nan=False, indent=2) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def compact_json(value: Any) -> str:
    return json.dumps(value, ensure_ascii=False, allow_nan=False, separators=(",", ":"))


def command_fingerprint(path: Path) -> dict[str, Any] | None:
    try:
        info = path.stat()
        data = path.read_bytes()
    except FileNotFoundError:
        return None
    return {
        "inode": info.st_ino,
        "size": info.st_size,
        "mtime_ns": info.st_mtime_ns,
        "sha256": hashlib.sha256(data).hexdigest(),
    }


def git_metadata(repository: Path) -> dict[str, Any]:
    def run(*arguments: str) -> str:
        try:
            result = subprocess.run(
                ["git", *arguments],
                cwd=repository,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=5,
            )
            return result.stdout.strip()
        except (OSError, subprocess.SubprocessError):
            return ""

    commit = run("rev-parse", "HEAD")
    status = run("status", "--short")
    return {
        "commit": commit or None,
        "dirty": bool(status),
        "status_short": status.splitlines() if status else [],
    }


def godot_version() -> str | None:
    candidates = [REPOSITORY_ROOT / ".tools" / "godot"]
    for candidate in candidates:
        if not candidate.is_file():
            continue
        try:
            result = subprocess.run(
                [str(candidate), "--version"],
                cwd=REPOSITORY_ROOT,
                check=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.DEVNULL,
                text=True,
                timeout=5,
            )
            return result.stdout.strip() or None
        except (OSError, subprocess.SubprocessError):
            pass
    return None


def resolve_config_path(value: str | None) -> Path | None:
    if not value:
        return None
    if value.startswith("res://"):
        return REPOSITORY_ROOT / value.removeprefix("res://")
    candidate = Path(value).expanduser()
    if not candidate.is_absolute():
        candidate = REPOSITORY_ROOT / candidate
    return candidate


def scene_declaration(args: argparse.Namespace) -> dict[str, Any]:
    result: dict[str, Any] = {
        "scene_id": args.scene_id,
        "config_declaration": args.config,
        "random_seed": args.random_seed,
        "trajectory_id": args.trajectory_id or args.episode_id,
        "source": "cli_declaration_not_synchronized_by_frame_protocol",
    }
    config_path = resolve_config_path(args.config)
    if config_path is None:
        result["config_note"] = "No config declared; frame.json does not carry target geometry."
        return result
    result["resolved_config_path"] = str(config_path.resolve(strict=False))
    try:
        raw = config_path.read_bytes()
    except OSError as error:
        result["config_read_error"] = str(error)
        return result
    result["config_sha256"] = hashlib.sha256(raw).hexdigest()
    parser = ConfigParser(interpolation=None)
    try:
        parser.read_string(raw.decode("utf-8"))
        if parser.has_section("target"):
            result["declared_target_geometry"] = {
                key: parser.get("target", key)
                for key in (
                    "lightbar_spacing",
                    "lightbar_length",
                    "module_width",
                    "module_height",
                    "armor_width",
                    "armor_height",
                )
                if parser.has_option("target", key)
            }
        if parser.has_option("ipc", "channel"):
            result["declared_ipc_channel"] = parser.get("ipc", "channel").strip('"')
    except (UnicodeDecodeError, ValueError) as error:
        result["config_parse_error"] = str(error)
    return result


def classes_manifest() -> dict[str, Any]:
    return {
        "schema_version": 1,
        "canonical": {
            "team": ["blue", "red"],
            "robot": {
                "hero": {"robot_number": 1, "armor_type": "large", "yolov5_num_id": 1},
                "infantry_3": {"robot_number": 3, "armor_type": "small", "yolov5_num_id": 3},
                "infantry_4": {"robot_number": 4, "armor_type": "small", "yolov5_num_id": 4},
                "infantry_5": {"robot_number": 5, "armor_type": "small", "yolov5_num_id": 5},
                "sentry": {"robot_number": 7, "armor_type": "small", "yolov5_num_id": 0},
            },
            "armor_id": "physical face index 0..3; never a detector class or robot number",
            "corner_order": ["TL", "TR", "BR", "BL"],
        },
        "current_auto_aim_detector": {
            "backend_observed_in_configs": "custom yolov5",
            "output_shape": [1, 25200, 22],
            "heads": {
                "keypoint_coordinates": 8,
                "objectness_logit": 1,
                "color_logits": 4,
                "number_logits": 9,
            },
            "runtime_color_head": {
                "0": "blue",
                "1": "red",
                "2": "extinguish",
                "3": "extinguish",
            },
            "number_head": {
                "0": "sentry",
                "1": "one(hero)",
                "2": "two",
                "3": "three(infantry_3)",
                "4": "four(infantry_4)",
                "5": "five(infantry_5)",
                "6": "outpost",
                "7": "base",
                "8": "not_armor",
            },
            "runtime_corner_order": ["TL", "TR", "BR", "BL"],
            "network_pair_to_runtime_corner_indices": [0, 3, 2, 1],
            "training_compatibility": (
                "not direct: this checkout has no training reader/loss/export contract, and the "
                "deployed network uses separate color and number heads"
            ),
        },
        "coco_keypoints_converter": {
            "category": {"id": 1, "name": "armor"},
            "keypoints": ["TL", "TR", "BR", "BL"],
            "class_fields_retained_in_attributes": ["team", "robot", "armor_type", "armor_id"],
            "direct_current_model_training": False,
        },
    }


class DatasetWriter:
    def __init__(
        self,
        root: Path,
        episode_id: str,
        *,
        preview_every: int,
        minimum_area: float,
        scene: dict[str, Any],
    ):
        self.root = root
        self.episode_id = episode_id
        self.preview_every = preview_every
        self.minimum_area = minimum_area
        self.scene = scene
        self.episode_root = root / "episodes" / episode_id
        self.images = self.episode_root / "images"
        self.previews = self.episode_root / "previews"
        self.annotation_path = self.episode_root / "annotations.jsonl"
        self.frame_path = self.episode_root / "frames.jsonl"
        self.canonical_path = root / "annotations" / "canonical.jsonl"
        self.images.mkdir(parents=True)
        self.previews.mkdir(parents=True)
        self.canonical_path.parent.mkdir(parents=True)
        (root / "reports").mkdir(parents=True)
        self._episode_annotations = self.annotation_path.open("w", encoding="utf-8")
        self._frames = self.frame_path.open("w", encoding="utf-8")
        self._canonical = self.canonical_path.open("w", encoding="utf-8")

    def close(self) -> None:
        for stream in (self._episode_annotations, self._frames, self._canonical):
            if not stream.closed:
                stream.close()

    def write(self, frame: dict[str, Any], raw: bytes, index: int) -> tuple[dict[str, Any], str]:
        width = int(frame["camera"]["width"])
        height = int(frame["camera"]["height"])
        if (width, height) != (EXPECTED_WIDTH, EXPECTED_HEIGHT):
            raise FrameProtocolError(
                f"dataset exporter v1 requires {EXPECTED_WIDTH}x{EXPECTED_HEIGHT}, got {width}x{height}"
            )
        relative_image = f"episodes/{self.episode_id}/images/frame_{index:06d}.png"
        # Validate every annotation before creating the image.  A rejected
        # near-degenerate polygon must not leave an orphan PNG behind.
        record = canonical_record(
            frame,
            relative_image,
            self.episode_id,
            index,
            minimum_area=self.minimum_area,
            scene=self.scene,
        )
        rgb = raw_to_rgb(raw, width, height, frame["encoding"])
        image_path = self.root / relative_image
        write_png(image_path, width, height, rgb)
        verified_width, verified_height, verified_rgb = read_png(image_path)
        if (verified_width, verified_height) != (width, height) or verified_rgb != rgb:
            raise RuntimeError("written PNG failed lossless decode verification")
        serialized = compact_json(record) + "\n"
        self._episode_annotations.write(serialized)
        self._canonical.write(serialized)
        self._frames.write(
            compact_json(
                {
                    "schema_version": 1,
                    "episode_id": self.episode_id,
                    "frame_index": index,
                    "image": relative_image,
                    "frame_metadata": frame,
                }
            )
            + "\n"
        )
        for stream in (self._episode_annotations, self._frames, self._canonical):
            stream.flush()
        if self.preview_every and index % self.preview_every == 0:
            write_preview(image_path, record, self.previews / f"frame_{index:06d}_overlay.png")
        return record, hashlib.sha256(rgb).hexdigest()


def prepare_output(path: Path) -> None:
    if path.exists():
        if not path.is_dir():
            raise ValueError("--output exists and is not a directory")
        if any(path.iterdir()):
            raise ValueError("--output must not exist or must be an empty directory")
    else:
        path.mkdir(parents=True)


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--channel", default="default", help="IPC channel; one client per channel")
    parser.add_argument("--output", required=True, type=Path, help="new or empty dataset directory")
    parser.add_argument("--episode-id", required=True, help="safe episode identifier")
    parser.add_argument("--fps", type=float, default=10.0, help="target simulation-time sampling FPS")
    parser.add_argument("--duration", type=float, default=0.0, help="simulation seconds; 0 is unlimited")
    parser.add_argument("--max-frames", type=int, default=0, help="saved frames; 0 is unlimited")
    parser.add_argument("--image-format", choices=("png",), default="png")
    empty_group = parser.add_mutually_exclusive_group()
    empty_group.add_argument("--include-empty", dest="include_empty", action="store_true")
    empty_group.add_argument("--exclude-empty", dest="include_empty", action="store_false")
    parser.set_defaults(include_empty=True)
    parser.add_argument("--preview-every", type=int, default=0, help="write one overlay every N saved frames")
    parser.add_argument("--config", help="scene config declaration/path; recorded, never applied")
    parser.add_argument("--scene-id", help="free-form reproducible scene identifier")
    parser.add_argument("--trajectory-id", help="trajectory grouping key; defaults to episode id")
    parser.add_argument("--random-seed", type=int, help="declared simulator/domain-randomization seed")
    parser.add_argument("--timeout", type=float, default=5.0, help="state/frame wall-clock timeout")
    parser.add_argument("--minimum-area", type=float, default=1.0, help="minimum armor polygon area px^2")
    args = parser.parse_args()
    import re

    if not re.fullmatch(r"[A-Za-z0-9_-]{1,80}", args.episode_id):
        parser.error("--episode-id must contain 1-80 letters, digits, '_' or '-'")
    if not math.isfinite(args.fps) or args.fps <= 0:
        parser.error("--fps must be finite and positive")
    if not math.isfinite(args.duration) or args.duration < 0:
        parser.error("--duration must be finite and nonnegative")
    if args.max_frames < 0 or args.preview_every < 0:
        parser.error("--max-frames and --preview-every must be nonnegative")
    if not math.isfinite(args.timeout) or args.timeout <= 0:
        parser.error("--timeout must be finite and positive")
    if not math.isfinite(args.minimum_area) or args.minimum_area <= 0:
        parser.error("--minimum-area must be finite and positive")
    return args


def main() -> int:
    args = parse_arguments()
    output = args.output.expanduser().resolve()
    try:
        prepare_output(output)
    except (OSError, ValueError) as error:
        print(f"dataset exporter: {error}", file=sys.stderr)
        return 2

    created_at = utc_now()
    scene = scene_declaration(args)
    atomic_json(output / "classes.json", classes_manifest())
    writer = DatasetWriter(
        output,
        args.episode_id,
        preview_every=args.preview_every,
        minimum_area=args.minimum_area,
        scene=scene,
    )
    stats: Counter[str] = Counter()
    distributions: dict[str, Counter[str]] = {
        "team": Counter(),
        "robot": Counter(),
        "armor_type": Counter(),
    }
    continuity = StreamContinuity()
    interval = 1.0 / args.fps
    first_seen_sim: float | None = None
    first_saved_sim: float | None = None
    last_saved_sim: float | None = None
    last_sampled_sim: float | None = None
    first_frame: dict[str, Any] | None = None
    encodings: set[str] = set()
    failure: str | None = None
    interrupted = False
    client: StrictFrameClient | None = None
    command_before: dict[str, Any] | None = None
    command_after: dict[str, Any] | None = None
    wall_started = time.monotonic()
    exit_code = 0

    try:
        client = StrictFrameClient(args.channel, timeout=args.timeout)
        command_path = client.directory / "command.json"
        command_before = command_fingerprint(command_path)
        while True:
            state = client.read_state(timeout=args.timeout)
            if state.get("paused"):
                stats["paused_states"] += 1
                continue
            if not state.get("ipc", {}).get("enabled"):
                raise RuntimeError("simulator external IPC is disabled; start it with --remote")
            request_id = client.request_frame()
            stats["requested_frames"] += 1
            try:
                frame, raw = client.receive_frame(request_id, timeout=args.timeout)
            except ContextChangedError:
                raise
            except FrameProtocolError as error:
                stats["rejected_frames"] += 1
                print(f"Rejected frame request {request_id}: {error}", file=sys.stderr)
                continue
            stats["received_frames"] += 1
            try:
                duplicate_reason = continuity.observe(frame)
            except ContinuityError:
                raise
            if duplicate_reason:
                stats["duplicate_frames"] += 1
                stats[duplicate_reason] += 1
                continue
            sim_time = float(frame["sim_time"])
            if first_seen_sim is None:
                first_seen_sim = sim_time
            if args.duration and sim_time - first_seen_sim >= args.duration:
                break
            if last_sampled_sim is not None and sim_time - last_sampled_sim < interval - 1e-9:
                stats["cadence_skipped_frames"] += 1
                continue
            last_sampled_sim = sim_time
            source_armors = frame["camera"]["armors"]
            if not source_armors and not args.include_empty:
                stats["excluded_empty_frames"] += 1
                continue
            width = int(frame["camera"]["width"])
            height = int(frame["camera"]["height"])
            rgb = raw_to_rgb(raw, width, height, frame["encoding"])
            digest = hashlib.sha256(rgb).hexdigest()
            if not continuity.observe_saved_image(digest):
                stats["duplicate_frames"] += 1
                stats["duplicate_image"] += 1
                continue

            index = stats["saved_frames"]
            try:
                record, written_digest = writer.write(frame, raw, index)
            except AnnotationError as error:
                stats["rejected_frames"] += 1
                stats["rejected_annotation_frames"] += 1
                print(f"Rejected annotation for request {request_id}: {error}", file=sys.stderr)
                continue
            if written_digest != digest:
                raise RuntimeError("image digest changed during export")
            if first_frame is None:
                first_frame = frame
            encodings.add(frame["encoding"])
            stats["saved_frames"] += 1
            if record["armors"]:
                stats["armor_frames"] += 1
            else:
                stats["empty_frames"] += 1
            stats["armor_instances"] += len(record["armors"])
            for armor in record["armors"]:
                for field in distributions:
                    distributions[field][str(armor[field])] += 1
            if first_saved_sim is None:
                first_saved_sim = sim_time
            last_saved_sim = sim_time
            if stats["saved_frames"] == 1 or stats["saved_frames"] % 10 == 0:
                print(
                    compact_json(
                        {
                            "type": "dataset_export_progress",
                            "requested": stats["requested_frames"],
                            "saved": stats["saved_frames"],
                            "sim_time": sim_time,
                            "instances": stats["armor_instances"],
                        }
                    ),
                    flush=True,
                )
            if args.max_frames and stats["saved_frames"] >= args.max_frames:
                break
    except KeyboardInterrupt:
        interrupted = True
    except (TimeoutError, OSError, ValueError, RuntimeError, KeyError) as error:
        failure = str(error)
        exit_code = 1
    finally:
        if client is not None:
            command_path = client.directory / "command.json"
            command_after = command_fingerprint(command_path)
            try:
                client.close(send_stop=False)
            except OSError as error:
                failure = failure or str(error)
                exit_code = 1
        writer.close()

    command_unchanged = command_before == command_after
    if not command_unchanged:
        failure = failure or "command.json changed while read-only exporter held the channel"
        exit_code = 1
    actual_fps = 0.0
    if stats["saved_frames"] > 1 and first_saved_sim is not None and last_saved_sim is not None:
        span = last_saved_sim - first_saved_sim
        if span > 0:
            actual_fps = (stats["saved_frames"] - 1) / span
    for field in (
        "requested_frames",
        "received_frames",
        "saved_frames",
        "rejected_frames",
        "duplicate_frames",
        "armor_frames",
        "empty_frames",
        "armor_instances",
        "cadence_skipped_frames",
        "excluded_empty_frames",
        "rejected_annotation_frames",
    ):
        stats[field] += 0
    summary = {
        **dict(stats),
        "actual_sim_sampling_fps": actual_fps,
        "first_saved_sim_time": first_saved_sim,
        "last_saved_sim_time": last_saved_sim,
        "wall_seconds": time.monotonic() - wall_started,
        "distributions": {name: dict(counter) for name, counter in distributions.items()},
        "distance_distribution": "unavailable_in_frame_protocol",
        "interrupted": interrupted,
        "failure": failure,
        "command_json_unchanged": command_unchanged,
        "command_json_present_before": command_before is not None,
        "command_json_present_after": command_after is not None,
        "command_json_created_by_exporter": command_before is None and command_after is not None,
    }
    atomic_json(output / "reports" / "export_summary.json", summary)

    camera_metadata = None
    publisher = session = None
    resolution = None
    if first_frame is not None:
        camera = first_frame["camera"]
        camera_metadata = {
            "K": camera.get("K"),
            "distortion": camera.get("distortion"),
            "R_camera2gimbal": camera.get("R_camera2gimbal"),
            "t_camera2gimbal": camera.get("t_camera2gimbal"),
        }
        publisher = first_frame.get("publisher_id")
        session = first_frame.get("session_id")
        resolution = [int(camera["width"]), int(camera["height"])]
    repository_git = git_metadata(REPOSITORY_ROOT)
    manifest = {
        "schema_version": 1,
        "created_at": created_at,
        "completed_at": utc_now(),
        "status": "failed" if failure else ("interrupted" if interrupted else "complete"),
        "episode": args.episode_id,
        "episodes": [
            {
                "episode_id": args.episode_id,
                "publisher_id": publisher,
                "session_id": session,
                "random_seed": args.random_seed,
                "trajectory_id": args.trajectory_id or args.episode_id,
                "frames": stats["saved_frames"],
                "instances": stats["armor_instances"],
            }
        ],
        "simulator_git": repository_git,
        "exporter_git": repository_git,
        "godot_version": godot_version(),
        "channel": args.channel,
        "publisher_id": publisher,
        "session_id": session,
        "image_encoding": sorted(encodings),
        "resolution": resolution,
        "dataset_fps": args.fps,
        "cli": {key: str(value) if isinstance(value, Path) else value for key, value in vars(args).items()},
        "camera": camera_metadata,
        "scene": scene,
        "random_seed": args.random_seed,
        "classes": "classes.json",
        "canonical_annotations": "annotations/canonical.jsonl",
        "totals": {"frames": stats["saved_frames"], "instances": stats["armor_instances"]},
        "statistics": summary,
        "visibility_limitations": (
            "camera.armors is a lit/front-facing/all-four-keypoints-in-frame geometric projection. "
            "It has no pixel-depth/raycast occlusion test; occlusion and difficult remain unknown."
        ),
        "resolution_policy": (
            "Exporter v1 requires 1440x1080 for this dataset contract; the IPC protocol itself "
            "supports other resolutions."
        ),
        "command_policy": "read-only; exporter does not write command.json",
    }
    atomic_json(output / "dataset.json", manifest)
    print(compact_json({"type": "dataset_export_result", "output": str(output), **summary}), flush=True)
    if failure:
        print(f"dataset exporter stopped: {failure}", file=sys.stderr)
    return exit_code


if __name__ == "__main__":
    raise SystemExit(main())
