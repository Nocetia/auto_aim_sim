from __future__ import annotations

import copy
import hashlib
import json
import math
from pathlib import Path
import tempfile
import unittest

from tools.dataset_export.annotations import (
    AnnotationError,
    canonical_record,
    validate_corners,
)
from tools.dataset_export.convert_coco_keypoints import convert_records
from tools.dataset_export.dataset_protocol import (
    ContinuityError,
    FrameProtocolError,
    StreamContinuity,
    frame_context,
    validate_frame_metadata,
)
from tools.dataset_export.export_dataset import DatasetWriter
from tools.dataset_export.png_codec import PngError, RawImageError, raw_to_rgb, read_png, write_png
from tools.dataset_export.preview_render import write_preview
from tools.dataset_export.split_dataset import create_split
from tools.dataset_export.validate_dataset import validate_dataset


def make_frame(
    *,
    request_id: str = "test-1",
    publisher_id: str = "publisher-1",
    session_id: str = "session-1",
    tick: int = 100,
    sim_time: float = 1.0,
    encoding: str = "rgb8",
    width: int = 1440,
    height: int = 1080,
    corners: list[list[float]] | None = None,
) -> dict:
    if corners is None:
        corners = [[600.0, 450.0], [800.0, 450.0], [800.0, 600.0], [600.0, 600.0]]
    channels = 3 if encoding == "rgb8" else 4
    camera = {
        "width": width,
        "height": height,
        "timestamp": sim_time,
        "tick": tick,
        "brightness_gain": 0.4,
        "K": [1303.0, 0.0, width / 2, 0.0, 1303.0, height / 2, 0.0, 0.0, 1.0],
        "distortion": [0.0] * 5,
        "R_camera2gimbal": [0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0],
        "t_camera2gimbal": [0.08, 0.0, 0.04],
        "origin": [0.08, 0.0, 0.69],
        "yaw": 0.0,
        "pitch": 0.0,
        "armors": [{"id": 2, "corners": corners}],
    }
    return {
        "protocol": "rm_aim_sim/2",
        "type": "frame",
        "publisher_id": publisher_id,
        "session_id": session_id,
        "request_id": request_id,
        "frame_file": f"frame-{request_id}.raw",
        "encoding": encoding,
        "image_bytes": width * height * channels,
        "sim_time": sim_time,
        "tick": tick,
        "paused": False,
        "appearance": {
            "robot": "infantry_3",
            "team": "blue",
            "armor_model": "",
            "body_model": "",
            "armor_scale": 1.0,
            "body_scale": 1.0,
        },
        "gimbal": {
            "origin": [0.0, 0.0, 0.65],
            "yaw": 0.0,
            "pitch": 0.0,
            "yaw_vel": 0.0,
            "pitch_vel": 0.0,
            "quaternion_wxyz": [1.0, 0.0, 0.0, 0.0],
        },
        "weapon": {
            "speed": 23.0,
            "rate": 10.0,
            "delay": 0.015,
            "barrel_length": 0.22,
            "gravity": 9.7833,
            "damage": 20,
        },
        "health": {"current": 350, "maximum": 350, "alive": True, "death_time": -1.0},
        "stats": {},
        "ipc": {"enabled": True, "last_seq": -1},
        "camera": camera,
        "capture_timing": {"render_ms": 1.0, "readback_ms": 1.0, "publish_ms": 1.0},
    }


def state_for_frame(frame: dict, *, publication: int = 1) -> dict:
    state = copy.deepcopy(frame)
    state["type"] = "state"
    state.pop("request_id", None)
    state.pop("frame_file", None)
    state.pop("encoding", None)
    state.pop("image_bytes", None)
    state.pop("capture_timing", None)
    state["publication"] = publication
    state["wall_unix_sec"] = 1.0
    state["ipc"] = {"enabled": True, "last_seq": -1, "received": 0, "rejected": 0}
    return state


class ImageCodecTests(unittest.TestCase):
    def test_rgb_raw_to_png_is_lossless(self) -> None:
        raw = bytes([255, 0, 0, 0, 128, 255])
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "rgb.png"
            rgb = raw_to_rgb(raw, 2, 1, "RGB8")
            write_png(output, 2, 1, rgb)
            self.assertEqual(read_png(output), (2, 1, raw))

    def test_rgba_raw_to_rgb_png_drops_only_alpha(self) -> None:
        raw = bytes([10, 20, 30, 0, 40, 50, 60, 255])
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "rgba.png"
            rgb = raw_to_rgb(raw, 2, 1, "rgba8")
            self.assertEqual(rgb, bytes([10, 20, 30, 40, 50, 60]))
            write_png(output, 2, 1, rgb)
            self.assertEqual(read_png(output)[2], rgb)

    def test_bad_raw_length_is_rejected(self) -> None:
        with self.assertRaises(RawImageError):
            raw_to_rgb(b"\x00" * 5, 2, 1, "rgb8")

    def test_invalid_encoding_is_rejected(self) -> None:
        with self.assertRaises(RawImageError):
            raw_to_rgb(b"\x00" * 6, 2, 1, "bgr8")

    def test_corrupt_png_is_rejected(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "image.png"
            write_png(output, 1, 1, bytes([1, 2, 3]))
            damaged = bytearray(output.read_bytes())
            damaged[-5] ^= 0x40
            output.write_bytes(damaged)
            with self.assertRaises(PngError):
                read_png(output)


class AnnotationTests(unittest.TestCase):
    def test_tl_tr_br_bl_order_is_preserved(self) -> None:
        corners = [[10.1, 20.2], [80.3, 21.4], [75.5, 60.6], [12.7, 59.8]]
        record = canonical_record(make_frame(corners=corners), "image.png", "ep", 0)
        self.assertEqual(record["armors"][0]["corners_tl_tr_br_bl"], corners)

    def test_bbox_is_corner_min_max(self) -> None:
        corners = [[12.0, 10.0], [90.0, 14.0], [85.0, 70.0], [8.0, 66.0]]
        geometry = validate_corners(corners, 100, 80)
        self.assertEqual(geometry["bbox_xyxy"], [8.0, 10.0, 90.0, 70.0])
        self.assertGreater(geometry["polygon_area_px2"], 0)

    def test_partial_out_of_frame_is_retained_and_marked_truncated(self) -> None:
        corners = [[-5.0, 10.0], [40.0, 10.0], [40.0, 50.0], [-5.0, 50.0]]
        record = canonical_record(make_frame(corners=corners), "image.png", "ep", 0)
        armor = record["armors"][0]
        self.assertEqual(armor["corners_tl_tr_br_bl"], corners)
        self.assertTrue(armor["truncated"])
        self.assertEqual(armor["bbox_clipped_xyxy"], [0.0, 10.0, 40.0, 50.0])

    def test_nan_and_infinity_are_rejected(self) -> None:
        for invalid in (math.nan, math.inf, -math.inf):
            corners = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [0.0, invalid]]
            with self.assertRaises(AnnotationError):
                validate_corners(corners, 100, 100)

    def test_reversed_or_self_intersecting_order_is_rejected(self) -> None:
        with self.assertRaises(AnnotationError):
            validate_corners([[0, 0], [0, 10], [10, 10], [10, 0]], 100, 100)
        with self.assertRaises(AnnotationError):
            validate_corners([[0, 0], [10, 10], [10, 0], [0, 10]], 100, 100)


class ProtocolTests(unittest.TestCase):
    def test_metadata_contract_and_image_bytes(self) -> None:
        frame = make_frame(width=160, height=120)
        state = state_for_frame(frame)
        self.assertEqual(
            validate_frame_metadata(frame, frame["request_id"], frame_context(state), state),
            (160, 120, 3, "rgb8", 160 * 120 * 3),
        )
        frame["image_bytes"] -= 1
        with self.assertRaises(FrameProtocolError):
            validate_frame_metadata(frame, frame["request_id"], frame_context(state), state)

    def test_duplicate_request_and_tick_are_rejected(self) -> None:
        continuity = StreamContinuity()
        first = make_frame()
        self.assertIsNone(continuity.observe(first))
        self.assertEqual(continuity.observe(first), "duplicate_request")
        second = make_frame(request_id="test-2")
        self.assertEqual(continuity.observe(second), "duplicate_tick")

    def test_session_change_stops_episode(self) -> None:
        continuity = StreamContinuity()
        continuity.observe(make_frame())
        changed = make_frame(request_id="test-2", session_id="session-2", tick=101, sim_time=1.1)
        with self.assertRaises(ContinuityError):
            continuity.observe(changed)

    def test_sim_time_rollback_stops_episode(self) -> None:
        continuity = StreamContinuity()
        continuity.observe(make_frame(sim_time=2.0))
        rollback = make_frame(request_id="test-2", tick=101, sim_time=1.0)
        with self.assertRaises(ContinuityError):
            continuity.observe(rollback)


class DatasetToolTests(unittest.TestCase):
    def _write_small_valid_dataset(self, root: Path) -> dict:
        frame = make_frame()
        raw = bytes([12, 34, 56]) * (1440 * 1080)
        writer = DatasetWriter(
            root,
            "episode_001",
            preview_every=0,
            minimum_area=1.0,
            scene={"trajectory_id": "t1", "random_seed": 7},
        )
        try:
            record, _ = writer.write(frame, raw, 0)
        finally:
            writer.close()
        (root / "classes.json").write_text("{}\n", encoding="utf-8")
        (root / "dataset.json").write_text(
            json.dumps(
                {
                    "schema_version": 1,
                    "episodes": [
                        {
                            "episode_id": "episode_001",
                            "random_seed": 7,
                            "trajectory_id": "t1",
                        }
                    ],
                    "totals": {"frames": 1, "instances": 1},
                }
            ),
            encoding="utf-8",
        )
        return record

    def test_preview_does_not_modify_original_image(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            record = self._write_small_valid_dataset(root)
            image = root / record["image"]
            before = hashlib.sha256(image.read_bytes()).hexdigest()
            preview = root / "preview.png"
            write_preview(image, record, preview)
            self.assertEqual(hashlib.sha256(image.read_bytes()).hexdigest(), before)
            self.assertNotEqual(hashlib.sha256(preview.read_bytes()).hexdigest(), before)

    def test_validator_accepts_complete_dataset(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            self._write_small_valid_dataset(root)
            result = validate_dataset(root)
            self.assertTrue(result["valid"], result["errors"])
            self.assertEqual(result["statistics"]["frames"], 1)

    def test_episode_split_has_no_leakage(self) -> None:
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            (root / "annotations").mkdir()
            episodes = [f"episode_{index}" for index in range(6)]
            records = []
            for episode_index, episode in enumerate(episodes):
                for frame_index in range(2):
                    records.append(
                        {
                            "episode_id": episode,
                            "frame_index": frame_index,
                            "image": f"episodes/{episode}/images/frame_{frame_index:06d}.png",
                            "scene": {"trajectory_id": f"trajectory_{episode_index // 2}"},
                        }
                    )
            (root / "annotations" / "canonical.jsonl").write_text(
                "".join(json.dumps(record) + "\n" for record in records), encoding="utf-8"
            )
            (root / "dataset.json").write_text(
                json.dumps(
                    {
                        "episodes": [
                            {"episode_id": episode, "trajectory_id": f"trajectory_{index // 2}"}
                            for index, episode in enumerate(episodes)
                        ]
                    }
                ),
                encoding="utf-8",
            )
            split = create_split(
                root,
                seed=123,
                ratios={"train": 0.7, "val": 0.15, "test": 0.15},
                group_by="trajectory",
            )
            ownership = {}
            for split_name, payload in split["splits"].items():
                for episode in payload["episodes"]:
                    self.assertNotIn(episode, ownership)
                    ownership[episode] = split_name
            self.assertEqual(set(ownership), set(episodes))
            for pair_start in range(0, 6, 2):
                self.assertEqual(
                    ownership[episodes[pair_start]], ownership[episodes[pair_start + 1]]
                )

    def test_canonical_to_coco_preserves_corner_and_class_contract(self) -> None:
        record = canonical_record(make_frame(), "image.png", "ep", 0)
        converted = convert_records([record])
        annotation = converted["annotations"][0]
        expected = [
            coordinate
            for point in record["armors"][0]["corners_tl_tr_br_bl"]
            for coordinate in (point[0], point[1], 1)
        ]
        self.assertEqual(annotation["keypoints"], expected)
        self.assertEqual(annotation["attributes"]["robot"], "infantry_3")
        self.assertFalse(converted["info"]["current_auto_aim_training_compatible"])


if __name__ == "__main__":
    unittest.main()
