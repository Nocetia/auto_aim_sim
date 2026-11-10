#!/usr/bin/env python3
"""Convert canonical JSONL to generic COCO Keypoints JSON (offline only)."""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import sys
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from tools.dataset_export.validate_dataset import load_jsonl  # noqa: E402


def convert_records(records: list[dict[str, Any]], *, assume_visible: bool = False) -> dict[str, Any]:
    """Convert records while retaining canonical class fields as attributes.

    COCO visibility defaults to 1 (labeled, visibility not asserted) because
    camera.armors has no pixel-occlusion test.  --assume-visible opts into 2.
    """

    images: list[dict[str, Any]] = []
    annotations: list[dict[str, Any]] = []
    annotation_id = 1
    visibility = 2 if assume_visible else 1
    for image_id, record in enumerate(records, 1):
        images.append(
            {
                "id": image_id,
                "file_name": record["image"],
                "width": record["width"],
                "height": record["height"],
                "episode_id": record["episode_id"],
                "frame_index": record["frame_index"],
                "request_id": record["request_id"],
                "tick": record["tick"],
                "sim_time": record["sim_time"],
            }
        )
        for armor in record.get("armors", []):
            x0, y0, x1, y1 = armor["bbox_clipped_xyxy"]
            keypoints = [
                coordinate
                for point in armor["corners_tl_tr_br_bl"]
                for coordinate in (point[0], point[1], visibility)
            ]
            annotations.append(
                {
                    "id": annotation_id,
                    "image_id": image_id,
                    "category_id": 1,
                    "bbox": [x0, y0, x1 - x0, y1 - y0],
                    "area": armor["polygon_area_px2"],
                    "iscrowd": 0,
                    "keypoints": keypoints,
                    "num_keypoints": 4,
                    "segmentation": [
                        [coordinate for point in armor["corners_tl_tr_br_bl"] for coordinate in point]
                    ],
                    "attributes": {
                        "team": armor["team"],
                        "robot": armor["robot"],
                        "robot_number": armor.get("robot_number"),
                        "armor_type": armor["armor_type"],
                        "armor_id": armor["armor_id"],
                        "truncated": armor["truncated"],
                        "occlusion": armor.get("occlusion"),
                        "canonical_corner_order": "TL_TR_BR_BL",
                    },
                }
            )
            annotation_id += 1
    return {
        "info": {
            "description": "rm_aim_sim canonical export converted to generic COCO Keypoints",
            "version": "1",
            "date_created": datetime.now(timezone.utc).isoformat().replace("+00:00", "Z"),
            "current_auto_aim_training_compatible": False,
            "compatibility_note": (
                "The deployed custom YOLOv5 has separate color/number heads and no training "
                "reader or export contract in the audited checkout."
            ),
            "keypoint_visibility": (
                "2=explicitly assumed visible" if assume_visible else "1=labeled; pixel visibility not asserted"
            ),
        },
        "licenses": [],
        "images": images,
        "annotations": annotations,
        "categories": [
            {
                "id": 1,
                "name": "armor",
                "supercategory": "robomaster",
                "keypoints": ["TL", "TR", "BR", "BL"],
                "skeleton": [[1, 2], [2, 3], [3, 4], [4, 1]],
            }
        ],
    }


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument(
        "--output", type=Path, help="default: DATASET/annotations/coco_keypoints.json"
    )
    parser.add_argument(
        "--assume-visible",
        action="store_true",
        help="set COCO v=2; default v=1 because source has no pixel occlusion test",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    dataset = args.dataset.expanduser().resolve()
    records, errors = load_jsonl(dataset / "annotations" / "canonical.jsonl")
    if errors:
        print(f"convert_coco_keypoints: {errors[0]}", file=sys.stderr)
        return 1
    if not records:
        print("convert_coco_keypoints: canonical dataset is empty", file=sys.stderr)
        return 1
    output = (args.output or dataset / "annotations" / "coco_keypoints.json").expanduser().resolve()
    try:
        converted = convert_records(records, assume_visible=args.assume_visible)
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_name(output.name + ".tmp")
        temporary.write_text(
            json.dumps(converted, ensure_ascii=False, allow_nan=False, indent=2) + "\n",
            encoding="utf-8",
        )
        temporary.replace(output)
    except (OSError, ValueError, KeyError) as error:
        print(f"convert_coco_keypoints: {error}", file=sys.stderr)
        return 1
    print(
        json.dumps(
            {
                "output": str(output),
                "images": len(converted["images"]),
                "annotations": len(converted["annotations"]),
            }
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
