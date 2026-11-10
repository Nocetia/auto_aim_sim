#!/usr/bin/env python3
"""Render separate GT overlays without modifying original training images."""

from __future__ import annotations

import argparse
import hashlib
import json
from pathlib import Path
import random
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from tools.dataset_export.preview_render import write_preview  # noqa: E402
from tools.dataset_export.validate_dataset import load_jsonl, safe_dataset_path  # noqa: E402


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--count", type=int, default=50)
    parser.add_argument("--seed", type=int, default=0)
    args = parser.parse_args()
    if args.count < 0:
        parser.error("--count must be nonnegative")
    return args


def main() -> int:
    args = parse_arguments()
    root = args.dataset.expanduser().resolve()
    records, errors = load_jsonl(root / "annotations" / "canonical.jsonl")
    if errors:
        print(f"preview_dataset: {errors[0]}", file=sys.stderr)
        return 1
    if not records:
        print("preview_dataset: dataset contains no frames", file=sys.stderr)
        return 1
    selected = list(records)
    random.Random(args.seed).shuffle(selected)
    selected = selected[: min(args.count, len(selected))]
    outputs: list[str] = []
    try:
        for record in selected:
            image_path, error = safe_dataset_path(root, record.get("image"))
            if error or image_path is None:
                raise ValueError(f"unsafe image path: {error}")
            original_digest = hashlib.sha256(image_path.read_bytes()).hexdigest()
            episode = record["episode_id"]
            output = (
                root
                / "episodes"
                / episode
                / "previews"
                / f"manual_frame_{int(record['frame_index']):06d}_overlay.png"
            )
            output.parent.mkdir(parents=True, exist_ok=True)
            write_preview(image_path, record, output)
            if hashlib.sha256(image_path.read_bytes()).hexdigest() != original_digest:
                raise RuntimeError("original training image changed while rendering preview")
            outputs.append(str(output.relative_to(root)))
    except (OSError, ValueError, KeyError, RuntimeError) as error:
        print(f"preview_dataset: {error}", file=sys.stderr)
        return 1
    print(json.dumps({"dataset": str(root), "count": len(outputs), "previews": outputs}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
