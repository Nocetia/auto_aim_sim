#!/usr/bin/env python3
"""Create leakage-safe dataset split manifests using episode-level groups."""

from __future__ import annotations

import argparse
from collections import defaultdict
import json
import math
from pathlib import Path
import random
import sys
from typing import Any


REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
if str(REPOSITORY_ROOT) not in sys.path:
    sys.path.insert(0, str(REPOSITORY_ROOT))

from tools.dataset_export.validate_dataset import load_json, load_jsonl  # noqa: E402


def _episode_metadata(dataset: dict[str, Any]) -> dict[str, dict[str, Any]]:
    result: dict[str, dict[str, Any]] = {}
    episodes = dataset.get("episodes")
    if isinstance(episodes, list):
        for item in episodes:
            if isinstance(item, dict) and isinstance(item.get("episode_id"), str):
                result[item["episode_id"]] = item
    return result


def group_value(
    group_by: str,
    episode: str,
    records: list[dict[str, Any]],
    metadata: dict[str, Any],
) -> str:
    if group_by == "episode":
        return episode
    if group_by == "session":
        values = {(record.get("publisher_id"), record.get("session_id")) for record in records}
        if len(values) != 1:
            raise ValueError(f"episode {episode!r} does not have exactly one publisher/session")
        publisher, session = next(iter(values))
        if not publisher or not session:
            raise ValueError(f"episode {episode!r} has an empty publisher/session")
        return f"{publisher}/{session}"
    if group_by == "seed":
        value = metadata.get("random_seed")
        if value is None and records:
            value = records[0].get("scene", {}).get("random_seed")
        if value is None:
            raise ValueError(f"episode {episode!r} has no random_seed")
        return str(value)
    if group_by == "trajectory":
        value = metadata.get("trajectory_id")
        if value is None and records:
            value = records[0].get("scene", {}).get("trajectory_id")
        if value is None:
            raise ValueError(f"episode {episode!r} has no trajectory_id")
        return str(value)
    raise ValueError(f"unsupported grouping unit: {group_by}")


def create_split(
    dataset_root: Path,
    *,
    seed: int,
    ratios: dict[str, float],
    group_by: str = "episode",
) -> dict[str, Any]:
    dataset_root = dataset_root.expanduser().resolve()
    dataset = load_json(dataset_root / "dataset.json")
    records, errors = load_jsonl(dataset_root / "annotations" / "canonical.jsonl")
    if errors:
        raise ValueError(errors[0])
    if not records:
        raise ValueError("cannot split an empty dataset")
    total_ratio = sum(ratios.values())
    if any(not math.isfinite(value) or value < 0 for value in ratios.values()) or abs(
        total_ratio - 1.0
    ) > 1e-9:
        raise ValueError("split ratios must be finite, nonnegative, and sum to 1")

    by_episode: dict[str, list[dict[str, Any]]] = defaultdict(list)
    for record in records:
        episode = record.get("episode_id")
        if not isinstance(episode, str) or not episode:
            raise ValueError("every canonical record needs episode_id")
        by_episode[episode].append(record)
    metadata = _episode_metadata(dataset)
    grouped: dict[str, list[str]] = defaultdict(list)
    for episode, episode_records in by_episode.items():
        key = group_value(group_by, episode, episode_records, metadata.get(episode, {}))
        grouped[key].append(episode)

    groups = list(grouped.items())
    random.Random(seed).shuffle(groups)
    group_sizes = {
        key: sum(len(by_episode[episode]) for episode in episodes) for key, episodes in groups
    }
    total_frames = len(records)
    targets = {name: ratio * total_frames for name, ratio in ratios.items()}
    assigned_frames = {name: 0 for name in ratios}
    assigned_groups: dict[str, list[tuple[str, list[str]]]] = {name: [] for name in ratios}
    for key, episodes in groups:
        # Choose the split furthest below its target.  Stable dict order breaks
        # ties train -> val -> test; the shuffled group order is seed-controlled.
        split_name = max(ratios, key=lambda name: targets[name] - assigned_frames[name])
        assigned_groups[split_name].append((key, episodes))
        assigned_frames[split_name] += group_sizes[key]

    result_splits: dict[str, Any] = {}
    ownership: dict[str, str] = {}
    for split_name, split_groups in assigned_groups.items():
        episodes = sorted(episode for _, members in split_groups for episode in members)
        split_records = [record for episode in episodes for record in by_episode[episode]]
        for episode in episodes:
            if episode in ownership:
                raise AssertionError(f"episode leakage: {episode}")
            ownership[episode] = split_name
        result_splits[split_name] = {
            "groups": [key for key, _ in split_groups],
            "episodes": episodes,
            "frames": len(split_records),
            "images": [record["image"] for record in split_records],
            "records": [
                {
                    "episode_id": record["episode_id"],
                    "frame_index": record["frame_index"],
                    "image": record["image"],
                }
                for record in split_records
            ],
        }
    if set(ownership) != set(by_episode):
        raise AssertionError("not every episode was assigned exactly once")
    return {
        "schema_version": 1,
        "seed": seed,
        "unit": group_by,
        "ratios": ratios,
        "mode": "manifest_only_no_image_copies",
        "splits": result_splits,
    }


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dataset", required=True, type=Path)
    parser.add_argument("--seed", required=True, type=int, help="explicit deterministic split seed")
    parser.add_argument("--train", type=float, default=0.70)
    parser.add_argument("--val", type=float, default=0.15)
    parser.add_argument("--test", type=float, default=0.15)
    parser.add_argument(
        "--group-by", choices=("episode", "trajectory", "seed", "session"), default="episode"
    )
    parser.add_argument("--output", type=Path, help="default: DATASET/split.json")
    return parser.parse_args()


def main() -> int:
    args = parse_arguments()
    try:
        split = create_split(
            args.dataset,
            seed=args.seed,
            ratios={"train": args.train, "val": args.val, "test": args.test},
            group_by=args.group_by,
        )
        output = args.output or args.dataset / "split.json"
        output = output.expanduser().resolve()
        temporary = output.with_name(output.name + ".tmp")
        temporary.write_text(
            json.dumps(split, ensure_ascii=False, allow_nan=False, indent=2) + "\n", encoding="utf-8"
        )
        temporary.replace(output)
    except (OSError, ValueError, json.JSONDecodeError) as error:
        print(f"split_dataset: {error}", file=sys.stderr)
        return 1
    print(
        json.dumps(
            {
                "output": str(output),
                "splits": {
                    name: {
                        "groups": len(payload["groups"]),
                        "episodes": len(payload["episodes"]),
                        "frames": payload["frames"],
                    }
                    for name, payload in split["splits"].items()
                },
            },
            ensure_ascii=False,
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
