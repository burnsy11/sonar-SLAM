#!/usr/bin/env python3
"""Extract a rectangular pile boundary map from the simulator occupancy grid."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from scipy.spatial.distance import pdist


REPO_ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO_ROOT / "bruce_slam" / "src"))

from bruce_slam.map_processing import process_map  # noqa: E402


def write_preview_image(output_path: Path, pile_map) -> None:
    """Write a simple preview image with occupancy, boundaries, and centres."""

    base = np.where(pile_map.binary > 0, 0, 255).astype(np.uint8)
    preview = cv2.cvtColor(base, cv2.COLOR_GRAY2BGR)
    preview[pile_map.boundary_mask > 0] = (0, 0, 255)

    for x, y in pile_map.pile_centres:
        col = int(round((x - pile_map.origin_xy[0]) / pile_map.resolution))
        row = int(round((y - pile_map.origin_xy[1]) / pile_map.resolution))
        cv2.circle(preview, (col, row), 1, (0, 255, 0), thickness=-1)

    preview = cv2.flip(preview, 0)
    cv2.imwrite(str(output_path), preview)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--map",
        default=str(REPO_ROOT / "gt_occupancy_grid.yaml"),
        help="Path to map_server YAML for the prior occupancy grid",
    )
    parser.add_argument(
        "--output",
        default=str(REPO_ROOT / "evaluation" / "boundary_map_rectangular.npz"),
        help="Path to the output .npz archive",
    )
    parser.add_argument(
        "--summary",
        default=str(REPO_ROOT / "evaluation" / "boundary_map_rectangular_summary.json"),
        help="Path to the output JSON summary",
    )
    parser.add_argument(
        "--preview",
        default=str(REPO_ROOT / "evaluation" / "boundary_map_rectangular_preview.png"),
        help="Path to a preview PNG of the extracted map",
    )
    parser.add_argument("--pile-width", type=float, default=None)
    parser.add_argument("--pile-height", type=float, default=None)
    parser.add_argument("--dt-max-dist", type=float, default=5.0)
    args = parser.parse_args()

    output_path = Path(args.output)
    summary_path = Path(args.summary)
    preview_path = Path(args.preview)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    pile_map = process_map(
        args.map,
        pile_width=args.pile_width,
        pile_height=args.pile_height,
        dt_max_dist=args.dt_max_dist,
    )

    np.savez_compressed(
        output_path,
        pile_centres=pile_map.pile_centres,
        pile_width=np.array(pile_map.pile_width, dtype=np.float64),
        pile_height=np.array(pile_map.pile_height, dtype=np.float64),
        segments=pile_map.segments,
        bdt_metres=pile_map.bdt_metres,
        resolution=np.array(pile_map.resolution, dtype=np.float64),
        origin_xy=pile_map.origin_xy,
        binary=pile_map.binary,
        boundary_mask=pile_map.boundary_mask,
    )

    pairwise_distances = pdist(pile_map.pile_centres)
    summary = {
        "map_yaml": str(Path(args.map).resolve()),
        "n_piles": int(len(pile_map.pile_centres)),
        "pile_width": float(pile_map.pile_width),
        "pile_height": float(pile_map.pile_height),
        "resolution": float(pile_map.resolution),
        "origin_xy": [float(value) for value in pile_map.origin_xy],
        "bdt_shape": [int(v) for v in pile_map.bdt_metres.shape],
        "bdt_max_dist": float(args.dt_max_dist),
        "boundary_pixels": int(np.count_nonzero(pile_map.boundary_mask)),
        "min_pile_spacing": float(np.min(pairwise_distances))
        if len(pile_map.pile_centres) > 1
        else None,
        "x_range": [
            float(np.min(pile_map.pile_centres[:, 0])),
            float(np.max(pile_map.pile_centres[:, 0])),
        ],
        "y_range": [
            float(np.min(pile_map.pile_centres[:, 1])),
            float(np.max(pile_map.pile_centres[:, 1])),
        ],
    }
    summary_path.write_text(json.dumps(summary, indent=2), encoding="utf-8")

    write_preview_image(preview_path, pile_map)

    print(f"Saved boundary map to {output_path}")
    print(f"Saved summary to {summary_path}")
    print(f"Saved preview to {preview_path}")
    print(
        "Extracted "
        f"{summary['n_piles']} piles, width={summary['pile_width']:.3f} m, "
        f"height={summary['pile_height']:.3f} m"
    )


if __name__ == "__main__":
    main()
