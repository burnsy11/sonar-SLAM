#!/usr/bin/env python3
"""Estimate a global XY translation aligning projected features to the boundary map."""

from __future__ import annotations

import argparse
import json
import pickle
from pathlib import Path

import numpy as np
from scipy.ndimage import map_coordinates


def score_translation(points: np.ndarray, bdt: np.ndarray, origin: np.ndarray, resolution: float, dx: float, dy: float) -> tuple[float, float, int]:
    shifted = points + np.array([dx, dy], dtype=np.float32)
    cols = (shifted[:, 0] - origin[0]) / resolution
    rows = (shifted[:, 1] - origin[1]) / resolution
    height, width = bdt.shape
    mask = (cols >= 0.0) & (cols <= width - 1) & (rows >= 0.0) & (rows <= height - 1)
    if not np.any(mask):
        return float("inf"), float("inf"), 0
    values = map_coordinates(bdt, [rows[mask], cols[mask]], order=1, mode="nearest")
    return float(np.median(values)), float(np.mean(values)), int(np.count_nonzero(mask))


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default="evaluation/recorded_raw_cfar.pkl")
    parser.add_argument("--boundary-map", default="evaluation/boundary_map_rectangular.npz")
    parser.add_argument("--output", default="evaluation/results_rectangular/alignment_estimate.json")
    parser.add_argument("--frame-step", type=int, default=10)
    parser.add_argument("--max-points-per-frame", type=int, default=300)
    parser.add_argument("--dx-min", type=float, default=-1.0)
    parser.add_argument("--dx-max", type=float, default=1.0)
    parser.add_argument("--dy-min", type=float, default=-2.0)
    parser.add_argument("--dy-max", type=float, default=2.0)
    parser.add_argument("--coarse-step", type=float, default=0.25)
    parser.add_argument("--fine-step", type=float, default=0.05)
    parser.add_argument("--fine-radius", type=float, default=0.3)
    args = parser.parse_args()

    with open(args.data, "rb") as handle:
        records = pickle.load(handle)
    boundary_map = np.load(args.boundary_map)
    bdt = boundary_map["bdt_metres"]
    origin = boundary_map["origin_xy"]
    resolution = float(boundary_map["resolution"])

    rng = np.random.default_rng(0)
    sampled_points = []
    for record in records[:: args.frame_step]:
        points = np.asarray(record["points_map"], dtype=np.float32)
        if len(points) == 0:
            continue
        if len(points) > args.max_points_per_frame:
            idx = rng.choice(len(points), size=args.max_points_per_frame, replace=False)
            points = points[idx]
        sampled_points.append(points)
    sampled_points = np.concatenate(sampled_points, axis=0)

    best = None
    for dy in np.arange(args.dy_min, args.dy_max + 1e-6, args.coarse_step):
        for dx in np.arange(args.dx_min, args.dx_max + 1e-6, args.coarse_step):
            median_dist, mean_dist, n = score_translation(
                sampled_points, bdt, origin, resolution, dx, dy
            )
            candidate = (median_dist, mean_dist, dx, dy, n)
            if best is None or candidate < best:
                best = candidate

    coarse_best = best
    _, _, best_dx, best_dy, _ = coarse_best
    best = None
    for dy in np.arange(best_dy - args.fine_radius, best_dy + args.fine_radius + 1e-6, args.fine_step):
        for dx in np.arange(best_dx - args.fine_radius, best_dx + args.fine_radius + 1e-6, args.fine_step):
            median_dist, mean_dist, n = score_translation(
                sampled_points, bdt, origin, resolution, dx, dy
            )
            candidate = (median_dist, mean_dist, dx, dy, n)
            if best is None or candidate < best:
                best = candidate

    median_dist, mean_dist, dx, dy, in_bounds = best
    result = {
        "data_path": str(Path(args.data).resolve()),
        "boundary_map_path": str(Path(args.boundary_map).resolve()),
        "sampled_points": int(len(sampled_points)),
        "coarse_best": {
            "median_dist": float(coarse_best[0]),
            "mean_dist": float(coarse_best[1]),
            "dx": float(coarse_best[2]),
            "dy": float(coarse_best[3]),
            "in_bounds": int(coarse_best[4]),
        },
        "best_translation_xy": [float(dx), float(dy)],
        "best_median_dist": float(median_dist),
        "best_mean_dist": float(mean_dist),
        "best_in_bounds_points": int(in_bounds),
    }

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text(json.dumps(result, indent=2), encoding="utf-8")
    print(json.dumps(result, indent=2))


if __name__ == "__main__":
    main()
