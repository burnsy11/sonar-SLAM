#!/usr/bin/env python3
"""Analyse frontend feature compatibility with the extracted boundary map."""

from __future__ import annotations

import argparse
import json
import pickle
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
from scipy.ndimage import map_coordinates


def load_boundary_map(boundary_map_path: str | Path) -> dict[str, np.ndarray]:
    """Load a saved boundary-map archive."""

    data = np.load(boundary_map_path)
    return {key: data[key] for key in data.files}


def lookup_bdt(
    points_map: np.ndarray,
    bdt_metres: np.ndarray,
    origin_xy: np.ndarray,
    resolution: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Look up BDT distances for points in map frame using bilinear sampling."""

    cols = (points_map[:, 0] - origin_xy[0]) / resolution
    rows = (points_map[:, 1] - origin_xy[1]) / resolution
    height, width = bdt_metres.shape

    in_bounds = (
        (cols >= 0.0)
        & (cols <= width - 1)
        & (rows >= 0.0)
        & (rows <= height - 1)
    )

    distances = np.full(points_map.shape[0], np.nan, dtype=np.float32)
    if np.any(in_bounds):
        sampled = map_coordinates(
            bdt_metres,
            [rows[in_bounds], cols[in_bounds]],
            order=1,
            mode="nearest",
        )
        distances[in_bounds] = sampled.astype(np.float32)

    return distances, in_bounds


def reservoir_update(
    sample_points: np.ndarray,
    sample_dists: np.ndarray,
    seen: int,
    points: np.ndarray,
    dists: np.ndarray,
    max_sample_points: int,
    rng: np.random.Generator,
) -> tuple[np.ndarray, np.ndarray, int]:
    """Reservoir-sample points for the spatial scatter plot."""

    valid = np.isfinite(dists)
    points = points[valid]
    dists = dists[valid]

    for point, dist in zip(points, dists, strict=True):
        if seen < max_sample_points:
            sample_points[seen] = point
            sample_dists[seen] = dist
        else:
            index = int(rng.integers(0, seen + 1))
            if index < max_sample_points:
                sample_points[index] = point
                sample_dists[index] = dist
        seen += 1

    return sample_points, sample_dists, seen


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--data",
        default="evaluation/recorded_raw_cfar.pkl",
        help="Path to recorded frontend features paired with GT pose",
    )
    parser.add_argument(
        "--boundary-map",
        default="evaluation/boundary_map_rectangular.npz",
        help="Path to the extracted boundary-map archive",
    )
    parser.add_argument(
        "--output",
        default="evaluation/results_rectangular",
        help="Directory for plots and summary outputs",
    )
    parser.add_argument(
        "--max-scatter-points",
        type=int,
        default=200000,
        help="Maximum number of sampled points to show in the spatial scatter plot",
    )
    parser.add_argument(
        "--hist-max-dist",
        type=float,
        default=3.0,
        help="Max x-range for the histogram plot in metres",
    )
    parser.add_argument(
        "--scatter-max-dist",
        type=float,
        default=2.0,
        help="Max colourbar range for the spatial scatter plot in metres",
    )
    parser.add_argument(
        "--world-offset-x",
        type=float,
        default=0.0,
        help="Global x translation applied to GT/projected points before map lookup",
    )
    parser.add_argument(
        "--world-offset-y",
        type=float,
        default=0.0,
        help="Global y translation applied to GT/projected points before map lookup",
    )
    args = parser.parse_args()

    output_dir = Path(args.output)
    output_dir.mkdir(parents=True, exist_ok=True)

    with open(args.data, "rb") as handle:
        records = pickle.load(handle)
    boundary_map = load_boundary_map(args.boundary_map)

    bdt_metres = boundary_map["bdt_metres"]
    origin_xy = boundary_map["origin_xy"]
    resolution = float(boundary_map["resolution"])
    pile_centres = boundary_map["pile_centres"]
    segments = boundary_map["segments"]

    print(f"Loaded {len(records)} recorded frames from {args.data}")
    print(
        "Loaded boundary map with "
        f"{len(pile_centres)} piles from {args.boundary_map}"
    )

    all_distances = []
    frame_stats = []
    total_points = 0
    total_in_bounds = 0
    total_out_of_bounds = 0

    rng = np.random.default_rng(0)
    sample_points = np.empty((args.max_scatter_points, 2), dtype=np.float32)
    sample_dists = np.empty((args.max_scatter_points,), dtype=np.float32)
    sampled_seen = 0

    world_offset = np.array([args.world_offset_x, args.world_offset_y], dtype=np.float32)

    for index, record in enumerate(records, start=1):
        points_map = np.asarray(record["points_map"], dtype=np.float32) + world_offset
        n_points = int(record.get("n_points", len(points_map)))
        total_points += n_points

        if n_points == 0:
            frame_stats.append(
                {
                    "stamp": float(record["stamp"]),
                    "n_points": 0,
                    "n_in_bounds": 0,
                    "n_out_of_bounds": 0,
                    "median_dist": None,
                    "mean_dist": None,
                    "p90_dist": None,
                    "std_dist": None,
                }
            )
            continue

        distances, in_bounds = lookup_bdt(points_map, bdt_metres, origin_xy, resolution)
        in_bounds_distances = distances[in_bounds]
        n_in_bounds = int(np.count_nonzero(in_bounds))
        n_out_of_bounds = n_points - n_in_bounds

        total_in_bounds += n_in_bounds
        total_out_of_bounds += n_out_of_bounds

        if n_in_bounds > 0:
            all_distances.append(in_bounds_distances.astype(np.float32, copy=False))
            frame_stats.append(
                {
                    "stamp": float(record["stamp"]),
                    "n_points": n_points,
                    "n_in_bounds": n_in_bounds,
                    "n_out_of_bounds": n_out_of_bounds,
                    "median_dist": float(np.median(in_bounds_distances)),
                    "mean_dist": float(np.mean(in_bounds_distances)),
                    "p90_dist": float(np.percentile(in_bounds_distances, 90)),
                    "std_dist": float(np.std(in_bounds_distances)),
                }
            )
            sample_points, sample_dists, sampled_seen = reservoir_update(
                sample_points,
                sample_dists,
                sampled_seen,
                points_map[in_bounds],
                in_bounds_distances,
                args.max_scatter_points,
                rng,
            )
        else:
            frame_stats.append(
                {
                    "stamp": float(record["stamp"]),
                    "n_points": n_points,
                    "n_in_bounds": 0,
                    "n_out_of_bounds": n_out_of_bounds,
                    "median_dist": None,
                    "mean_dist": None,
                    "p90_dist": None,
                    "std_dist": None,
                }
            )

        if index % 500 == 0:
            print(f"Processed {index}/{len(records)} frames")

    if not all_distances:
        raise RuntimeError("No in-bounds feature points were found for analysis")

    all_distances = np.concatenate(all_distances).astype(np.float32, copy=False)
    sample_points = sample_points[: min(sampled_seen, args.max_scatter_points)]
    sample_dists = sample_dists[: min(sampled_seen, args.max_scatter_points)]

    print("\n=== RESULTS ===")
    print(f"Total frames: {len(records)}")
    print(f"Total points: {total_points}")
    print(f"In-bounds points: {total_in_bounds}")
    print(f"Out-of-bounds points: {total_out_of_bounds}")
    print(
        "Boundary distance stats: "
        f"median={np.median(all_distances):.3f} m, "
        f"mean={np.mean(all_distances):.3f} m, "
        f"std={np.std(all_distances):.3f} m, "
        f"p90={np.percentile(all_distances, 90):.3f} m"
    )

    stamps = np.array([entry["stamp"] for entry in frame_stats], dtype=np.float64)
    median_dist = np.array(
        [
            np.nan if entry["median_dist"] is None else entry["median_dist"]
            for entry in frame_stats
        ],
        dtype=np.float64,
    )
    point_count = np.array([entry["n_points"] for entry in frame_stats], dtype=np.int32)
    in_bounds_count = np.array(
        [entry["n_in_bounds"] for entry in frame_stats], dtype=np.int32
    )

    fig, ax = plt.subplots(figsize=(10, 5))
    ax.hist(
        all_distances,
        bins=100,
        range=(0.0, args.hist_max_dist),
        edgecolor="black",
        linewidth=0.2,
        alpha=0.8,
    )
    ax.axvline(
        np.median(all_distances),
        color="red",
        linestyle="--",
        label=f"median={np.median(all_distances):.2f} m",
    )
    ax.set_xlabel("Distance to nearest boundary (m)")
    ax.set_ylabel("Count")
    ax.set_title("Frontend Feature Points: Distance to Nearest Pile Boundary")
    ax.legend()
    fig.savefig(
        output_dir / "boundary_distance_histogram.png",
        dpi=150,
        bbox_inches="tight",
    )
    plt.close(fig)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    ax1.plot(stamps, median_dist, ".", markersize=2)
    ax1.set_ylabel("Median boundary dist (m)")
    ax1.set_title("Per-Frame Median Boundary Distance")
    ax2.plot(stamps, point_count, ".", markersize=2, color="green", label="all points")
    ax2.plot(
        stamps,
        in_bounds_count,
        ".",
        markersize=2,
        color="blue",
        alpha=0.5,
        label="in-bounds points",
    )
    ax2.set_ylabel("Point count")
    ax2.set_xlabel("Time (s)")
    ax2.legend()
    fig.savefig(
        output_dir / "boundary_distance_timeseries.png",
        dpi=150,
        bbox_inches="tight",
    )
    plt.close(fig)

    fig, ax = plt.subplots(figsize=(14, 10))
    for segment in segments:
        p1, p2 = segment
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]], "k-", linewidth=0.4, alpha=0.4)
    ax.plot(pile_centres[:, 0], pile_centres[:, 1], "rx", markersize=3)
    scatter = ax.scatter(
        sample_points[:, 0],
        sample_points[:, 1],
        c=sample_dists,
        cmap="RdYlGn_r",
        s=1,
        vmin=0.0,
        vmax=args.scatter_max_dist,
        alpha=0.3,
    )
    plt.colorbar(scatter, ax=ax, label="Boundary distance (m)")
    ax.set_aspect("equal")
    ax.set_title("Feature Points Coloured by Boundary Distance")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    fig.savefig(
        output_dir / "boundary_distance_spatial.png",
        dpi=150,
        bbox_inches="tight",
    )
    plt.close(fig)

    summary = {
        "data_path": str(Path(args.data).resolve()),
        "boundary_map_path": str(Path(args.boundary_map).resolve()),
        "world_offset_xy": [float(world_offset[0]), float(world_offset[1])],
        "n_frames": int(len(records)),
        "n_points_total": int(total_points),
        "n_points_in_bounds": int(total_in_bounds),
        "n_points_out_of_bounds": int(total_out_of_bounds),
        "boundary_dist_median": float(np.median(all_distances)),
        "boundary_dist_mean": float(np.mean(all_distances)),
        "boundary_dist_std": float(np.std(all_distances)),
        "boundary_dist_p10": float(np.percentile(all_distances, 10)),
        "boundary_dist_p50": float(np.percentile(all_distances, 50)),
        "boundary_dist_p90": float(np.percentile(all_distances, 90)),
        "sampled_scatter_points": int(sample_points.shape[0]),
    }

    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2), encoding="utf-8"
    )
    (output_dir / "frame_stats.json").write_text(
        json.dumps(frame_stats, indent=2), encoding="utf-8"
    )

    print(f"Saved analysis outputs to {output_dir}")


if __name__ == "__main__":
    main()
