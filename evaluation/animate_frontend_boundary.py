#!/usr/bin/env python3
"""Render an MP4 of the extracted map, GT trajectory, and projected features."""

from __future__ import annotations

import argparse
import pickle
from pathlib import Path

import cv2
import matplotlib.cm as cm
import numpy as np
from scipy.ndimage import map_coordinates


def load_boundary_map(boundary_map_path: str | Path) -> dict[str, np.ndarray]:
    data = np.load(boundary_map_path)
    return {key: data[key] for key in data.files}


def lookup_bdt(
    points_map: np.ndarray,
    bdt_metres: np.ndarray,
    origin_xy: np.ndarray,
    resolution: float,
) -> tuple[np.ndarray, np.ndarray]:
    cols = (points_map[:, 0] - origin_xy[0]) / resolution
    rows = (points_map[:, 1] - origin_xy[1]) / resolution
    height, width = bdt_metres.shape

    in_bounds = (
        (cols >= 0.0)
        & (cols <= width - 1)
        & (rows >= 0.0)
        & (rows <= height - 1)
    )

    dists = np.full(points_map.shape[0], np.nan, dtype=np.float32)
    if np.any(in_bounds):
        sampled = map_coordinates(
            bdt_metres,
            [rows[in_bounds], cols[in_bounds]],
            order=1,
            mode="nearest",
        )
        dists[in_bounds] = sampled.astype(np.float32)
    return dists, in_bounds


def make_world_transform(
    all_points: np.ndarray, image_width: int, image_height: int, margin_px: int = 40
):
    x_min, y_min = np.min(all_points, axis=0)
    x_max, y_max = np.max(all_points, axis=0)

    x_span = max(float(x_max - x_min), 1.0)
    y_span = max(float(y_max - y_min), 1.0)

    usable_w = image_width - 2 * margin_px
    usable_h = image_height - 2 * margin_px
    scale = min(usable_w / x_span, usable_h / y_span)

    x_offset = margin_px + (usable_w - x_span * scale) / 2.0
    y_offset = margin_px + (usable_h - y_span * scale) / 2.0

    def world_to_pixel(points: np.ndarray) -> np.ndarray:
        pts = np.asarray(points, dtype=np.float32)
        px = x_offset + (pts[:, 0] - x_min) * scale
        py = image_height - (y_offset + (pts[:, 1] - y_min) * scale)
        return np.column_stack((px, py)).astype(np.int32)

    return world_to_pixel


def draw_polyline(image: np.ndarray, points_px: np.ndarray, color, thickness: int):
    if len(points_px) >= 2:
        cv2.polylines(
            image, [points_px.reshape(-1, 1, 2)], False, color, thickness, cv2.LINE_AA
        )


def draw_segments(
    image: np.ndarray, segments_px: np.ndarray, color=(40, 40, 40), thickness: int = 1
):
    for segment in segments_px:
        p1, p2 = segment
        cv2.line(
            image,
            tuple(int(v) for v in p1),
            tuple(int(v) for v in p2),
            color,
            thickness,
            cv2.LINE_AA,
        )


def draw_points(
    image: np.ndarray,
    points_px: np.ndarray,
    bgr_colors: np.ndarray,
    radius: int = 1,
    alpha: float = 1.0,
):
    if len(points_px) == 0:
        return

    if alpha >= 0.999:
        for pt, color in zip(points_px, bgr_colors, strict=True):
            cv2.circle(
                image,
                tuple(int(v) for v in pt),
                radius,
                tuple(int(c) for c in color),
                thickness=-1,
                lineType=cv2.LINE_AA,
            )
        return

    overlay = image.copy()
    for pt, color in zip(points_px, bgr_colors, strict=True):
        cv2.circle(
            overlay,
            tuple(int(v) for v in pt),
            radius,
            tuple(int(c) for c in color),
            thickness=-1,
            lineType=cv2.LINE_AA,
        )
    cv2.addWeighted(overlay, alpha, image, 1.0 - alpha, 0.0, dst=image)


def make_colorbar(
    width: int,
    height: int,
    max_dist: float,
    cmap_name: str = "RdYlGn_r",
) -> np.ndarray:
    cmap = cm.get_cmap(cmap_name)
    values = np.linspace(0.0, 1.0, height, dtype=np.float32)
    colors = (cmap(values)[:, :3][:, ::-1] * 255.0).astype(np.uint8)
    bar = np.repeat(colors[:, None, :], width, axis=1)
    bar = np.ascontiguousarray(np.flipud(bar))

    canvas = np.full((height + 40, width + 70, 3), 255, dtype=np.uint8)
    canvas[20 : 20 + height, 20 : 20 + width] = bar
    cv2.rectangle(canvas, (20, 20), (20 + width, 20 + height), (0, 0, 0), 1)
    cv2.putText(
        canvas,
        f"{max_dist:.1f} m",
        (20 + width + 8, 28),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 0, 0),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        "0.0 m",
        (20 + width + 8, 20 + height),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 0, 0),
        1,
        cv2.LINE_AA,
    )
    cv2.putText(
        canvas,
        "Boundary dist",
        (0, 15),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.45,
        (0, 0, 0),
        1,
        cv2.LINE_AA,
    )
    return canvas


def choose_video_codec(output_path: Path) -> tuple[int, str]:
    """Choose a writer codec from the file extension."""

    suffix = output_path.suffix.lower()
    if suffix == ".avi":
        return cv2.VideoWriter_fourcc(*"MJPG"), "MJPG"
    if suffix == ".mp4":
        return cv2.VideoWriter_fourcc(*"mp4v"), "mp4v"
    raise ValueError(
        f"Unsupported video extension '{suffix}'. Use .avi for MJPG or .mp4 for mp4v."
    )


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--data", default="evaluation/recorded_raw_cfar.pkl")
    parser.add_argument(
        "--boundary-map", default="evaluation/boundary_map_rectangular.npz"
    )
    parser.add_argument(
        "--output",
        default="evaluation/results_rectangular/frontend_boundary_animation.mp4",
    )
    parser.add_argument("--fps", type=int, default=15)
    parser.add_argument(
        "--frame-step",
        type=int,
        default=6,
        help="Render every Nth frame from the recorded dataset",
    )
    parser.add_argument(
        "--max-points-per-frame",
        type=int,
        default=3000,
        help="Maximum projected points drawn per rendered frame",
    )
    parser.add_argument(
        "--trail-frames",
        type=int,
        default=4,
        help="Number of previous rendered frames to show with fading",
    )
    parser.add_argument("--width", type=int, default=1600)
    parser.add_argument("--height", type=int, default=900)
    parser.add_argument(
        "--scatter-max-dist",
        type=float,
        default=2.0,
        help="Distance corresponding to the top of the point color scale",
    )
    parser.add_argument(
        "--world-offset-x",
        type=float,
        default=0.0,
        help="Global x translation applied to GT/projected points before rendering",
    )
    parser.add_argument(
        "--world-offset-y",
        type=float,
        default=0.0,
        help="Global y translation applied to GT/projected points before rendering",
    )
    args = parser.parse_args()

    output_path = Path(args.output)
    output_path.parent.mkdir(parents=True, exist_ok=True)

    world_offset = np.array([args.world_offset_x, args.world_offset_y], dtype=np.float32)

    with open(args.data, "rb") as handle:
        records = pickle.load(handle)
    boundary_map = load_boundary_map(args.boundary_map)

    pile_centres = boundary_map["pile_centres"]
    segments = boundary_map["segments"]
    bdt_metres = boundary_map["bdt_metres"]
    origin_xy = boundary_map["origin_xy"]
    resolution = float(boundary_map["resolution"])

    trajectory = (
        np.array([record["gt_pose"][:2] for record in records], dtype=np.float32)
        + world_offset
    )
    map_points = pile_centres.astype(np.float32)
    world_to_pixel = make_world_transform(
        np.vstack((trajectory, map_points)), args.width, args.height
    )

    segments_px = np.array(
        [world_to_pixel(segment.astype(np.float32)) for segment in segments],
        dtype=np.int32,
    )
    all_traj_px = world_to_pixel(trajectory)

    background = np.full((args.height, args.width, 3), 255, dtype=np.uint8)
    draw_segments(background, segments_px, color=(80, 80, 80), thickness=1)
    centres_px = world_to_pixel(pile_centres.astype(np.float32))
    for point in centres_px:
        cv2.drawMarker(
            background,
            tuple(int(v) for v in point),
            (0, 0, 255),
            markerType=cv2.MARKER_TILTED_CROSS,
            markerSize=6,
            thickness=1,
            line_type=cv2.LINE_AA,
        )
    draw_polyline(background, all_traj_px, color=(210, 210, 210), thickness=1)

    colorbar = make_colorbar(24, 220, args.scatter_max_dist)
    rng = np.random.default_rng(0)
    cmap = cm.get_cmap("RdYlGn_r")

    fourcc, codec_name = choose_video_codec(output_path)
    writer = cv2.VideoWriter(
        str(output_path), fourcc, float(args.fps), (args.width, args.height)
    )
    if not writer.isOpened():
        raise RuntimeError(f"Failed to open video writer for {output_path}")

    frame_indices = list(range(0, len(records), args.frame_step))
    rendered_cache: list[tuple[np.ndarray, np.ndarray]] = []

    try:
        for render_idx, record_idx in enumerate(frame_indices):
            record = records[record_idx]
            frame = background.copy()

            traj_so_far = all_traj_px[: record_idx + 1]
            draw_polyline(frame, traj_so_far, color=(200, 90, 20), thickness=2)

            rendered_cache = rendered_cache[-max(args.trail_frames - 1, 0) :]
            for history_idx, (hist_points_px, hist_colors) in enumerate(rendered_cache):
                alpha = 0.18 + 0.12 * history_idx
                draw_points(frame, hist_points_px, hist_colors, radius=1, alpha=alpha)

            points_map = np.asarray(record["points_map"], dtype=np.float32) + world_offset
            if len(points_map) > args.max_points_per_frame:
                selection = rng.choice(
                    len(points_map), size=args.max_points_per_frame, replace=False
                )
                points_map = points_map[selection]

            dists, in_bounds = lookup_bdt(points_map, bdt_metres, origin_xy, resolution)
            points_map = points_map[in_bounds]
            dists = dists[in_bounds]

            points_px = world_to_pixel(points_map)
            clipped = np.clip(dists / args.scatter_max_dist, 0.0, 1.0)
            rgb = (cmap(clipped)[:, :3] * 255.0).astype(np.uint8)
            bgr = rgb[:, ::-1]
            draw_points(frame, points_px, bgr, radius=1, alpha=0.95)

            rendered_cache.append((points_px.copy(), bgr.copy()))

            pose = np.asarray(record["gt_pose"], dtype=np.float32).copy()
            pose[:2] += world_offset
            pose_px = world_to_pixel(pose[:2].reshape(1, 2))[0]
            heading_len = 20
            heading_px = np.array(
                [
                    pose_px[0] + int(np.cos(pose[2]) * heading_len),
                    pose_px[1] - int(np.sin(pose[2]) * heading_len),
                ],
                dtype=np.int32,
            )
            cv2.arrowedLine(
                frame,
                tuple(int(v) for v in pose_px),
                tuple(int(v) for v in heading_px),
                (30, 30, 30),
                2,
                cv2.LINE_AA,
                tipLength=0.35,
            )
            cv2.circle(
                frame,
                tuple(int(v) for v in pose_px),
                4,
                (255, 120, 0),
                thickness=-1,
                lineType=cv2.LINE_AA,
            )

            frame[20 : 20 + colorbar.shape[0], args.width - 20 - colorbar.shape[1] : args.width - 20] = colorbar

            cv2.putText(
                frame,
                "Extracted map + GT trajectory + projected raw CFAR points",
                (30, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.85,
                (20, 20, 20),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                f"time={record['stamp']:.2f}s  frame={record_idx + 1}/{len(records)}  rendered={render_idx + 1}/{len(frame_indices)}",
                (30, 68),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                (20, 20, 20),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                frame,
                f"world offset=({world_offset[0]:.2f}, {world_offset[1]:.2f}) m",
                (30, 128),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (20, 20, 20),
                2,
                cv2.LINE_AA,
            )
            if len(dists) > 0:
                cv2.putText(
                    frame,
                    f"current points={len(dists)}  median BDT={float(np.median(dists)):.2f} m",
                    (30, 158),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (20, 20, 20),
                    2,
                    cv2.LINE_AA,
                )

            writer.write(frame)
            if (render_idx + 1) % 50 == 0:
                print(f"Rendered {render_idx + 1}/{len(frame_indices)} frames")
    finally:
        writer.release()

    print(f"Saved animation to {output_path} using codec {codec_name}")


if __name__ == "__main__":
    main()
