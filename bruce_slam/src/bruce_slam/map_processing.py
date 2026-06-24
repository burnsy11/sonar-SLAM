"""Pile-map processing utilities for offline evaluation and localisation.

This module is intentionally ROS-free. It loads a map_server-style occupancy
grid, extracts repeated rectangular pile primitives, builds their boundary
segments, and computes a boundary distance transform (BDT).
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import scipy.ndimage
import yaml


@dataclass
class PileMap:
    """Processed pile map representation."""

    pile_centres: np.ndarray
    pile_width: float
    pile_height: float
    segments: np.ndarray
    bdt_metres: np.ndarray
    resolution: float
    origin_xy: np.ndarray
    binary: np.ndarray
    boundary_mask: np.ndarray


def load_occupancy(map_yaml_path: str | Path) -> tuple[np.ndarray, float, np.ndarray]:
    """Load a map_server occupancy grid as a binary occupied mask.

    The returned binary image is vertically flipped so row 0 corresponds to the
    minimum world-frame y coordinate, matching the project's convention.
    """

    map_yaml_path = Path(map_yaml_path)
    with map_yaml_path.open("r", encoding="utf-8") as handle:
        metadata = yaml.safe_load(handle)

    image_path = map_yaml_path.parent / metadata["image"]
    image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    if image is None:
        raise FileNotFoundError(f"Failed to load occupancy image: {image_path}")

    image = cv2.flip(image, 0)

    image_norm = image.astype(np.float32) / 255.0
    negate = int(metadata.get("negate", 0))
    if negate:
        occupancy_prob = image_norm
    else:
        occupancy_prob = 1.0 - image_norm

    occupied_thresh = float(metadata["occupied_thresh"])
    binary = (occupancy_prob >= occupied_thresh).astype(np.uint8)

    resolution = float(metadata["resolution"])
    origin_xy = np.asarray(metadata["origin"][:2], dtype=np.float64)
    return binary, resolution, origin_xy


def extract_pile_centres(
    binary: np.ndarray,
    resolution: float,
    origin_xy: np.ndarray,
    pile_width: float | None = None,
    pile_height: float | None = None,
    area_tolerance_low: float = 0.25,
    area_tolerance_high: float = 4.0,
    repeat_threshold: int = 3,
) -> tuple[np.ndarray, float, float]:
    """Extract repeated rectangular pile components from a binary occupancy map.

    If width/height are not provided, the extractor infers the pile components
    from repeated connected-component dimensions and estimates global width and
    height from the surviving components.
    """

    num_labels, _labels, stats, centroids = cv2.connectedComponentsWithStats(
        binary, connectivity=8
    )

    component_ids = np.arange(1, num_labels)
    if component_ids.size == 0:
        raise RuntimeError("No occupied connected components found in occupancy grid")

    areas = stats[component_ids, cv2.CC_STAT_AREA].astype(np.float64)
    widths_px = stats[component_ids, cv2.CC_STAT_WIDTH].astype(np.float64)
    heights_px = stats[component_ids, cv2.CC_STAT_HEIGHT].astype(np.float64)

    if pile_width is not None and pile_height is not None:
        expected_area_px = (pile_width * pile_height) / (resolution**2)
        min_area = expected_area_px * area_tolerance_low
        max_area = expected_area_px * area_tolerance_high
        valid = component_ids[(areas >= min_area) & (areas <= max_area)]
    else:
        width_counts = Counter(widths_px.astype(int).tolist())
        height_counts = Counter(heights_px.astype(int).tolist())
        area_counts = Counter(areas.astype(int).tolist())

        frequent_widths = {
            width for width, count in width_counts.items() if count >= repeat_threshold
        }
        frequent_heights = {
            height
            for height, count in height_counts.items()
            if count >= repeat_threshold
        }
        frequent_areas = {
            area for area, count in area_counts.items() if count >= repeat_threshold
        }

        valid_mask = (
            np.isin(widths_px.astype(int), list(frequent_widths))
            & np.isin(heights_px.astype(int), list(frequent_heights))
            & np.isin(areas.astype(int), list(frequent_areas))
        )
        valid = component_ids[valid_mask]

    if valid.size == 0:
        raise RuntimeError("No pile-like components detected in occupancy grid")

    centres_px = centroids[valid]
    centres_world = np.column_stack(
        (
            origin_xy[0] + centres_px[:, 0] * resolution,
            origin_xy[1] + centres_px[:, 1] * resolution,
        )
    )

    widths_m = stats[valid, cv2.CC_STAT_WIDTH].astype(np.float64) * resolution
    heights_m = stats[valid, cv2.CC_STAT_HEIGHT].astype(np.float64) * resolution

    # For very small rasterised rectangles, the observed bounding box toggles
    # between neighbouring pixel counts. Mean width/height is a better global
    # estimate than the median for this map.
    estimated_width = float(np.mean(widths_m))
    estimated_height = float(np.mean(heights_m))

    if pile_width is not None:
        if abs(estimated_width - pile_width) > 0.5 * pile_width:
            raise RuntimeError(
                "Extracted pile width "
                f"{estimated_width:.3f} m differs from configured {pile_width:.3f} m"
            )
        width = float(pile_width)
    else:
        width = estimated_width

    if pile_height is not None:
        if abs(estimated_height - pile_height) > 0.5 * pile_height:
            raise RuntimeError(
                "Extracted pile height "
                f"{estimated_height:.3f} m differs from configured {pile_height:.3f} m"
            )
        height = float(pile_height)
    else:
        height = estimated_height

    order = np.lexsort((centres_world[:, 0], centres_world[:, 1]))
    return centres_world[order], width, height


def build_boundary_segments(
    pile_centres: np.ndarray, pile_width: float, pile_height: float
) -> np.ndarray:
    """Build axis-aligned rectangle boundary segments for each pile."""

    half_width = pile_width / 2.0
    half_height = pile_height / 2.0
    segments = []

    for cx, cy in pile_centres:
        corners = np.array(
            [
                [cx - half_width, cy - half_height],
                [cx + half_width, cy - half_height],
                [cx + half_width, cy + half_height],
                [cx - half_width, cy + half_height],
            ],
            dtype=np.float64,
        )
        for index in range(4):
            segments.append(np.stack((corners[index], corners[(index + 1) % 4])))

    return np.asarray(segments, dtype=np.float64)


def rasterise_segments(
    segments: np.ndarray,
    origin_xy: np.ndarray,
    resolution: float,
    grid_shape: tuple[int, int],
) -> np.ndarray:
    """Rasterise world-frame line segments into a binary boundary mask."""

    mask = np.zeros(grid_shape, dtype=np.uint8)
    height, width = grid_shape

    for segment in segments:
        p1, p2 = segment
        c1 = int(round((p1[0] - origin_xy[0]) / resolution))
        r1 = int(round((p1[1] - origin_xy[1]) / resolution))
        c2 = int(round((p2[0] - origin_xy[0]) / resolution))
        r2 = int(round((p2[1] - origin_xy[1]) / resolution))

        c1 = int(np.clip(c1, 0, width - 1))
        c2 = int(np.clip(c2, 0, width - 1))
        r1 = int(np.clip(r1, 0, height - 1))
        r2 = int(np.clip(r2, 0, height - 1))
        cv2.line(mask, (c1, r1), (c2, r2), color=1, thickness=1)

    return mask


def compute_bdt(
    segments: np.ndarray,
    origin_xy: np.ndarray,
    resolution: float,
    grid_shape: tuple[int, int],
    dt_max_dist: float = 5.0,
) -> tuple[np.ndarray, np.ndarray]:
    """Compute the boundary distance transform in metres."""

    boundary_mask = rasterise_segments(segments, origin_xy, resolution, grid_shape)
    bdt_pixels = scipy.ndimage.distance_transform_edt(boundary_mask == 0)
    bdt_metres = np.clip(bdt_pixels * resolution, 0.0, dt_max_dist)
    return bdt_metres, boundary_mask


def process_map(
    map_yaml_path: str | Path,
    pile_width: float | None = None,
    pile_height: float | None = None,
    dt_max_dist: float = 5.0,
) -> PileMap:
    """Run the full occupancy-grid to BDT processing pipeline."""

    binary, resolution, origin_xy = load_occupancy(map_yaml_path)
    pile_centres, pile_width, pile_height = extract_pile_centres(
        binary,
        resolution,
        origin_xy,
        pile_width=pile_width,
        pile_height=pile_height,
    )
    segments = build_boundary_segments(pile_centres, pile_width, pile_height)
    bdt_metres, boundary_mask = compute_bdt(
        segments,
        origin_xy,
        resolution,
        binary.shape,
        dt_max_dist=dt_max_dist,
    )

    return PileMap(
        pile_centres=pile_centres,
        pile_width=pile_width,
        pile_height=pile_height,
        segments=segments,
        bdt_metres=bdt_metres,
        resolution=resolution,
        origin_xy=origin_xy,
        binary=binary,
        boundary_mask=boundary_mask,
    )
