# Frontend-to-Boundary Compatibility Evaluation

## Purpose

Before implementing the localisation backend, validate the core assumption:

> At the correct pose, do frontend feature points consistently land near the pile boundaries the backend expects?

If this assumption is wrong, the boundary distance field cost function will be optimising against the wrong measurement model, and the entire backend direction needs adjustment.

## Progress Checklist

- [x] Clarify evaluation intent: compare boundary compatibility against raw CFAR points, not clustered blob representatives
- [x] Step 1a: Add recorder node for synchronized feature and GT pose capture
- [x] Step 1b: Add a runnable launch path that disables clustering and replays `/oceansim/robot/gt_pose`
- [x] Step 1c: Run the recorder on the bag and produce the raw-CFAR dataset
- [x] Step 2: Extract boundary map offline
- [x] Step 3: Analyse boundary distances and plots offline
- [ ] Step 4: Debug residual registration / warping mismatch across `sonar-SLAM` and the pile simulation repo

## Current Status

### Step 1 result

Step 1 was rerun successfully against `testing_data/dvl_fallback_0.2trans_0.4rot_fixed/`.
The recorded dataset now exists at `evaluation/recorded_raw_cfar.pkl`.

Observed output summary:
- 4016 synced frames recorded
- 4010 non-empty frames
- 6 empty frames
- Mean points per frame: 2473.564
- Median points per frame: 2096
- 95th percentile points per frame: 6036
- Max points in a frame: 12719

Implementation notes from the run:
- `bruce_slam.utils.conversions.r2n()` needed a `PointCloud2` fix: `read_points()` was producing a 1-D structured array on this ROS 2 setup, so the old reshape path crashed the recorder. Using `read_points_numpy()` fixed it.
- `frontend_boundary_eval_launch.py` must play the bag once, not with `-l`. Looping prevented Step 1c from terminating naturally and interrupted pickle saving on shutdown.

### Geometry correction

The simulator piles are **not square pylons**. For this evaluation, Step 2 and Step 3 must use a **rectangular pile boundary model with independent width and height** extracted from the occupancy grid:
- detect pile centres from connected components
- estimate global pile width and pile height from component bounding boxes
- build rectangular boundaries and the corresponding boundary distance field

The older square-boundary wording below is stale where it conflicts with this correction.

### Step 2 result

Step 2 was completed against the updated high-resolution `gt_occupancy_grid.png` / `gt_occupancy_grid.yaml` map, which now contains only piles and no extra non-pile features.

Produced artifacts:
- `evaluation/boundary_map_rectangular.npz`
- `evaluation/boundary_map_rectangular_summary.json`
- `evaluation/boundary_map_rectangular_preview.png`

Observed extraction summary:
- 62 piles extracted
- Map resolution: 0.01 m/pixel
- Estimated pile width: 0.1481 m
- Estimated pile height: 0.1076 m
- Min pile spacing: 0.7184 m
- BDT shape: 1499 x 4100
- Boundary pixels rasterised: 3030

Sanity checks:
- The preview overlay shows the extracted rectangular boundaries aligned with the pile glyphs in the updated map.
- BDT at pile centres is 0.05 m for all extracted piles, consistent with the pile interior being bounded by the rasterised rectangle edges.

### Step 3 result

Step 3 was run against:
- `evaluation/recorded_raw_cfar.pkl`
- `evaluation/boundary_map_rectangular.npz`

Produced artifacts:
- `evaluation/results_rectangular/summary.json`
- `evaluation/results_rectangular/frame_stats.json`
- `evaluation/results_rectangular/boundary_distance_histogram.png`
- `evaluation/results_rectangular/boundary_distance_timeseries.png`
- `evaluation/results_rectangular/boundary_distance_spatial.png`

Observed analysis summary:
- 4016 frames analysed
- 9,933,833 total feature points
- 9,927,576 in-bounds points
- 6,257 out-of-bounds points
- Median boundary distance: 1.029 m
- Mean boundary distance: 1.101 m
- Std boundary distance: 0.541 m
- P10 / P50 / P90: 0.458 m / 1.029 m / 1.904 m

Interpretation against the outcome table:
- This is **not** a tight near-zero boundary-distance distribution.
- The histogram is broad and multimodal, with the dominant mass far from 0 m.
- The spatial scatter suggests structured arcs and bands around the pile field rather than points sitting directly on the extracted pile rectangles.

Current conclusion:
- The core Step 3 evaluation result is **negative** for the current measurement-model assumption.
- With the current raw-CFAR data, GT transform, and extracted rectangular map, frontend points do **not** consistently land near the pile boundaries the backend expects.
- Before committing to the backend cost function as-is, the project needs to explain this mismatch: possible causes include a frame/map alignment issue, a systematic measurement bias, or a deeper mismatch between what the frontend emits and the boundary representation being scored.

### Post-Step-3 debugging update: global alignment bug

Follow-up debugging found that the large visual mismatch was not caused by skipped frames and was not primarily caused by a local point-frame rotation bug. The dominant issue was a **constant world-frame translation error** between the projected points / GT trajectory and the extracted map.

Empirical alignment estimate from the offline data:
- best global translation: approximately **(+0.45 m, +1.00 m)** in `(x, y)`

Evidence:
- unaligned full-dataset median boundary distance: **1.029 m**
- aligned full-dataset median boundary distance after applying `(dx, dy) = (0.45, 1.00)`: **0.463 m**
- out-of-bounds points dropped from **6257** to **446**

Interpretation:
- This strongly suggests a **map-origin or world-to-map registration error** in the evaluation setup.
- The issue is consistent with the updated map image and YAML not matching the world-frame origin used by `/oceansim/robot/gt_pose`.
- The local feature point transform itself appears broadly consistent; global XY offset explains far more error than alternative point-frame rotations or flips.
- Cross-repo debugging in the `oceansim` repo did **not** find evidence that `/oceansim/robot/gt_pose` itself is published in the wrong frame. The report found that the pose publisher uses the robot's Isaac Sim **world transform directly**, labels it as `map`, and also publishes `map -> base_link` TF with the same pose values.
- The strongest simulator-side hypothesis from that report is therefore **map-image / YAML registration semantics** rather than a broken GT pose publisher.
- Relevant frame-convention clue from the `oceansim` side: their navigation utilities treat YAML `origin` as the **bottom-left world corner** of the image and apply an image-row inversion for `+y`. If the Isaac occupancy-grid plugin export uses different origin semantics, the current evaluation loader would introduce a fixed translation even when the YAML looks plausible.

New artifacts produced during debugging:
- `evaluation/results_rectangular/alignment_estimate.json`
- `evaluation/results_rectangular_aligned/summary.json`
- `evaluation/results_rectangular_aligned/boundary_distance_histogram.png`
- `evaluation/results_rectangular_aligned/boundary_distance_timeseries.png`
- `evaluation/results_rectangular_aligned/boundary_distance_spatial.png`
- `evaluation/results_rectangular_aligned/frontend_boundary_animation_mjpg_aligned.avi`

Updated conclusion:
- The original Step 3 result was partially invalidated by a global registration bug.
- After correcting the estimated offset, compatibility improves substantially but is still **not** a tight near-zero distribution.
- The measurement-model question is therefore still open, but the first thing to trust is the **aligned** results, not the unaligned ones.

### Current unresolved state

The user later updated `gt_occupancy_grid.yaml` so the map origin should match the world-frame origin of the trajectory. That change improved the raw unaligned metrics somewhat, but **did not remove** the dominant fitted translation:

- new unaligned median boundary distance: **0.519 m**
- best fitted translation is still approximately **(+0.45 m, +1.00 m)**

This means the remaining issue is **not explained** by the latest YAML origin edit alone.

An additional visual clue is that the sonar returns appear to show some **warping / distortion over motion**, not just a rigid translation error. The mismatch changes slightly over time, which suggests the next debugging target is no longer only map registration.

Leading hypotheses now include:
- residual map/world registration bug or Isaac occupancy-export convention mismatch
- sonar rendering distortion in the simulator
- frontend interpretation / Cartesian remapping error in `feature_extraction.py`
- a vertical-FOV / 3D-to-2D projection effect causing range-dependent or pose-dependent distortion

Current ranked suspicion:
1. Isaac occupancy-grid export / YAML semantics mismatch
2. residual sonar rendering or frontend remapping distortion
3. vertical-FOV / projection effect
4. least likely: `/oceansim/robot/gt_pose` publisher bug

### Step 4: Debug residual registration / warping mismatch

This step should be run with an agent that has access to **both** this repo and the pile simulation repo.

Goals:
1. Validate the exact world/map/image registration semantics end-to-end.
2. Determine whether the remaining error is rigid registration, non-rigid distortion, or both.
3. Trace the sonar image formation path from simulator rendering through ROS messages into frontend Cartesian points.
4. Check whether vertical FOV, depth variation, or sonar rendering assumptions can explain the observed warp.

Recommended outputs:
- a written cross-repo debugging report
- a small set of controlled overlays or quantitative checks isolating:
  - rigid XY offset
  - yaw error
  - range scale/bias
  - non-rigid warp over time / range / bearing
  - any vertical-FOV-induced projection mismatch

## What we need

1. **Ground truth pose** for each sonar frame — available from `/oceansim/robot/gt_pose` in the bag.
2. **Frontend feature points** for each frame — produced by the existing feature extraction pipeline from `/sonar/ping`.
3. **Boundary map** — extracted from `gt_occupancy_grid.png` using a pile-centre + rectangular-boundary pipeline derived from the simulator occupancy grid.

## Approach

### Step 1: Record features with ground truth

Run the existing pipeline (Kalman + feature extraction) against the bag, and simultaneously record the GT pose and frontend features to a CSV or pickle file for offline analysis.

For this evaluation, **disable clustering** and record the **raw CFAR points** only. The goal is to test whether the unclustered frontend evidence already lands near pile boundaries before any blob-representative abstraction is introduced.

This requires a small **evaluation recorder node** that:
- Subscribes to `/bruce/slam/feature_extraction/feature` (PointCloud2, `base_link` frame)
- Subscribes to `/oceansim/robot/gt_pose` (PoseStamped, `map` frame)
- Uses `ApproximateTimeSynchronizer` to pair them (same pattern as SLAM node: queue=20, max_delay=0.5s)
- On each synchronized pair: transforms the feature points from `base_link` to `map` frame using the GT pose, and saves `(timestamp, gt_pose, points_in_map_frame)` to disk

No changes to the existing frontend are needed. The recorder is a new standalone node.
Only a parameter override is needed so the existing feature extraction node publishes raw CFAR points (`clustering.enable: false`).

### Step 2: Extract boundary map (offline)

A standalone Python script that:
1. Loads `gt_occupancy_grid.png` + `map.yaml`
2. Extracts pile centres via connected components (PRD S6 stage 2)
3. Constructs rectangular pile boundaries from the extracted centres and global pile width / height
4. Computes the boundary distance field (PRD S6 stage 4)
5. Saves the boundary map (pile centres, segments, BDT array) to disk

This script is also useful for confirming the map extraction pipeline works correctly before the full backend is implemented.

### Step 3: Analyse (offline)

A Jupyter notebook or Python script that:

1. Loads the recorded features-with-GT-pose data
2. Loads the boundary map
3. For each frame, computes:
   - **Per-point boundary distance**: for each feature point (in map frame), look up the BDT value
   - **Frame-level statistics**: median, mean, std, 90th percentile of boundary distances
   - **Point count**: how many feature points per frame
4. Produces:
   - **Histogram** of per-point boundary distances across all frames (the key plot)
   - **Time series** of per-frame median boundary distance
   - **Scatter plot** of point locations coloured by boundary distance, overlaid on the rectangular boundary map
   - **Per-frame point count** time series

### What we're looking for

| Outcome | Interpretation | Action |
|---|---|---|
| Tight unimodal distribution near 0 | Frontend points land on boundaries | Proceed with BDT cost as-is |
| Tight distribution with consistent offset (e.g. 0.2-0.5m) | Points are biased inward/outward from boundary | Add bias correction to cost function (like `r_pile` but empirically measured) |
| Bimodal distribution (some near 0, some far) | Mix of boundary hits and clutter/false positives | Fine — Huber loss handles this; check that the near-0 mode is dominant |
| Wide/flat distribution, no clear mode near 0 | Frontend output doesn't correspond to boundaries | Measurement model needs rethinking; consider frontend changes |

## Implementation

### New files needed

```
bruce_slam/
  scripts/
    eval_recorder_node.py       # ROS2 node: records features + GT pose
  src/bruce_slam/
    map_processing.py           # Standalone: occupancy → pile centres → rectangular boundaries → BDT
                                # (also reusable by the eventual backend)
evaluation/
  extract_boundary_map.py       # Script: runs map_processing, saves to disk
  analyse_frontend_boundary.py  # Script/notebook: loads data, produces plots
```

### eval_recorder_node.py

```python
#!/usr/bin/env python3
"""Record frontend features paired with GT pose for offline evaluation."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped
from message_filters import ApproximateTimeSynchronizer, Subscriber
import numpy as np
import pickle
from pathlib import Path

from bruce_slam.utils.conversions import r2n  # PointCloud2 → numpy


class EvalRecorderNode(Node):
    def __init__(self):
        super().__init__('eval_recorder')

        self.declare_parameter('output_path', 'evaluation/recorded_data.pkl')
        self.output_path = Path(
            self.get_parameter('output_path').get_parameter_value().string_value
        )
        self.output_path.parent.mkdir(parents=True, exist_ok=True)

        # Subscribe to frontend features and GT pose
        self.feature_sub = Subscriber(
            self, PointCloud2, '/bruce/slam/feature_extraction/feature'
        )
        self.gt_sub = Subscriber(
            self, PoseStamped, '/oceansim/robot/gt_pose'
        )

        # Approximate time sync (same as SLAM node)
        self.sync = ApproximateTimeSynchronizer(
            [self.feature_sub, self.gt_sub],
            queue_size=20,
            slop=0.5,
        )
        self.sync.registerCallback(self.callback)

        self.records = []
        self.get_logger().info(f'Recording to {self.output_path}')

    def callback(self, feature_msg, gt_msg):
        # Extract feature points (Nx3 in base_link frame)
        points_bl = r2n(feature_msg)  # returns Nx3 numpy array

        # Skip NaN placeholder frames
        if points_bl is None or len(points_bl) == 0:
            return
        if np.any(np.isnan(points_bl)):
            return

        # Extract GT pose (x, y, theta) from PoseStamped
        p = gt_msg.pose.position
        q = gt_msg.pose.orientation
        # Quaternion to yaw (2D)
        import math
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        gt_pose = np.array([p.x, p.y, yaw])

        # Transform points from base_link to map frame using GT pose
        cos_t, sin_t = np.cos(yaw), np.sin(yaw)
        R = np.array([[cos_t, -sin_t], [sin_t, cos_t]])
        t = np.array([p.x, p.y])
        points_map = (R @ points_bl[:, :2].T).T + t

        stamp = feature_msg.header.stamp.sec + feature_msg.header.stamp.nanosec * 1e-9

        self.records.append({
            'stamp': stamp,
            'gt_pose': gt_pose,
            'points_base_link': points_bl[:, :2].copy(),
            'points_map': points_map.copy(),
            'n_points': len(points_map),
        })

        if len(self.records) % 100 == 0:
            self.get_logger().info(f'Recorded {len(self.records)} frames')

    def save(self):
        with open(self.output_path, 'wb') as f:
            pickle.dump(self.records, f)
        self.get_logger().info(
            f'Saved {len(self.records)} frames to {self.output_path}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = EvalRecorderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.save()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

### map_processing.py (reusable module)

```python
"""
Pile map processing: occupancy grid → pile centres → boundary segments → BDT.

This module is used by both the evaluation scripts and the eventual
PileLocalisation backend. It has no ROS dependencies.
"""

import numpy as np
import cv2
import scipy.ndimage
import yaml
from dataclasses import dataclass
from pathlib import Path


@dataclass
class PileMap:
    """Processed pile map ready for localisation or evaluation."""
    pile_centres: np.ndarray       # (N, 2) world coordinates
    pile_side_length: float        # metres
    segments: list                 # list of ((x1,y1), (x2,y2)) boundary segments
    bdt_metres: np.ndarray         # (H, W) boundary distance field in metres
    resolution: float              # metres per pixel
    origin_xy: np.ndarray          # (2,) world coord of pixel (0,0)
    binary: np.ndarray             # (H, W) binary occupancy grid


def load_occupancy(map_yaml_path: str) -> tuple:
    """Load and orient occupancy grid from map_server YAML + PNG."""
    meta = yaml.safe_load(open(map_yaml_path))
    resolution = meta['resolution']
    origin_xy = np.array(meta['origin'][:2])
    occ_thresh = meta['occupied_thresh']
    negate = meta.get('negate', 0)

    image_path = Path(map_yaml_path).parent / meta['image']
    img = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
    img = cv2.flip(img, 0)  # row 0 → minimum y
    img_norm = img / 255.0
    if negate:
        img_norm = 1.0 - img_norm
    binary = (img_norm > occ_thresh).astype(np.uint8)

    return binary, resolution, origin_xy


def extract_pile_centres(
    binary: np.ndarray,
    resolution: float,
    origin_xy: np.ndarray,
    pile_side_length: float = None,
    area_tolerance_low: float = 0.25,
    area_tolerance_high: float = 4.0,
) -> tuple:
    """Extract pile centres and estimate side length from binary occupancy."""
    num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary)

    # Filter by area
    if pile_side_length is not None:
        expected_area_px = (pile_side_length / resolution) ** 2
        min_area = expected_area_px * area_tolerance_low
        max_area = expected_area_px * area_tolerance_high
    else:
        min_area = 4
        max_area = binary.size * 0.1

    valid = []
    for i in range(1, num_labels):
        area = stats[i, cv2.CC_STAT_AREA]
        if min_area <= area <= max_area:
            valid.append(i)

    if not valid:
        raise RuntimeError("No piles detected in occupancy grid")

    centres_px = centroids[valid]  # (N, 2) as (col, row)
    centres_world = np.column_stack([
        origin_xy[0] + centres_px[:, 0] * resolution,
        origin_xy[1] + centres_px[:, 1] * resolution,
    ])

    # Estimate side length from median bounding box
    widths = stats[valid, cv2.CC_STAT_WIDTH] * resolution
    heights = stats[valid, cv2.CC_STAT_HEIGHT] * resolution
    estimated_side = float(np.median(np.concatenate([widths, heights])))

    if pile_side_length is not None:
        if abs(estimated_side - pile_side_length) > 0.5 * pile_side_length:
            raise RuntimeError(
                f"Extracted side {estimated_side:.3f}m vs configured {pile_side_length:.3f}m"
            )
        side = pile_side_length
    else:
        side = estimated_side

    return centres_world, side


def build_boundary_segments(centres: np.ndarray, side_length: float) -> list:
    """Build boundary line segments for square piles."""
    s2 = side_length / 2.0
    segments = []
    for cx, cy in centres:
        corners = [
            (cx - s2, cy - s2),
            (cx + s2, cy - s2),
            (cx + s2, cy + s2),
            (cx - s2, cy + s2),
        ]
        for i in range(4):
            segments.append((corners[i], corners[(i + 1) % 4]))
    return segments


def compute_bdt(
    segments: list,
    origin_xy: np.ndarray,
    resolution: float,
    grid_shape: tuple,
    dt_max_dist: float = 5.0,
) -> np.ndarray:
    """Compute boundary distance field from segments."""
    mask = np.zeros(grid_shape, dtype=np.uint8)
    H, W = grid_shape

    for (x1, y1), (x2, y2) in segments:
        c1 = int(round((x1 - origin_xy[0]) / resolution))
        r1 = int(round((y1 - origin_xy[1]) / resolution))
        c2 = int(round((x2 - origin_xy[0]) / resolution))
        r2 = int(round((y2 - origin_xy[1]) / resolution))
        c1, c2 = np.clip([c1, c2], 0, W - 1)
        r1, r2 = np.clip([r1, r2], 0, H - 1)
        cv2.line(mask, (c1, r1), (c2, r2), 1, thickness=1)

    bdt_px = scipy.ndimage.distance_transform_edt(mask == 0)
    bdt_m = np.clip(bdt_px * resolution, 0.0, dt_max_dist)
    return bdt_m


def process_map(map_yaml_path: str, pile_side_length: float = None,
                dt_max_dist: float = 5.0) -> PileMap:
    """Full pipeline: YAML → PileMap."""
    binary, resolution, origin_xy = load_occupancy(map_yaml_path)
    centres, side = extract_pile_centres(binary, resolution, origin_xy, pile_side_length)
    segments = build_boundary_segments(centres, side)
    bdt = compute_bdt(segments, origin_xy, resolution, binary.shape, dt_max_dist)

    return PileMap(
        pile_centres=centres,
        pile_side_length=side,
        segments=segments,
        bdt_metres=bdt,
        resolution=resolution,
        origin_xy=origin_xy,
        binary=binary,
    )
```

### analyse_frontend_boundary.py

```python
"""
Analyse frontend feature compatibility with boundary map.

Usage:
    python analyse_frontend_boundary.py \
        --data evaluation/recorded_data.pkl \
        --map bruce_slam/config/map.yaml \
        [--pile-side-length 1.0] \
        [--output evaluation/results/]
"""

import argparse
import pickle
import numpy as np
import matplotlib.pyplot as plt
from pathlib import Path

# Assumes map_processing.py is importable (add to PYTHONPATH or use relative import)
from bruce_slam.map_processing import process_map


def lookup_bdt(points_map, pile_map):
    """Look up BDT values for points in map frame."""
    cols = (points_map[:, 0] - pile_map.origin_xy[0]) / pile_map.resolution
    rows = (points_map[:, 1] - pile_map.origin_xy[1]) / pile_map.resolution
    H, W = pile_map.bdt_metres.shape
    cols_i = np.clip(np.round(cols).astype(int), 0, W - 1)
    rows_i = np.clip(np.round(rows).astype(int), 0, H - 1)
    return pile_map.bdt_metres[rows_i, cols_i]


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--data', required=True, help='Path to recorded_data.pkl')
    parser.add_argument('--map', required=True, help='Path to map.yaml')
    parser.add_argument('--pile-side-length', type=float, default=None)
    parser.add_argument('--output', default='evaluation/results/')
    args = parser.parse_args()

    out = Path(args.output)
    out.mkdir(parents=True, exist_ok=True)

    # Load data
    with open(args.data, 'rb') as f:
        records = pickle.load(f)
    print(f"Loaded {len(records)} frames")

    # Load map
    pile_map = process_map(args.map, args.pile_side_length)
    print(f"Map: {len(pile_map.pile_centres)} piles, side={pile_map.pile_side_length:.3f}m")

    # Compute per-point boundary distances
    all_dists = []
    all_points = []
    frame_stats = []

    for rec in records:
        pts = rec['points_map']
        if len(pts) == 0:
            continue
        dists = lookup_bdt(pts, pile_map)
        all_dists.append(dists)
        all_points.append(pts)
        frame_stats.append({
            'stamp': rec['stamp'],
            'n_points': len(pts),
            'median_dist': float(np.median(dists)),
            'mean_dist': float(np.mean(dists)),
            'p90_dist': float(np.percentile(dists, 90)),
            'std_dist': float(np.std(dists)),
        })

    all_dists = np.concatenate(all_dists)
    all_points = np.concatenate(all_points)
    stamps = [s['stamp'] for s in frame_stats]

    print(f"\n=== RESULTS ===")
    print(f"Total points: {len(all_dists)}")
    print(f"Boundary distance: median={np.median(all_dists):.3f}m, "
          f"mean={np.mean(all_dists):.3f}m, std={np.std(all_dists):.3f}m")
    print(f"  p10={np.percentile(all_dists, 10):.3f}m, "
          f"p50={np.percentile(all_dists, 50):.3f}m, "
          f"p90={np.percentile(all_dists, 90):.3f}m")

    # --- Plot 1: Histogram of boundary distances ---
    fig, ax = plt.subplots(figsize=(10, 5))
    ax.hist(all_dists, bins=100, range=(0, 3.0), edgecolor='black', alpha=0.7)
    ax.axvline(np.median(all_dists), color='red', linestyle='--', label=f'median={np.median(all_dists):.2f}m')
    ax.set_xlabel('Distance to nearest boundary (m)')
    ax.set_ylabel('Count')
    ax.set_title('Frontend Feature Points: Distance to Nearest Pile Boundary')
    ax.legend()
    fig.savefig(out / 'boundary_distance_histogram.png', dpi=150, bbox_inches='tight')
    print(f"Saved {out / 'boundary_distance_histogram.png'}")

    # --- Plot 2: Time series of per-frame median distance ---
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 6), sharex=True)
    ax1.plot(stamps, [s['median_dist'] for s in frame_stats], '.', markersize=2)
    ax1.set_ylabel('Median boundary dist (m)')
    ax1.set_title('Per-Frame Median Boundary Distance')
    ax2.plot(stamps, [s['n_points'] for s in frame_stats], '.', markersize=2, color='green')
    ax2.set_ylabel('Point count')
    ax2.set_xlabel('Time (s)')
    fig.savefig(out / 'boundary_distance_timeseries.png', dpi=150, bbox_inches='tight')
    print(f"Saved {out / 'boundary_distance_timeseries.png'}")

    # --- Plot 3: Spatial scatter coloured by boundary distance ---
    fig, ax = plt.subplots(figsize=(12, 12))
    # Plot boundary segments
    for (x1, y1), (x2, y2) in pile_map.segments:
        ax.plot([x1, x2], [y1, y2], 'k-', linewidth=0.5, alpha=0.5)
    # Plot pile centres
    ax.plot(pile_map.pile_centres[:, 0], pile_map.pile_centres[:, 1], 'rx', markersize=4)
    # Plot points coloured by distance
    sc = ax.scatter(all_points[:, 0], all_points[:, 1], c=all_dists,
                    cmap='RdYlGn_r', s=1, vmin=0, vmax=2.0, alpha=0.3)
    plt.colorbar(sc, ax=ax, label='Boundary distance (m)')
    ax.set_aspect('equal')
    ax.set_title('Feature Points Coloured by Boundary Distance')
    ax.set_xlabel('x (m)')
    ax.set_ylabel('y (m)')
    fig.savefig(out / 'boundary_distance_spatial.png', dpi=150, bbox_inches='tight')
    print(f"Saved {out / 'boundary_distance_spatial.png'}")

    # Save stats
    import json
    summary = {
        'n_frames': len(frame_stats),
        'n_points_total': int(len(all_dists)),
        'boundary_dist_median': float(np.median(all_dists)),
        'boundary_dist_mean': float(np.mean(all_dists)),
        'boundary_dist_std': float(np.std(all_dists)),
        'boundary_dist_p10': float(np.percentile(all_dists, 10)),
        'boundary_dist_p50': float(np.percentile(all_dists, 50)),
        'boundary_dist_p90': float(np.percentile(all_dists, 90)),
    }
    with open(out / 'summary.json', 'w') as f:
        json.dump(summary, f, indent=2)
    print(f"Saved {out / 'summary.json'}")


if __name__ == '__main__':
    main()
```

## How to run

### Step 1: Record data

```bash
# Terminal 1: play the bag with GT pose topic included
ros2 bag play testing_data/dvl_fallback_0.2trans_0.4rot_fixed/ -l --clock \
    --topics /dvl/data /sonar/ping /oceansim/robot/imu /oceansim/robot/gt_pose

# Terminal 2: launch frontend (Kalman + feature extraction only, no SLAM)
ros2 launch bruce_slam test_launch.py enable_slam:=false

# Terminal 3: run the recorder
ros2 run bruce_slam eval_recorder_node.py --ros-args \
    -p output_path:=evaluation/recorded_data.pkl \
    -p use_sim_time:=true
```

Wait for the bag to play through once (or a representative segment), then Ctrl+C the recorder. It saves on shutdown.

### Step 2: Extract boundary map and analyse

```bash
# Extract map (one-time, no ROS needed)
python3 evaluation/extract_boundary_map.py

# Run analysis
python3 evaluation/analyse_frontend_boundary.py \
    --data evaluation/recorded_data.pkl \
    --map bruce_slam/config/map.yaml \
    --output evaluation/results/
```

### Step 3: Interpret results

Open `evaluation/results/boundary_distance_histogram.png` and check against the outcome table above. The spatial scatter plot shows where the biggest mismatches are.

## Prerequisites

- `map.yaml` must exist with at least placeholder values (the current ones in the PRD are fine for this evaluation — they don't need to be confirmed from blueprint yet, as long as they're roughly right for the simulation)
- `gt_occupancy_grid.png` exists at the repo root
- The bag must be played with `/oceansim/robot/gt_pose` included in `--topics`

## What this does NOT test

- Whether the LM optimiser converges correctly (that's a backend unit test)
- Whether accumulation helps or hurts (that needs the full pipeline)
- Whether ambiguity handling works (needs multi-hypothesis search)

This evaluation tests only the foundational assumption: does the measurement model match what the sensor produces?
