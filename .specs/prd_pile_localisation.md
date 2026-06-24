# PRD: Boundary-Centric Pile-Field Localisation Backend

## 1. Overview

Replace the occupancy-grid distance-field localisation concept (`GridLocalisationNode`) with a boundary-centric, ambiguity-aware localisation backend (`PileLocalisationNode`) that matches accumulated sonar feature evidence against a continuous structural boundary map derived from known pile geometry.

The frontend (CFAR feature extraction, Kalman dead-reckoning) is unchanged. The core pose/odometry/TF contract is preserved exactly. Legacy SLAM-specific visualisation topics are retired and replaced with debug topics suited to the new backend.

### Motivation

The previous occupancy-grid approach aligns feature points against a filled-cell distance field. This creates a semantic mismatch: sonar returns are surface/boundary observations, but the distance field rewards proximity to solid interior pixels. In repetitive pile layouts this mismatch, combined with the lack of ambiguity handling, leads to wrong-pile corrections and overconfident updates.

This backend addresses those problems by:

* matching against continuous pile-boundary geometry instead of filled occupancy
* treating sonar features as probabilistic, partial, anonymous boundary evidence
* explicitly detecting and handling ambiguous multi-hypothesis situations
* gating corrections so that only trustworthy updates reach the EKF

---

## 2. External Interface Compatibility

### 2a. Hard Contract (pose / odometry / TF)

These outputs must be preserved exactly — topic name, message type, header `frame_id`, child frame IDs, TF parent/child, and semantic meaning. They are the only outputs that other pipeline nodes depend on.

| Output | Topic | Type | `frame_id` | `child_frame_id` | Semantic meaning |
|---|---|---|---|---|---|
| Pose | `/bruce/slam/slam/pose` | `PoseWithCovarianceStamped` | `map` | — | Best estimate of robot pose in map frame |
| Odometry | `/bruce/slam/slam/odom` | `Odometry` | `map` | `base_link` | Same pose as above, in odometry message form |
| TF | `map` → `odom` | `TransformStamped` | `map` | `odom` | Correction transform from dead-reckoning frame to map |

### 2b. Inputs (unchanged)

| Input | Topic | Type | Notes |
|---|---|---|---|
| Feature cloud | `/bruce/slam/feature_extraction/feature` | `PointCloud2` | Frame `base_link` |
| Dead-reckoning | `/bruce/slam/localization/odom` | `Odometry` | Frame `odom`, child `base_link`; absolute pose |

### 2c. Retired Topics

These legacy SLAM-specific topics have no equivalent in the new backend and are removed:

| Topic | Reason |
|---|---|
| `/bruce/slam/slam/cloud` | No accumulated SLAM keypoint cloud concept |
| `/bruce/slam/slam/constraint` | No factor graph or loop closures |

If `mapping_node.py` or other consumers currently subscribe to these, they must be updated before the new backend is deployed.

---

## 3. Visualisation and Debug Outputs

These are not hard compatibility requirements. They exist for RViz inspection, tuning, and diagnostics. All are published by the new backend unless noted otherwise.

| Output | Topic | Type | Published when | Purpose |
|---|---|---|---|---|
| Feature image | `/bruce/slam/feature_extraction/feature_img` | `Image` | Every ping (frontend) | Cartesian feature image — **not published by this backend** |
| Trajectory | `/bruce/slam/slam/traj` | `PointCloud2` | Every callback | Continuously advancing robot estimate (full history, z=0) |
| Accumulated cloud | `/bruce/slam/localisation/accumulated_cloud` | `PointCloud2` | Every matching step | The motion-compensated cloud actually used for matching, in `map` frame |
| Prior map | `/bruce/slam/map/prior` | `nav_msgs/OccupancyGrid` | Once at startup (transient local) | GT occupancy grid for RViz overlay |
| Boundaries | `/bruce/slam/map/boundaries` | `visualization_msgs/MarkerArray` | Once at startup (transient local) | Extracted pile boundary segments in `map` frame — visual check against prior image |
| Hypotheses | `/bruce/slam/localisation/hypotheses` | `geometry_msgs/PoseArray` | Every matching step | All surviving candidate poses before ambiguity filtering |
| Status | `/bruce/slam/localisation/status` | `diagnostic_msgs/DiagnosticStatus` | Every callback | Regime, num_hypotheses, best_cost, gate result, degeneracy flag |

**Trajectory semantics:** `/bruce/slam/slam/traj` is published on **every callback** with the current EKF mean appended, regardless of whether an update was accepted. This preserves continuous-tracking semantics expected by existing consumers (e.g. `mapping_node.py`).

---

## 4. Launch Plan

`test_launch.py` and `slam_launch.py` are updated to launch `PileLocalisationNode` in place of `SLAMNode` (or `GridLocalisationNode` if it exists). This is a replacement, not an optional mode.

`slam_node.py`, `slam.py`, `slam_ros.py`, and any `grid_localisation_node.py` are retained in the repository but no longer launched by default.

---

## 5. Map Metadata — Implementation Prerequisite

> **This must be resolved before implementation begins.** Map metadata values are geometric invariants, not tuning parameters.

### 5a. Unresolved Geometric Invariants

These values must be confirmed from the jetty blueprint and filled into `map.yaml` / `pile_localisation.yaml` before implementation can proceed. They are **not** free tuning parameters.

| Value | Source | Current status | Config location |
|---|---|---|---|
| `resolution` | Blueprint / simulation world | `0.2` (placeholder — **CONFIRM**) | `map.yaml` |
| `origin` | Blueprint / simulation world | `[-100.0, -100.0, 0.0]` (placeholder — **CONFIRM**) | `map.yaml` |
| `pile_side_length` | Blueprint / simulation world | **UNRESOLVED** | `pile_localisation.yaml` |

### 5b. Map YAML Format

Map metadata is loaded from a standard ROS `map_server` YAML sidecar file alongside the PNG.

**`map.yaml` format:**
```yaml
image: gt_occupancy_grid.png
resolution: 0.2          # metres per pixel — CONFIRM FROM BLUEPRINT
origin: [-100.0, -100.0, 0.0]  # [x, y, yaw] of bottom-left pixel in world frame — CONFIRM
occupied_thresh: 0.65
free_thresh: 0.196
negate: 0
```

All values in `map.yaml` are loaded at runtime from configuration — not hardcoded. However, they represent physical ground truth, not algorithm knobs. Changing them without re-deriving from the actual environment will break localisation.

**Coordinate convention** is identical to the occupancy-grid PRD: PNG loaded, vertically flipped so row 0 = minimum y, `origin` = world coordinate of pixel `(0, 0)` after flip.

---

## 6. Prior Map Processing (Startup, One-Time)

Map processing happens in two stages: occupancy extraction, then boundary-centric conversion.

### Stage 1: Occupancy Extraction

```python
# Parse map.yaml
meta = yaml.safe_load(open(map_yaml_path))
resolution   = meta['resolution']
origin_xy    = meta['origin'][:2]
occ_thresh   = meta['occupied_thresh']
negate       = meta.get('negate', 0)

# Load and orient image
img = cv2.imread(map_image_path, cv2.IMREAD_GRAYSCALE)
img = cv2.flip(img, 0)                      # row 0 → minimum y
img_norm = img / 255.0
if negate:
    img_norm = 1.0 - img_norm
binary = (img_norm > occ_thresh).astype(np.uint8)   # 1 = occupied
```

### Stage 2: Pile Centre Extraction

Extract individual pile centres and a global side length from the binary occupancy grid using connected-component analysis:

```python
# Derive expected pile area in pixels from geometry (not a free tuning parameter)
expected_pile_area_px = (pile_side_length / resolution) ** 2
min_pile_area_px = expected_pile_area_px * pile_area_tolerance_low   # default: 0.25
max_pile_area_px = expected_pile_area_px * pile_area_tolerance_high  # default: 4.0
# If pile_side_length is not yet confirmed, use fallback pixel-area bounds from config

# Label connected occupied regions
num_labels, labels, stats, centroids = cv2.connectedComponentsWithStats(binary)

# Filter: reject background (label 0) and components outside expected pile area range
pile_centres_px = []
for i in range(1, num_labels):
    area = stats[i, cv2.CC_STAT_AREA]
    if min_pile_area_px <= area <= max_pile_area_px:
        pile_centres_px.append(centroids[i])   # (col, row) sub-pixel

# Convert to world coordinates
pile_centres = np.array([
    [origin_xy[0] + col * resolution, origin_xy[1] + row * resolution]
    for col, row in pile_centres_px
])

# Estimate global pile side length from median component bounding box
widths  = stats[valid_labels, cv2.CC_STAT_WIDTH] * resolution
heights = stats[valid_labels, cv2.CC_STAT_HEIGHT] * resolution
pile_side_length = np.median(np.concatenate([widths, heights]))
```

> If `pile_side_length` is provided as a parameter, the extracted value is used only as a sanity check.

### Startup Validation

At startup, after pile extraction, validate the extracted geometry and **fail loudly** if it does not match expectations:

```python
# Validate pile count
if len(pile_centres) == 0:
    raise RuntimeError("No piles detected in occupancy grid")
logger.info(f"Extracted {len(pile_centres)} piles")

# Validate side length against configured value (if provided)
if abs(estimated_side_length - pile_side_length) > 0.5 * pile_side_length:
    raise RuntimeError(
        f"Extracted side length {estimated_side_length:.3f}m differs "
        f"from configured {pile_side_length:.3f}m by more than 50%"
    )

# Validate spacing consistency
dists = scipy.spatial.distance.pdist(pile_centres)
min_spacing = np.min(dists)
logger.info(f"Min pile spacing: {min_spacing:.2f}m, pile side: {pile_side_length:.3f}m")
```

### Stage 3: Boundary Representation

From the pile centres and side length, construct continuous boundary segments:

```python
# For each square pile with centre (cx, cy) and half-side s/2:
# Four boundary segments (CCW):
#   bottom: (cx - s/2, cy - s/2) → (cx + s/2, cy - s/2)
#   right:  (cx + s/2, cy - s/2) → (cx + s/2, cy + s/2)
#   top:    (cx + s/2, cy + s/2) → (cx - s/2, cy + s/2)
#   left:   (cx - s/2, cy + s/2) → (cx - s/2, cy - s/2)
```

This yields a list of line segments representing all pile boundaries. A KD-tree (or similar spatial index) is built over a dense sampling of these segments for efficient nearest-boundary-distance queries at runtime.

### Stage 4: Boundary Distance Field

A boundary distance field is also precomputed on the grid for fast lookup:

```python
# Rasterise boundary segments onto a binary mask (1 = on boundary)
boundary_mask = rasterise_segments(segments, origin_xy, resolution, grid_shape)

# Distance transform from boundary pixels
bdt_pixels = scipy.ndimage.distance_transform_edt(boundary_mask == 0)
bdt_metres = np.clip(bdt_pixels * resolution, 0.0, dt_max_dist)
```

`bdt_metres[r, c]` is the Euclidean distance from pixel `(c, r)` to the nearest pile **boundary** (not interior). This is the primary runtime lookup table.

At startup, also publish the prior occupancy grid on `/bruce/slam/map/prior` and boundary markers on `/bruce/slam/map/boundaries`.

---

## 7. Per-Frame Pipeline

Triggered by `ApproximateTimeSynchronizer` on `SONAR_FEATURE_TOPIC` + `LOCALIZATION_ODOM_TOPIC` (queue=20, max_delay=0.5 s).

```
1. EKF Predict                          (S8)
2. Feature Accumulation                  (S9)  → may yield NO_EVIDENCE regime
3. Boundary-Centric Scoring              (S10)
4. Multi-Hypothesis Search & Pruning     (S11) → may yield NO_CONVERGENCE regime
5. Regime Classification & Ambiguity     (S12) → TRACKING, AMBIGUOUS, or fallback
6. Hessian→Covariance & Mahalanobis Gate (S13) → may yield GATE_REJECT regime
7. EKF Update (TRACKING + gate pass)    (S8)
8. Publish                               (S14)
```

---

## 8. EKF State and Equations

Identical to the occupancy-grid PRD. Reproduced for completeness.

**State:** `mu = [x, y, theta]^T` in the `map` frame. All covariances in `[x (m), y (m), theta (rad)]`.

**Initialisation (first callback):**
```
mu_0  = initial_pose param (x, y, theta)
Sigma_0  = diag(initial_covariance_stddev)^2
prev_odom_pose = current absolute odom pose
```

If a `geometry_msgs/PoseWithCovarianceStamped` arrives on `/initialpose` before the first callback, use it to override `mu_0` and `Sigma_0`.

### Odometry Delta Computation

```python
T_prev = SE2(prev_odom_pose)
T_curr = SE2(curr_odom_pose)
delta  = T_prev.inverse() * T_curr   # relative motion in body frame
prev_odom_pose = curr_odom_pose
# Always normalise theta to (-pi, pi]
```

First callback: delta = (0, 0, 0), no covariance inflation.

### Predict Step
```
mu_pred = mu + R(mu.theta) * [dx, dy, dtheta]^T
mu_pred.theta = wrap(mu_pred.theta)
F  = Jacobian of motion model w.r.t. [x, y, theta]
Sigma_pred = F * Sigma * F^T + Q
  Q = diag(odom_process_noise_stddev)^2
```

### Update Step (after gating, S13)
```
H = I_3
innovation = z - mu_pred
innovation.theta = wrap(innovation.theta)
S = H * Sigma_pred * H^T + R_match
K = Sigma_pred * H^T * S^-1
mu = mu_pred + K * innovation
mu.theta = wrap(mu.theta)
Sigma = (I - K * H) * Sigma_pred
```

---

## 9. Feature Accumulation (Sliding Window)

Identical to the occupancy-grid PRD with one addition.

- Ring buffer of the last `accumulation_window` (default: 5) feature clouds.
- Each entry stores: feature points in `base_link` frame + absolute odom pose at that timestamp.
- At scoring time, transform all stored clouds into the **current predicted pose frame** using relative odometry transforms, then concatenate.
- Skip frames where the feature cloud contains a NaN placeholder.

**Spatial filtering (uniform distance-based):** After concatenation, filter so that no two points are closer than `min_point_spacing` (default: 0.3 m). Shuffle points randomly, then greedily accept each point only if it is at least `min_point_spacing` from all already-accepted points (KD-tree for efficiency).

**Minimum evidence threshold:** If the accumulated cloud after filtering contains fewer than `min_evidence_points` (default: 15) points, skip the matching step entirely and output the predict-only estimate. This prevents noisy matches from near-empty clouds.

---

## 10. Boundary-Centric Scoring

### Cost Function

For candidate pose `T = (x, y, theta)` and accumulated feature cloud `{p_i}`:

```
cost(T) = sum_i  rho( BDT( R(theta) * p_i + t ) )

  rho(d) = Huber loss, threshold delta = huber_delta (metres):
             d^2 / 2              if |d| <= delta
             delta * (|d| - delta/2)  otherwise
  BDT(.) = bilinear interpolation into bdt_metres (boundary distance field)
```

**Key difference from the occupancy-grid approach:** The boundary distance field `bdt_metres` measures distance to the nearest pile **boundary segment**, not to the nearest occupied pixel. There is no `r_pile` subtraction needed because the distance is measured directly to the surface where sonar returns are physically expected.

**Bilinear interpolation** uses sub-pixel world-to-pixel mapping:
```python
col = (world_x - origin_xy[0]) / resolution
row = (world_y - origin_xy[1]) / resolution
# clamp to [0, width-1] x [0, height-1]; out-of-bounds points score dt_max_dist
```

### Solver: Levenberg-Marquardt (LM)

```
Per iteration:
  Compute residuals r_i and Jacobian rows J_i = drho/dT via chain rule through BDT
  J = stacked Jacobians, r = stacked residuals
  Solve: (J^T J + lambda * diag(J^T J)) * delta_pose = -J^T r
  If cost decreases: accept delta_pose, reduce lambda
  If cost increases: reject delta_pose, increase lambda
  Converged when ||delta_pose|| < convergence_tol or max_iterations reached
```

Initial `lambda = lm_lambda_init` (default: 1e-2). Scale factor `lm_lambda_factor` (default: 10).

---

## 11. Multi-Hypothesis Local Search

Unlike the single-optimizer occupancy-grid approach, this backend evaluates multiple candidate poses within a bounded neighbourhood of the odometry prediction.

### Candidate Generation

```python
# Grid of initial guesses around mu_pred
offsets_xy = regular grid within [-search_radius, +search_radius] at search_step spacing
offsets_theta = [-search_angle, 0, +search_angle]  (or finer if configured)

candidates = [mu_pred + (dx, dy, dtheta) for dx, dy in offsets_xy for dtheta in offsets_theta]
```

Default parameters:
- `search_radius`: 2.0 m (should be less than pile spacing)
- `search_step`: 1.0 m
- `search_angle`: 0.175 rad (~10 deg)

### Per-Candidate Refinement

Each candidate is refined independently using the LM solver (S10). This produces a set of locally optimised hypotheses, each with:

- converged pose `T_k`
- final cost `c_k`
- Hessian `H_k = J^T J` at convergence

### Pruning

Discard hypotheses that:
- did not converge (hit max iterations without `||delta_pose|| < convergence_tol`)
- converged to the same pose as another hypothesis (within `merge_distance` = 0.2 m and `merge_angle` = 0.05 rad) — keep the one with lower cost
- have cost above `max_cost_ratio * min_cost` (default: 3.0) — clearly worse than the best

---

## 12. Runtime Regimes and Ambiguity Assessment

The per-frame pipeline classifies each callback into exactly one of the following regimes. Each regime defines the EKF action, covariance handling, and status output.

### Regime Table

| Regime | Trigger | EKF action | Covariance | Status field |
|---|---|---|---|---|
| `TRACKING` | Single dominant hypothesis after pruning (S12a) | Proceed to gating (S13) → update if gate passes | Fixed covariance, inflated if degenerate (S13) | `TRACKING` |
| `AMBIGUOUS` | Multiple plausible hypotheses survive (S12a) | Skip update; output `mu_pred` | Inflate by `ambiguity_inflation_factor` (default: 2.0) | `AMBIGUOUS` |
| `NO_EVIDENCE` | Accumulated cloud < `min_evidence_points` (S9), or all frames were NaN | Skip update; output `mu_pred` | Unchanged (odometry predict only) | `NO_EVIDENCE` |
| `NO_CONVERGENCE` | All LM candidates fail to converge (S11 pruning removes all) | Skip update; output `mu_pred` | Inflate by `ambiguity_inflation_factor` | `NO_CONVERGENCE` |
| `GATE_REJECT` | Best hypothesis in tracking regime, but Mahalanobis gate rejects (S13) | Skip update; output `mu_pred` | Unchanged | `GATE_REJECT` |

In all non-`TRACKING` regimes and `GATE_REJECT`:
- `/bruce/slam/localisation/hypotheses` is published empty (empty `PoseArray`) for `NO_EVIDENCE` and `NO_CONVERGENCE`, or with the surviving hypotheses for `AMBIGUOUS` and `GATE_REJECT`.
- `/bruce/slam/localisation/status` is published with the regime name and any available diagnostics (num_hypotheses=0 for no-evidence/no-convergence, best_cost for others).
- The system continues operating on odometry alone until a future frame yields `TRACKING` with a passing gate.

### 12a. Ambiguity Classification (when hypotheses survive)

```python
costs = sorted([h.cost for h in surviving_hypotheses])
if len(costs) == 1:
    regime = TRACKING
elif (costs[1] - costs[0]) / max(costs[0], epsilon) > ambiguity_cost_ratio:
    regime = TRACKING
else:
    regime = AMBIGUOUS
```

Default `ambiguity_cost_ratio`: 0.5 (the best must be at least 50% cheaper than runner-up in relative terms).

### 12b. Consecutive-Tracking Gate

After a period of non-`TRACKING` regimes (`AMBIGUOUS`, `NO_EVIDENCE`, `NO_CONVERGENCE`), the system requires `min_consecutive_tracking` (default: 3) consecutive `TRACKING` frames with consistent best-hypothesis pose (within `merge_distance`) before the first EKF update is accepted. This prevents a single spuriously dominant frame from committing the filter after a period of uncertainty.

During the "warming up" period (consecutive tracking count < threshold), the system behaves like `AMBIGUOUS`: skip update, inflate covariance.

Once the consecutive count is met, proceed to gating (S13) with the best hypothesis as `z` and fixed measurement covariance as `R_match`. Subsequent `TRACKING` frames update immediately (the counter only re-activates after a non-`TRACKING` frame).

---

## 13. Measurement Covariance, Degeneracy Detection, and Gating

### Measurement Covariance Strategy

For the MVP, `R_match` is a **fixed conservative covariance** with hard floors and ceilings. The Hessian is used only for degeneracy detection and diagnostic output, not for EKF update weighting.

**Rationale:** The cost function uses robust Huber loss over a clipped boundary distance field. The Hessian inverse from this objective is not a calibrated Gaussian covariance — it is a local curvature proxy. Using it directly in the EKF couples a poorly-understood quantity to the most safety-critical part of the pipeline. A fixed covariance is predictable and tunable on bag replay.

```python
# Fixed measurement covariance (MVP)
R_match = np.diag(match_covariance_stddev) ** 2   # default: [0.5, 0.5, 0.05]

# Hard bounds (always enforced, even if adaptive covariance is added later)
R_match = np.clip(R_match, R_match_floor, R_match_ceiling)
#   R_match_floor:   diag([0.01, 0.01, 0.001])²  — never trust a match more than this
#   R_match_ceiling: diag([2.0,  2.0,  0.2])²     — never distrust a match more than this
```

### Degeneracy Detection (from Hessian)

The Hessian from the best hypothesis is still computed and analysed for degeneracy:

```python
H_cost = J_T_J     # from best hypothesis final LM iteration
H_scaled = H_cost / max(n_points, 1)
eigenvalues, eigenvectors = np.linalg.eigh(H_scaled)

if np.min(eigenvalues) < min_eigenvalue_threshold:
    is_degenerate = True
    # Inflate R_match in degenerate eigendirection(s)
    degen_dirs = eigenvectors[:, eigenvalues < min_eigenvalue_threshold]
    for d in degen_dirs.T:
        R_match += degeneracy_inflation_factor * np.outer(d, d)
```

The Hessian eigenvalues are also published in the status topic for offline analysis.

### Mahalanobis Gating

```python
innovation = z - mu_pred
innovation[2] = wrap(innovation[2])
S = Sigma_pred + R_match   # H = I
mahal_sq = innovation @ np.linalg.inv(S) @ innovation
if mahal_sq > mahal_threshold**2:
    # reject: skip EKF update, keep mu_pred as output
```

Default `mahal_threshold`: 3.0.

### Open Issue: Adaptive Covariance (post-MVP)

Once bag replay with ground truth is available, the fixed covariance can be replaced with a Hessian-derived adaptive covariance if validation shows the Hessian eigenvalues correlate monotonically with actual pose error. Strategies:

1. **Scaled Hessian inverse** — `R_match = scale_factor * inv(H_floored)`, where `scale_factor` is empirically calibrated.
2. **Hybrid** — Use Hessian for directional shape but fix the overall magnitude to a calibrated scalar.

The hard floors and ceilings remain in either case as safety bounds.

---

## 14. Publishing

On every callback (whether or not the map-match update was accepted):

**Pose** (`/bruce/slam/slam/pose`, `PoseWithCovarianceStamped`):
- `header.stamp` = feature message stamp
- `header.frame_id` = `"map"`
- `pose.pose` = EKF mean `mu` as 3D pose (z=0, roll=0, pitch=0)
- `pose.covariance` = 6x6 matrix with `[x, y, theta]` block from `Sigma`; off-diagonal DOF set to `1e-4 * I`

**Odometry** (`/bruce/slam/slam/odom`, `Odometry`):
- `header` = same as pose
- `child_frame_id` = `"base_link"`
- `pose.pose` = same as above
- `twist.twist` = pass-through from current odometry message

**TF** `map` to `odom`:
- `T_map_odom = mu_map_base_link * T_odom_base_link^-1`
- `header.stamp` = feature message stamp

**Trajectory** (`/bruce/slam/slam/traj`, `PointCloud2`):
- Append EKF mean to trajectory list on **every callback** (not just accepted updates)
- Publish full list as 3D point cloud (z = 0)
- This preserves continuous-tracking semantics for existing consumers

**Accumulated cloud** (`/bruce/slam/localisation/accumulated_cloud`, `PointCloud2`):
- Published on every matching step (skipped during `NO_EVIDENCE` regime)
- The motion-compensated, spatially filtered cloud actually used for scoring, transformed into `map` frame
- Useful for visually verifying that accumulated evidence aligns with boundary markers

**Prior map** (`/bruce/slam/map/prior`, `nav_msgs/OccupancyGrid`):
- Published once at startup (QoS: transient local)

**Boundaries** (`/bruce/slam/map/boundaries`, `visualization_msgs/MarkerArray`):
- Published once at startup (QoS: transient local)
- One `LINE_LIST` marker per pile, showing boundary segments

**Hypotheses** (`/bruce/slam/localisation/hypotheses`, `geometry_msgs/PoseArray`):
- Published on every matching step; shows all surviving hypotheses before ambiguity filtering
- Useful for tuning and debugging multi-hypothesis behaviour

**Status** (`/bruce/slam/localisation/status`, `diagnostic_msgs/DiagnosticStatus`):
- Published every callback
- Fields: regime (tracking/ambiguous), num_hypotheses, best_cost, gate_passed, is_degenerate

---

## 15. New Node: `PileLocalisationNode`

### Class Structure
```
PileLocalisation                    # pure logic, no ROS deps
  load_map(image_path, yaml_path)
    → extract_pile_centres(binary_grid)
    → build_boundary_segments(centres, side_length)
    → compute_boundary_distance_field(segments)
  predict(delta_pose)
  accumulate(points, odom_pose)
  generate_candidates(mu_pred) → List[pose]
  refine_candidate(pose, cloud) → (pose, cost, JtJ)
  match_to_map(cloud) → List[Hypothesis]
  assess_ambiguity(hypotheses) → (regime, best_hypothesis)
  hessian_to_covariance(JtJ, n_points) → (R_match, is_degenerate)
  gate(z, mu_pred, Sigma_pred, R_match) → bool
  update(z, R_match)

PileLocalisationNode(PileLocalisation, Node)   # ROS2 wrapper
  Subscriptions: SONAR_FEATURE_TOPIC, LOCALIZATION_ODOM_TOPIC, /initialpose
  Synchroniser: ApproximateTimeSynchronizer (queue=20, max_delay=0.5s)
  Publishers: SLAM_POSE_TOPIC, SLAM_ODOM_TOPIC, SLAM_TRAJ_TOPIC,
              /bruce/slam/map/prior, /bruce/slam/map/boundaries,
              /bruce/slam/localisation/accumulated_cloud,
              /bruce/slam/localisation/hypotheses, /bruce/slam/localisation/status
  TF broadcaster
```

### New script: `bruce_slam/scripts/pile_localisation_node.py`
### New map metadata: `bruce_slam/config/map.yaml` (standard ROS map_server format)
### New config: `bruce_slam/config/pile_localisation.yaml`

```yaml
"/**":
  ros__parameters:
    # Map — loaded from standard ROS map_server YAML
    map_yaml: "config/map.yaml"
    dt_max_dist: 5.0                  # BDT saturation (metres)

    # Pile geometry (geometric invariant — confirm from blueprint)
    pile_side_length: !!UNRESOLVED!!  # metres — MUST BE CONFIRMED before implementation
    pile_area_tolerance_low: 0.25     # accept components >= 25% of expected pile area
    pile_area_tolerance_high: 4.0     # accept components <= 400% of expected pile area

    # Feature accumulation
    accumulation_window: 5
    min_point_spacing: 0.3            # uniform spatial filter min distance (metres)
    min_evidence_points: 15           # skip matching if fewer points

    # Cost function
    huber_delta: 1.0                  # Huber loss threshold (metres)
    max_iterations: 20
    convergence_tol: 1.0e-4

    # LM solver
    lm_lambda_init: 0.01
    lm_lambda_factor: 10.0

    # Multi-hypothesis search
    search_radius: 2.0               # metres around mu_pred
    search_step: 1.0                 # grid spacing (metres)
    search_angle: 0.175              # angular offset (rad, ~10 deg)
    merge_distance: 0.2              # merge hypotheses closer than this (metres)
    merge_angle: 0.05                # merge hypotheses closer than this (rad)
    max_cost_ratio: 3.0              # prune hypotheses with cost > ratio * best

    # Ambiguity
    ambiguity_cost_ratio: 0.5        # best must be this fraction cheaper than runner-up
    ambiguity_inflation_factor: 2.0  # covariance inflation in ambiguous regime
    min_consecutive_tracking: 3      # require N consistent TRACKING frames after ambiguity

    # Measurement covariance (fixed for MVP)
    match_covariance_stddev: [0.5, 0.5, 0.05]   # fixed R_match std devs [x, y, theta]
    R_match_floor_stddev: [0.01, 0.01, 0.001]   # min trust bound
    R_match_ceiling_stddev: [2.0, 2.0, 0.2]     # max distrust bound

    # Degeneracy detection (from Hessian)
    min_eigenvalue_threshold: 0.1    # below this → degenerate
    degeneracy_inflation_factor: 100.0

    # EKF
    mahal_threshold: 3.0
    odom_process_noise_stddev: [0.05, 0.05, 0.01]
    initial_pose: [0.0, 0.0, 0.0]
    initial_covariance_stddev: [2.0, 2.0, 0.175]    # ~2m, ~10deg
```

---

## 16. Dependencies

Add to `bruce_slam/package.xml`:
```xml
<exec_depend>python3-numpy</exec_depend>
<exec_depend>python3-scipy</exec_depend>
<exec_depend>python3-opencv</exec_depend>
<exec_depend>diagnostic_msgs</exec_depend>
```

`diagnostic_msgs` is required for the `/bruce/slam/localisation/status` topic. The other three are standard in ROS2 Python workspaces but must be declared explicitly.

---

## 17. Testing Plan

1. **Unit: pile extraction** — Load test PNG with known pile layout. Verify extracted centres match expected world coordinates within `resolution/2`. Verify estimated side length matches ground truth.
2. **Unit: boundary distance field** — Verify BDT is zero at known boundary pixel locations, increases monotonically away from boundaries, saturates at `dt_max_dist`. Verify BDT is *not* zero at pile interior centres (unlike an occupancy DT).
3. **Unit: cost function** — Synthetic cloud with points placed exactly on pile boundaries → cost approximately 0. Deliberately offset cloud → cost higher. Random clutter points added → solution still converges (Huber robustness).
4. **Unit: multi-hypothesis search** — Construct a scenario with two nearby piles of identical geometry. Verify that two distinct hypotheses survive pruning. Verify ambiguity regime is triggered.
5. **Unit: ambiguity assessment** — Single clear match → tracking regime. Two equal-cost matches → ambiguous regime, no EKF update. Best-cost match with large gap to runner-up → tracking regime.
6. **Unit: EKF** — Predict grows covariance; update shrinks it. Gating correctly rejects a 5-sigma outlier. Ambiguity inflation increases covariance.
7. **Unit: Hessian to covariance** — Degenerate input (collinear points) → eigenvalue flooring fires, covariance finite.
8. **Integration (bag replay):** Run full pipeline against `testing_data/dvl_fallback_0.2trans_0.4rot_fixed/` bag. Plot localisation error vs ground truth.
9. **Robustness:** Inject synthetic random false-positive points at 50% of total count. Verify pose error degrades gracefully.
10. **Ambiguity robustness:** Replay bag with artificially increased initial pose offset (~1.5x pile spacing). Verify the system defers corrections rather than locking onto the wrong pile.
11. **Regime coverage:** On bag replay, verify all five regimes (`TRACKING`, `AMBIGUOUS`, `NO_EVIDENCE`, `NO_CONVERGENCE`, `GATE_REJECT`) are reachable and produce correct status output. Inject conditions to trigger each if they don't occur naturally.
12. **Covariance calibration (experimental):** On bag replay with ground truth, scatter-plot `R_match` eigenvalues against actual pose error magnitude per axis. Assess whether the heuristic covariance is at least monotonically correlated with real error. Document findings — this is validation, not a pass/fail gate.

---

## 18. Comparison: Occupancy-Grid vs Boundary-Centric

| Aspect | Occupancy-Grid (previous) | Boundary-Centric (this PRD) |
|---|---|---|
| Distance field target | Nearest occupied pixel (pile interior) | Nearest boundary segment (pile surface) |
| `r_pile` subtraction | Required to compensate for interior-vs-surface mismatch | Not needed; field is already surface-referenced |
| Hypothesis count | Single LM from odometry prior | Multi-hypothesis grid search + per-candidate LM |
| Ambiguity handling | None (Mahalanobis gate only) | Explicit regime classification; reject/defer in ambiguous case |
| Covariance under ambiguity | Unchanged | Inflated to reflect unresolved uncertainty |
| Map representation | Raw occupancy grid + DT | Extracted pile centres + continuous boundary segments + BDT |
| Semantic match to sonar | Poor (sonar sees surfaces, DT rewards interiors) | Good (BDT rewards proximity to surfaces) |

---

## 19. Open Issues

These are known weaknesses that require experimental validation or design iteration. They are not implementation blockers, but they should be actively monitored during development.

### 19a. Frontend-to-boundary compatibility is unvalidated

The backend assumes frontend points land near pile boundaries. The frontend actually emits blob representatives (strongest-intensity pixel or centroid per CFAR cluster). Whether these consistently land near true visible boundaries is unknown.

**Status:** A dedicated evaluation experiment is planned (see `.specs/frontend_boundary_evaluation.md`). This should be completed before or during early backend implementation.

**Fix strategies if the assumption is wrong:**
1. **Bias correction** — If points are consistently offset from boundaries (e.g. 0.3m inside), add a learned or configured offset to the cost function, similar to the old `r_pile` subtraction but empirically grounded.
2. **Frontend adjustment** — Change `cluster_repr` from `"max"` to `"centroid"`, or emit multiple points per blob (e.g. blob boundary pixels), or emit the nearest-to-sensor edge of each blob. Small frontend changes may be the highest-leverage fix.
3. **Wider Huber threshold** — If the offset is noisy but bounded, increase `huber_delta` to tolerate it. Least desirable — weakens outlier rejection.

### 19b. Scoring ignores visibility and occlusion

The BDT cost rewards proximity to any pile boundary, including back-facing and occluded faces. In sparse observations, this can over-credit wrong-pile hypotheses.

**Fix strategies (phase 2):**
1. **Front-face filter** — For each candidate pose, only score against boundary segments whose normal faces the sensor (dot product check). Cheap, catches the worst cases.
2. **Per-point support count** — Weight by how many nearby boundary segments are geometrically visible, not just nearest distance.
3. **Full ray-cast model** — Score against expected visible returns from each pose. Most accurate, most expensive.

### 19c. No recovery from prolonged degradation

If odometry drifts beyond the search radius, the system stays in `AMBIGUOUS`/`NO_EVIDENCE` indefinitely with no recovery path.

**Fix strategies (phase 2):**
1. **Escalation policy** — After N consecutive non-`TRACKING` frames, widen `search_radius` and `search_step`. After M frames, trigger a wider re-acquisition search.
2. **Covariance-based widening** — Automatically grow search radius proportional to EKF covariance magnitude.
3. **Hierarchical search** — Coarse discrete neighbourhood identification first, then fine local optimisation.

### 19d. Accumulation window may amplify odom errors

If short-term odometry is inaccurate (DVL dropout, timing jitter), motion-compensated accumulation can create sharper but incorrect clouds.

**Fix strategies (phase 2):**
1. **Baseline comparison** — Test window=1 vs window=5 on bag replay. If single-frame works comparably, accumulation may be optional.
2. **Adaptive windowing** — Shrink window during high vehicle motion or when odom quality is suspect.
3. **Quality check** — Reject accumulated clouds where inter-frame alignment residuals are high.

---

## 20. Future Work (Explicitly Deferred)

> Note: Items in S19 (Open Issues) may graduate to this section if they are resolved or deprioritised.

These are known limitations deferred to a later phase. They should not influence the current implementation.

| Item | Reason for deferral |
|---|---|
| **Cylindrical pile geometry** | Current simulation uses square pylons. When cylindrical piles are needed, boundary segments become circular arcs; the BDT computation and cost function change but the architecture does not. |
| **2D projection correction for pitch/roll** | Same as occupancy-grid PRD — requires IMU beam geometry, deferred until basic localisation validated. |
| **Multi-resolution BDT (coarse-to-fine)** | Would widen convergence basin beyond current ~2m assumption. Not needed given known initial pose quality. |
| **Particle filter for global relocalisation** | The multi-hypothesis search is local only. A particle filter could bootstrap from unknown initial pose but adds significant complexity. |
| **Divergence recovery / adaptive EKF reset** | If Mahalanobis gate rejects too many consecutive frames, inflate covariance and attempt recovery. Adds resilience after tracking loss. |
| **Learned sonar feature matching** | Potential replacement for CFAR front-end; out of scope for this phase. |
| **Explicit view-dependent observation model** | Modelling which pile faces are visible from a given pose would improve scoring but requires sonar beam geometry integration. |

---

## 21. Out of Scope

- Global localisation / particle filter (initial pose assumed within ~2 m, ~10 deg)
- Online map updates (prior grid is static)
- Loop closure / non-sequential scan matching
- The mapping node (`mapping.py`) — retained separately for visualisation
- Front-end changes (CFAR feature extraction, Kalman dead-reckoning)
- `rov_id` multi-robot namespacing
