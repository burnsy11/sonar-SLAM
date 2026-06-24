# Memory

## Project Overview

This is an underwater sonar SLAM system (forked from Bruce SLAM) for localising a ROV in a known pile field (jetty inspection). The project is transitioning from a GTSAM pose-graph SLAM backend to a **boundary-centric, ambiguity-aware map-based localisation** backend. The frontend (CFAR feature extraction, Kalman DVL+IMU dead-reckoning) is unchanged.

## Durable Context

- **Environment:** Structured pile fields — repetitive rectangular piles in the current simulator, with cylindrical piles expected in later real deployments. The earlier square-pylon simulation assumption was incorrect. [high]
- **Sonar returns are boundary observations, not interior detections.** Sonar sees pile surfaces, not filled regions. Any map representation must respect this. This is the core insight that drove the shift from occupancy-grid to boundary-centric matching. [high]
- **Pile extraction from sonar is unreliable.** CFAR produces noisy, partial, cluttered point clouds with false positives. The system cannot assume clean pile-centre detections. [high]
- **Initial pose is known to ~2 m, ~10 deg.** Global relocalisation is explicitly out of scope for phase one; local tracking only. [high]
- **External ROS2 interface hard contract is scoped to pose/odom/TF only.** `/bruce/slam/slam/pose`, `/bruce/slam/slam/odom`, and `map->odom` TF must be preserved exactly. Legacy SLAM-only topics (`slam/cloud`, `slam/constraint`) are retired. Visualisation/debug topics are new and backend-specific. [high]
- **User wants strong RViz debug visibility:** accumulated cloud, boundary overlay, hypotheses, regime status, and the prior map must all be inspectable in RViz. Do not optimise away debug topics. [high]
- **The real mission is pile inspection, not just localisation.** Downstream consumer is a pile inspection controller that executes vertical sweeps around a target pile at a standoff distance. Localisation quality matters insofar as it enables reliable pile-relative navigation. (durable fact) [high]
- **Occupancy map output is expected for planner integration (NAV2).** The prior grid published on `/bruce/slam/map/prior` is intended to feed an existing planner, not just RViz visualisation. (durable fact) [medium]
- **Map metadata (resolution, origin) is a geometric invariant, not a tuning parameter.** Must be confirmed from the jetty blueprint before implementation. Uses standard ROS `map_server` YAML sidecar format. [high]
- **Coordinate convention:** PNG vertically flipped after load so row 0 = minimum y. Origin = bottom-left pixel in world frame. [medium]
- **Pile centres plus rectangular width/height are geometric invariants** extracted from the blueprint/occupancy grid. The old single `pile_side_length` assumption is stale for the simulator map; use independent width and height instead. [high]

## Key Insights

- **Semantic mismatch was the root problem.** The occupancy-grid DT measures distance to solid interior pixels. Sonar returns land on surfaces. Subtracting `r_pile` was a patch; using a boundary distance field eliminates the mismatch entirely. (validated lesson) [high]
- **Repetitive geometry is the central design challenge, not noise.** Noise is handled by Huber loss and accumulation. Wrong-pile corrections from ambiguous multi-hypothesis situations are the real failure mode. (validated lesson) [high]
- **Multi-hypothesis search + ambiguity gating is the architectural response.** Rather than trusting a single LM optimiser from one initial guess, evaluate a grid of candidates, prune, and classify into tracking vs ambiguous regime before committing. (promising idea) [high]
- **Accumulation helps, but short windows only.** Multi-frame accumulation cancels spatially random false positives, but long windows blur geometry due to motion compensation error. Default window = 5 frames. (working assumption) [medium]
- **Odometry is not just an initialiser — it is the persistent motion prior.** Map corrections only happen when evidence is strong. Between corrections, odometry carries the state. (durable fact) [high]

## Lessons Learned

- **The first PRD (occupancy-grid) was algorithmically sound but underspecified on integration details.** A review found: launch plan inconsistency, loose interface preservation language, unresolved map metadata, underspecified EKF deltas and covariance conversion, and undeclared dependencies. The revised PRD (pile localisation) addressed all of these. (validated lesson) [medium]
- **Brainstorming phase was valuable for narrowing the design space.** Four backend options were considered (direct scan-to-map, feature-to-feature, probabilistic, hybrid coarse-to-fine). The final design is closest to hybrid coarse-to-fine: multi-hypothesis local search with per-candidate LM refinement. (validated lesson) [medium]
- **User feedback: "it is impossible to exactly extract piles from sonar data."** This killed the explicit pile-landmark approach (Option B) early and anchored the design on treating features as anonymous boundary evidence. (validated lesson) [high]
- **Step 1 of the frontend-boundary evaluation is now complete.** Running the recorder on `testing_data/dvl_fallback_0.2trans_0.4rot_fixed/` produced `evaluation/recorded_raw_cfar.pkl` with 4016 synced frames, 4010 non-empty frames, and 6 empty frames. (validated lesson) [high]
- **Simulator geometry correction:** the piles in the simulator map are rectangular with unequal side lengths, not square. Step 2 of the frontend-boundary evaluation must therefore extract rectangular boundaries and a rectangular BDT from the occupancy grid. Any square-boundary wording in older specs is stale. (validated lesson) [high]
- **Step 2 of the frontend-boundary evaluation is now complete on the updated fixed map.** Using the cleaned high-resolution `gt_occupancy_grid.png` / `gt_occupancy_grid.yaml` (0.01 m/pixel, piles only), the offline extractor produced `evaluation/boundary_map_rectangular.npz`, `evaluation/boundary_map_rectangular_summary.json`, and `evaluation/boundary_map_rectangular_preview.png`. The extracted map contains 62 piles with estimated global dimensions 0.1481 m x 0.1076 m and minimum spacing 0.7184 m. (validated lesson) [high]
- **The updated fixed GT map supersedes the earlier lower-resolution map for Step 2 work.** The older map had extra non-pile features; the new one is piles-only and materially simplifies connected-component extraction. (durable fact) [high]
- **Step 3 of the frontend-boundary evaluation is now complete, and the result is negative for the current measurement-model assumption.** Running the analysis on `evaluation/recorded_raw_cfar.pkl` against `evaluation/boundary_map_rectangular.npz` produced a broad, multimodal boundary-distance distribution with median 1.029 m, mean 1.101 m, std 0.541 m, and p90 1.904 m across 9,927,576 in-bounds points. The expected near-zero mode is absent. (validated lesson) [high]
- **The Step 3 mismatch is not mainly an out-of-bounds issue.** Only 6,257 of 9,933,833 feature points fell outside the map bounds, so the poor compatibility result is driven by in-bounds geometry mismatch rather than points missing the map entirely. (validated lesson) [high]
- **The Step 3 spatial plot shows structured arcs and bands rather than boundary-hugging returns.** This points toward either a frame/map alignment problem, a systematic range/bias issue, or a deeper mismatch between raw CFAR outputs and the rectangle-boundary measurement model. (inference from evaluation outputs) [high]
- **An offline animation visualizer now exists for the evaluation artifacts.** `evaluation/animate_frontend_boundary.py` renders the extracted rectangular map, GT trajectory, current pose, and projected raw CFAR points directly from the offline pickle/NPZ artifacts. It now supports extension-based export so `.avi` uses MJPG and `.mp4` uses `mp4v`. The preferred compatibility artifact is currently `evaluation/results_rectangular/frontend_boundary_animation_mjpg.avi`. (durable fact) [medium]
- **Debugging the visualization mismatch found a dominant constant world-frame translation error.** Fitting the projected raw CFAR points against the extracted map yields an empirical correction of approximately **(+0.45 m, +1.00 m)** in `(x, y)`. This is much more consistent with a map-origin / world-to-map registration problem than with skipped frames or a local point-frame rotation bug. (validated lesson) [high]
- **User clarification on map generation materially sharpens the root-cause hypothesis.** The occupancy grid came from the Isaac Sim Occupancy Grid plugin, and the user set the map origin to the robot start location. That strongly suggests the exported YAML origin may not mean “bottom-left pixel in world coordinates” in the standard ROS `map_server` sense, which would explain why treating it as lower-left in the evaluation pipeline introduces a fixed translation. (high-confidence inference) [high]
- **Cross-repo `oceansim` debugging did not find evidence that `/oceansim/robot/gt_pose` is wrong at the publisher.** The report found that the publisher uses the robot's Isaac Sim world transform directly, labels it as `map`, and also publishes `map -> base_link` TF with the same values. That pushes the suspicion away from GT-pose publication and toward map export / origin semantics. (validated lesson from external report) [high]
- **The `oceansim` report's strongest sim-side hypothesis is map-image/YAML registration mismatch, not a GT frame bug.** It also noted that Oceansim-side navigation utilities interpret YAML `origin` as the bottom-left world corner with image-row inversion for `+y`, which is compatible with standard `map_server` semantics but may differ from the Isaac occupancy-grid plugin's export semantics. (validated lesson from external report) [high]
- **A hidden static frame bridge was not supported by the `oceansim` report.** The bag metadata showed `/tf` but no obvious `/tf_static` clue, and the report did not find an extra world-to-map transform in the inspected sim-side code path. (medium-confidence external finding) [medium]
- **An alignment estimator now exists for the offline evaluation.** `evaluation/estimate_boundary_alignment.py` searches for the best global XY translation between projected points and the boundary map and writes `evaluation/results_rectangular/alignment_estimate.json`. (durable fact) [medium]
- **The aligned rerun improves Step 3 substantially but does not fully solve the mismatch.** Applying the fitted offset drops the full-dataset median boundary distance from 1.029 m to 0.463 m and reduces out-of-bounds points from 6257 to 446, but the distribution is still not a tight near-zero mode. (validated lesson) [high]
- **A corrected aligned animation now exists.** `evaluation/results_rectangular_aligned/frontend_boundary_animation_mjpg_aligned.avi` is the preferred visualization artifact for inspecting the corrected overlay. (durable fact) [medium]
- **Changing the YAML origin to the world-frame origin did not eliminate the fitted translation.** After rebuilding the map with `origin: [-0.995, -4.995, 0.0]`, the raw unaligned median distance improved to 0.519 m, but the best-fit alignment offset still remained approximately `(+0.45 m, +1.00 m)`. The origin edit alone is therefore not the whole fix. (validated lesson) [high]
- **A residual non-rigid mismatch is now suspected.** The user observed that the sonar overlay appears to warp/distort slightly as the robot moves, and the error changes somewhat over time instead of behaving like a purely rigid offset. That raises the likelihood of a sonar rendering, remapping, or vertical-FOV / 3D-projection issue. (working hypothesis) [high]
- **The next debugging step should span both repositories.** The remaining issue likely sits across the boundary between simulator-side sonar/map generation and `sonar-SLAM` frontend interpretation, so the next agent should have access to both this repo and the pile simulation repo. (durable fact) [high]
- **ROS 2 `PointCloud2` conversion cannot assume `read_points()` returns a dense 2-D ndarray.** On this setup it yielded a 1-D structured array, which crashed the recorder until `r2n()` switched to `read_points_numpy()`. Reuse that path for future PointCloud2-to-numpy work. (validated lesson) [high]
- **The frontend boundary evaluation launch must not loop the bag for Step 1c.** A looping bag (`ros2 bag play -l`) prevents natural completion and can interrupt pickle saving during forced shutdown. Single-pass playback is the correct evaluation mode. (validated lesson) [high]

## Decisions and Working Assumptions

- **Phase one should use rectangular pile geometry in the simulator.** The boundary model is still boundary-centric, but the geometry primitive is axis-aligned rectangles with separate width and height rather than square segments from a single side length. (durable fact) [high]
- **No backend selector parameter.** The new node is a replacement, not an optional mode. Old nodes retained in repo but not launched. (durable fact) [medium]
- **EKF state is `[x, y, theta]` in map frame.** 3-DOF, 2D. No pitch/roll correction (deferred). (working assumption) [medium]
- **Five runtime regimes define all fallback behaviour:** `TRACKING`, `AMBIGUOUS`, `NO_EVIDENCE`, `NO_CONVERGENCE`, `GATE_REJECT`. Each has defined EKF action, covariance handling, and status output. Conservative by design — prefer drift over wrong-pile lock. (durable fact) [high]
- **Consecutive-tracking gate:** After a non-tracking period, require N consecutive consistent `TRACKING` frames before the first EKF update is accepted. Prevents single-frame spurious commits. Default N=3. (durable fact) [high]
- **Fixed measurement covariance for MVP.** `R_match` is a fixed diagonal matrix with hard floors/ceilings, not derived from Hessian. Hessian used only for degeneracy detection and diagnostics. Adaptive covariance deferred until empirical validation. (durable fact) [high]
- **Mahalanobis gate at 3 sigma.** Rejects corrections inconsistent with prediction. (working assumption) [low]
- **LM solver replaces Gauss-Newton.** DT bilinear interpolation is piecewise-linear; GN oscillates at discontinuities. (durable fact) [medium]

## Risks and Caveats

- **The current raw-CFAR-to-boundary compatibility assumption did not validate on the updated simulator map.** Step 3 showed a broad multimodal distribution centered far from zero, so the backend should not proceed as if "frontend points land on pile boundaries" has already been established. (risk) [high]
- **The unaligned Step 3 result should not be treated as authoritative by itself.** A world-frame registration bug inflated the original distances; use the aligned rerun as the current baseline when reasoning about measurement compatibility. (risk / caveat) [high]
- **Least likely current explanation: a broken `/oceansim/robot/gt_pose` publisher frame.** The external report found that code path to be internally consistent; remaining effort should focus first on map export semantics and sonar/image distortion. (risk prioritisation) [medium]
- **Ambiguity appears quickly as odometry drift approaches pile spacing.** If odometry noise is larger than expected, the system may spend most of its time in ambiguous regime and rarely correct. (risk) [high]
- **Map/world mismatch.** If the prior map has systematic errors, the boundary representation inherits them. No online map correction is planned. (risk) [medium]
- **The boundary distance field must be precomputed correctly.** Unlike the occupancy DT (distance to nearest occupied pixel), the BDT must measure distance to rasterised pile boundaries; for the current simulator that means rectangular boundaries with independent width/height, not square segments. An implementation error here silently breaks everything. (caveat) [medium]
- **Short-window motion compensation assumes odometry is locally accurate.** If DVL dropouts or IMU drift corrupt short-term odom, accumulated clouds will be misaligned. (risk) [medium]
- **R_match is fixed for MVP, not Hessian-derived.** Decision to decouple Hessian from EKF update until empirical validation. Hessian still used for degeneracy detection. (durable fact, replaces earlier caveat) [medium]
- **Frontend feature points may not land on pile boundaries.** CFAR clusters emit blob representatives (strongest pixel or centroid), not boundary traces. The BDT cost assumes boundary proximity. This is the single most important experimental risk. A dedicated evaluation experiment is defined in `.specs/frontend_boundary_evaluation.md`. (risk) [high]
- **Raw CFAR points on the current bag do not cluster near the extracted pile boundaries.** This is now an observed Step 3 result, not just a hypothetical risk. The next task is to determine whether the cause is map/frame alignment, systematic bias, or an invalid measurement abstraction. (validated lesson / risk) [high]
- **After correcting the global XY offset, residual mismatch still remains.** The next debugging target is no longer “is there a gross translation error?” but rather whether the remaining error comes from range bias, pile geometry abstraction, or frontend measurement semantics. (validated lesson / risk) [high]
- **Residual warp means a rigid-offset-only fix is probably insufficient.** The remaining mismatch may involve simulator sonar rendering, frontend Cartesian remapping, or vertical-FOV effects, not just map registration. (risk) [high]
- **Scoring ignores visibility/occlusion.** BDT rewards proximity to any boundary, including back-facing and occluded faces. Can over-credit wrong-pile hypotheses. Deferred to phase 2; front-face filter is the simplest fix. (risk) [medium]
- **No recovery from prolonged degradation.** If odom drifts beyond search radius, system stays in non-tracking regimes indefinitely. Escalation policy deferred to phase 2. (risk) [medium]

## Open Questions

- **`pile_side_length` is still `!!UNRESOLVED!!`.** Must be confirmed from the jetty blueprint before implementation. [high]
- **`map.yaml` resolution and origin are still marked CONFIRM.** Same blocker. [high]
- **How quickly does ambiguity degrade tracking in practice?** Needs experimental validation with the actual bag data. [high]
- **Why are Step 3 boundary distances so large despite using GT pose and a cleaned piles-only map?** Need targeted checks of frame conventions, map origin/orientation, and whether the frontend points represent a different geometric quantity than the extracted rectangles. [high]
- **Is the fitted `(+0.45 m, +1.00 m)` offset the true map-origin correction?** It behaves like one, but the user-supplied `gt_occupancy_grid.yaml` has not yet been formally edited to bake that correction into the map metadata. [high]
- **Is there a simulator-side sonar warp or vertical-FOV projection effect?** The overlay appears to deform over motion, suggesting a possible non-rigid error in sonar rendering or in how the frontend reads/remaps the sonar image. [high]
- **What exactly does the Isaac Occupancy Grid plugin mean by “origin”?** This remains the highest-value missing fact. The current evidence says that assuming standard ROS lower-left semantics for the exported YAML may be wrong. [high]
- **Does the confidence/ambiguity signal reliably separate correct matches from wrong-but-plausible ones?** The `ambiguity_cost_ratio` threshold (default 0.5) is a guess. [medium]
- **When should adaptive (Hessian-derived) covariance replace fixed covariance?** Only after bag replay shows Hessian eigenvalues correlate with actual error. [medium]
- **How tolerant is the method to discretisation artifacts in the original occupancy grid?** Extracted pile centres and boundaries inherit grid quantisation error. [medium]
- **Which spec documents still assume square simulator piles?** `prd_pile_localisation.md` and `pile_localisation_spec.md` still contain stale square-pile wording and should be updated before backend implementation hardens around the wrong geometry. [medium]

## What Changed Over Time

- **Occupancy-grid DT approach -> boundary-centric BDT approach.** Motivated by the semantic mismatch insight: sonar sees surfaces, not interiors. The `r_pile` subtraction in the old cost function was a symptom. [high]
- **Single-hypothesis LM -> multi-hypothesis local search.** Motivated by the realisation that repetitive pile geometry is not an edge case but the central operating condition. [high]
- **"Replace backend" -> "replace backend with explicit ambiguity handling."** The brainstorm identified ambiguity as a first-class problem. The first PRD (occupancy-grid) had only Mahalanobis gating. The new PRD adds regime classification and covariance inflation. [high]
- **Feature-to-feature / pile-landmark approach was considered and rejected.** User confirmed pile extraction from sonar is unreliable. Anonymous boundary evidence is the right abstraction level. [medium]
- **PRD v2 review tightened five areas:** (1) scoped interface contract to pose/odom/TF only, (2) added five explicit runtime regimes for all fallback cases, (3) separated geometric invariants from tuning params, (4) added `diagnostic_msgs` dependency, (5) labelled R_match as heuristic with experimental validation required. [medium]
- **PRD v3 review (deep stress-test) led to four MVP changes:** (1) fixed measurement covariance instead of Hessian-derived, (2) consecutive-tracking gate for temporal consistency, (3) startup map validation, (4) open issues section with fix strategies. Also created frontend-boundary evaluation plan. Deferred: visibility-aware scoring, recovery policy, adaptive accumulation, target-relative architecture. [medium]

## Future-Agent Onboarding Notes

1. **Read the PRD first:** `.specs/prd_pile_localisation.md` is the current authoritative implementation spec. Pay attention to S19 (Open Issues) for known risks.
0. **Run the frontend evaluation first:** `.specs/frontend_boundary_evaluation.md` describes the experiment that must validate the core measurement assumption before or during early backend work.
2. **The frontend is unchanged.** Do not modify CFAR, feature extraction, or the Kalman node unless explicitly asked.
3. **Hard interface contract = pose/odom/TF only.** See PRD S2a. Legacy SLAM topics are retired; new debug topics are defined in S3.
4. **Map geometry metadata is still important.** `resolution` and `origin` in `map.yaml` still matter, but the simulator map should now be treated as rectangular-pile geometry with separate width and height; do not keep propagating `pile_side_length` as the primary invariant for this dataset.
5. **The core logic class (`PileLocalisation`) must be ROS-free.** ROS wrapper (`PileLocalisationNode`) handles subscriptions and publishing. This separation is by design.
6. **Test with:** `testing_data/dvl_fallback_0.2trans_0.4rot_fixed/` bag file. It contains `/oceansim/robot/gt_pose` (PoseStamped, 10062 msgs) for ground truth.
7. **Step 1 output already exists:** `evaluation/recorded_raw_cfar.pkl` is the current raw-CFAR + GT dataset from the fixed bag replay.
8. **Step 2 outputs now exist:** `evaluation/boundary_map_rectangular.npz`, `evaluation/boundary_map_rectangular_summary.json`, and `evaluation/boundary_map_rectangular_preview.png` were generated from the updated high-resolution piles-only GT map.
9. **Step 3 outputs now exist:** `evaluation/results_rectangular/summary.json`, `evaluation/results_rectangular/frame_stats.json`, `evaluation/results_rectangular/boundary_distance_histogram.png`, `evaluation/results_rectangular/boundary_distance_timeseries.png`, and `evaluation/results_rectangular/boundary_distance_spatial.png`.
10. **Alignment debugging outputs now exist:** `evaluation/results_rectangular/alignment_estimate.json` and the corrected rerun directory `evaluation/results_rectangular_aligned/`.
11. **Current-origin rerun outputs now exist:** `evaluation/results_rectangular_current_origin/` contains the rerun after the user changed `gt_occupancy_grid.yaml` origin to the world-frame origin; it did not remove the fitted `(+0.45 m, +1.00 m)` offset.
12. **Visualization artifacts now exist:** `evaluation/results_rectangular/frontend_boundary_animation.mp4`, `evaluation/results_rectangular/frontend_boundary_animation_mjpg.avi`, `evaluation/results_rectangular_aligned/frontend_boundary_animation_mjpg_aligned.avi`, and `evaluation/results_rectangular_current_origin/frontend_boundary_animation_mjpg_current_origin.avi`.
13. **Evaluation launch detail:** `bruce_slam/launch/frontend_boundary_eval_launch.py` should run the bag once, not loop it.
14. **PointCloud2 conversion detail:** Prefer the current `r2n(PointCloud2)` implementation using `read_points_numpy()`; the previous reshape logic was not robust on this ROS 2 setup.
15. **Old SLAM code is retained but not launched.** `slam.py`, `slam_ros.py`, `slam_node.py` stay in repo.
16. **Build:** `colcon build --packages-up-to bruce_slam && source install/setup.bash`
17. **Run:** `ros2 launch bruce_slam test_launch.py`

## Source Coverage Notes

- **`.specs/pile_localisation_spec.md`** — High-level method spec; the conceptual "why" behind the boundary-centric approach. Most durable document.
- **`.specs/prd_pile_localisation.md`** — Implementation PRD; the "what and how". Authoritative for implementation decisions. Has been through three review rounds; all findings incorporated.
- **`.specs/frontend_boundary_evaluation.md`** — Evaluation plan for the core measurement model assumption. Includes recorder node, map processing module, and analysis script code.
- Superseded documents (removed from repo, context preserved in this memory): occupancy-grid PRD, brainstorm, user answers, and three PRD reviews.
