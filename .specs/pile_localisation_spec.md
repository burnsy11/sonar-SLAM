# High-Level Method Spec: Proposed Sonar Localization / SLAM Backend

## 1. Proposed System Summary

The proposed system is a prior-map-based localization backend for underwater sonar navigation in structured pile fields. It retains the existing sonar front-end and odometry pipeline, but changes how sonar observations are interpreted and matched to the map.

At a high level, the method treats sonar front-end outputs as noisy, partial, and cluttered evidence of nearby pile boundaries rather than as exact obstacle points or reliable pile-center detections. The backend performs local map matching against a boundary-centric prior representation, evaluates whether the match is locally informative and unambiguous, and only then fuses the result with odometry.

The main shift from the old approach to the new one is:

* from direct alignment of accumulated feature points to a raw occupancy-grid distance field
* to boundary-centric, probabilistic, ambiguity-aware local tracking against a prior structural map

This is intended to better match expected sonar observations, improve robustness to false positives and partial visibility, and avoid overconfident updates in repetitive pile layouts.

---

## 2. Problem Statement

The system is intended to localize an underwater robot in a known pile field using sonar and odometry, with sufficient stability and accuracy for close-proximity inspection.

The current approach attempts to do this by aligning accumulated sonar feature clouds to a prior occupancy-grid map and fusing the resulting correction with odometry.

That approach is insufficient because the discussion identified several structural issues:

* sonar returns do not map cleanly to filled occupancy regions or pile centers
* pile observations are often partial, view-dependent, and cluttered
* repetitive pile layouts create multiple locally plausible alignments
* long accumulated clouds can blur or distort the effective measurement
* a single local optimizer output is not always safe to fuse as though it were unimodal and reliable

The redesign is therefore motivated by a mismatch between what the sonar front-end produces and what the current map-matching formulation assumes, combined with the need for stronger ambiguity handling.

---

## 3. Design Goals

The new system should:

* provide robust local tracking in a known pile field using sonar and odometry
* correct odometry drift using the prior map without requiring exact pile extraction
* remain compatible with the current sonar front-end outputs
* tolerate clutter, false positives, partial observations, and imperfect accumulation
* explicitly handle ambiguity caused by repetitive pile structure
* accept map-based corrections only when the evidence is strong enough to justify them
* remain practical as a first implementation for local tracking rather than requiring full global localization

---

## 4. Non-Goals

This spec does not attempt to solve, in the first phase:

* full global localization under arbitrary initialization
* full kidnapped-robot recovery
* direct acoustic-image alignment as the primary method
* learned sonar feature matching
* fully explicit pile landmark extraction and association as the main backend
* a particle-filter-first architecture
* implementation planning, code architecture, or repository changes

These may become later-phase extensions if the proposed method proves insufficient.

---

## 5. Method Overview

### Front-end role

The front-end continues to produce sonar-derived local feature evidence. For this method, those outputs are interpreted as uncertain observations of nearby pile-boundary structure, not as exact map points and not as stable pile-center detections.

### Backend role

The backend performs map-based local tracking against a known prior. Its role is to:

* predict pose from odometry
* build a short, motion-compensated local evidence set
* compare that evidence to the prior map in a boundary-compatible way
* evaluate one or more nearby pose hypotheses
* determine whether the resulting correction is trustworthy enough to fuse
* update the running pose estimate when appropriate

### Map / representation choice

The map representation should be boundary-centric rather than purely occupancy-centric.

For phase one, the map input will still be a high-resolution occupancy grid from the current simulation setup. A separate preprocessing step will convert that occupancy grid into a continuous, high-fidelity structural boundary representation used by the localizer.

In the current simulation, the piles are square pylons rather than cylinders. The extracted map representation can therefore be described simply as:

* an array of known pile center points
* a globally known pile size parameter, expressed as square side length

From the pile centers and side length, the system can recover a continuous representation of the expected pile boundaries and therefore where boundary returns are geometrically plausible.

Although the phase-one map originates as an occupancy grid, the localizer should conceptually match against a continuous boundary map derived from pile geometry rather than against filled occupancy alone.

### Measurement interpretation

Measurements are treated as probabilistic, partial, anonymous evidence of nearby structural boundaries. Missing returns, clutter, and view-dependent effects are expected. The measurement model should therefore reward agreement with plausible boundary structure while remaining tolerant to outliers and incomplete observations.

### Local search / matching

The system performs bounded local search around the odometry prior, followed by local refinement. This is a local tracking system, so the search is intentionally limited in spatial extent, but it should still permit more than one nearby hypothesis when the geometry is ambiguous.

### Ambiguity handling

The backend must distinguish between:

* a tracking regime, in which one local match is clearly dominant and safe to use
* an ambiguity regime, in which multiple nearby alignments remain plausible

In the ambiguity regime, the system should reject, defer, or otherwise avoid overcommitting to a single strong correction.

### Fusion with odometry / EKF or smoother

Odometry remains a core motion prior. Map-based localization does not replace odometry; it constrains and corrects it.

Single-hypothesis fusion remains acceptable only when the match is sufficiently isolated, locally informative, and supported by a meaningful uncertainty estimate. When that is not true, the system should prefer conservative behavior over forced correction.

---

## 6. Current vs Proposed Conceptual Architecture

### Current approach

Conceptually, the current approach is:

1. the front-end produces sonar features
2. features are accumulated
3. the accumulated cloud is aligned to a previous feature map or SLAM-derived map representation
4. one best local solution is selected
5. that solution is fused with odometry

### Proposed approach

Conceptually, the proposed approach is:

At startup, the prior occupancy map is converted into a continuous boundary-centric structural representation.

Then, during operation:

1. the front-end produces sonar feature evidence
2. recent evidence is accumulated and motion-compensated over a short window
3. the sonar evidence is scored probabilistically against the boundary-compatible map
4. bounded local search evaluates one or more nearby pose hypotheses
5. an ambiguity stage determines whether the result is trustworthy enough to use as a single correction
6. only accepted updates are fused with odometry

### What stays the same

* use of the existing front-end
* use of odometry as a persistent motion prior
* use of a prior map rather than full online SLAM as the main localization source
* focus on local tracking as the first operating mode

### What changes

* the map representation used for matching
* the semantic interpretation of sonar features
* the introduction of explicit ambiguity handling
* stricter criteria for accepting map-based corrections
* a shift from SLAM-oriented feature-map alignment toward prior-map localization

---

## 7. Functional Requirements

The system must:

* consume the existing sonar front-end outputs as localization evidence
* consume odometry as a motion prior
* use a prior structural map derived from the known pile field
* support local pose estimation through scan-to-map-style matching
* use a boundary-compatible map representation for localization
* support robust scoring under clutter, partial observations, and false positives
* evaluate bounded local pose hypotheses around the predicted pose
* detect ambiguous or weak localization situations
* reject or defer unsafe corrections rather than forcing them
* output a pose estimate together with confidence or quality information
* continue operating when sonar evidence is weak by relying more heavily on odometry until stronger evidence appears

---

## 8. Quality Requirements

The system must:

* be robust to clutter and false positives in sonar feature clouds
* remain stable in repetitive pile geometry where neighboring alignments may appear similar
* reject ambiguous map updates gracefully
* avoid systematic bias caused by mismatched map semantics
* rely only on bounded odometry assumptions appropriate for local tracking
* expose uncertainty or confidence in a way that reflects real ambiguity rather than optimizer convergence alone
* degrade conservatively when assumptions are violated

---

## 9. Interfaces Between Major Subsystems

### Front-end

Responsibility: produce sonar-derived feature evidence from incoming sensor data.

### Odometry

Responsibility: provide continuous motion prediction and maintain pose propagation between accepted map-based corrections. This stage may itself use fused inertial and velocity information, but for the purposes of this spec its role is to provide the motion prior.

### Map representation stage

Responsibility: convert the prior map input into the structural representation used by localization.

For phase one, this means taking a high-resolution occupancy map and extracting a continuous boundary-centric representation based on known square pile center locations and a globally known side length.

### Localizer / matcher

Responsibility: compare recent sonar evidence to the boundary-centric prior map in a bounded neighborhood around the predicted pose and generate one or more candidate alignments.

### Ambiguity manager

Responsibility: assess whether the local evidence supports a safe single correction or whether the situation remains ambiguous.

### Fusion stage

Responsibility: combine accepted map-based corrections with odometry into the running state estimate. Only trusted corrections should be fused as strong updates.

---

## 10. Assumptions and Constraints

### Assumptions

* the front-end outputs contain enough consistent local structure to constrain pose against the prior map
* those outputs are better interpreted as boundary-like observations than as pile-center detections
* odometry is usually good enough to keep localization within a bounded local search region during normal tracking
* short-window motion compensation is good enough to keep accumulated evidence meaningful
* single-hypothesis fusion is only valid when the local match is effectively unimodal and informative
* for phase one, the prior occupancy map is high enough resolution that a high-fidelity boundary representation can be extracted from it without losing the relevant pile geometry
* in the current simulation, square piles can be modeled adequately using center locations and a global side-length parameter

### Constraints

* the front-end should ideally remain unchanged at first
* exact pile extraction from sonar cannot be assumed
* repetitive pile geometry must be treated as a central design condition
* the first system is for practical local tracking, not yet full global relocalization
* uncertainty and ambiguity should be preserved explicitly where the evidence does not justify stronger assumptions

---

## 11. Risks and Open Questions

The following remain uncertain and require experimental validation:

* whether front-end features at the correct pose actually align well with the chosen boundary-centric representation
* whether the extracted continuous pile-boundary map is sufficiently faithful for localization in practice
* whether short motion-compensated accumulation helps more than it harms
* how quickly ambiguity appears as odometry drift approaches pile spacing
* whether confidence and ambiguity signals can reliably separate correct matches from wrong but plausible ones
* how tolerant the method is to map error, discretization artifacts in the original occupancy input, or structural mismatch between prior and reality
* whether the phase-one square-pile abstraction transfers cleanly to later cylindrical-pile scenarios

---

## 12. Evaluation / Acceptance Criteria

The method should be considered successful at a high level if it demonstrates that:

* it tracks stably in the intended local operating regime
* it improves on odometry-only tracking when sonar evidence is informative
* it reduces wrong-pile or wrong-cell corrections relative to the original occupancy-grid alignment concept
* it handles clutter and false positives without frequent catastrophic updates
* it rejects or defers ambiguous corrections rather than overcommitting to them
* its confidence signals meaningfully correlate with actual localization quality
* the boundary-centric map representation yields better localization behavior than matching directly against raw filled occupancy

Evaluation should compare the proposed method against the original concept on stability, ambiguity handling, wrong-match frequency, sensitivity to clutter, and practical usability for local pile-field inspection.
