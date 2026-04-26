# ORB-SLAM PatchDefense Defense Implementation Notes (Based on `src/ORBSLAM2`)

This document describes the current **paper implementation version** in this repository. Its goal is not to explain how to run the repository, but to clarify the PatchDefense / MotionGuard defense pipeline under `src/ORBSLAM2`: where each module lives, what each step actually computes, which parameters are truly read by the code, and how the final result affects the frontend, backend, and loop closing.

Core code entry points:

- PatchDefense: `src/ORBSLAM2/include/PatchDefense.h`, `src/ORBSLAM2/src/PatchDefense.cc`
- MotionGuard: `src/ORBSLAM2/include/MotionGuard.h`, `src/ORBSLAM2/src/MotionGuard.cc`
- Tracking integration: `src/ORBSLAM2/src/Tracking.cc`
- Weight persistence: `src/ORBSLAM2/include/Frame.h`, `src/ORBSLAM2/src/KeyFrame.cc`
- Frontend/backend weighting: `src/ORBSLAM2/src/Initializer.cc`, `src/ORBSLAM2/src/PnPsolver.cc`, `src/ORBSLAM2/src/Sim3Solver.cc`, `src/ORBSLAM2/src/Optimizer.cc`
- Loop defense: `src/ORBSLAM2/src/LoopClosing.cc`

---

## 1. Goal and Threat Model

### 1.1 Goal

Suppress drift or jumps caused by feature-forging patch attacks in pose estimation, local optimization, and loop correction, while minimizing damage to normal tracking and mapping stability.

### 1.2 Threat Model (Engineering View)

An attacker injects a local patch into the image, causing that region to produce:

- **Abnormal feature density**: local feature bursts
- **Abnormal descriptor statistics**: overly homogeneous or unusually scattered descriptors
- **Abnormal orientation distribution**: excessively concentrated gradient directions
- **Abnormal texture statistics**: local entropy inconsistent with the surrounding context
- **Temporal bursts or persistent deviation**: some statistics remain unusually stable or change abruptly across consecutive frames

---

## 2. Overall Per-Frame Pipeline

When `use_patch_defense=1`, PatchDefense runs inside `Tracking::GrabImage*()` after ORB feature extraction for the current frame and before the frame formally enters `Track()`.

The full pipeline can be summarized as:

1. **Stage-A: multi-scale local anomaly statistics**
   Compute three local risk terms, density, descriptor dispersion, and orientation concentration, yielding
   $r_D, r_\sigma, r_\theta$ for each point.
2. **Candidate Gate**
   Use a weighted score to select the candidate set $\mathcal{C}$.
3. **Stage-B: RiskMask**
   Compute texture/orientation risk over the full image or near candidates, yielding $r_{\text{mask}}$.
4. **Stage-T: temporal consistency**
   Use EMA plus burst risk to obtain $r_{\text{temp}}$, then convert it to $w_{\text{temp}}$.
5. **Fusion**
   Combine the `risk mask` and `temporal` branches to obtain the final weight $w_i$.

The weights eventually affect SLAM through:

- **Frontend**: Initializer, PnP RANSAC, Sim3 RANSAC, PoseOptimization
- **Backend**: BA information-matrix scaling
- **Loop closing**: safe ratio + coverage + structural-distribution KL + hysteresis switch
- **Dynamic interference**: MotionGuard freezes the pose and can optionally block keyframe insertion

Two implementation details:

- If `use_patch_defense=0`, `PatchDefense::ProcessFrame()` returns immediately without applying any defense.
- If candidate gate is enabled but the current frame has no candidates, the RiskMask stage is skipped entirely and `mvPatchRisk` is set to zero everywhere.

---

## 3. Configuration Loading (`args.yaml`)

PatchDefense parameters are read from the repository-root `args.yaml`, using OpenCV `FileStorage` YAML format.

The actual search order is:

1. Environment variable `ORB_SLAM_ARGS`
2. `args.yaml`
3. `../args.yaml`
4. `../../args.yaml`
5. `../../../args.yaml`
6. `../../../../args.yaml`
7. `../../../../../args.yaml`

If no file is found, the default values in `PatchDefenseConfig` are used and a message is printed.

The most relevant file in this repository is:

- `args.yaml`
  Parameter snapshot actually used by the current experiment.

Top-level structure:

```yaml
use_patch_defense: 1
patch_defense:
  enable_risk_mask: 1
  ...
```
