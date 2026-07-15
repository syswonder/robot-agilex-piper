# robot-agilex-piper

Deploy workspace for the **AgileX Piper arm + Orbbec Dabai DCW** vertical-grasp pipeline on [Robonix](https://github.com/syswonder/robonix).

This branch keeps the `pick_ok` package shape:

- `primitive:` `orbbec_camera`, `piper_ctl`, `piper_description`
- `service:` `llm_detect`, `grasp_pose`, `roboarm_ik`
- `skill:` `pick`

Compared with the older `main` branch deploy, this variant uses `grasp_pose` and `roboarm_ik` instead of `yolo_grasp` and `piper_moveit`, and all remote packages now point at the `syswonder` organization.

## Repository Layout

- `robonix_manifest.yaml`: top-level deploy manifest; defines package order, upstream repos, runtime config, and catalog metadata.
- `soma.yaml`: robot body description served by `robonix-soma`.
- `soma_config.local.yaml`: local soma runtime config.
- `urdf/`: vendored Piper URDF used by soma.
- `tools/`: 2D hand-eye calibration utilities for this vertical-grasp pipeline.
- `patches/`: optional Piper SDK patch for draining stale CAN feedback backlog.
- `stop.sh`: best-effort shutdown script for processes launched by `rbnx boot`.
- `SOMA_DEPLOY.md`: soma-specific deployment checklist and troubleshooting notes.

## Package Set

| Role | Instance name | Upstream repo | Purpose |
|---|---|---|---|
| primitive | `orbbec_camera` | `syswonder/primitive-orbbec-dabai_dcw-camera-rbnx` | Wrist RGBD camera provider; vertical mode disables depth usage in downstream grasping. |
| primitive | `piper_ctl` | `syswonder/primitive-agilex-piper-arm-rbnx` | CAN-backed Piper arm driver; publishes joint state, arm status, end pose, and accepts position commands. |
| primitive | `piper_description` | `syswonder/primitive-agilex-piper-description-rbnx` | Publishes the Piper TF / URDF view used by the rest of the stack. |
| service | `llm_detect` | `syswonder/service-object-detect-rbnx` | VLM/LLM-based 2D object detection on the wrist RGB image. |
| service | `grasp_pose` | `syswonder/service-grasp-pose-rbnx` | Projects the 2D detection result into an `arm/base_link` top-down grasp pose using the calibrated homography. |
| service | `roboarm_ik` | `syswonder/service-roboarm-ik-rbnx` | Executes reset / teach-safe / execute-grasp manipulation actions. |
| skill | `pick` | `syswonder/skill-pick-vertical-grasp-rbnx` | User-facing Robonix skill that orchestrates the whole pick pipeline. |

All package refs in `robonix_manifest.yaml` use `branch: main`.

## Runtime Flow

A typical grasp request goes through this chain:

```text
pilot / executor
  -> pick.pick(object_name)
  -> llm_detect.detect_object      # RGB image -> object bbox
  -> grasp_pose.grasp_request      # bbox -> grasp pose in arm/base_link
  -> roboarm_ik.execute_grasp      # IK + motion sequence + gripper action
  -> piper_ctl                     # sends the actual command to the real arm
```

Important runtime properties of this deploy:

- `llm_detect` runs in **vertical-grasp mode** with `skip_depth=true`.
- `grasp_pose` uses `rbnx-boot/hand-eye-data/2d_homography.npy` and `default_desktop_height` to recover the final grasp target.
- `pick` is still **lazy-activated**: it is initialized during boot and activated on the first tool call.

## Quick Start

### 1. Prepare environment variables

At minimum, export the model endpoint variables used by `pilot` and `llm_detect`:

```bash
export VLM_BASE_URL=...
export VLM_API_KEY=...
export VLM_MODEL=...
export PILOT_MODEL=...
```

### 2. Bring CAN up if needed

If `auto_can_setup: false` stays unchanged, prepare the CAN interface before booting real hardware.

### 3. Boot the stack

```bash
rbnx boot
```

### 4. Stop the stack

```bash
bash stop.sh
```

## Calibration and Ops Notes

### 2D hand-eye calibration

This deploy depends on the 2D homography consumed by `grasp_pose`:

- collect / compute with `python3 tools/calibrate_handeye_2d.py`
- verify with `python3 tools/test_handeye_2d.py --z -0.19 --orientation-mode vertical`

See [`tools/README.md`](./tools/README.md) for the full workflow.

### Piper SDK CAN backlog patch

If Piper feedback topics become increasingly stale over time, apply the optional patch under:

- [`patches/piper_sdk_can_drain_latest`](./patches/piper_sdk_can_drain_latest/README.md)

It drains accumulated CAN frames and keeps only the latest frame per arbitration id.

## Catalog Metadata

This deploy manifest now includes a `catalog` block so it is ready for catalog-facing tooling:

- `catalog.name`: `robonix.robot.agilex.piper_grasp`
- `catalog.version`: `0.2.0`
- tags cover robot / deploy / manipulation / perception / grasp

The catalog description intentionally reflects the **current `pick_ok` package set**, not the older `main`-branch `yolo_grasp + piper_moveit` deploy shape.

## References

- [`robonix_manifest.yaml`](./robonix_manifest.yaml)
- [`SOMA_DEPLOY.md`](./SOMA_DEPLOY.md)
- [`soma.yaml`](./soma.yaml)
- [`tools/README.md`](./tools/README.md)
- Robonix framework: `https://github.com/syswonder/robonix`
