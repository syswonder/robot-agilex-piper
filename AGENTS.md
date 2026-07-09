# AGENTS.md

This repository is a Robonix deploy workspace for the Piper arm + Orbbec camera
vertical-grasp pipeline. It is mostly configuration; the runnable package code
is cloned and built under `rbnx-boot/cache/` by `rbnx boot`.

## Related Workspaces

- Roboarm reference workspace: `/home/syswonder/lhw/roboarm`

## Start / Stop

Start the system from the repository root:

```bash
rbnx boot
```

Stop leaked Robonix / ROS / Docker processes with:

```bash
bash stop.sh
```

Before booting real hardware, make sure CAN is already up for Piper unless
`auto_can_setup` is enabled in `robonix_manifest.yaml`.

## Boot Flow

`rbnx boot` reads `robonix_manifest.yaml`.

1. `rbnx` starts system processes: `atlas`, `executor`, `pilot`, `liaison`, and
   builtin `soma`.
2. `soma` stage 1 starts `primitive:` packages in order:
   `orbbec_camera`, `piper_ctl`, `piper_description`.
3. `rbnx` starts `service:` packages in order:
   `llm_detect`, `grasp_pose`, `roboarm_ik`.
4. `rbnx` writes the `stage2` trigger to `soma`.
5. `soma` stage 2 initializes `skill:` packages. `pick` remains inactive until
   the first tool call, then resolves its upstream MCP services.

## Grasp Flow

User command to pick an object goes through this chain:

```text
pilot/executor
  -> pick_skill.pick(object_name)
  -> llm_detect.detect_object       # RGB image -> LLM/VLM bbox
  -> grasp_pose.grasp_request       # bbox -> base_link grasp pose
  -> roboarm_ik.execute_grasp       # IK + joint/gripper command sequence
  -> piper_ctl                      # publishes to the real Piper arm
  -> roboarm_ik.reset               # park arm / clear sticky state
```

In this deploy, depth is skipped for detection. `grasp_pose` computes xy by
projecting the bbox center through the calibrated 2D homography at
`rbnx-boot/hand-eye-data/2d_homography.npy`, then uses
`default_desktop_height` as the grasp z in `arm/base_link`.

## Useful Debug Files

- Main deploy config: `robonix_manifest.yaml`
- Robot body / URDF pointer: `soma.yaml`
- Runtime logs: `rbnx-boot/logs/*.log`
- Package source cache: `rbnx-boot/cache/*_rbnx/`
- Latest LLM detection images: `~/.llm_detect/detections/latest.jpg`

For grasp failures, check logs in this order:

1. `rbnx-boot/logs/pick.log`
2. `rbnx-boot/logs/llm_detect.log`
3. `rbnx-boot/logs/grasp_pose.log`
4. `rbnx-boot/logs/roboarm_ik.log`

Common failure areas are invalid LLM bbox output, missing or stale 2D
homography, incorrect `default_desktop_height`, IK reachability failures,
and missing Piper joint / gripper feedback.
