# urdf

Vendored copy of the AgileX Piper URDF, served by `robonix-soma` at
`robonix/system/soma/get_urdf`. Referenced by `../soma.yaml`
(`urdf.path: ./urdf/piper.urdf`).

## What's here

- `piper.urdf` — WITH-GRIPPER variant. `<robot name="piper">` with
  links `base_link → link1..link6 → link7/link8` (link7/link8 are the
  gripper fingers) and joints `joint1..joint8`. Source of truth:
  `/Users/howenliu/lab/package_syswonder/primitive-agilex-piper-description-rbnx/src/piper_description/urdf/piper_description.urdf`
  (upstream: <https://github.com/syswonder/primitive-agilex-piper-description-rbnx>).

## Frame conventions

The vendored URDF is the exact byte content that gets loaded into
`robot_state_publisher` by the `piper_description` primitive (see
`../robonix_manifest.yaml`), so `/tf` published on the ROS 2 side is
guaranteed to match what soma serves over gRPC.

Key links referenced elsewhere in this deploy:

- `base_link` — arm mount frame. All primitives in
  `../soma.yaml`'s `tree:` anchor here.
- `link6` — Piper end-effector body. The eye-in-hand Orbbec Dabai
  DCW is calibrated relative to this frame via `easy_handeye2`
  (static TF `link6 → camera_color_optical_frame` staged by
  `easy_handeye2_rbnx/scripts/atlas_register_and_launch.py`).
- `link7` / `link8` — parallel-jaw gripper fingers. Currently driven
  as a **binary** open/close via `primitive/arm/pos_cmd[6]` (see
  `openvla_client` config in `../robonix_manifest.yaml`).

## Swapping to the no-gripper variant

If a naked arm is needed (e.g. rehearsing VLA output without the
gripper attached), copy the sibling file from the upstream package
tree:

```bash
cp /Users/howenliu/lab/package_syswonder/primitive-agilex-piper-description-rbnx/src/piper_description/urdf/piper_no_gripper_description.urdf \
   /Users/howenliu/lab/package_syswonder/robot-agilex-piper/urdf/piper.urdf
```

and update `../soma.yaml`'s `tree.children` to drop the `gripper`
component. (No env-var override at the soma layer — soma always
reads whatever bytes live at `urdf.path`.)

For the ROS 2 side, the equivalent switch is the top-level
`env: PIPER_URDF_PATH: ...` in `../robonix_manifest.yaml`, which the
`piper_description` package's launch file reads. Keep both sides
consistent so `/tf` and `get_urdf` agree.
