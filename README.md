# rbnx_piper_packages — Piper Vertical-Grasp Deploy

Deploy workspace for the **AgileX Piper 6-DoF arm + Orbbec Dabai DCW RGBD**
grasp pipeline running on the [robonix](https://github.com/syswonder/robonix)
framework. This directory is a *manifest-only* deploy: none of the actual
package source lives here — every package is fetched from GitHub by
`rbnx boot` at boot time. What lives here is:

- `robonix_manifest.yaml` — the top-level manifest that lists every
  package to bring up, in dependency order, with their runtime config.
- `soma.yaml` + `urdf/piper.urdf` + `soma_config.local.yaml` — body
  description + URDF served by `robonix-soma` (see [SOMA_DEPLOY.md](./SOMA_DEPLOY.md)).
- `stop.sh` — teardown script that kills everything `rbnx boot` spawned.

For the migration history behind this deploy, see
[`/Users/howenliu/lab/docs/PIPER_PIPELINE_MIGRATION_PLAN.md`](../docs/PIPER_PIPELINE_MIGRATION_PLAN.md).
For the soma bring-up cheatsheet, see [SOMA_DEPLOY.md](./SOMA_DEPLOY.md).


---

## 1. Overview

A single `rbnx boot` command brings up **8 packages** organised in three
tiers (primitive / service / skill), plus the `robonix` system builtins
(`atlas` / `executor` / `pilot` / `liaison` / `soma`). At runtime the
pipeline offers a single high-level MCP tool — `robonix/skill/pick/pick`
— that the pilot LLM can call to grasp any table-top object by name.

### High-level flow

```
                 Pilot LLM   ─── "pick the comb"  ────┐
                                                     │ MCP
                                                     ▼
                                        ┌────────────────────────┐
                                        │       pick_skill       │
                                        │  (Stage 6, skill:)     │
                                        └────┬──────┬──────┬─────┘
                                             │ MCP  │ MCP  │ MCP
                                             ▼      ▼      ▼
                          ┌──────────────────┐ ┌──────────┐ ┌──────────────┐
                          │   llm_detect     │ │yolo_grasp│ │ piper_moveit │
                          │(VLM 2D detector) │ │(top-down │ │(MoveIt exec) │
                          │                  │ │ pose)    │ │              │
                          └────────┬─────────┘ └────┬─────┘ └──────┬───────┘
                                   │ ROS topic     │ TF lookup    │
                                   ▼                ▼              ▼
                    ┌────────────┐ ┌───────────────┐ ┌──────────────────┐
                    │orbbec_cam. │ │ easy_handeye2 │ │    piper_ctl     │
                    │(RGB/Depth) │ │ (link6 → cam) │ │  (CAN driver)    │
                    │            │ │piper_desc.    │ │                  │
                    │            │ │(URDF /tf)     │ │                  │
                    └────┬───────┘ └───────────────┘ └────────┬─────────┘
                         │ USB                                │ USB-CAN
                         ▼                                    ▼
                    Orbbec Dabai DCW                    AgileX Piper arm
```

### Grasp geometry (vertical mode)

The pipeline is currently configured for **top-down vertical grasps only**:

1. `llm_detect` receives an RGB frame + object name (e.g. `"comb"`), asks
   the VLM for a 2D bounding box.
2. `yolo_grasp` takes the bbox center pixel, back-projects a ray through
   the camera intrinsics, intersects that ray with the plane `z = z_table`
   in `arm/base_link` frame → `(x, y, z_table)`. Orientation is locked
   (roll=π, pitch=0), yaw is either fixed or set to `atan2(y, x)`
   (`radial_yaw: true`). Depth stream is unused (`skip_depth: true`).
3. `pick_skill` drives a 3-stage sequence: **pre-grasp hover → descend
   & close gripper → post-grasp lift**, using `approach_dist` as the
   z-offset for the hover.
4. `piper_moveit` runs MoveIt planning + executes on the real hardware
   via a vendored C++ `moveit_control_node_yolo` bridge.

---

## 2. Package roster

Order in the manifest matches boot order. `provider_id` in the table
matches the `- name:` in `robonix_manifest.yaml` and is what
`rbnx caps` shows.

### 2.1 Primitives (hardware + TF)

| # | provider_id | Repo (upstream) | Owns on atlas | Purpose |
|---|---|---|---|---|
| 1 | `orbbec_camera`     | `lhw2002426/OrbbecSDK_rbnx`          | `primitive/camera/{driver,rgb,depth,camera_info}` | Wraps Orbbec `dabai_dcw.launch.py`. Publishes ROS 2 topics `/camera/color/image_raw`, `/camera/depth/image_raw`, `/camera/color/camera_info`. Waits for first RGB frame before declaring ACTIVE. |
| 2 | `piper_ctl`         | `lhw2002426/piper_ctl_rbnx`          | `primitive/arm/{driver,joint_states,arm_status,end_pose,pos_cmd}` | Wraps `piper start_single_piper.launch.py` (CAN driver). Publishes `/arm/joint_states_single`, `/arm/arm_status`, `/arm/end_pose`; subscribes `/arm/pos_cmd`. |
| 3 | `piper_description` | `lhw2002426/piper_description_rbnx`  | *(no atlas caps — launch wrapper)* | Runs `robot_state_publisher` with the vendored Piper URDF, remapping `joint_states` → `/arm/joint_states_single`. Publishes the **unprefixed** joint-driven TF subtree `base_link → link1..link6`. |
| 4 | `easy_handeye2`     | `lhw2002426/easy_handeye2_rbnx`      | *(no atlas caps — launch wrapper)* | Stages `~/.ros2/easy_handeye2/calibrations/<name>.calib`, then runs `easy_handeye2 publish.launch.py` to broadcast the static hand-eye TF `link6 → camera_color_optical_frame`. |

### 2.2 Services (algorithms + motion)

| # | provider_id | Repo (upstream) | Owns on atlas | Purpose |
|---|---|---|---|---|
| 5 | `llm_detect`   | `lhw2002426/llm_detect_rbnx` (branch `feature/vertical-grasp`) | `service/perception/object_detect/{driver, detect_object}` | 2D open-vocabulary object detection using an OpenAI-compatible VLM. Consumes RGB + camera_info from `orbbec_camera`. **Replaces** the older `yolo_world_rbnx` (GPU/YOLOE) — no GPU needed. |
| 6 | `yolo_grasp`   | `lhw2002426/yolo_grasp_rbnx` (branch `feature/vertical-grasp`) | `service/perception/grasp_pose/{driver, grasp_request, grasps}` | Given a 2D bbox, computes a top-down grasp pose by intersecting the camera ray with `z=z_table` in `arm/base_link` frame. Depth is not used in this branch. |
| 7 | `piper_moveit` | `lhw2002426/piper_moveit_rbnx`       | `service/manipulation/{driver, execute_grasp}` | Runs MoveIt `move_group` + vendored C++ `moveit_control_node_yolo`. MCP handler bridges `execute_grasp(PoseStamped, gripper_width, timeout)` → ROS topic `/graspnet/grasps` + polls `/arm/arm_status` busy→idle. |

### 2.3 Skill (LLM entrypoint)

| # | provider_id | Repo (upstream) | Owns on atlas | Purpose |
|---|---|---|---|---|
| 8 | `pick` | `lhw2002426/pick_skill_rbnx` (branch `feature/vertical-grasp`) | `skill/pick/{driver, pick}` | The one MCP tool the pilot LLM actually calls. Orchestrates `detect_object` → `grasp_request` (with retries) → `execute_grasp` (three-stage pre/grasp/post sequence for the vertical pipeline). |

### 2.4 System builtins

Started by `rbnx boot` from the `system:` block of the manifest — no
per-package repo, they ship with the robonix source tree that
`rbnx setup` was pointed at:

- **`atlas`** (`127.0.0.1:50051`) — the capability registry. Every
  package registers here at startup and looks up its dependencies by
  contract id.
- **`executor`** (`127.0.0.1:50061`) — routes MCP tool calls; sends
  `Driver(CMD_INIT / CMD_ACTIVATE)` to package providers.
- **`pilot`** (`127.0.0.1:50071`) — the LLM chat frontend
  (`rbnx ask "..."`). Config is under `system.pilot.vlm` in the
  manifest (`upstream` URL + `api_key` + `model`).
- **`liaison`** (`127.0.0.1:50081`) — bridges external chat interfaces
  to the pilot.
- **`soma`** (`127.0.0.1:50091`) — body-description service; owns
  `system/soma/{get_yaml, get_urdf}`. Also drives the **two-stage
  bring-up**: `soma` itself launches the `primitive:` block in stage 1
  and the `skill:` block in stage 2 (after `rbnx` has finished the
  `service:` block and pipes `stage2\n` to it). See
  [`docs/soma_two_stage_bringup.md`](../docs/soma_two_stage_bringup.md).

---

## 3. Inter-package communication

Two independent channels run in parallel — **atlas / MCP** for the
LLM-facing surface, and **ROS 2 topics / TF** for the hardware and
motion-control loop. They intersect only where they must.

### 3.1 Atlas contracts (MCP over HTTP + gRPC)

Every package that owns capabilities registers them on the atlas gRPC
service at `127.0.0.1:50051`. Consumers resolve endpoints by contract
id, not by hostname:port. Example: `pick_skill` on `on_activate`
resolves three URLs from atlas:

- `robonix/service/perception/object_detect/detect_object`
- `robonix/service/perception/grasp_pose/grasp_request`
- `robonix/service/manipulation/execute_grasp`

Then talks to each via a FastMCP HTTP client. The pilot LLM uses the
same mechanism to see all available `mcp` tools.

### 3.2 ROS 2 topics (data plane)

Everything hardware-adjacent flows over ROS 2 topics on the local
DDS domain. The critical topics:

```
Producer                         Topic                                        Consumer(s)
──────────                       ─────                                        ──────────
orbbec_camera                    /camera/color/image_raw                      llm_detect
orbbec_camera                    /camera/color/camera_info                    yolo_grasp
orbbec_camera                    /camera/depth/image_raw                      (unused in vertical mode)

piper_ctl                        /arm/joint_states_single                     piper_description, piper_moveit (move_group)
piper_ctl                        /arm/arm_status                              piper_moveit (busy/idle poll)
piper_ctl                        /arm/end_pose                                (diagnostics)
piper_ctl                        (subscribes)  /arm/joint_states              ← piper_moveit fake ros2_control_node (COMMAND path)

piper_description                /tf   base_link → link1..link6               everyone doing tf2 lookups
easy_handeye2                    /tf_static  link6 → camera_color_optical_frame yolo_grasp, piper_moveit (cpp)
piper_moveit                     /tf_static  arm/link6 ↔ link6 (identity bridge)  piper_moveit (cpp)

yolo_grasp                       /graspnet/grasps                             piper_moveit (cpp moveit_control_node_yolo)
piper_moveit (MCP handler)       /graspnet/grasps                             piper_moveit (cpp)  — bridges MCP → topic
```

Two subtle "gotchas" worth internalising:

- **`/arm/joint_states` vs `/arm/joint_states_single`.** `piper_ctl`
  *publishes feedback* on `/arm/joint_states_single`. `piper_moveit`'s
  launch fork remaps the move_group planner to that topic, AND runs a
  fake `ros2_control_node` that publishes **command** trajectories on
  `/arm/joint_states` (which `piper_ctl` subscribes to and forwards
  to CAN). Two different topics, two different directions.
- **`/graspnet/grasps` has TWO publishers**: `yolo_grasp` (for legacy
  direct-topic callers) and `piper_moveit`'s own bridge (for MCP
  `execute_grasp` callers). Both feed the same C++ subscriber. The
  cpp node uses `is_busy_` mutex to serialise them.

### 3.3 TF tree (post-boot, expected shape)

```
Unprefixed subtree                        Prefixed subtree (MoveIt)
(from piper_description +                 (from piper_moveit launch fork)
 easy_handeye2)

base_link                                 arm/world
   └─ link1..link6                           └─ arm/base_link
         └─ camera_color_optical_frame             └─ arm/link1..arm/link6
                                                              ▲
                                                              │ identity static TF
                                                              │ (the bridge added
                                                              │  by piper_moveit)
         link6 ────────────────────────────────────────────── arm/link6
```

Verify after boot:

```bash
ros2 run tf2_ros tf2_echo arm/base_link camera_color_optical_frame
# Should print a stable composite transform.
```

---

## 4. Configuration — what you MUST set before running

The four things that always need attention on a fresh deploy machine.
Every other config knob has a working default.

### 4.1 Hand-eye calibration (`easy_handeye2`)

The vendored `packages/easy_handeye2_rbnx/config/calibrations/my_eih_calib.calib`
in the upstream repo ships as an **identity transform** — a placeholder.
Running with the placeholder means the camera is assumed to be at the
exact origin of `link6` with no rotation, and every grasp pose will be
wildly off.

The `env:` block of `robonix_manifest.yaml` currently points at an
absolute path on the deploy machine:

```yaml
env:
  EASY_HANDEYE2_CALIB_PATH: /home/syswonder/.ros2/easy_handeye2/calibrations/my_eih_calib.calib
```

Change this to wherever your real calibration file lives on the deploy
machine, OR unset it and let the package fall back to
`EASY_HANDEYE2_CALIB_NAME` (default `my_eih_calib`) which resolves to
`<easy_handeye2_rbnx>/config/calibrations/my_eih_calib.calib`.

To calibrate fresh:

```bash
# On the robot host, with piper_ctl + orbbec_camera + piper_description up:
ros2 launch easy_handeye2 calibrate.launch.py \
    calibration_type:=eye_in_hand \
    name:=my_eih_calib \
    robot_base_frame:=base_link \
    robot_effector_frame:=link6 \
    tracking_base_frame:=camera_color_optical_frame \
    tracking_marker_frame:=<aruco_marker_id>

# Take 10–20 samples across varied arm poses, click "Compute" → "Save".
# Upstream writes ~/.ros2/easy_handeye2/calibrations/my_eih_calib.calib.
```

See the [`easy_handeye2_rbnx` README](https://github.com/lhw2002426/easy_handeye2_rbnx)
"Replacing the calibration" section for the full workflow.

### 4.2 Arm home pose / initial position

The Piper does **not** auto-home on boot. Before the first `rbnx boot`
of a session, physically position the arm somewhere that is:

1. **Inside the reachable workspace of the vertical grasp** — anything
   deployed at `sqrt(x² + y²) > ~0.42 m` from the `base_link` is
   outside Piper's reach and MoveIt will return `GOAL_STATE_INVALID`.
2. **Not colliding with the tabletop or fixtures** when the arm
   descends by `approach_dist` (default 0.10 m).
3. **With the wrist-mounted camera pointing at the workspace** — the
   VLM only sees what the camera sees.

If you need to command a specific home pose, the easiest way is to
`ros2 topic pub` a `piper_msgs/PosCmd` on `/arm/pos_cmd` before boot
(with `rbnx boot` down, run `piper_ctl` by hand — see below), or run
the AgileX teach pendant tool. Example safe pose (used by the original
`skill/pick/pick.py` upstream):

```bash
ros2 topic pub --once /arm/pos_cmd piper_msgs/msg/PosCmd \
    "{x: 0.30, y: 0.0, z: 0.25, roll: 0.0, pitch: 1.57, yaw: 0.0,
      gripper: 0.05, mode1: 0, mode2: 0}"
```

The critical table-height constant `z_table` in `yolo_grasp.config`
must reflect the real table height in `arm/base_link` frame:

```bash
# Touch the gripper tip to the table, then:
ros2 run tf2_ros tf2_echo arm/base_link arm/link6
# Read the `z` component and put it into robonix_manifest.yaml:
#   yolo_grasp.config.z_table
# (The manifest currently ships -0.186 as an example measurement.)
```

Wrong `z_table` → arm either grasps in mid-air or crashes into the
table. This is the single most important physical constant in the
pipeline.

### 4.3 ROS 2 topics (usually fine as-is)

`llm_detect` and `yolo_grasp` resolve their input topics via atlas
from the `orbbec_camera` provider. As long as `orbbec_camera` runs
with the default `camera_name: camera` (the manifest's setting), the
downstream topic names line up with what upstream code hard-codes:

- `/camera/color/image_raw`
- `/camera/color/camera_info`
- `/camera/depth/image_raw` (unused in vertical mode)

If you rename the Orbbec ROS namespace (e.g. to run two cameras side
by side), you MUST also override every `*_topic` config in
`llm_detect` and `yolo_grasp`. The comments in the manifest show all
the overridable knobs — leave them empty to accept the atlas-resolved
default.

Fixed topic names that the vendored C++ code hard-codes and cannot be
renamed via config:

- `/graspnet/grasps` — `moveit_control_node_yolo` subscribes to this
  literal string.
- `/arm/joint_states` / `/arm/joint_states_single` — hard-coded in
  `piper_ctl` upstream (`namespace='/arm'` at launch time).

Don't rename these.

### 4.4 CAN bring-up for Piper

Before every `rbnx boot`, `can_piper` must be up. Two paths:

```bash
# Path A (recommended, no sudo coupling to boot):
cd /path/to/piper_ctl_rbnx  # e.g. cloned by rbnx boot under ~/.cache/rbnx/…
bash scripts/can_activate.sh can_piper 1000000 "1-4.4:1.0"
# Adjust the USB bus path ("1-4.4:1.0") to match your hardware.
# Discover it with `lsusb -t`.
```

Or set `auto_can_setup: true` in `piper_ctl.config` and give the
operator passwordless sudo (Path B — see the `piper_ctl_rbnx` README).

### 4.5 LLM API credentials

Two independent LLMs are used:

- **Pilot's chat LLM** — `system.pilot.vlm` in the manifest. Powers
  `rbnx ask "..."`.
- **`llm_detect`'s vision-language model** — `llm_detect.config.llm_*`.
  Powers 2D object detection.

Both use an OpenAI-compatible API. The manifest ships with example
keys; **replace them with your own** before running (the shipped
values may not stay valid).

---

## 5. Running

### 5.1 One-time host setup

```bash
# 1. Clone the robonix source tree and install rbnx + all system builtins:
git clone https://github.com/syswonder/robonix ~/robonix
cd ~/robonix
make build && make install     # installs rbnx + robonix-atlas/executor/pilot/liaison/soma to PATH

# 2. Point rbnx at that source tree (so it knows where to find system builtins):
rbnx setup     # interactively; or `rbnx path root set ~/robonix`

# 3. Get this deploy workspace:
git clone https://github.com/... rbnx_piper_packages
cd rbnx_piper_packages

# 4. Prep the calibration file (see §4.1):
mkdir -p ~/.ros2/easy_handeye2/calibrations
cp /path/to/your/calibrated.calib ~/.ros2/easy_handeye2/calibrations/my_eih_calib.calib

# 5. Edit robonix_manifest.yaml — set:
#    - env.EASY_HANDEYE2_CALIB_PATH   (§4.1)
#    - yolo_grasp.config.z_table      (§4.2)
#    - system.pilot.vlm.api_key       (§4.5)
#    - llm_detect.config.llm_api_key  (§4.5)
```

### 5.2 Per-session bring-up

```bash
# 1. Bring up CAN:
bash /path/to/piper_ctl_rbnx/scripts/can_activate.sh can_piper 1000000 "1-4.4:1.0"

# 2. Sanity-check the manifest:
cd /path/to/rbnx_piper_packages
rbnx validate

# 3. Boot the whole stack:
rbnx boot
# Expect logs from atlas / executor / pilot / liaison / soma coming up,
# then soma stage 1 launching orbbec_camera → piper_ctl → piper_description
# → easy_handeye2, then rbnx launching llm_detect → yolo_grasp → piper_moveit,
# then soma stage 2 launching pick (CMD_INIT only; ACTIVATE is deferred).

# 4. In another shell — verify:
rbnx caps
# expect ACTIVE state for orbbec_camera, piper_ctl, llm_detect, yolo_grasp,
# piper_moveit; ACTIVE (registration-only) for piper_description +
# easy_handeye2; INITIALIZED for pick (lazy activate).

# 5. Trigger a pick via the pilot LLM:
rbnx ask "please pick up the comb on the table"
# Pilot should select `robonix/skill/pick/pick(object_name="comb")` and run
# the whole pipeline.

# 6. Tear it all down:
bash /path/to/rbnx_piper_packages/stop.sh
```

### 5.3 Direct MCP call (skip pilot)

Useful for debugging when the LLM keeps mis-selecting tools:

```bash
# Find the pick_skill port (from `rbnx caps -v`, or start log):
curl -s http://127.0.0.1:<pick_port>/mcp/ \
    -H "Content-Type: application/json" \
    -H "Accept: application/json, text/event-stream" \
    -d '{
      "jsonrpc":"2.0","id":1,
      "method":"tools/call",
      "params":{
        "name":"pick",
        "arguments":{"object_name":"comb","timeout_s":60.0,"max_retries":5}
      }
    }'
```

---

## 6. Troubleshooting

| symptom | likely cause | fix |
|---|---|---|
| `rbnx validate` reports `unknown key system.soma` or `robot_yaml` | Installed `rbnx` is older than PR #109 (v2 flat soma schema) | Re-run `make install` from the robonix source root; see [SOMA_DEPLOY.md](./SOMA_DEPLOY.md) §2.1 |
| `rbnx caps` shows `pick` as `INITIALIZED` and never activates | Expected — `pick` is lazy-activate. First MCP call triggers CMD_ACTIVATE. |
| `pick skill cannot find dependencies on atlas: missing [...]` | One of `llm_detect` / `yolo_grasp` / `piper_moveit` failed to activate | `rbnx caps` to find the missing one; look at its `rbnx-boot/logs/<name>.log` |
| `MoveIt` returns `GOAL_STATE_INVALID` | Target pose outside Piper's reach (arm span ≈ 0.42 m). See §4.2 | Move the target object into `sqrt(x²+y²) < 0.4 m` from `base_link`, or lower `approach_dist` |
| Arm executes and reports success, but doesn't physically move | The fake `ros2_control_node` in `piper_moveit`'s launch fork is not remapping `joint_states` → `/arm/joint_states` | Verify with `ros2 topic hz /arm/joint_states` during a grasp — should spike. Check the launch file wasn't hand-edited to isolate the remap. |
| `yolo_grasp` logs `TF lookup arm/base_link ← camera_color_optical_frame timed out` | `easy_handeye2` not ACTIVE, or the identity bridge `link6 ↔ arm/link6` didn't come up | `rbnx caps -v \| grep easy_handeye2` should be ACTIVE; `ros2 run tf2_ros tf2_echo link6 arm/link6` should print identity |
| `orbbec_camera` sentinel timeout | USB permissions or wrong camera model | Check `lsusb`; ensure the Orbbec udev rules are installed; the deploy expects Dabai DCW |
| `piper_ctl` sentinel timeout | CAN is not up | Re-run `can_activate.sh`; verify with `ip link show can_piper` |
| `llm_detect` returns 401 / connection refused | API key expired or wrong `llm_base_url` | Update `llm_detect.config.llm_api_key` / `llm_base_url` |
| Everything looks fine but grasps miss by ~10 cm | `z_table` is wrong, or hand-eye calibration is off | Re-measure `z_table` (§4.2); re-run calibration (§4.1) |

For deeper debugging, each package writes to `rbnx-boot/logs/<name>.log`
(under the deploy directory after `rbnx boot`). The pilot LLM's chat
history and tool-call trace live under `~/.robonix/memory/`.

---

## 7. Alternate mode — VLA (not currently active)

The manifest has an `openvla_client` section commented out. It provides
a closed-loop VLA (Vision-Language-Action) policy that writes directly
to `/arm/pos_cmd` at 2 Hz. **It cannot run alongside the vertical-grasp
pipeline** because both would fight for control of `/arm/pos_cmd`.

To switch modes:

1. Comment out `llm_detect`, `yolo_grasp`, `piper_moveit`, `pick`.
2. Uncomment `openvla_client` and set `vla_server_url` to your VLA
   server (see the `vla_client_rbnx` README for expected action ranges).
3. Also update `soma.yaml`'s `description.can_do` / `cannot_do` /
   `notes` sections so the pilot LLM sees consistent capabilities.

A future refactor will split this into `robonix_manifest.grasp.yaml`
and `robonix_manifest.vla.yaml` selectable via symlink.

---

## 8. Reference

- Migration history: [`../docs/PIPER_PIPELINE_MIGRATION_PLAN.md`](../docs/PIPER_PIPELINE_MIGRATION_PLAN.md)
- Soma bring-up cheatsheet: [`./SOMA_DEPLOY.md`](./SOMA_DEPLOY.md)
- Two-stage bring-up design: [`../docs/soma_two_stage_bringup.md`](../docs/soma_two_stage_bringup.md)
- Package sources: <https://github.com/lhw2002426/> (all 8 repos)
- robonix framework: <https://github.com/syswonder/robonix>

Per-package READMEs on each upstream repo document the internals
(algorithm, launch layout, failure modes) in depth. Start there when a
particular package misbehaves.
