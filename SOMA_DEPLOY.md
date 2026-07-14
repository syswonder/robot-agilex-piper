# Soma integration deploy cheatsheet — Piper grasp deploy

> Quick reference for the soma integration used by the Piper-arm + Orbbec + grasp pipeline.
> The design rationale and historical decisions live in
> `/Users/howenliu/lab/docs/soma_two_stage_bringup.md` (the v2 flat schema is
> covered in §12, and the PR review follow-up is in §13). The full-length sister
> deploy is `/Users/howenliu/lab/ranger_mini_deploy/SOMA_DEPLOY.md` — the soma
> integration pattern is aligned across the two deploys; only the body
> description / URDF / manifest contents differ.

## 0. Preconditions

Before doing anything below, make sure all of the following are true:

- [x] The robonix source tree already contains PR #91 (first landing of soma)
      and PR #109 (v2 flat schema + pipe-based stage-2 trigger). After those two PRs:
        * `system/soma/src/{config,store,deployment,launcher,main}.rs`
          use the flat schema;
        * `tools/rbnx/src/cmd/deploy.rs::spawn_soma_binary` uses `pipe(2)`
          + `dup2` to pin the read end to child fd 3;
        * `capabilities/system/soma/get_yaml.v1.toml` / `get_urdf.v1.toml`
          have `kind = "service"` (not `"system"`).
- [ ] No task is currently running on the Piper side of the robot
      (avoid colliding with an ongoing grasp).
- [ ] CAN is already up (`ip link show can_piper` reports `UP`); if not, run
      `bash /Users/howenliu/lab/packages/piper_ctl_rbnx/scripts/can_activate.sh can_piper 1000000 "1-4.4:1.0"` first.
- [ ] **Extra precondition for the vertical-grasp variant**: this manifest now
      points `llm_detect`, `yolo_grasp`, and `pick` at `branch: feature/vertical-grasp`.
      Those three branches currently exist only in local `packages/*_rbnx/`
      working trees and have **not been pushed to GitHub yet**. Since
      `rbnx boot` will `git clone -b feature/vertical-grasp`, the boot will fail
      immediately if the branch does not exist remotely. Before deploying to the robot host,
      run:
      ```bash
      for pkg in llm_detect_rbnx yolo_grasp_rbnx pick_skill_rbnx; do
          git -C /Users/howenliu/lab/packages/$pkg push -u origin feature/vertical-grasp
      done
      ```
      After those branches are merged back to `main`, switch the three
      `branch: feature/vertical-grasp` entries in the manifest back to
      `branch: main`.
- [ ] **`openvla_client` is disabled in this manifest** (it conflicts with the
      grasp pipeline; see the comment block around `openvla_client` in
      `robonix_manifest.yaml`). To switch back to the VLA demo, do the reverse:
      comment out `llm_detect` / `yolo_grasp` / `piper_moveit` / `pick`, and
      uncomment `openvla_client`.

## 0.5 Startup model at a glance (Piper side; source-of-truth matches ranger)

```
rbnx boot
  ├─ system builtin stage (forked directly by rbnx)
  │   atlas → executor → pilot → liaison → soma
  │     ↑                                  ↑
  │     bin_map in deploy.rs               spawn_soma_binary (PR #109):
  │                                        pipe(2) + dup2(read_fd, 3)
  │                                        env ROBONIX_SOMA_STAGE_FD=3
  │
  ├─ non-builtin system stage (rbnx spawn + Driver(CMD_INIT))
  │   currently unused in this manifest
  │   (memory / scene / speech are all commented out)
  │
  ├─ soma stage 1 (launched by soma itself; rbnx does not manage this block)
  │   primitive: orbbec_camera → piper_ctl → piper_description → easy_handeye2
  │   each entry goes through spawn + wait_for_registration + CMD_INIT + CMD_ACTIVATE
  │
  ├─ service stage (rbnx spawn + Driver(CMD_INIT))
  │   llm_detect → yolo_grasp → piper_moveit
  │   (`openvla_client` is commented out in this manifest; it conflicts with the grasp pipeline)
  │
  ├─ stage 2 trigger
  │   after rbnx finishes the service block, it `write_all(b"stage2\n")`
  │   into soma's pipe write end
  │
  └─ soma stage 2
      skill block in this manifest: pick. soma spawns pick_skill and sends
      CMD_INIT, but does NOT send CMD_ACTIVATE — pick uses lazy activation,
      so executor sends CMD_ACTIVATE only when pilot makes the first MCP call.
```

After soma starts, it registers two gRPC capabilities on atlas by itself
(`system/soma/src/main.rs::register_soma_services_and_activate`):

```
robonix/system/soma/get_yaml   Transport::Grpc  port 50091
robonix/system/soma/get_urdf   Transport::Grpc  port 50091
```

## 1. Relevant files in this deploy directory

```
rbnx_piper_packages/
├── robonix_manifest.yaml       ← already includes the system.soma: block
├── soma.yaml                   ← generated locally (robot=piper_grasp_01, urdf=./urdf/piper.urdf)
├── soma_config.local.yaml      ← generated locally (v2 flat schema, four fields)
├── urdf/
│   ├── piper.urdf              ← vendored from packages/piper_description_rbnx (with-gripper variant)
│   └── README.md
├── stop.sh                     ← existing teardown script
└── SOMA_DEPLOY.md              ← the document you are reading now
```

## 2. What to do on the deploy host (in order)

### 2.1 Build the `robonix-soma` binary (full same-source install)

```bash
ssh robot     # or your actual SSH alias for the deploy machine
export ROBONIX_SOURCE_PATH=/home/syswonder/wheatfox/robonix   # adjust to the real path on that machine
cd "$ROBONIX_SOURCE_PATH"

git fetch origin
git checkout main
git pull --ff-only

make build && make install   # full same-source install; avoid mixed atlas/soma/rbnx versions

which robonix-soma && robonix-soma --help
```

> ⚠️ Do not run `cargo install -p robonix-soma` in isolation. Either install the
> full source tree together, or leave it alone. Since PR #91, the wire protocol of
> rbnx / atlas / soma has evolved together; split installs can lead to odd failures
> like `unknown key system.soma` or broken pipe-fd handoff.

### 2.2 Copy every locally generated file to the deploy host via `scp`

On your Mac:

```bash
cd /Users/howenliu/lab/rbnx_piper_packages
DEPLOY_REMOTE=robot:~/lhw/rbnx_piper_packages/   # adjust to the real path on the deploy host

# Make a backup before overwriting
ssh robot 'cp ~/lhw/rbnx_piper_packages/robonix_manifest.yaml ~/lhw/rbnx_piper_packages/robonix_manifest.yaml.before-soma 2>/dev/null || true'

scp robonix_manifest.yaml            "$DEPLOY_REMOTE"      # ★ overwrite
scp soma.yaml                        "$DEPLOY_REMOTE"
scp soma_config.local.yaml           "$DEPLOY_REMOTE"
ssh robot 'mkdir -p ~/lhw/rbnx_piper_packages/urdf'
scp urdf/piper.urdf urdf/README.md   "$DEPLOY_REMOTE/urdf/"
scp SOMA_DEPLOY.md                   "$DEPLOY_REMOTE"
```

### 2.3 Static validation

```bash
ssh robot
cd ~/lhw/rbnx_piper_packages/
rbnx validate
```

If `rbnx validate` reports `unknown key system.soma` or `unknown key robot_yaml`,
your installed `rbnx` is older than PR #109. Go back to §2.1 and re-run
`make install`.

Also do one plain YAML syntax check (does not require `rbnx`, so it still works
when the network is down):

```bash
python3 -c "import yaml; yaml.safe_load(open('robonix_manifest.yaml'))"
python3 -c "import yaml; yaml.safe_load(open('soma.yaml'))"
python3 -c "import yaml; yaml.safe_load(open('soma_config.local.yaml'))"
```

### 2.4 Boot + acceptance checks

```bash
ssh robot
cd ~/lhw/rbnx_piper_packages/
rbnx boot
```

With PR #109, `rbnx boot` translates `system.soma:` into:

```
robonix-soma \
  --listen 127.0.0.1:50091 \
  --atlas 127.0.0.1:50051 \
  --provider-id soma \
  --robot-yaml <abs>/rbnx_piper_packages/soma.yaml \
  --config    <abs>/rbnx_piper_packages/soma_config.local.yaml \
  --log info
```

When `spawn_soma_binary` launches the process, it additionally uses
`pipe(2)` + `dup2(read_fd, 3)` + env `ROBONIX_SOMA_STAGE_FD=3`. It does
**not** `chdir`, so `--robot-yaml` and `--config` must be absolute paths —
`ensure_soma_defaults` in rbnx already joins them against the
`robonix_manifest.yaml` directory for you.

Open another SSH session and run these acceptance checks:

```bash
# 1) files exist
ls soma.yaml soma_config.local.yaml urdf/piper.urdf

# 2) soma has registered both gRPC capabilities on atlas
rbnx caps | grep soma
rbnx caps -v | grep -A 3 'robonix/system/soma'
# Expected: endpoint=127.0.0.1:50091, transport=grpc, state=ACTIVE

# 3) direct gRPC call (using grpcurl) — verify soma loaded both yaml + urdf
grpcurl -plaintext -d '{"robot_id":""}' 127.0.0.1:50091 \
    robonix.contracts.RobonixSystemSomaGetYaml/GetYaml
# Expected: yaml_text contains "id: piper_grasp_01" and "Piper 6-DoF"

grpcurl -plaintext -d '{"robot_id":""}' 127.0.0.1:50091 \
    robonix.contracts.RobonixSystemSomaGetUrdf/GetUrdf
# Expected: urdf_xml contains <robot name="piper"> and link name="link6"

# 4) primitive block (launched by soma stage 1) should all be ACTIVE
rbnx caps -v | grep -E 'orbbec_camera|piper_ctl|piper_description|easy_handeye2'
# Expected: orbbec_camera / piper_ctl show ACTIVE;
#           piper_description / easy_handeye2 have capabilities: [],
#           so atlas only shows provider registration with no capability rows.

# 5) service block (launched by rbnx; the three-piece vertical-grasp pipeline)
rbnx caps -v | grep -E 'llm_detect|yolo_grasp|piper_moveit'
# Expected: all three are ACTIVE.
#   * llm_detect   → service/perception/object_detect/*  ACTIVE
#   * yolo_grasp   → service/perception/grasp_pose/*     ACTIVE
#   * piper_moveit → service/manipulation/execute_grasp ACTIVE

# 6) skill block: pick is spawned by soma stage 2 and receives CMD_INIT,
#                 but CMD_ACTIVATE is only sent by executor on the first MCP call.
rbnx caps -v | grep pick
# Expected: INACTIVE (lazy activation). After the first pilot MCP call to
# pick, it should become ACTIVE.
```

## 3. When something goes wrong

| Symptom | Likely cause | Fix |
|---|---|---|
| `robonix-soma: command not found` | install failed or `PATH` not refreshed | Go back to §2.1, re-run `make install`; then `source ~/.cargo/env` |
| `rbnx validate` reports `unknown key system.soma` or `robot_yaml` | `rbnx` is older than PR #109 | Re-do §2.1 |
| `rbnx caps` does not show soma's two capabilities | atlas did not start, or soma crashed after launch | Check `rbnx-boot/logs/soma.log` (soma's own stdout/stderr). Common crash: `read URDF '<path>': No such file` → you forgot to `scp` `urdf/piper.urdf` in §2.2 |
| `Error: parse '<path>/soma.yaml'` | syntax error in `soma.yaml` (usually introduced while copy/pasting comments) | Run `python3 -c "import yaml; yaml.safe_load(open('soma.yaml'))"` to locate it quickly; then re-`scp` the file |
| `read URDF '<path>/urdf/piper.urdf': No such file` | `urdf/` was not copied to the deploy host | Go back to §2.2 and `scp` `urdf/piper.urdf` |
| `<robot name="piper">` exists but `<link name="link6">` is missing | you copied an outdated or wrong URDF | Re-copy from `/Users/howenliu/lab/packages/piper_description_rbnx/src/piper_description/urdf/piper_description.urdf` |
| soma stage 1 hangs at `waiting for provider ... to register` | orbbec or piper_ctl did not come up | Check the corresponding `rbnx-boot/logs/<name>.log`; common causes are USB permissions for Orbbec and CAN not being up for piper_ctl |
| After the stage-2 trigger, soma never launches the skill | the `pick` block was commented again manually, or the feature branch was not pushed and `git clone` failed | Check `rbnx-boot/logs/pick.log`; confirm the `feature/vertical-grasp` branches in §0 were pushed to origin |
| `llm_detect` reports 401 / connection refused in `rbnx caps` | invalid `llm_api_key` or unreachable `llm_base_url` in `robonix_manifest.yaml` | Replace with a valid key; for offline integration testing you can temporarily point `llm_base_url` to a local OpenAI-compatible service |
| `yolo_grasp` logs `TF lookup arm/base_link ← camera_color_optical_frame timed out` | `easy_handeye2` is not up, or the static TF is missing | `rbnx caps -v \| grep easy_handeye2` should show ACTIVE; or verify manually with `ros2 run tf2_ros tf2_echo arm/base_link camera_color_optical_frame` |
| pick always misses or crashes into the table | `yolo_grasp.config.z_table` does not match the real tabletop height | Measure the tabletop z with `ros2 run tf2_ros tf2_echo arm/base_link <table_marker>` and update `yolo_grasp.config.z_table` in `robonix_manifest.yaml`; use `z_offset` for fine adjustment |

## 4. Next steps

- This manifest already enables the vertical-grasp pipeline
  (`llm_detect` + `yolo_grasp` + `piper_moveit` + `pick`) and keeps
  `openvla_client` commented out. To switch back to the VLA demo:
  comment out those four packages, uncomment `openvla_client`, and also
  switch the `description.can_do` / `cannot_do` / `notes` sections in
  `soma.yaml` back to the VLA wording — those descriptions are read by the
  pilot LLM, and lying there means pilot may invent tool combinations that
  cannot actually run.
- After the three `feature/vertical-grasp` branches are merged back into
  `main`, change the three `branch: feature/vertical-grasp` entries for
  `llm_detect`, `yolo_grasp`, and `pick` in `robonix_manifest.yaml` back
  to `branch: main`.
- `yolo_grasp.config.z_table` is the most critical physical constant in
  this pipeline. The factory-style default `0.02 m` is only a placeholder;
  before deploying to real hardware, you must measure it on the real table
  with `tf2_echo` or a ruler and write the result back. Also note that
  `default_yaw_rad` and `default_gripper_width` need tuning per object
  shape; it is a good idea to add a dedicated "calibrate tabletop" step to
  the SOP.
- `pick` uses lazy activation, so `rbnx caps` will show it as INACTIVE
  until the pilot makes the first MCP call; no manual pre-warm is needed.
- If you want to remove the gripper, follow the "Swapping to the no-gripper
  variant" section in `urdf/README.md`; then delete the gripper subsection
  from the tree in `soma.yaml` as well so the description stays consistent.
  Without a gripper there is no grasping semantics in vertical-grasp mode,
  so only the arm section should remain.
