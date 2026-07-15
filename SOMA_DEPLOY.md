# Soma integration deployment cheatsheet — Piper grasp deploy

> Quick reference for bringing up Soma with the Piper-arm + Orbbec + grasp pipeline. Design notes and rationale live in
> `/Users/howenliu/lab/docs/soma_two_stage_bringup.md` (the v2 flat schema notes are in §12 and the PR review resolution is in §13). The fuller sibling deploy note is
> `/Users/howenliu/lab/ranger_mini_deploy/SOMA_DEPLOY.md` — the two deploys use the same Soma integration pattern, with different body descriptions, URDFs, and manifest contents.

## 0. Preconditions

Before doing anything below, confirm all of the following:

- [x] The `robonix` source tree already includes PR #91 (initial Soma landing) and PR #109 (v2 flat schema + pipe IPC stage-2 trigger). After those PRs:
      * `system/soma/src/{config,store,deployment,launcher,main}.rs` uses the flat schema;
      * `tools/rbnx/src/cmd/deploy.rs::spawn_soma_binary` uses `pipe(2)` + `dup2` to pin the read end to child fd 3;
      * `capabilities/system/soma/get_yaml.v1.toml` and `get_urdf.v1.toml` use `kind = "service"` instead of `"system"`.
- [ ] No task is currently running on the Piper robot side.
- [ ] CAN is already up (`ip link show can_piper` shows `UP`). If not, run
      `bash /Users/howenliu/lab/package_syswonder/primitive-agilex-piper-arm-rbnx/scripts/can_activate.sh can_piper 1000000 "1-4.1:1.0"` first.
- [ ] **Repository source check**: every remote package in this manifest now points to the `syswonder/*` repositories and uses `branch: main`.
      The deploy host no longer depends on a local `feature/vertical-grasp` branch and no longer requires a personal fork branch to be pushed first.
      If you need to pin a temporary debug version, edit the `url:` or `branch:` field for that package directly in `robonix_manifest.yaml`.
- [ ] **`openvla_client` stays disabled in this manifest** because it conflicts with the grasp pipeline.
      See the commented `openvla_client` section in `robonix_manifest.yaml`.
      To switch back to the VLA demo, comment out `llm_detect`, `grasp_pose`, `roboarm_ik`, and `pick`, then uncomment `openvla_client`.

## 0.5 Startup flow at a glance (Piper side; same source-level facts as ranger)

```
rbnx boot
  ├─ builtin system stage (forked directly by rbnx)
  │   atlas → executor → pilot → liaison → soma
  │     ↑                                  ↑
  │     bin_map in deploy.rs               spawn_soma_binary (PR #109):
  │                                        pipe(2) + dup2(read_fd, 3)
  │                                        env ROBONIX_SOMA_STAGE_FD=3
  │
  ├─ non-builtin system stage (rbnx spawn + Driver(CMD_INIT))
  │   not enabled in this manifest for now
  │   (memory / scene / speech remain commented out)
  │
  ├─ soma stage 1 (started by soma itself; rbnx does not manage this part)
  │   primitive: orbbec_camera → piper_ctl → piper_description
  │   each one is spawned, waits for registration, then gets CMD_INIT + CMD_ACTIVATE
  │
  ├─ service stage (rbnx spawn + Driver(CMD_INIT))
  │   llm_detect → grasp_pose → roboarm_ik
  │   (`openvla_client` stays commented out in this manifest; it conflicts with the grasp pipeline)
  │
  ├─ stage 2 trigger
  │   after rbnx finishes services it writes `stage2\n` to soma through the pipe write end
  │
  └─ soma stage 2
      skill section in this manifest: pick. Soma spawns `pick_skill` and sends
      CMD_INIT, but it does **not** send CMD_ACTIVATE. `pick` uses lazy activate
      and is activated by executor on the first pilot MCP call.
```

After Soma comes up, it registers two gRPC capabilities to atlas in `system/soma/src/main.rs::register_soma_services_and_activate`:

```
robonix/system/soma/get_yaml   Transport::Grpc  port 50091
robonix/system/soma/get_urdf   Transport::Grpc  port 50091
```

## 1. Relevant files in this deploy directory

```
rbnx_piper_packages/
├── robonix_manifest.yaml       ← includes the `system.soma:` block
├── soma.yaml                   ← local output (robot=piper_grasp_01, urdf=./urdf/piper.urdf)
├── soma_config.local.yaml      ← local output (v2 flat four-field schema)
├── urdf/
│   ├── piper.urdf              ← copied from the description package's with-gripper variant
│   └── README.md
├── stop.sh                     ← existing stop helper
└── SOMA_DEPLOY.md              ← this file
```

## 2. Tasks to perform on the deploy host, in order

### 2.1 Build the `robonix-soma` binary (full same-source install)

```bash
ssh robot     # or the matching ssh alias for your deploy host
export ROBONIX_SOURCE_PATH=/home/syswonder/wheatfox/robonix   # adjust to the real path on that machine
cd "$ROBONIX_SOURCE_PATH"

git fetch origin
git checkout main
git pull --ff-only

make build && make install   # install the whole stack from one source snapshot

which robonix-soma && robonix-soma --help
```

> Warning: do not run `cargo install -p robonix-soma` by itself.
> Either install the full stack together or leave it untouched.
> After PR #91, the wire protocol between `rbnx`, `atlas`, and `soma` evolves together, and a split install can trigger weird failures such as `unknown key system.soma` or broken pipe-fd handoff.

### 2.2 Copy the locally generated files to the deploy host

On your Mac:

```bash
cd /Users/howenliu/lab/rbnx_piper_packages
DEPLOY_REMOTE=robot:~/lhw/rbnx_piper_packages/   # adjust to the real target path on the deploy host

# Backup before overwriting is recommended
ssh robot 'cp ~/lhw/rbnx_piper_packages/robonix_manifest.yaml ~/lhw/rbnx_piper_packages/robonix_manifest.yaml.before-soma 2>/dev/null || true'

scp robonix_manifest.yaml            "$DEPLOY_REMOTE"      # overwrite intentionally
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

If `rbnx validate` reports `unknown key system.soma` or `unknown key robot_yaml`, the deploy host is still running an `rbnx` version older than PR #109. Go back to §2.1 and redo `make install`.

It is also useful to run a pure YAML syntax check that does not depend on `rbnx`:

```bash
python3 -c "import yaml; yaml.safe_load(open('robonix_manifest.yaml'))"
python3 -c "import yaml; yaml.safe_load(open('soma.yaml'))"
python3 -c "import yaml; yaml.safe_load(open('soma_config.local.yaml'))"
```

### 2.4 Boot and acceptance check

```bash
ssh robot
cd ~/lhw/rbnx_piper_packages/
rbnx boot
```

In PR #109, `rbnx boot` translates `system.soma:` into:

```
robonix-soma \
  --listen 127.0.0.1:50091 \
  --atlas 127.0.0.1:50051 \
  --provider-id soma \
  --robot-yaml <abs>/rbnx_piper_packages/soma.yaml \
  --config    <abs>/rbnx_piper_packages/soma_config.local.yaml \
  --log info
```

`spawn_soma_binary` also applies `pipe(2)` + `dup2(read_fd, 3)` and sets `ROBONIX_SOMA_STAGE_FD=3`.
It does **not** `chdir`, so `--robot-yaml` and `--config` must be absolute paths. `rbnx` already resolves those paths from the `robonix_manifest.yaml` directory in `ensure_soma_defaults`.

Open another ssh session for acceptance commands:

```bash
# 1) Files are present
ls soma.yaml soma_config.local.yaml urdf/piper.urdf

# 2) Soma registered its two gRPC capabilities in atlas
rbnx caps | grep soma
rbnx caps -v | grep -A 3 'robonix/system/soma'
# Expect: endpoint=127.0.0.1:50091, transport=grpc, state=ACTIVE

# 3) Direct gRPC check with grpcurl: verify soma loaded both YAML and URDF
grpcurl -plaintext -d '{"robot_id":""}' 127.0.0.1:50091 \
    robonix.contracts.RobonixSystemSomaGetYaml/GetYaml
# Expect: yaml_text contains "id: piper_grasp_01" and "Piper 6-DoF"

grpcurl -plaintext -d '{"robot_id":""}' 127.0.0.1:50091 \
    robonix.contracts.RobonixSystemSomaGetUrdf/GetUrdf
# Expect: urdf_xml contains <robot name="piper"> and link name="link6"

# 4) Primitive stage (started by soma stage 1) should be ACTIVE
rbnx caps -v | grep -E 'orbbec_camera|piper_ctl|piper_description'
# Expect: orbbec_camera / piper_ctl are ACTIVE.
#         piper_description exposes capabilities: [], so atlas only shows provider
#         registration; `rbnx caps -v` shows provider ACTIVE and no capability rows.

# 5) Service stage (started by rbnx; the three-piece vertical-grasp pipeline)
rbnx caps -v | grep -E 'llm_detect|grasp_pose|roboarm_ik'
# Expect: all three are ACTIVE.
#   * llm_detect  → service/perception/object_detect/*  ACTIVE
#   * grasp_pose  → service/perception/grasp_pose/*     ACTIVE
#   * roboarm_ik  → service/manipulation/execute_grasp  ACTIVE

# 6) Skill stage: pick is spawned by soma stage 2 and only initialized.
#    CMD_ACTIVATE is sent later by executor on the first MCP call.
rbnx caps -v | grep pick
# Expect: INACTIVE (lazy activate). After the first pilot MCP call to pick,
#         it should become ACTIVE.
```

## 3. When things fail

| Symptom | Likely cause | What to do |
|---|---|---|
| `robonix-soma: command not found` | install failed or PATH was not refreshed | Go back to §2.1 and rerun `make install`; also `source ~/.cargo/env` |
| `rbnx validate` reports `unknown key system.soma` or `robot_yaml` | `rbnx` is older than PR #109 | Redo §2.1 |
| `rbnx caps` does not show the two Soma capabilities | atlas is down, or Soma crashed after startup | Check `rbnx-boot/logs/soma.log`; a common crash is `read URDF '<path>': No such file` when `urdf/piper.urdf` was not copied in §2.2 |
| `Error: parse '<path>/soma.yaml'` | `soma.yaml` syntax error, often caused by accidental edits while copying | Run `python3 -c "import yaml; yaml.safe_load(open('soma.yaml'))"` to pinpoint it, then copy again |
| `read URDF '<path>/urdf/piper.urdf': No such file` | `urdf/` was not copied to the target host | Go back to §2.2 and copy `urdf/piper.urdf` |
| `<robot name="piper">` appears but `<link name="link6">` is missing | the URDF is outdated or the wrong file was copied | Re-copy from `/Users/howenliu/lab/package_syswonder/primitive-agilex-piper-description-rbnx/src/piper_description/urdf/piper_description.urdf` |
| Soma stage 1 stalls on `waiting for provider ... to register` | `orbbec_camera` or `piper_ctl` did not come up | Check `rbnx-boot/logs/<name>.log`; common causes are USB permission issues for Orbbec and CAN not being up for `piper_ctl` |
| After the stage 2 trigger, Soma never starts the skill | the `pick` section was commented again, or `skill-pick-vertical-grasp-rbnx` failed to fetch/build | Check `rbnx-boot/logs/pick.log`; confirm the deploy host can access GitHub and that `robonix_manifest.yaml` points to the intended `pick` repository and branch |
| `llm_detect` reports 401 or connection refused in `rbnx caps` / logs | `llm_api_key` is invalid or `llm_base_url` is unreachable | Use a valid key; for offline integration testing, point `llm_base_url` to a local OpenAI-compatible service |
| `grasp_pose` reports `hand_eye_calibration_file does not exist` or a homography shape error | the 2D calibration file is missing, the path is wrong, or the file is not a 3x3 homography | Check `grasp_pose.config.hand_eye_calibration_file` in `robonix_manifest.yaml`; rerun `tools/calibrate_handeye_2d.py` if needed |
| pick keeps missing the target or colliding with the desk | the 2D homography is stale, or `default_desktop_height` does not match the real desk height | Recalibrate `rbnx-boot/hand-eye-data/2d_homography.npy` and update `grasp_pose.config.default_desktop_height` in `robonix_manifest.yaml` |

## 4. Follow-up notes

- This manifest currently enables the vertical-grasp pipeline (`llm_detect` + `grasp_pose` + `roboarm_ik` + `pick`) and comments out `openvla_client`.
  If you want to switch back to the VLA demo, comment out those four packages, uncomment `openvla_client`, and also revert the `description.can_do`, `cannot_do`, and `notes` sections in `soma.yaml` so the pilot-side LLM sees an accurate capability description.
- This manifest now uniformly points to the `syswonder/*` repositories on `main`.
  If you need to validate unpublished work temporarily, edit the matching package `branch:` or `url:` directly.
- `grasp_pose.config.default_desktop_height` is the most important physical constant in this pipeline.
  Calibrate it against the real desk height before deployment, and make sure `hand_eye_calibration_file` points to the newest 2D homography.
- `pick` uses lazy activate, so `rbnx caps` shows it as INACTIVE until the first pilot MCP call.
  No manual warmup is required.
- If you need to remove the gripper, follow `urdf/README.md` under "Swapping to the no-gripper variant".
  Update the tree section in `soma.yaml` accordingly so the robot description remains consistent. In vertical-grasp mode, a no-gripper setup removes the actual grasping semantics and leaves only the arm portion.
