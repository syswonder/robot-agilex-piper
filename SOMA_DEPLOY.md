# Soma 接入部署 cheatsheet — Piper grasp deploy

> Piper-arm + Orbbec + grasp pipeline 的 soma 接入速查页。设计与决策依据见
> `/Users/howenliu/lab/docs/soma_two_stage_bringup.md`（v2 flat schema
> 是 §12，PR review 意见落地是 §13）。姊妹部署的完整版是
> `/Users/howenliu/lab/ranger_mini_deploy/SOMA_DEPLOY.md`——两个 deploy 的
> Soma 接入姿势对齐，只是 body 描述 / URDF / manifest 内容不同。

## 0. 触发条件

执行下面任何一步之前确认：

- [x] robonix 源码已合入 PR #91（soma 首次落地）+ PR #109（v2 flat schema
      + pipe IPC stage-2 trigger）。这两个 PR 之后：
        * `system/soma/src/{config,store,deployment,launcher,main}.rs`
          是 flat schema；
        * `tools/rbnx/src/cmd/deploy.rs::spawn_soma_binary` 用 `pipe(2)`
          + `dup2` 把读端固定到子进程 fd 3；
        * `capabilities/system/soma/get_yaml.v1.toml` / `get_urdf.v1.toml`
          的 `kind = "service"`（不是 `"system"`）。
- [ ] 当前 Piper 机器人 side 无在跑任务（避免撞坏正在跑的 grasp）。
- [ ] CAN 已 up（`ip link show can_piper` 有 `UP`）；如未 up，先跑
      `bash /Users/howenliu/lab/packages/piper_ctl_rbnx/scripts/can_activate.sh can_piper 1000000 "1-4.4:1.0"`。
- [ ] **垂直抓取版新增前置**：本 manifest 现在把 `llm_detect` /
      `pick` 指向 `branch: feature/vertical-grasp`，`grasp_pose` 使用本地
      `rbnx-boot/cache/grasp_pose_rbnx`。
      这三条分支目前只在本机 `packages/*_rbnx/` 里，**尚未推送到
      GitHub**。`rbnx boot` 会 `git clone -b feature/vertical-grasp`，
      分支不存在会直接 fail。上机前必须：
      ```bash
      for pkg in llm_detect_rbnx pick_skill_rbnx; do
          git -C /Users/howenliu/lab/packages/$pkg push -u origin feature/vertical-grasp
      done
      ```
      合并回 `main` 之后，把 manifest 里三处 `branch: feature/vertical-grasp`
      改回 `branch: main` 即可。
- [ ] **openvla_client 已在本 manifest 关闭**（与 grasp pipeline
      互斥，见 `robonix_manifest.yaml` 里 `openvla_client` 段的注释）。
      如需切回 VLA 演示，反过来把 llm_detect / grasp_pose /
      roboarm_ik / pick 都注释掉、解注释 openvla_client。

## 0.5 启动机制速览（Piper 侧一份，源码事实同 ranger）

```
rbnx boot
  ├─ system builtin 阶段（rbnx 直接 fork）
  │   atlas → executor → pilot → liaison → soma
  │     ↑                                  ↑
  │     bin_map 见 deploy.rs                spawn_soma_binary（PR #109）：
  │                                        pipe(2) + dup2(read_fd, 3)
  │                                        env ROBONIX_SOMA_STAGE_FD=3
  │
  ├─ 非 builtin system 阶段（rbnx spawn + Driver(CMD_INIT)）
  │   本 manifest 目前不启用（memory/scene/speech 都注释掉了）
  │
  ├─ soma stage 1（soma 自己起，rbnx 不管这段）
  │   primitive: orbbec_camera → piper_ctl → piper_description
  │   每个都 spawn + wait_for_registration + CMD_INIT + CMD_ACTIVATE
  │
  ├─ service 阶段（rbnx spawn + Driver(CMD_INIT)）
  │   llm_detect → grasp_pose → roboarm_ik
  │   （openvla_client 已在本 manifest 注释；与 grasp pipeline 互斥）
  │
  ├─ stage 2 trigger
  │   rbnx 起完 service 后 `write_all(b"stage2\n")` 到 soma 的 pipe 写端
  │
  └─ soma stage 2
      本 manifest 的 skill 段：pick。soma spawn pick_skill 并
      CMD_INIT，但 **不** CMD_ACTIVATE —— pick 走 lazy activate，
      等 pilot 首次 MCP 调用时 executor 再发 CMD_ACTIVATE。
```

soma 起来后自己向 atlas 注册两条 gRPC cap（`system/soma/src/main.rs`
`register_soma_services_and_activate`）：

```
robonix/system/soma/get_yaml   Transport::Grpc  port 50091
robonix/system/soma/get_urdf   Transport::Grpc  port 50091
```

## 1. 本 deploy 目录的相关文件

```
rbnx_piper_packages/
├── robonix_manifest.yaml       ← 已加 system.soma: 块
├── soma.yaml                   ← 本机产出（robot=piper_grasp_01, urdf=./urdf/piper.urdf）
├── soma_config.local.yaml      ← 本机产出（v2 flat 四字段）
├── urdf/
│   ├── piper.urdf              ← 从 packages/piper_description_rbnx vendor 过来的 with-gripper 版
│   └── README.md
├── stop.sh                     ← 原有
└── SOMA_DEPLOY.md              ← 你在读的这份
```

## 2. 在部署机上要做的事（按顺序）

### 2.1 编 robonix-soma binary（整套同源 install）

```bash
ssh robot     # 或对应部署机的 ssh 别名
export ROBONIX_SOURCE_PATH=/home/syswonder/wheatfox/robonix   # 按机器实际路径调整
cd "$ROBONIX_SOURCE_PATH"

git fetch origin
git checkout main
git pull --ff-only

make build && make install   # 整套同源 install，避免 atlas/soma 半新半旧

which robonix-soma && robonix-soma --help
```

> ⚠️ 不要单独 `cargo install -p robonix-soma`。要么整套 install，要么不动。
> PR #91 之后 rbnx / atlas / soma 三者的 wire 一起演进，割裂 install 会
> 触发 `unknown key system.soma` 或 pipe fd 传递失败等诡异 case。

### 2.2 把本机产出的全部文件 scp 到部署机

在你的 Mac 上：

```bash
cd /Users/howenliu/lab/rbnx_piper_packages
DEPLOY_REMOTE=robot:~/lhw/rbnx_piper_packages/   # 按部署机上实际路径调整

# 覆盖前建议先备份
ssh robot 'cp ~/lhw/rbnx_piper_packages/robonix_manifest.yaml ~/lhw/rbnx_piper_packages/robonix_manifest.yaml.before-soma 2>/dev/null || true'

scp robonix_manifest.yaml            "$DEPLOY_REMOTE"      # ★ 覆盖
scp soma.yaml                        "$DEPLOY_REMOTE"
scp soma_config.local.yaml           "$DEPLOY_REMOTE"
ssh robot 'mkdir -p ~/lhw/rbnx_piper_packages/urdf'
scp urdf/piper.urdf urdf/README.md   "$DEPLOY_REMOTE/urdf/"
scp SOMA_DEPLOY.md                   "$DEPLOY_REMOTE"
```

### 2.3 静态校验

```bash
ssh robot
cd ~/lhw/rbnx_piper_packages/
rbnx validate
```

如果 `rbnx validate` 报 `unknown key system.soma` 或 `unknown key
robot_yaml`，说明部署机上 `rbnx` 不是 PR #109 之后的版本 → 回到 §2.1
重做 `make install`。

顺手做一次纯 YAML 语法校验（不需要 rbnx，能在网络断开时也跑）：

```bash
python3 -c "import yaml; yaml.safe_load(open('robonix_manifest.yaml'))"
python3 -c "import yaml; yaml.safe_load(open('soma.yaml'))"
python3 -c "import yaml; yaml.safe_load(open('soma_config.local.yaml'))"
```

### 2.4 启动 + 验收

```bash
ssh robot
cd ~/lhw/rbnx_piper_packages/
rbnx boot
```

PR #109 的 rbnx boot 会把 `system.soma:` 翻译成：

```
robonix-soma \
  --listen 127.0.0.1:50091 \
  --atlas 127.0.0.1:50051 \
  --provider-id soma \
  --robot-yaml <abs>/rbnx_piper_packages/soma.yaml \
  --config    <abs>/rbnx_piper_packages/soma_config.local.yaml \
  --log info
```

`spawn_soma_binary` 起进程时会额外 `pipe(2)` + `dup2(read_fd, 3)` +
env `ROBONIX_SOMA_STAGE_FD=3`。**不 chdir**，所以 `--robot-yaml` /
`--config` 必须是绝对路径——rbnx 的 `ensure_soma_defaults` 已经帮你从
`robonix_manifest.yaml` 目录 join 好绝对路径了。

另开一个 ssh 跑验收命令：

```bash
# 1) 文件在
ls soma.yaml soma_config.local.yaml urdf/piper.urdf

# 2) soma 已注册 atlas 上的两条 grpc cap
rbnx caps | grep soma
rbnx caps -v | grep -A 3 'robonix/system/soma'
# 期望：endpoint=127.0.0.1:50091, transport=grpc, state=ACTIVE

# 3) gRPC 直连（用 grpcurl）—— 验证 soma 内部 store 载好了 yaml + urdf
grpcurl -plaintext -d '{"robot_id":""}' 127.0.0.1:50091 \
    robonix.contracts.RobonixSystemSomaGetYaml/GetYaml
# 期望：yaml_text 里包含 "id: piper_grasp_01" 和 "Piper 6-DoF"

grpcurl -plaintext -d '{"robot_id":""}' 127.0.0.1:50091 \
    robonix.contracts.RobonixSystemSomaGetUrdf/GetUrdf
# 期望：urdf_xml 包含 <robot name="piper"> 和 link name="link6"

# 4) primitive 段（soma stage 1 起）应该都 ACTIVE
rbnx caps -v | grep -E 'orbbec_camera|piper_ctl|piper_description'
# 期望：orbbec_camera / piper_ctl 显示 ACTIVE；
#       piper_description 是 capabilities: []，在 atlas 上只有 provider
#       registration 没有 cap，rbnx caps -v 里体现为 provider ACTIVE +
#       no capability rows。

# 5) service 段（rbnx 起，垂直抓取 pipeline 三件套）
rbnx caps -v | grep -E 'llm_detect|grasp_pose|roboarm_ik'
# 期望：三者全部 ACTIVE。
#   * llm_detect  → service/perception/object_detect/*  ACTIVE
#   * grasp_pose  → service/perception/grasp_pose/*     ACTIVE
#   * roboarm_ik  → service/manipulation/execute_grasp ACTIVE

# 6) skill 段：pick 由 soma stage 2 spawn + CMD_INIT，
#              但 CMD_ACTIVATE 由 executor 在首次 MCP 调用时才发。
rbnx caps -v | grep pick
# 期望：INACTIVE（lazy activate）。首次通过 pilot MCP 调用 pick 后
# 再 grep 应变为 ACTIVE。
```

## 3. 出错时

| 症状 | 原因 | 处理 |
|---|---|---|
| `robonix-soma: command not found` | install 失败 / PATH 没刷新 | 回 §2.1 重做 `make install`；`source ~/.cargo/env` |
| `rbnx validate` 报 `unknown key system.soma` 或 `robot_yaml` | rbnx 不是 PR #109 之后版本 | §2.1 重做 |
| `rbnx caps` 看不到 soma 两条 cap | atlas 没起，或 soma 起来后 crash | 看 `rbnx-boot/logs/soma.log`（soma 自己的 stdout/stderr）；常见 crash：`read URDF '<path>': No such file` → §2.2 忘 scp `urdf/piper.urdf` |
| `Error: parse '<path>/soma.yaml'` | soma.yaml 语法错（多半是复制粘贴时把注释里的 `#` 删了） | 用 `python3 -c "import yaml; yaml.safe_load(open('soma.yaml'))"` 快速定位；重新 scp |
| `read URDF '<path>/urdf/piper.urdf': No such file` | urdf/ 没 scp 过去 | 回 §2.2 补 scp `urdf/piper.urdf` |
| `<robot name="piper">` 出现但 `<link name="link6">` 缺失 | URDF 是老版 / 拷错文件 | 从 `/Users/howenliu/lab/packages/piper_description_rbnx/src/piper_description/urdf/piper_description.urdf` 重拷 |
| soma stage 1 卡在 `waiting for provider ... to register` | orbbec 或 piper_ctl 没起来 | 看对应包的 `rbnx-boot/logs/<name>.log`；orbbec 常见 USB 权限，piper_ctl 常见 CAN 没 up |
| stage 2 trigger 后 soma 没起 skill | pick 段被人为再次注释 / feature 分支未推送导致 git clone 失败 | 看 `rbnx-boot/logs/pick.log`；确认 §0 前置条件里的 `feature/vertical-grasp` 已 `git push` 到 origin |
| `rbnx caps` 里 `llm_detect` 报 401 / connection refused | `robonix_manifest.yaml` 里的 `llm_api_key` 无效或 base_url 不通 | 换有效 key；离线联调可临时把 `llm_base_url` 指向本地 OpenAI-compatible 服务 |
| `grasp_pose` 日志报 `hand_eye_calibration_file does not exist` 或 homography 维度错误 | 2D 标定文件缺失 / 路径错 / 文件不是 3x3 homography | 检查 `robonix_manifest.yaml` 里 `grasp_pose.config.hand_eye_calibration_file`，必要时重跑 `tools/calibrate_handeye_2d.py` |
| pick 一直落空 / 撞桌 | 2D homography 已过期，或 `default_desktop_height` 与实际桌面高度不符 | 重新标定 `rbnx-boot/hand-eye-data/2d_homography.npy`；调整 `robonix_manifest.yaml` 里 `grasp_pose.config.default_desktop_height` |

## 4. 后续

- 本 manifest 已启用垂直抓取 pipeline（`llm_detect` + `grasp_pose` +
  `roboarm_ik` + `pick`），`openvla_client` 已注释。若要切回 VLA
  演示：注释掉这四个包、解注释 `openvla_client`，同时把 `soma.yaml`
  的 `description.can_do` / `cannot_do` / `notes` 段一起翻回 VLA 版本
  —— 那些描述是给 pilot 的 LLM 看的，撒谎的代价是 pilot 可能编造出
  实际跑不动的工具组合。
- `llm_detect` / `pick` 的 `feature/vertical-grasp` 分支合并回 `main`
  之后，把对应 `branch:` 改回 `main`；`grasp_pose` 当前走本地 path。
- `grasp_pose.config.default_desktop_height` 是本 pipeline 最关键的物理常量。
  上机前务必用真实桌面高度校准并回写。同时确认
  `hand_eye_calibration_file` 指向最新的 2D homography。
- `pick` 走 lazy activate，`rbnx caps` 会显示 INACTIVE 直到 pilot
  首次 MCP 调用；不用手工预热。
- 若要拿掉 gripper，照 `urdf/README.md` 里的 "Swapping to the
  no-gripper variant" 走；`soma.yaml` 的 tree 里也把 gripper 那节
  删掉，保持描述一致。垂直抓取模式下没有夹爪就没抓取语义，只保
  留 arm 段。
