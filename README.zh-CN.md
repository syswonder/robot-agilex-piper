# rbnx_piper_packages — Piper 垂直抓取部署

基于 [robonix](https://github.com/syswonder/robonix) 框架的
**AgileX Piper 6 自由度机械臂 + Orbbec Dabai DCW RGBD** 抓取 pipeline
的部署工作区。本目录是一个 *manifest-only* 部署：这里**不含任何真实
package 源码**，每个包都是 `rbnx boot` 时按 manifest 从 GitHub 拉取的。
本目录只装：

- `robonix_manifest.yaml` — 顶层清单，按依赖顺序列出所有 package 及其运行时配置。
- `soma.yaml` + `urdf/piper.urdf` + `soma_config.local.yaml` — 供
  `robonix-soma` 服务的机体描述与 URDF（详见 [SOMA_DEPLOY.md](./SOMA_DEPLOY.md)）。
- `stop.sh` — 收尾脚本，杀掉 `rbnx boot` 起来的所有进程。

迁移背景请看
[`/Users/howenliu/lab/docs/PIPER_PIPELINE_MIGRATION_PLAN.md`](../docs/PIPER_PIPELINE_MIGRATION_PLAN.md)。
Soma 接入速查页见 [SOMA_DEPLOY.md](./SOMA_DEPLOY.md)。

> English version: [README.md](./README.md)

---

## 1. 总览

一条 `rbnx boot` 命令会拉起 **8 个 package**（按 primitive / service /
skill 三层组织），外加 `robonix` 系统 builtin
（`atlas` / `executor` / `pilot` / `liaison` / `soma`）。运行起来后，
pipeline 对外只暴露一个高层 MCP 工具 —— `robonix/skill/pick/pick`
—— 供 pilot LLM 按物体名调用抓取。

### 顶层数据流

```
                 Pilot LLM   ─── "把梳子拿起来"  ────┐
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
                          │(VLM 2D 检测)     │ │(top-down │ │(MoveIt 执行) │
                          │                  │ │ 位姿)    │ │              │
                          └────────┬─────────┘ └────┬─────┘ └──────┬───────┘
                                   │ ROS topic     │ TF 查询       │
                                   ▼                ▼              ▼
                    ┌────────────┐ ┌───────────────┐ ┌──────────────────┐
                    │orbbec_cam. │ │ easy_handeye2 │ │    piper_ctl     │
                    │(RGB/Depth) │ │ (link6 → cam) │ │  (CAN 驱动)      │
                    │            │ │piper_desc.    │ │                  │
                    │            │ │(URDF /tf)     │ │                  │
                    └────┬───────┘ └───────────────┘ └────────┬─────────┘
                         │ USB                                │ USB-CAN
                         ▼                                    ▼
                    Orbbec Dabai DCW                    AgileX Piper 机械臂
```

### 抓取几何（垂直模式）

本 pipeline 当前**只支持自顶向下的垂直抓取**：

1. `llm_detect` 拿到一帧 RGB + 物体名（如 `"comb"`），把请求发给 VLM
   得到 2D bbox。
2. `yolo_grasp` 用 bbox 中心像素通过相机内参反投影出一条射线，与
   `arm/base_link` 系下的 `z = z_table` 平面求交，得到 `(x, y, z_table)`。
   姿态锁定为 roll=π、pitch=0；yaw 取常量或 `atan2(y, x)`
   （`radial_yaw: true`）。深度流不使用（`skip_depth: true`）。
3. `pick_skill` 执行三段动作：**pre-grasp 悬停 → 下降并闭合夹爪 →
   post-grasp 抬升**，悬停量由 `approach_dist` 控制。
4. `piper_moveit` 跑 MoveIt 规划 + 内置的 C++ `moveit_control_node_yolo`
   在真机上执行。

---

## 2. Package 清单

manifest 中的顺序即为启动顺序。表中 `provider_id` 与
`robonix_manifest.yaml` 里 `- name:` 一致，也是 `rbnx caps` 显示的名字。

### 2.1 Primitive 层（硬件 + TF）

| # | provider_id | 上游仓库 | atlas 上的能力 | 用途 |
|---|---|---|---|---|
| 1 | `orbbec_camera`     | `lhw2002426/OrbbecSDK_rbnx`          | `primitive/camera/{driver,rgb,depth,camera_info}` | 包装 Orbbec `dabai_dcw.launch.py`。发布 ROS 2 topic `/camera/color/image_raw`、`/camera/depth/image_raw`、`/camera/color/camera_info`。等到第一帧 RGB 后才 ACTIVE。 |
| 2 | `piper_ctl`         | `lhw2002426/piper_ctl_rbnx`          | `primitive/arm/{driver,joint_states,arm_status,end_pose,pos_cmd}` | 包装 `piper start_single_piper.launch.py`（CAN 驱动）。发布 `/arm/joint_states_single`、`/arm/arm_status`、`/arm/end_pose`；订阅 `/arm/pos_cmd`。 |
| 3 | `piper_description` | `lhw2002426/piper_description_rbnx`  | *(无 atlas 能力 — launch wrapper)* | 用内置的 Piper URDF 跑 `robot_state_publisher`，把 `joint_states` 重映射到 `/arm/joint_states_single`。发布**无前缀**的关节 TF 子树 `base_link → link1..link6`。 |
| 4 | `easy_handeye2`     | `lhw2002426/easy_handeye2_rbnx`      | *(无 atlas 能力 — launch wrapper)* | 把标定文件 stage 到 `~/.ros2/easy_handeye2/calibrations/<name>.calib`，再跑 `easy_handeye2 publish.launch.py` 广播手眼静态 TF `link6 → camera_color_optical_frame`。 |

### 2.2 Service 层（算法 + 运动）

| # | provider_id | 上游仓库 | atlas 上的能力 | 用途 |
|---|---|---|---|---|
| 5 | `llm_detect`   | `lhw2002426/llm_detect_rbnx` (分支 `feature/vertical-grasp`) | `service/perception/object_detect/{driver, detect_object}` | 用 OpenAI 兼容的 VLM 做 2D 开放词表检测。消费 `orbbec_camera` 的 RGB + camera_info。**替代**了老的 `yolo_world_rbnx`（GPU/YOLOE），本节点无需 GPU。 |
| 6 | `yolo_grasp`   | `lhw2002426/yolo_grasp_rbnx` (分支 `feature/vertical-grasp`) | `service/perception/grasp_pose/{driver, grasp_request, grasps}` | 拿到 2D bbox 后，把相机射线与 `arm/base_link` 下 `z=z_table` 平面求交得到顶端抓取位姿。本分支**不用深度**。 |
| 7 | `piper_moveit` | `lhw2002426/piper_moveit_rbnx`       | `service/manipulation/{driver, execute_grasp}` | 跑 MoveIt `move_group` + 内置 C++ `moveit_control_node_yolo`。MCP handler 把 `execute_grasp(PoseStamped, gripper_width, timeout)` 桥接到 ROS topic `/graspnet/grasps`，并轮询 `/arm/arm_status` 的 busy→idle。 |

### 2.3 Skill 层（LLM 入口）

| # | provider_id | 上游仓库 | atlas 上的能力 | 用途 |
|---|---|---|---|---|
| 8 | `pick` | `lhw2002426/pick_skill_rbnx` (分支 `feature/vertical-grasp`) | `skill/pick/{driver, pick}` | pilot LLM 真正调用的**唯一**工具。编排 `detect_object` → `grasp_request`（可重试）→ `execute_grasp`（垂直抓取的三段 pre/grasp/post 序列）。 |

### 2.4 系统 builtin

由 `rbnx boot` 从 manifest 的 `system:` 段直接起，不走 per-package 仓库，
随 `rbnx setup` 指定的 robonix 源码树一起安装：

- **`atlas`**（`127.0.0.1:50051`）— 能力注册中心。每个包启动时都在此注册，
  再按 contract id 查依赖 endpoint。
- **`executor`**（`127.0.0.1:50061`）— MCP 工具调用路由；向 provider 发
  `Driver(CMD_INIT / CMD_ACTIVATE)`。
- **`pilot`**（`127.0.0.1:50071`）— LLM 聊天前端（`rbnx ask "..."`）。
  配置在 manifest 的 `system.pilot.vlm` 里（`upstream` URL + `api_key` +
  `model`）。
- **`liaison`**（`127.0.0.1:50081`）— 把外部聊天入口桥接到 pilot。
- **`soma`**（`127.0.0.1:50091`）— 机体描述服务，拥有
  `system/soma/{get_yaml, get_urdf}`。同时驱动**两阶段 bring-up**：
  `soma` 自己在 stage 1 起 `primitive:` 段、在 stage 2 起 `skill:` 段
  （等 `rbnx` 起完 `service:` 段后通过 pipe 写 `stage2\n` 触发）。
  详见 [`docs/soma_two_stage_bringup.md`](../docs/soma_two_stage_bringup.md)。

---

## 3. Package 间通信

两条相互独立的通道并行运行 —— **atlas / MCP** 承载 LLM 面对的接口，
**ROS 2 topic / TF** 承载硬件与运动控制回路。两者只在必要处相交。

### 3.1 Atlas 契约（MCP over HTTP + gRPC）

每个持有能力的 package 都会向 `127.0.0.1:50051` 的 atlas gRPC 服务
注册自己。消费方按 contract id（而不是 host:port）查询 endpoint。
例：`pick_skill` 在 `on_activate` 时向 atlas 解析三个 URL：

- `robonix/service/perception/object_detect/detect_object`
- `robonix/service/perception/grasp_pose/grasp_request`
- `robonix/service/manipulation/execute_grasp`

然后各自用 FastMCP HTTP client 调用。pilot LLM 通过同一机制发现所有
`mcp` 工具。

### 3.2 ROS 2 topic（数据面）

所有硬件相关流量走本地 DDS 域上的 ROS 2 topic。关键的几个：

```
生产者                            Topic                                         消费者
──────────                        ─────                                         ──────────
orbbec_camera                    /camera/color/image_raw                       llm_detect
orbbec_camera                    /camera/color/camera_info                     yolo_grasp
orbbec_camera                    /camera/depth/image_raw                       (垂直模式下不用)

piper_ctl                        /arm/joint_states_single                      piper_description、piper_moveit (move_group)
piper_ctl                        /arm/arm_status                               piper_moveit (busy/idle 轮询)
piper_ctl                        /arm/end_pose                                 (诊断用)
piper_ctl                        (订阅) /arm/joint_states                     ← piper_moveit fake ros2_control_node (命令路径)

piper_description                /tf   base_link → link1..link6                所有做 tf2 查询的节点
easy_handeye2                    /tf_static  link6 → camera_color_optical_frame yolo_grasp、piper_moveit (cpp)
piper_moveit                     /tf_static  arm/link6 ↔ link6 identity 桥     piper_moveit (cpp)

yolo_grasp                       /graspnet/grasps                              piper_moveit (cpp moveit_control_node_yolo)
piper_moveit (MCP handler)       /graspnet/grasps                              piper_moveit (cpp) — MCP → topic 桥
```

两个容易踩坑的点：

- **`/arm/joint_states` vs `/arm/joint_states_single`**：`piper_ctl`
  *发布反馈* 到 `/arm/joint_states_single`；`piper_moveit` 的 launch
  fork 把 move_group planner remap 到这个 topic，同时又跑一个 fake
  `ros2_control_node` 把**命令**轨迹发到 `/arm/joint_states`（`piper_ctl`
  订阅这个 topic 并转成 CAN）。两个不同的 topic、方向也不同。
- **`/graspnet/grasps` 有两个 publisher**：`yolo_grasp`（服务传统的
  直接 topic 调用者）和 `piper_moveit` 自己的桥（服务 MCP `execute_grasp`
  调用者）。两者共用同一个 C++ 订阅端；cpp 里用 `is_busy_` 互斥锁串行化。

### 3.3 TF 树（启动完成后的期望形态）

```
无前缀子树                                有前缀子树 (MoveIt)
(来自 piper_description +                 (来自 piper_moveit launch fork)
 easy_handeye2)

base_link                                 arm/world
   └─ link1..link6                           └─ arm/base_link
         └─ camera_color_optical_frame             └─ arm/link1..arm/link6
                                                              ▲
                                                              │ identity 静态 TF
                                                              │ （piper_moveit
                                                              │  加的桥）
         link6 ────────────────────────────────────────────── arm/link6
```

启动后验证：

```bash
ros2 run tf2_ros tf2_echo arm/base_link camera_color_optical_frame
# 应打印一条稳定的复合变换。
```

---

## 4. 配置 —— 上机前**必须**改的几处

一台新部署机上，永远需要关注的就是下面这几处。其它配置项都有可用默认值。

### 4.1 手眼标定（`easy_handeye2`）

上游仓库里 `packages/easy_handeye2_rbnx/config/calibrations/my_eih_calib.calib`
出厂是 **identity 单位矩阵占位符**。用占位符跑，就等于假设相机安在
`link6` 原点、朝向和 `link6` 一致 —— 每次抓取的位置都会大偏。

`robonix_manifest.yaml` 顶部的 `env:` 段目前指向部署机上的绝对路径：

```yaml
env:
  EASY_HANDEYE2_CALIB_PATH: /home/syswonder/.ros2/easy_handeye2/calibrations/my_eih_calib.calib
```

把它改成你部署机上标定结果的实际路径；或者干脆不写这一行，让包退回
到 `EASY_HANDEYE2_CALIB_NAME`（默认 `my_eih_calib`），也就是
`<easy_handeye2_rbnx>/config/calibrations/my_eih_calib.calib`。

现场重标流程：

```bash
# 在机器人上，先把 piper_ctl + orbbec_camera + piper_description 起来：
ros2 launch easy_handeye2 calibrate.launch.py \
    calibration_type:=eye_in_hand \
    name:=my_eih_calib \
    robot_base_frame:=base_link \
    robot_effector_frame:=link6 \
    tracking_base_frame:=camera_color_optical_frame \
    tracking_marker_frame:=<ArUco marker id>

# 挪机械臂到 10–20 个不同姿态，各点一次 "Take Sample"，然后
# "Compute" → "Save"。上游会写到：
#   ~/.ros2/easy_handeye2/calibrations/my_eih_calib.calib
```

完整流程见 [`easy_handeye2_rbnx` README](https://github.com/lhw2002426/easy_handeye2_rbnx)
的 "Replacing the calibration" 一节。

### 4.2 机械臂初始位置

Piper 不会在启动时自动回到 home。每个 session 第一次 `rbnx boot` 之前，
先把机械臂物理挪到一个满足下列条件的位置：

1. **落在垂直抓取的可达工作空间内** —— 目标物离 `base_link` 若
   `sqrt(x² + y²) > ~0.42 m`，就超出 Piper 臂展，MoveIt 会返回
   `GOAL_STATE_INVALID`。
2. **下降 `approach_dist`（默认 0.10 m）时不撞桌面或夹具**。
3. **腕部相机能看到工作区** —— VLM 只能识别相机视野里的东西。

需要下发指定 home 姿态时，最省事的方式是 `rbnx boot` 前手工跑
`piper_ctl` 再 `ros2 topic pub` 一个 `piper_msgs/PosCmd` 到
`/arm/pos_cmd`，或者用 AgileX 的示教工具。一个安全示例（原
`skill/pick/pick.py` 用的位姿）：

```bash
ros2 topic pub --once /arm/pos_cmd piper_msgs/msg/PosCmd \
    "{x: 0.30, y: 0.0, z: 0.25, roll: 0.0, pitch: 1.57, yaw: 0.0,
      gripper: 0.05, mode1: 0, mode2: 0}"
```

`yolo_grasp.config.z_table` 这个桌面高度常量**必须**反映 `arm/base_link`
系下的真实桌面 z：

```bash
# 让夹爪指尖触桌，然后：
ros2 run tf2_ros tf2_echo arm/base_link arm/link6
# 读 z 分量填到 robonix_manifest.yaml 的：
#   yolo_grasp.config.z_table
# （manifest 当前带一个示例测量值 -0.186）
```

`z_table` 填错要么抓空要么撞桌 —— 这是整套 pipeline 里**最关键的物理
常量**。

### 4.3 ROS 2 topic 名（一般不需要改）

`llm_detect` 和 `yolo_grasp` 通过 atlas 从 `orbbec_camera` 那里解析
输入 topic。只要 `orbbec_camera` 用默认 `camera_name: camera`
（manifest 的现值），下游 topic 名和上游代码硬编码的完全对上：

- `/camera/color/image_raw`
- `/camera/color/camera_info`
- `/camera/depth/image_raw`（垂直模式下不用）

如果要改 Orbbec 的 ROS namespace（比如并排跑两个相机），就**必须**在
`llm_detect` 和 `yolo_grasp` 的 config 里也逐个覆盖 `*_topic`。manifest
里所有可覆盖的 topic 参数都留了注释；留空即接受 atlas 解析出来的默认值。

被下游 C++ 代码写死的 topic 名（**不能通过 config 改**）：

- `/graspnet/grasps` —— `moveit_control_node_yolo` 硬订这个字面字符串。
- `/arm/joint_states` / `/arm/joint_states_single` —— `piper_ctl` 上游
  硬编码 `namespace='/arm'`。

不要重命名这些 topic。

### 4.4 Piper CAN 拉起

每次 `rbnx boot` 前，`can_piper` 必须处于 up 状态。两种途径：

```bash
# 路径 A（推荐；不把 sudo 耦合进 boot 流程）：
cd /path/to/piper_ctl_rbnx     # rbnx boot 会 clone 到 ~/.cache/rbnx/…
bash scripts/can_activate.sh can_piper 1000000 "1-4.4:1.0"
# 把 "1-4.4:1.0" 换成你机器上真实的 USB 总线路径。
# 用 `lsusb -t` 定位。
```

或者在 `piper_ctl.config` 里设 `auto_can_setup: true` 并给操作员配
免密 sudo（路径 B —— 见 `piper_ctl_rbnx` README）。

### 4.5 LLM API 凭据

两个独立的 LLM：

- **pilot 的聊天 LLM** —— manifest 里 `system.pilot.vlm`。用于
  `rbnx ask "..."`。
- **`llm_detect` 的 VLM** —— `llm_detect.config.llm_*`。用于 2D 检测。

都用 OpenAI 兼容 API。manifest 里带着示例 key，**上机前替换成你自己的**
（示例 key 未必长期有效）。

---

## 5. 运行

### 5.1 一次性主机准备

```bash
# 1. clone robonix 源码，安装 rbnx + 所有系统 builtin：
git clone https://github.com/syswonder/robonix ~/robonix
cd ~/robonix
make build && make install     # 把 rbnx + robonix-atlas/executor/pilot/liaison/soma 装到 PATH

# 2. 让 rbnx 认识这份源码树（这样它才知道去哪找 system builtin）：
rbnx setup     # 交互式；或 `rbnx path root set ~/robonix`

# 3. 拉这个部署工作区：
git clone https://github.com/... rbnx_piper_packages
cd rbnx_piper_packages

# 4. 准备好标定文件（见 §4.1）：
mkdir -p ~/.ros2/easy_handeye2/calibrations
cp /path/to/your/calibrated.calib ~/.ros2/easy_handeye2/calibrations/my_eih_calib.calib

# 5. 编辑 robonix_manifest.yaml，设定：
#    - env.EASY_HANDEYE2_CALIB_PATH   (§4.1)
#    - yolo_grasp.config.z_table      (§4.2)
#    - system.pilot.vlm.api_key       (§4.5)
#    - llm_detect.config.llm_api_key  (§4.5)
```

### 5.2 每次 session 拉起

```bash
# 1. 拉起 CAN：
bash /path/to/piper_ctl_rbnx/scripts/can_activate.sh can_piper 1000000 "1-4.4:1.0"

# 2. 静态校验 manifest：
cd /path/to/rbnx_piper_packages
rbnx validate

# 3. 拉起整套：
rbnx boot
# 顺序：atlas / executor / pilot / liaison / soma 起来 →
# soma stage 1 依次启 orbbec_camera → piper_ctl → piper_description → easy_handeye2 →
# rbnx 起 llm_detect → yolo_grasp → piper_moveit →
# soma stage 2 起 pick（只 CMD_INIT，ACTIVATE 延后）。

# 4. 另开一个 shell 验收：
rbnx caps
# 期望：orbbec_camera / piper_ctl / llm_detect / yolo_grasp / piper_moveit 全 ACTIVE；
# piper_description / easy_handeye2 是仅注册（registration-only）→ 显示 ACTIVE；
# pick 是 INITIALIZED（lazy activate）。

# 5. 通过 pilot LLM 触发一次抓取：
rbnx ask "请帮我把桌上的梳子拿起来"
# pilot 应该自动选中 `robonix/skill/pick/pick(object_name="comb")` 并跑完整个 pipeline。

# 6. 收尾：
bash /path/to/rbnx_piper_packages/stop.sh
```

### 5.3 直接 MCP 调用（绕过 pilot）

debug 时 LLM 老选错工具就直接 curl：

```bash
# 从 `rbnx caps -v` 或启动日志里找 pick_skill 的端口：
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

## 6. 常见问题排查

| 症状 | 可能原因 | 处理 |
|---|---|---|
| `rbnx validate` 报 `unknown key system.soma` 或 `robot_yaml` | 部署机上 `rbnx` 版本旧于 PR #109（v2 flat soma schema） | 在 robonix 源码根重跑 `make install`；见 [SOMA_DEPLOY.md](./SOMA_DEPLOY.md) §2.1 |
| `rbnx caps` 里 `pick` 一直 `INITIALIZED` 不 ACTIVE | 符合预期 —— `pick` 是 lazy activate。首次 MCP 调用会触发 CMD_ACTIVATE。 |
| `pick skill cannot find dependencies on atlas: missing [...]` | `llm_detect` / `yolo_grasp` / `piper_moveit` 之一没起来 | `rbnx caps` 找到缺失的那个，看它的 `rbnx-boot/logs/<name>.log` |
| MoveIt 返回 `GOAL_STATE_INVALID` | 目标位姿超出 Piper 可达范围（臂展约 0.42 m）。见 §4.2 | 把目标物挪到 `sqrt(x²+y²) < 0.4 m` 范围内，或调小 `approach_dist` |
| 执行端报告 success，但机械臂物理不动 | `piper_moveit` launch fork 里的 fake `ros2_control_node` 没有把 `joint_states` remap 到 `/arm/joint_states` | 抓取时 `ros2 topic hz /arm/joint_states` 应有明显飙升；确认没人手改过 launch 把 remap 隔离掉 |
| `yolo_grasp` 日志报 `TF lookup arm/base_link ← camera_color_optical_frame timed out` | `easy_handeye2` 没 ACTIVE，或 `link6 ↔ arm/link6` identity 桥没起 | `rbnx caps -v \| grep easy_handeye2` 应 ACTIVE；`ros2 run tf2_ros tf2_echo link6 arm/link6` 应是 identity |
| `orbbec_camera` 哨兵超时 | USB 权限问题 / 相机型号不对 | `lsusb` 检查设备；确认 Orbbec 的 udev 规则已装；本部署只支持 Dabai DCW |
| `piper_ctl` 哨兵超时 | CAN 没起 | 重跑 `can_activate.sh`；`ip link show can_piper` 验证 |
| `llm_detect` 返 401 / connection refused | API key 过期或 `llm_base_url` 不通 | 更新 `llm_detect.config.llm_api_key` / `llm_base_url` |
| 所有节点看着正常，但抓取偏 ~10 cm | `z_table` 错了或手眼标定不对 | 重测 `z_table`（§4.2）；重新标定手眼（§4.1） |

要更深入排查，每个 package 都会在部署目录下 `rbnx-boot/logs/<name>.log`
里留 stdout/stderr。pilot 的对话历史与工具调用轨迹在 `~/.robonix/memory/`。

---

## 7. 替代模式 —— VLA（当前未启用）

manifest 里 `openvla_client` 段是注释掉的。它提供一个闭环 VLA
（Vision-Language-Action）策略，以 2 Hz 直接往 `/arm/pos_cmd` 上写。
**它不能与垂直抓取 pipeline 同时跑** —— 两者都会去争 `/arm/pos_cmd`
的控制权。

切换到 VLA：

1. 把 `llm_detect`、`yolo_grasp`、`piper_moveit`、`pick` 都注释掉。
2. 解注释 `openvla_client`，把 `vla_server_url` 指向你的 VLA server
   （action 范围见 `vla_client_rbnx` README）。
3. 同步更新 `soma.yaml` 的 `description.can_do` / `cannot_do` / `notes`，
   让 pilot LLM 看到的能力描述与实际匹配。

后续会重构为 `robonix_manifest.grasp.yaml` + `robonix_manifest.vla.yaml`
两份，用 symlink 切换。

---

## 8. 参考资料

- 迁移历史：[`../docs/PIPER_PIPELINE_MIGRATION_PLAN.md`](../docs/PIPER_PIPELINE_MIGRATION_PLAN.md)
- Soma 接入速查页：[`./SOMA_DEPLOY.md`](./SOMA_DEPLOY.md)
- 两阶段 bring-up 设计：[`../docs/soma_two_stage_bringup.md`](../docs/soma_two_stage_bringup.md)
- 各 package 源码：<https://github.com/lhw2002426/>（共 8 个仓库）
- robonix 框架：<https://github.com/syswonder/robonix>

各个 package 上游仓库的 README 里详细写了各自的算法、launch 结构、故障
模式。哪个 package 出问题，先去看它自己的 README。
