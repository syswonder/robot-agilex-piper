# robot-agilex-piper

这是一个基于 [Robonix](https://github.com/syswonder/robonix) 的 **AgileX Piper 机械臂 + Orbbec Dabai DCW** 垂直抓取部署工作区。

当前分支保留的是 `pick_ok` 这套包结构：

- `primitive:` `orbbec_camera`、`piper_ctl`、`piper_description`
- `service:` `llm_detect`、`grasp_pose`、`roboarm_ik`
- `skill:` `pick`

和旧的 `main` 分支部署相比，这里使用的是 **`grasp_pose + roboarm_ik`** 这条抓取链路，而不是 `yolo_grasp + piper_moveit`；同时 `robonix_manifest.yaml` 中所有远程包都已经切换到 `syswonder` 组织下的仓库。

> English version: [`README.md`](./README.md)

## 目录说明

- `robonix_manifest.yaml`：顶层部署清单，定义包顺序、上游仓库、运行配置和 catalog 元数据。
- `soma.yaml`：由 `robonix-soma` 提供的机器人 body 描述。
- `soma_config.local.yaml`：本地 soma 运行配置。
- `urdf/`：当前部署使用的 Piper URDF。
- `tools/`：2D 手眼标定与验证工具。
- `patches/`：可选的 Piper SDK CAN backlog 补丁。
- `stop.sh`：停止 `rbnx boot` 拉起进程的收尾脚本。
- `SOMA_DEPLOY.md`：soma 部署与排障清单。

## 当前 package 组合

| 角色 | 实例名 | 上游仓库 | 作用 |
|---|---|---|---|
| primitive | `orbbec_camera` | `syswonder/primitive-orbbec-dabai_dcw-camera-rbnx` | 提供腕部 RGBD 相机能力；当前垂直抓取模式中下游不使用 depth。 |
| primitive | `piper_ctl` | `syswonder/primitive-agilex-piper-arm-rbnx` | Piper 机械臂 CAN 驱动，发布关节状态 / 末端位姿 / 机械臂状态，并接收位置指令。 |
| primitive | `piper_description` | `syswonder/primitive-agilex-piper-description-rbnx` | 提供 Piper TF / URDF 视图。 |
| service | `llm_detect` | `syswonder/service-object-detect-rbnx` | 基于 VLM/LLM 的 2D 目标检测服务。 |
| service | `grasp_pose` | `syswonder/service-grasp-pose-rbnx` | 用 2D 检测结果 + 同步保存的 homography 计算 `arm/base_link` 下的抓取位姿。 |
| service | `roboarm_ik` | `syswonder/service-roboarm-ik-rbnx` | 执行 reset / teach-safe / execute-grasp 这类机械臂动作。 |
| skill | `pick` | `syswonder/skill-pick-vertical-grasp-rbnx` | 面向用户 / Pilot 的抓取技能，负责串起整条 pipeline。 |

`robonix_manifest.yaml` 里的这些包现在都统一使用 `branch: main`。

## 运行链路

一次典型抓取会经过下面这条链：

```text
pilot / executor
  -> pick.pick(object_name)
  -> llm_detect.detect_object      # RGB 图像 -> 目标 bbox
  -> grasp_pose.grasp_request      # bbox -> arm/base_link 抓取位姿
  -> roboarm_ik.execute_grasp      # IK + 动作序列 + 夹爪控制
  -> piper_ctl                     # 真正下发到机械臂
```

当前部署的几个关键点：

- `llm_detect` 以 **垂直抓取模式**运行，`skip_depth=true`。
- `grasp_pose` 依赖 `rbnx-boot/hand-eye-data/2d_homography.npy` 和 `default_desktop_height` 来恢复最终抓取目标。
- `pick` 仍然是 **lazy activate**：开机时只初始化，首次工具调用时才激活。

## 快速使用

### 1. 准备环境变量

至少需要设置 `pilot` 和 `llm_detect` 用到的模型服务参数：

```bash
export VLM_BASE_URL=...
export VLM_API_KEY=...
export VLM_MODEL=...
export PILOT_MODEL=...
```

### 2. 按需准备 CAN

如果继续使用 `auto_can_setup: false`，那么在真机启动前请先把 CAN 接口拉起来。

### 3. 启动部署

```bash
rbnx boot
```

### 4. 停止部署

```bash
bash stop.sh
```

## 标定与运维补充

### 2D 手眼标定

当前抓取链路依赖 `grasp_pose` 读取的 2D homography：

- 采集 / 计算：`python3 tools/calibrate_handeye_2d.py`
- 验证：`python3 tools/test_handeye_2d.py --z -0.19 --orientation-mode vertical`

完整流程见 [`tools/README.md`](./tools/README.md)。

### Piper SDK CAN backlog 补丁

如果发现 Piper 反馈 topic 随运行时间变长越来越滞后，可以参考：

- [`patches/piper_sdk_can_drain_latest`](./patches/piper_sdk_can_drain_latest/README.md)

这个补丁会在 CAN 接收侧尽量追最新帧，减少旧反馈积压。

## Catalog 相关信息

`robonix_manifest.yaml` 现在已经加入 `catalog` 元数据，便于后续接 catalog / 展示工具：

- `catalog.name`：`robonix.robot.agilex.piper_grasp`
- `catalog.version`：`0.2.0`
- `catalog.description` 已明确写成当前 **`pick_ok` 这套 `llm_detect + grasp_pose + roboarm_ik`** 的部署形态，而不是旧 `main` 分支那套 `yolo_grasp + piper_moveit`

## 参考

- [`robonix_manifest.yaml`](./robonix_manifest.yaml)
- [`SOMA_DEPLOY.md`](./SOMA_DEPLOY.md)
- [`soma.yaml`](./soma.yaml)
- [`tools/README.md`](./tools/README.md)
- Robonix 框架：`https://github.com/syswonder/robonix`
