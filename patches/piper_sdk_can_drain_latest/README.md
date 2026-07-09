# Piper SDK CAN receive backlog patch

这个目录保存了一个针对 `piper_sdk 0.2.19` 的轻量补丁，用来解决 Piper 反馈 topic 随运行时间变长而逐渐滞后的问题。

## 问题现象

在当前 ROS2 部署中，机械臂实际已经移动，但 `/arm/end_pose`、`/arm/joint_states_single` 等反馈 topic 需要过较长时间才更新；运行越久，延迟越明显。

排查后确认，问题主要来自 Piper SDK 的 CAN 接收逻辑：`ReadCanMessage()` 每轮只阻塞读取并解析一帧 CAN 消息。如果 Python 层解析速度低于 CAN 帧到达速度，socket 接收队列会逐渐积压，SDK 后续解析到的就是旧反馈帧。

## 修复思路

补丁修改 `piper_sdk/hardware_port/can_encapsulation.py` 中的 `ReadCanMessage()`：

- 先保持原逻辑，阻塞读取一帧；
- 然后用 `recv(timeout=0.0)` 非阻塞读空当前 socket 中已经积压的帧；
- 对同一个 `arbitration_id` 只保留最新一帧；
- 最后把这些最新帧交给 SDK 原有回调解析。

这样 SDK 会尽快追到最新反馈状态，而不是逐帧消化旧状态。

## 文件

- `piper_sdk_0.2.19_can_drain_latest.patch`

## 打补丁方法

先确认当前 Python 环境使用的 `piper_sdk` 位置：

```bash
python3 - <<'PY'
import piper_sdk
print(piper_sdk.__file__)
PY
```

如果输出类似：

```text
/home/syswonder/.local/lib/python3.10/site-packages/piper_sdk/__init__.py
```

则进入 `site-packages` 目录并打补丁：

```bash
cd /home/syswonder/.local/lib/python3.10/site-packages
patch -p1 < /home/syswonder/lhw/rbnx_piper_packages/patches/piper_sdk_can_drain_latest/piper_sdk_0.2.19_can_drain_latest.patch
```

检查语法：

```bash
python3 -m py_compile /home/syswonder/.local/lib/python3.10/site-packages/piper_sdk/hardware_port/can_encapsulation.py
```

重启 Piper/Robonix 进程使补丁生效：

```bash
cd /home/syswonder/lhw/rbnx_piper_packages
bash stop.sh
rbnx boot
```

## 验证

观察反馈 topic 是否能及时跟随机械臂实际运动：

```bash
ros2 topic echo /arm/end_pose
ros2 topic hz /arm/end_pose
```

如果补丁生效，手动移动机械臂或执行 MoveIt 动作后，`/arm/end_pose` 应该能较快更新，不再出现越运行越滞后的现象。

## 回退

如果补丁是通过 `patch` 命令应用的，可以在同一目录反向应用：

```bash
cd /home/syswonder/.local/lib/python3.10/site-packages
patch -R -p1 < /home/syswonder/lhw/rbnx_piper_packages/patches/piper_sdk_can_drain_latest/piper_sdk_0.2.19_can_drain_latest.patch
```

也可以重新安装 `piper_sdk 0.2.19` 恢复官方版本。

## 注意

这个补丁不会修改 ROS topic 格式，也不会修改运动控制接口。它只改变 SDK 底层 CAN 接收策略，让接收逻辑优先追最新反馈帧。
