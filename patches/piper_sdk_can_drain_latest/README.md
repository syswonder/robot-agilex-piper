# Piper SDK CAN receive backlog workaround

This directory contains an English-only workaround for `piper_sdk 0.2.19` that addresses feedback topics drifting further behind real arm motion over time.

## Symptom

In the current ROS2 deployment, the arm may already have moved while feedback topics such as `/arm/end_pose` and `/arm/joint_states_single` still update much later. The longer the process runs, the more obvious the lag becomes.

The root cause is the CAN receive loop inside `ReadCanMessage()`: it blocks for one frame at a time and immediately parses that single frame. If Python-side processing is slower than the incoming CAN frame rate, the socket receive queue keeps accumulating old frames and the SDK continues to process stale feedback.

## Workaround approach

The helper script rewrites `piper_sdk/hardware_port/can_encapsulation.py` so that `ReadCanMessage()`:

- still blocks for the first frame;
- then drains any queued frames with `recv(timeout=0.0)`;
- keeps only the latest frame for each `arbitration_id`;
- finally feeds only those latest frames into the original callback path.

This lets the SDK catch up to the newest feedback state instead of consuming stale feedback frame by frame.

## Files

- `apply_piper_sdk_0.2.19_can_drain_latest.py`

## How to apply

First confirm where the current Python environment loads `piper_sdk` from:

```bash
python3 - <<'PY'
import piper_sdk
print(piper_sdk.__file__)
PY
```

If the output looks like this:

```text
/home/syswonder/.local/lib/python3.10/site-packages/piper_sdk/__init__.py
```

run the helper script:

```bash
python3 /home/syswonder/lhw/rbnx_piper_packages/patches/piper_sdk_can_drain_latest/apply_piper_sdk_0.2.19_can_drain_latest.py
```

The script updates the installed `can_encapsulation.py` in place and writes a `.bak` backup next to it before the first modification.

Check syntax after patching:

```bash
python3 -m py_compile /home/syswonder/.local/lib/python3.10/site-packages/piper_sdk/hardware_port/can_encapsulation.py
```

Restart the Piper / Robonix processes so the change takes effect:

```bash
cd /home/syswonder/lhw/rbnx_piper_packages
bash stop.sh
rbnx boot
```

## Verify

Check whether the feedback topics now track real arm motion promptly:

```bash
ros2 topic echo /arm/end_pose
ros2 topic hz /arm/end_pose
```

If the workaround is active, `/arm/end_pose` should refresh much sooner after manually moving the arm or executing a motion sequence.

## Revert

Use the same helper script with `--revert`:

```bash
python3 /home/syswonder/lhw/rbnx_piper_packages/patches/piper_sdk_can_drain_latest/apply_piper_sdk_0.2.19_can_drain_latest.py --revert
```

You can also reinstall `piper_sdk 0.2.19` to restore the official version.

## Notes

This workaround does not change ROS topic schemas or motion control interfaces. It only changes the SDK's low-level CAN receive strategy so the receiver prioritizes the newest feedback frames.
