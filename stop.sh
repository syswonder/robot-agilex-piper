#!/usr/bin/env bash
# SPDX-License-Identifier: MulanPSL-2.0
# Tear down everything `bash sim/start.sh` + `rbnx boot` brought up:
# webots compose stack + rviz2 + rtabmap_viz (via `docker rm` on the
# mapping container) + every host process and in-container driver
# spawned by rbnx boot.
#
# Symmetric pair to sim/start.sh: every GUI / process that script
# brings up MUST be killed here, otherwise the next start hits port
# / container-name collisions.
set -euo pipefail

cd "$(dirname "$0")"

echo "[sim/stop] killing host-side robonix processes (atlas / pilot / executor / liaison / rbnx boot)..."
# Every binary spawned by `rbnx boot`'s system: block must be listed here,
# otherwise its TCP port leaks across boot cycles and the next boot fails
# with `listen address ':50081' is taken`. Add new ones to deploy.rs's
# system-bin table AND to this regex.
pkill -9 -f "rbnx boot|rbnx deploy|rbnx start -p|robonix-atlas|robonix-pilot|robonix-executor|robonix-liaison" 2>/dev/null || true

echo "[sim/stop] killing host-side python service zombies (speech / memsearch / scene / audio drivers / nav bridges)..."

# ─── helpers ────────────────────────────────────────────────────────
_kill_term() {
    # arg: cmdline-pattern. TERM the matches, ignore "no match" rc.
    pkill -TERM -f "$1" 2>/dev/null || true
}
_kill_kill() {
    pkill -KILL -f "$1" 2>/dev/null || true
}

echo "[piper-grasp/stop] sending SIGTERM to deploy processes…"

# ─── 1) skill (LLM-facing entry, no children to orphan) ─────────────
_kill_term "pick_skill\.atlas_bridge"

# ─── 2) services (kill these before their primitive deps) ───────────
# piper_moveit owns a `ros2 launch piper_moveit_rbnx.launch.py`
# subprocess group → kill the wrapper AND its launch group.
_kill_term "piper_moveit\.main"
_kill_term "piper_moveit_rbnx\.launch\.py"
_kill_term "moveit_control_node_yolo"          # the cpp grasp executor

_kill_term "yolo_grasp\.main"
_kill_term "yolo_world\.main"
_kill_term "llm_detect\.main"

# ─── 3) primitives ──────────────────────────────────────────────────
# easy_handeye2 + piper_description: atlas_register_and_launch.py
# wrappers AND the ros2-launch children they spawned.
_kill_term "easy_handeye2_rbnx/scripts/atlas_register_and_launch\.py"
_kill_term "easy_handeye2 publish\.launch\.py"
_kill_term "handeye_publisher"

_kill_term "piper_description_rbnx/scripts/atlas_register_and_launch\.py"
_kill_term "piper_urdf\.launch\.py"
# robot_state_publisher: only kill the one bound to our piper URDF.
# (matching just `robot_state_publisher` alone would be too greedy —
#  other deploys on this host might run RSP too.) We rely on the
#  launch file already being killed above; RSP is a child of that
#  launch and will exit on its own. Add a targeted kill only if it
#  doesn't:
# _kill_term "robot_state_publisher.*piper_description"

# piper_ctl: python wrapper + its child `ros2 launch piper start_single_piper.launch.py`
_kill_term "piper_ctl\.main"
_kill_term "start_single_piper\.launch\.py"

# orbbec_camera: python wrapper + its child `ros2 launch orbbec_camera dabai_dcw.launch.py`
_kill_term "orbbec_camera\.main"
_kill_term "dabai_dcw\.launch\.py"

# ─── 4) rbnx system services (atlas / executor / pilot / liaison + memory) ──
# These are the 4 builtin gRPC services + the memory service spawned
# by `rbnx boot` from system: in robonix_manifest.yaml. Match on the
# rbnx-cli's child cmdline shape — `python -m robonix.<svc>.main` or
# similar. Adjust if rbnx-cli renames them.
_kill_term "robonix\.system\.atlas"
_kill_term "robonix\.system\.executor"
_kill_term "robonix\.system\.pilot"
_kill_term "robonix\.system\.liaison"
_kill_term "robonix\.system\.memory"
# rbnx boot itself (the CLI process holding the manifest open). Last
# so it doesn't try to "restart" anything we just killed.
_kill_term "rbnx-cli.*boot"
_kill_term "rbnx[ -]boot"

# ─── 5) wait, then KILL the survivors ───────────────────────────────
sleep 2
echo "[piper-grasp/stop] sending SIGKILL to survivors…"

_kill_kill "pick_skill\.atlas_bridge"
_kill_kill "piper_moveit\.main"
_kill_kill "piper_moveit_rbnx\.launch\.py"
_kill_kill "moveit_control_node_yolo"
_kill_kill "yolo_grasp\.main"
_kill_kill "yolo_world\.main"
_kill_kill "llm_detect\.main"
_kill_kill "easy_handeye2_rbnx/scripts/atlas_register_and_launch\.py"
_kill_kill "easy_handeye2 publish\.launch\.py"
_kill_kill "handeye_publisher"
_kill_kill "piper_description_rbnx/scripts/atlas_register_and_launch\.py"
_kill_kill "piper_urdf\.launch\.py"
_kill_kill "piper_ctl\.main"
_kill_kill "start_single_piper\.launch\.py"
_kill_kill "orbbec_camera\.main"
_kill_kill "dabai_dcw\.launch\.py"
_kill_kill "robonix\.system\.atlas"
_kill_kill "robonix\.system\.executor"
_kill_kill "robonix\.system\.pilot"
_kill_kill "robonix\.system\.liaison"
_kill_kill "robonix\.system\.memory"
_kill_kill "rbnx-cli.*boot"
_kill_kill "rbnx[ -]boot"

# ─── 6) report ──────────────────────────────────────────────────────
sleep 0.5
remaining=$(pgrep -f \
    'pick_skill\.atlas_bridge|piper_moveit\.main|piper_moveit_rbnx\.launch|moveit_control_node_yolo|yolo_grasp\.main|yolo_world\.main|llm_detect\.main|easy_handeye2_rbnx/scripts/atlas_register|piper_description_rbnx/scripts/atlas_register|piper_ctl\.main|orbbec_camera\.main|dabai_dcw\.launch|start_single_piper\.launch|piper_urdf\.launch|easy_handeye2 publish\.launch|handeye_publisher|robonix\.system\.|rbnx-cli.*boot|rbnx[ -]boot' \
    2>/dev/null | wc -l | tr -d ' ')

if [[ "$remaining" -eq 0 ]]; then
    echo "[piper-grasp/stop] all deploy processes terminated."
else
    echo "[piper-grasp/stop] WARN: $remaining process(es) still alive:"
    pgrep -af \
        'pick_skill\.atlas_bridge|piper_moveit\.main|piper_moveit_rbnx\.launch|moveit_control_node_yolo|yolo_grasp\.main|yolo_world\.main|llm_detect\.main|easy_handeye2_rbnx/scripts/atlas_register|piper_description_rbnx/scripts/atlas_register|piper_ctl\.main|orbbec_camera\.main|dabai_dcw\.launch|start_single_piper\.launch|piper_urdf\.launch|easy_handeye2 publish\.launch|handeye_publisher|robonix\.system\.|rbnx-cli.*boot|rbnx[ -]boot' \
        2>/dev/null | sed 's/^/    /'
    echo "[piper-grasp/stop] re-run stop.sh, or kill them manually."
fi

# Quick GPU memory check — visible signal that the kills actually
# released CUDA. nvidia-smi may be absent on non-GPU hosts; that's fine.
if command -v nvidia-smi >/dev/null 2>&1; then
    used_free=$(nvidia-smi --query-gpu=memory.used,memory.free --format=csv,noheader 2>/dev/null | head -1 || true)
    if [ -n "${used_free:-}" ]; then
        echo "[sim/stop] GPU after host-side cleanup: ${used_free}"
    fi
fi

echo "[sim/stop] killing host-side rviz2 wrapper (docker exec into sim)..."
# sim/start.sh launches `bash sim/start_rviz.sh` in the background;
# that script does `docker exec robonix_tiago_sim ... rviz2`. Kill the
# wrapper before the compose-down below so docker doesn't have to GC
# a half-dead exec.
pkill -9 -f "start_rviz.sh|rviz2 -d /tmp/rviz2_default.rviz" 2>/dev/null || true

echo "[sim/stop] killing in-container drivers + sim-side GUIs..."
# Match the actual driver module names spawned by rbnx boot:
#   camera_driver.driver, chassis_driver.driver, lidar_driver.driver,
#   simple_nav.atlas_bridge.
# These leak across boot cycles when rbnx shutdown kills only the host
# wrappers — the python inside the sim container survives and holds
# its gRPC port (50212/50122/...), making the next boot fail with
# "address already in use".
docker exec robonix_tiago_sim sh -c \
  'pkill -9 -f "_driver\\.driver|simple_nav\\.atlas_bridge|nav2_bringup|memsearch_service|rviz2 -d" 2>/dev/null || true' \
  2>/dev/null || true

echo "[sim/stop] removing per-package containers (mapping/scene/explore)..."
# These are spawned by rbnx boot (one container per service package).
# `rbnx shutdown` SIGTERMs the host wrapper, but if docker-stop raced
# or boot crashed, the containers leak and the next boot hits a name
# collision. Force-remove so the next start is clean. This also takes
# rtabmap_viz with it (it's a child of the mapping container).
for ct in robonix_mapping robonix_scene robonix_explore; do
    docker rm -f "$ct" >/dev/null 2>&1 || true
done

echo "[sim/stop] docker compose down (sim container + volumes left intact)..."
docker compose -f compose.yaml down 2>/dev/null || true

echo "[sim/stop] done."
