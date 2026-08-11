#!/usr/bin/env bash

set -eo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LAUNCH_LOG="/tmp/vlm_grasp_full_stack.log"
READY_LOG="/tmp/vlm_grasp_full_stack_ready.log"
launch_pid=""

cleanup() {
    if [[ -n "${launch_pid}" ]] && kill -0 "${launch_pid}" 2>/dev/null; then
        kill -INT "${launch_pid}" 2>/dev/null || true
        for _ in $(seq 1 20); do
            kill -0 "${launch_pid}" 2>/dev/null || break
            sleep 0.5
        done
        kill -TERM "${launch_pid}" 2>/dev/null || true
    fi
    if [[ -n "${launch_pid}" ]]; then
        wait "${launch_pid}" 2>/dev/null || true
    fi
}
trap cleanup EXIT

source /opt/ros/humble/setup.bash
cd "${PROJECT_ROOT}"
source install/setup.bash
set -u
rm -f "${LAUNCH_LOG}" "${READY_LOG}"

ros2 launch ifl_air_ur_launch \
    cell_small_full_mujoco_moveit.launch.py \
    scene_mode:=random \
    sim_headless:=true \
    launch_moveit_rviz:=false \
    launch_robot_rviz:=false \
    >"${LAUNCH_LOG}" 2>&1 &
launch_pid=$!

ready=0
for _ in $(seq 1 55); do
    if grep -q "UR10eRos2Interface started" "${LAUNCH_LOG}" 2>/dev/null; then
        ready=1
        break
    fi
    sleep 1
done

if [[ "${ready}" -ne 1 ]]; then
    cp "${LAUNCH_LOG}" "${READY_LOG}"
    echo "Full stack did not become ready within 55 seconds."
    tail -n 120 "${READY_LOG}"
    exit 1
fi

# Exercise one camera interval, then force the gripper adapter's delayed
# reconnect path now that the simulator socket is accepting connections.
sleep 8
timeout 8s ros2 topic pub --once \
    /robotiq_2f_urcap_adapter/gripper_command_topic \
    control_msgs/msg/GripperCommand \
    "{position: 0.0, max_effort: 140.0}" \
    >/dev/null
sleep 3
cp "${LAUNCH_LOG}" "${READY_LOG}"

grep -E \
    "Selected scene mode|Scene contains|Camera fps|Robotiq server|UR10eRos2Interface started|Connected to URCAP|Activated Gripper" \
    "${READY_LOG}" || true

if grep -qE \
    "gladLoadGL|FatalError|Exception in thread|process has died" \
    "${READY_LOG}"; then
    echo "Full stack reported a fatal startup error."
    tail -n 160 "${READY_LOG}"
    exit 1
fi

if ! grep -q "Connected to URCAP" "${READY_LOG}"; then
    echo "Gripper adapter did not reconnect to the simulator socket."
    tail -n 120 "${READY_LOG}"
    exit 1
fi

echo "Full-stack headless smoke test passed."
