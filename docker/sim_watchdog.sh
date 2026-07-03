#!/usr/bin/env bash
# Self-healing watchdog for the DroneTrack SITL camera pipeline. Runs on the HOST.
#
# THE PROBLEM: the Gazebo camera stream occasionally stalls server-side (a fresh
# bridge attached to the same gz server stays starved), so frames stop reaching
# perception while everything else looks alive. Only a full recovery — container
# restart, PX4 relaunch, stack relaunch — reliably clears it.
#
# THE FIX: poll the dashboard's /api/status. When the camera frame age exceeds
# STALL_S while the stack is otherwise up, diagnose which side died (for the
# log), land the drone if it is flying, and run the full recovery unattended.
#
# Usage:
#   docker/sim_watchdog.sh            # daemon: poll forever, recover on stall
#   docker/sim_watchdog.sh --once     # single check+recovery pass, then exit
#   touch /tmp/dronetrack_watchdog_pause   # pause (e.g. during manual work)
#
# Env overrides: DASH (default http://127.0.0.1:8091), STALL_S (25), POLL_S (5),
# CONTAINER (dronetrack-sim), PLAN (orbit_red_ball).
set -u

DASH="${DASH:-http://127.0.0.1:8091}"
STALL_S="${STALL_S:-25}"
POLL_S="${POLL_S:-5}"
CONTAINER="${CONTAINER:-dronetrack-sim}"
PLAN="${PLAN:-orbit_red_ball}"
PAUSE_FILE="/tmp/dronetrack_watchdog_pause"
LOG="/tmp/dronetrack_watchdog.log"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

log() { echo "$(date "+%F %T") $*" | tee -a "$LOG"; }

status_field() {  # status_field <jq-ish field> -> value or empty
    curl -s -m 3 "$DASH/api/status" 2>/dev/null \
      | python3 -c "import json,sys; print(json.load(sys.stdin).get(\"$1\",\"\"))" 2>/dev/null
}

diagnose() {
    # Best-effort forensics BEFORE tearing things down, so the log tells us
    # which layer died this time (gz sensor vs bridge vs px4).
    local px4_alive bridge_alive gz_frames
    px4_alive=$(docker exec "$CONTAINER" bash -c 'pgrep -fc "bin/px4" 2>/dev/null' || echo 0)
    bridge_alive=$(docker exec "$CONTAINER" bash -c 'pgrep -fc "image_bridge|parameter_bridge" 2>/dev/null' || echo 0)
    gz_frames=$(docker exec "$CONTAINER" bash -c \
      't=$(gz topic -l 2>/dev/null | grep -m1 "sensor/camera/image\|sensor/imager/image"); \
       [ -n "$t" ] && timeout 5 gz topic -e -t "$t" -n 3 2>/dev/null | grep -c "sec:" || echo 0')
    log "DIAGNOSIS: px4_procs=$px4_alive bridge_procs=$bridge_alive gz_frames_in_5s=$gz_frames"
    if [ "${gz_frames:-0}" -gt 0 ] && [ "${bridge_alive:-0}" -gt 0 ]; then
        log "DIAGNOSIS: gz still publishing but ROS side starved -> bridge-side hang"
    elif [ "${px4_alive:-0}" -eq 0 ]; then
        log "DIAGNOSIS: PX4 process dead -> px4/gz crash"
    else
        log "DIAGNOSIS: gz camera stopped publishing -> gz-server-side hang"
    fi
}

ensure_container_scripts() {
    # The container's /tmp launchers can be missing on a brand-new container.
    docker exec "$CONTAINER" test -x /tmp/px4_launch.sh 2>/dev/null || \
        docker cp "$SCRIPT_DIR/px4_headless.sh" "$CONTAINER:/tmp/px4_launch.sh"
    docker exec "$CONTAINER" test -x /tmp/stack_launch.sh 2>/dev/null || \
        docker cp "$SCRIPT_DIR/stack_headless.sh" "$CONTAINER:/tmp/stack_launch.sh"
    docker exec "$CONTAINER" chmod +x /tmp/px4_launch.sh /tmp/stack_launch.sh
}

recover() {
    log "RECOVERY: starting full sim recovery (container restart -> PX4 -> stack)"

    # Land first if flying: better a controlled land than yanking the sim
    # out from under an armed drone.
    if [ "$(status_field armed)" = "True" ]; then
        log "RECOVERY: drone armed; requesting land"
        curl -s -m 3 -X POST "$DASH/api/land" -H "Content-Type: application/json" \
             -d '{"confirm":true}' >/dev/null 2>&1
        for _ in $(seq 1 15); do
            sleep 2
            [ "$(status_field armed)" != "True" ] && break
        done
    fi

    docker restart "$CONTAINER" >/dev/null || { log "RECOVERY: container restart FAILED"; return 1; }
    sleep 8
    ensure_container_scripts
    docker exec -d "$CONTAINER" bash -lc 'setsid /tmp/px4_launch.sh </dev/null >/dev/null 2>&1 &'
    sleep 30
    docker exec "$CONTAINER" bash -c 'echo "param set COM_DISARM_LAND 2.0" > /tmp/px4in' 2>/dev/null
    docker exec -d "$CONTAINER" bash -lc "setsid /tmp/stack_launch.sh $PLAN </dev/null >/tmp/stack_launch.out 2>&1 &"
    sleep 45

    local age
    age=$(status_field camera_frame_age_s)
    if python3 -c "exit(0 if 0 <= float('${age:-999}') < 3 else 1)" 2>/dev/null; then
        log "RECOVERY: complete; camera fresh (age=${age}s)"
        return 0
    fi
    log "RECOVERY: FAILED to restore camera (age=${age:-unreachable}); will retry on next stall detection"
    return 1
}

check_once() {
    [ -e "$PAUSE_FILE" ] && return 0
    docker inspect -f '{{.State.Running}}' "$CONTAINER" 2>/dev/null | grep -q true || return 0

    local age
    age=$(status_field camera_frame_age_s)
    # Dashboard down or no frame yet (-1): the stack is starting/stopped on
    # purpose — not the camera hang. Never auto-start a stack the user stopped.
    [ -z "$age" ] && return 0
    python3 -c "exit(0 if float('$age') > $STALL_S else 1)" 2>/dev/null || return 0

    log "STALL DETECTED: camera_frame_age_s=$age > ${STALL_S}s"
    diagnose
    recover
}

if [ "${1:-}" = "--once" ]; then
    check_once
    exit 0
fi

log "watchdog up | dash=$DASH stall>${STALL_S}s poll=${POLL_S}s container=$CONTAINER (pause: touch $PAUSE_FILE)"
while true; do
    check_once
    sleep "$POLL_S"
done
