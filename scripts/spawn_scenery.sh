#!/usr/bin/env bash
# Spawn a ring of tall, distinctly-colored posts around the ball so the orbit is
# visually obvious in the camera feed (an empty ground plane gives no parallax).
# Idempotent: removes any posts it previously spawned before re-adding them.
#
# Run INSIDE the sim container (or via: docker exec dronetrack-sim bash -lc '...').
#   bash scripts/spawn_scenery.sh            # default ring
#   CENTER_X=6 CENTER_Y=0 RING_R=9 N=8 bash scripts/spawn_scenery.sh
# NB: no `set -u` — sourcing ROS setup.bash trips unbound-var checks.

WORLD="${WORLD:-default}"
CENTER_X="${CENTER_X:-6.0}"
CENTER_Y="${CENTER_Y:-0.0}"
RING_R="${RING_R:-9.0}"
N="${N:-8}"
POST_H="${POST_H:-3.0}"
POST_R="${POST_R:-0.3}"

source /opt/ros/jazzy/setup.bash 2>/dev/null || true
source "$HOME/dronetrack_groundstation_ros/ros_ws/install/setup.bash" 2>/dev/null || true

# 8 bright, well-separated colors (red is the ball, so skip it).
COLORS=("1 0.5 0" "1 1 0" "0 1 0" "0 1 1" "0 0.4 1" "0.6 0 1" "1 1 1" "1 0 1")

post_sdf() {  # $1=name  $2="r g b"
  cat <<EOF
<?xml version="1.0" ?>
<sdf version="1.9">
  <model name="$1">
    <static>true</static>
    <link name="l">
      <visual name="v">
        <geometry><cylinder><radius>${POST_R}</radius><length>${POST_H}</length></cylinder></geometry>
        <material><ambient>$2 1</ambient><diffuse>$2 1</diffuse><specular>0.1 0.1 0.1 1</specular></material>
      </visual>
      <collision name="c">
        <geometry><cylinder><radius>${POST_R}</radius><length>${POST_H}</length></cylinder></geometry>
      </collision>
    </link>
  </model>
</sdf>
EOF
}

for i in $(seq 0 $((N-1))); do
  name="scenery_post_${i}"
  # remove a prior instance so re-runs don't error on duplicate names
  gz service -s "/world/${WORLD}/remove" \
    --reqtype gz.msgs.Entity --reptype gz.msgs.Boolean --timeout 1000 \
    --req "name: \"${name}\" type: MODEL" >/dev/null 2>&1 || true

  ang=$(python3 -c "import math;print(2*math.pi*${i}/${N})")
  x=$(python3 -c "import math;print(${CENTER_X}+${RING_R}*math.cos(${ang}))")
  y=$(python3 -c "import math;print(${CENTER_Y}+${RING_R}*math.sin(${ang}))")
  color="${COLORS[$((i % ${#COLORS[@]}))]}"

  ros2 run ros_gz_sim create \
    -world "${WORLD}" -name "${name}" \
    -string "$(post_sdf "${name}" "${color}")" \
    -x "${x}" -y "${y}" -z "$(python3 -c "print(${POST_H}/2)")" \
    >/dev/null 2>&1 &&
    echo "spawned ${name} at (${x%.*}, ${y%.*}) color=[${color}]" ||
    echo "FAILED to spawn ${name}"
done
wait
echo "done: ${N} posts in a ${RING_R} m ring around (${CENTER_X}, ${CENTER_Y}) in world '${WORLD}'"
