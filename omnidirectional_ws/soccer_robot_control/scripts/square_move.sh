#!/bin/bash

echo "========================================"
echo "Robot Coordinate Movement Controller Start!"
echo "========================================"

# ================= CONFIG =================
SCALE_FACTOR=100.0
CORRECTION_FACTOR=12.84
MAX_VELOCITY=2.8
ANGULAR_VEL=2.8

ROT_PUB_RATE=30
ROTATION_GAIN=14.0  # <<< FIX UTAMA (tuning yaw)
PAUSE_DURATION=2

# ================= STATE =================
CURRENT_X=0.0
CURRENT_Y=0.0
CURRENT_ANGLE_DEG=0.0   # estimasi saja (open-loop)

PI=3.141592653589793

# ================= SCALE =================
scale_from_log_with_correction() {
    echo "scale=6; ($1 / $SCALE_FACTOR) * $CORRECTION_FACTOR" | bc -l
}

scale_to_log_with_correction() {
    echo "scale=6; ($1 / $CORRECTION_FACTOR) * $SCALE_FACTOR" | bc -l
}

# ================= LOW LEVEL =================
stop_robot() {
    rostopic pub -1 /robot_1/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0" > /dev/null
    sleep 0.2
}

send_velocity() {
    local vx=$1
    local vy=$2
    local wz=$3
    local duration=$4

    echo "Send cmd_vel: vx=$vx vy=$vy wz=$wz  time=$duration"

    rostopic pub -r 30 /robot_1/cmd_vel geometry_msgs/Twist "linear:
  x: $vx
  y: $vy
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: $wz" &
    local PID=$!

    sleep $duration
    kill $PID 2>/dev/null
    wait $PID 2>/dev/null
}

# ================= ROTATION =================
rotate_relative() {
    local delta_deg=$1

    echo "========================================"
    echo "Rotate relative: $delta_deg° (RIGHT = +)"
    echo "========================================"

    # === BALIK DEFINISI USER ===
    # +deg = kanan (CW) → angular.z NEGATIF
    local delta_rad=$(echo "-1 * $delta_deg * $PI / 180" | bc -l)

    local wz=$ANGULAR_VEL
    if [ $(echo "$delta_rad < 0" | bc) -eq 1 ]; then
        wz=$(echo "-1 * $ANGULAR_VEL" | bc -l)
    fi

    local duration=$(echo "sqrt($delta_rad^2) * $ROTATION_GAIN / $ANGULAR_VEL" | bc -l)

    echo "ω_cmd : $wz rad/s"
    echo "time  : $duration s"

    rostopic pub -r $ROT_PUB_RATE /robot_1/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: $wz" &
    local PID=$!

    sleep $duration
    kill $PID
    wait $PID 2>/dev/null

    stop_robot

    # === UPDATE ESTIMASI YAW (KANAN = +) ===
    CURRENT_ANGLE_DEG=$(echo "$CURRENT_ANGLE_DEG + $delta_deg" | bc -l)
    CURRENT_ANGLE_DEG=$(echo "$CURRENT_ANGLE_DEG % 360" | bc -l)
    if [ $(echo "$CURRENT_ANGLE_DEG < 0" | bc) -eq 1 ]; then
        CURRENT_ANGLE_DEG=$(echo "$CURRENT_ANGLE_DEG + 360" | bc -l)
    fi

    echo "Estimated yaw: $CURRENT_ANGLE_DEG°"
}

# ================= TRANSLATION =================
move_forward() {
    local distance_log=$1
    local distance=$(scale_from_log_with_correction $distance_log)
    local duration=$(echo "$distance / $MAX_VELOCITY" | bc -l)

    echo "========================================"
    echo "Move Forward"
    echo "Distance : $distance m"
    echo "Duration : $duration s"
    echo "========================================"

    send_velocity $MAX_VELOCITY 0.0 0.0 $duration
    stop_robot

    CURRENT_X=$(echo "$CURRENT_X + $distance * c($CURRENT_ANGLE_DEG*$PI/180)" | bc -l)
    CURRENT_Y=$(echo "$CURRENT_Y + $distance * s($CURRENT_ANGLE_DEG*$PI/180)" | bc -l)

    echo "Pos est: X=$CURRENT_X Y=$CURRENT_Y"
}

# ================= PATTERN =================
move_square_with_yaw() {
    local side_log=$1

    echo "========================================"
    echo "SQUARE (FORWARD + YAW)"
    echo "Side (log): $side_log"
    echo "========================================"

    for i in 1 2 3 4; do
        echo ""
        echo "--- Side $i ---"
        move_forward $side_log
        sleep $PAUSE_DURATION

        echo "--- Yaw 90 deg ---"
        rotate_relative 90
        sleep $PAUSE_DURATION
    done
}

# ================= START =================
countdown() {
    for ((i=$1;i>0;i--)); do
        echo -n "$i "
        sleep 1
    done
    echo "GO!"
}

countdown 3

echo ""
echo "Starting movement..."
echo ""

move_square_with_yaw 300
# rotate_relative 90

echo ""
echo "========================================"
echo "Robot Coordinate Movement Completed!"
echo "========================================"
echo ""
echo "FINAL ESTIMATION:"
echo "X=$CURRENT_X  Y=$CURRENT_Y  Yaw=$CURRENT_ANGLE_DEG°"
echo ""
echo "ROTATION_GAIN = $ROTATION_GAIN"
echo "Tune this if yaw not exactly 90°"
