#!/bin/bash

echo "========================================"
echo "Robot Coordinate Movement Controller Start!"
echo "========================================"

# ================= CONFIG =================
SCALE_FACTOR=100.0
CORRECTION_FACTOR=15.5
MAX_VELOCITY=2.8
ANGULAR_VEL=2.8

ROT_PUB_RATE=144
ROTATION_GAIN=14.8  # <<< FIX UTAMA (tuning yaw)
PAUSE_DURATION=5

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

    rostopic pub -r 144 /robot_1/cmd_vel geometry_msgs/Twist "linear:
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

# ================= GERAKAN SAMPING (LATERAL) =================

move_right() {
    local distance_log=$1
    local distance=$(scale_from_log_with_correction $distance_log)
    local duration=$(echo "$distance / $MAX_VELOCITY" | bc -l)

    echo "========================================"
    echo "Move Right (Lateral)"
    echo "Distance : $distance m"
    echo "Duration : $duration s"
    echo "========================================"

    # REVISI: vy negatif untuk gerak ke KANAN
    send_velocity 0.0 -$MAX_VELOCITY 0.0 $duration
    stop_robot

    # Update posisi (bergerak ke kanan = positif Y) - tetap sama
    CURRENT_Y=$(echo "$CURRENT_Y + $distance" | bc -l)

    echo "Pos est: X=$CURRENT_X Y=$CURRENT_Y"
}

# Gerakan ke kiri (negative Y direction) - DIREVISI
move_left() {
    local distance_log=$1
    local distance=$(scale_from_log_with_correction $distance_log)
    local duration=$(echo "$distance / $MAX_VELOCITY" | bc -l)

    echo "========================================"
    echo "Move Left (Lateral)"
    echo "Distance : $distance m"
    echo "Duration : $duration s"
    echo "========================================"

    # REVISI: vy positif untuk gerak ke KIRI
    send_velocity 0.0 $MAX_VELOCITY 0.0 $duration
    stop_robot

    # Update posisi (bergerak ke kiri = negatif Y) - tetap sama
    CURRENT_Y=$(echo "$CURRENT_Y - $distance" | bc -l)

    echo "Pos est: X=$CURRENT_X Y=$CURRENT_Y"
}

# Gerakan mundur (negative X direction)
move_backward() {
    local distance_log=$1
    local distance=$(scale_from_log_with_correction $distance_log)
    local duration=$(echo "$distance / $MAX_VELOCITY" | bc -l)

    echo "========================================"
    echo "Move Backward"
    echo "Distance : $distance m"
    echo "Duration : $duration s"
    echo "========================================"

    # Bergerak mundur (negative X axis)
    send_velocity -$MAX_VELOCITY 0.0 0.0 $duration
    stop_robot

    # Update posisi (mundur = negatif X)
    CURRENT_X=$(echo "$CURRENT_X - $distance" | bc -l)

    echo "Pos est: X=$CURRENT_X Y=$CURRENT_Y"
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

# ================= POLA-POLA GERAKAN =================

# Square pattern using forward and right movements
move_square_lateral() {
    local side_log=$1

    echo "========================================"
    echo "SQUARE PATTERN (Forward + Right)"
    echo "Side (log): $side_log"
    echo "========================================"

    move_forward $side_log
    sleep $PAUSE_DURATION

    move_right $side_log
    sleep $PAUSE_DURATION

    move_backward $side_log
    sleep $PAUSE_DURATION

    move_left $side_log
    sleep $PAUSE_DURATION
}

# Cross pattern (plus sign)
move_cross_pattern() {
    local arm_log=$1

    echo "========================================"
    echo "CROSS PATTERN"
    echo "Arm length (log): $arm_log"
    echo "========================================"

    move_forward $arm_log
    sleep $PAUSE_DURATION
    
    move_backward $(echo "$arm_log * 2" | bc -l)
    sleep $PAUSE_DURATION
    
    move_forward $arm_log  # Kembali ke tengah
    sleep $PAUSE_DURATION
    
    move_right $arm_log
    sleep $PAUSE_DURATION
    
    move_left $(echo "$arm_log * 2" | bc -l)
    sleep $PAUSE_DURATION
    
    move_right $arm_log  # Kembali ke tengah
    sleep $PAUSE_DURATION
}

# Rectangle pattern (panjang dan lebar berbeda)
move_rectangle() {
    local length_log=$1
    local width_log=$2

    echo "========================================"
    echo "RECTANGLE PATTERN"
    echo "Length (log): $length_log, Width (log): $width_log"
    echo "========================================"

    move_forward $length_log
    sleep $PAUSE_DURATION

    move_right $width_log
    sleep $PAUSE_DURATION

    move_backward $length_log
    sleep $PAUSE_DURATION

    move_left $width_log
    sleep $PAUSE_DURATION
}

# Diamond pattern
move_diamond() {
    local diagonal_log=$1

    echo "========================================"
    echo "DIAMOND PATTERN"
    echo "Diagonal (log): $diagonal_log"
    echo "========================================"

    local side_log=$(echo "$diagonal_log * 0.7071" | bc -l)  # diagonal/sqrt(2)
    
    # Sudut 45 derajat
    rotate_relative 45
    sleep $PAUSE_DURATION
    
    move_forward $side_log
    sleep $PAUSE_DURATION
    
    rotate_relative 90
    sleep $PAUSE_DURATION
    
    move_forward $side_log
    sleep $PAUSE_DURATION
    
    rotate_relative 90
    sleep $PAUSE_DURATION
    
    move_forward $side_log
    sleep $PAUSE_DURATION
    
    rotate_relative 90
    sleep $PAUSE_DURATION
    
    move_forward $side_log
    sleep $PAUSE_DURATION
    
    # Kembali ke orientasi awal
    rotate_relative 45
}

# ================= PATTERN =================
move_square_with_yaw() {
    local side_log=$1

    echo "========================================"
    echo "SQUARE (FORWARD + YAW)"
    echo "Side (log): $side_log"
    echo "========================================"

    move_forward $side_log
    sleep $PAUSE_DURATION

    rotate_relative 90
    sleep $PAUSE_DURATION

    move_forward $side_log
    sleep $PAUSE_DURATION

    rotate_relative 90
    sleep $PAUSE_DURATION

    move_forward $side_log
    sleep $PAUSE_DURATION

    rotate_relative 90
    sleep $PAUSE_DURATION

    move_forward $side_log
    sleep $PAUSE_DURATION
}

# ================= START =================
countdown() {
    for ((i=$1;i>0;i--)); do
        echo -n "$i "
        sleep 1
    done
    echo "GO!"
}

# ================= DEMO GERAKAN =================
demo_lateral_movements() {
    echo ""
    echo "========================================"
    echo "DEMO LATERAL MOVEMENTS"
    echo "========================================"
    
    echo "1. Moving right 100"
    move_right 100
    sleep 2
    
    echo ""
    echo "2. Moving left 100"
    move_left 100
    sleep 2
    
    echo ""
    echo "3. Moving forward 100"
    move_forward 100
    sleep 2
    
    echo ""
    echo "4. Moving backward 100"
    move_backward 100
    sleep 2
}

# ================= MAIN PROGRAM =================
countdown 3

echo ""
echo "Starting movement..."
echo ""

move_forward 212
sleep 5
move_right 127
sleep 5
move_forward 175
# rotate_relative 90

# move_forward 400

# Pilih salah satu pola untuk dijalankan:

# 1. Square dengan rotasi (sebelumnya)
# move_square_with_yaw 300

# 2. Square dengan gerakan lateral (tanpa rotasi)
# move_square_lateral 300

# 3. Demo gerakan lateral
# demo_lateral_movements

# 4. Cross pattern
# move_cross_pattern 200

# 5. Rectangle pattern
# move_rectangle 400 200

# 6. Diamond pattern
# move_diamond 300

# 7. Single movements
# move_forward 400
# move_right 100
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