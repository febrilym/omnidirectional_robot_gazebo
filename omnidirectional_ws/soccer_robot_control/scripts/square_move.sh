#!/bin/bash

echo "========================================"
echo "Robot Coordinate Movement Controller Start!"
echo "========================================"

# Konfigurasi kecepatan (m/s)
MAX_VELOCITY=2.8
ANGULAR_VEL=2.8
PAUSE_DURATION=0.5

# Global position (estimasi)
CURRENT_X=0.0
CURRENT_Y=0.0

send_velocity() {
    local linear_x=$1
    local linear_y=$2
    local angular_z=$3
    local duration=$4
    
    echo "Send: linear.x=$linear_x, linear.y=$linear_y, angular.z=$angular_z"
    rostopic pub -1 /robot_1/cmd_vel geometry_msgs/Twist "linear:
  x: $linear_x
  y: $linear_y
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: $angular_z" &
    
    local PUB_PID=$!
    sleep $duration
    kill $PUB_PID 2>/dev/null
    wait $PUB_PID 2>/dev/null
    
    # Update estimated position (open-loop)
    CURRENT_X=$(echo "$CURRENT_X + $linear_x * $duration" | bc -l)
    CURRENT_Y=$(echo "$CURRENT_Y + $linear_y * $duration" | bc -l)
}

stop_robot() {
    echo "Menghentikan robot..."
    rostopic pub -1 /robot_1/cmd_vel geometry_msgs/Twist "linear:
  x: 0.0
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
    sleep 0.5
}

move_to_coordinate() {
    local target_x=$1
    local target_y=$2
    local use_diagonal=${3:-false}
    
    echo "========================================"
    echo "Moving to coordinate: X=$target_x, Y=$target_y"
    echo "Current position: X=$CURRENT_X, Y=$CURRENT_Y"
    
    # Hitung jarak dan arah
    local dx=$(echo "$target_x - $CURRENT_X" | bc -l)
    local dy=$(echo "$target_y - $CURRENT_Y" | bc -l)
    local distance=$(echo "sqrt($dx^2 + $dy^2)" | bc -l)
    
    if [ $(echo "$distance < 0.001" | bc) -eq 1 ]; then
        echo "Already at target position"
        return
    fi
    
    # Mode pergerakan
    if [ "$use_diagonal" = true ]; then
        # Gerakan diagonal langsung
        local angle=$(echo "a($dy/$dx)" | bc -l)  # arctan(dy/dx)
        
        # Tentukan kuadran
        if [ $(echo "$dx < 0" | bc) -eq 1 ]; then
            angle=$(echo "$angle + 3.14159" | bc -l)
        fi
        
        # Hitung komponen kecepatan
        local velocity_x=$(echo "$MAX_VELOCITY * c($angle)" | bc -l)
        local velocity_y=$(echo "$MAX_VELOCITY * s($angle)" | bc -l)
        local duration=$(echo "$distance / $MAX_VELOCITY" | bc -l)
        
        echo "Moving diagonally: Vx=$velocity_x, Vy=$velocity_y, Time=$duration s"
        send_velocity $velocity_x $velocity_y 0.0 $duration
        
    else
        # Gerakan sumbu terpisah (X lalu Y)
        if [ $(echo "$dx != 0" | bc) -eq 1 ]; then
            local duration_x=$(echo "sqrt($dx^2) / $MAX_VELOCITY" | bc -l)
            local velocity_x=$(echo "$dx / sqrt($dx^2) * $MAX_VELOCITY" | bc -l)
            echo "Moving in X: Vx=$velocity_x, Time=$duration_x s"
            send_velocity $velocity_x 0.0 0.0 $duration_x
            stop_robot
            sleep $PAUSE_DURATION
        fi
        
        if [ $(echo "$dy != 0" | bc) -eq 1 ]; then
            local duration_y=$(echo "sqrt($dy^2) / $MAX_VELOCITY" | bc -l)
            local velocity_y=$(echo "$dy / sqrt($dy^2) * $MAX_VELOCITY" | bc -l)
            echo "Moving in Y: Vy=$velocity_y, Time=$duration_y s"
            send_velocity 0.0 $velocity_y 0.0 $duration_y
        fi
    fi
    
    stop_robot
    CURRENT_X=$target_x
    CURRENT_Y=$target_y
    echo "Arrived at: X=$CURRENT_X, Y=$CURRENT_Y"
}

rotate_to_angle() {
    local target_angle_deg=$1  # dalam derajat
    local current_angle_deg=0  # asumsi awal menghadap sumbu X positif
    
    local angle_diff=$(echo "$target_angle_deg - $current_angle_deg" | bc -l)
    local angle_diff_rad=$(echo "$angle_diff * 3.14159 / 180" | bc -l)
    
    # Tentukan arah putaran
    local angular_vel=$ANGULAR_VEL
    if [ $(echo "$angle_diff_rad < 0" | bc) -eq 1 ]; then
        angular_vel=$(echo "-1 * $ANGULAR_VEL" | bc -l)
        angle_diff_rad=$(echo "-1 * $angle_diff_rad" | bc -l)
    fi
    
    local duration=$(echo "$angle_diff_rad / $ANGULAR_VEL" | bc -l)
    
    echo "Rotating to $target_angle_deg degrees: ω=$angular_vel, Time=$duration s"
    send_velocity 0.0 0.0 $angular_vel $duration
    stop_robot
}

countdown() {
    local seconds=$1
    echo -n "Start in: "
    for ((i=seconds; i>0; i--)); do
        echo -n "$i "
        sleep 1
    done
    echo "GO!"
}

# Contoh penggunaan
countdown 3

echo ""
echo "Starting coordinate-based movement..."
echo ""

# Contoh 1: Gerakan diagonal ke (10, 10)
move_to_coordinate 10 0 true

# # Tunggu sebentar
# sleep 2

# # Contoh 2: Gerakan ke (-5, 5) dengan sumbu terpisah
# move_to_coordinate -5 5 false

# Contoh 3: Rotasi 90 derajat
# rotate_to_angle 90

# # Contoh 4: Gerakan diagonal ke (0, 0)
# move_to_coordinate 0 0 true

echo ""
echo "========================================"
echo "Robot Coordinate Movement Completed!"
echo "========================================"