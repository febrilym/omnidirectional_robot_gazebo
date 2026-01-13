#!/bin/bash

echo "========================================"
echo "Robot Square Movement Controller Start!"
echo "========================================"

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
    
    # save PID from background process
    local PUB_PID=$!
    
    # duration
    sleep $duration
    
    # stop background
    kill $PUB_PID 2>/dev/null
    wait $PUB_PID 2>/dev/null
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

countdown() {
    local seconds=$1
    echo -n "Start in: "
    for ((i=seconds; i>0; i--)); do
        echo -n "$i "
        sleep 1
    done
    echo "GO!"
}

# movement param
VELOCITY=2.8 # linear velocity (m/s)
ANGULAR_VEL=2.8 # angular velocity for rotation 90° (rad/s)
MOVE_DURATION=13.0 # movement duration
TURN_DURATION=7.82 # rotation duration 90° (/s)
PAUSE_DURATION=0.01 # pause duration (for every movement)

# countdown
countdown 3

echo ""
echo "Start omnidirectional robot movement . . ."
echo ""

# robot movement
# step 1: forward
echo "STEP 1: Move forward"
send_velocity $VELOCITY 0.0 0.0 $MOVE_DURATION

# step 2: stop
echo "STEP 2: Stop"
stop_robot
sleep $PAUSE_DURATION

# step 3: turn right
echo "STEP 3: Turn right 90°"
send_velocity 0.0 0.0 -$ANGULAR_VEL $TURN_DURATION

# step 4: stop
echo "STEP 4: Stop"
stop_robot
sleep $PAUSE_DURATION

# step 5: forward
echo "STEP 5: Move forward"
send_velocity $VELOCITY 0.0 0.0 $MOVE_DURATION

# step 6: stop
echo "STEP 6: Stop"
stop_robot
sleep $PAUSE_DURATION

# step 7: turn right
echo "STEP 7: Turn right 90°"
send_velocity 0.0 0.0 -$ANGULAR_VEL $TURN_DURATION

# step 8: stop
echo "STEP 8: Stop"
stop_robot
sleep $PAUSE_DURATION

# step 9: forward
echo "STEP 9: Move forward"
send_velocity $VELOCITY 0.0 0.0 $MOVE_DURATION

# step 10: stop
echo "STEP 10: Stop"
stop_robot
sleep $PAUSE_DURATION

# step 11: turn right
echo "STEP 11: Turn right 90°"
send_velocity 0.0 0.0 -$ANGULAR_VEL $TURN_DURATION

# step 12: stop
echo "STEP 12: Stop"
stop_robot
sleep $PAUSE_DURATION

# step 13: forward
echo "STEP 13: Move forward"
send_velocity $VELOCITY 0.0 0.0 $MOVE_DURATION

# step 14: stop
echo "STEP 14: Stop"
stop_robot
sleep $PAUSE_DURATION

# step 15: turn right
echo "STEP 15: Turn right 90°"
send_velocity 0.0 0.0 -$ANGULAR_VEL $TURN_DURATION

# step 15: stop
echo "STEP 16: End stop robot"
stop_robot

echo ""
echo "========================================"
echo "Robot Square Movement Doned!"
echo "========================================"