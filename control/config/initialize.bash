#!/bin/bash

ros2 service call /drone0/set_arming_state std_srvs/srv/SetBool "data: true" 

ros2 service call /drone0/set_offboard_mode std_srvs/srv/SetBool "data: true"

ros2 service call /drone0/platform_takeoff std_srvs/srv/SetBool "data: true"

## CONTROL MODE SERVICE CALL
# ros2 service call /drone0/controller/set_control_mode as2_msgs/srv/SetControlMode "control_mode:
#   header:
#     stamp:
#       sec: 0
#       nanosec: 0
#     frame_id: ''
#   yaw_mode: 1
#   control_mode: 2
#   reference_frame: 1"