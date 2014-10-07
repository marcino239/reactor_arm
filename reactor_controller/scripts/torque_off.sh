#!/bin/bash

rosservice call /shoulder_yaw_controller/torque_enable False
rosservice call /shoulder_pitch_controller/torque_enable False
rosservice call /elbow_pitch_controller/torque_enable False
rosservice call /wrist_pitch_controller/torque_enable False
rosservice call /wrist_roll_controller/torque_enable False

