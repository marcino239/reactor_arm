Ros platform for Phantom reactor arm based on the following examples:

```
github.com/a-price      reactor files
github.com/davetcoleman clam arm
```

launch-sequence:

```
export ROS_WS=...
make ros-indigo-run

xfce4-terminal
roscore
roslaunch reactor_gazebo reactor_world.launch 
roslaunch reactor_controller reactor_control.launch
roslaunch reactor_description display.launch 				
```

joint command:
```
rostopic pub -1 /reactor_arm/elbow_pitch_position_controller/command std_msgs/Float64 "data: 1.5"
```
