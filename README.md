# auvsime2
Auv Simulator Experimental 2

To launch

```
ros2 launch auv_description cube_thrust_test.launch.py
```

To activate teleoperation

```
ros2 run auv_description wasd_teleop.py   --ros-args -p topic:=/auve1/force_body -p force:=50.0 -p decay:=1.0 -p rate_hz:=500000000.0
```

force, decay and rate_hz can be edited as you see fit, just change the number in the teleop launch command.