# Tutorial for px4ctrl

The node name for the PX4 Controller is "px4ctrl".
It takes Odometry and the aircraft's IMU as feedback, receives desired position and attitude control commands, and provides precise control of the aircraft.

## Brief Introduction to the Controller

While the framework of this px4ctrl controller is similar to the DJI N3 controller, the internal state machine and control algorithms have been completely rewritten to achieve higher stability and control accuracy.
The controller has three modes:

- **Manual Mode**: The terminal prints in green "MANUAL_CTRL(L1)". At this time, the controller does not function, and the flight control is entirely controlled by the remote control.
- **Fixed Point Mode**: The terminal prints in green "AUTO_HOVER(L2)". The controller will take over flight control through code, and the flight control will be in the "OFFBOARD" mode.
  The controller uses odometry feedback for fixed-point control. At this time, if you toggle the direction and throttle joysticks, the aircraft will move slowly, similar to flying in DJI Mavic's fixed-point mode.
  However, it will not receive the commands you send.
- **Command Control Mode**: The terminal prints in green "CMD_CTRL(L3)". At this time, the controller can execute the flight commands you send to it.
  This mode requires you to be in fixed-point mode, toggle the 6-channel joystick, and then send the control command to start, to ensure a thorough safety check.

The controller has different control options for different precision levels:

- Angle/Angular Speed Control: The former has a lower control accuracy than the latter but is relatively safe, and the aircraft usually won't flip.
- Simple/Precise Throttle Force Model: The former requires fewer parameters and is easier to measure, while the latter requires about half a day to measure data and identify parameters, but the former has lower control accuracy.
- Air Drag Coefficient: It's more effective in high-speed, large-range flights, and the impact is not significant in small range, high mobility flights.

### Coordinate System Definition

Odometry's pose is defined as forward x, left y, and up z. The aircraft's head points in the positive direction of the x-axis, and the throttle pull direction is the positive direction of the z-axis. It must be strictly aligned!
The speed definition of Odometry is slightly different from the official ROS. It's essential to note this. The official ROS defines speed under the body system, but most open-source VIOs define speed in the relative world system. By default, px4ctrl is also defined in the world system. If the speed you use is defined under the body system, change the #define VEL_IN_BODY 0 in input.cpp to 1.

### Remote Control Channel Settings

Set and check in QGC:
Channel 5: Flight mode switch
Channel 6: Assign a joystick but do not set any function
Channel 7: Emergency Stop
Channel 8: Assign a joystick but do not set any function, recommend assigning a joystick that can auto-rebound.

Their corresponding functions in px4ctrl are:
Channel 5: Switch px4ctrl control (Offboard mode) or the original flight mode of the flight control
Channel 6: Whether px4ctrl is allowed to receive the control instructions you send to px4ctrl
Channel 7: Emergency Stop
Channel 8: One-click restart when the flight control is not unlocked (often used when selecting the ekf2 estimator in the flight control)

### Controller Topics and Services

Check the px4ctrl_node.cpp file for the controller's topics and services.
Modify the two topics in run_ctrl.launch to the topic names you use. Use rqt_graph to check the topic reception to ensure the controller is connected to:
`/mavros/state`
`/mavros/imu/data`
`/mavros/rc/in`
`/mavros/setpoint_raw/attitude`
`/your odometry`.
And check the frequency of `/mavros/imu/data` and `/your odometry` is above 100Hz, `/mavros/rc/` in is around 5Hz.
