# Tutorial for PX4CTRL

The node name for the PX4 Controller is PX4CTRL.
It takes Odometry and the aircraft's IMU as feedback, receives desired position and attitude control commands, and provides precise control of the aircraft.

## Brief Introduction to the Controller

While the framework of this PX4CTRL controller is similar to the DJI N3 controller, the internal state machine and control algorithms have been completely rewritten to achieve higher stability and control accuracy.
The controller has three modes:

- **Manual Mode**: The terminal prints in green "MANUAL_CTRL(L1)". At this time, the controller does not function, and the flight control is entirely controlled by the remote control.
- **Fixed Point Mode**: The terminal prints in green "AUTO_HOVER(L2)". The controller will take over flight control through code, and the flight control will be in the "OFFBOARD" mode.
  The controller uses odometry feedback for fixed-point control. At this time, if you toggle the direction and throttle joysticks, the aircraft will move slowly, similar to flying in DJI Mavic's fixed-point mode.
  It will not receive the commands you send.
- **Command Control Mode**: The terminal prints in green "CMD_CTRL(L3)". At this time, the controller can execute the flight commands you send to it.
  This mode requires you to be in fixed-point mode, toggle the 6-channel joystick, and then send the control command to start, to ensure a thorough safety check.

### Coordinate System Definition

Odometry's pose is defined as forward x, left y, and up z. The aircraft's head points in the positive direction of the x-axis, and the throttle pull direction is the positive direction of the z-axis. It must be strictly aligned!
The speed definition of Odometry is slightly different from the official ROS. It's essential to note this. The official ROS defines speed under the body system, but most open-source VIOs define speed in the relative world system.
By default, PX4CTRL is also defined in the world system. If the speed you use is defined under the body system, change the `#define VEL_IN_BODY 0` in `input.cpp` to 1.

### Remote Control Channel Settings

PX4CTRL will subscribe `/mavros/rc/in` topic to read the raw RC input.
You need to set and check these values to triggers in QGC:

- Channel 5: Flight mode switch (Recommend: LOW: Manual mode, MID: Position mode, HIGH: Offboard mode)
- Channel 6: Assign a trigger but do not set any function
- Channel 7: Emergency Stop
- Channel 8: Assign a trigger but do not set any function, recommend assigning a joystick that can auto-rebound. (DEPRECATED)

Their corresponding functionalities in `PX4CTRL` are:

- Channel 5: Switch PX4CTRL control (Offboard mode) or the original flight mode of the flight control
- Channel 6: Whether command control is enabled (LOW: Disabled, HIGH: Enabled). When enabled, PX4CTRL will switch to **Command Control Mode** and execute the commands you send.
- Channel 7: Emergency Stop
- Channel 8: One-click restart when the flight control is not unlocked (often used when selecting the ekf2 estimator in the flight control)

### Trajectory Commands

The trajectory command received by PX4CTRL should be a sequence of waypoints in the world coordinate frame.
It is send to the `~cmd` topic as a `quadrotor_msgs/PositionCommand` message.

```
Header header
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
geometry_msgs/Vector3 acceleration
geometry_msgs/Vector3 jerk
float64 yaw
float64 yaw_dot
float64[3] kx
float64[3] kv

uint32 trajectory_id

uint8 TRAJECTORY_STATUS_EMPTY = 0
uint8 TRAJECTORY_STATUS_READY = 1
uint8 TRAJECTORY_STATUS_COMPLETED = 3
uint8 TRAJECTROY_STATUS_ABORT = 4
uint8 TRAJECTORY_STATUS_ILLEGAL_START = 5
uint8 TRAJECTORY_STATUS_ILLEGAL_FINAL = 6
uint8 TRAJECTORY_STATUS_IMPOSSIBLE = 7

# Its ID number will start from 1, allowing you comparing it with 0.
uint8 trajectory_flag
```

### Controller Topics and Services

Check the `px4ctrl_node.cpp` file for the controller's topics and services.
Modify the two topics in `run_ctrl.launch` to the topic names you use. Use `rqt_graph` to check the topic reception to ensure the controller is connected to:
`/mavros/state`
`/mavros/imu/data`
`/mavros/rc/in`
`/mavros/setpoint_raw/attitude`
`/your_odometry`.
And check the frequency of `/mavros/imu/data` and `/your_odometry` is above 100Hz, `/mavros/rc/in` is around 5Hz.

## Usage

The controller runs at the lowest control precision by default. Here, it's as adaptive as possible, so the default parameters are usually sufficient.

Brief usage of the controller is as follows:

1.  Start mavros and your odometry node:

    ```
    roslaunch mavros px4.launch
    roslaunch <your odometry node>
    ```

2.  Start px4ctrl:

        ```
        roslaunch px4ctrl run_ctrl.launch
        ```

3.  Check while the drone is on the ground and not unlocked. In the not unlocked state of the flight controller, toggle the 5th channel switch.
    If the screen prints a message `[px4ctrl] MANUAL_CTRL(L1) --> AUTO_HOVER(L2)` in green, it means the controller check has passed and switched to the hover mode.
    Then toggle the 6th channel joystick. If the screen prints green "[px4ctrl] TRIGGER sent, allow user command.", it means you can switch to command control mode, allowing external control commands to be accessed.
    If errors occur, correct them according to the displayed errors.

4.  Arm the drone on the ground, manually take off to an appropriate height, toggle the 5th channel remote to switch to auto-hover mode, and then toggle the 6th channel remote to switch to control command reception mode.
    Note: Initially, when unfamiliar with the stability of the odometry, it's recommended to check the positioning for drift before switching modes, otherwise the drone may crush as soon as the mode is switched.

5.  When toggling the 6th channel to the **Command Control Mode**, the controller will publish a message to `/traj_start_trigger` topic, which can be used as a trigger to start the planner.
    It also contains the current pose of the drone.

### Troubleshooting

- Although px4ctrl will print detected errors, sometimes errors are emitted by the flight controller and cannot be detected by px4ctrl, manifesting as inability to take off, no response when switching to hover, etc.
  These in-flight-controller errors will be printed on the QGC ground station.
- If mavros reports a serial port error, check if the serial port has not been granted 777 permissions (sudo chmod 777 /dev/<serial port name>).
  If the serial port opens successfully but there's no response, check if the cable is connected correctly or if the MAV_1_CONFIG parameter of the flight controller is set incorrectly.
- If other errors are reported, check if you missed installing the geographiclib.
- When using ekf2 as the flight controller's state estimate, it's common to encounter the inability to unlock after landing, reporting attitude estimation errors.
  At this time, toggle the remote's 8th channel, and the flight controller will restart, resolving the error.
- If the drone continuously flies up or down after switching to hover mode, excluding positioning drift, it's mostly because the throttle stick hasn't returned to the center.
  Because in hover mode, remote control is allowed. Solution 1: Manually return the throttle stick to the center, as there is a dead zone set near the center; Solution 2: Set max_manual_vel in the config to 0, i.e., disable the remote control function in hover mode.
- If the drone's position fluctuates up and down after switching to hover mode, excluding positioning drift, it's mostly due to the throttle model estimation, which is caused by excessive IMU vibration during flight, leading to inaccurate throttle model feedback.
  Poorly damped IMUs can have accelerometer vibrations between ±5~10m/s^2, while well-damped IMUs (like DJI N3, Lei Xun Nora, X7, etc.) usually don't exceed ±0.3m/s^2.
  You can record an imu bag file during flight to check. Although you can alleviate the issue by adjusting controller parameters, it's still recommended to have proper damping for the entire drone.
  Therefore, the parameter "rho2" is set to be unchangeable. To dampen, buy dedicated IMU sponge dampers and stick them to the four corners of the flight controller. The expected vibration should be less than 2m/s^2.
- Continuing from the previous issue, if you really can't reduce the vibration, you can choose
  - 1: Change the noisy_imu parameter in the config file to true, which will change the acceleration feedback to speed feedback but will significantly sacrifice dynamic performance;
  - 2: If you don't mind steady-state control errors, you can directly comment this parts out from the code
