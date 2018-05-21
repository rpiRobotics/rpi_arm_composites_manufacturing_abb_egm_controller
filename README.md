# rpi_arm_composites_manufacturing_abb_egm_controller

ROS package providing the low level ABB EGM controller for the RPI ARM Quick Start Project on Robotic Composites Assistant. This controller is designed to provide a controller with teleoperative, shared-control, and fully automatic control. It commands the robot using the ABB Externally Guided Motion interface. This controller is capable of accepting FollowJointTrajectAction commands from MoveIt, and will execute these commands. This controller is also capable of stopping motion based on a specified threshold from an ATI Net F/T device.

The launch file `arm_composites_controller.launch` will start the `arm_composites_manufacturing_controller` and the `abb_irc5_rapid_node` interface to the RAPID Web Services. See below for arguments to the launch file.

**WARNING: EXPERIMENTAL PACKAGE. USE AT YOUR OWN (SIGNIFICANT) RISK!**

## Operational Modes

The controller has five operational modes:
* 0: Halt - No motion of robot
* 1: Joint Teleoperative Control through XBox Gamepad - Control each joint independently using the XBox Gamepad. See below for joint mappings.
* 2: Cartesian Teleoperative Control through XBox Gamepad - Control the cartesian motion of the robot using the XBox Gamepad. See below for direction mappings.
* 3: Shared trajectory control - Use the XBox Gamepad to move through a trajectory. This allows the operator to control the speed the robot moves through a trajectory.
* 4: Fully automatic control - The robot will automatically execute trajectories as they are received.

## Topics Published

### `controller_state`

The `controller_state` topic publishes information about the current state of the controller with the message type `rpi_arm_composites_manufacturing_abb_egm_controller/ControllerState`. It contains the following fields:

* `Header header` - The standard ROS message header
* `ControllerMode mode` - The current mode of the controller. See the "Operational Modes" above.
* `string error_msg` - The current error message from the controller. This will be blank if there is currently no error.
* `string[] joint_name` - The name of the joints controlled by this controller.
* `float64[] joint_position` - The current positions of the joints.
* `float64[] joint_command_position` - The commanded positions of the joints.
* `geometry_msgs/Wrench` ft_wrench - The current force/torque measured by the sensor.
* `bool ft_wrench_valid` - True if the current force/torque reading is valid, otherwise false.
* `bool trajectory_valid` - True if there is a valid trajectory available to follow, otherwise false.
* `float64 trajectory_time` - The current time progress through a trajectory. This can be controlled by the gamepad in shared control mode.
* `float64 trajectory_max_time` - The length of the currently loaded trajectory.

### ControllerMode message type

This type is represents the operational mode of the robot, and contains constants representing the different modes.

* `int32 MODE_HALT=0`
* `int32 MODE_JOINT_TELEOP=1`
* `int32 MODE_CARTESIAN_TELEOP=2`
* `int32 MODE_SHARED_TRAJECTORY=3`
* `int32 MODE_AUTO_TRAJECTORY=4`
* `int32 mode` # The mode, set to one of the constants above

### `joint_states`

The standard `joint_states` topic used by MoveIt! and other ROS components to receive the current joint positions.

## Services

### `set_controller_mode`

The `set_controller_mode` service will set the active mode of the controller. It can be called at any time, although this may result in tracking errors if called during fast motion. It has the service type `rpi_arm_composites_manufacturing_abb_egm_controller/SetControllerMode`.

**Request**

* `ControllerMode mode` - The desired mode of the controller
* `float64 speed_scalar` - Scale the speed. 1 =  normal speed, &lt; 1 is slower than real time, &gt; 1 is faster than real time. Value mest be between 0 and 10.
* `float64[] force_torque_stop_threshold` - A force/toruqe threshold to stop motion. This may either be an empty array to disable stopping the motion, or may be a 6 entry spatial force vector. Note that this form places the torque is positions 1-3, and forces in positions 4-6. A zero entry in this array will be ignored. The absolute value of the sensor spatial force vector is compared to this array. If any entry of the sensor spatial force is greater than a nonzero entry in this vector, the motion will be stopped. This array must be cleared by setting it to zero length, or the threshold must be increased before motion can continue.

**Response**

* `success` - True if the request was successful, otherwise false.

## Action Servers

### `joint_trajectory_action`

The standard action interface for sending trajectories to a robot for execution. Normally this is called by MoveIt!

## RAPID services

The auxillary `abb_irc5_rapid_node` provides several services to help with controlling the ABB IRC5 controller. These services are utilized to start RAPID tasks, stop RAPID tasks, and read status information from the robot.

### `rapid/start`

Starts the current RAPID task. This service has the type `rpi_arm_composites_manufacturing_abb_egm_controller/RapidStart`.

**Request**

* `string CYCLE_ASIS="asis"`
* `string CYCLE_ONCE="once"` - Run the RAPID task once.
* `string CYCLE_ONCE_DONE="oncedone"`
* `string CYCLE_FOREVER="forever"` - Run the RAPID task in a loop forever.
* `bool reset_pp` - If True, the RAPID Program Pointer will be set to "Main" before starting.
* `string cycle`  - Set to one of the `CYCLE_` constants. Normally this will be set to "forever".

**Response**

* `bool success` - True if the request was successful, otherwise false

### `rapid/stop`

Stops the current RAPID task. This service has the type `rpi_arm_composites_manufacturing_abb_egm_controller/RapidStop`.

**Request**

*empty*

**Response**

* `bool success` - True if the request was successful, otherwise false

### `rapid/get_digital_io`

Reads a digital I/O signal. This service has the type `rpi_arm_composites_manufacturing_abb_egm_controller/RapidGetDigitalIO`.

**Request**

* `string signal` - The name of the signal to read.

**Response**

* `bool success` - True if the request was successful, otherwise false
* `int32 lvalue` - The logical value of the signal.

### `rapid/get_digital_io`

Sets a digital I/O signal. This service has the type `rpi_arm_composites_manufacturing_abb_egm_controller/RapidSetDigitalIO`.

**Request**

* `int32 lvalue` - The new logical value of the signal.
* `string signal` - The name of the signal to read.

**Response**

* `bool success` - True if the request was successful, otherwise false

### rapid/status

Returns the current status of the controller. This service has the type `rpi_arm_composites_manufacturing_abb_egm_controller/RapidGetStatus`.

**Request**

*empty*

**Response**

* `string CYCLE_ASIS="asis"`
*` string CYCLE_ONCE="once"`
* `string CYCLE_ONCE_DONE="oncedone"`
* `string CYCLE_FOREVER="forever"`
* `string OPMODE_INIT="INIT"           #State init`
* `string OPMODE_AUTO_CH="AUTO_CH"     #State change request for automatic mode`
* `string OPMODE_MANF_CH="MANF_CH"     #State change request for manual mode & full speed`
* `string OPMODE_MANR="MANR"           #State manual mode & reduced speed`
* `string OPMODE_MANF="MANF"           #State manual mode & full speed`
* `string OPMODE_AUTO="AUTO"           #State automatic mode`
* `string OPMODE_UNDEFINED="UNDEFINED" #Undefined`
* `string CTRLSTATE_INIT="init"`
*` string CTRLSTATE_MOTORON="motoron"`
* `string CTRLSTATE_MOTOROFF="motoroff"`
* `string CTRLSTATE_GUARDSTOP="guardstop"`
* `string CTRLSTATE_EMERGENCYSTOP="emergencystop"`
* `string CTRLSTATE_EMERGENCYSTOPRESET="emergencystopreset"`
* `string CTRLSTATE_SYSFAIL="sysfail"`
* `bool success` - True if the request was successful, otherwise false
* `bool running` - True if the RAPID task is running, otherwise false
* `string cycle` - The current cycle mode. Will be one of the `CYCLE_` constants above
* `string opmode` - The current operational mode of the controller. Will be one of the `OPMODE_` constants above.
* `string ctrlstate` - The current state of the controller. Will be one of the `CTRLSTATE_` constants above. Use to detect if the motors are on or if a fault has occurred.

### rapid/read_event_log

Reads the event log of the controller. The event log provides diagnostic information to help determine why an error occured, and also logs the general activity of the robot. This service has the type `rpi_arm_composites_manufacturing_abb_egm_controller/RapidReadEventLog`.

**Request**

*empty*

**Response**

* `bool success` - True if the request was successful, otherwise false
* `RapidEventLogMessage[] messages` - An array of `rpi_arm_composites_manufacturing_abb_egm_controller/RapidEventLogMessage` each containing an individual message.

**RapidEventLogMessage**

This message type contains an individual event log message. See the ABB documentation for more information about these messages.

* `string MSG_TYPE_INFO=1`
* `string MSG_TYPE_WARNING=2`
* `string MSG_TYPE_ERROR=3`
* `int32 msgtype` - The type of message. Will be one of the `MSG_TYPE_` constants above.
* `int32 code` - The message/error code. See the ABB documentation for more information.
* `time tstamp` - The time that the error occured, in the robot`s time zone.
* `string[] args`
* `string title` - Human readable short description of the message.
* `string desc` - Human readable detailed description of the message.
* `string conseqs` - Human readable description of the consequences of this message.
* `string causes` - Human readable description of what caused this message.
* `string actions` - Human readable description of what actions can be taken to fix any errors.

## XBox Gamepad Mapping

### Joint Teloperative Mode

Controller must be in Mode 1 for operation in this mode.

* Enable Motion - LB (Must be held to enable motion)
* Joint 1 - Left Stick Left/Right
* Joint 2 - Left Stick Up/Down
* Joint 3 - Right Stick Up/Down
* Joint 4 - DPad Left/Right
* Joint 5 - DPad Up/Down
* Joint 6 - Right Stick Left/Right

### Cartesian Teleoperative Mode

Controller must be in Mode 2 for operation in this mode.

* Enable Motion - RB (Must be held to enable motion)
* X - Left Stick Left/Right
* Y - Left Stick Up/Down
* Z - Right Stick Up/Down
* Yaw - Right Stick Left/Right
* Pitch - D-Pad Left/Right
* Roll - D-Pad Up/Down

### Shared Trajectory Control

In this mode, the gamepad is used to move the robot through a trajectory specified through MoveIt!

* A - 1/4 speed controlled using Left Stick Up/Down for velocity input
* B - Full speed controlled using Left Stick Up/Down for velocity input
* X - 1/4 speed while held
* Y - Full speed while held


## arm_composites_controller.launch args

`joy_dev` - specify the joystick device to use for teleoperative control.
`start_joy` - set to `false` to disable launching of the /joy node. The /joy node must be launched separately.
`netft_host` - the IP address of the ATI Net F/T sensor. Leave blank if no sensor is available.
`abb_irc5_uri` - The IP address of the ABB IRC5 robot controller.
`abb_irc5_rapid_auto_stop` - Set to `false` to disable the RAPID controller stopping the RAPID program when the `abb_irc_rapid_node; is shutdown.

## License

BSD

