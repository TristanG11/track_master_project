# Joystick Control Node

This ROS 2 node allows you to control a mobile robot (e.g., differential drive) using a joystick such as a PS4 controller.

It listens to joystick input using the `pygame` library and publishes velocity commands (`Twist` messages) based on the axes and buttons of the controller. The node also integrates diagnostic feedback to monitor the status of the joystick in real time.


## Features

* Publishes `Twist` messages to control robot motion.
* Supports dynamic enabling/disabling based on a `/cmd_type` topic.
* Configurable joystick mapping (axes and buttons).
* Applies a deadzone to joystick values for better precision.
* Automatically handles joystick connection and disconnection.
* Publishes diagnostic updates with connection timestamps.
* Supports smooth linear and angular speed scaling.

## How it works

* Axes of the joystick are mapped to linear and angular velocities.
* A start/stop button allows enabling or disabling motion commands.
* A diagnostic updater tracks the connection status of the joystick.
* The controller only sends commands if the `/cmd_type` topic equals `"ps4_controller"`.


## Parameters

You can configure topics, speed limits, joystick axes, deadzone threshold, and buttons. This makes the node flexible for various joystick models and robot configurations.

