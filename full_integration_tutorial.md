# Tutorial: Using teleop_twist_keyboard ROS node to control a diff drive robot in Ignition

This tutorial intends to give a step-by-step guide into how to use ROS tools in conjunction with an Ignition based simulation. For this tutorial, we will be using Ignition Dome and ROS Melodic, however, please note that this tutorial is valid for Ignition releases compatibles with `ros_ign_bridge` and ROS releases compatibles with `teleop_twist_keyboard` and `ros_ign_bridge`.

First will be presented the steps to complete the tutorial, and later will be explained what's going on behind the scene.

## Pre-Requisites

+ Install Ignition Dome:
	+ Instructions can be found here: [Dome Installation](https://ignitionrobotics.org/docs/dome)
+ Install ROS Melodic
	+ Instructions can be found here: [ROS Melodic Installation](http://wiki.ros.org/melodic/Installation)
+ Install `ros_ign_bridge`. We'll be using ROS Melodic version.
	+ Main repository can be found here: [README ros_ign_bridge](https://github.com/ignitionrobotics/ros_ign/blob/melodic/ros_ign_bridge/README.md)
	+ An overview tutorial for ROS + Ignition integration using `ros_ign_bridge` (with installation instructions can be found here): [Ignition Dome Tutorial: ROS integration](https://ignitionrobotics.org/docs/dome/ros_integration)
+ Install `teleop_twist_keyboard` or `teleop_twist_keyboard_cpp`. For this tutorial we will use the cpp implementation. 
	+ Installation instructions + example: [Wiki teleop_twist_keyboard_cpp](http://wiki.ros.org/teleop_twist_keyboard_cpp)

## Steps to complete this tutorial

We will open four different terminals sharing the same environment. In each, we'll run the following commands:

#### Start `roscore` executable
```bash
# Shell 1
source /opt/ros/melodic/setup.bash # Source ROS installation
roscore
```
#### Start `ign gazebo` GUI simulation with an example world model
```bash
# Shell 2
source ign_ws/install/setup.bash # Source ignition if it was installed from source
ign gazebo -v4 tunnel.sdf
```

#### Start `ros_ign_bridge`
```bash
# Shell 3
source /opt/ros/melodic/setup.bash # Source ROS installation
rosrun ros_ign_bridge parameter_bridge /cmd_vel@geometry_msgs/Twist@ignition.msgs.Twist
```

#### Start `teleop_twist_keyboard` node
```bash
# Shell 4
source ros_ws/devel/setup.bash # Source ROS workspace if the node was installed from source
rosrun teleop_twist_keyboard_cpp teleop_twist_keyboard
```

After running this, you should have an environment similar to this one:

And in the teleop_twist_keyboard terminal you can send commands to the system to move the vehicle robot:

A video demo with the result from this can be seen here:

## Understanding the system architecture

For this tutorial, we've used the example world `tunnel.sdf` that includes a diff drive ground vehicle and shows various features available within the simulator. Including the ignition simulator, the whole system works like this:

<<Inserte diagram>>

* The `teleop_twist_keyboard` node transforms the user inputs into ROS Twist commands. 
* The `ros_ign_bridge` subscribes to those ROS Twist commands and converts them to Ignition Transport msgs.
*  The Ignition Transport msgs are used by diff drive plugin inside the tunnel.sdf that's running in the simulator.
* The simulator moves the robot accordingly, based on the diff drive plugin output.

We can see from the command used for the ros_bridge: 
```bash
rosrun ros_ign_bridge parameter_bridge /cmd_vel@geometry_msgs/Twist@ignition.msgs.Twist
```
The msg input/output goes to the /cmd_vel topic (this topic applies to both the Ignition Transport and ROS), it takes `geometry_msgs/Twist` as input and uses `ignition.msgs.Twist` as output.

Also, taking a look to the SDF file from `ign_ws/src/ign-gazebo/examples/worlds/tunnel.sdf` we can see:

``` xml
<plugin
  filename="ignition-gazebo-diff-drive-system"
  name="ignition::gazebo::systems::DiffDrive">
  <left_joint>left_rear_wheel</left_joint>
  <left_joint>left_front_wheel</left_joint>
  <right_joint>right_rear_wheel</right_joint>
  <right_joint>right_front_wheel</right_joint>
  <wheel_separation>1.25</wheel_separation>
  <wheel_radius>0.3</wheel_radius>
  <odom_publish_frequency>1</odom_publish_frequency>
  <topic>cmd_vel</topic>
</plugin>
```
This is the diff drive plugin configured to receive commands in the `cmd_vel` topic and to control the four joints: `left_rear_wheel`, `left_front_wheel`, `right_rear_wheel` and `right_front_wheel` that control the movement of the ground vehicle.

## Further steps

As homework, can you tackle some challenges to continue the work provided here?

* Try sending commands to the robot using directly the Ignition Transport layer.
* Can you run this tutorial using a different physics engine? Like TPE.
* The `diff_drive.sdf` example world (`ign_ws/src/ign-gazebo/examples/worlds/diff_drive.sdf`) does not use the same `cmd_vel` topic to receive velocity commands, can you modify the commands to make it work in this scenario?
* The Diff::drive plugin also provides odometry information output within the Ignition. Can you run another bridge to get this output into ROS?

Want to dig deeper inside Ignition gazebo? Browse the following links:

* Browse the main [Ignition portal](https://ignitionrobotics.org/home).
* Browse the official [tutorials for Ignition Dome](https://ignitionrobotics.org/docs/dome).
* Check discussions and announcements in the [Gazebo/Ignition community forum](https://community.gazebosim.org/).
