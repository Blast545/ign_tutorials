# Tutorial: Using teleop_twist_keyboard ROS node to control a diff drive robot in Ignition
This tutorial intends to give a step-by-step guide into how to use ROS tools in conjunction with an Ignition based simulation.
You should be able to achieve a result like the one in this [video](https://youtu.be/2ZiRw2ZkTOY) at the end of the tutorial.

We will be using Ignition Dome and ROS Melodic; however, please note that this tutorial is valid for Ignition releases that are compatible with `ros_ign_bridge` and ROS releases that are compatible with `teleop_twist_keyboard` and `ros_ign`.

We'll start with the steps to complete the tutorial, and later will be explained what's going on behind the scenes.

## Pre-Requisites

+ Install Ignition Dome
	+ Instructions can be found here: [Dome Installation](https://ignitionrobotics.org/docs/dome)

+ Install ROS Melodic
	+ Instructions can be found here: [ROS Melodic Installation](http://wiki.ros.org/melodic/Installation)

+ Install `ros_ign`. We'll be using ROS Melodic version
	``` bash
        # Add https://packages.osrfoundation.org
        sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
        wget https://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -
        sudo apt-get update
        # Install `ros_ign`
        sudo apt install ros-melodic-ros-ign
	```

+ Install `teleop_twist_keyboard`
	``` bash
	sudo apt-get install ros-melodic-teleop-twist-keyboard
	```

## Steps to complete this tutorial

### Install this demo package

``` bash
mkdir -p ~/catkin_ws/src
git clone git@github.com:Blast545/ign_tutorials.git ~/catkin_ws/src
cd catkin_ws
catkin_make
```

We will open two different terminals sharing the same environment. In each, we'll run the following commands:

#### Run launchfile with ROS, Ignition and `ros_ign`
```bash
# Shell 1
source ~/catkin_ws/devel/setup.bash
roslaunch roslaunch ign_tutorials diff_drive_demo.launch
```

#### Start `teleop_twist_keyboard` node
```bash
# Shell 4
source ros_ws/devel/setup.bash # Source ROS workspace if the node was installed from source
rosrun teleop_twist_keyboard_cpp teleop_twist_keyboard
```

After running this, you should have an environment similar to this one:
![Selection_092](https://user-images.githubusercontent.com/8069967/113123464-fe509200-91ea-11eb-89a4-6d4a8406e6ef.png)

And in the teleop_twist_keyboard terminal you can send commands to the system to move the vehicle robot:
![Selection_091](https://user-images.githubusercontent.com/8069967/113123514-0ad4ea80-91eb-11eb-989b-46ce8fbaefdc.png)

## Understanding the system architecture

For this tutorial, we've used the example world `tunnel.sdf` that includes a diff drive ground vehicle and shows various features available within the simulator. Including the ignition simulator, the whole system works like this:

![diagrama_tutorial (1)](https://user-images.githubusercontent.com/8069967/113031878-99068d80-9165-11eb-8fce-2e31c104e5eb.png)

* The `teleop_twist_keyboard` node transforms the user inputs into ROS Twist commands. 
* The `ros_ign_bridge` subscribes to those ROS Twist commands and converts them to Ignition Transport msgs.
*  The Ignition Transport msgs are used by diff drive plugin inside the tunnel.sdf that's running in the simulator.
* The simulator moves the robot accordingly, based on the diff drive plugin output.

We can see from the command used for the ros_bridge within the launchfile: 
```xml
  <node
    pkg="ros_ign_bridge"
    type="parameter_bridge"
    name="$(anon ros_ign_bridge)"
    output="screen"
    args="/model/vehicle/odometry@nav_msgs/Odometry@ignition.msgs.Odometry /cmd_vel@geometry_msgs/Twist@ignition.msgs.Twist ">
  </node>
```
The msg input/output goes to the /cmd_vel topic (this topic applies to both the Ignition Transport and ROS), it takes `geometry_msgs/Twist` as input and uses `ignition.msgs.Twist` as output. Same principle applies for the odometry information displayed in `rviz`.

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
* Can you change the launchile to use a different physics engine? Like TPE.
* The `diff_drive.sdf` example world (`ign_ws/src/ign-gazebo/examples/worlds/diff_drive.sdf`) does not use the same `cmd_vel` topic to receive velocity commands, can you modify the commands to make it work in this scenario?

Want to dig deeper inside Ignition gazebo? Browse the following links:

* Browse the main [Ignition portal](https://ignitionrobotics.org/home).
* Browse the official [tutorials for Ignition Dome](https://ignitionrobotics.org/docs/dome).
* Check discussions and announcements in the [Gazebo/Ignition community forum](https://community.gazebosim.org/).
