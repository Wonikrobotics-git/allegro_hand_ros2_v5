
# Allegro Hand V5 ROS2

This is the official release to control Allegro Hand V5 with ROS2(**Only V5 supported, V4 is not supported**). Mostly, it is based on the release of Allegro Hand V5 ROS1 package.
You can find our latest Allegro Hand V5 ROS1 package :(https://github.com/Wonikrobotics-git/allegro_hand_ros_v5)
And the interfaces and controllers have been improved and rewritten by Soohoon Yang(Hibo) from Wonik Robotics.

We have integrated the core elements of two ROS packages(allegro_hand_description, allegro_hand_parameters) into a main package(allegro_hand_controllers).

**For now, we support one additional action.**
- Visualize Allegro Hand V5 changing in real time using Rviz2.

**We plan to update with more actions as they are developed:**
- Save customize pose using MOVEIT2 and move to the saved pose.
- Simply control hand with GUI tool instead of using keyboard.

These packages are tested on ROS2 Foxy(Ubuntu 20.04) and Humble(Ubuntu 22.04). It will likely not work with newer versions(Rolling ...).

## Useful Links
- Official Allegro Hand Website : https://www.allegrohand.com/
- Community Forum :  https://www.allegrohand.com/forum

## Packages

**From Allegro Hand V5, the hand is fully based on torque controller.** 

- allegro_hand_controllers : Contain two main nodes for control the hand and urdf descriptions,meshes.
	- node : Receive encoder data and compute torque using `computeDesiredTorque`.
	- grasp : Apply various pre-defined grasps or customized poses.
- allegro_hand_driver : Main driver for sending and receiving data with the Allegro Hand.
- allegro_hand_keyboard : Node that sends the command to control Allegro Hand. All commands need to be pre-defined.
- bhand : Library files for the predefined grasps and actions., available on 64 bit versions.

## Install the PCAN driver

**With ROS2, you don't need to install PCAN driver anymore!**

If you have already installed PCAN driver, please follow instructions below.
- [Optional] Disable previously installed PEAK-CAN driver.
~~~bash
# Temporarily
sudo rmmod pcan
sudo modprobe peak_usb

# Permanent
sudo rm /etc/modprobe.d/pcan.conf
sudo rm /etc/modprobe.d/blacklist-peak.conf
~~~

## Run main controller nodes

1. Make new workspace.
~~~bash
mkdir allegro_ws
~~~

2. Install necessart package.
~~~bash
sudo apt-get update
sudo apt-get install ros-<distro>-xacro
~~~

3. Download ROS2 package for Allegro Hand V5 using below command.
~~~bash
cd ~/allegro_ws
git clone https://github.com/Wonikrobotics-git/allegro_hand_ros2_v5.git
~~~

4. Build.
~~~bash
cd ~/allegro_ws
source /opt/ros/<distro>/setup.bash
colcon build
~~~

5. Launch allegro main node.
~~~bash
source install/setup.bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right TYPE:=A
~~~
**Please check 'Launch file instructions below'.**

6. Run allegro hand keyboard node.
~~~bash
cd ~/allegro_ws
source install/setup.bash
ros2 run allegro_hand_keyboards allegro_hand_keyboard
~~~

7. Control Hand using Keyboard command.

## Launch file instructions

Same as the ROS1 package, you can simply control Allegro Hand V5 by launching *allegro_hand.launch.py* . At a minimum, you must specify the handedness and the hand type:
~~~bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right|left TYPE:=A|B
~~~

Optional arguments:
~~~
VISUALIZE:=true|false (default is false)
~~~

- If you want to visualize Allegro Hand on Rviz2:
~~~bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right TYPE:=A VISUALIZE:=true
~~~

## Control more than one hand

To control more than one hand, you must specify CAN port number and Allegro Hand number.

Terminal 1:
~~~bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=right TYPE:=A CAN_DEVICE:=can0 NUM:=0
~~~

Termianl 2:
~~~bash
ros2 launch allegro_hand_controllers allegro_hand.launch.py HAND:=left TYPE:=A CAN_DEVICE:=can1 NUM:=1
~~~

To control first hand,

Terminal 3:
~~~bash
ros2 run allegro_hand_keyboards allegro_hand_keyboard --ros-args allegroHand_0/lib_cmd:=allegroHand_0/lib_cmd
~~~

To control second hand,

Terminal 4:
~~~bash
ros2 run allegro_hand_keyboards allegro_hand_keyboard --ros-args allegroHand_0/lib_cmd:=allegroHand_1/lib_cmd
~~~


