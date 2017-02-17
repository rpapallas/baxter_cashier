**Note:** This repository is hosting a Final Year Project at the University of Leeds. This project is in progress and hence the code is not final.

Table of Contents
=================

  * [Table of Contents](#table-of-contents)
  * [About The Project](#about-the-project)
  * [Installation](#installation)
    * [Prerequisites](#prerequisites)
    * [Clone Project](#clone-project)
    * [Important `cob_people_perception` modification](#important-cob-people-perception-modification)
  * [Running](#Running)
    * [Run just the Skeleton Tracker](#run-just-the-skeleton-tracker)

About The Project
=================

This project is focusing on developing packages and algorithms for Baxter Robot by Rethink Robotics. The project's aim is to make Baxter a cashier in a sweetshop. 

This involves several aspects:
- Perception: Skeleton Tracking to identify hand-pose of the customer.
- Manipulation: Baxter to get and give money to the customer's hand.

Installation
============

Prerequisites
-------------
- Install Openni2
```
sudo apt-get install ros-indigo-openni2-launch
sudo apt-get install ros-indigo-openni2-camera
```
- Install `cob_people_perception` library:
```
cd ~/catkin_ws/src

git clone git@github.com:papallas/cob_people_perception.git
cd cob_people_perception
rosdep install -r --from-paths .

git clone git@github.com:ipa-rmb/cob_perception_common.git
cd cob_perception_common
rosdep install -r --from-paths .

cd ~/catkin_ws
catkin_make
```

Clone Project
-------------
Either in `catkin_ws/src` or `ros_ws/src` clone this project:
```
git clone git@github.com:papallas/baxter_cashier.git
```

Important `cob_people_perception` modification
-----------------------------------------------
`cob_people_perception` project provides a pacakage called `cob_openni2_tracker` which as the name implies, allows us to have a skeleton tracker. However, if we need to get `tf`s broadcasted we need to make an alteration to the `.yaml` file of the pacakge.

Edit the file `cob_people_perception/cob_openni2_tracker/launch/body_tracker_params.yaml` and find the line with the parameter named `drawFrames`. This parameter will be set to false by default but we need to set it to true. So go ahead and change it to `true`. This will allow the `cob_openni2_tracker` to publish the body parts as `tf`s.

Running
=======

(Optional) Run just the Skeleton Tracker
-----------------------------
To run the skeleton tracker individually, here are the steps required (in separate terminal windows):
```
roslaunch openni2_launch openni2.launch depth_registration:=true
roslaunch cob_openni2_tracker body_tracker_nodelet.launch
rosrun rviz rviz
```
In Rviz now, set the `fixed_frame` to `camera_link`. Now add a `tf`. If you move in front of the camera sensor you should be able to see tf shown in space for each skeleton part.

Note how the `tf` are published: 
- Parent: `/camera_depth_optical_frame`
- Child: `cob_body_tracker/user_1/left_hand`
- Child: `cob_body_tracker/user_1/head`
- etc
