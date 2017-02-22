0. Introduction
===============

This repository host the "Human-Robot Interaction for Cashier Robot" project, a undergraduate final year project at the School of Computing, University of Leeds. 

Although this project can possibly be improved, this is the final version submited as part of my dissertation and hence I will not consider any pull requests for changes, since I would like to keep the project at the state it has been submitted. Feel free though to have a walk around.

This project is **not** under active development.

Finally, the project was developed with Ubuntu 14.04 (LTS), ROS Indigo and Python2. The robot used was [Baxter Robot](http://www.rethinkrobotics.com/baxter/) by Rethink Robotics.

![Baxter](https://cloud.githubusercontent.com/assets/6514550/23082358/ef6cd810-f550-11e6-9088-c54a4a1f7a3b.png)



1. Table of Contents
====================

  * [0. Introduction](#0-introduction)
  * [1. Table of Contents](#1-table-of-contents)
  * [2. About The Project](#2-about-the-project)
    * [2.1 How the repository is organised](#21-how-the-repository-is-organised)
  * [3. Installation](#3-installation)
    * [3.1 Prerequisites](#31-prerequisites)
    * [3.2 Clone Project](#32-clone-project)
  * [4. Money Bills](#4-money-bills)
    * [4.1 Edit Templates](#41-edit-templates)
    * [4.2 Print bills](#42-print-bills)
  * [5. Running](#5-running)
    * [5.1 Run just the Skeleton Tracker](#51-optional-run-just-the-skeleton-tracker)



2. About The Project
====================

This project is focusing on developing packages and algorithms for Baxter Robot by Rethink Robotics. The project's aim is to make Baxter a cashier in a sweetshop. 

This involves several aspects:
- Perception: Skeleton Tracking to identify hand-pose of the customer.
- Manipulation: Baxter to get and give money to the customer's hand.

2.1 How the repository is organised
-----------------------------------
This repository makes use of most of the project managment features provided by GitHub. Including, issues, milestones, projects/boards.

* Project's [milestones](https://github.com/papallas/baxter_cashier/milestones).
* Project's [issues](https://github.com/papallas/baxter_cashier/issues).
* Project's [boards](https://github.com/papallas/baxter_cashier/projects) per phase (up to three).

By the end of the project probably most of the issues will be closed and hence you need to find the closed issues. Alternative, the Project's boards allows you to see the phase's issues under four columns (Someday/Maybe, To Do, In Progress and Done). Note that the issues/notes under Someday/Maybe are ideas or suggestions that are not of priority but will be good if implemented, however is not guaranteed that will ever be implemented.

Milestones are great way to visualise the project in different phases and issues are assigned to phases.



3. Installation
===============

This section describes what you need to install so you can have the project running on Baxter or the Gazebo simulator.

3.1 Prerequisites
-----------------
- Install [ROS Indigo](http://wiki.ros.org/indigo/Installation)
- Install [Baxter SDK](http://sdk.rethinkrobotics.com/wiki/Workstation_Setup)
- Install Openni2
```
sudo apt-get install ros-indigo-openni2-launch
sudo apt-get install ros-indigo-openni2-camera
```
- Install [`cob_people_perception`](https://github.com/ipa-rmb/cob_people_perception) and [`cob_perception_common`](https://github.com/ipa-rmb/cob_perception_common) library:
```
cd ~/catkin_ws/src

git clone git@github.com:ipa-rmb/cob_people_perception.git
cd cob_people_perception
rosdep install -r --from-paths .

cd ~/catkin_ws/src

git clone git@github.com:ipa-rmb/cob_perception_common.git
cd cob_perception_common
rosdep install -r --from-paths .

cd ~/catkin_ws
catkin_make
```

**Important `cob_people_perception` modification**  
`cob_people_perception` project provides a pacakage called `cob_openni2_tracker` which as the name implies, allows us to have a skeleton tracker. However, if we need to get `tf`s broadcasted we need to make an alteration to the `.yaml` file of the pacakge.

Edit the file `cob_people_perception/cob_openni2_tracker/launch/body_tracker_params.yaml` and find the line with the parameter named `drawFrames`. This parameter will be set to false by default but we need to set it to true. So go ahead and change it to `true`. This will allow the `cob_openni2_tracker` to publish the body parts as `tf`s.

- Install [`cv_bridge`](http://wiki.ros.org/cv_bridge) and [`vision_opencv`](http://wiki.ros.org/vision_opencv) required for the perception part of the project:
```
sudo apt-get install ros-indigo-cv-bridge
sudo apt-get install ros-indigo-vision-opencv
```
- Install `imutils`
```
sudo pip install imutils
```

3.2 Clone Project
-----------------
Now that you have installed all required libraries you need to clone this project.

Either in `catkin_ws/src` or `ros_ws/src` clone this project:
```
git clone git@github.com:papallas/baxter_cashier.git
```



4. Money Bills
==============
This section is about the money bills that are used in the project. Money bills are used between Baxter and the customer to establish a payment. This project works with defined money bills (fake money bills).

<table>
 <tr>
   <td>
    <img src="https://cloud.githubusercontent.com/assets/6514550/23165412/e91d7e84-f833-11e6-8ffa-8bc387f479c5.png" alt="Five Bill" width="200">
   </td>
   <td>
    <img src="https://cloud.githubusercontent.com/assets/6514550/23165411/e9199c1a-f833-11e6-9d5e-c13342916926.png" alt="One Bill" width="200">
   </td>
 </tr>
</table>

4.1 Edit Templates
------------------
The money bills were created using the Pixelmator software. If you need to edit or create new notes the files are available under `TODO`.

4.2 Print bills
---------------
You can print bills as images. The images are available under `TODO` and when printing A4 paper put the scale to `45%` and then with a scissor cut the bill.



5. Running
==========

5.1 (Optional) Run just the Skeleton Tracker
--------------------------------------------
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
