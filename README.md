# Collision Avoidance for static and dynamic obstacles

The repository presents the algorithms developed in order to avoid collision of both static and dynamic obstacles. The static obstacles follow an edited version of the Dynamic Window Approach whereas for the dynamic obstacles, a COLREGS compliant algorithm was developed and implemented.

## Getting Started

These instructions will get you a copy of the project up and running on your local machine for development and testing purposes. 

### Prerequisites

Installing ROS and Ubuntu

For this section it is needed the following:

* Laptop/computer with Ubuntu installed
* Vehicle able to take in ROS commands (optional)

The version used and tested was Ubuntu 16.04 (Xenial Xerus) and can be downloaded from the  [Ubuntu's main page](https://www.ubuntu.com/download/desktop). 

After having Ubuntu installed, it is time to install ROS. The version used and tested for the algorithm which was presented is [ROS Kinetic](http://wiki.ros.org/kinetic).

The need to install ROS in the computer can be, either to use a simulated vehicle in a software such as [Gazebo](http://gazebosim.org/), or in case the user is in possession of a vehicle able to also run ROS, for a debugging phase and to exchange information.


### Getting started with ROS

It is not the purpose to point out how to build a robot or vehicle. 
The intention is to specify the necessary inputs needed from the system and the type of outputs which it gives out to the control frame. 

The following commands create a personal workspace on ROS, in case one is not yet created.

Create the workspace:

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspaces
```

In order to source the workspace each time a new terminal is opened,  the following command open the bashrc-file:


```
sudo gedit ~/.bashrc
```

Then, the following line should be added at the bottom of the bashrc-file:

```
source gedit ~/catkin_ws/build/setup.bash
```

### Package installation

The installation of the algorithms nodes can be done via git clone:

```
cd ~/catkin_ws/src
git clone https://github.com/uc2013171665/asv_colregs.git
cd..
catkin_make
```

## Inputs and Outputs

The physical vehicle, or the simulated vehicle, must have working sensors, such as the proprioceptive and the exteroceptive sensors. 
Thus, the inputs needed are: a viable odometry, GNSS data, and AIS data from other vehicles. For outputs, the algorithm produces data for the control module, AIS data, and debugging data. 
All these are published and subscribed with the respective topic names. Thus, in order to implement in other vehicle, open the Ownship_vehicle.py file, and change the topic names to the ones used by the vehicle, and take into account the message used by the topic.

### Odometry

The Odometry is essential to know the ownship's position, heading, and velocity, which are necessary for the algorithm. 
Usually, odometry is used with the message type nav_msgs/Odometry. It is also this type of message used in order to set up the robot with the navigation package. The name of the topic can be set to the preferred, having to be consistent with the one used by the vehicle.

### GPS

The GPS data, from the GNSS system, is used to get the local coordinates, as previously explained. 
GNSS and sensors data are frequently used with the message type sensor_msgs/NavSatFix. Again, the name of the topic can be set to the preferred, having to be consistent with the one used by the vehicle.

### AIS

The AIS is subscribed and published by the Ownship node. In the algorithm, the current topic name is ``chatter``, with the message type being ASV_messages/AIS.

### Control module

The ownship node publishes 4 arguments which the control block is subscribing: ``desired.yaw``; ``desired.velocity``; ``desired.vid``; ``Reset``. 
Therefore, the control frame of the vehicle must subscribe to a topic with these arguments as the input. The name of the topic can be defined as preferred, taking into account that both node and the vehicle must have the coherent topic name, and the message arguments have to be as detailed.

The ``desired.yaw`` and ``desired.velocity`` are as the names imply. The ``desired.vid`` was introduced in order to identify which vehicle was publishing the data in the simulation, as both ownship and contact published to the same topic, thus the need to introduce this reference. The ``Reset`` is a boolean variable which when set true, the values sent are reseted by the control module.

### Debugging

A debugging message is essential to facilitate corrections and to understand the anomalies which appear through the development.
The Ownship node publishes a topic named ``DEBUG`` with the message type being ASV_messages/debug. 

### Laser scan 

For static obstacles, the Lidar is used to scan data in order to detect and generate an array of obstacles for the algorithm. 
A Lidar system or a simulated laser scan data, are needed for the subscription by the algorithm, having the topic being ``/mybot/laser/scan``, with the msg type being sensor_msgs.msg.

### Debugging2 

A debugging message was also implemented when debugging the DWA algorithm, brought by the lidar node.
The lidar node publishes a topic named ``DEBUG2`` with the message type being ASV_messages/debug2. 

## Launch the program

The directory /catkin_ws/src/ASV/launch has the file ASV.launch and ASV2.launch. The launch file documentation can be seen in [XML](http://wiki.ros.org/roslaunch/XML), which describes the XML format used for roslaunch .launch files. 
The launch file has most of the arguments in comment as default, as it will depend on its use. 
It can be used to launch a Gazebo simulation, define the vehicle properties, or it could be used to launch the physical vehicle.
In the ASV.launch, the only argument not in comment is the COLREGS node launch, which is independent of the case, whereas in the ASV2.launch it is the LIDAR node.
The COLREGS algorithm and the DWA algorithm were separated to facilitate the testing and debugging.

Having defined the arguments in the desired launch file, the launch is done following one of the command:

```
roslaunch ASV ASV.launch
```
or 
```
roslaunch ASV ASV2.launch
```

## Authors

* **[André Santos](uc2013171665@student.uc.pt)** - *Master Thesis student* - [University of Coimbra](https://www.uc.pt/)

