# ROS Noetic package for TeraRanger EVO 33  
This package allows to use teraranger nodes for ROS Noetic (python 3) on a Raspberry Pi Model 3B

## Dependencies

This package depends on this [serial](http://wiki.ros.org/serial) library. To get it, execute the following command:

```
sudo apt-get install ros-noetic-serial
```


## 1. Run on ROS Noetic the TeraRanger Evo Thermal 33 node on Raspberry Pi

After your workspace is built and sourced:
```
rosrun teraranger evo_thermal.py _portname:=/dev/ttyACM0
```

This node is publishing on two topics:

* /teraranger_evo_thermal/rgb_image: a color mapped RGB image based on thermal data
* /teraranger_evo_thermal/raw_temp_array: an array of 1024 raw thermal data
* /teraranger_evo_thermal/ptat: internal temperature of the sensor

## 2. Run the ros1_bridge on the PC
Donwload the ros1_bridge repository 
```
git clone https://github.com/ros2/ros1_bridge.git
```
Follow the [instructions](https://github.com/ros2/ros1_bridge#building-the-bridge-from-source) to build the package


**Run the dynamic bridge**

Setup the environment variables of ros noetic
```
source /opt/ros/noetic/setup.bash
```
Setup the comunication with Raspberry Pi nodes
```
export ROS_MASTER_URI=http://<raspberry_ip>:11311
export ROS_IP=<pc_ip
```
Setup the environment variables of ros2 foxy
```
source /opt/ros/foxy/setup.bash
```
Set the bridge between ros1 noetic and ros2 foxy topics
```
ros2 run ros1_bridge dynamic_bridge
```



-----------------------------------------------------------------
Possible **issue**: 

- The command *rostopic list* returns the topic's name correctly
- The command *rostopic echo* doesn't returns the topic's values

**Solution**: 
At /etc/hosts file add ```<raspberry_ip> raspberrypi```

-------------------------------------------------------------

## 3. Run the ROS2 Foxy  subscriber/publisher on the PC
In the ROS2_sub_pub folder, is possible to find the necesary package to republish the ROS Noetic topics with ROS2 Foxy topics.

To use it you have to: 

Setup the environment variables of ros2 foxy
```
source /opt/ros/foxy/setup.bash
```
Run the ros2 node to republish the topics on ros2.
```
ros2 run thermal_camera camera_sub_pub
```

