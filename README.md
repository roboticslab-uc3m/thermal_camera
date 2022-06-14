# ROS package for TeraRanger modules  
This package allows to use teraranger nodes for ROS Noetic (python 3) on a Raspberry Pi Model 3B

## Dependencies

This package depends on this [serial](http://wiki.ros.org/serial) library. To get it, execute the following command:

```
sudo apt-get install ros-noetic-serial
```


## Running the TeraRanger Evo Thermal 33

After your workspace is built and sourced:
```
rosrun teraranger evo_thermal.py _portname:=/dev/ttyACM0
```

This node is publishing on two topics:

* /teraranger_evo_thermal/rgb_image: a color mapped RGB image based on thermal data
* /teraranger_evo_thermal/raw_temp_array: an array of 1024 raw thermal data
* /teraranger_evo_thermal/ptat: internal temperature of the sensor

