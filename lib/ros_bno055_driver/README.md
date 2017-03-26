# ROS Ardafruit BNO055 IMU Node

This package is a wrapper of the Adafruit_Python_BNO055 driver for the Robot
Operating Sysytem (ROS).

The package has been tested with ROS Indigo and the Ardafruit BNO055 Breakout
using UART , but might work with other setup.

## Requirements

For the hardware

- Ardafruit BNO055 Breakout
- UART + USB cable

https://learn.adafruit.com/adafruit-bno055-absolute-orientation-sensor/assembly

For the software 
- Ubuntu 14.04 with ROS indigo
- Adafruit_Python_BNO055 python library

## Installation

1. Create a catkin workspace.

```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
catkin_init_workspace
```

2. clone this repository

```
cd ~/catkin_ws/src
git clone https://github.com/markusgrotz/bosch_bno055_driver.git
```

3. build your workspace
```
cd ~/catkin_ws/
catkin_make
```

4. launch the node
```
cd ~/catkin_ws/
source devel/setup.bash
roslaunch bosch_bno055_driver bosch_bno055_driver.launch
```


## Adafruit_Python_BNO055 Installation

```
git clone https://github.com/markusgrotz/Adafruit_Python_BNO055`
cd Adafruit_Python_BNO055
python setup.py develop --user
```

## Troubleshooting


### ImportError: No module named Adafruit_BNO055.BNO055

You need to install the Ardafruit python library.
See section [Adafruit_Python_BNO055 Installation](#Adafruit_Python_BNO055 Installation).


### Unable to open /dev/ttyUSB0

Check if your serial is attached to */dev/ttyUSB0*. If otherwise edit the
parameter in the launch file. 

Additionally, check if you have sufficient permissions. Add your user to the
dialout group by running `sudo usermod -a -G dialout <username>`. Changes will
apply once you've logged out. 
You can also simply run `sudo chmod 777 /dev/ttyUSB0`.
