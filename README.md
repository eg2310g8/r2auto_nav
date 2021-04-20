<h1> r2auto_nav </h1>

ROS2 auto_nav code for EG2310 module

## Structure

This repository contains all the code that is necessary for a TurtleBot to map a closed connected unknown maze (by wall following the wall on the robot's right) and shoot a ping-pong ball to an IR target (provided the firing mechanism used is the same as ours). You can look at the Documentation file to see more details about our robot.

- [Extras](Extras) folder contains all the extra files that are <b>not</b> necessary for the TurtleBot to navigate the maze by wall following, identify the IR target, and fire.
- [Original_Files](Original_Files) folder contains all the original files that comes with the initial fork from [shihchengyen's r2auto_nav repository](https://github.com/shihchengyen/r2auto_nav) and are <b>not</b> necessary for the TurtleBot to navigate the maze by wall following, identify the IR target, and fire.
- [Partially_Working_Navigation](Partially_Working_Navigation) folder contains all the partially working navigation code that have not been integrated with the targeting code and may not fully complete the navigation. These files are <b>not</b> necessary for the TurtleBot to navigate the maze by wall following, identify the IR target, and fire.
- [r2wallfollower.py](r2wallfollower.py) file is the wall following code which has been calibrated to fit our robots' needs. This code also includes a subscriber to 'targeting_status' topic being published by the TurtleBot such that the robot can navigate the maze while continuously trying to detect the target. The robot will then fire when the target is detected before resuming its navigation once again if the map is not yet complete.
- [r2targeting.py](r2targeting.py) file is the targeting and firing code which has been calibrated to fit our robots' needs. This file will be put into the RPi on the TurtleBot and will be run from inside the RPi on the TurtleBot. This code includes the publisher which will publish the 'targeting_status' topic to which the wall following code will subscribe.

## How to Use

In your laptop:
- Create a ROS2 package and clone this repository into that package. Make sure to edit the setup.py file so that you can run the wall following code.
- Build the package.

In the RPi on the TurtleBot:
- Create a ROS2 package and copy [r2targeting.py](r2targeting.py) file into that package. Make sure to edit the setup.py file so that you can run the targeting code.
- Build the package.

Running Instructions:
- Start rosbu from the RPi on the TurtleBot.
- Start the targeting code from the RPi on the TurtleBot: 
  ```ros2 run <package_name> <entry_point_specified_in_the_setup.py> ```
- Start the wall following code from your laptop: 
  ```ros2 run <package_name> <entry_point_specified_in_the_setup.py> ```
