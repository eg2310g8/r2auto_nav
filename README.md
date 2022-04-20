<h1> r2auto_nav </h1>

ROS2 auto_nav code for EG2310 module

## Structure

This repository contains all the code that is necessary for a TurtleBot to map a closed connected unknown maze by following a wall (users can indicate if they want the robot to follow the left or right wall) and shoot a ping-pong ball to an IR target (provided the firing mechanism used is the same as ours). You can go to our [Documentations](Documentations) folder to obtain more detailed information about our mechanical, electrical and software design. 

- [dtnav.py](dtnav.py) file is the wall following code which has been calibrated to fit our robots' needs. The code uses a decision tree to decide its actions when wall following.
- [trinav.py](trinav.py) file is the wall following code which calculates the angle of the robot to the wall and determine its locomotion when following the wall
- [tri2nav.py](tri2nav.py) file is similar to trinav. However, the parameters has been tuned for the robot and targetting and firing has been improved to move away from surrounding obstacles. 
- [Original_Files](Original_Files) folder contains all the original files that comes with the initial fork from [shihchengyen's r2auto_nav repository](https://github.com/shihchengyen/r2auto_nav) and are <b>not</b> necessary for the TurtleBot to navigate the maze by wall following, identify the IR target, and fire.
- TODO: add code to be run on turtlebot and add code that was running on Justin Laptop
- TODO: add scripts we used for testing

## Media
- [Photos](Photos) contain images of our robot
- Prof Yen uploaded videos of our graded run [here](https://photos.google.com/share/AF1QipMOHCC4Hmy2iMv2cuJiwt46ZaMUjUX0MDv22KL1oktZ5YMMdFfMDikiKv0L4tRCtg?key=cUpqMlcxdmlxRlFrZUQwS3BXZ1cxc0tzVDR4VG5B)

## Variables Configuration

- ```rotatechange``` to adjust the rotation speed of the TurtleBot. 
- ```self.follow``` to change the side of the wall to follow
- ```self.d``` to change detection range for obstacle. E.g.: If ```self.d == 0.35```, if detected distance on LiDAR is less than 0.35m the programme will treat it as an obstacle.
- ```self.wall``` to change wall tracking distance from the wall
- ```self.k_diff``` to tune the robot correction to its distance from the wall
- ```self.k_theta``` to tune the robot correction to its angle from the wall
- ```self.follow_speed``` to tune the robot movement speed
- ```self.heated``` which determines the target detection temperature threshold
  
## Software Block Diagram
![Software Block Diagram](Documentations/software_block_diagram.png)

## How to Use
In your laptop:
- Create a ROS2 package and clone this repository into that package. Make sure to edit the setup.py file so that you can run the wall following code.
- Build the package.
- TODO Include code ran on justin laptop

In the RPi on the TurtleBot:
- TODO UPDATE

Running Instructions:
- Start rosbu from the RPi on the TurtleBot after the robot is placed in the maze. Do not touch the robot until the gyro calibration is complete.
- Run ```sudo pigpiod``` on the RPi
- Start rslam from your laptop: 
  ``` ros2 launch turtlebot3_cartographer cartographer.launch.py ```
- Start the code from the RPi on the TurtleBot: 
  ```ros2 run <package_name> <entry_point_specified_in_the_setup.py> ```
- Start the wall following code from your laptop: 
  ```ros2 run <package_name> tri2nav ```

## Acknowledgements

 The structure of the repository has been forked from  [alvynben's r2auto_nav repository](https://github.com/alvynben/r2auto_nav)

 Group 8 would like to thank the teaching team for their guidance and support throught out the module
