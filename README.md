# robotarm_pkg

"robotarm_pkg" is a ros package to move robot arm.

# DEMO
 
You can move the robot arm like this video.

[demo1](https://www.youtube.com/watch?v=fsXm2Fhp12A), 
[demo2](https://www.youtube.com/watch?v=fUrJpv8W9eo&t=7s)
 
# Requirement
 
* [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
 
Environments under [Ubuntu 20.04](https://ubuntu.com/download/desktop) is tested.
 
# Installation
 
Install robotarm_pkg with git command.
 
```bash
git clone https://github.com/jagatama/robotarm_pkg armrobot_py
cd ..
catkin_make
```
 
# Usage

 keyboard control

```bash
roslaunch armrobot_py run_command.launch
```

csv file control

```bash
roslaunch armrobot_py run_csv.launch
```

ps3 control

```bash
roslaunch armrobot_py ps3_control.launch
```

If you want to move the robot arm using the keyboard and csv file, write the `RobotArm.ino` file to arduino.  
If you want to move the robot arm using the ps3 controller, write the `robotarm_ps3.ino` file to arduino. 

# Note
 
I don't test environments under Windows and Mac.
