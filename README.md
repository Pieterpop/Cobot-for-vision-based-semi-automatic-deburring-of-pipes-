# Cobot-for-vision-based-semi-automatic-deburring-of-pipes-2.2 How to set up  

The setup for this system requires many different programs and some unexpected commands. And the tool is a wireless rechargeable battery tool. Making it have less wires that can get tangled. It has been modified with cables so it can be hooked up to some external source that shorts the purple and white cables twice to turn on the tool, short green and blue to speed up the tool and yellow and orange to slow down the tool. 

 

 


To set up the ur3e properly follow the instructions from the Robotiq wrist camera setup quick start guide. Then follow the instructions on ‘Universal Robots ROS2 Documentation’ and follow the instructions for E-series robot 


On the computer first download a ubuntu 24.04 boot drive. Then open and setup Ubuntu. Then download ROS2 jazzy, Universal_Robots_ROS2_Driver, Gazebo, Universal_Robots_ROS2_GZ_Simulation, MATLAB R2025a and Simulink. And all the files from the GitHub. Then open Matlab using this ‘LD_PRELOAD=/usr/lib/x86_64-linux-gnu/libstdc++.so.6 <matlab file>’ in the terminal. Then in two new terminals  source /opt/ros/jazzy/setup.bash and in one of those ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur3e robot_ip:=192.168.56.101 launch_rviz:=true use_fake_hardware:=false 

 
                        

When everything is setup.  

    First place the pipe into the vice.  

    Measure the pipe. 

    Set the tool on the surface of the pipe as seen in figure ... .  

    Run the Matlab script Zpos. 

    Move the robot up a bit and start running the program until the popup. Make sure you have the installations->URCaps->Camera->Camera open have. 

    Fill in the measurements and run the Matlab script called Working.m. 

    Click ok on the popup and run SIMUR10E.slx. 
