# RoboPuppetMQP
The repository for the MQP project

# What you will need
You will need to git clone and setup TRINA-WPI-2.0 before using the robopuppet  
https://github.com/hiro-wpi/TRINA-WPI-2.0

# How to use?
## Setup
1. Go to your ros workspace  
2. Enter 'src' folder  
3. Create a new package   
4. Enter that package  
5. git clone https://github.com/CluelessTimeTraveler/RoboPuppetMQP.git  

## Connect to Serial Port
1. Connect the Robopuppet via USB port  
2. Check the port name, if it is not '/dev/ttyACM0', then modify line31 in the SerialToRosMessage.py file accordingly.  
3. use roscore and rosrun SerialToRosMessage.py to see if the serial connection succeed  

## Use Robopuppet
1. If everything setup correctly, use roslaunch RoboPuupetMQP trina2.launch  
2. Hit the play button in Gazebo  
3. Login use 'admin admin' or 'user user' or sign up one  

# Potential Problems during setup
1. Missing Module. If you meet this problem, try apt-get that module,and double check the instruction in Trina2 repository 
