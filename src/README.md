How to use GUI?
1. roslaunch kortex_gazebo spawn_kortex_robot.launch 
2. rosrun <your package name> designwindow.py

design.ui is the UI file which can be edited graphically
design.py is the python version of design.ui, so once you make change to design.ui, you will need to run 'pyuic5 design.ui -o design.py'
designwindow.py is the node for the newest GUI


