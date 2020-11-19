How to use GUI for trina2?

1. roslaunch trina2_gazebo trina2.launch 
2. rosrun -your package name- designwindow_trina2.py

design.ui is the UI file which can be edited graphically  
design.py is the python version of design.ui, so once you make change to design.ui, you will need to run 'pyuic5 design.ui -o design.py'  


GUI Lite
-Robopuppet setting
-Current angle & velocity
-*Camera view



GUI Pro
-Robopuppet setting
-Current angle & velocity
-Cam view
-Direct arm control
-Plot
-Learning function

