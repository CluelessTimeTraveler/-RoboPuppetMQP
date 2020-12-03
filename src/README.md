# How to use GUI for trina2?
gui_login.py is the login interface  
gui_signup.pu is the sign up interface  
gui_lite and gui_pro are two versions of Robopuppet GUI  
Each of them can be rosrun individually  

## UI files
In /ui are the ui files where design.ui is the UI file which can be edited graphically using Qt Creator  
design.py is the python version of design.ui, so once you make change to design.ui, you will need to run 'pyuic5 design.ui -o design.py'  

## Difference between GUI Lite and Pro
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

## Common problems
1. Image in the intro page does not show up. You need to check and modify the file path

