# **HandsFree_Sim**

## **Idea**
Control the turtle in the turtlesim just by your hand gestures!

#### **Available gestures**: 
Victory Sign (✌️) --> Go straight  
Pointing finger (☝️) ( left tilted ) --> Turn left on its spot  
Pointing finger (☝️) ( right tilted ) --> Turn right on its spot  
Raised Hand (✋) ( left tilted ) --> Turn left and go straight as well  
Raised Hand (✋) ( right tilted ) --> Turn right and go straight as well  
Fist (✊) --> Stop  

*Note: 5 seconds of inactivity/no gesture will automatically stop the program.*  
*This package has been created and tested on Ubuntu 22.04 with ROS2 Humble.*

## **How to build**
*Creating a workspace to build the package*
```
mkdir -p ~/handsfreesim_ws/src && cd ~/handsfreesim_ws/src
```
*Cloning the package*
```
git clone https://github.com/kalashjain23/handsfree_sim
cd ~/handsfreesim_ws
```
*Installing the dependencies and building the workspace*
```
rosdep install --from-paths src -y --ignore-src
colcon build
```
## **How to use**
```
# source the workspace
source ~/handsfreesim_ws/install/setup.bash

# All set to go! Run the launch file
ros2 launch handsfree_sim main.launch.py
```  
*Now you can control turtles in the turtlesim just by your hand gestures :D*

https://github.com/kalashjain23/handsfree_sim/assets/97672680/087287e8-2538-4ddc-a57e-bf14682f3f57
