# The rule_based_obstacle_avoidance ROS2 package

- Author: Roberto Zegers
- Date: July 2022

## Description

- Minimal rule based obstacle avoidance node that uses data from laser scan readings (c++)

## Instructions

Build with:  

`colcon build --packages-select rule_based_obstacle_avoidance`  

After compiling, source the workspace otherwise you will get the `Package 'rule_based_obstacle_avoidance' not found` error message:  

`source install/setup.bash`  


## Dependencies
- ROS2 Galactic  