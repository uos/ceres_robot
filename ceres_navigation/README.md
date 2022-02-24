# Ceres-Navigation

**Description of how to start and operate a navigation launch-file for a ceres-robot to navigate in a completed map using ceres_navigation.launch and RViz.**
***
### 1. Preparation
You should have already started the localization. If you have not, do it first.

### 2. Start the navigation launch-file
open a new terminal
```
roslaunch ceres_localization ceres_localization.launch
```
There is a good chance for quiete a few errors to show up. If that is the case, look in the file ceres_navigation/launch/ceres_navigation.launch for solutions.

### 3. Navigation process

To start the navigation process you have to show the robot it`s goal position in RViz. To do so click on "2D Nav Goal" and draw a pink arrow. The beginning of this arrow shows the roboter´s goal position while it´s direction shows the roboter´s goal orientation.

![ceres_robot](docs/images/screenshot_navigation.png?raw=true "gazebo")

The robot will now move to it`s goal position on it´s own choosing the most efficient path. It should also dodge stable and moving obstacles.

![ceres_robot](docs/images/screenshot_navigation2.png?raw=true "gazebo")

If the robot cannot find a safe enough path, an error will show up in the terminal and you will have to restart the launchfall.
