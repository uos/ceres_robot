# Ceres-Localization

**Description of how to localize a ceres-robot (in a gazebo simulation) using a completed map and ceres_localization.launch.**

****
## Preparation

### 1. Execute these commands to be sure
```
source /opt/ros/noetic/setup.bash
source ~/catkin_ws/devel/setup.bash
```

### 2. Make sure to have no connection with a robot
```
unset ROS_MASTER_URI
unset ROS_IP
unset ROS_HOSTNAME
```

### 3. Start the simulation
```
roscore
```
open a new terminal
```
roslaunch ceres_gazebo ceres.launch
```
That is what you should see:

![ceres_robot](docs/images/screenshot_gazebo.png?raw=true "gazebo")

![ceres_robot](docs/images/screenshot_gazebo2.png?raw=true "gazebo")

You should preferably put the roboter modell in a quite empty room.

![ceres_robot](docs/images/screenshot_gazebo3.png?raw=true "gazebo")

***
## Localization

open a new terminal

### 1. Start the localization launch-file

```
roslaunch ceres_localization ceres_localization.launch
```
If the amcl package cannot be found, you will have to install it first.

```
sudo apt install ros-noetic-amcl
```

### 2. Open RViz
open a new terminal and then open RViz
```
rviz
```
You should now see this window.

![ceres_robot](docs/images/screenshot_rviz.png?raw=true "RViz")


Adjust the parameters as seen in the picture below.
![ceres_robot](docs/images/screenshot_rviz2.png?raw=true "RViz")

A map of the gazebo world should show up.
![ceres_robot](docs/images/screenshot_rviz3.png?raw=true "RViz")

### 3. Localization process

To start the localization process you have to show the roboter´s approximate start position in RViz. To do so click on "2D Pose Estimate" and draw a green arrow. The beginning of this arrow shows the roboter´s position while it´s direction shows the roboter´s orientation.

![ceres_robot](docs/images/screenshot_rviz4.png?raw=true "RViz")

You can check whether you have put the model in the right place by comparing the laser scan to the walls of the map as well as looking for the roboter´s position in the gazebo simulation.

If the given position is wrong, you can adjust it by drawing the "2D Pose Estimate" - arrow one more time.

Now you can drive the model around to check whether the localization works as it should. The scan should roughly match the map walls.

```
(open a new terminal)
roslaunch uos_diffdrive_teleop key.launch
```
If everything is functioning, you can now try out the localization on a real roboter or start with the navigation.

***
## Differences to a localization of a real roboter

You can follow the instructions above to localize a roboter in a real-life invironment as well. There only few things you should do different.

### 1. Build a connection with the robot
```
ssh _____(the name of the robot)
```

### 2. Start ceres.rc and tmux
```
cd ~/catkin_ws
source src/ceres_robot/ceres_util/ceres.rc
cmux
tmux a
```
### 3. Start the bringup
But first check whether roscore is started:
* to change the directory:  `Ctrl + b` then `s`
* go to roscore using the arrows
* to choose the directory press `Enter`

Then go to the bringup directory and start the bringup.
```
roslaunch ceres_bringup ceres.launch
```

### 4. Sign on the comunication between the robot and your computer
open a new terminal
```
export ROS_MASTER_URI=_______
export ROS_IP=_______
```
You can look up your ROS_IP by typing `ifconfig` in your terminal. It can be found in the second line from above: inet ____

You can look up your roboter´s ROS_MASTER_URI in the roscore directory in the terminal, where you have connected with your roboter.

### 5. Make sure the map you need is on the roboter

 It should lay in the ceres_localization/maps folder.
 ```
 (on the roboter)

 cd ~/catkin_ws/src/ceres_robot/ceres_localization/maps
 ls
 ```

 If it does not, copy it from your computer.
 ```
 (on your computer)

 scp (the name of your map).pgm (the name of your roboter):~/catkin_ws/src/ceres_robot/ceres_localization/maps/
 scp (the name of your map).yaml  (the name of your roboter):~/catkin_ws/src/ceres_robot/ceres_localization/maps/

 ```

### 6. Make sure the map name in ceres_localization/launch/map_server.launch is the right one

```
cd ~/catkin_ws/src/ceres_robot/ceres_localization/launch
vim map_server.launch
```
You will find the name of the map in the third line. If it is the right one, leave the editing programm by typing `:q`.

If the mape name is wrong, change it first. By presseng `i` you can open an editing mode and then change the map name. When you have finished, press `Esc`, then type `:w` to save the changes and `:q` to close the editing programm.

### Now you can continue with the tutorial above (from Localization).
! Start your launch-files on your roboter, e.g. in debug !

! Start RViz in the terminal with the export commands !

