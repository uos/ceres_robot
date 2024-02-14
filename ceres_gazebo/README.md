# Ceres Gazebo

Start the simulation of the Ceres robot in AVZ world

```console
ros2 launch ceres_gazebo ceres_gazebo_launch.py
```


## SLAM

In a new terminal run:

```console
ros2 launch ceres_localization slam_toolbox_launch.py
```

Open RViz and display the topic `/map`. Steer the robot via teleop and record a map.

```console
ros2 launch uos_diffdrive_teleop key.launch
```

Save the map via

```
ros2 service call /map_saver_server/save_map nav2_msgs/srv/SaveMap "{map_topic: map, map_url: my_map, image_format: pgm, map_mode: trinary, free_thresh: 0.25, occupied_thresh: 0.65}"
```

I created an alias for that in my `.bashrc`.

## AMCL

First start RViz. Then run

```console
ros2 launch ceres_gazebo amcl_avz_launch.py
```

I the map is not visible in RViz you need to restart the amcl launch file