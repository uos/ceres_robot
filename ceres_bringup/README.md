# Ceres Bringup

Start the real the Ceres robot

```console
ros2 launch ceres_bringup ceres_launch.py
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

```console
ros2 launch ceres_bringup amcl_cic_launch.py
```

## Nav2

...

