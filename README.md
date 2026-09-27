# rsf_slam

Generate a 2D map from a point cloud rosbag:

```bash
ros2 launch rsf_slam mapping.launch.py bag:=/path/to/bag output:=/path/to/map
```

The launch replays `/tf`, `/tf_static`, and `/rsf/hokuyo_cloud2`, converts the point cloud to `/scan`, runs slam_toolbox, and saves the map as YAML and PGM when playback ends.
