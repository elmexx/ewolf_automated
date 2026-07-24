# radar_bev_visualizer

ROS2 Foxy package for converting radar PointCloud2 topics into a BEV radar display image.

## Input topics
- `/radar_detection_pcl_infra`
- `/radar_detection_pcl_non_infra`

Expected PointCloud2 fields:
- `x`
- `y`
- `z`
- `doppler`
- `range`
- `azimuth`
- `azimuth_std`

## Output topic
- `/radar_bev_image` (`sensor_msgs/msg/Image`, encoding `bgr8`)

## Features
- BEV radar display image for NDI/tablet pipeline
- ROI filtering by range, azimuth, z, and azimuth uncertainty
- Separate rendering for infra and non-infra points
- Doppler-based coloring for dynamic non-infra points
- Short history window to improve stability and readability
- Status overlay for demo use

## Build
Put the package inside your ROS2 Foxy workspace `src` folder:

```bash
cd ~/ros2_ws/src
cp -r /path/to/radar_bev_visualizer .
cd ~/ros2_ws
colcon build --packages-select radar_bev_visualizer
source install/setup.bash
```

## Run
```bash
ros2 launch radar_bev_visualizer radar_bev_visualizer.launch.py
```

Or directly:

```bash
ros2 run radar_bev_visualizer radar_bev_visualizer_node --ros-args \
  -p infra_topic:=/radar_detection_pcl_infra \
  -p non_infra_topic:=/radar_detection_pcl_non_infra \
  -p output_image_topic:=/radar_bev_image
```

## Recommended demo tuning in indoor lab
Start with:
- `max_range_m: 12.0`
- `max_abs_azimuth_deg: 25.0`
- `max_azimuth_std_deg: 6.0`
- `history_window_sec: 0.4`
- `static_doppler_threshold_mps: 0.2`

## Notes
- Infra points are rendered as dim gray background clutter.
- Non-infra points are emphasized.
- Dynamic non-infra points are color coded using doppler.
- If your radar publishes timestamps as zero, the node uses local ROS time.
