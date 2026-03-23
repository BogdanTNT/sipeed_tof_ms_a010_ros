# Sipeed MS-A010 ROS 2 Driver

## Overview

This package provides a ROS 2 driver for the Sipeed MS-A010 ToF sensor.

- raw depth image publishing
- colored point cloud publishing
- `CameraInfo` publishing
- launch + YAML configuration
- multi-camera launch from separate YAML files
- TF-friendly timestamping for RViz
- published points in real world metric space
- optional sensor-side setup commands with ROS params
- timestamp offset parameter to reduce intermittent RViz TF errors

The package name is:

`sipeed_tof_ms_a010`


## What It Publishes

The node itself publishes these relative topic names:

- `depth`
  Raw 8-bit depth image from the sensor as `sensor_msgs/msg/Image`
- `cloud`
  Colored `sensor_msgs/msg/PointCloud2`
- `depth/camera_info`
  `sensor_msgs/msg/CameraInfo`

If you launch a single camera with the default launch file:

```bash
ros2 launch sipeed_tof_ms_a010 tof_node.launch.py
```

the launch file uses the YAML filename stem as the namespace. With the default
`tof_params.yaml`, the full topics become:

- `/tof_params/depth`
- `/tof_params/cloud`
- `/tof_params/depth/camera_info`

If you launch multiple cameras from different YAML files:

```bash
ros2 launch sipeed_tof_ms_a010 tof_node.launch.py \
  params_file:=/path/to/front_tof.yaml

ros2 launch sipeed_tof_ms_a010 tof_node.launch.py \
  params_file:=/path/to/wrist_tof.yaml
```

then each camera gets its own namespace automatically:

- `front_tof.yaml` publishes to:
  `/front_tof/depth`, `/front_tof/cloud`, `/front_tof/depth/camera_info`
- `wrist_tof.yaml` publishes to:
  `/wrist_tof/depth`, `/wrist_tof/cloud`, `/wrist_tof/depth/camera_info`

You can also override the namespace explicitly:

```bash
ros2 launch sipeed_tof_ms_a010 tof_node.launch.py \
  namespace:=tof_front \
  params_file:=/path/to/front_tof.yaml
```

which gives:

- `/tof_front/depth`
- `/tof_front/cloud`
- `/tof_front/depth/camera_info`

Important:

- The `depth` image is the raw `mono8` payload from the sensor
- The point cloud is where the driver converts raw values into metric `x/y/z`

## Requirements

Tested in this workspace with ROS 2 Jazzy.

Typical dependencies:

- ROS 2
- `cv_bridge`
- OpenCV
- a serial device for the camera, for example `/dev/ttyUSB0`

If needed:

```bash
sudo apt install ros-jazzy-cv-bridge libopencv-dev
```


## Build

From the workspace root:

```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sipeed_tof_ms_a010
source install/setup.bash
```


## Run

Recommended:

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch sipeed_tof_ms_a010 tof_node.launch.py
```

You can also launch the same driver multiple times for multiple cameras by
giving each instance its own YAML file. By default, the launch file now uses
the YAML filename stem as the namespace and `<stem>_node` as the node name.

```bash
ros2 launch sipeed_tof_ms_a010 tof_node.launch.py \
  params_file:=/path/to/front_tof.yaml

ros2 launch sipeed_tof_ms_a010 tof_node.launch.py \
  params_file:=/path/to/wrist_tof.yaml
```

That gives you separate topics such as:

- `/tof_front/depth`
- `/tof_front/cloud`
- `/tof_wrist/depth`
- `/tof_wrist/cloud`

You can still override `namespace:=...` or `node_name:=...` explicitly if you
want.

The node publishes relative topic names:

- `depth`
- `cloud`
- `depth/camera_info`

So with `front_tof.yaml`, those become:

- `/front_tof/depth`
- `/front_tof/cloud`
- `/front_tof/depth/camera_info`

You can also run the node directly:

```bash
ros2 run sipeed_tof_ms_a010 sipeed_tof_node --ros-args \
  --params-file src/MaixSense_ROS-main/sipeed_tof_ms_a010_ros/ros2/config/tof_params.yaml
```

Backward-compatible executable:

```bash
ros2 run sipeed_tof_ms_a010 publisher
```


## Parameters

The default parameters live in `config/tof_params.yaml`.

- `device`
  Serial device path, for example `/dev/ttyUSB0`
- `frame_id`
  TF frame used in published messages, for example `camera_optical_frame`
- `timer_period_ms`
  Legacy compatibility parameter. The current driver uses a continuous serial
  reader thread, so this no longer controls the real publish cadence.
- `timestamp_offset_ms`
  Stamps messages slightly in the past so RViz can find the matching TF more
  reliably
- `apply_sensor_settings`
  Enables the extra startup `AT+FPS` command. The default is `false` so the
  node comes up on the known-good baseline stream first.
- `binning`
  Sensor binning mode
  Note: `binning: 2` is slower to settle on this sensor than `1` or `4`
  and may take about 1-2 seconds before valid frames resume.
- `display_mode`
  Sensor display/output mode
- `fps`
  Sensor frame rate command value
- `quantization_unit`
  Sensor quantization unit command value. This one is enforced at startup even
  when `apply_sensor_settings` is `false`, because it changes the depth model.

For multiple cameras, each YAML should at least use a different:

- `device`
- `frame_id`

Strongly recommended:

- use stable serial paths such as `/dev/serial/by-id/...`
- use distinct YAML filenames so the auto-derived namespaces do not collide

Live update:

```bash
ros2 param set /sipeed_tof_node quantization_unit 1
```

The node will stop the stream, send `AT+UNIT=<value>`, and restart the stream
without requiring a full restart.

If you use the auto-derived multi-camera namespacing, the parameter path also
changes. For example, with `front_tof.yaml`:

```bash
ros2 param set /front_tof/front_tof_node quantization_unit 1
```

If a live reconfigure or startup reconfigure makes the camera stop producing
frames, the driver now falls back to the baseline stream automatically so
publishing can recover.


## TF and RViz Notes

This sensor publishes in `frame_id`, and RViz often uses `base_link` as the
fixed frame. For RViz to display the cloud, TF must know how to transform:

```text
base_link -> ... -> camera_optical_frame
```

If your robot uses a frame prefix, the actual TF frame may be something like:

```text
my_robot_camera_optical_frame
```

In that case, set:

```yaml
frame_id: "my_robot_camera_optical_frame"
```

If RViz sometimes shows:

```text
Could not transform from [camera_optical_frame] to [base_link]
```

but works most of the time, that is usually a timestamp alignment issue, not a
broken frame tree. In that case:

1. keep the correct `frame_id`
2. raise `timestamp_offset_ms`

Suggested values:

- start with `100`
- if needed, try `150`
- if needed, try `200`

Higher values reduce TF errors but add more visual lag.


## Troubleshooting

### No points in RViz

Check:

- the node is running
- `/cloud` is publishing
- `frame_id` matches the real TF frame
- RViz fixed frame is correct

Useful commands:

```bash
ros2 topic echo /cloud --once
ros2 topic hz /cloud
```


### Intermittent RViz transform errors

Increase:

```yaml
timestamp_offset_ms: 150
```

If that still drops frames:

```yaml
timestamp_offset_ms: 200
```


### Wrong real-world scale

Remember:

- the raw `depth` image is not metric depth
- only the point cloud uses the metric conversion
- if you change sensor mode, re-check scale


### Node starts but camera setup fails

Set:

```yaml
apply_sensor_settings: false
```

This avoids extra setup commands that some firmware variants may not like.
Note that `binning` and `quantization_unit` are still enforced separately at
startup.


## Known Limitations

- Linux serial path only
- current metric depth uses the sensor's documented unit-dependent conversion,
  not a measurement-based LUT
- `frame_struct.h` still uses the old flexible-array style payload definition,
  which causes a pedantic compiler warning


## Package Layout

```text
ros2/
  CMakeLists.txt
  package.xml
  README.md
  config/
    tof_params.yaml
  launch/
    tof_node.launch.py
  src/
    main.cc
    serial.cc
    serial.hh
    frame_handle.cc
    frame_struct.h
    cJSON.c
    cJSON.h
```
