# Sipeed MS-A010 ROS 2 Driver

## Overview

This package provides a ROS 2 driver for the Sipeed MS-A010 ToF sensor.

This local version is not trying to be a generic driver for many devices and
many firmware variations. It is set up for one real sensor in one real robot
stack, with practical defaults for:

- raw depth image publishing
- colored point cloud publishing
- `CameraInfo` publishing
- launch + YAML configuration
- TF-friendly timestamping for RViz

The package name is:

`sipeed_tof_ms_a010`


## What It Publishes

Topics:

- `depth`
  Raw 8-bit depth image from the sensor as `sensor_msgs/msg/Image`
- `cloud`
  Colored `sensor_msgs/msg/PointCloud2`
- `depth/camera_info`
  `sensor_msgs/msg/CameraInfo`

Important:

- The `depth` image is the raw `mono8` payload from the sensor
- The point cloud is where the driver converts raw values into metric `x/y/z`


## Current Depth Model

This driver currently uses the sensor's documented unit-dependent depth
conversion:

```text
if UNIT == 0:
  z = (raw / 5.1)^2 / 1000

if UNIT > 0:
  z = raw * UNIT / 1000
```

Then:

```text
x = (u - u0) * z / fx
y = (v - v0) * z / fy
```

Where `fx`, `fy`, `u0`, and `v0` come from the camera coefficients returned by
`AT+COEFF?`.

This means:

- `depth` stays raw
- `cloud` is metric
- changing `quantization_unit` changes how the raw 8-bit payload maps to meters


## Features In This Local Version

- ROS 2 launch file
- YAML configuration file
- `depth/camera_info` publishing
- clean serial helper and command handling
- optional sensor-side setup commands
- backward-compatible `publisher` executable
- friendlier `sipeed_tof_node` executable
- timestamp offset parameter to reduce intermittent RViz TF errors


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
  Publish loop period in milliseconds
- `timestamp_offset_ms`
  Stamps messages slightly in the past so RViz can find the matching TF more
  reliably
- `apply_sensor_settings`
  Enables the extra `AT+BINN`, `AT+FPS`, and `AT+UNIT` commands during
  startup. The default is `false` so the node comes up on the known-good
  baseline stream first.
- `binning`
  Sensor binning mode
- `display_mode`
  Sensor display/output mode
- `fps`
  Sensor frame rate command value
- `quantization_unit`
  Sensor quantization unit command value

For multiple cameras, each YAML should at least use a different:

- `device`
- `frame_id`

Live update:

```bash
ros2 param set /sipeed_tof_node quantization_unit 1
```

The node will stop the stream, send `AT+UNIT=<value>`, and restart the stream
without requiring a full restart.

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


## Calibration Notes

Right now the driver uses the GitHub repo's depth model on purpose.

Because this setup is for one fixed device, the most accurate long-term
approach would be a per-device calibration, ideally:

- a lookup table from raw sensor value to meters, or
- a fitted curve validated against real measurements

For now, the driver intentionally uses the GitHub repo's method because that is
the behavior you requested to trust and keep.


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


## Known Limitations

- Linux serial path only
- current metric depth comes from the GitHub repo formula, not from a
  measurement-based LUT
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
