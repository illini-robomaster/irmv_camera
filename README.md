# irmv_camera

> Fork from [ros2-mindvision-camera](https://gitlab.com/rm_vision/ros2-mindvision-camera)

ROS2 MindVision Camera Package providing ROS API

![iRM](docs/iRM.png)

Only tested under Ubuntu 22.04 with ROS2 Humble

![Build Status](https://github.com/illini-robomaster/irmv_camera/actions/workflows/ros_ci.yml/badge.svg)

## Usage

### Build from source

#### Dependencies

- [Robot Operating System 2 (ROS2)](https://docs.ros.org/en/humble/) (middleware for robotics),

#### Building

To build from source, clone the latest version from this repository into your colcon workspace and compile the package using

```shell
mkdir -p ros_ws/src
cd ros_ws/src
git clone https://github.com/illini-robomaster/irmv_camera.git
cd ..
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-up-to mindvision_camera
```

### Calibration

#### Tutorials

- [Calibration Tutorial](https://navigation.ros.org/tutorials/docs/camera_calibration.html)
- [Parameters Reference](http://wiki.ros.org/camera_calibration)

After calibration, the parameters will be stored in `/tmp/calibrationdata.tar.gz`

### Launch Camera Node

```shell
ros2 launch mindvision_camera mv_launch.py
```

Parameters supported：

1. params_file： path for the camera parameters file 
2. camera_info_url： path for the camera intrinsics file
3. use_sensor_data_qos： whether camera Publisher uses SensorDataQoS (default: `true`)

### Dynamically turn camera parameters through RQt

Open RQt

```shell
rqt
```

Add `Configuration -> Dynamic Reconfigure` and `Visualization -> Image View` in Plugins