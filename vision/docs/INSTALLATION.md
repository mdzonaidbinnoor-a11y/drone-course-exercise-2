# Installation and Prerequisites

This document describes both supported setup paths:

- Native Ubuntu + ROS 2 Humble (Python and C++)
- Docker

## Prerequisites (Native)

- Ubuntu with ROS 2 Humble installed.
- Python 3 with `pip` available.
- `colcon` build tool available.

Install Python dependencies:

```bash
pip install opencv-python numpy
```

### Native Setup

1. Source ROS 2 Humble:

```bash
source /opt/ros/humble/setup.bash
```

2. Install workspace dependencies:

```bash
cd vision/vision_ws
sudo rosdep init
rosdep update
rosdep install --from-paths src -y --ignore-src
```

3. Build workspace:

```bash
cd vision/vision_ws
colcon build --symlink-install
source install/setup.bash
```

## Docker Setup

1. Build the Docker image:

```bash
docker compose build
```

2. Start the container:

```bash
docker compose up -d
```

3. Connect via X forwarding or VNC (default VNC port: `5901`).

## Run the Python Exercises

From the `vision/python_interface` directory:

```bash
python exercise_1.py
python exercise_2.py
```

Pass a custom dataset path if needed:

```bash
python exercise_1.py --dataset_path /path/to/yolo_masks
python exercise_2.py --dataset_path /path/to/yolo_pose
```
