# UPM Drone Course – Control and Vision Exercises

**Author:** Md Zonaid Bin Noor

This repository contains my solutions for Control Exercise 1, Control Exercise 2, and Vision Exercise 3 of the UPM drone course.

## Demonstration videos

Replace the placeholders below with the uploaded video links.

| Exercise | Video |
| --- | --- |
| Control Exercise 1 | ADD_CONTROL_EXERCISE_1_VIDEO_LINK |
| Control Exercise 2 | ADD_CONTROL_EXERCISE_2_VIDEO_LINK |
| Vision Exercise 3 | ADD_VISION_EXERCISE_3_VIDEO_LINK |

## Control Exercise 1 – Position commands and path following

The first part uses position references to command the simulated drone. The second part requests a path from the course service and follows its targets in sequence.

The completed Exercise 1.2 program:
- Requests the path asynchronously from `/request_path`.
- Selects position control through `/drone0/controller/set_control_mode`.
- Receives position feedback on `/drone0/self_localization/pose`.
- Publishes position references on `/drone0/motion_reference/pose`.
- Advances when the drone is within 0.2 metres of the current target.
- Holds the final target after completing the path.
- Handles unavailable services, empty paths, and stale feedback.

The course's existing position controller tracks these references. A target-reached message alone does not establish a successful gate crossing; the flight is also checked visually in the simulator.

### Run

Start the simulator as described below. In a separate terminal, load the ROS 2 environment and workspace, then run **one** of these programs from the repository root:

```bash
python3 control/python_interface/exercise_1/exercise_1_1.py
```

```bash
python3 control/python_interface/exercise_1/exercise_1_2.py
```

## Control Exercise 2 – Velocity control

This exercise uses velocity commands and controller tuning to guide the simulated drone through the required gate sequence. Position feedback is compared with the current target, and the controller generates velocity references to reduce the error.

The demonstration should show the actual gate crossings and the terminal's progress messages. Controller gains, command limits, and waypoint acceptance conditions should be checked in the submitted source code.

### Run

With the simulator running and the ROS 2 environment loaded, run from the repository root:

```bash
python3 control/python_interface/exercise_2/exercise_2.py
```

This command assumes the completed file is stored at the path above. If the course checkout uses a different filename, use that filename.

## Simulator and ROS 2 setup

The commands below assume Ubuntu 22.04, ROS 2 Humble, and an already installed and built course workspace.

### Terminal 1 – Start the simulator

From the repository root:

```bash
source /opt/ros/humble/setup.bash
cd control
bash launch_as2.bash
```

Wait for the simulator and RViz to start. Leave this terminal running.

### Terminal 2 – Load the controller environment

From the repository root:

```bash
source /opt/ros/humble/setup.bash
source control/drone_course_ws/install/setup.bash
```

Then run the required control exercise. Run only one exercise controller at a time. Press **Ctrl+C** to stop it before starting another.

To check whether position feedback is available:

```bash
ros2 topic echo /drone0/self_localization/pose --once
```

## Vision Exercise 3 – Gate pose estimation

The objective is to estimate the gate's three-dimensional position and orientation from a single image.

### Method

1. Use colour and grayscale masks to identify the gate frame.
2. Detect four outer corners and four inner corners.
3. Order the image corners and match them to the corresponding known 3D gate corners.
4. Use the supplied camera intrinsics and OpenCV's `solvePnP` to estimate rotation and translation.
5. Convert the rotation to a quaternion.
6. Evaluate the estimated poses against the dataset reference labels.

The gate model uses an outer size of 2.7 metres and an inner opening size of 1.5 metres.

The `localize_gate(image)` function returns the gate pose in the OpenCV camera frame, with x right, y down, and z forward:

```python
((x, y, z), (qw, qx, qy, qz))
```

### Coordinate frames

The supplied 65-image dataset labels were found to be consistent with camera pose in a world frame. The evaluation therefore uses `camera_world_pose()` to invert the PnP transformation and convert the axes. This interpretation assumes a fixed gate centre at `(0, 0, 1.35)` metres and world axes x forward, y left, and z up.

The reference labels are used to calculate evaluation errors, rather than to predict each image's pose. The reported errors depend on this documented frame interpretation.

### Results on the supplied dataset

| Metric | Result |
| --- | ---: |
| Images processed | 65 |
| Images with gate detections | 65 |
| Images evaluated | 65 |
| Mean translation error | 0.052057 m (about 5.2 cm) |
| Mean orientation error | 0.431381° |

These results describe the supplied dataset. Performance on unseen scenes has not been established. Corner ordering assumes an upright gate and modest camera roll.

### Run

Use the Python environment containing compatible installations of NumPy, SciPy, and OpenCV. From the repository root:

```bash
python3 vision/python_interface/exercise_3.py --dataset_path "/absolute/path/to/positions" --show
```

Click the image window and press **Space** to advance. Press **Q** to stop the visual demonstration.

For a full automatic evaluation:

```bash
python3 vision/python_interface/exercise_3.py --dataset_path "/absolute/path/to/positions"
```

Replace `/absolute/path/to/positions` with the location of the supplied dataset. If the completed file is still named `exercise_3_corrected.py`, use that filename.

## Acknowledgements

These solutions build on the UPM drone course starter code and simulator. Original copyright and licence notices are retained in the source files.
