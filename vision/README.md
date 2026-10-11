# Vision Track

This track contains the practical computer vision exercises for the drone racing environment.
The progression moves from classical image segmentation to corner keypoint detection and 6-DoF pose estimation on racing gate images.

Before starting, follow the setup guide in [docs/INSTALLATION.md](docs/INSTALLATION.md).
For API links and library references, use [docs/REFERENCES.md](docs/REFERENCES.md).

## Exercises

### Exercise 1: Gate Image Segmentation

Statement: [exercises/Exercise1.md](exercises/Exercise1.md)

You implement a classical computer vision pipeline to segment drone racing gates in images.
The goal is to detect gate regions and evaluate segmentation quality using IoU, precision, and recall against ground-truth YOLO segmentation labels.

### Exercise 2: Gate Corner Detection

Statement: [exercises/Exercise2.md](exercises/Exercise2.md)

You implement a corner detection algorithm to locate the 8 keypoints of a racing gate (four outer and four inner corners) using classical computer vision techniques.
Performance is measured as the average pixel distance between detected and ground-truth corners.

### Exercise 3: Gate Pose Estimation

Statement: [exercises/Exercise3.md](exercises/Exercise3.md)

You estimate the 6-DoF pose (3D position and orientation) of a racing gate from a single image using the PnP algorithm.
Building on the corner detection from Exercise 2, the result is evaluated against ground-truth poses by mean translation and orientation error.

## Code and Workspace

Python templates are available at:
- [python_interface/exercise_1.py](python_interface/exercise_1.py)
- [python_interface/exercise_2.py](python_interface/exercise_2.py)
- [python_interface/exercise_3.py](python_interface/exercise_3.py)

C++ packages are available in [vision_ws/](vision_ws/) under `exercise_1`, `exercise_2`, and `exercise_3`.

## Datasets

The datasets must be downloaded from [this link](https://drive.google.com/drive/folders/1LDprSJ5n-bKzrA1hBUanKm2r8UyMSCgB?usp=sharing) and placed in the following folders:

| Exercise | Dataset | Folder |
|----------|---------|--------|
| Exercise 1 | Segmentation dataset | `datasets/yolo_masks/` |
| Exercise 2 | Keypoint dataset | `datasets/yolo_pose/` |
| Exercise 3 | Pose dataset | `datasets/positions/` |

Download the datasets from the course Moodle page and extract each archive into the corresponding folder listed above. After extraction the structure should look like:

```
datasets/
├── yolo_masks/      # Exercise 1 — segmentation labels
├── yolo_pose/       # Exercise 2 — keypoint labels
└── positions/       # Exercise 3 — pose labels
    ├── images/
    └── labels/
```

## Quick Start

Run the Python interface for Exercise 1:

```bash
cd vision/python_interface
python exercise_1.py
```

Run the Python interface for Exercise 2:

```bash
cd vision/python_interface
python exercise_2.py
```

Run the Python interface for Exercise 3:

```bash
cd vision/python_interface
python exercise_3.py
```

Build and run the C++ packages from `vision/vision_ws`:

```bash
colcon build --packages-select exercise_1
source install/setup.bash
ros2 run vision_exercise_<exercise_number> exercise_<exercise_number> --dataset_path <absolute path to the dataset>
```
