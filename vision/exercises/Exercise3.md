# Exercise 3: Gate Pose Estimation

## Objective

Estimate the 6-DoF pose of a drone racing gate (3D position and orientation) from a single image using the PnP algorithm. You will build on the corner detection from Exercise 2 and evaluate results against ground-truth poses.

## Background

Given a set of 2D image points and their corresponding 3D world points, the **Perspective-n-Point (PnP)** algorithm recovers the rotation and translation that maps the 3D points into the image plane. In this exercise, the 2D points are the gate corners detected in Exercise 2, and the 3D points are the known physical positions of those corners on the gate.

### Label format

Each label file contains one line per gate:

```
<x> <y> <z> <qw> <qx> <qy> <qz> 
```

- `x y z`: 3D position of the gate origin in the camera frame (metres).
- `qx qy qz qw`: unit quaternion representing the gate orientation.

### Dataset

The dataset is located at:

```
../../datasets/positions/
├── images/   # .jpg images
└── labels/   # .txt label files (same stem as the image)
```

## Task

Open `exercise_3.py` and implement the function marked with `TODO (Exercise 3)`.
Alternatively, open `vision/vision_ws/src/exercise_3/src/exercise_3.cpp` and complete the function marked with `TODO (Exercise 3)`.

### `localize_gate(image)`

Takes a BGR image (`np.ndarray`) as input and returns a list of `Localization` objects. Each `Localization` is a tuple:

```python
((x, y, z), (qw, qx, qy, qz))
```

The recommended approach is:

1. **Detect corners** — reuse your `detect_corners` implementation from Exercise 2 to obtain the 2D image coordinates of the gate corners.
2. **Define 3D model points** — build the corresponding 3D coordinates of the gate corners in the gate's local frame using the known gate dimensions.
3. **Set camera intrinsics** — construct the camera matrix `K` from the focal lengths and principal point. These parameters can be read from the image metadata or assumed from the simulation setup.
4. **Solve PnP** — call `cv2.solvePnP` (Python) or `cv::solvePnP` (C++) to obtain the rotation vector and translation vector.
5. **Convert to quaternion** — convert the rotation vector to a rotation matrix (`cv2.Rodrigues`) and then to a unit quaternion.

## Running the Python script

From the `vision/python_interface/exercise_3/` directory:

```bash
python exercise_3.py
```

To specify a custom dataset path:

```bash
python exercise_3.py --dataset_path /path/to/positions
```

Expected output:

```
Loaded 90 images and their corresponding labels.
Mean translation error =  X.XX
Mean orientation error =  X.XX
```

## Running the C++ program

From the `vision/vision_ws` directory run:

```bash
colcon build --packages-select vision_exercise_3
```

Then source the workspace:

```bash
source install/setup.bash
```

Then run your program with:

```bash
ros2 run vision_exercise_3 exercise_3 --dataset_path <absolute path to the dataset>
```

## Tips

- Start by hardcoding the gate's 3D corner positions. A square gate of side length `s` centred at the origin has outer corners at `(±s/2, ±s/2, 0)`.
- `cv2.solvePnP` returns a rotation **vector** (Rodrigues). Use `cv2.Rodrigues` to convert it to a 3×3 rotation matrix, then extract the quaternion.
- If `solvePnP` fails or produces unreasonable results, check that the 2D–3D point correspondences are consistent (same ordering in both lists).
- Errors in corner detection propagate directly to the pose estimate — a robust corner detector from Exercise 2 will significantly improve results here.

## Deliverables

1. **Code** — submit either a `.zip` archive of your solution or a link to your fork of the repository on GitHub.
2. **Demo video** — record a short video showing your solution running on the dataset. Verbally explain the approach you took, the key design decisions, and the results you obtained.
