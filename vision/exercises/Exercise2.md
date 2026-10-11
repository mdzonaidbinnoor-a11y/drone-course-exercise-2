# Exercise 2: Gate Corner Detection

## Objective

Implement a corner detection algorithm to locate the keypoints of drone racing gates in images, using classical computer vision techniques.

## Background

The dataset contains images of simulated racing gates paired with ground-truth **keypoint labels** in **YOLO pose format**. Each gate has 8 annotated keypoints — the four outer corners and four inner corners of the gate frame.

### Keypoint layout

```
top_left_outer (0)       top_right_outer (1)
     +-----------------------------+
     |  top_left_inner (2)  top_right_inner (3)
     |       +-----------+        |
     |       |           |        |
     |       +-----------+        |
     |  bot_left_inner (4)  bot_right_inner (5)
     +-----------------------------+
bot_left_outer (6)       bot_right_outer (7)
```

### Label format

One object per line:

```
<class_id> <cx> <cy> <w> <h>  <x0> <y0> <v0>  <x1> <y1> <v1>  ...  <x7> <y7> <v7>
```

- `cx cy w h`: normalized bounding box center and size (skipped by the loader).
- `xi yi`: normalized keypoint coordinates.
- `vi`: visibility flag (`2` = visible).

The dataset loader already parses these labels into pixel-coordinate `(x, y)` tuples and sorts them top-to-bottom, left-to-right via `sort_corners`. For this exercise, the visibility flag is ignored.

The dataset is located at:

```
../../datasets/yolo_pose/
├── images/       # .jpg images
├── labels/       # .txt label files (same stem as the image)
└── dataset.yaml
```

## Task

Open `exercise_2.py` and implement:

### `detect_corners(image)`

Takes a BGR image (`np.ndarray`) as input and returns a list of `CornerLabel` — i.e., `(x, y)` pixel-coordinate tuples for each detected corner.

## Running the Script

From the `vision/python_interface/exercise_2/` directory:

```bash
python exercise_2.py
```

To specify a custom dataset path:

```bash
python exercise_2.py --dataset_path /path/to/yolo_pose
```

Expected output:

```
Loaded 90 images and their corresponding labels.
Average distance: 1.0
```
## Running the C++ program

From the `vision/vision_ws` directory run:

```bash
colcon build --packages-select vision_exercise_2
```

Then source the workspace.

```bash
source install/setup.bash
```

Then you are ready to run your program with:

```bash
ros2 run vision_exercise_2 exercise_2 --dataset_path <absolute path to the dataset> 
```

## Deliverables

1. **Code** — submit either a `.zip` archive of your solution or a link to your fork of the repository on GitHub.
2. **Demo video** — record a short video showing your solution running on the dataset. Verbally explain the approach you took, the key design decisions, and the results you obtained.

## Tips

The script currently loads the dataset and calls `detect_corners`  To verify your implementation during development, you could add visualization inside the main loop:

```python
for image, labels in dataset:
    corners = detect_corners(image)
    vis = image.copy()
    for (x, y) in labels:
        cv2.circle(vis, (int(x), int(y)), 6, (0, 255, 0), -1)   # ground truth: green
    for (x, y) in corners:
        cv2.circle(vis, (int(x), int(y)), 4, (0, 0, 255), -1)   # detected: red
    cv2.imshow('corners', vis)
    cv2.waitKey(0)
```


