# Exercise 1: Gate Image Segmentation

## Objective

Implement a classic computer vision pipeline to segment drone racing gates in images. You will also implement standard evaluation metrics to measure the quality of your segmentation.

## Background

The dataset contains images captured from a simulated drone flying around racing gates. Each image is paired with a ground-truth label file in **YOLO segmentation format**: a polygon that tightly outlines each gate in the image.

Label format (one object per line):

```
<class_id> <x1> <y1> <x2> <y2> ... <xn> <yn>
```

Coordinates are **normalized** (values in `[0, 1]`) relative to image width and height. The only class in this dataset is `0: gate`.

The dataset is located at:

```
project_upm_drone_course/datasets/yolo_masks/
├── images/   # .jpg images
├── labels/   # .txt label files (same stem as the image)
└── dataset.yaml
```

## Tasks

Open `exercise_1.py` and implement the three functions marked with `TODO (Exercise 1)`.
Alternatively, open `vision/vision_ws/src/exercise_1/exercise_1.cpp`and complete the functions marked with `TODO (Exercise 1)`.

### 1. `segment_image(image)`

Implement an image segmentation algorithm that takes a BGR image (`np.ndarray`) as input and returns a list of `SegmentationLabel` objects. Each object must have:

- `polygon`: list of `(x, y)` pixel-coordinate points forming the detected region contour.
- `class_id`: integer class identifier (use `0` for gate).


### 2. `compute_iou(image, labels, segmentation)`

Compute the **intersection over union** score of a segmentation.

### 3. `compute_precision(image, labels, segmentation)`

Compute the **pixelwise precision** of the segmentation.

### 4. `compute_recall(image, labels, segmentation)`

Compute the **pixelwise recall** of the segmentation.

## Running the python script

From the `vision/python_interface/exercise_1/` directory:

```bash
python exercise_1.py
```

To specify a custom dataset path:

```bash
python exercise_1.py --dataset_path /path/to/yolo_masks
```

## Running the C++ program

From the `vision/vision_ws` directory run:

```bash
colcon build --packages-select vision_exercise_1
```

Then source the workspace.

```bash
source install/setup.bash
```

Then you are ready to run your program with:

```bash
ros2 run vision_exercise_1 exercise_1 --dataset_path <absolute path to the dataset> 
```


## Tips

- Inspect several images and their labels before coding. Use `cv2.imshow` or save intermediate results to understand the appearance of the gates.
- Gates are typically rectangular structures with distinct color or contrast compared to the background.
- Tune your thresholds on a subset of images before running the full evaluation.

## Deliverables

1. **Code** — submit either a `.zip` archive of your solution or a link to your fork of the repository on GitHub.
2. **Demo video** — record a short video showing your solution running on the dataset. Verbally explain the approach you took, the key design decisions, and the results you obtained.

