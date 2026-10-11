# Copyright 2026 Universidad Politécnica de Madrid
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


import argparse
import os

import cv2
import numpy as np

Coord3D = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]
Localization = tuple[Coord3D, Quaternion]

# Camera intrinsics (from dataset_info.yaml)
IMAGE_WIDTH = 1640
IMAGE_HEIGHT = 1232

K = np.array(
    [[365.77364921569824, 0.0, 820.0], [0.0, 365.773645401001, 616.0], [0.0, 0.0, 1.0]],
    dtype=np.float64,
)

# Distortion coefficients [k1, k2, p1, p2, k3] - plumb_bob model
DIST_COEFFS = np.zeros((5, 1), dtype=np.float64)  # all zero = no distortion

OUTER_GATE_SIZE = 2.7
INNER_GATE_SIZE = 1.5


def load_dataset(dataset_path: str) -> list[tuple[cv2.Mat, list[Localization]]]:
    dataset = []
    images_path = os.path.join(dataset_path, 'images')
    if not os.path.exists(images_path):
        print(f'Images directory not found: {images_path}')
        return dataset

    labels_path = os.path.join(dataset_path, 'labels')
    if not os.path.exists(labels_path):
        print(f'Labels directory not found: {labels_path}')
        return dataset

    for filename in os.listdir(images_path):
        if filename.endswith('.jpg') or filename.endswith('.png'):
            image_path = os.path.join(images_path, filename)
            image: np.ndarray | None = cv2.imread(image_path)
            if image is None:
                print(f'Failed to load image: {image_path}')
                continue
            h, w = image.shape[:2]

            label_path = os.path.join(
                labels_path, filename.replace('.jpg', '.txt').replace('.png', '.txt')
            )
            labels: list[Localization] = []
            with open(label_path, 'r') as f:
                for line in f:
                    parts = line.strip().split()
                    parts = list(map(float, parts))
                    coords: Coord3D = (parts[0], parts[1], parts[2])
                    orient: Quaternion = (parts[3], parts[4], parts[5], parts[6])
                    labels.append((coords, orient))

            dataset.append((image, labels))

    return dataset


def translation_error(t1: Coord3D, t2: Coord3D) -> float:
    v1 = np.array(list(t1))
    v2 = np.array(list(t2))
    return float(np.linalg.norm(v1 - v2))


def rotation_error(o1: Quaternion, o2: Quaternion):
    q1 = np.array(o1)
    q2 = np.array(o2)

    q1 = q1 / np.linalg.norm(q1)
    q2 = q2 / np.linalg.norm(q2)

    dot = abs(np.dot(q1, q2))

    dot = np.clip(dot, 0.0, 1.0)

    angle_rad = 2.0 * np.arccos(dot)
    return np.degrees(angle_rad)


def localize_gate(image: np.ndarray) -> list[Localization]:
    coord_X = 0
    coord_Y = 0
    coord_Z = 0
    quat_x = 0
    quat_y = 0
    quat_z = 0
    quat_w = 0
    # TODO (Exercise 3): Use the code from previous tasks to calculate the camera position relative
    # to the gate. Use OUTER_GATE_SIZE and INNER_GATE_SIZE constants defined above
    return [((coord_X, coord_Y, coord_Z), (quat_x, quat_y, quat_z, quat_w))]


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Load a dataset of images and segmentation labels.'
    )
    parser.add_argument(
        '--dataset_path',
        type=str,
        help='Path to the dataset directory',
        default='../datasets/positions',
    )
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_arguments()
    dataset = load_dataset(args.dataset_path)
    print(f'Loaded {len(dataset)} images and their corresponding labels.')

    result: list[tuple[np.ndarray, list[Localization], list[Localization]]] = []
    for image, labels in dataset:
        calculated_position: list[Localization] = localize_gate(image)
        result.append((image, labels, calculated_position))

    translation_errors: list[float] = []
    orientation_errors: list[float] = []

    for r in result:
        image: np.ndarray = r[0]
        gt_pose: Localization = r[1][0]
        calculated_pose: Localization = r[2][0]

        t_error = translation_error(gt_pose[0], calculated_pose[0])
        r_error = rotation_error(gt_pose[1], gt_pose[1])

        translation_errors.append(t_error)
        orientation_errors.append(r_error)

    print('Mean translation error = ', np.mean(translation_errors))
    print('Mean orientation error = ', np.mean(orientation_errors))
