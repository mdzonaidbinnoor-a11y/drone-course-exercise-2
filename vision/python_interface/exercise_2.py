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

CornerLabel = tuple[float, float]


def sort_corners(points):
    return sorted(points, key=lambda p: (p[1], p[0]))


def detect_corners(image: np.ndarray) -> list[CornerLabel]:
    # TODO (Exercise 2): Implement a corner detection algorithm that takes an image as input and returns a list of CornerLabel objects.
    return []


def load_dataset(dataset_path: str) -> list[tuple[cv2.Mat, list[CornerLabel]]]:
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
            labels: list[CornerLabel] = []
            with open(label_path, 'r') as f:
                for line in f:
                    parts = line.strip().split()
                    parts = parts[5:]
                    for i in range(0, len(parts), 3):
                        x = float(parts[i])
                        y = float(parts[i + 1])
                        labels.append((x, y))

            labels = sort_corners(labels)

            dataset.append((image, labels))
    return dataset


def parse_arguments():
    parser = argparse.ArgumentParser(
        description='Load a dataset of images and segmentation labels.'
    )
    parser.add_argument(
        '--dataset_path',
        type=str,
        help='Path to the dataset directory',
        default='../datasets/yolo_pose',
    )
    return parser.parse_args()


if __name__ == '__main__':
    args = parse_arguments()
    dataset = load_dataset(args.dataset_path)
    print(f'Loaded {len(dataset)} images and their corresponding labels.')

    result: list[list[CornerLabel]] = []
    for image, labels in dataset:
        corners = detect_corners(image)
        corners = sort_corners(corners)
        result.append(corners)

    distances = [1]
    for corners, labels in zip(result, dataset):
        for corner, label in zip(corners, labels[1]):
            distance = np.sqrt((corner[0] - label[0]) ** 2 + (corner[1] - label[1]) ** 2)
            distances.append(distance)

    print(f'Average distance: {np.mean(distances)}')
