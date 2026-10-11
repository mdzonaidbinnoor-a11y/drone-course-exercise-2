// Copyright 2026 Universidad Politécnica de Madrid
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Universidad Politécnica de Madrid nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.


#include <filesystem>
#include <fstream>
#include <iostream>
#include <numeric>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

using Point = std::pair<float, float>;

struct SegmentationLabel
{
  std::vector<Point> polygon;
  int class_id = -1;
};

std::vector<SegmentationLabel> segment_image(const cv::Mat & image)
{
  // TODO(Exercise 1): Implement an image segmentation algorithm that takes
  // an image as input and returns a list of SegmentationLabel objects.
  (void)image;
  return {};
}

float compute_iou(
  const cv::Mat & image,
  const std::vector<SegmentationLabel> & labels,
  const std::vector<SegmentationLabel> & segmentation)
{
  // TODO(Exercise 1): Implement a function that computes the Intersection
  // over Union (IoU) score between the ground truth labels and the predicted
  // segmentation.
  (void)image;
  (void)labels;
  (void)segmentation;
  return 0.0f;
}

float compute_precision(
  const cv::Mat & image,
  const std::vector<SegmentationLabel> & labels,
  const std::vector<SegmentationLabel> & segmentation)
{
  // TODO(Exercise 1): Implement a function that computes the Precision score
  // between the ground truth labels and the predicted segmentation.
  (void)image;
  (void)labels;
  (void)segmentation;
  return 0.0f;
}

float compute_recall(
  const cv::Mat & image,
  const std::vector<SegmentationLabel> & labels,
  const std::vector<SegmentationLabel> & segmentation)
{
  // TODO(Exercise 1): Implement a function that computes the Recall score
  // between the ground truth labels and the predicted segmentation.
  (void)image;
  (void)labels;
  (void)segmentation;
  return 0.0f;
}

using DatasetEntry = std::pair<cv::Mat, std::vector<SegmentationLabel>>;

std::vector<DatasetEntry> load_dataset(const std::string & dataset_path)
{
  std::vector<DatasetEntry> dataset;

  fs::path images_path = fs::path(dataset_path) / "images";
  if (!fs::exists(images_path)) {
    std::cout << "Images directory not found: " << images_path << std::endl;
    return dataset;
  }

  fs::path labels_path = fs::path(dataset_path) / "labels";
  if (!fs::exists(labels_path)) {
    std::cout << "Labels directory not found: " << labels_path << std::endl;
    return dataset;
  }

  for (const auto & entry : fs::directory_iterator(images_path)) {
    std::string ext = entry.path().extension().string();
    if (ext != ".jpg" && ext != ".png") {
      continue;
    }

    cv::Mat image = cv::imread(entry.path().string());
    if (image.empty()) {
      std::cout << "Failed to load image: " << entry.path() << std::endl;
      continue;
    }

    std::string label_filename = entry.path().stem().string() + ".txt";
    fs::path label_path = labels_path / label_filename;

    std::vector<SegmentationLabel> labels;
    std::ifstream label_file(label_path);
    if (label_file.is_open()) {
      std::string line;
      while (std::getline(label_file, line)) {
        std::istringstream iss(line);
        SegmentationLabel label;
        iss >> label.class_id;

        std::vector<float> coords;
        float val;
        while (iss >> val) {
          coords.push_back(val);
        }

        for (size_t i = 0; i + 1 < coords.size(); i += 2) {
          label.polygon.emplace_back(coords[i], coords[i + 1]);
        }
        labels.push_back(std::move(label));
      }
    }

    dataset.emplace_back(std::move(image), std::move(labels));
  }
  return dataset;
}

int main(int argc, char * argv[])
{
  std::string dataset_path = "../datasets/yolo_masks";
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--dataset_path" && i + 1 < argc) {
      dataset_path = argv[++i];
    }
  }

  auto dataset = load_dataset(dataset_path);
  std::cout << "Loaded " << dataset.size()
            << " images and their corresponding labels." << std::endl;

  std::vector<std::vector<SegmentationLabel>> result;
  for (const auto & [image, labels] : dataset) {
    result.push_back(segment_image(image));
  }

  std::vector<float> iou_scores;
  std::vector<float> recall_scores;
  std::vector<float> precision_scores;

  for (size_t i = 0; i < dataset.size(); ++i) {
    const auto & [image, labels] = dataset[i];
    const auto & segmentation = result[i];

    iou_scores.push_back(compute_iou(image, labels, segmentation));
    precision_scores.push_back(compute_precision(image, labels, segmentation));
    recall_scores.push_back(compute_recall(image, labels, segmentation));
  }

  auto avg = [](const std::vector<float> & v) -> float {
      return v.empty() ? 0.0f :
             std::accumulate(v.begin(), v.end(), 0.0f) / static_cast<float>(v.size());
    };

  std::cout << std::fixed;
  std::cout.precision(4);
  std::cout << "Average IoU score: " << avg(iou_scores) << std::endl;
  std::cout << "Average Precision score: " << avg(precision_scores) << std::endl;
  std::cout << "Average Recall score: " << avg(recall_scores) << std::endl;

  return 0;
}
