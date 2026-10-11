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


#include <algorithm>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

using CornerLabel = std::pair<float, float>;

std::vector<CornerLabel> sort_corners(std::vector<CornerLabel> points)
{
  std::sort(
    points.begin(), points.end(), [](const CornerLabel & a, const CornerLabel & b) {
      if (a.second != b.second) {return a.second < b.second;}
      return a.first < b.first;
    });
  return points;
}

std::vector<CornerLabel> detect_corners(const cv::Mat & image)
{
  // TODO(Exercise 2): Implement a corner detection algorithm that takes an
  // image as input and returns a list of CornerLabel objects.
  (void)image;
  return {};
}

using DatasetEntry = std::pair<cv::Mat, std::vector<CornerLabel>>;

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

    std::vector<CornerLabel> labels;
    std::ifstream label_file(label_path);
    if (label_file.is_open()) {
      std::string line;
      while (std::getline(label_file, line)) {
        std::istringstream iss(line);
        std::vector<float> parts;
        float val;
        while (iss >> val) {
          parts.push_back(val);
        }

        // Skip first 5 values
        for (size_t i = 5; i + 2 < parts.size(); i += 3) {
          float x = parts[i];
          float y = parts[i + 1];
          labels.emplace_back(x, y);
        }
      }
    }

    labels = sort_corners(labels);
    dataset.emplace_back(std::move(image), std::move(labels));
  }
  return dataset;
}

int main(int argc, char * argv[])
{
  std::string dataset_path = "../datasets/yolo_pose";
  for (int i = 1; i < argc; ++i) {
    std::string arg = argv[i];
    if (arg == "--dataset_path" && i + 1 < argc) {
      dataset_path = argv[++i];
    }
  }

  auto dataset = load_dataset(dataset_path);
  std::cout << "Loaded " << dataset.size()
            << " images and their corresponding labels." << std::endl;

  std::vector<std::vector<CornerLabel>> result;
  for (const auto & [image, labels] : dataset) {
    result.push_back(sort_corners(detect_corners(image)));
  }

  // Calculate euclidean distance between detected corners and ground truth labels
  std::vector<float> distances;
  for (size_t i = 0; i < result.size(); ++i) {
    const auto & corners = result[i];
    const auto & labels = dataset[i].second;
    for (size_t j = 0; j < std::min(
        corners.size(),
        labels.size()); ++j)
    {
      float distance = std::sqrt(
        std::pow(corners[j].first - labels[j].first, 2) +
        std::pow(corners[j].second - labels[j].second, 2));
      distances.push_back(distance);
    }
  }

  float average_distance = 0;
  if (!distances.empty()) {
    for (float d : distances) {
      average_distance += d;
    }
    average_distance /= distances.size();
  }
  std::cout << "Average distance: " << average_distance << std::endl;

  return 0;
}
