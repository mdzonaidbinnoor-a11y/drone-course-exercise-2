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


#include <iostream>
#include <fstream>
#include <filesystem>
#include <vector>
#include <string>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <opencv2/opencv.hpp>

namespace fs = std::filesystem;

// Type aliases
using Coord3D = std::array<double, 3>;
using Quaternion = std::array<double, 4>;
using Localization = std::pair<Coord3D, Quaternion>;
using DatasetEntry = std::pair<cv::Mat, std::vector<Localization>>;

constexpr int IMAGE_WIDTH = 1640;
constexpr int IMAGE_HEIGHT = 1232;

const cv::Mat K = (cv::Mat_<double>(3, 3) <<
  365.77364921569824, 0.0, 820.0,
  0.0, 365.773645401001, 616.0,
  0.0, 0.0, 1.0
);

const float OUTER_GATE_SIZE = 2.7;
const float INNER_GATE_SIZE = 1.5;

// Distortion coefficients [k1, k2, p1, p2, k3] - plumb_bob, all zero = no distortion
const cv::Mat DIST_COEFFS = cv::Mat::zeros(5, 1, CV_64F);

double quaternionNorm(const Quaternion & q)
{
  return std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

Quaternion quaternionNormalize(const Quaternion & q)
{
  double n = quaternionNorm(q);
  return {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
}

double quaternionDot(const Quaternion & q1, const Quaternion & q2)
{
  return q1[0] * q2[0] + q1[1] * q2[1] + q1[2] * q2[2] + q1[3] * q2[3];
}


double translationError(const Coord3D & t1, const Coord3D & t2)
{
  double dx = t1[0] - t2[0], dy = t1[1] - t2[1], dz = t1[2] - t2[2];
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

double rotationError(const Quaternion & o1, const Quaternion & o2)
{
  Quaternion q1 = quaternionNormalize(o1);
  Quaternion q2 = quaternionNormalize(o2);
  double dot = std::abs(quaternionDot(q1, q2));
  dot = std::clamp(dot, 0.0, 1.0);
  double angle_rad = 2.0 * std::acos(dot);
  return angle_rad * (180.0 / M_PI);
}


std::vector<DatasetEntry> loadDataset(const std::string & datasetPath)
{
  std::vector<DatasetEntry> dataset;

  fs::path imagesPath = fs::path(datasetPath) / "images";
  fs::path labelsPath = fs::path(datasetPath) / "labels";

  if (!fs::exists(imagesPath)) {
    std::cerr << "Images directory not found: " << imagesPath << "\n";
    return dataset;
  }
  if (!fs::exists(labelsPath)) {
    std::cerr << "Labels directory not found: " << labelsPath << "\n";
    return dataset;
  }

  for (const auto & entry : fs::directory_iterator(imagesPath)) {
    std::string filename = entry.path().filename().string();
    std::string ext = entry.path().extension().string();

    if (ext != ".jpg" && ext != ".png") {continue;}

    cv::Mat image = cv::imread(entry.path().string());
    if (image.empty()) {
      std::cerr << "Failed to load image: " << entry.path() << "\n";
      continue;
    }

    // Build label path by replacing extension with .txt
    std::string labelFilename = filename.substr(0, filename.find_last_of('.')) + ".txt";
    fs::path labelPath = labelsPath / labelFilename;

    std::vector<Localization> labels;
    std::ifstream labelFile(labelPath);
    if (!labelFile.is_open()) {
      std::cerr << "Failed to open label: " << labelPath << "\n";
      continue;
    }

    std::string line;
    while (std::getline(labelFile, line)) {
      std::istringstream iss(line);
      std::vector<double> parts;
      double val;
      while (iss >> val) {parts.push_back(val);}
      if (parts.size() < 7) {continue;}

      Coord3D coords = {parts[0], parts[1], parts[2]};
      Quaternion orient = {parts[3], parts[4], parts[5], parts[6]};
      labels.push_back({coords, orient});
    }

    dataset.push_back({image, labels});
  }

  return dataset;
}


std::vector<Localization> localizeGate(const cv::Mat & image)
{
  // TODO(Exercise 3): Use PnP to calculate the camera position relative to the gate.
  // Use INNER_GATE_SIZE and OUTER_GATE_SIZE constants defined above
  Coord3D coord = {0, 0, 0};
  Quaternion quat = {0, 0, 0, 0};
  return {{coord, quat}};
}


int main(int argc, char ** argv)
{
  std::string datasetPath = "../datasets/yolo_pose";
  for (int i = 1; i < argc; ++i) {
    if (std::string(argv[i]) == "--dataset_path" && i + 1 < argc) {
      datasetPath = argv[++i];
    }
  }

  auto dataset = loadDataset(datasetPath);
  std::cout << "Loaded " << dataset.size() << " images and their corresponding labels.\n";

  std::vector<double> translationErrors;
  std::vector<double> orientationErrors;

  for (const auto & [image, labels] : dataset) {
    auto calculatedPose = localizeGate(image);

    const Localization & gtPose = labels[0];
    const Localization & calcPose = calculatedPose[0];

    translationErrors.push_back(translationError(gtPose.first, calcPose.first));
    orientationErrors.push_back(rotationError(gtPose.second, calcPose.second));
  }

  auto mean = [](const std::vector<double> & v) {
      return std::accumulate(v.begin(), v.end(), 0.0) / v.size();
    };

  std::cout << "Mean translation error = " << mean(translationErrors) << "\n";
  std::cout << "Mean orientation error = " << mean(orientationErrors) << "\n";

  return 0;
}
