#include <config_reader/config_reader.h>
#include <gflags/gflags.h>
#include <glog/logging.h>

#include <iostream>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <thread>

#include "bev/bev.h"

CONFIG_STRING(camera_calibration_config_path,
              "BEVParameters.camera_calibration_config_path");
CONFIG_UINT(input_image_width, "BEVParameters.input_image_width");
CONFIG_UINT(input_image_height, "BEVParameters.input_image_height");
CONFIG_FLOAT(bev_pixels_per_meter, "BEVParameters.bev_pixels_per_meter");
CONFIG_FLOAT(bev_horizon_distance, "BEVParameters.bev_horizon_distance");

CONFIG_FLOAT(T_ground_camera_x, "BEVParameters.T_ground_camera.x");
CONFIG_FLOAT(T_ground_camera_y, "BEVParameters.T_ground_camera.y");
CONFIG_FLOAT(T_ground_camera_z, "BEVParameters.T_ground_camera.z");
CONFIG_FLOAT(T_ground_camera_pitch, "BEVParameters.T_ground_camera.pitch");

DEFINE_string(config, "config/bev_node.lua", "path to config file");

Eigen::Affine3f Read_T_ground_camera() {
  return Eigen::Translation3f(CONFIG_T_ground_camera_x,
                              CONFIG_T_ground_camera_y,
                              CONFIG_T_ground_camera_z) *
         Eigen::AngleAxisf(CONFIG_T_ground_camera_pitch,
                           Eigen::Vector3f::UnitY());
}

Eigen::Matrix3f ReadIntrinsicMatrix() {
  cv::FileStorage camera_settings(CONFIG_camera_calibration_config_path,
                                  cv::FileStorage::READ);

  cv::FileNode node = camera_settings["K"];
  if (!node.empty()) {
    Eigen::Matrix3f intrinsics_matrix;
    cv::cv2eigen(node.mat(), intrinsics_matrix);
    return intrinsics_matrix;
  } else {
    LOG(FATAL) << "Could not read intrinsics matrix";
    throw std::runtime_error("");
  }
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  config_reader::ConfigReader config_reader_({FLAGS_config});


  bev::BirdsEyeView bev(
      ReadIntrinsicMatrix(), Read_T_ground_camera(), CONFIG_input_image_height,
      CONFIG_input_image_width, CONFIG_bev_pixels_per_meter,
      CONFIG_bev_horizon_distance);

  cv::Mat3b input_image(CONFIG_input_image_height, CONFIG_input_image_width);

  const unsigned int max_threads = std::thread::hardware_concurrency();
  for (unsigned int cv_threads = 1; cv_threads <= max_threads; cv_threads++) {
    cv::setNumThreads(cv_threads);

    std::cout << "Threads: " << cv_threads << "\n";

    double seconds_elapsed = 0.0;
    int runs = 0;

    while (seconds_elapsed < 3.0 || runs < 10) {
      auto start = std::chrono::steady_clock::now();
      cv::Mat3b bev_image = bev.WarpPerspective(input_image);
      auto end = std::chrono::steady_clock::now();

      auto fn_elapsed = std::chrono::duration<double>(end - start);

      runs++;
      seconds_elapsed += fn_elapsed.count();
    }

    double mean_runtime_seconds = seconds_elapsed / runs;
    std::cout << "Mean: " << mean_runtime_seconds * 1000 << " ms per call\n";
    std::cout << "      " << 1 / mean_runtime_seconds << " Hz\n------\n";
  }

  return 0;
}
