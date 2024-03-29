//========================================================================
//  This software is free: you can redistribute it and/or modify
//  it under the terms of the GNU Lesser General Public License Version 3,
//  as published by the Free Software Foundation.
//
//  This software is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU Lesser General Public License for more details.
//
//  You should have received a copy of the GNU Lesser General Public License
//  Version 3 in the file COPYING that came with this distribution.
//  If not, see <http://www.gnu.org/licenses/>.
//========================================================================
/*!
\file    navigation_main.cc
\brief   Main entry point for reference Navigation implementation
\author  Joydeep Biswas, (C) 2019
*/
//========================================================================

#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <opencv2/core/matx.hpp>
#include <vector>

#include "amrl_msgs/Localization2DMsg.h"
#include "config_reader/config_reader.h"
#include "cv_bridge/cv_bridge.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "image_transport/image_transport.h"
#include "nav_msgs/Odometry.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "ros/message_traits.h"
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/timer.h"
#include "std_msgs/String.h"

using amrl_msgs::Localization2DMsg;
using Eigen::Vector2f;
using std::vector;

CONFIG_STRING(localization_topic, "BEVParameters.localization_topic");
CONFIG_STRING(odometry_topic, "BEVParameters.odometry_topic");
CONFIG_STRING(image_topic, "BEVParameters.image_topic");
CONFIG_INT(image_width, "BEVParameters.image_width");
CONFIG_INT(image_height, "BEVParameters.image_height");
CONFIG_INT(pixels_per_meter, "BEVParameters.pixels_per_meter");
CONFIG_INT(bev_size, "BEVParameters.bev_size");
CONFIG_STRING(camera_calibration_path,
              "BEVParameters.camera_calibration_config_path");

DEFINE_string(config, "config/local_rgb_map.lua", "path to config file");
DEFINE_bool(visualize, false, "show opencv visualization of map");
DEFINE_bool(camera_correction, true,
            "apply distortion and camera matrices to images received");
DEFINE_double(gamma, 0.5, "moving average factor");
DEFINE_double(update_dist, 0.25,
              "distance at which to force an update of the local map");
DEFINE_double(update_angle, 0.08,
              "angle difference at which to force an update of the local map");

bool run_ = true;

cv::Mat last_image_;
cv::Mat bev_image_;

Vector2f last_image_loc_ = {0, 0};
float last_image_angle_ = 0;
Vector2f current_loc_ = {0, 0};
float current_angle_ = 0;

cv::Mat camera_K;
cv::Mat camera_D;
cv::Mat camera_H;

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  cv::destroyAllWindows();
  run_ = false;
}

void UpdateFrame(Eigen::Vector2f loc, float angle) {
  auto current_transform =
      Eigen::Translation2f(current_loc_) * Eigen::Rotation2Df(current_angle_);
  auto new_transform = Eigen::Translation2f(loc) * Eigen::Rotation2Df(angle);
  Eigen::Affine2f delta_transform = current_transform.inverse() * new_transform;

  cv::Mat flipped_image;
  cv::flip(bev_image_, flipped_image, 0);

  cv::Mat translation_mat;
  Eigen::Matrix2f eigen_rot = Eigen::Rotation2Df(-M_PI_2) *
                              delta_transform.rotation().matrix().inverse() *
                              Eigen::Rotation2Df(M_PI_2);
  Eigen::Vector2f eigen_trans = -eigen_rot * Eigen::Rotation2Df(-M_PI_2) *
                                delta_transform.translation().matrix();
  eigen_trans = eigen_trans.cwiseProduct(Eigen::Vector2f{100, 100});
  cv::eigen2cv(eigen_trans, translation_mat);

  Eigen::Rotation2Df rot;
  rot.fromRotationMatrix(eigen_rot);

  auto transform_matrix = cv::getRotationMatrix2D(
      cv::Point2f{(float)CONFIG_image_width / 2, (float)CONFIG_image_height},
      -rot.angle() * (180 / M_PI), 1.0);

  transform_matrix(cv::Rect(2, 0, 1, 2)) -= translation_mat;

  auto prev_map = flipped_image.clone();
  flipped_image.setTo(0);
  cv::warpAffine(prev_map, flipped_image, transform_matrix,
                 flipped_image.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                 0);
  bev_image_.setTo(0);
  cv::flip(flipped_image, bev_image_, 0);
}

void ConvertToBEV(const cv::Mat& original, cv::Mat& output) {
  static cv::Mat M;
  static cv::Mat translation_matrix;

  if (M.size[0] < 3) {
    vector<cv::Point2f> input_points;
    vector<cv::Point2f> output_points;
    for (int i = 0; i < camera_H.rows; i++) {
      auto input_point =
          cv::Point2f(camera_H.at<double>(i, 2), camera_H.at<double>(i, 3));
      auto output_point =
          cv::Point2f(camera_H.at<double>(i, 0), camera_H.at<double>(i, 1));
      input_points.push_back(input_point);
      output_points.push_back(output_point);
    }

    auto offset =
        cv::Point2f{(float)last_image_.size[1] / 2, (float)last_image_.size[0]};
    for (auto& p : output_points) {
      p *= CONFIG_pixels_per_meter;
      p += offset;
    }

    M = cv::getPerspectiveTransform(input_points, output_points);

    float vals[] = {1.0,
                    0.0,
                    (float)(CONFIG_pixels_per_meter * CONFIG_bev_size / 2 -
                            last_image_.size[1] / 2),
                    0.0,
                    1.0,
                    (float)(CONFIG_pixels_per_meter * CONFIG_bev_size / 2 -
                            last_image_.size[0])};
    translation_matrix =
        cv::Mat(2, 3, CV_32F, vals)
            .clone();  // clone is necessary so we don't have a pointer to out
                       // of scope stack array "vals"
  }

  // maybe make this static later for better performance
  cv::Mat cp = original.clone();
  cv::warpPerspective(original, cp, M, cp.size());
  cv::warpAffine(cp, output, translation_matrix, output.size());
}

void OdometryCallback(const nav_msgs::Odometry msg) {
  Vector2f new_loc = {msg.pose.pose.position.x, msg.pose.pose.position.y};
  float new_angle = 2 * acos(msg.pose.pose.orientation.w);

  static Vector2f last_update_loc = new_loc;
  static float last_update_angle = new_angle;

  if (FLAGS_v > 0) {
    printf("Localization t=%f\n", GetWallTime());
  }

  if (last_image_.size[0] > 10) {
    UpdateFrame(new_loc, new_angle);

    if ((new_loc - last_update_loc).norm() >= FLAGS_update_dist ||
        abs(new_angle - last_update_angle) >= FLAGS_update_angle) {
      last_update_loc = new_loc;
      last_update_angle = new_angle;

      cv::Mat image = cv::Mat::zeros(bev_image_.size(), last_image_.type());
      ConvertToBEV(last_image_, image);

      // combine latest image with local map
      for (int y = 0; y < bev_image_.rows; y++) {
        for (int x = 0; x < bev_image_.cols; x++) {
          auto img_pixel = image.at<cv::Vec4b>(y, x);
          if (img_pixel[3] > 0) {
            bev_image_.at<cv::Vec4b>(y, x) =
                bev_image_.at<cv::Vec4b>(y, x) * FLAGS_gamma +
                img_pixel * (1 - FLAGS_gamma);
          }
        }
      }
    }

    if (FLAGS_visualize) {
      cv::Mat vis_map = bev_image_.clone();
      cv::circle(vis_map,
                 {CONFIG_bev_size * CONFIG_pixels_per_meter / 2,
                  CONFIG_bev_size * CONFIG_pixels_per_meter / 2},
                 5, 0x0000FF);
      cv::imshow("lmap", vis_map);
      cv::waitKey(1);
    }
  }

  current_loc_ = new_loc;
  current_angle_ = new_angle;
}

void ImageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
  static cv::Mat M1, M2;
  if (M1.size[0] < 1) {
    auto R = cv::Mat::eye(3, 3, CV_32F);
    cv::initUndistortRectifyMap(
        camera_K, camera_D, R, camera_K,
        cv::Size(CONFIG_image_width, CONFIG_image_height), CV_32F, M1, M2);
  }

  try {
    cv_bridge::CvImagePtr image =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    last_image_ = image->image;
    cv::cvtColor(last_image_, last_image_, cv::COLOR_BGR2BGRA);
    if (FLAGS_camera_correction)
      cv::remap(last_image_, last_image_, M1, M2, cv::INTER_LINEAR);
    last_image_loc_ = current_loc_;
    last_image_angle_ = current_angle_;
    if (FLAGS_v > 0) {
      printf("Got image\n");
    }
  } catch (cv_bridge::Exception& e) {
    fprintf(stderr, "cv_bridge exception: %s\n", e.what());
    return;
  }
}

// Mostly grabbed from graph_navigation
int LoadCameraCalibrationCV() {
  cv::FileStorage camera_settings(CONFIG_camera_calibration_path,
                                  cv::FileStorage::READ);

  if (!camera_settings.isOpened()) {
    std::cerr << "Failed to open camera settings file at: "
              << CONFIG_camera_calibration_path << std::endl;
    return -1;
  }

  cv::FileNode node = camera_settings["K"];
  if (!node.empty()) {
    camera_K = node.mat();
  } else {
    std::cerr << "Camera calibration matrix not read! Check configuration "
                 "file is in default yaml format.";
  }

  node = camera_settings["D"];
  if (!node.empty()) {
    camera_D = node.mat();
  } else {
    std::cerr << "Camera distortion coefficients not read! Check "
                 "configuration file is in default yaml format.";
  }

  node = camera_settings["H"];
  if (!node.empty()) {
    camera_H = node.mat();
  } else {
    std::cerr << "Camera homography matrix not read! Check configuration file "
                 "is in default yaml format.";
  }

  return 0;
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);

  config_reader::ConfigReader reader({FLAGS_config});
  bev_image_ =
      cv::Mat::zeros(CONFIG_pixels_per_meter * CONFIG_bev_size,
                     CONFIG_pixels_per_meter * CONFIG_bev_size, CV_8UC4);
  LoadCameraCalibrationCV();

  // Initialize ROS.
  ros::init(argc, argv, "local_rgb_map", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;
  image_transport::ImageTransport it(n);

  auto bev_pub = it.advertise("bev_image", 1);
  auto bev_img = cv_bridge::CvImage(std_msgs::Header(), "bgr8", bev_image_);

  ros::Subscriber odometry_sub =
      n.subscribe(CONFIG_odometry_topic, 1, &OdometryCallback);
  ros::Subscriber image_sub =
      n.subscribe(CONFIG_image_topic, 10, &ImageCallback);

  RateLoop loop(30.0);
  while (run_ && ros::ok()) {
    bev_img.image = bev_image_;
    bev_pub.publish(bev_img.toImageMsg());

    ros::spinOnce();
    loop.Sleep();
  }

  return 0;
}
