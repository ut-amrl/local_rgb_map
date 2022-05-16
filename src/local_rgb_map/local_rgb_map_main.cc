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

#include <vector>

#include "amrl_msgs/Localization2DMsg.h"
#include "cv_bridge/cv_bridge.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
#include "opencv2/calib3d.hpp"
#include "opencv2/core/eigen.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include "ros/ros.h"
#include "sensor_msgs/CompressedImage.h"
#include "shared/math/math_util.h"
#include "shared/ros/ros_helpers.h"
#include "shared/util/timer.h"
#include "std_msgs/String.h"

using amrl_msgs::Localization2DMsg;
using Eigen::Vector2f;
using ros::Time;
using std::vector;

// Create command line arguments
DEFINE_string(loc_topic, "localization", "Name of ROS topic for localization");
DEFINE_string(image_topic, "/camera/rgb/image_raw/compressed",
              "Name of ROS topic for compressed image data");
DEFINE_bool(visualize, false, "show opencv visualization of cost map");

bool run_ = true;
cv::Mat last_image_;
Vector2f last_image_loc_ = {0, 0};
float last_image_angle_ = 0;
Vector2f current_loc_ = {0, 0};
float current_angle_ = 0;
cv::Mat local_cost_map_ = cv::Mat::zeros(800, 800, CV_8UC4);

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  cv::destroyAllWindows();
  run_ = false;
}

void UpdateMapFrame(Eigen::Vector2f loc, float angle) {
  auto current_transform =
      Eigen::Translation2f(current_loc_) * Eigen::Rotation2Df(current_angle_);
  auto new_transform = Eigen::Translation2f(loc) * Eigen::Rotation2Df(angle);
  Eigen::Affine2f delta_transform = current_transform.inverse() * new_transform;

  cv::Mat flipped_cost_map;
  cv::flip(local_cost_map_, flipped_cost_map, 0);

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
      cv::Point2f{640, 780}, -rot.angle() * (180 / M_PI), 1.0);

  transform_matrix(cv::Rect(2, 0, 1, 2)) -= translation_mat;

  auto prev_cost_map = flipped_cost_map.clone();
  flipped_cost_map.setTo(0);
  cv::warpAffine(prev_cost_map, flipped_cost_map, transform_matrix,
                 flipped_cost_map.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                 0);
  local_cost_map_.setTo(0);
  cv::flip(flipped_cost_map, local_cost_map_, 0);
}

void LocalizationCallback(const amrl_msgs::Localization2DMsg msg) {
  // TODO: think about if there's a better place to put these
  static const cv::Point2f image_offset = {1280 / 2, 720};
  static const int pixels_per_meter = 100;
  static const vector<cv::Point2f> src_pts = {
      {358, 512}, {472, 383}, {743, 378}, {835, 503}};
  static const vector<cv::Point2f> dst_pts = {
      cv::Point2f{-0.5, -1.5} * pixels_per_meter + image_offset,
      cv::Point2f{-0.5, -2.5} * pixels_per_meter + image_offset,
      cv::Point2f{0.5, -2.5} * pixels_per_meter + image_offset,
      cv::Point2f{0.5, -1.5} * pixels_per_meter + image_offset};
  static const cv::Mat M = cv::getPerspectiveTransform(src_pts, dst_pts);
  static float vals[] = {1.0, 0.0, -240, 0.0, 1.0, -720 / 2};
  static const cv::Mat translation_matrix = cv::Mat(2, 3, CV_32F, vals);

  Vector2f new_loc = {msg.pose.x, msg.pose.y};
  float new_angle = msg.pose.theta;

  auto img_copy = last_image_.clone();
  cv::Mat image = cv::Mat::zeros(800, 800, last_image_.type());
  if (last_image_.size[0] > 10) {
    // transform existing image
    cv::warpPerspective(img_copy, img_copy, M, img_copy.size());
    cv::warpAffine(img_copy, image, translation_matrix, image.size());

    UpdateMapFrame(new_loc, new_angle);

    // combine latest image with local map
    for (int y = 0; y < local_cost_map_.rows; y++) {
      for (int x = 0; x < local_cost_map_.cols; x++) {
        auto img_pixel = image.at<cv::Vec4b>(y, x);
        if (img_pixel[3] > 0) {
          local_cost_map_.at<cv::Vec4b>(y, x) = img_pixel;
        }
      }
    }

    if (FLAGS_visualize) {
      cv::Mat vis_map = local_cost_map_.clone();
      cv::circle(vis_map, {400, 400}, 5, 0x0000FF);
      cv::imshow("lmap", vis_map);
      cv::waitKey(1);
    }
  }

  current_loc_ = new_loc;
  current_angle_ = new_angle;

  if (FLAGS_v > 0) {
    printf("Localization t=%f\n", GetWallTime());
  }
}

void ImageCallback(const sensor_msgs::CompressedImageConstPtr& msg) {
  try {
    cv_bridge::CvImagePtr image =
        cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    last_image_ = image->image;
    cv::cvtColor(last_image_, last_image_, cv::COLOR_BGR2BGRA);
    last_image_loc_ = current_loc_;
    last_image_angle_ = current_angle_;
    if (FLAGS_v > 0) {
      printf("Got image");
    }
  } catch (cv_bridge::Exception& e) {
    fprintf(stderr, "cv_bridge exception: %s\n", e.what());
    return;
  }
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  signal(SIGINT, SignalHandler);
  // Initialize ROS.
  ros::init(argc, argv, "local_rgb_map", ros::init_options::NoSigintHandler);
  ros::NodeHandle n;

  ros::Subscriber localization_sub =
      n.subscribe(FLAGS_loc_topic, 1, &LocalizationCallback);
  ros::Subscriber image_sub =
      n.subscribe(FLAGS_image_topic, 10, &ImageCallback);

  RateLoop loop(20.0);
  while (run_ && ros::ok()) {
    ros::spinOnce();
    loop.Sleep();
  }
  return 0;
}
