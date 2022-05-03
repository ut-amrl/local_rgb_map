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

#include <cv_bridge/cv_bridge.h>
#include <inttypes.h>
#include <math.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <vector>

#include "amrl_msgs/Localization2DMsg.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/Geometry"
#include "gflags/gflags.h"
#include "glog/logging.h"
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

bool run_ = true;
cv::Mat last_image_;
Vector2f last_image_loc_ = {0, 0};
float last_image_angle_ = 0;
Vector2f current_loc_ = {0, 0};
float current_angle_ = 0;

void SignalHandler(int) {
  if (!run_) {
    printf("Force Exit.\n");
    exit(0);
  }
  printf("Exiting.\n");
  cv::destroyAllWindows();
  run_ = false;
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

  Vector2f new_loc = {msg.pose.x, msg.pose.y};
  float new_angle = msg.pose.theta;

  auto image = last_image_.clone();
  if (image.size[0] > 10) {
    // transform existing image
    cv::warpPerspective(image, image, M, image.size());
    // cv::imshow("image", image);
    // cv::waitKey(33);

    // combine latest image with local map
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
  ros::init(argc, argv, "navigation", ros::init_options::NoSigintHandler);
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
