#include <config_reader/config_reader.h>
#include <cv_bridge/cv_bridge.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>

#include <iostream>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "bev/bev.h"

namespace {
CONFIG_STRING(camera_calibration_config_path,
              "SingleBEVParameters.camera_calibration_config_path");
CONFIG_STRING(input_image_topic, "SingleBEVParameters.input_image_topic");
CONFIG_UINT(input_image_width, "SingleBEVParameters.input_image_width");
CONFIG_UINT(input_image_height, "SingleBEVParameters.input_image_height");
CONFIG_STRING(bev_image_topic, "SingleBEVParameters.bev_image_topic");
CONFIG_FLOAT(bev_pixels_per_meter, "SingleBEVParameters.bev_pixels_per_meter");
CONFIG_FLOAT(bev_horizon_distance, "SingleBEVParameters.bev_horizon_distance");

CONFIG_FLOAT(T_ground_camera_x, "SingleBEVParameters.T_ground_camera.x");
CONFIG_FLOAT(T_ground_camera_y, "SingleBEVParameters.T_ground_camera.y");
CONFIG_FLOAT(T_ground_camera_z, "SingleBEVParameters.T_ground_camera.z");
CONFIG_FLOAT(T_ground_camera_pitch,
             "SingleBEVParameters.T_ground_camera.pitch");

CONFIG_UINT(cv_num_threads, "SingleBEVParameters.cv_num_threads");

DEFINE_string(config, "config/bev_node.lua", "path to config file");

image_transport::Publisher bev_image_publisher_;
std::unique_ptr<bev::BirdsEyeView> bev_transformer_;
}  // namespace

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

void InputImageCallback(const sensor_msgs::CompressedImageConstPtr msg) {
  cv_bridge::CvImagePtr cv_image =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  cv::Mat3b bev_image = bev_transformer_->WarpPerspective(cv_image->image);
  // Reuse the information from the input message header.
  sensor_msgs::ImagePtr bev_image_msg =
      cv_bridge::CvImage(msg->header, "bgr8", bev_image).toImageMsg();
  bev_image_publisher_.publish(bev_image_msg);
}

int main(int argc, char** argv) {
  google::ParseCommandLineFlags(&argc, &argv, false);
  google::InitGoogleLogging(argv[0]);
  FLAGS_logtostderr = true;
  FLAGS_colorlogtostderr = true;

  config_reader::ConfigReader config_reader_({FLAGS_config});

  cv::setNumThreads(CONFIG_cv_num_threads);

  ros::init(argc, argv, "bev_transform");
  ros::NodeHandle node_handle;
  image_transport::ImageTransport image_transport(node_handle);

  ros::Subscriber input_image_subscriber =
      node_handle.subscribe(CONFIG_input_image_topic, 1, &InputImageCallback);
  bev_image_publisher_ = image_transport.advertise(CONFIG_bev_image_topic, 1);

  bev_transformer_ = std::make_unique<bev::BirdsEyeView>(
      ReadIntrinsicMatrix(), Read_T_ground_camera(), CONFIG_input_image_height,
      CONFIG_input_image_width, CONFIG_bev_pixels_per_meter,
      CONFIG_bev_horizon_distance);

  ros::spin();

  return 0;
}
