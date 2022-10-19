#include <config_reader/config_reader.h>
#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <image_transport/image_transport.h>
#include <ros/ros.h>
#include <sensor_msgs/CompressedImage.h>
#include <std_msgs/Float32.h>

#include <iostream>
#include <memory>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include "bev/bev.h"
#include "bev/bev_stitcher.h"

namespace {
CONFIG_STRING(camera_calibration_config_path,
              "BEVParameters.camera_calibration_config_path");
CONFIG_STRING(input_image_topic, "BEVParameters.input_image_topic");
CONFIG_UINT(input_image_width, "BEVParameters.input_image_width");
CONFIG_UINT(input_image_height, "BEVParameters.input_image_height");

CONFIG_STRING(bev_image_topic, "BEVParameters.bev_image_topic");
CONFIG_FLOAT(bev_pixels_per_meter, "BEVParameters.bev_pixels_per_meter");
CONFIG_FLOAT(bev_horizon_distance, "BEVParameters.bev_horizon_distance");

CONFIG_STRING(stitched_bev_image_topic,
              "BEVParameters.stitched_bev_image_topic");
CONFIG_STRING(stitched_bev_angle_topic,
              "BEVParameters.stitched_bev_angle_topic");
CONFIG_FLOAT(stitched_bev_horizon_distance,
             "BEVParameters.stitched_bev_horizon_distance");
CONFIG_FLOAT(stitched_bev_ema_gamma, "BEVParameters.stitched_bev_ema_gamma");
CONFIG_FLOAT(stitched_bev_update_distance,
             "BEVParameters.stitched_bev_update_distance");
CONFIG_FLOAT(stitched_bev_update_angle,
             "BEVParameters.stitched_bev_update_angle");

CONFIG_STRING(pose_topic, "BEVParameters.pose_topic");

CONFIG_FLOAT(T_ground_camera_x, "BEVParameters.T_ground_camera.x");
CONFIG_FLOAT(T_ground_camera_y, "BEVParameters.T_ground_camera.y");
CONFIG_FLOAT(T_ground_camera_z, "BEVParameters.T_ground_camera.z");
CONFIG_FLOAT(T_ground_camera_pitch, "BEVParameters.T_ground_camera.pitch");

CONFIG_UINT(cv_num_threads, "BEVParameters.cv_num_threads");

DEFINE_string(config, "config/bev_node.lua", "path to config file");

image_transport::Publisher bev_image_publisher_;
image_transport::Publisher stitched_bev_image_publisher_;
ros::Publisher stitched_bev_angle_publisher_;
std::unique_ptr<bev::BirdsEyeView> bev_transformer_;
std::unique_ptr<bev::BevStitcher> bev_stitcher_;
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

void InputImageCallback(const sensor_msgs::ImageConstPtr& msg) {
  cv_bridge::CvImagePtr cv_image =
      cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

  cv::Mat3b bev_image = bev_transformer_->WarpPerspective(cv_image->image);

  bev_stitcher_->UpdateBev(bev_image);

  // Reuse the information from the input message header.
  sensor_msgs::ImagePtr bev_image_msg =
      cv_bridge::CvImage(msg->header, "bgr8", bev_image).toImageMsg();
  bev_image_publisher_.publish(bev_image_msg);
}

void PoseCallback(const geometry_msgs::PoseWithCovarianceStampedConstPtr msg) {
  const Eigen::Vector3f position{(float)msg->pose.pose.position.x,
                                 (float)msg->pose.pose.position.y,
                                 (float)msg->pose.pose.position.z};
  const Eigen::Quaternionf orientation{
      (float)msg->pose.pose.orientation.w, (float)msg->pose.pose.orientation.x,
      (float)msg->pose.pose.orientation.y, (float)msg->pose.pose.orientation.z};

  cv::Mat3b stitched_bev_image =
      bev_stitcher_->UpdatePose(position, orientation);
  sensor_msgs::ImagePtr stitched_bev_image_msg =
      cv_bridge::CvImage(msg->header, "bgr8", stitched_bev_image).toImageMsg();

  std_msgs::Float32 stitched_bev_angle_msg;
  stitched_bev_angle_msg.data = bev_stitcher_->GetRobotMapAngle();
  stitched_bev_image_publisher_.publish(stitched_bev_image_msg);
  stitched_bev_angle_publisher_.publish(stitched_bev_angle_msg);
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

  image_transport::Subscriber input_image_subscriber =
      image_transport.subscribe(CONFIG_input_image_topic, 1,
                                &InputImageCallback,
                                image_transport::TransportHints("compressed"));
  ros::Subscriber pose_subscriber =
      node_handle.subscribe(CONFIG_pose_topic, 1, &PoseCallback);

  bev_image_publisher_ = image_transport.advertise(CONFIG_bev_image_topic, 1);
  stitched_bev_image_publisher_ =
      image_transport.advertise(CONFIG_stitched_bev_image_topic, 1);
  stitched_bev_angle_publisher_ = node_handle.advertise<std_msgs::Float32>(
      CONFIG_stitched_bev_angle_topic, 1);

  bev_transformer_ = std::make_unique<bev::BirdsEyeView>(
      ReadIntrinsicMatrix(), Read_T_ground_camera(), CONFIG_input_image_height,
      CONFIG_input_image_width, CONFIG_bev_pixels_per_meter,
      CONFIG_bev_horizon_distance);

  auto bev_size = bev_transformer_->GetBevSize();
  bev_stitcher_ = std::make_unique<bev::BevStitcher>(
      bev_size.height, bev_size.width, CONFIG_bev_pixels_per_meter,
      CONFIG_stitched_bev_horizon_distance, CONFIG_stitched_bev_ema_gamma,
      CONFIG_stitched_bev_update_distance, CONFIG_stitched_bev_update_angle);

  ros::spin();

  return 0;
}
