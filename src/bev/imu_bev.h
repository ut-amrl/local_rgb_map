#pragma once

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace bev {

class ImuBirdsEyeView {
 public:
  ImuBirdsEyeView(const Eigen::Matrix3f& intrinsic_matrix,
                  const Eigen::Affine3f& T_imu_camera,
                  const Eigen::Affine3f& T_ground_imu_initial,
                  const int input_image_rows, const int input_image_cols,
                  const float bev_pixels_per_meter,
                  const float bev_horizon_distance);

  cv::Mat3b WarpPerspective(const cv::Mat3b& input_image) const;

  void UpdateOrientation(const Eigen::Quaternionf& orientation);

  cv::Size GetBevSize() const;

 protected:
  Eigen::Affine3f GroundToCameraTransform(const Eigen::Quaternionf& orientation) const;
  Eigen::Matrix2Xf GroundToPixelCoordinates(
      const Eigen::Matrix2Xf& ground_coordinates) const;
  Eigen::Matrix2Xf PixelToGroundCoordinates(
      const Eigen::Matrix2Xf& pixel_coordinates) const;
  void GeneratePerspectiveMatrix(const Eigen::Quaternionf& orientation);

 protected:
  Eigen::Matrix3f intrinsic_matrix_;
  Eigen::Affine3f T_imu_camera_;
  Eigen::Affine3f T_ground_imu_;
  Eigen::Affine3f T_ground_pixel_;
  int input_image_rows_;
  int input_image_cols_;
  int bev_image_rows_;
  int bev_image_cols_;
  float bev_pixels_per_meter_;
  float bev_horizon_distance_;
  cv::Mat perspective_matrix_;
};

Eigen::Quaternionf IntrinsicRotation(float x, float y, float z);
Eigen::Quaternionf ExtrinsicRotation(float x, float y, float z);
}  // namespace bev
