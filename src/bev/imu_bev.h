#pragma once

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace bev {

class ImuBirdsEyeView {
 public:
  ImuBirdsEyeView(const Eigen::Matrix3f& intrinsic_matrix,
                  const Eigen::Affine3f& T_imu_camera,
                  const int input_image_rows, const int input_image_cols,
                  const float bev_pixels_per_meter,
                  const float bev_horizon_distance);

  cv::Mat3b WarpPerspective(const cv::Mat3b& input_image) const;

  void UpdateOrientation(const Eigen::Quaternionf& orientation);

  cv::Size GetBevSize() const;

 protected:
  Eigen::Matrix2Xf GroundToPixelCoordinates(
      const Eigen::Matrix2Xf& ground_coordinates) const;

  Eigen::Matrix2Xf PixelToGroundCoordinates(
      const Eigen::Matrix2Xf& pixel_coordinates) const;

  cv::Mat CreatePerspectiveMatrix(const Eigen::Quaternionf& orientation) const;

 protected:
  Eigen::Matrix3f intrinsic_matrix_;
  Eigen::Affine3f T_ground_pixel_;
  int input_image_rows_;
  int input_image_cols_;
  int bev_image_rows_;
  int bev_image_cols_;
  cv::Mat perspective_matrix_;
};
}  // namespace bev
