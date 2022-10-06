#pragma once

#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>

namespace bev {

class BirdsEyeView {
 public:
  BirdsEyeView(const Eigen::Matrix3f& intrinsic_matrix,
               const Eigen::Affine3f& T_ground_camera,
               const int input_image_rows, const int input_image_cols,
               const float bev_pixels_per_meter,
               const float bev_horizon_distance);

  cv::Mat3b WarpPerspective(const cv::Mat3b& input_image) const;

  cv::Size GetBevSize() const;

 protected:
  Eigen::Matrix2Xf GroundToPixelCoordinates(
      const Eigen::Matrix2Xf& ground_coordinates) const;

  Eigen::Matrix2Xf PixelToGroundCoordinates(
      const Eigen::Matrix2Xf& pixel_coordinates) const;

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
