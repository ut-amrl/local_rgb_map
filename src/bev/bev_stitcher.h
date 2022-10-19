#pragma once

#include <eigen3/Eigen/Dense>
#include <opencv2/core/mat.hpp>

namespace bev {
class BevStitcher {
 public:
  BevStitcher(const int input_image_rows, const int input_image_cols,
              const float pixels_per_meter, const float horizon_distance,
              const float ema_gamma,
              const float stitch_overlay_threshold = 1.0);

  void UpdateBev(const cv::Mat3b& image);
  cv::Mat3b UpdatePose(const Eigen::Vector3f& position,
                       const Eigen::Quaternionf& orientation);
  float GetRobotMapAngle() const;

 protected:
  void StitchBev();
  cv::Mat3b CreateResizedBev() const;
  void UpdateStitchedPose(const Eigen::Vector3f& position,
                          const Eigen::Quaternionf& orientation);

 protected:
  cv::Mat3b stitched_bev_;
  cv::Mat3b last_bev_;

  Eigen::Vector3f last_position_;
  Eigen::Quaternionf last_orientation_;

  const int input_image_rows_;
  const int input_image_cols_;
  const float pixels_per_meter_;
  const float horizon_distance_;

  const float ema_gamma_;
  const float stitch_overlay_threshold_;
};
}  // namespace bev
