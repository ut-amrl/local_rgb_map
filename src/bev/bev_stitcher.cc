#include "bev/bev_stitcher.h"

#include <glog/logging.h>

#include <cmath>
#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

namespace bev {

BevStitcher::BevStitcher(const int input_image_rows, const int input_image_cols,
                         const float pixels_per_meter,
                         const float horizon_distance, const float ema_gamma,
                         const float stitch_overlay_threshold)
    : input_image_rows_(input_image_rows),
      input_image_cols_(input_image_cols),
      pixels_per_meter_(pixels_per_meter),
      horizon_distance_(horizon_distance),
      ema_gamma_(ema_gamma),
      stitch_overlay_threshold_(stitch_overlay_threshold) {
  // the stitched BEV map places the robot in the center of the image so the
  // image spans horizon_distance_ in the x and y axes from the robot
  stitched_bev_ =
      cv::Mat::zeros(2 * pixels_per_meter_ * horizon_distance_,
                     2 * pixels_per_meter_ * horizon_distance_, CV_32F);
}

void BevStitcher::UpdateBev(const cv::Mat3b& image) {
  LOG_IF(ERROR,
         image.rows != input_image_rows_ || image.cols != input_image_cols_)
      << "Input image dimensions (" << image.size()
      << ") does not match configuration (" << input_image_rows_ << " x "
      << input_image_cols_ << ")";

  last_bev_ = image.clone();
  StitchBev();
}

cv::Mat3b BevStitcher::UpdatePose(const Eigen::Vector3f& position,
                                  const Eigen::Quaternionf& orientation) {
  UpdateStitchedPose(position, orientation);

  last_position_ = position;
  last_orientation_ = orientation;

  return stitched_bev_;
}

void BevStitcher::StitchBev() {
  cv::Mat3b bev_resized = CreateResizedBev();

  for (int y = 0; y < bev_resized.rows; y++) {
    for (int x = 0; x < bev_resized.cols; x++) {
      auto bev_pixel = bev_resized.at<cv::Vec3b>(y, x);
      if (cv::norm(bev_pixel) > stitch_overlay_threshold_)
        stitched_bev_.at<cv::Vec3b>(y, x) =
            ema_gamma_ * stitched_bev_.at<cv::Vec3b>(y, x) +
            (1 - ema_gamma_) * bev_pixel;
    }
  }
}

cv::Mat3b BevStitcher::CreateResizedBev() const {
  float vals[] = {
      1.0, 0.0, (float)(stitched_bev_.size[1] / 2.0 - last_bev_.size[1] / 2.0),
      0.0, 1.0, (float)(stitched_bev_.size[0] / 2.0 - last_bev_.size[0])};
  cv::Mat translation_matrix =
      cv::Mat(2, 3, CV_32F, vals)
          .clone();  // clone is necessary so we don't have a pointer to out
                     // of scope stack array "vals"
  cv::Mat3b bev_resized =
      cv::Mat::zeros(stitched_bev_.size(), last_bev_.type());
  cv::warpAffine(last_bev_, bev_resized, translation_matrix,
                 bev_resized.size());

  return bev_resized;
}

void BevStitcher::UpdateStitchedPose(const Eigen::Vector3f& position,
                                     const Eigen::Quaternionf& orientation) {
  // TODO: use 3d transforms instead of 2d ones
  const float last_angle =
      std::atan2(2.0 * (last_orientation_.w() * last_orientation_.z() +
                        last_orientation_.x() * last_orientation_.y()),
                 1.0 - 2.0 * (last_orientation_.y() * last_orientation_.y() +
                              last_orientation_.z() * last_orientation_.z()));
  Eigen::Affine2f T_current = Eigen::Translation2f(last_position_.topRows(2)) *
                              Eigen::Rotation2Df(last_angle);
  const float new_angle =
      std::atan2(2.0 * (orientation.w() * orientation.z() +
                        orientation.x() * orientation.y()),
                 1.0 - 2.0 * (orientation.y() * orientation.y() +
                              orientation.z() * orientation.z()));
  Eigen::Affine2f T_new =
      Eigen::Translation2f(position.topRows(2)) * Eigen::Rotation2Df(new_angle);
  Eigen::Affine2f T_delta = T_current.inverse() * T_new;

  // flip across the x axis
  cv::Mat flipped_image;
  cv::flip(stitched_bev_, flipped_image, 0);

  cv::Mat cv_translation;
  Eigen::Matrix2f rotation_mat = Eigen::Rotation2Df(-M_PI_2) *
                                 T_delta.rotation().matrix().inverse() *
                                 Eigen::Rotation2Df(M_PI_2);
  Eigen::Vector2f translation = -rotation_mat * Eigen::Rotation2Df(-M_PI_2) *
                                T_delta.translation().matrix();
  translation *= pixels_per_meter_;
  cv::eigen2cv(translation, cv_translation);

  Eigen::Rotation2Df rotation(rotation_mat);

  cv::Mat cv_transform = cv::getRotationMatrix2D(
      cv::Point2f{(float)stitched_bev_.size[1] / 2, (float)stitched_bev_.size[0] / 2},
      -rotation.angle() * (180 / M_PI), 1.0);
  cv_transform(cv::Rect(2, 0, 1, 2)) -= cv_translation;

  cv::Mat previous_image = flipped_image.clone();
  flipped_image.setTo(0);
  cv::warpAffine(previous_image, flipped_image, cv_transform,
                 flipped_image.size(), cv::INTER_LINEAR, cv::BORDER_CONSTANT,
                 0);
  stitched_bev_.setTo(0);
  cv::flip(flipped_image, stitched_bev_, 0);
}

}  // namespace bev
