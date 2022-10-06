#include "bev/bev.h"

#include <glog/logging.h>

#include <eigen3/Eigen/Dense>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

namespace bev {

BirdsEyeView::BirdsEyeView(const Eigen::Matrix3f& intrinsic_matrix,
                           const Eigen::Affine3f& T_ground_camera,
                           const int input_image_rows,
                           const int input_image_cols,
                           const float bev_pixels_per_meter,
                           const float bev_horizon_distance)
    : intrinsic_matrix_(intrinsic_matrix),
      input_image_rows_(input_image_rows),
      input_image_cols_(input_image_cols) {
  // The pixel frame's X axis points to the right, the Y axis points down, and
  // the Z axis points out of the lens. Prerotate the T_ground_camera transform
  // using right-to-left extrinsic rotations.
  T_ground_pixel_ = T_ground_camera *
                    Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitZ()) *
                    Eigen::AngleAxisf(-M_PI_2, Eigen::Vector3f::UnitX());

  // Here we calculate the perspective matrix by finding the approximate
  // coordinates corresponding to the visible limits of the ground plane up to
  // the horizon distance. We assume the bottom two corners of the input image
  // contain the ground plane, i.e. the camera is mounted with negligible
  // roll.

  // Finds the pixel coordinates of a point at the horizon directly in front of
  // the camera.
  const Eigen::Vector2f P_pixel_horizon =
      GroundToPixelCoordinates(Eigen::Vector2f(bev_horizon_distance, 0));

  // The pixel coordinates of the four corners of the visible ground plane in
  // the original image as [x y] column vectors.
  Eigen::Matrix2Xf P_pixel_planeCorners(2, 4);
  // bottom-left corner
  P_pixel_planeCorners.col(0) << 0, input_image_rows_ - 1;
  // bottom-right corner
  P_pixel_planeCorners.col(1) << input_image_cols_ - 1, input_image_rows_ - 1;
  // top-left corner
  P_pixel_planeCorners.col(2) << 0, P_pixel_horizon.y();
  // top-right corner
  P_pixel_planeCorners.col(3) << input_image_cols_ - 1, P_pixel_horizon.y();

  const Eigen::Matrix2Xf P_ground_planeCorners =
      PixelToGroundCoordinates(P_pixel_planeCorners);

  // Ground coordinates corresponding to the limits of the output
  // birds-eye-view image.
  const float P_ground_planeMaxX = P_ground_planeCorners.row(0).maxCoeff();
  // Assume the camera is forward-facing.
  const float P_ground_planeMinX = 0;
  const float P_ground_planeMaxY = P_ground_planeCorners.row(1).maxCoeff();
  const float P_ground_planeMinY = P_ground_planeCorners.row(1).minCoeff();

  // Note that the image and world xy axes are swapped
  bev_image_rows_ =
      (P_ground_planeMaxX - P_ground_planeMinX) * bev_pixels_per_meter;
  bev_image_cols_ =
      (P_ground_planeMaxY - P_ground_planeMinY) * bev_pixels_per_meter;

  Eigen::Matrix2Xf P_bevPixel_planeCorners(2, 4);
  for (Eigen::Index c = 0; c < 4; ++c) {
    // Note again that the image and world xy axes are swapped and negated.
    // (x: 0, y: 0) in the BEV image corresponds to (maxY, maxX) in the world
    // coordinates.
    auto bevPixel = P_bevPixel_planeCorners.col(c);
    bevPixel.x() = P_ground_planeMaxY - P_ground_planeCorners.col(c).y();
    bevPixel.y() = P_ground_planeMaxX - P_ground_planeCorners.col(c).x();
    bevPixel *= bev_pixels_per_meter;
  }

  cv::Mat cv_P_pixel_planeCorners, cv_P_bevPixel_planeCorners;
  cv::eigen2cv(P_pixel_planeCorners, cv_P_pixel_planeCorners);
  cv::eigen2cv(P_bevPixel_planeCorners, cv_P_bevPixel_planeCorners);
  // getPerspectiveTransform expects row vectors
  perspective_matrix_ = cv::getPerspectiveTransform(
      cv_P_pixel_planeCorners.t(), cv_P_bevPixel_planeCorners.t());
}

cv::Mat3b BirdsEyeView::WarpPerspective(const cv::Mat3b& input_image) const {
  LOG_IF(ERROR, input_image.rows != input_image_rows_ ||
                    input_image.cols != input_image_cols_)
      << "Input image dimensions (" << input_image.size()
      << ") does not match configuration (" << input_image_rows_ << " x "
      << input_image_cols_ << ")";

  cv::Mat3b bev_image = cv::Mat3b::zeros(bev_image_rows_, bev_image_cols_);
  cv::warpPerspective(input_image, bev_image, perspective_matrix_,
                      bev_image.size());
  return bev_image;
}

Eigen::Matrix2Xf BirdsEyeView::GroundToPixelCoordinates(
    const Eigen::Matrix2Xf& ground_coords) const {
  // Adds back the implicit z row
  Eigen::Matrix3Xf ground_coords_with_z =
      Eigen::Matrix3Xf::Zero(3, ground_coords.cols());
  ground_coords_with_z.topRows(2) = ground_coords;

  // From right to left, transforms the ground frame coordinates into the pixel
  // frame and turns the 3D coordinates into 2D homogeneous coordinates.
  const Eigen::Affine3f T_pixel_ground = T_ground_pixel_.inverse();
  const Eigen::Matrix3Xf pixel_coords =
      intrinsic_matrix_ * T_pixel_ground * ground_coords_with_z;

  // Normalizes the homogeneous coordinates and drops the homogeneous row.
  return pixel_coords.colwise().hnormalized();
}

Eigen::Matrix2Xf BirdsEyeView::PixelToGroundCoordinates(
    const Eigen::Matrix2Xf& pixel_coordinates) const {
  // Calculate the rays originating from the camera corresponding to each pixel,
  // expressed in the world frame. These rays are column vectors with arbitrary
  // magnitude.
  //
  // From right to left, maps 2D pixel coordinates back into 3D coordinates with
  // arbitrary magnitude and rotates these coordinates into the world frame.
  Eigen::Matrix3Xf pixel_rays = T_ground_pixel_.rotation() *
                                intrinsic_matrix_.inverse() *
                                pixel_coordinates.colwise().homogeneous();

  // Scales each ray vector to intersect with the ground plane (z=0)
  for (Eigen::Index c = 0; c < pixel_rays.cols(); ++c) {
    auto ray = pixel_rays.col(c);

    // A camera ray pointing above the ground plane will not intersect the
    // ground plane.
    LOG_IF(ERROR, ray.z() >= 0) << "Camera ray points above the ground plane";

    // Assumes the ray points downwards (z < 0)
    const float scale_factor = T_ground_pixel_.translation().z() / -ray.z();
    ray = T_ground_pixel_.translation() + scale_factor * ray;
  }

  // Drops the z=0 row
  return pixel_rays.topRows(2);
}

}  // namespace bev
