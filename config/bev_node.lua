function deg2rad(deg)
  return deg * (math.pi / 180)
end

BEVParameters = {
    camera_calibration_config_path = "config/camera_calibration_kinect.yaml";
    input_image_topic = "/camera/rgb/image_raw/compressed";
    input_image_width = 1920;
    input_image_height = 1080;

    bev_image_topic = "/bev/single"; -- image_transport takes the base topic name
    bev_pixels_per_meter = 150.0;
    bev_horizon_distance = 5.0;

    stitched_bev_image_topic = "/bev/stitched";
    stitched_bev_horizon_distance = 6.0;
    stitched_bev_ema_gamma = 0.5;

    pose_topic = "/husky_inekf_estimation/pose";

    cv_num_threads = 2;

    T_ground_camera = {
        x = 0.15;
        y = 0.0;
        z = 0.31 + 0.2;
        pitch = deg2rad(17.0);
    };
}
