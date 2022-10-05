function deg2rad(deg)
  return deg * (math.pi / 180)
end

SingleBEVParameters = {
    camera_calibration_config_path = "config/camera_calibration_kinect_spot.yaml";
    input_image_topic = "/camera/rgb/image_raw/compressed";
    input_image_width = 2048;
    input_image_height = 1536;
    bev_image_topic = "/bev/single"; -- image_transport takes the base topic name
    bev_pixels_per_meter = 150.0;
    bev_horizon_distance = 5.0;

    cv_num_threads = 2;

    T_ground_camera = {
        x = 0.34;
        y = 0.0;
        z = 0.233 + 0.5; -- spot to camera + spot to ground
        pitch = deg2rad(15.0);
    };
}
