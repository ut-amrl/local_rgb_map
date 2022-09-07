BEVParameters = {
    localization_topic = "/localization";
    odometry_topic = "/jackal_velocity_controller/odom";
    image_topic = "/camera/rgb/image_raw/compressed";
    image_width = 1280; -- width of image coming from camera
    image_height = 720; -- height of image coming from camera
    pixels_per_meter = 100;
    bev_size = 8; -- size of square bev image in meters, with robot in the center
    camera_calibration_config_path = "config/camera_calibration_kinect.yaml";
}
