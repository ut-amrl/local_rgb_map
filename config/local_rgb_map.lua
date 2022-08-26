BEVParameters = {
    localization_topic = "/localization";
    image_topic = "/camera/rgb/image_raw/compressed";
    pixels_per_meter = 100;
    image_size = 8; -- size of square bev image in meters, with robot in the center
    camera_calibration_config_path = "config/camera_calibration_kinect.yaml";
}
