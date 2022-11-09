function deg2rad(deg)
  return deg * (math.pi / 180)
end


-- https://stackoverflow.com/a/23535333 Returns the absolute path with a
-- trailing slash to the directory containing this script.
function config_dir()
  local script_path = debug.getinfo(2, "S").source:sub(2)
  return script_path:match("(.*/)")
end


BEVParameters = {
    camera_calibration_config_path = config_dir() .. "camera_calibration_kinect_spot.yaml";

    input_image_topic = "/camera/rgb/image_raw";
    input_image_width = 2048;
    input_image_height = 1536;

    bev_image_topic = "/bev/single"; -- image_transport takes the base topic name
    bev_pixels_per_meter = 150.0;
    bev_horizon_distance = 5.0;

    bev_empty_region_rgb = {0, 0, 0};

    stitched_bev_image_topic = "/bev/stitched";
    stitched_bev_angle_topic = "/bev/stitched_angle";
    stitched_bev_horizon_distance = 6.0;
    stitched_bev_ema_gamma = 0.1;
    stitched_bev_update_distance = 1.0; -- meters
    stitched_bev_update_angle = 10; -- degrees


    -- stitching is not required for the spot at this time
    -- pose_topic = "/husky/inekf_estimation/pose";
    pose_topic = "/foo_9498125";

    cv_num_threads = 2;

    T_ground_camera = {
        x = 0.34;
        y = 0.0;
        z = 0.233 + 0.5; -- spot to camera + spot to ground
        pitch = deg2rad(17.0);
    };
}
