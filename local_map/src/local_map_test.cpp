#include <iostream>             // for cout
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // include OpenCV header file

int main(int argc, char* argv[])
try
{
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline pipe;
    rs2::config   cfg;

    // std::string serial;
    // cfg.enable_device(serial);
    cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 60);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);

    // Configure and start the pipeline
    rs2::pipeline_profile profile = pipe.start(cfg);

    auto device = profile.get_device();
    std::cout << device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) <<std::endl;

    // Define two align objects. One will be used to align
    // to depth viewport and the other to color.
    // Creating align object is an expensive operation
    // that should not be performed in the main loop
    rs2::align align_to_depth(RS2_STREAM_DEPTH);
    rs2::align align_to_color(RS2_STREAM_COLOR);

    // Declare filters
    rs2::colorizer           color_map;                // Declare depth colorizer for pretty visualization of depth data
    rs2::rates_printer       printer;                  // Declare rates printer for showing streaming rates of the enabled streams.
    rs2::decimation_filter   dec_filter;               // Decimation - reduces depth frame density
    rs2::threshold_filter    thr_filter(0.15, 1.5);    // Threshold  - removes values outside recommended range
    rs2::spatial_filter      spat_filter;              // Spatial    - edge-preserving spatial smoothing
    rs2::temporal_filter     temp_filter;              // Temporal   - reduces temporal noise
    rs2::disparity_transform depth_to_disparity(true); // Declare disparity transform from depth to disparity and vice versa
    rs2::disparity_transform disparity_to_depth(false);

    rs2::pointcloud pc;     // Declare pointcloud object, for calculating pointclouds and texture mappings
    rs2::points     points; // We want the points object to be persistent so we can display the last cloud when a frame drops

    // cv::namedWindow("Color", cv::WINDOW_AUTOSIZE);
    // cv::namedWindow("Depth", cv::WINDOW_AUTOSIZE);

    // Camera warmup - dropping several first frames to let auto-exposure stabilize
    rs2::frameset frames;
    for (int i = 0; i < 30; i++)
    {
        // Wait for all configured streams to produce a frame
        frames = pipe.wait_for_frames();
    }

    while (true)
    {
        // Block program until frames arrive
        frames = pipe.wait_for_frames();
        frames = align_to_color.process(frames);

        // Try to get a frame of a depth image
        rs2::frame color_frame = frames.get_color_frame();
        rs2::frame ir_frame    = frames.first(RS2_STREAM_INFRARED);
        rs2::frame depth_frame = frames.get_depth_frame();

        // auto filtered = dec_filter.process(depth_frame);
        auto filtered_frame = thr_filter.process(depth_frame);
        filtered_frame      = spat_filter.process(filtered_frame);
        filtered_frame      = temp_filter.process(filtered_frame);

        rs2::video_frame colorized_depth = color_map.process(filtered_frame);

        // Creating OpenCV Matrix from a color image
        cv::Mat color(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        // cv::Mat depth(cv::Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth(cv::Size(640, 480), CV_8UC3, (void*)colorized_depth.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat ir(cv::Size(640, 480), CV_8UC1, (void*)ir_frame.get_data(), cv::Mat::AUTO_STEP);

        pc.map_to(color_frame);                             // Tell pointcloud object to map to this color frame
        points          = pc.calculate(filtered_frame);     // Generate the pointcloud and texture mappings
        auto vertices   = points.get_vertices();            // get vertices
        auto tex_coords = points.get_texture_coordinates(); // and texture coordinates
        float* points_ptr = (float*)(vertices);
        for (int i = 0; i < points.size(); i++)
        {
            if (vertices[i].z)
            {
                // TODO
            }
        }

        equalizeHist(ir, ir);
        applyColorMap(ir, ir, cv::COLORMAP_JET);

        // equalizeHist(depth, depth);
        // applyColorMap(depth, depth, cv::COLORMAP_JET);

        imshow("Depth", depth);
        imshow("Color", color);
        imshow("IR", ir);
        if (cv::waitKey(1) == 27)
            break;
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error& e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
