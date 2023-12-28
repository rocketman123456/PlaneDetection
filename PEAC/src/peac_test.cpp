#include <iostream>             // for cout
#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include <opencv2/opencv.hpp>   // include OpenCV header file

#include "plane_detection.h"

PlaneDetection plane_detection;

//-----------------------------------------------------------------
// MRF energy functions
MRF::CostVal dCost(int pix, int label) { return plane_detection.dCost(pix, label); }

MRF::CostVal fnCost(int pix1, int pix2, int i, int j) { return plane_detection.fnCost(pix1, pix2, i, j); }

void runMRFOptimization()
{
    DataCost*       data   = new DataCost(dCost);
    SmoothnessCost* smooth = new SmoothnessCost(fnCost);
    EnergyFunction* energy = new EnergyFunction(data, smooth);
    int             width = kDepthWidth, height = kDepthHeight;
    MRF*            mrf = new Expansion(width * height, plane_detection.plane_num_ + 1, energy);
    // Set neighbors for the graph
    for (int row = 0; row < height; row++)
    {
        for (int col = 0; col < width; col++)
        {
            int pix = row * width + col;
            if (col < width - 1) // horizontal neighbor
                mrf->setNeighbors(pix, pix + 1, 1);
            if (row < height - 1) // vertical
                mrf->setNeighbors(pix, pix + width, 1);
            if (row < height - 1 && col < width - 1) // diagonal
                mrf->setNeighbors(pix, pix + width + 1, 1);
        }
    }
    mrf->initialize();
    mrf->clearAnswer();
    float t;
    mrf->optimize(5, t); // run for 5 iterations, store time t it took
    MRF::EnergyVal E_smooth = mrf->smoothnessEnergy();
    MRF::EnergyVal E_data   = mrf->dataEnergy();
    cout << "Optimized Energy: smooth = " << E_smooth << ", data = " << E_data << endl;
    cout << "Time consumed in MRF: " << t << endl;

    // Get MRF result
    for (int row = 0; row < height; row++)
    {
        for (int col = 0; col < width; col++)
        {
            int pix                                               = row * width + col;
            plane_detection.opt_seg_img_.at<cv::Vec3b>(row, col)  = plane_detection.plane_colors_[mrf->getLabel(pix)];
            plane_detection.opt_membership_img_.at<int>(row, col) = mrf->getLabel(pix);
        }
    }
    delete mrf;
    delete energy;
    delete smooth;
    delete data;
}
//-----------------------------------------------------------------

int main(int argc, char* argv[])
try
{
    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline pipe;
    rs2::config   cfg;

    cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 60);
    cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 60);
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 60);

    // Configure and start the pipeline
    pipe.start(cfg);

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
        rs2::frame       color_frame = frames.get_color_frame();
        rs2::frame       ir_frame    = frames.first(RS2_STREAM_INFRARED);
        rs2::depth_frame depth_frame = frames.get_depth_frame();

        // auto filtered_frame = dec_filter.process(depth_frame);
        auto filtered_frame = thr_filter.process(depth_frame);
        filtered_frame = spat_filter.process(filtered_frame);
        filtered_frame = temp_filter.process(filtered_frame);

        rs2::video_frame colorized_depth_frame = color_map.process(filtered_frame);

        // Creating OpenCV Matrix from a color image
        cv::Mat color(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth(cv::Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat colorized_depth(cv::Size(640, 480), CV_8UC3, (void*)colorized_depth_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat ir(cv::Size(640, 480), CV_8UC1, (void*)ir_frame.get_data(), cv::Mat::AUTO_STEP);

        equalizeHist(ir, ir);
        applyColorMap(ir, ir, cv::COLORMAP_JET);

        // equalizeHist(depth, depth);
        // applyColorMap(depth, depth, cv::COLORMAP_JET);

        plane_detection.setColorImage(color);
        plane_detection.setDepthImage(depth);
        plane_detection.runPlaneDetection();
        plane_detection.prepareForMRF();
        // runMRFOptimization();

        imshow("Segment", plane_detection.seg_img_);
        imshow("Depth", colorized_depth);
        imshow("Color", color);
        imshow("IR", ir);
        if (cv::waitKey(1) == 27)
            break;

        // Get the depth frame's dimensions
        //auto width  = depth_frame.get_width();
        //auto height = depth_frame.get_height();

        // Query the distance from the camera to the object in the center of the image
        //float dist_to_center = depth_frame.get_distance(width / 2, height / 2);

        // Print the distance
        //std::cout << "The camera is facing an object " << dist_to_center << " meters away \r";
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
