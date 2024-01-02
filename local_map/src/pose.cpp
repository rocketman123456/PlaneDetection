// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2019 Intel Corporation. All Rights Reserved.
#include <librealsense2/rs.hpp>
#include <iostream>
#include <iomanip>
#include "example-utils.hpp"

// typedef struct rs2_pose
// {
//     rs2_vector      translation;          /**< X, Y, Z values of translation, in meters (relative to initial position)                                    */
//     rs2_vector      velocity;             /**< X, Y, Z values of velocity, in meters/sec                                                                  */
//     rs2_vector      acceleration;         /**< X, Y, Z values of acceleration, in meters/sec^2                                                            */
//     rs2_quaternion  rotation;             /**< Qi, Qj, Qk, Qr components of rotation as represented in quaternion rotation (relative to initial position) */
//     rs2_vector      angular_velocity;     /**< X, Y, Z values of angular velocity, in radians/sec                                                         */
//     rs2_vector      angular_acceleration; /**< X, Y, Z values of angular acceleration, in radians/sec^2                                                   */
//     unsigned int    tracker_confidence;   /**< Pose confidence 0x0 - Failed, 0x1 - Low, 0x2 - Medium, 0x3 - High                                          */
//     unsigned int    mapper_confidence;    /**< Pose map confidence 0x0 - Failed, 0x1 - Low, 0x2 - Medium, 0x3 - High                                      */
// } rs2_pose;

int main(int argc, char * argv[]) try
{
    std::string serial;
    if (!device_with_streams({ RS2_STREAM_POSE}, serial))
        return EXIT_SUCCESS;

    // Declare RealSense pipeline, encapsulating the actual device and sensors
    rs2::pipeline pipe;
    // Create a configuration for configuring the pipeline with a non default profile
    rs2::config cfg;
    if (!serial.empty())
        cfg.enable_device(serial);
    // Add pose stream
    cfg.enable_stream(RS2_STREAM_POSE, RS2_FORMAT_6DOF);
    // Start pipeline with chosen configuration
    pipe.start(cfg);

    // Main loop
    while (true)
    {
        // Wait for the next set of frames from the camera
        auto frames = pipe.wait_for_frames();
        // Get a frame from the pose stream
        auto f = frames.first_or_default(RS2_STREAM_POSE);
        // Cast the frame to pose_frame and get its data
        auto pose_data = f.as<rs2::pose_frame>().get_pose_data();

        // Print the x, y, z values of the translation, relative to initial position
        std::cout << "\r" << "Device Position: " << std::setprecision(3) << std::fixed << pose_data.translation.x << " " <<
            pose_data.translation.y << " " << pose_data.translation.z << " (meters)";
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}
