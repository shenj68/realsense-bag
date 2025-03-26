#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>

int main() try {
    bool enableDepth = true;
    bool enableIR = true;
    bool enableColor = false;
    bool postProcessing = true;
    
    rs2::pipeline pipe;
    rs2::config config;

    if (enableDepth) {
        config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
    }
    if (enableIR) {
        config.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 15);
        config.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 15);
    }
    if (enableColor) {
        config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 15);
    }

    std::string rsVersion = RS2_API_VERSION_STR;
    std::cout << "librealsense SDK version: " << rsVersion << std::endl;

    rs2::pipeline_profile profile = pipe.start(config);
    rs2::depth_sensor depth_sensor = profile.get_device().first<rs2::depth_sensor>();

    if (enableDepth && depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
        depth_sensor.set_option(RS2_OPTION_LASER_POWER, 0); 
    }

    //rs2::decimation_filter decimation;
    rs2::threshold_filter threshold;
    rs2::spatial_filter spatial;
    rs2::temporal_filter temporal;

    while (true) {
        rs2::frameset frames = pipe.wait_for_frames();

        if (enableDepth) {
            rs2::depth_frame depth_frame = frames.get_depth_frame();
            if (depth_frame) {
                if (postProcessing) {
                    auto filtered = depth_frame;
                    //filtered = decimation.process(filtered);
                    filtered = threshold.process(filtered);
                    filtered = spatial.process(filtered);
                    filtered = temporal.process(filtered);

                    int w = filtered.as<rs2::video_frame>().get_width();
                    int h = filtered.as<rs2::video_frame>().get_height();
                    std::cout << "w: " << w << std::endl;
                    std::cout << "h: " << h << std::endl;

                    cv::Mat depth_mat(cv::Size(w, h), CV_16UC1, (void*)filtered.get_data());
                    
                    cv::Mat normalized, color_depth;
                    cv::normalize(depth_mat, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                    cv::applyColorMap(normalized, color_depth, cv::COLORMAP_BONE);
                    cv::imshow("Depth", color_depth);
                } else {
                    cv::Mat depth_mat(cv::Size(640, 480), CV_16UC1, (void*)depth_frame.get_data());
                    cv::Mat normalized, color_depth;
                    cv::normalize(depth_mat, normalized, 0, 255, cv::NORM_MINMAX, CV_8UC1);
                    cv::applyColorMap(normalized, color_depth, cv::COLORMAP_BONE);
                    cv::imshow("Depth", color_depth);
                }
            }
        }

        // Process IR streams
        if (enableIR) {
            rs2::video_frame left_ir = frames.get_infrared_frame(1);
            rs2::video_frame right_ir = frames.get_infrared_frame(2);
            
            if (left_ir) {
                cv::Mat left_ir_mat(cv::Size(640, 480), CV_8UC1, (void*)left_ir.get_data());
                cv::imshow("Left IR", left_ir_mat);
            }
            if (right_ir) {
                cv::Mat right_ir_mat(cv::Size(640, 480), CV_8UC1, (void*)right_ir.get_data());
                cv::imshow("Right IR", right_ir_mat);
            }
        }

        // Process color stream
        if (enableColor) {
            rs2::video_frame color_frame = frames.get_color_frame();
            if (color_frame) {
                cv::Mat color_mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data());
                cv::imshow("Color", color_mat);
            }
        }

        if (cv::waitKey(1) == 27) break;
    }

    return 0;
}
catch (const rs2::error &e) {
    std::cerr << "RealSense error: " << e.what() << std::endl;
    return 1;
}
