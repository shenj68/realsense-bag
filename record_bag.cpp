#include <librealsense2/rs.hpp>
#include <iostream>
#include <opencv2/opencv.hpp>

int rs2FormatToOpenCvFormat(rs2_format format) {
    switch (format) {
        case RS2_FORMAT_Z16: return CV_16UC1;
        default: throw std::runtime_error("Unsupported RealSense format");
    }
}

int main() {
    try {
        rs2::pipeline pipe;
        rs2::config cfg;

        // Enable IR1, IR2, and IMU streams
        cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 15);
        cfg.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 15);
        cfg.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 15);
        cfg.enable_stream(RS2_STREAM_GYRO);
        cfg.enable_stream(RS2_STREAM_ACCEL);

        cfg.enable_record_to_file("/home/fe/Downloads/bags/mar24imutest4.bag");

        rs2::pipeline_profile profile = pipe.start(cfg);
        rs2::device selected_device = profile.get_device();
        rs2::depth_sensor depth_sensor = selected_device.first<rs2::depth_sensor>();

        if (depth_sensor.supports(RS2_OPTION_LASER_POWER)) {
            int desired_laser_power = 0;
            depth_sensor.set_option(RS2_OPTION_LASER_POWER, desired_laser_power);
            std::cout << "Laser power set to: " << desired_laser_power << std::endl;
        } else {
            std::cerr << "This device does not support setting laser power." << std::endl;
        }

        rs2::spatial_filter spatialFilter;
        rs2::temporal_filter temporalFilter;

        spatialFilter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f);
        spatialFilter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
        temporalFilter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f);
        temporalFilter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);

        std::cout << "Recording to 'mar24imutest.bag'. Press ENTER to stop...\n";

        while (true) {
            rs2::frameset frames = pipe.wait_for_frames();
            rs2::frame depth_frame = frames.get_depth_frame();
            if (!depth_frame) continue;
            rs2::frame filtered_frame = depth_frame;
            filtered_frame = spatialFilter.process(filtered_frame);
            filtered_frame = temporalFilter.process(filtered_frame);

            auto vf = filtered_frame.as<rs2::video_frame>();
            cv::Mat depth_image(cv::Size(vf.get_width(), vf.get_height()),
                                rs2FormatToOpenCvFormat(filtered_frame.get_profile().format()),
                                (void*)filtered_frame.get_data(),
                                cv::Mat::AUTO_STEP);
            depth_image = depth_image.clone();

            cv::imshow("Depth Image", depth_image);
            char key = cv::waitKey(1);
            if (key == 27) break;


            if (std::cin.peek() != EOF) break;
        }

        pipe.stop();
        std::cout << "Recording finished.\n";
    }
    catch (const rs2::error &e) {
        std::cerr << "RealSense error: " << e.what() << std::endl;
    }
    catch (const std::exception &e) {
        std::cerr << "Standard exception: " << e.what() << std::endl;
    }
    catch (...) {
        std::cerr << "Unknown error occurred." << std::endl;
    }

    return 0;
}
