#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <deque>
#include <sstream>
#include <iomanip>

// IMU Data Structure with timestamp
struct IMUData {
    double timestamp; // in milliseconds
    float accel_x, accel_y, accel_z;
    float gyro_x, gyro_y, gyro_z;
};

// Buffer to store IMU data for visualization
std::deque<IMUData> imu_buffer;
const int buffer_size = 100; // Keep last 100 IMU readings

// IMU Scale Constants
const int plot_width = 1000;
const int plot_height = 1000;
const float scale_factor = 15.0; // Adjusted for -10 to 10 range

// Function to draw the IMU data plot with scale and timestamps
cv::Mat draw_imu_plot() {
    cv::Mat plot(plot_height, plot_width, CV_8UC3, cv::Scalar(255, 255, 255));
    // Draw Y-axis grid lines and labels (-10 to 10)
    for (int i = -20; i <= 20; i += 2) {
        int y = plot_height / 2 - (i * scale_factor);
        cv::line(plot, cv::Point(0, y), cv::Point(plot_width, y), cv::Scalar(200, 200, 200), 1);
        cv::putText(plot, std::to_string(i), cv::Point(5, y + 5), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    }
    if (imu_buffer.empty()) return plot;
    double min_ts = imu_buffer.front().timestamp;
    double max_ts = imu_buffer.back().timestamp;
    double time_span = max_ts - min_ts;
    if (time_span == 0) time_span = 1; // Prevent division by zero
    int x_offset = 50;
    double x_scale = (plot_width - x_offset) / time_span;

    // Draw X-axis labels (elapsed time in seconds)
    double elapsed_max = (max_ts - min_ts) / 1000.0; // Convert to seconds
    cv::putText(plot, "0s", cv::Point(x_offset, plot_height - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);
    cv::putText(plot, std::to_string(elapsed_max) + "s", cv::Point(plot_width - 50, plot_height - 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0), 1);

    // Draw data lines
    for (size_t i = 1; i < imu_buffer.size(); ++i) {
        double ts_prev = imu_buffer[i - 1].timestamp;
        double ts_current = imu_buffer[i].timestamp;
        int x_prev = x_offset + (ts_prev - min_ts) * x_scale;
        int x_current = x_offset + (ts_current - min_ts) * x_scale;

        // Acceleration X
        int y_prev_accel = plot_height / 2 - imu_buffer[i - 1].accel_x * scale_factor;
        int y_current_accel = plot_height / 2 - imu_buffer[i].accel_x * scale_factor;
        cv::line(plot, cv::Point(x_prev, y_prev_accel), cv::Point(x_current, y_current_accel), cv::Scalar(0, 0, 255), 2);

        // Gyro X
        int y_prev_gyro = plot_height / 2 - imu_buffer[i - 1].gyro_x * scale_factor;
        int y_current_gyro = plot_height / 2 - imu_buffer[i].gyro_x * scale_factor;
        cv::line(plot, cv::Point(x_prev, y_prev_gyro), cv::Point(x_current, y_current_gyro), cv::Scalar(0, 255, 0), 2);
    }

    cv::putText(plot, "IMU Data (Red: Accel X, Green: Gyro X)",
                cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5,
                cv::Scalar(0, 0, 0), 1);
    return plot;
}

// Helper function to convert RealSense format to OpenCV format
int rs2FormatToOpenCvFormat(rs2_format format) {
    switch (format) {
        case RS2_FORMAT_Z16:
            return CV_16UC1;
        default:
            throw std::runtime_error("Unsupported RealSense format");
    }
}

int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <path_to_bag>\n";
        return 1;
    }

    rs2::pipeline pipe;
    rs2::config cfg;

    // Disable repeat playback
    cfg.enable_device_from_file(argv[1], false); // <-- CHANGED: Added 'false' to disable looping
    pipe.start(cfg);

    std::cout << "Playing back: " << argv[1] << " (ONCE)" << std::endl;

    // Create filters
    rs2::decimation_filter decimationFilter;
    rs2::threshold_filter thresholdFilter;
    rs2::spatial_filter spatialFilter;
    rs2::temporal_filter temporalFilter;

    // Configure filters (optional)
    decimationFilter.set_option(RS2_OPTION_FILTER_MAGNITUDE, 2); // Downsample by a factor of 2
    thresholdFilter.set_option(RS2_OPTION_MIN_DISTANCE, 0.1f);   // Minimum distance in meters
    thresholdFilter.set_option(RS2_OPTION_MAX_DISTANCE, 10.0f);  // Maximum distance in meters
    spatialFilter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.5f);
    spatialFilter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20);
    temporalFilter.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.4f);
    temporalFilter.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 100);

    while (true) {
        rs2::frameset frames;
        if (pipe.poll_for_frames(&frames)) {
            // Process depth frame
            if (rs2::depth_frame f = frames.get_depth_frame()) {
                // Apply filters
                rs2::frame filtered_frame = f;
                filtered_frame = decimationFilter.process(filtered_frame);
                filtered_frame = thresholdFilter.process(filtered_frame);
                filtered_frame = spatialFilter.process(filtered_frame);
                filtered_frame = temporalFilter.process(filtered_frame);

                // Convert filtered depth frame to OpenCV Mat
                cv::Mat depth_mat(cv::Size(f.as<rs2::video_frame>().get_width(),
                                           f.as<rs2::video_frame>().get_height()),
                                  rs2FormatToOpenCvFormat(f.get_profile().format()),
                                  (void*)f.get_data(), cv::Mat::AUTO_STEP);

                // Display the depth image
                cv::imshow("Depth Stream", depth_mat);
            }

            // Process IR1 frame
            if (rs2::video_frame f = frames.get_infrared_frame(1)) {
                rs2::frame ir1 = frames.get_infrared_frame(1);
                cv::Mat ir1_mat(cv::Size(ir1.as<rs2::video_frame>().get_width(),
                                         ir1.as<rs2::video_frame>().get_height()),
                                CV_8UC1, (void*)ir1.get_data(), cv::Mat::AUTO_STEP);
                std::ostringstream oss_ir1;
                oss_ir1 << "TS: " << std::fixed << std::setprecision(2) << ir1.get_timestamp();
                cv::putText(ir1_mat, oss_ir1.str(), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                cv::imshow("IR1 Stream", ir1_mat);
            }

            // Process IR2 frame
            if (rs2::video_frame f = frames.get_infrared_frame(2)) {
                rs2::frame ir2 = frames.get_infrared_frame(2);
                cv::Mat ir2_mat(cv::Size(ir2.as<rs2::video_frame>().get_width(),
                                         ir2.as<rs2::video_frame>().get_height()),
                                CV_8UC1, (void*)ir2.get_data(), cv::Mat::AUTO_STEP);
                std::ostringstream oss_ir2;
                oss_ir2 << "TS: " << std::fixed << std::setprecision(2) << ir2.get_timestamp();
                cv::putText(ir2_mat, oss_ir2.str(), cv::Point(10, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
                cv::imshow("IR2 Stream", ir2_mat);
            }

            // Get IMU data (Accel + Gyro)
            rs2::motion_frame accel = frames.first_or_default(rs2_stream::RS2_STREAM_ACCEL);
            rs2::motion_frame gyro = frames.first_or_default(rs2_stream::RS2_STREAM_GYRO);
            if (accel && gyro) {
                rs2_vector accel_data = accel.get_motion_data();
                rs2_vector gyro_data = gyro.get_motion_data();
                double ts = accel.get_timestamp(); // Using accel's timestamp

                // Store IMU data in buffer
                if (imu_buffer.size() >= buffer_size) imu_buffer.pop_front();
                imu_buffer.push_back({ts, accel_data.x, accel_data.y, accel_data.z,
                                      gyro_data.x, gyro_data.y, gyro_data.z});

                // Draw IMU plot and display
                cv::Mat imu_plot = draw_imu_plot();
                cv::imshow("IMU Data Visualization", imu_plot);
            }
        }

        // Break loop on 'q' key press or end of file
        if (cv::waitKey(1) == 'q') break;
    }

    pipe.stop();
    cv::destroyAllWindows();
    return 0;
}
