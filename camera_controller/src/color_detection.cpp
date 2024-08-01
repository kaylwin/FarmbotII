#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <libserialport.h>
#include <algorithm>
#include <thread>
#include <mutex>
#include <atomic>
#include <csignal>
#include <cmath>

extern "C"{
	#include "vl53l5cx_api.h"
}


#define TDEPTH 90

using namespace cv;
using namespace std;




struct sp_port *serial_port;
VL53L5CX_Configuration dev; // VL53L5CX sensor configuration
VL53L5CX_ResultsData results; // VL53L5CX results data
std::atomic<bool> exitFlag(false);

std::mutex imageMutex;
Mat distance_mat(8, 8, CV_8U); // Create an 8x8 matrix to store the distance data

int servo_5_command = 1500;
int servo_4_command = 1500;


double convert_s5(double deg){
    return -12.135 * deg + 2525.5;
}

double reverse_s5(double c5){
    return (c5 - 2525.5)/(-12.135);
}

double reverse_s3(double c3){
    return (c3 - 2601) / 10.629;
}

double reverse_s4(double c4){
    return (c4 - 1516.2) / 11.539;
}

double convert_s4(double deg){
    return 11.539 * deg + 1516.2;
}


void calculate_desired_position(double c3_command, double& c4_deg, double& c5_deg, double dist){


    // Don't do anything if the distance error is zero
    if (dist == -1){
        return;
    }

    // Don't reach for something out of range
    if (dist > 230){
        return;
    }
    double c3_deg = reverse_s3(c3_command);

    std::cout << "###############   START  " << std::endl;
    std::cout << "c3_deg: " << c3_deg << " c4_deg: " << c4_deg << " c5_deg: " << c5_deg << std::endl;
    std::cout << "DEPTH: " << dist << std::endl;
    // Arm measurements
    double l1 = 10.4;
    double l2 = 8.9;
    double l1_sq = std::pow(l1, 2);
    double l2_sq = std::pow(l2, 2);

    double c3_rad = c3_deg * M_PI / 180.0;
    double c4_rad = c4_deg * M_PI / 180.0;
    double c5_rad = c5_deg * M_PI / 180.0;

    // Calculate where we think the servos are given the last position
    double x_old = l1 * std::cos(c5_rad) + l2 * std::cos(c5_rad + c4_rad);
    double y_old = l1 * std::sin(c5_rad) + l2 * std::sin(c5_rad + c4_rad);

    std::cout << "xold: " << x_old << " yold: " << y_old << std::endl;

    // Compute current position and increment the pose 
    // TODO: Adjust by distance -- add ramp filter
    double dp = (dist - TDEPTH); // Subtract target error cvt to cm
    dp = std::max({dp, -20.0});
    dp = std::min({dp, 150.0});
    std::cout << "Depth error: " << dp << std::endl;
    dp *= 1.0 / 500.0;

    // Ramp filter on forwards measurement
    double max_diff = 0.01; // Move 1/10th of a cm
    if (abs(dp) > max_diff){
        dp = (dp > 0) ? max_diff : -max_diff;
    }
    std::cout << "Depth adjustment: " << dp << std::endl;

    double x = x_old + dp * std::cos(c3_rad + c4_rad + c5_rad);
    double y = y_old + dp * std::sin(c3_rad + c4_rad + c5_rad);

    std::cout << "xnew: " << x << " ynew: " << y << std::endl;

    // Compute d for inverse kinematics
    double d_sq = std::pow(x, 2) + std::pow(y, 2);
    double d = std::sqrt(d_sq);

    // Compute phi2
    double phi2_ratio = (-1 * d_sq + l1_sq + l2_sq) / (2 * l1 * l2);
    if (std::abs(phi2_ratio) > 1){
        std::cerr << "Phi2 out of range: " << phi2_ratio << std::endl;
        return;
    }

    auto phi2 = std::acos(phi2_ratio);

    // Compute phi1
    double phi1 = std::atan(y / x);

    // Compute phi3
    double phi3_ratio = (-l2_sq + l1_sq + d_sq) / (2 * l1 * d);
    if (abs(phi3_ratio) > 1){
        std::cerr << "Phi3 out of range" << std::endl;
        return;
    }
    auto phi3 = std::acos(phi3_ratio);

    // Integrate and calculate inverse kinematics
    double theta1 = phi1 + phi3;
    double theta2 = -1 * (M_PI - phi2);


    // Convert to degrees
    theta1 *= 180.0 / M_PI;
    theta2 *= 180.0 / M_PI;

    std::cout << "new theta1: " << theta1 << " new theta2: " << theta2 << std::endl;

    // Bounds check then assignment
    if (theta1 > 90.0 || theta1 < 0){
        std::cerr << "Theta 1 out of bounds: " << theta1 << std::endl;
        return;
    }
    if (theta2 > 0 || theta2 < -90){
        std::cerr << "Theta 2 out of bounds: " << theta2 << std::endl;
        return;
    }

    c5_deg = theta1;  // First joint by base is 5
    c4_deg = theta2;  // Second joint by base is 4
}



void send_command(int servo_id, int position, int duration) {
    position = static_cast<int>(position);
    duration = static_cast<int>(duration);
    uint8_t command[] = {
        0x55, 0x55, // Header
        8,
        0x03, // COMMAND SERVO MOVE
        1,
        static_cast<uint8_t>(duration & 0xFF), static_cast<uint8_t>((duration >> 8) & 0xFF), // Low byte and high byte of duration
        static_cast<uint8_t>(servo_id),
        static_cast<uint8_t>(position & 0xFF), static_cast<uint8_t>((position >> 8) & 0xFF) // Low byte and high byte of position
    };

    // Send the command
    sp_nonblocking_write(serial_port, command, sizeof(command));
}


void signalHandler(int signal){
    if (signal == SIGINT){
       std::cout << "Received sigint, exiting" << std::endl;
       exitFlag.store(true);
    }
}

double global_depth = 255;

std::pair<double, double> calculateCentroid(const cv::Mat& image) {
    double sumX = 0;
    double sumY = 0;
    int count = 0;

    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            uchar pixelValue = image.at<uchar>(y, x);
            if (pixelValue != 255) { // Non-background pixel
                sumX += x;
                sumY += y;
                ++count;
            }
        }
    }

    if (count == 0) {
        return {0, 0}; // Avoid division by zero
    }

    double meanX = sumX / count;
    double meanY = sumY / count;
    return {meanX, meanY};
}

void run_lidar(int thread_id){
    // Read distance data from the VL53L5CX sensor
    while (!exitFlag.load()){
        if (vl53l5cx_get_ranging_data(&dev, &results) == 0) {
            std::lock_guard<std::mutex> guard(imageMutex);
            for (int i = 0; i < 64; i++) {
                int row = i / 8;
                int col = i % 8;
                float dist = static_cast<float>(results.distance_mm[i]);

                if (dist > 255){
                    dist = 255;
                }

                distance_mat.at<uint8_t>(row, col) = static_cast<uint8_t>(dist);
            }


            auto& image = distance_mat;
            // Create a Gaussian kernel
            int rows = image.rows;
            int cols = image.cols;

            // TODO: Calculate the center
            auto center = calculateCentroid(image);
            double sigma = 2.0;
            double meanX = center.first;
            double meanY = center.second;

            double sumWeightedPixelValues = 0;
            double sumWeights = 0;

            for (int y = 0; y < rows; ++y) {
                for (int x = 0; x < cols; ++x) {
                    uchar pixelValue = image.at<uchar>(y, x);
                    if (pixelValue != 255){
                        // Compute the Gaussian weight
                        double weight = std::exp(-0.5 * (std::pow((x - meanX) / sigma, 2) + std::pow((y - meanY) / sigma, 2)));
                        sumWeightedPixelValues += weight * pixelValue;
                        sumWeights += weight;
                    }
                }
            }


            if (sumWeights != 0) {
                // Compute the weighted average pixel value
                double weightedAveragePixelValue = sumWeightedPixelValues / sumWeights;
                std::cout << "Weighted Average Pixel Value: " << weightedAveragePixelValue << std::endl;
                global_depth += 0.25 * (weightedAveragePixelValue - global_depth);
            }
        }
    }
}


double process_depth(){
    std::lock_guard<std::mutex> guard(imageMutex);  // Lock the lidar image
    auto image = distance_mat.clone();
    imshow("lidar", image);
    return global_depth;
}


int main() {

    std::signal(SIGINT, signalHandler);
   // Open the serial port
    sp_get_port_by_name("/dev/ttyAMA0", &serial_port);
    if (sp_open(serial_port, SP_MODE_WRITE) != SP_OK) {
        cerr << "Error: Could not open serial port." << endl;
        return -1;
    }
    sp_set_baudrate(serial_port, 9600);


	auto status = vl53l5cx_comms_init(&dev.platform);
	if(status)
	{
			printf("VL53L5CX ULD Loading failed\n");
			return status;
	}

	// Setup the lidar
	//vl53l5cx_start_ranging(&dev);
      if (vl53l5cx_init(&dev) != 0) {
        cerr << "VL53L5CX: Failed to initialize sensor" << endl;
        return -1;
    }

    // Set the resolution to 8x8
    if (vl53l5cx_set_resolution(&dev, VL53L5CX_RESOLUTION_8X8) != 0) {
        cerr << "VL53L5CX: Failed to set resolution to 8x8" << endl;
        return -1;
    }

    // Set the ranging frequency to 10 Hz
    if (vl53l5cx_set_ranging_frequency_hz(&dev, 10) != 0) {
        cerr << "VL53L5CX: Failed to set ranging frequency" << endl;
        return -1;
    }

    // Start ranging
    if (vl53l5cx_start_ranging(&dev) != 0) {
        cerr << "VL53L5CX: Failed to start ranging" << endl;
        return -1;
    }

    int command = 2000;
    int last_command = 2000;
    double error = 0;
    
    VideoCapture capture_thread(0); // Open the default camera
    if (!capture_thread.isOpened()) {
        cerr << "Error: Could not open the camera." << endl;
        return -1;
    }


    double vert_error = 0;
    double last_vert_error = 0;
    double last_vert_command = 1500;
    double vert_command = 1500;

    Mat frame, hsv_img, mask, mask1, mask2;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;

    // Disable auto white balance
    capture_thread.set(CAP_PROP_AUTO_WB, 0);

    namedWindow("Contours", WINDOW_NORMAL); // Create a window to display the contours
    resizeWindow("Contours", 400, 400);
    namedWindow("lidar", WINDOW_NORMAL); // Create a window to display the contours
    resizeWindow("lidar", 400, 400);


    // Start out c4 and c5 with basic values
    double c4_deg = 0;
    double c5_deg = 80;

	send_command(4, convert_s4(c4_deg), 1000);
	send_command(5, convert_s5(c5_deg), 1000);


    // Initialize to known predictable values
	send_command(6, command, 1000);
	send_command(3, vert_command, 1000);
	usleep(1'000'000);

    auto lidar_thread = std::thread(run_lidar, 1);

    double depth = 255;
    double filtered_depth = 255;
    while (!exitFlag.load()) {
        // Step 2: Capture the frame
        capture_thread >> frame;
        if (frame.empty()) {
            cerr << "Error: Could not capture frame." << endl;
            break;
        }

        // Process depth
        depth = process_depth();

        // Convert to HSV
        cvtColor(frame, hsv_img, COLOR_BGR2HSV);

        // Define color range and create mask
        Scalar lower_bound(0, 30, 60);
        Scalar upper_bound(15, 255, 255);
        inRange(hsv_img, lower_bound, upper_bound, mask1);

		Scalar lower_bound2(170, 30, 60);
        Scalar upper_bound2(180, 255, 255);
        inRange(hsv_img, lower_bound2, upper_bound2, mask2);

        // Combine the masks
        mask = mask1 | mask2;

        // Erode and dilate to clean up the mask
       //Mat kernel = getStructuringElement(MORPH_RECT, Size(3, 3));
        //erode(mask, mask, kernel, Point(-1, -1), 2);
        //dilate(mask, mask, kernel, Point(-1, -1), 2);

        // Find contours
        findContours(mask, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);

        // Filter contours based on minimum area (20000 pixels)
        int min_contour_area = 2000; vector<vector<Point>> filtered_contours;
        for (const auto& cnt : contours) {
            if (contourArea(cnt) >= min_contour_area) {
                filtered_contours.push_back(cnt);
            }
        }

        Mat contour_image = frame.clone(); // Create a copy of the frame to draw contours

        if (!filtered_contours.empty()) {
            // Find the largest contour
            auto largest_contour = max_element(filtered_contours.begin(), filtered_contours.end(),
                    [](const vector<Point>& a, const vector<Point>& b) {
                    return contourArea(a) < contourArea(b);
                    });

            Moments M = moments(*largest_contour);
            int cx = 0, cy = 0;
            if (M.m00 != 0) {
                cx = static_cast<int>(M.m10 / M.m00);
                cy = static_cast<int>(M.m01 / M.m00);
            }

            // Draw the centroid on the image
            circle(contour_image, Point(cx, cy), 20, Scalar(0, 0, 255), -1); // Red circle

            // Draw contours on the original image
            drawContours(contour_image, filtered_contours, -1, Scalar(0, 255, 0), 2); // Green contours

            // Calculate the average hue in the largest contour
            Mat mask_largest_contour = Mat::zeros(mask.size(), CV_8UC1);
            drawContours(mask_largest_contour, vector<vector<Point>>{*largest_contour}, -1, Scalar(255), FILLED);

            Scalar mean_hue = mean(hsv_img, mask_largest_contour);
            double average_hue = mean_hue[0];

            ///#cout << "Average Hue in Largest Contour: " << average_hue << endl;
            double compensation = 0.1;
            double depth_error = depth - TDEPTH;
            depth_error = std::max({depth_error, 20.0});
            depth_error = std::min({depth_error, 150.0});
            compensation *= depth_error / 150.0;
            std::cout << "Copmensation: " << compensation << std::endl;


            std::cout << "Center (" << cx <<", " << cy ")" << std::endl;

            int image_width = frame.cols;
            double new_error = image_width * 0.4 - cx;

            // Update the error using a low-pass filter
            error += 1.0 * (new_error - error);
            command += compensation * error;

            // Ramp filter to prevent large gaps
            int max_diff = 100;
            if (abs(command - last_command) > max_diff) {
                if (command - last_command < 0) {
                    command = last_command - max_diff;
                } else {
                    command = last_command + max_diff;
                }
            }

            // Adjust command based on the centroid position
            if (command < 1000) command = 1200;
            if (command > 2600) command = 2400;

            last_command = command;
            std::cout << "Command: " << command << std::endl;

            vert_error = frame.rows * 0.6 - cy;
            vert_command += compensation * vert_error;

            if (abs(vert_command - last_vert_command) > max_diff){
                if (vert_command - last_vert_command < 0){
                    vert_command = last_vert_command - max_diff;
                }
                else{
                    vert_command = last_vert_command + max_diff;
                }
            }


            // Add limits to the vertical command
            vert_command = std::min({vert_command, 2600.0});
            vert_command = std::max({vert_command, 1300.0});
            last_vert_command = vert_command;

            // TODO: Send the command 


            //send_command(3, vert_command, 60);
            //send_command(6, command, 60);


            std::cout << "Vert_erorr: " << vert_error <<  "Lat Error: " << error << std::endl;
            std::cout << "Vert_COMMAND: " << vert_command << std::endl;

            // Ensure that the strawberry is correctly aimed at by the algorithm first
            if (abs(vert_error) < 240 * 0.7 && abs(error) < 320 * 0.7){
            // Compute the inverse kinematics of the robot arm
                calculate_desired_position(vert_command, c4_deg, c5_deg, depth);
                auto s4_command = convert_s4(c4_deg);
                auto s5_command = convert_s5(c5_deg);
                //send_command(4, s4_command, 60);
                //send_command(5, s5_command, 60);
                //usleep(200);
            }
            else{
                std::cout << "No depth adjustment!!" << std::endl;
            }



        
            // Display the contours
            imshow("Contours", contour_image);

            if (waitKey(5) >= 0) {
                break;
            }

            filtered_depth += 0.125 * (depth - filtered_depth);

            std::cout << "Filtered Depth: " << filtered_depth << std::endl;
            usleep(200);

        }

    }

    lidar_thread.join();

    capture_thread.release();
    destroyAllWindows(); // Destroy all the windows created
    return 0;
}

