#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <libserialport.h>
#include <algorithm>

using namespace cv;
using namespace std;


struct sp_port *serial_port;

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

int main() {

   // Open the serial port
    sp_get_port_by_name("/dev/ttyAMA0", &serial_port);
    if (sp_open(serial_port, SP_MODE_WRITE) != SP_OK) {
        cerr << "Error: Could not open serial port." << endl;
        return -1;
    }
    sp_set_baudrate(serial_port, 9600);


    int command = 1500;
    int last_command = 1500;
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

    namedWindow("Contours", WINDOW_AUTOSIZE); // Create a window to display the contours

	send_command(5, 1500, 1000);
	send_command(4, 1500, 1000);
	send_command(6, 1500, 1000);
	send_command(3, vert_command, 1000);
	usleep(1'000'000);

    while (true) {
        // Step 2: Capture the frame
        capture_thread >> frame;
        if (frame.empty()) {
            cerr << "Error: Could not capture frame." << endl;
            break;
        }


        // Convert to HSV
        cvtColor(frame, hsv_img, COLOR_BGR2HSV);

        // Define color range and create mask
        Scalar lower_bound(0, 50, 30);
        Scalar upper_bound(10, 255, 255);
        inRange(hsv_img, lower_bound, upper_bound, mask1);

		Scalar lower_bound2(175, 50, 30);
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

            // Adjust command based on the centroid position
            if (command < 800) command = 800;
            if (command > 2600) command = 2400;

            int image_width = frame.cols;
            double new_error = image_width / 2.0 - cx;

            // Update the error using a low-pass filter
            error += 1.0 * (new_error - error);
            command += 0.5 * error;

            // Ramp filter to prevent large gaps
            int max_diff = 100;
            if (abs(command - last_command) > max_diff) {
                if (command - last_command < 0) {
                    command = last_command - max_diff;
                } else {
                    command = last_command + max_diff;
                }
            }

            last_command = command;



	   // TODO: Compute vertical command here
	    vert_error = frame.rows * 0.75 - cy;
	    vert_command += 0.5 * vert_error;

	    if (abs(vert_command - last_vert_command) > max_diff){
		    if (vert_command - last_vert_command < 0){
			    vert_command = last_vert_command - max_diff;
		    }
		    else{
			    vert_command = last_vert_command + max_diff;
		    }
	    }

	    last_vert_command = vert_command;


		// Add limits to the vertical command
		vert_command = std::min({vert_command, 2600.0});
		vert_command = std::max({vert_command, 1300.0});


		usleep(60'000); // Sleep for 150ms was at 40
		send_command(6, command, 40); // was at 40
		send_command(3, vert_command, 40);

            cout << "Command: " << command << " Error: " << error << endl;
        }

        // Display the contours
        imshow("Contours", contour_image);

        if (waitKey(30) >= 0) {
            break;
        }
    }

    capture_thread.release();
    destroyAllWindows(); // Destroy all the windows created
    return 0;
}

