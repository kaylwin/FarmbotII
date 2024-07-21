import cv2
import matplotlib.pyplot as plt
import numpy as np
#from IPython.display import clear_output
import serial
import time




#cap = cv2.VideoCapture("v4l2src device=/dev/video0 ! videoconvert ! appsink", cv2.CAP_GSTREAMER)
#cap = cv2.VideoCapture(0)

# Function to calculate LewanSoul-specific checksum
# Open the serial port
ser = serial.Serial('/dev/ttyAMA0', 9600, timeout=1)  # Adjust the device and baud rate

import threading

class VideoCaptureThread(threading.Thread):
    def __init__(self, src=0):
        super(VideoCaptureThread, self).__init__()
        self.cap = cv2.VideoCapture(src)
        self.cap.set(cv2.CAP_PROP_POS_AVI_RATIO, 1)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # Set to desired width
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # Set to desired height
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # Set the buffer size to 1 frame
        self.cap.set(cv2.CAP_PROP_FPS, 60)
        self.ret, self.frame = self.cap.read()
        self.running = True

    def run(self):
        while self.running:
            self.ret, self.frame = self.cap.read()

    def stop(self):
        self.running = False
        self.cap.release()

    def get_frame(self):
        return self.ret, self.frame

capture_thread = VideoCaptureThread(0)
capture_thread.start()

def send_command(servo_id, position, duration):
    print("Sending Command")
    position = int(position)
    duration = int(duration)
    command = [0x55,
               0x55,  # Header
               8, 
               0x03, # COMMAND SERVO MOVE
               1, 
               duration & 0xFF, (duration >> 8) & 0xFF,  # Low byte and high byte of duration
               servo_id,
               position & 0xFF, (position >> 8) & 0xFF]  # Low byte and high byte of position
        

    # Send the command
    ser.write(bytearray(command))



###################################################################################
###################################################################################
#    FIND THE STRAWBERRY
###################################################################################
###################################################################################

command = 1500
last_command = 1500

send_command(6, 1500, 1500)
error = 0

while True:
# Step 2: Flatten the
    ret, frame = capture_thread.get_frame()
    hsv_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    flatten_image = hsv_img[:, :, 0].flatten()

    # Frame comes in a bgr
    bgr_img = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    # Step 3: Plot the histogram

    #fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(12, 12))

    h_img = hsv_img[:, :, 0]
    s_img = hsv_img[:, :, 1]
    v_img = hsv_img[:, :, 2]

    lower_bound = np.array([0, 50, 140])
    upper_bound = np.array([25, 255, 255])
    mask1 = cv2.inRange(hsv_img, lower_bound, upper_bound)

    #lower_bound = np.array([170, 50, 50])
    #upper_bound = np.array([180, 255, 255])

    #mask2 = cv2.inRange(hsv_img, lower_bound, upper_bound)
    mask = mask1

    # Define a 3x3 kernel
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.erode(mask, kernel, iterations=2)
    mask = cv2.dilate(mask, kernel, iterations=2)


    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    # Filter contours based on minimum area (40 pixels)
    min_contour_area = 20_000
    filtered_contours = [cnt for cnt in contours if cv2.contourArea(cnt) >= min_contour_area]
    countour_image = []

    if filtered_contours:
        largest_contour = max(contours, key=cv2.contourArea)
        M = cv2.moments(largest_contour)

        # Calculate centroid
        if M['m00'] != 0:
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])
        else:
            cx, cy = 0, 0  # Handles division by zero if the contour has no area

        # Draw the centroid on the image
        contour_image = np.copy(frame)
        #cv2.circle(contour_image, (cx, cy), 20, (0, 0, 255), -1)  # Red circle

        # Draw contours on the original image
        #cv2.drawContours(contour_image,filtered_contours, -1, (0, 255, 0), 2)  # Draw all contours in green with thickness 2


        if command < 800:
            command = 800
        elif command > 2600:
            command = 2400

        image_width = contour_image.shape[1]
        print("Image width: {}".format(image_width))
        print(cx, cy)

        new_error = image_width / 2.0 - cx

        error += 1/4.0 *(new_error - error)


        command += 0.15 * error

        # Ramp filter to prevent large gaps

        max_diff = 200
        if abs(command - last_command) > max_diff:
            if command - last_command < 0:
                command = last_command - max_diff
            else:
                command = last_command + max_diff

        last_command = command
        time.sleep(0.150)
        send_command(6, command, 100)
        print("Command:  {} Error: {}".format(command ,error))



        #plt.hist(flatten_image, bins=256, range=(0, 180), density=True, color='blue', alpha=0.7)
        #plt.imshow(contour_image, cmap="grey")
        #plt.grid(True)
        #plt.show()
