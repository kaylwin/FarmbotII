
To run motors from joystick
cd ChassisController/src then run `python3 joystick.py`
If you get a /dev/ttyACM0 not found error then the USB to micro controller is not plugged in


To run the arm
>>cd camera_controller/build
>>make
>>./color_detection

To change arm paramters -- Edit script below
cd camera_controller/src/color_detection.cpp


As a general note for debugging -- comms to arm go over /dev/ttyAMA0





