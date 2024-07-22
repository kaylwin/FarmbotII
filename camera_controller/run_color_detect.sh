#! /bin/bash

g++ -o color_detection src/color_detection.cpp uld-driver/libuld_driver.a -lserialport `pkg-config --cflags --libs opencv4` -I./uld-driver/inc 


