#include <Arduino.h>
#include "../lib/SparkFun_VL53L5CX_Arduino_Library/src/SparkFun_VL53L5CX_Library.h"
#include "LobotServoController.h"

#include <Wire.h>

/*
  Read an 8x8 array of distances from the VL53L5CX
  By: Nathan Seidle
  SparkFun Electronics
  Date: October 26, 2021
  License: MIT. See license file for more information but you can
  basically do whatever you want with this code.

  This example shows how to increase output frequency.

  Default is 1Hz.
  Using 4x4, min frequency is 1Hz and max is 60Hz
  Using 8x8, min frequency is 1Hz and max is 15Hz

  Feel like supporting our work? Buy a board from SparkFun!
  https://www.sparkfun.com/products/18642

*/

#include <Wire.h>

#include <SparkFun_VL53L5CX_Library.h> //http://librarymanager/All#SparkFun_VL53L5CX
#include <string>

//SparkFun_VL53L5CX myImager;
//VL53L5CX_ResultsData measurementData; // Result data class structure, 1356 byes of RAM

LobotServoController controller(Serial3);

int imageResolution = 0; //Used to pretty print output
int imageWidth = 0; //Used to pretty print output

void setup()
{
    Serial.begin(115200);
    Serial3.begin(9600);
    delay(1000);
    Serial.println("SparkFun VL53L5CX Imager Example");

    /*
    Wire.begin(); //This resets I2C bus to 100kHz
    Wire.setClock(400000); //Sensor has max I2C freq of 400kHz
     */


    /*
    Serial.println("Initializing sensor board. This can take up to 10s. Please wait.");
    if (myImager.begin() == false)
    {
        while (1){
            delay(1000);
            Serial.println(F("Sensor not found - check your wiring. Freezing"));
        }
    }


    myImager.setResolution(8 * 8); //Enable all 64 pads

    imageResolution = myImager.getResolution(); //Query sensor for current resolution - either 4x4 or 8x8
    imageWidth = sqrt(imageResolution); //Calculate printing width

    //Using 4x4, min frequency is 1Hz and max is 60Hz
    //Using 8x8, min frequency is 1Hz and max is 15Hz
    bool response = myImager.setRangingFrequency(15);
    if (response == true)
    {
        int frequency = myImager.getRangingFrequency();
        if (frequency > 0)
        {
            Serial.print("Ranging frequency set to ");
            Serial.print(frequency);
            Serial.println(" Hz.");
        }
        else
            Serial.println(F("Error recovering ranging frequency."));
    }
    else
    {
        Serial.println(F("Cannot set ranging frequency requested. Freezing..."));
        while (1) ;
    }

    myImager.startRanging();
     */
}

void loop()
{

    char bytes[10000];
    int bytes_read = Serial.readBytesUntil('\n', bytes, 10000 - 1);

    int pos = -1;
    if (bytes_read > 0){
        bytes[bytes_read + 1] = '\0';

        int id;
        int res = sscanf(bytes, "%d %d", &id, &pos);
        if (res == 2){
            controller.moveServo(id, pos, 1500);
        }
    }

    //  Servo6 is the base
    // Servo5 is the base joint

    delay(1000);
    //controller.moveServo(4, 15, 500);

    //Poll sensor for new data
    /*
    if (myImager.isDataReady() == true)
    {
        if (myImager.getRangingData(&measurementData)) //Read distance data into array
        {
            //The ST library returns the data transposed from zone mapping shown in datasheet
            //Pretty-print data with increasing y, decreasing x to reflect reality
            for (int y = 0 ; y <= imageWidth * (imageWidth - 1) ; y += imageWidth)
            {
                for (int x = imageWidth - 1 ; x >= 0 ; x--)
                {

                    Serial.print(measurementData.distance_mm[x + y]);

                    // Don't print the last comma
                    if (x != 0 || y != imageWidth * (imageWidth - 1)){
                        Serial.print(",");
                    }
                }
            }
            Serial.println();
        }
    }
     */

}