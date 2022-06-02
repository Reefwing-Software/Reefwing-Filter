/******************************************************************
  @file       simpleAHRS.ino
  @brief      Plots roll, pitch and yaw for Nano 33 BLE
  @author     David Such
  @copyright  Please see the accompanying LICENSE file.

  Code:        David Such
  Version:     1.0
  Date:        07/03/22

  1.0 Original Release          07/03/22

  This sketch is designed to be used with the Arduino IDE Serial 
  Plotter (CTRL+SHFT+L). The vertical (y-axis) on the plotter
  auto scales and the x-axis is a fixed 500 points with each tick
  mark indicative of a Serial.println(). With a 20 ms delay you see
  10 seconds of data (i.e., 500 x 20 ms).

  If you want to stop autoscaling, add the following to loop():
  Serial.println("Min:0,Max:1023");

******************************************************************/

#include <NexgenFilter.h>

#TODO