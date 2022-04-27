/* DIY calibration program for the LSM9DS1 chip 
 *  
 * Follow the instructions on the screen how to do calibration measurements. 
 * See instruction video https://youtu.be/BLvYFXoP33o
 * No special tools or setups are needed, however it is handy if the board with the LSM9DS1 chip is fitted inside 
 * a non-metalic rectangular box.
 * The Full Scale (FS)and Output DATA Rate (ODR) settings as well as offset and slope factors 
 * are displayed on screen as code that can be copy/pasted directly into a sketch.  
 * Each new instance of the chip will require it's own unique set of calibration factors. 
 * It is recommended that the sketch uses the same FS and ODR settings as the calibration program
 * 
 * Menu operation: type a letter in the input box of the serial monitor followed by enter
 * 
 * written by Femme Verbeek 6 July 2020
 * 
 * This program uses V2 of the LSM9DS1 library 
 * Tested on an Arduino Nano 33 BLE Sense board.
 * 
 * 
 */

#include <Arduino_LSM9DS1.h> 
#include "functions.h"

uint8_t accelODRindex=5; // Sample Rate 0:off, 1:10Hz, 2:50Hz, 3:119Hz, 4:238Hz, 5:476Hz, (6:952Hz=na) 
uint8_t accelFSindex=3;   // Full Scale// 0: ±2g ; 1: ±24g ; 2: ±4g ; 3: ±8g

int main() {
  Serial.begin(9600); 
  while (!Serial);
  pinMode(LED_BUILTIN,OUTPUT); 
  delay(10);
  if (!IMU.begin()) { Serial.println(F("Failed to initialize IMU!")); while (1);  }
  IMU.setAccelFS(accelFSindex);   
  IMU.setAccelODR(accelODRindex);   
  calibrateAccelMenu(accelFSindex, accelODRindex);

  for(;;){

  }
}


