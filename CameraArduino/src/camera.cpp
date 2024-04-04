#ifdef NANO_EVERY

#include <Wire.h>
#include <ArduCAM.h>
#include <SPI.h>
#include "memorysaver.h"
#include <Arduino.h>

// Make sure to define the correct CS pin for your Arduino Nano Every
#define CS_PIN 10
const int ledPin = 9;


// Initialize ArduCAM using the settings for OV2640_MINI_2MP_PLUS
ArduCAM myCAM(OV2640, CS_PIN);

void setup() {
  // Initialize serial communication
  Serial.begin(921600);
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, HIGH);

  // Initialize SPI
  SPI.begin();
  
  // Initialize I2C
  Wire.begin();

  // Check if the camera module is detected
  myCAM.write_reg(ARDUCHIP_TEST1, 0x55);
  uint8_t test_val = myCAM.read_reg(ARDUCHIP_TEST1);
  
  if (test_val != 0x55) {
    Serial.println(F("SPI interface error!"));
    while (1);
  }

  // Initialize the camera
  myCAM.set_format(JPEG);
  myCAM.InitCAM();
  
  // Check if the camera module type is OV2640
  uint8_t vid, pid;
  myCAM.wrSensorReg8_8(0xff, 0x01);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_HIGH, &vid);
  myCAM.rdSensorReg8_8(OV2640_CHIPID_LOW, &pid);
  
  if(vid != 0x26 || pid != 0x42) {
    Serial.println(F("Cannot find OV2640 module!"));
    while(1);
  }
  
  // Set the camera to output a smaller resolution to ensure it works within the memory constraints
  myCAM.OV2640_set_JPEG_size(OV2640_320x240);
  
  // Let the serial port settle before starting the main loop
  delay(1000);
}

void loop() 
{
  // Flush the FIFO buffer and clear the capture done flag
  myCAM.flush_fifo();
  myCAM.clear_fifo_flag();
  
  // Start the capture
  myCAM.start_capture();
  //Serial.println(F("Starting image capture..."));
  
  // Wait for the capture to complete
  while (!myCAM.get_bit(ARDUCHIP_TRIG, CAP_DONE_MASK)) {
    // Wait for capture to complete or timeout after a certain period
    delay(100);
  }
  
  //Serial.println(F("Capture complete."));
  
  // Read the FIFO length to know the size of the image
  uint32_t length = myCAM.read_fifo_length();
  
  // Check if there is valid data in FIFO
  if (length >= MAX_FIFO_SIZE || length == 0) {
    Serial.println(F("Invalid data."));
    return;
  }
  
  // Prepare to read from FIFO buffer
  myCAM.CS_LOW();
  myCAM.set_fifo_burst();
  
  // Read data from FIFO and send to serial
  //Serial.println(F("Sending image data..."));
  for (uint32_t i = 0; i < length; i++) {
    uint8_t data = SPI.transfer(0x00);
    Serial.write(data);
  }
  
  // Finish reading from FIFO
  myCAM.CS_HIGH();
  
  // Clear the capture done flag and FIFO buffer for the next image capture
  myCAM.clear_fifo_flag();
  
  // Add a delay between captures or end the loop based on your requirement
  //delay(5000); // For example, a 5-second delay
}

#endif