#include <CAN.h>
#include <Servo.h>

const uint16_t DEVICE_ID = 0x04;
#define ENCODER1 = pin_number; //idk whether to use uint or define
#define servo1 = pin_number;

#define ENCODER2 = pin_number;
#define servo2 = pin_number;

#define ENCODER3 = pin_number;
#define servo3 = pin_number;


void setup() {

//setting to pins D21 and D22 on ESP32 to match pinlayout SN65HVD235D chip
  CAN.setPins(22, 21);
  

  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

void loop() {
    WBCAN_packet_t receivedPacket;
    if (recv_canbus(&receivedPacket)) {
        // Ensure payload size is adequate for our data structure
        // Assuming the first byte is servoNumber, followed by two bytes for the angle
        if (receivedPacket.payload_size >= 3) {
            int servoNumber = receivedPacket.payload[0];
            int angleHigh = receivedPacket.payload[1];
            int angleLow = receivedPacket.payload[2];
            
            // Combine the high and low parts of the angle
            int angleInt = (angleHigh << 8) | angleLow;
            // Convert back to the original float value (assuming the angle was scaled by 100)
            float angle = angleInt / 100.0f;
            

            // Now you have servoNumber and angle, proceed to control the servo
           
        }
    }
}