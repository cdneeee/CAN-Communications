#include <CAN.h>

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
  static unsigned long lastSendTime = 0;
  const long sendInterval = 500; // Send sensor data every 0.5 seconds

  // Try to parse a packet
  int packetSize = CAN.parsePacket();

  // Only proceed if a packet was received
  if (packetSize > 0) {
    String receivedMsg = "";
    while (CAN.available()) {
      receivedMsg += (char)CAN.read(); // no it can't read hehe
    }

    if (receivedMsg == "KILL") {
      Serial.println("Kill command received. Shutting down.");
      esp_deep_sleep_start(); // Shut down the device
    } else {
      Serial.println("Received message: " + receivedMsg);
    }
  }

  // Send sensor data at regular intervals
  if (millis() - lastSendTime > sendInterval) {
    // Read sensor value 
    int sensorValue = analogRead(pin here);
    
    // Prepare the sensor data message
    String message = String(sensorValue);
    CAN.beginPacket(0x01); // Use an appropriate ID for your sensor data packet
    CAN.write((const uint8_t*)message.c_str(), message.length());
    CAN.endPacket();

    Serial.println("Sensor data sent: " + message);
    lastSendTime = millis();
  }
}
