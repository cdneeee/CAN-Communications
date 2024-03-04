#include <CAN.h>

const uint16_t DEVICE_ID = 0x07;


void setup() {

//setting to pins D21 and D22 on ESP32 to match pinlayout SN65HVD235D chip
  CAN.setPins(22, 21);
  
  Serial.begin(9600);
  while (!Serial);

  Serial.println("CAN Receiver");

  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }
}

void loop() {
  // try to parse packet
  int packetSize = CAN.parsePacket();

  //only start receiving date if both packetsize and ID is valid ( 1< )
  if (packetSize || CAN.packetId() != -1) {
    
    // received a packet
    Serial.print("Received ");

    if (CAN.packetExtended()) {
      Serial.print("extended ");
    }

    if (CAN.packetRtr()) {
      // Remote transmission request, packet contains no data
      Serial.print("RTR ");
    }

    Serial.print("packet with id 0x");
    Serial.print(CAN.packetId(), HEX);

    if (CAN.packetRtr()) {
      Serial.print(" and requested length ");
      Serial.println(CAN.packetDlc());
    } else {
      Serial.print(" and length ");
      Serial.println(packetSize);

      // only print packet data for non-RTR packets
      while (CAN.available()) {
        Serial.print(CAN.read());
      }
      Serial.println();
    }

    Serial.println();
  }
  delay(1000);
}
