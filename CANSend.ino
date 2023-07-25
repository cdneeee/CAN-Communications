#include <CAN.h>


int sendingInt = 254;


void setup() {

//setting to pins D21 and D22 on ESP32 to match pinlayout SN65HVD235D chip
  CAN.setPins(22, 21);
  
  Serial.begin(9600);
  while (!Serial);

  Serial.println("CAN Sender");

  // start the CAN bus at 500 kbps
  if (!CAN.begin(500E3)) {
    Serial.println("Starting CAN failed!");
    while (1);
  }

  
}

void loop() {
  // send packet: id is 11 bits, packet can contain up to 8 bytes of data
  Serial.print("Sending packet ... ");


  CAN.beginPacket(0x12);
  CAN.write(sendingInt);
  CAN.endPacket( );


  Serial.println("done");

  delay(1000);

  // send extended packet: id is 29 bits, packet can contain up to 8 bytes of data
  Serial.print("Sending extended packet ... ");

  CAN.beginExtendedPacket(0xabcdef);
//  CAN.write('w');
//  CAN.write('o');
//  CAN.write('r');
//  CAN.write('l');
//  CAN.write('d');
  CAN.endPacket();

  Serial.println("done");

  delay(1000);
}
