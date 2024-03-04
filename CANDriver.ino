#include <Arduino.h>

#include <WB_Canbus.h>

bool sent = false;

const uint16_t DEVICE_ID = 0x00;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
 // Initialize the CAN bus at your desired speed
  
  init_canbus(0,GPIO_NUM_21,GPIO_NUM_22);

}

void loop() {
  if (Serial.available()) {
    String serialData = Serial.readStringUntil('\n');

    if (serialData.length() > 2 && serialData.charAt(1) == '|') {
      char deviceId = serialData.charAt(0);
      String action = serialData.substring(2);
      
      // Handling the kill switch command
      if (action.equals("KILL")) {
        char killMsg[] = "KILL";
        send_canbus(0, 0xFF, strlen(killMsg), (byte*)killMsg); // Broadcast KILL command
        Serial.println("Kill signal sent to all devices.");
        return; // Exit the loop after sending kill command
      }

      byte msg[256];
      action.getBytes(msg, 256);
      int msgsize = action.length()

      uint16_t charToDeviceCANID(char deviceChar) {
        switch (deviceChar) {
            case '0': return 0x00;
            case '1': return 0x01;
            case '2': return 0x02;
            case '3': return 0x03;
            case '4': return 0x04;
            case '5': return 0x05;
            case '6': return 0x06;
            case '7': return 0x07;
            case '8': return 0x08;
            case '9': return 0x09;
            case '10': return 0x0A;
            default: return 0x0B; // Unknown device
        }
      }
      

    
//#TODO Add some basic commands, finalize send/receive functions, test running time, collisions, sensor every 0.5s????

          



      // Assuming deviceId maps to the 'to' field in CAN packet and using a fixed priority and sequence for simplicity
      
      //byte priority, byte to, byte payload_size, byte* payload
      uint16_t canId = charToDeviceCANID(deviceChar);
      send_canbus(0, canId, msgsize, (byte*)msg); // Send the CAN message
      Serial.println("Serial data converted to CAN message and sent successfully");
     else {
      Serial.println("Invalid serial data format");
    }
  }}

  // Receiving part
  WBCAN_packet_t rx_pkt;
  if (recv_canbus(&rx_pkt)) {
    Serial.print("Received pkt {pri: ");
    Serial.print(rx_pkt.priority);
    Serial.print(", seq: ");
    Serial.print(rx_pkt.sequence);
    Serial.print(", to: ");
    Serial.print(rx_pkt.to);
    Serial.print(", len: ");
    Serial.print(rx_pkt.payload_size);
    Serial.print("}\nData: '");
    for (int i = 0; i < rx_pkt.payload_size; i++) {
      Serial.print((char)rx_pkt.payload[i]);
    }
    Serial.println("'");
  }
}
