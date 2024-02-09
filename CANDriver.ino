#include <Arduino.h>

#include <WB_Canbus.h>

bool sent = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
 // Initialize the CAN bus at your desired speed
  
  init_canbus(0,GPIO_NUM_21,GPIO_NUM_22);
  int priority;

}

void loop() {
  // Sending part
  if (Serial.available()) {
    String serialData = Serial.readStringUntil('\n'); // Read the entire line until a newline character

    // Check if the received data has the correct format "A|do this"
    if (serialData.length() > 2 && serialData.charAt(1) == '|') {
      char deviceId = serialData.charAt(0);
      String action = serialData.substring(2);
      char msg[256];
      action.toCharArray(msg, 256);
      int msgsize = strlen(msg);
      switch (deviceId):
        case "K":
          priority = 0;
        case "A":
          priority = 1;
          break;
        case "B";
    
//#TODO implement the killswitch and the queue, make all of them same priority

          



      // Assuming deviceId maps to the 'to' field in CAN packet and using a fixed priority and sequence for simplicity
      //byte priority, byte to, byte payload_size, byte* payload
      send_canbus(0, int(deviceId), msgsize, (byte*)msg); // Send the CAN message
      Serial.println("Serial data converted to CAN message and sent successfully");
    } else {
      Serial.println("Invalid serial data format");
    }
  }

  // Receiving part
  WBCAN_packet_t rx_pkt;
  if (recv_canbus(&rx_pkt) == true) {
    Serial.printf("Received pkt {pri:%d,seq:%d,to:%d,len:%d}\nData: '", rx_pkt.priority, rx_pkt.sequence, rx_pkt.to, rx_pkt.payload_size);
    for (int i = 0; i < rx_pkt.payload_size; i++) {
      Serial.printf("%c", rx_pkt.payload[i]);
    }
    Serial.println("'");
  }
}
