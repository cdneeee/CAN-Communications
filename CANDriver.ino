#include <Arduino.h>

#include <WB_Canbus.h>

bool sent = false;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  
  init_canbus(0,GPIO_NUM_21,GPIO_NUM_22);
}

void loop() {

  if(Serial.available() > 0){
    String msgs = Serial.readString();
    char msg[256];
    msgs.toCharArray(msg,256);
    int msgsize = strlen(msg);
    send_canbus(0,0,msgsize,(byte*)msg);
    sent = true;
//    Serial.println("sent");
  }
  
  WBCAN_packet_t rx_pkt;
  // put your main code here, to run repeatedly:
  if(recv_canbus(&rx_pkt) == true){
      Serial.printf("Received pkt {pri:%d,seq:%d,to:%d,len:%d}\nData: '",rx_pkt.priority,rx_pkt.sequence,rx_pkt.to,rx_pkt.payload_size);
      for(int i=0;i<rx_pkt.payload_size;i++){
        Serial.printf("%c",rx_pkt.payload[i]);
      }
      Serial.printf("'\n");
  }
}