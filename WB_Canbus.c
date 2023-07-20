//  Canbus communications library
//  2019 Sepehr Ahmadi

/* This protocol was developed simply to provide a simplistic layer 2 destination-based 
 *  messaging protocol on top of the pre-existing CANBUS protocol. There are many oddities
 *  involved, most of them are to reduce packet overhead - as canbus does not provide much
 *  room for data - while maintaining stability. 
 *  
 * This protocol features multi-frame packets with CRC checking, 
 *   as well as address-based filtering so that you only receive packets meant for you. 
 */

#include <ESP32CAN.h>
#include <CAN_config.h>

#include <can_regdef.h>

struct WBCAN_packet_t {
  byte priority;
  byte to;
  byte sequence;
  byte payload_size;
  uint16_t crc;
  byte payload[256];
};

/**
 * This protocol is based WITHIN the CAN protocol, 
 *   This is why the frames are exactly CAN frames, with a specific id&data format.
 *   Unmodified parts of the frame are in |#|, to emphasize our spec 
 *   while maintaining sizes.
 *    
 * Multi-Packet: Initial Frame Struct (pkt_type==0):
 * ----------------------------------------------------------------------------
 * |#|  3 bit   |  4 bit  | 4 bit |#|   2 bit  |   6 bit  | 1 byte | 6 byte |#|
 * |#| priority | to addr | seq # |#| pkt type | pkts rem | frmlen |  data  |#|
 * ----------------------------------------------------------------------------
 * 
 * Multi-Packet: Middle Frame Struct (pkt_type==1):
 * ---------------------------------------------------------------------------
 * |#|  3 bit   |  4 bit  | 4 bit |CAN|   2 bit  |      6 bit     | 7 byte |#|
 * |#| priority | to addr | seq # |DAT| pkt type | pkts remaining |  data  |#|
 * ---------------------------------------------------------------------------
 * 
 * Multi-Packet: Terminating Frame Struct (pkt_type== 2):
 * ----------------------------------------------------------------------------------
 * |#|  3 bit   |  4 bit  | 4 bit |#|     2 bit   |    6 bit   | 5 byte |  2 byte |#|
 * |#| priority | to addr | seq # |#| pkt_type=10 |  bruh 0    |  data  | frm crc |#|
 * ----------------------------------------------------------------------------------
 * 
 * Single-Packet: Packet in Frame Struct (pkt_type== 3):
 * ---------------------------------------------------------------------
 * |#|  3 bit   |  4 bit  | 4 bit |#|     2 bit   |  6 bit  | 7 byte |#|
 * |#| priority | to addr | seq # |#| pkt_type=11 | pktsize |  data  |#|
 * ---------------------------------------------------------------------
 */


 /*
  * As we receive data, *DATA* bytes (and only data bytes) are passed to a CRC calc, 
  *   to ensure data is unscathed.
  * 
  * Checks and explanations:
  * 
  * if pkt type == 0, this is a new frame, 
  *                   throw away old frame and construct new based on this 
  *   data[1] = entire frame size in bytes
  * 
  * if pkt_type == 1, this is a data packet, all is good.
  *  
  * if pkt_type == 2, this is the end of the frame,
  *   check: data[6] == crc[0] and data[7] == crc[1]
  *   
  * if pkt_type == 3, this is a single packet frame
  *   no checks required, this is the entire frame, and packets are CRC'd
  */

// Because of how arbitration works, and how we have setup our SEND flow,
//  only one packet can be on the line at once. Hence we only need one temp.
WBCAN_packet_t cur_recv; 

CAN_device_t CAN_cfg;

byte my_id;
byte tx_seq_counter;
byte rx_pkt_sum;

void init_canbus(byte id, gpio_num_t txpin, gpio_num_t rxpin){
  my_id = id;
  tx_seq_counter = 0b0000;
  
  CAN_cfg.speed=CAN_SPEED_1000KBPS;
  CAN_cfg.tx_pin_id = txpin;
  CAN_cfg.rx_pin_id = rxpin;
  CAN_cfg.rx_queue = xQueueCreate(10,sizeof(CAN_frame_t));
  
  Serial.println("Canbus Init");
  Serial.println(ESP32Can.CANInit());
}

uint16_t _crc16_update(uint16_t crc, uint8_t a)
{
  int i;
  crc ^= a;
  for (i = 0; i < 8; ++i)
  {
    if (crc & 1)crc = (crc >> 1) ^ 0xA001;
    else crc = (crc >> 1);
  }
  return crc;
}

boolean recv_canbus(WBCAN_packet_t *t){
  CAN_frame_t rx_frame;
  while(xQueueReceive(CAN_cfg.rx_queue, &rx_frame, 3*portTICK_PERIOD_MS)==pdTRUE){
    if(rx_frame.FIR.B.FF != CAN_frame_std){ 
      // This library only sends standard packets.
      //   this is a misplaced packet, ignore it.
      continue; 
    }
    
    byte id = rx_frame.MsgID; // assuming id is 11 bits
    
    byte prio = id>>8; // bits 8 to 10
    byte dest = (id>>4) & 0b1111; // bits 4 to 7
    byte seq  = id & 0b1111; // bits 0 to 3
    Serial.printf("id:%d => pri:%d | dest:%d | seq:%d\n",id,prio,dest,seq);

    byte pkt_meta = rx_frame.data.u8[0]; 

    // first 2 bits (MSB) = packet type
    byte pkt_type = pkt_meta>>6;
    // last 6 bits (MSB) = how many future packets are coming
    byte next_count = pkt_meta & 0b11111;
    Serial.printf("meta:%d => pkt:%d | nc:%d\n",pkt_meta,pkt_type,next_count);

    if(pkt_type == 3){
      // dont need to check CRC for single packet, that's already part of CANBUS.
      
      cur_recv = {};
      cur_recv.to = dest;
      cur_recv.sequence = seq;
      cur_recv.priority = prio;
      cur_recv.payload_size = next_count;
      for(byte i=0;i<7;i++){
         cur_recv.payload[i] = rx_frame.data.u8[1+i];
      }
      *t = cur_recv;
      return true;
    }

    if(pkt_type == 0){
      cur_recv = {};
      cur_recv.to = dest;
      cur_recv.crc = 0;
      cur_recv.sequence = seq;
      cur_recv.priority = prio;
      cur_recv.payload_size = rx_frame.data.u8[1];
      rx_pkt_sum = 0;
    }else{
      if(&cur_recv == NULL){
        continue;
      }

      if(cur_recv.sequence != seq){
        Serial.printf("Received pkt t:%d with seq%d but we are on seq%d\n",pkt_type,seq,cur_recv.sequence);
        continue;
      }
    }

    byte data_starts = 1; // t=1 and t=2 start data at data[1]
    if(pkt_type == 0){
      data_starts = 2; // t=0 starts at data[2]
    }

    byte data_bytes;
    if(pkt_type == 0){
      data_bytes = 6;
    }else if(pkt_type == 1){
      data_bytes = 7;
    }else{
      data_bytes = 5;
    }
    
    for(byte i=0;i<data_bytes;i++){
        if(rx_pkt_sum>cur_recv.payload_size){
          break;
        }
        cur_recv.crc = _crc16_update(cur_recv.crc, rx_frame.data.u8[i+data_starts]);
        cur_recv.payload[rx_pkt_sum]=rx_frame.data.u8[i+data_starts];
        rx_pkt_sum += 1;

        Serial.printf("%x %x %x\n",cur_recv.payload[rx_pkt_sum-1],(cur_recv.crc>>8),(cur_recv.crc&0b11111111));
        
    }
    
    if(pkt_type == 2){
      Serial.printf("%x=%x %x=%x",(cur_recv.crc>>8) ,rx_frame.data.u8[6] ,(cur_recv.crc&0b11111111),rx_frame.data.u8[7] );
      if((cur_recv.crc>>8) == rx_frame.data.u8[6] 
      && (cur_recv.crc&0b11111111) == rx_frame.data.u8[7]){
        *t = cur_recv;
        return true;
      }
      return false;
    }
  }
  return false;
}

void send_packet(CAN_frame_t *rx_frame){
  Serial.printf("Sending pkt :\n");
  for(int i=0;i<8;i++){
    Serial.printf("%c [%x]\n",rx_frame->data.u8[i],rx_frame->data.u8[i]);
  }
  Serial.printf("\n");
  ESP32Can.CANWriteFrame(rx_frame);
}

/**
 * Send a message on the canbus, to a recipient (or all), with a given payload.
 * Payload size can be 256 bytes.
 */
boolean send_canbus(byte priority, byte to, byte payload_size, byte* payload){
  // 'to' and 'seq' are both sent in 'msgid', clients ignore any 'to' that is not them.
  CAN_frame_t rx_frame;
  rx_frame.FIR.B.FF = CAN_frame_std;
  rx_frame.FIR.B.DLC = 8;
  rx_frame.MsgID = (priority&0b111) << 8 | (to & 0b1111) << 4 | (tx_seq_counter&0b1111);

  Serial.printf("!!! %d | %d | %d\n", priority, to, payload_size);

  if(payload_size <= 7){ 
    // it fits in a single packet. (pkt_type=3)

    // it's already assured that payload_size<0b1111, no need to force.
    rx_frame.data.u8[0] = 0b11000000 | payload_size; 

    for(int i=0;i<payload_size;i++){
      rx_frame.data.u8[1+i] = payload[i];
    }
    
    send_packet(&rx_frame);
  }else{
    // it needs to be fragmented.

    // send the init packet (type = 0)
    int data_offset = 0;

    uint16_t crc = 0;

    int num_packets = 2;
    int size_rem = payload_size - (5+6); // first 6 and last 5 packet MUST be there.
    // the remaining size/7 is how many people.

    float cbt = size_rem / 7.0 + 1.0;
    int middle_packets = (int)(cbt);

    num_packets += middle_packets;
    Serial.printf("Packet of size %d has %d packets\n",payload_size,num_packets);

    num_packets -= 1;
    //frame opener
    rx_frame.data.u8[0] = num_packets&0b111111; // 0b00<<6 + numpkts & 0b111111 
    rx_frame.data.u8[1] = payload_size; 

    for(int i=0;i<6;i++){
      crc= _crc16_update (crc, payload[data_offset]);
      Serial.printf("%x %x %x\n",payload[data_offset],(crc>>8),(crc&0b11111111));
        
      rx_frame.data.u8[i+2] = payload[data_offset];
      data_offset+=1;
    }

    send_packet(&rx_frame);

    for(int i=0;i<middle_packets;i++){
      num_packets -= 1;
      rx_frame.data.u8[0] = 0b01000000 | num_packets&0b111111;

      for(int i=0;i<7;i++){ 
        if(data_offset > payload_size){
          rx_frame.data.u8[i+1] = 0;
        }else{
          crc= _crc16_update (crc, payload[data_offset]);
          Serial.printf("%x %x %x\n",payload[data_offset],(crc>>8),(crc&0b11111111));
          rx_frame.data.u8[i+1] = payload[data_offset];
          data_offset+=1;
        }
      }
      
      send_packet(&rx_frame);
    }

    //frame closer
    rx_frame.data.u8[0] = 0b10000000; // 0b10<<6 + 0 & 0b111111 

    for(int i=0;i<5;i++){
      if(data_offset > payload_size){
        rx_frame.data.u8[i+1] = 0;
      }else{
        crc= _crc16_update (crc, payload[data_offset]);
        Serial.printf("%x %x %x\n",payload[data_offset],(crc>>8),(crc&0b11111111));
        rx_frame.data.u8[i+1] = payload[data_offset];
        data_offset+=1;
      }
    }

    rx_frame.data.u8[6] = crc>>8;
    rx_frame.data.u8[7] = crc&0b11111111;

    send_packet(&rx_frame);
  }
  
  tx_seq_counter += 1;
  if(tx_seq_counter > 0b1111){
    tx_seq_counter = 0;
  }
  return true;
}
