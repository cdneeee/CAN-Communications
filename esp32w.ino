

//Western Engineering RaB1 base code
//2019 E J Porter
//WEB Server AP 

// IP adress 192.168.128.1


/*
esp32
pins         description                        used
1             3v3                               PWR 3V3
2             gnd                               GND
3             GPIO15/AD2_3/T3/SD_CMD/               
4             GPIO2/AD2_2/T2/SD_D0              INDICATORLED
5             GPIO4/AD2_0/T0/SD_D1              
6             GPIO16/RX2
7             GPIO17/TX2
8             GPIO5                             ULTRSONIC TRIG
9             GPIO18                            ULTRASONIC ECHO
10            GPIO19/CTS0
11            GPIO21                            MPU6050 CLK              
12            GPIO3/RX0                         
13            GPIO1//TX0                        
14            GPIO22/RTS1                       MPU6050 SDA
15            GPIO23
16            EN
17            GPI36/VP/AD1_0                    ENCODER ERB
18            GPI39/VN/AD1_3/                   
19            GPI34/AD1_6/                      
20            GPI35/AD1_7                       ENCODER ELA
21            GPIO32/AD1_4/T9                   
22            GPIO33/AD1_5/T8                   
23            GPIO25/AD2_8/DAC1                 
24            GPIO26/A2_9/DAC2                  
25            GPIO27/AD2_7/T7                   MOTORRIGHTb
26            GPOP14/AD2_6/T6/SD_CLK            MOTORRIGHTa
27            GPIO12/AD2_5/T5/SD_D2/            MOTORLEFTb
28            GPIO13/AD2_4/T4/SD_D3/            MOTORLEFTa
29            GND                               GND
30            VIN                               PWR >= 5v

*/



//pins
#define INDICATORLED 2
#define MOTORLEFTa 13
#define MOTORLEFTb 12
#define MOTORRIGHTa 14
#define MOTORRIGHTb 27
#define MPU6050_SDA 22
#define MPU6050_SCL 21
#define ENCODER_ERB 36
#define ENCODER_ELA 35
#define ULTRASONIC_TRIGGER 5
#define ULTRASONIC_ECHO 18

#define LED1pin 2
#define LED2pin 3 

#include <Math.h>
#include "MyWEBserver.h"
#include <Servo.h>



int loopWEBServerButtonresponce(void);


char bToggleBit;
const int ciMainTimer =  200;//200;

unsigned int uiCommunticationTimer;

unsigned long ulPreviousMicros;
unsigned long ulCurrentMicros;

int bnState;
int bnStateOriginal;

//*****************************************************
// Claire's Set up
int steeringAngle = 70; // 70 is center. range is 40 to 110.
int carspeed = 90; // 90 is stop. Forwards is 85. Reverse is 95. 
Servo driveServo;
Servo turnServo;
unsigned long carlooptimer;
bool flag = false;

int curangle;

void setup() {
  Serial.begin(115200);
  pinMode(INDICATORLED, OUTPUT);
  
  driveServo.attach(13);  // attaches the servo on pin 13 to the servo object
  turnServo.attach(15);  // attaches the servo on pin 13 to the servo object

   setupWEbServer();

   //initialize variable
   bnStateOriginal = 0;
   bnState = 0;
   curangle = 70;

    //set servos
  driveServo.write(90);
  turnServo.write(70);
  delay(7000); //upon power up the drive servo needs to be at 90 for a period of time otherwise it enters program mode

  carlooptimer = micros();
 
}
void loop()
{

   //main timing loop enters if every ~200us (ciMainTimer). this loop time controls all other timers
  ulCurrentMicros = micros();
  if ((ulCurrentMicros - ulPreviousMicros) >= ciMainTimer)
  {
      ulPreviousMicros = ulCurrentMicros;

      //increment program timers
    
      uiCommunticationTimer = uiCommunticationTimer + 1;
      bToggleBit = bToggleBit + 1;
      if(bToggleBit&1)
      {
        digitalWrite(INDICATORLED, HIGH);
      }
      else
      {    
        digitalWrite(INDICATORLED, LOW);
      }
    
      bnStateOriginal = loopWEBServerButtonresponce();
      if (bnStateOriginal!=9){ //This is because the page refreshes and for some reason doesn't keep the button pressed state.
        bnState = bnStateOriginal;
      }

      Serial.print("Bn state in 200us loop ");
      Serial.println(bnState);
    }//end of code that occurs every 200us


//    carlooptimer = millis();
//    if ((carlooptimer % 100)>98){
//      if(flag == false){
//      Serial.print("Bn state");
//      Serial.println(bnState);
//      flag = true;
//      }
//      
//    }
//    else{flag = false;}

      
  
    //YOUR CODE HERE
    //Buttons: 0 == stop, 1 == forward, 2 == left, 3 == right, 4 == reverse
//     Serial.println(bnState); //check what is entering the case statement
//     -possible need to convert to ascii - bnstat may be string
//     -bnstate is probably being cleared somewhere
    
    switch(bnState){
      case 0:
        Serial.println("Case0");
        //driveServo.write(90);
        break;
      case 1: 
        Serial.println("Case1");
        break;
      case 2: //max 110
        Serial.println("Case2");
        if(curangle<100 && ulCurrentMicros%2000<100){
          curangle += 5;
        }
        turnServo.write(curangle);
        break;
      case 3: //min 40
        Serial.println("Case3");
        if(curangle>50 && ulCurrentMicros%2000<100){
          curangle -= 5;
        }
        turnServo.write(curangle);
        break;
      case 4:
      Serial.println("Case4");
      break;
    }//end of case
    Serial.print("Steering Angle: ");
    Serial.println(curangle);

}// end of loop
