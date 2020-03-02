
#include <NewPing.h>
#include <Servo.h>
Servo myservo; 
int pos = 0;   
int Distance=0;
int Angle=0; 

#define TRIGGER_PIN  A1 
#define ECHO_PIN     A0  
#define MAX_DISTANCE 200 
NewPing sonar(TRIGGER_PIN, ECHO_PIN, MAX_DISTANCE); 

int motorLpin1=5;
int motorLpin2=7;
int motorRpin1=8;
int motorRpin2=10;
int motorLpwm=6;
int motorRpwm=9;
int motorSpeed=250;

const int hmc5883Address = 0x1E; //0011110b, I2C 7bit address for compass
const byte hmc5883ModeRegister = 0x02;
const byte hmcContinuousMode = 0x00;
const byte hmcDataOutputXMSBAddress = 0x03;

int x,y,z; //triple axis data from HMC5883L.
#include "I2Cdev.h"
#include "Wire.h"
#include "MPU6050.h"
MPU6050 accelgyro;
int currentReading, lastReading;
int readCompass();

int initialorient , currentorient, alreadyroatated, angleorient;

void setup() { 

  myservo.attach(11);// attaches the servo on pin 9 to the servo object
  Serial.begin(9600);

  Serial.begin(9600);
   pinMode(motorLpin1,OUTPUT);
  pinMode(motorLpin2,OUTPUT);
  pinMode(motorRpin1,OUTPUT);
  pinMode(motorRpin2,OUTPUT);
  pinMode(motorLpwm,OUTPUT);
  pinMode(motorRpwm,OUTPUT);

  Wire.begin();
  accelgyro.initialize();
  accelgyro.setI2CBypassEnabled(true);
   Wire.beginTransmission(hmc5883Address);  //Begin communication with compass
    Wire.write(hmc5883ModeRegister);  //select the mode register
    Wire.write(hmcContinuousMode); //continuous measurement mode
    Wire.endTransmission();
}

void loop() {

//Rotate and calculate Distance and Angle of pole
 int pos = 0;    
 for (pos = 0; pos <= 180; pos += 10)
 {
   myservo.write(pos);  
   Serial.print(pos);
   delay(50);
   unsigned int uS = sonar.ping(); 
   Serial.print("  ");  Serial.println(" cm");
   Serial.print(uS / US_ROUNDTRIP_CM);
   if ( (uS / US_ROUNDTRIP_CM)<=150) {
    Angle=pos; 
    Distance= uS / US_ROUNDTRIP_CM;
    }  
    delay(800);                
 }

  angleorient=(90-Angle);
  stoprobot();  delay(300);
  initialorient= readCompass();
 

if( angleorient>0){   
     currentorient=readCompass();
     if(initialorient >= (360-angleorient)){
         currentorient=readCompass();
         while(abs(initialorient-currentorient)<abs(365-initialorient))  {       
            rotateanticlock();
            alreadyroatated=abs(initialorient-currentorient);
            delay(20);
            currentorient=readCompass();
          }
          stoprobot();    delay(300);
          currentorient=readCompass();
          while(abs(currentorient)<abs(angleorient-alreadyroatated-5)){
             rotateanticlock();
             currentorient=readCompass();
          }    
     }
     else {
          while(abs(initialorient-currentorient)<angleorient){
          rotateanticlock(); delay(20);
          currentorient=readCompass();
          }
     }
  }



if( angleorient<0){   
    currentorient=readCompass();  
    if(initialorient > abs(angleorient)){
          while(abs(initialorient-currentorient)<abs(angleorient)){
              rotateclock(); delay(20);
              currentorient=readCompass();
          }
     }
     else {    
         currentorient=readCompass(); initialorient=readCompass();     
         while(abs(initialorient-currentorient)<abs(initialorient+5)) {       
              rotateclock();       delay(20);
              alreadyroatated=abs(initialorient-currentorient);
              currentorient=readCompass();
        }
         stoprobot();    delay(100);
         currentorient=readCompass();
         initialorient=readCompass();
         while(abs(initialorient-currentorient)-5<abs(abs(angleorient)-alreadyroatated)){
               rotateclock(); delay(20);
               currentorient=readCompass();
         }    
     }
 }


 else{
  moveforward();
 }

  while(Distance>5)
  {
  moveforward();
  delay(50);
  }
 
}




  int readCompass()
{
   Wire.beginTransmission(hmc5883Address);
    Wire.write(hmcDataOutputXMSBAddress);  //Select register 3, X MSB register
    Wire.endTransmission();

    //Read data from each axis of the Digital Compass
    Wire.requestFrom(hmc5883Address,6);
    if(6<=Wire.available())
    {
      x = Wire.read()<<8; //X msb
      x |= Wire.read();   //X lsb
      z = Wire.read()<<8; //Z msb
      z |= Wire.read();   //Z lsb
      y = Wire.read()<<8; //Y msb
      y |= Wire.read();   //Y lsb    
    }
    int heading = atan2(-y,x)/M_PI*180;
  if(heading < 0)
    heading += 360;
  if(heading > 360)
    heading -= 360;   
return heading;
}


void moveforward()
{
  analogWrite(motorLpwm,motorSpeed);
   analogWrite(motorRpwm,motorSpeed);
  digitalWrite(motorLpin1,1);
  digitalWrite(motorLpin2,0);
  digitalWrite(motorRpin1,1);
  digitalWrite(motorRpin2,0);
}

void rotateclock(){
 analogWrite(motorLpwm,motorSpeed);
  analogWrite(motorRpwm,motorSpeed);
  digitalWrite(motorLpin1,1);
  digitalWrite(motorLpin2,0);
  digitalWrite(motorRpin1,0);
  digitalWrite(motorRpin2,1);
}

void rotateanticlock(){
  analogWrite(motorLpwm,motorSpeed);
 analogWrite(motorRpwm,motorSpeed);
 digitalWrite(motorLpin1,0);
  digitalWrite(motorLpin2,1);
  digitalWrite(motorRpin1,1);
 digitalWrite(motorRpin2,0);
}

void stoprobot(){
analogWrite(motorLpwm,0);
  analogWrite(motorRpwm,0);
  digitalWrite(motorLpin1,1);
  digitalWrite(motorLpin2,1);
  digitalWrite(motorRpin1,1);
  digitalWrite(motorRpin2,1);
}
