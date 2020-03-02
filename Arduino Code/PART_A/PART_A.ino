#include "TinyGPS++.h"   // library for GPS for NMEA Parsing
#include "SoftwareSerial.h"
SoftwareSerial serial_connection(4, 3); //  pin 4 on arduino is acting rx//// connect tx of gps to pin 4
TinyGPSPlus gps;

#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

const int hmc5883Address = 0x1E; //0011110b, I2C 7bit address for compass
const byte hmc5883ModeRegister = 0x02;
const byte hmcContinuousMode = 0x00;
const byte hmcDataOutputXMSBAddress = 0x03;

int LEDPin = 13;
bool blinkState = false;
int x,y,z; //triple axis data from HMC5883L.
int TargetDistance,TargetHeading, CurrentHeading, HeadingDifference;

//int waypoint_num = -1;
//double lat_array[7] = {23.17725181, 39.53869, 39.53842, 39.53820, 39.53842, 39.53869, 39.53862};
//double lon_array[7] = {80.01971435, -105.01678, -105.01649, -105.01672, -105.01649, -105.01678, -105.01686};
//const int total_wps = 6;

#define motorLpin1  10
#define motorLpwm  9
#define motorLpin2 8
#define motorRpin1  7
#define motorRpwm  6
#define motorRpin2  5

void setup()
{
  Serial.begin(9600);
  serial_connection.begin(9600);
  Serial.println("GPS Start");

////setup for gy87
 Wire.begin();
 accelgyro.initialize();
 accelgyro.setI2CBypassEnabled(true);  //This sets the bypass so the HMC5883L gets a look in.

   Wire.beginTransmission(hmc5883Address);  //Begin communication with compass
   Wire.write(hmc5883ModeRegister);  //select the mode register
   Wire.write(hmcContinuousMode); //continuous measurement mode
   Wire.endTransmission();

   pinMode(LEDPin, OUTPUT);
   delay(1000);

  pinMode(motorLpin1,OUTPUT);
  pinMode(motorLpin2,OUTPUT);
  pinMode(motorRpin1,OUTPUT);
  pinMode(motorRpin2,OUTPUT);
  pinMode(motorLpwm,OUTPUT);
  pinMode(motorRpwm,OUTPUT);
  
    digitalWrite(motorRpin1,LOW);      
    digitalWrite(motorRpin2,LOW); 
    digitalWrite(motorLpin1,LOW);       
    digitalWrite(motorLpin2,LOW); 
    }


void loop() {
  while(serial_connection.available())  // waiting for GPS to get stable data
  { gps.encode(serial_connection.read()); }  
   
  if(gps.location.isUpdated()){
 // Calculating the Target Diatstance using GPS data
  TargetDistance=distanceToWaypoint(gps.location.lng(), gps.location.lat(), 80.026000,23.178585);
  if (TargetDistance<=2) {
     stoprobot();
     delay(10000);}
     
 //Calculating the Target Heading using GPS data
  TargetHeading=courseToWaypoint(gps.location.lng(), gps.location.lat(), 80.026000,23.178585);
  
 //Calculating the Current Heading using Magnetometer data
  CurrentHeading=readCompass();

//Claculating Heading Error of Robot  
   HeadingDifference=calcDesiredTurn(TargetHeading,CurrentHeading);

//Choosing Left or Right Turn on basis of positive or negative Heading Error
  motorSpeed(HeadingDifference);  delay(500); 
  }
}  

  int distanceToWaypoint(float choose_currentLong,float choose_currentLat, float choose_targetLong, float choose_targetLat)
  {
  float currentLong = choose_currentLong;
  float currentLat=choose_currentLat; 
  float targetLong = choose_targetLong;
  float targetLat=choose_targetLat;
  float delta = radians(currentLong - targetLong);
  float sdlong = sin(delta);
  float cdlong = cos(delta);
  float lat1 = radians(currentLat);
  float lat2 = radians(targetLat);
  float slat1 = sin(lat1);
  float clat1 = cos(lat1);
  float slat2 = sin(lat2);
  float clat2 = cos(lat2);
  delta = (clat1 * slat2) - (slat1 * clat2 * cdlong); 
  delta = sq(delta); 
  delta += sq(clat2 * sdlong); 
  delta = sqrt(delta); 
  float denom = (slat1 * slat2) + (clat1 * clat2 * cdlong); 
  delta = atan2(delta, denom); 
  int distanceToTarget =  delta * 6372795;
  return distanceToTarget;
  }
     
  int courseToWaypoint(float choose_currentLong,float choose_currentLat, float choose_targetLong, float choose_targetLat){
  float currentLong = choose_currentLong;
  float currentLat=choose_currentLat;
  
  float targetLong = choose_targetLong;
  float targetLat=choose_targetLat;
  
  float dlon = radians(targetLong-currentLong);
  float cLat = radians(currentLat);
  float tLat = radians(targetLat);
  float a1 = sin(dlon) * cos(tLat);
  float a2 = sin(cLat) * cos(tLat) * cos(dlon);
  a2 = cos(cLat) * sin(tLat) - a2;
  a2 = atan2(a1, a2);
  if (a2 < 0.0)
  {
    a2 += TWO_PI;
  }
  int targetHeading = degrees(a2);
  return targetHeading;
 }


 
   int readCompass(){
    Wire.beginTransmission(hmc5883Address);
    Wire.write(hmcDataOutputXMSBAddress); 
    Wire.endTransmission();
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
    int heading = atan2(y,x)/M_PI*180;
      
  if(heading < 0)
    heading += 360;   
  if(heading > 360)
    heading -= 360;
 
  return((int)heading); 
   }



 int  calcDesiredTurn(int choose_targetHeading, int choose_heading){

  float targetHeading=choose_targetHeading;
  float heading=choose_heading;
 
   int headingError = targetHeading - heading;   
   headingError=headingError;
   
    if (headingError < -180)      
      headingError += 360;
    if (headingError > 180)
      headingError -= 360;
    delay(100);

    return(headingError);
 }



void motorSpeed( int choose_error){
int  TurningError=choose_error;
  
  if(TurningError>=5 &&  TurningError<=10){
  analogWrite(motorLpwm,180);
  analogWrite(motorRpwm,50);
  digitalWrite(motorLpin1,1);
  digitalWrite(motorLpin2,0);
  digitalWrite(motorRpin1,1);
  digitalWrite(motorRpin2,0);
  delay(1000);
  }
  else if(TurningError>= 11 &&  TurningError<=60){
  analogWrite(motorLpwm,255);
  analogWrite(motorRpwm,50);
  digitalWrite(motorLpin1,1);
  digitalWrite(motorLpin2,0);
  digitalWrite(motorRpin1,1);
  digitalWrite(motorRpin2,0);
  delay(1000);
  }


  else if(TurningError>= -10 &&  TurningError<=-5){
  analogWrite(motorLpwm,50);
  analogWrite(motorRpwm,180);
  digitalWrite(motorLpin1,1);
  digitalWrite(motorLpin2,0);
  digitalWrite(motorRpin1,1);
  digitalWrite(motorRpin2,0);
  delay(1000);
  }
  else if(TurningError>= -60 &&  TurningError<=-11 ){
  analogWrite(motorLpwm,50);
  analogWrite(motorRpwm,255);
  digitalWrite(motorLpin1,1);
  digitalWrite(motorLpin2,0);
  digitalWrite(motorRpin1,1);
  digitalWrite(motorRpin2,0);
  delay(1000);  
  }


 else if(TurningError<= -60){
  analogWrite(motorLpwm,255); //anticlockwise
  analogWrite(motorRpwm,255);
  digitalWrite(motorLpin1,0);
  digitalWrite(motorLpin2,1);
  digitalWrite(motorRpin1,1);
  digitalWrite(motorRpin2,0);
  delay(1000);
  }
  else if(TurningError>= 60){
  analogWrite(motorLpwm,255); //clockwise
  analogWrite(motorRpwm,255);
  digitalWrite(motorLpin1,1);
  digitalWrite(motorLpin2,0);
  digitalWrite(motorRpin1,0);
  digitalWrite(motorRpin2,1);
  delay(1000);
  }

  else if(TurningError<5 && TurningError > -5)
  {  //go forward
  analogWrite(motorLpwm,255);
  analogWrite(motorRpwm,255);
  digitalWrite(motorLpin1,1);
  digitalWrite(motorLpin2,0);
  digitalWrite(motorRpin1,1);
  digitalWrite(motorRpin2,0);
    Serial.println("Left motor speed 255"); 
    Serial.println("Right motor speed 255"); 
  delay(1000);
 }

}



void stoprobot(){
analogWrite(motorLpwm,0);
  analogWrite(motorRpwm,0);
  digitalWrite(motorLpin1,1);
  digitalWrite(motorLpin2,1);
  digitalWrite(motorRpin1,1);
  digitalWrite(motorRpin2,1);
}

