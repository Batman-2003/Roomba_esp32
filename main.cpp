
/* 
     Written by Gaurav Khaire for Capstone project of SY MTX Class ( March 2022 ).
     It uses PD control for orientation by the grace of mpu6050( hoping to upgrade to mpu 9150 )
     & 3 Ultrasonic Sensors for Obstacle Avoidance.

     Not enough words to express our gratitude towards " " for MPU6050_light & Team BlynkIoT
*/

//-------------------------------------===Includes===------------------------------------------------------

#include <Arduino.h>
#include "Wire.h"
#include <MPU6050_light.h>
#include <WiFi.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

#define BLYNK_TEMPLATE_ID "TMPLBmF9qs5n"
#define BLYNK_DEVICE_NAME "led"
#define BLYNK_AUTH_TOKEN "zqFs5rCKEvY-JJRD2ONpqhGVHeF8b2rz"

char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "TP-Link_E18C";
char pass[] = "31502719";

//-----------------------------------===Pin Definitions===-------------------------------------------------
#define led_pin 19
//--------------------------------------"Motor Pins"-------------------------------------------------------
#define ina1 33         // Right Forward
#define ina2 25
#define ena 32
#define inb1 26         // Left Forward
#define inb2 27 
#define enb 14

//-------------------------------------"Sensor Pins"-------------------------------------------------------
#define left_trig 15
#define left_echo 2
#define front_trig 0
#define front_echo 4
#define right_trig 16
#define right_echo 17

//-------------------------------------MPU Requirements----------------------------------------------------
MPU6050 mpu(Wire);
unsigned long timer_mpu = 0;

//------------------------------------Motor Variables------------------------------------------------------
int mspd = 200;
int m1spd,m2spd,thresh;

const int freq = 5000;
const int resolution = 8;
const int pwm_channel_0 = 0;
const int pwm_channel_1 = 1;

//------------------------------------Sensor Variables-----------------------------------------------------
int distance_right = 0, distance_front = 0 , distance_left = 0;
unsigned long duration_right = 0 , duration_front = 0 , duration_left = 0;
int thresh_right = 10 , thresh_front = 10 , thresh_left = 10;

//------------------------------------Timers and IMP-------------------------------------------------------
float angle = 0;
int ledValue = 0;                // "E-Stop"

unsigned long timer_for_testing = 0;
unsigned long timer_for_update_to_blynk = 0;
BlynkTimer timer;

//-----------------------------------------Flags-----------------------------------------------------------
bool forward_flag = true;
bool left_turn_complete_flag = true;
bool right_turn_complete_flag = true;

//-----------------------------------PD Control Variables--------------------------------------------------
int error = 0, P = 0 , D = 0 , PD = 0 , previousI = 0 , previousError = 0 ;
int Kp = 45, Ki = 0 , Kd = 7;

//----------------------------------Function Declarations--------------------------------------------------
void forward();
void calcError(float reading);
void calcPD();
void diffDrive(int s1, int s2);

void scan();
void main_loop();

float angle_handler(float angle, bool goingForward);
void left_turn();
void right_turn();

// ==========================================     MAIN PROGRAM    =========================================
void setup() 
{

  pinMode(ina1,OUTPUT);
  pinMode(ina2,OUTPUT);
  pinMode(ena,OUTPUT);
  pinMode(inb1,OUTPUT);
  pinMode(inb2,OUTPUT);
  pinMode(enb,OUTPUT);

  ledcSetup(pwm_channel_0, freq , resolution);
  ledcSetup(pwm_channel_1, freq , resolution);
  ledcAttachPin(ena,pwm_channel_0);
  ledcAttachPin(enb,pwm_channel_1);

  digitalWrite(ina1,LOW);
  digitalWrite(ina2,LOW);
  digitalWrite(inb1,LOW);
  digitalWrite(inb2,LOW);

  pinMode(right_trig,OUTPUT);
  pinMode(right_echo,INPUT);
  pinMode(front_trig,OUTPUT);
  pinMode(front_echo,INPUT);
  pinMode(left_trig,OUTPUT);
  pinMode(left_echo,INPUT);

  Serial.begin(115200);
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ }                   // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);

  mpu.calcOffsets();                    // gyro and accelerometer's
  Serial.println("Done!\n");
  thresh = mpu.getAngleZ();

  timer_for_testing = millis();
  timer_for_update_to_blynk = millis();

  Blynk.begin(auth, ssid, pass);
  
  delay(3000);

}

void loop() 
{
  mpu.update();
  Blynk.run();
  main_loop();

  if( (millis() - timer_for_testing) > 5000 )
  {
    left_turn();
    timer_for_testing = millis();
    Serial.print("\tZ : ");
    angle = angle_handler(mpu.getAngleZ(), forward_flag);
    Serial.println(angle);
  }
}


//-------------------------------===Function Definitions===-----------------------------------------------
void main_loop()
{
  if( (millis()-timer_mpu) > 10 )
  {
    Serial.print("\tZ : ");
    angle = angle_handler(mpu.getAngleZ(), forward_flag);
    Serial.println(angle);
    Serial.print("\tThresh = ");
    Serial.println(thresh);
    timer_mpu = millis();  
    calcError(angle);
    calcPD();
    diffDrive(m1spd, m2spd); 
  }
}

void forward()
{
    digitalWrite(ina1,HIGH);
    digitalWrite(ina2,LOW);
    digitalWrite(inb1,HIGH);
    digitalWrite(inb2,LOW);

    ledcWrite(pwm_channel_0, mspd);
    ledcWrite(pwm_channel_1, mspd);

    Serial.print("Ran forward\n");

    delay(50);
}

void calcError(float reading)
{
    error = reading - thresh;
}

void calcPD()
{
    P = error;
    D = error - previousError;
    PD = (Kp*P) - (Kd*D);
    previousError = error;

    m1spd = mspd + PD;
    m2spd = mspd - PD;

    m1spd = constrain(m1spd,0,255);
    m2spd = constrain(m2spd,0,255);
}

void diffDrive(int s1, int s2)
{
    digitalWrite(ina1,HIGH);
    digitalWrite(ina2,LOW);
    digitalWrite(inb1,HIGH);
    digitalWrite(inb2,LOW);

    ledcWrite(pwm_channel_0, s1);
    ledcWrite(pwm_channel_1, s2);

    delay(5);
}

float angle_handler(float angle ,bool goingForward)
{
  if(!goingForward & angle < 0)
  {
    return angle + 360;
  }
  return angle;
}

void left_turn()
{
  
  Serial.println("=================================================");
  Serial.println("Engaging Left Turn");
  Serial.println("=================================================");

  while( mpu.getAngleZ() > (thresh - 90) && left_turn_complete_flag )
  {
    mpu.update();
    digitalWrite(ina1,HIGH);          // Right Motor Forward
    digitalWrite(ina2,LOW);
    digitalWrite(inb1,LOW);
    digitalWrite(inb2,HIGH);          // Left Motor in Reverse

    ledcWrite(pwm_channel_0, mspd);
    ledcWrite(pwm_channel_1, mspd);
  }
  thresh = thresh - 90.0 ;
  left_turn_complete_flag = false;
}

void right_turn()
{
  
  Serial.println("=================================================");
  Serial.println("Engaging Right Turn");
  Serial.println("=================================================");

  while( mpu.getAngleZ() < (thresh + 90) && right_turn_complete_flag )
  {
    mpu.update();
    digitalWrite(ina1,HIGH);          // Right Motor Forward
    digitalWrite(ina2,LOW);
    digitalWrite(inb1,LOW);
    digitalWrite(inb2,HIGH);          // Left Motor in Reverse

    ledcWrite(pwm_channel_0, mspd);
    ledcWrite(pwm_channel_1, mspd);
  }
  thresh = thresh + 90.0 ;
  right_turn_complete_flag = true;
}

void scan()
{
  digitalWrite(right_trig, LOW);
  delayMicroseconds(2);

  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(right_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(right_trig, LOW);

  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration_right = pulseIn(right_echo, HIGH);

  // Calculating the distance
  distance_right = duration_right*0.034/2;       // Speed of sound and its duration gets doubled because of echo

  digitalWrite(front_trig, LOW);
  delayMicroseconds(2);

  digitalWrite(front_trig,HIGH);
  delayMicroseconds(10);
  digitalWrite(front_trig, LOW);

  duration_front = pulseIn(front_echo, HIGH);

  distance_front = duration_front * 0.034/2;

  digitalWrite(left_trig, LOW);
  delayMicroseconds(2);

  digitalWrite(left_trig, HIGH);
  delayMicroseconds(10);
  digitalWrite(left_trig, LOW);

  duration_left = pulseIn(left_echo, HIGH);

  distance_left = duration_left * 0.034/2;
}

void brake()
{
    digitalWrite(ina1,LOW);       
    digitalWrite(ina2,LOW);
    digitalWrite(inb1,LOW);
    digitalWrite(inb2,LOW);
}

//--------------------------------------Blynk Functions----------------------------------------------------

BLYNK_WRITE(V1)                             // V1 is the speed slider on Blynk Dashboard
{
  int val = param.asInt();
  mspd = val;
}

BLYNK_WRITE(V12)                            // V12 is the led key on Blynk Dashboard
{
  ledValue = param.asInt();
  digitalWrite(led_pin,ledValue);
  if(ledValue == 1)
  {
    Blynk.virtualWrite(V3, distance_right);
  }
  else 
  {
    brake();
    scan();
    Blynk.virtualWrite(V3, distance_right);
  }

}