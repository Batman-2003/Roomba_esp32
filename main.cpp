
/* 
     Written by Gaurav Khaire for Capstone project of SY MTX Class ( March/April 2023 ).
     It uses PD control for orientation by the grace of mpu6050( hoping to upgrade to mpu 9250 )
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
char ssid[] = "";                  // I won't shoot myself in the foot
char pass[] = "";

//-----------------------------------===Pin Definitions===-------------------------------------------------
#define led_pin 19
#define indicator_led 23

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
unsigned int counter = 1;

const int freq = 5000;
const int resolution = 8;
const int pwm_channel_0 = 0;
const int pwm_channel_1 = 1;

//------------------------------------Sensor Variables-----------------------------------------------------
int distance_right = 0, distance_front = 0 , distance_left = 0;
unsigned long duration_right = 0 , duration_front = 0 , duration_left = 0;
int thresh_right = 7 + 1 , thresh_front = 15 + 1 , thresh_left = 7 + 1;

//------------------------------------Timers and IMP-------------------------------------------------------
float angle = 0;
int ledValue = 0;                // "E-Stop"

unsigned long timer_for_testing = 0;
unsigned long timer_for_update_to_blynk = 0;
unsigned long timer_for_yaw_drift = 0;
unsigned long timer_for_u_turn = 0;
unsigned long timer_for_forward_forward = 0;
unsigned long timer_for_forward_sideways = 0;

unsigned long sideways_timer = 0;           // for now I will use millis()
unsigned long sideways_return_timer = 0;

int thresh_time_for_u_turn = 15000;

byte drifter = 0;
BlynkTimer timer;

//-----------------------------------------Flags-----------------------------------------------------------
bool forward_flag = true;
bool left_turn_complete_flag = false;
bool right_turn_complete_flag = false;
bool u_turn_flag = false;

//-----------------------------------PD Control Variables--------------------------------------------------
int error = 0, P = 0 , D = 0 , PD = 0 , previousError = 0 ;
int Kp = 45, Ki = 0 , Kd = 7;

//----------------------------------Function Declarations--------------------------------------------------
void u_turn(bool forward);
void left_u_turn();
void right_u_turn();

void forward(int till_ms);
void diffDrive(int s1, int s2);

void calcError(float reading);
void calcPD();

void scan();
void scan_front();
void sync_dashboard();

void main_loop();

float angle_handler(float angle);

void left_turn();
void right_turn();
void hug_left_wall();
void hug_right_wall();
void hug_left_wall_till();
void hug_right_wall_till();
void left_scoop();
void right_scoop();

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
    delay(5000);
    scan();
    Blynk.virtualWrite(V3, distance_right);
  }
}

// ==========================================     MAIN PROGRAM    =========================================
void setup() 
{

  pinMode(led_pin,OUTPUT);
  digitalWrite(led_pin,HIGH);

  pinMode(indicator_led, OUTPUT);

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

  if(status == 0)
  {
    digitalWrite(indicator_led, HIGH);
  }

  else
  {
    while(1)
    {
      digitalWrite(indicator_led, HIGH);
      delay(1000);
      digitalWrite(indicator_led, LOW);
      delay(1000);
    }
  }
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);

  mpu.calcOffsets();                    // gyro and accelerometer's
  Serial.println("Done!\n");
  thresh = mpu.getAngleZ();


  Blynk.begin(auth, ssid, pass);
  
  ledValue = 1;

  delay(3000);

  digitalWrite(indicator_led , LOW);

  timer_for_u_turn = timer_for_update_to_blynk = millis();

}

void loop() 
{
  mpu.update();
  Blynk.run();
  main_loop();
  flag_handler();
  // sync_dashboard();
}


//-------------------------------===Function Definitions===-----------------------------------------------

void main_loop()
{
  if( (millis()-timer_mpu) > 10 && ledValue )
  {
    Serial.print("\tZ : ");
    angle = angle_handler(mpu.getAngleZ());
    Serial.println(angle);
    Serial.print("\tThresh = ");
    Serial.println(thresh);
    timer_mpu = millis();  
    // scan_front();
    scan();
    if( distance_front < thresh_front + 10 )
    {
      if( u_turn_flag )
      {
        u_turn(forward_flag);
      }
      else
      {
        if( distance_front < thresh_front )
        {
          left_scoop();
        }
      }
    }
    calcError(angle);
    calcPD();
    diffDrive(m1spd, m2spd);
  }

  else
  {
    brake();
  }
}

void forward(int till_ms)
{
    timer_for_forward_forward = millis();
    while( (millis() - timer_for_forward_forward) < till_ms )
    {
      mpu.update();
      Blynk.run();
      scan();
      angle = angle_handler(mpu.getAngleZ());
      calcError(angle);
      calcPD();
      diffDrive(m1spd,m2spd);

      Serial.print("\nRan forward\n");
    }
    brake();
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

    delay(4);
}

void hug_left_wall()                      // Hugs left wall & PD stabilizes for ultrasonics sensors instead of gyro
{
  sideways_timer = millis();
  while( distance_left < 30 )
  {
    mpu.update();
    Blynk.run();
    scan();
    error = (distance_left - thresh_left) / 5;
    calcPD();
    diffDrive(m1spd, m2spd);
  }
  sideways_timer = millis() - sideways_timer;
}

void hug_right_wall()                     // Hugs right wall ^^^^^^^^^^^^^^^^^^
{
  sideways_timer = millis();
  Kp = 45;
  Kd = 7;

  while( distance_right < thresh_right + 5 )
  {
    mpu.update();
    Blynk.run();
    scan();
    error = -(distance_right - thresh_right) / 5.0;
    calcPD();
    diffDrive(m1spd, m2spd);
    Serial.println("Hugging Right");
  }
  sideways_timer = millis() - sideways_timer;
  Kp = 45;
  Kd = 7;
}

void hug_left_wall_till()
{
  sideways_return_timer = millis();
  while( distance_left < 30 && (millis() - sideways_return_timer) > sideways_timer )
  {
    mpu.update();
    Blynk.run();
    scan();
    error = (distance_left - thresh_left) / 5.0;
    calcPD();
    diffDrive(m1spd, m2spd);
  }
}
void hug_right_wall_till()
{
    sideways_return_timer = millis();
    while( distance_right < 30 && (millis() - sideways_return_timer) > sideways_timer - 500 )
  {
    mpu.update();
    Blynk.run();
    scan();
    error = (distance_right - thresh_right) / 5.0;
    calcPD();
    diffDrive(m1spd, m2spd);
  }
}

float angle_handler(float angle)
{
  if((millis() - timer_for_yaw_drift) > 75000 )
  {
    timer_for_yaw_drift = millis();
    // ++drifter;
    return angle + drifter;
  }
  return angle + drifter;
}

void left_turn()
{
  
  Serial.println("=================================================");
  Serial.println("Engaging Left Turn");
  Serial.println("=================================================");

  while( mpu.getAngleZ() > (thresh - 90) )
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
  left_turn_complete_flag = true;
}

void right_turn()
{
  
  Serial.println("=================================================");
  Serial.println("Engaging Right Turn");
  Serial.println("=================================================");

  while( angle < (thresh + 90) )
  {
    mpu.update();
    angle = angle_handler(mpu.getAngleZ());
    digitalWrite(ina1,LOW);          
    digitalWrite(ina2,HIGH);           // Right Motor Reverse
    digitalWrite(inb1,HIGH);           // Left Motor in Forward
    digitalWrite(inb2,LOW);          

    ledcWrite(pwm_channel_0, mspd);
    ledcWrite(pwm_channel_1, mspd);

    Serial.println("");
    Serial.println(angle);
    Serial.println(thresh);
  }
  thresh = thresh + 90.0 ;
  right_turn_complete_flag = true;
}

void left_scoop()
{
  left_turn();
  brake();
  forward(2000);
  hug_right_wall();
  forward(500);
  brake();
  right_turn();
  Serial.print(sideways_timer);
  brake();
  forward(5000);
  hug_right_wall();
  brake();
  forward(1500);
  right_turn();
  forward(2500);
  brake();
  hug_left_wall_till();
  left_turn();
  brake();
  delay(5000);
}

void right_scoop()
{
  right_turn();
  hug_left_wall();
  forward(2200);
  left_turn();
  forward(1000);
  hug_left_wall();
  forward(1000);
  left_turn();
  hug_left_wall_till();
  right_turn();
}

void scan_front()
{
  if(++counter % 3 == 0)
  {
    digitalWrite(front_trig, LOW);
    delayMicroseconds(2);

    digitalWrite(front_trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(front_trig, LOW);

    duration_front = pulseIn(front_echo, HIGH);

    distance_front = duration_front * 0.034/2;

    Serial.print("\tThe Front : ");
    Serial.println(distance_front);
  }

  else if(counter > 50000 )
  {
    counter = 1;
  }
}

void scan()
{
  if(++counter % 3 == 0)
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
    distance_right = duration_right * 0.034/2;       // Speed of sound and its duration gets doubled because of echo

    digitalWrite(front_trig, LOW);
    delayMicroseconds(2);

    digitalWrite(front_trig,HIGH);
    delayMicroseconds(10);
    digitalWrite(front_trig, LOW);

    duration_front = pulseIn(front_echo, HIGH);

    distance_front = duration_front * 0.034/2;

    // digitalWrite(left_trig, LOW);
    // delayMicroseconds(2);

    // digitalWrite(left_trig, HIGH);
    // delayMicroseconds(10);
    // digitalWrite(left_trig, LOW);

    // duration_left = pulseIn(left_echo, HIGH);

    // distance_left = duration_left * 0.034/2;


    Serial.print("Left : ");
    Serial.print(distance_left);
    Serial.print("\tFront : ");
    Serial.print(distance_front);
    Serial.print("\tRight : ");
    Serial.print(distance_right);

  }

  else if( counter > 50000 )
  {
    counter = 1;
  }
}

void brake()
{
    digitalWrite(ina1,LOW);       
    digitalWrite(ina2,LOW);
    digitalWrite(inb1,LOW);
    digitalWrite(inb2,LOW);
}

void u_turn(bool forward)
{
  Serial.println("Engaging U Turn");
  if(forward == true)
  {
    left_u_turn();
    forward_flag = false;

  }
  else
  {
    right_u_turn();
    forward_flag = true;
  }
}

void left_u_turn()
{
  left_turn();
  brake();
  forward(2000);
  brake();
  left_turn();
  brake();
  timer_for_u_turn = millis();
  u_turn_flag = false;
  forward_flag = !forward_flag;
}

void right_u_turn()
{
  right_turn();
  brake();
  forward(2000);
  brake();
  right_turn();
  brake();  
  timer_for_u_turn = millis();
  u_turn_flag = false;
  forward_flag = !forward_flag;
}

void flag_handler()
{
  if( ( millis() - timer_for_u_turn ) > thresh_time_for_u_turn ) 
  {
    u_turn_flag = true;
  }
  else
  {
    u_turn_flag = false;
  }
}

void sync_dashboard()
{
  if( (timer_for_update_to_blynk - millis() ) > 200 )
  {
    Blynk.virtualWrite(V3, distance_right);
    Blynk.virtualWrite(V4, distance_front);
    Blynk.virtualWrite(V6, distance_left);
    Blynk.virtualWrite(V5, angle);
    Blynk.virtualWrite(V8, forward_flag);
    timer_for_update_to_blynk = millis();
  }
}
