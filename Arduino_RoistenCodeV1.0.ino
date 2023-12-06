#include <IBusBM.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#define PI 3.1415926535897932384626433832795
//IBUS
IBusBM ibusRC;
HardwareSerial& ibusRcSerial = Serial1;
HardwareSerial& debugSerial = Serial;
int channel1 = 0;
int channel3 = 0;
int channel5 = 0;

ros::NodeHandle nh;

float d = 0.25;

float vel_left = 0;
float vel_right = 0;

const int Hall_Right_PIN = 4;
const int Hall_Left_PIN = 5;

const int Right_Brake = 2;
const int Left_Brake = 3;

const int Right_Reverse = A0;
const int Left_Reverse = A1;

int previousState_Right = LOW;
int previousState_Left = LOW;
void CmdVelCallback( const geometry_msgs::Twist& velocity){

  vel_left = velocity.linear.x - d * velocity.angular.z;
  vel_right = velocity.linear.x + d * velocity.angular.z;
  
  if (vel_right < 0.0)
  {
    vel_right = vel_right * (-1);
    digitalWrite(Right_Reverse,LOW);
  }else{
    digitalWrite(Right_Reverse,HIGH);
  }
  if (vel_left < 0.0)
  {
    vel_left = vel_left * (-1);
    digitalWrite(Left_Reverse,LOW);
  }else{
    digitalWrite(Left_Reverse,HIGH);
  }
  if (vel_right == 0.0 & vel_left == 0.0)
  {
    digitalWrite(Right_Brake,LOW);
    digitalWrite(Left_Brake,LOW);
  }else{
    digitalWrite(Right_Brake,HIGH);
    digitalWrite(Left_Brake,HIGH);
  }
  delay(10);
  
  analogWrite(6,vel_left*0.79);
  analogWrite(7,vel_right*0.60);
}

void RemoteControl(){
  channel3 = readChannel(2, -100, 100, 0);
  channel1 = readChannel(0, -100, 100, 0);
  channel5 = readChannel(4, -100, 100, 0);


  vel_left = channel3 + channel1+1;
  vel_right = channel3 - channel1+1;
  // Serial.println(vel_left);
  // Serial.println(vel_right);
  if (channel5 == 100)
  {
    nh.spinOnce();
  }else if (channel5 == 0){
    digitalWrite(Right_Brake,LOW);
    digitalWrite(Left_Brake,LOW);
  }else{
    digitalWrite(Right_Brake,HIGH);
    digitalWrite(Left_Brake,HIGH);
    if (vel_right < 0.0)
    {
        vel_right = vel_right * (-1);
        digitalWrite(Right_Reverse,LOW);
      }else{
        digitalWrite(Right_Reverse,HIGH);
      }
      if (vel_left < 0.0)
      {
        vel_left = vel_left * (-1);
        digitalWrite(Left_Reverse,LOW);
      }else{
        digitalWrite(Left_Reverse,HIGH);
      }
      if (vel_right == 0.0 & vel_left == 0.0)
      {
        digitalWrite(Right_Brake,LOW);
        digitalWrite(Left_Brake,LOW);
      }else{
        digitalWrite(Right_Brake,HIGH);
        digitalWrite(Left_Brake,HIGH);
      }


      delay(10);

      analogWrite(6,vel_left*0.79);
      analogWrite(7,vel_right*0.60);
  }
  
}
//subscriber
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , CmdVelCallback);
void setup() {
  // put your setup code here, to run once:
  pinMode(Hall_Right_PIN, INPUT);
  pinMode(Hall_Left_PIN, INPUT);
  pinMode(Right_Reverse, OUTPUT);
  pinMode(Left_Reverse, OUTPUT);
  pinMode(Right_Brake, OUTPUT);
  pinMode(Left_Brake, OUTPUT);
  pinMode(6, OUTPUT);
  pinMode(7, OUTPUT);


  nh.initNode();
  nh.subscribe(sub);
  debugSerial.begin(57600);
  ibusRC.begin(ibusRcSerial);
}
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue)
  {
  uint16_t ch = ibusRC.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  return map(ch, 1000, 2000, minLimit, maxLimit);
  }
void loop() {

  RemoteControl();
  Odometry();
  delay(10);
}