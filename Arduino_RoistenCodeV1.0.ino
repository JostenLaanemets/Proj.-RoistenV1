#include <IBusBM.h>
#include <ros.h>
#include <geometry_msgs/Twist.h>
#define PI 3.1415926535897932384626433832795

//IBUS
IBusBM ibusRC;
HardwareSerial& ibusRcSerial = Serial1;
HardwareSerial& debugSerial = Serial;

// Declare channels
int channel1 = 0;
int channel3 = 0;
int channel5 = 0;

// Start the node
ros::NodeHandle nh;

float d = 0.25;       // d = Wheel diameter
float velLeft = 0;
float velRight = 0;

// Define HALL pins
const int hallRightPin = 4;
const int hallLeftPin = 5;

// Define braking pins
const int rightBrake = 2;
const int leftBrake = 3;

// Define reversing pins
const int rightReverse = A0;
const int leftReverse = A1;

int previousStateRight = LOW;
int previousStateLeft = LOW;

float maxSpeed=0.7;

void CmdVelCallback( const geometry_msgs::Twist& velocity)
{
  velLeft = velocity.linear.x - d * velocity.angular.z;
  velRight = velocity.linear.x + d * velocity.angular.z;
  
  // Reverse and brake logic
  if (velRight < 0.0)
  {
    velRight = velRight * (-1);
    digitalWrite(rightReverse, LOW);
  }else
  {
    digitalWrite(rightReverse, HIGH);
  }

  if (velLeft < 0.0)
  {
    velLeft = velLeft * (-1);
    digitalWrite(leftReverse, LOW);
  }else
  {
    digitalWrite(leftReverse, HIGH);
  }

  if (velRight == 0.0 && velLeft == 0.0)
  {
    digitalWrite(rightBrake, LOW);
    digitalWrite(leftBrake, LOW);
  }else
  {
    digitalWrite(rightBrake, HIGH);
    digitalWrite(leftBrake, HIGH);
  }

  delay(10);
  analogWrite(6, velLeft * maxSpeed);
  analogWrite(7 ,velRight * maxSpeed);
}

void RemoteControl()
{
  channel3 = readChannel(2, -100, 100, 0);
  channel1 = readChannel(0, -100, 100, 0);
  channel5 = readChannel(4, -100, 100, 0);

  velLeft = channel3 + channel1+1;
  velRight = channel3 - channel1+1;

  if (channel5 == 100)
  {
    nh.spinOnce();
  }else if (channel5 == 0)
  {
    digitalWrite(rightBrake,LOW);
    digitalWrite(leftBrake,LOW);
  }else
  {
    digitalWrite(rightBrake,HIGH);
    digitalWrite(leftBrake,HIGH);
    if (velRight < 0.0)
    {
        velRight = velRight * (-1);
        digitalWrite(rightReverse,LOW);
      }else
      {
        digitalWrite(rightReverse,HIGH);
      }
      if (velLeft < 0.0)
      {
        velLeft = velLeft * (-1);
        digitalWrite(leftReverse,LOW);
      }else
      {
        digitalWrite(leftReverse,HIGH);
      }
      if (velRight == 0.0 && velLeft == 0.0)
      {
        digitalWrite(rightBrake,LOW);
        digitalWrite(leftBrake,LOW);
      }else
      {
        digitalWrite(rightBrake,HIGH);
        digitalWrite(leftBrake,HIGH);
      }

      delay(10);

      analogWrite(6, velLeft * maxSpeed);
      analogWrite(7 ,velRight * maxSpeed);
  }
  
}
// Subscriber
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , CmdVelCallback);

void setup() 
{
  // Put your setup code here, to run once:
  pinMode(hallRightPin, INPUT);
  pinMode(hallLeftPin, INPUT);
  pinMode(rightReverse, OUTPUT);
  pinMode(leftReverse, OUTPUT);
  pinMode(rightBrake, OUTPUT);
  pinMode(leftBrake, OUTPUT);
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

void loop() 
{
  RemoteControl();
  Odometry();
  delay(10);
}