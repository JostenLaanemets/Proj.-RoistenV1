#include <IBusBM.h>
#include <ros.h>
#include <std_msgs/Int32MultiArray.h>
#include <FastLED.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>
#define  PI 3.14159
#define numLeds 10
#define ledPin 42   // 53
CRGB leds[numLeds];

//IBUS
IBusBM ibusRC;
HardwareSerial& ibusRcSerial = Serial1;
HardwareSerial& debugSerial = Serial;

// Declare channels
int channel1 = 0;
int channel2 = 0;
int channel3 = 0;
int channel4 = 0;
int channel5 = 0;

// Start the node
ros::NodeHandle nh;

// d = Wheel diameter
const float d = 0.25;

// Variables for right and left velocities       
float velLeft = 0;
float velRight = 0;

// Define HALL pins
const int leftMotor = 6;
const int rightMotor = 7;

// Define HALL pins
const int hallRightPin1 = A8;   // 51
const int hallRightPin2 = A9;   // 49 
const int hallRightPin3 = A10;  // 47

const int hallLeftPin1 = A7;
const int hallLeftPin2 = A6;
const int hallLeftPin3 = A5;

// Define braking pins
const int rightBrake = 2;
const int leftBrake = 3;

// Define reversing pins
const int rightReverse = A0;
const int leftReverse = A1;

bool rightReverseOut = 0;
bool leftReverseOut = 0;

// Adding a limit to our output
const float maxSpeed = 0.7;

bool reached = false;

void LidarCallback(const std_msgs::Bool& value)
{
  reached = value.data;
}

void CmdVelCallback( const geometry_msgs::Twist& velocity)
{
  velLeft = velocity.linear.x - d * velocity.angular.z;
  velRight = velocity.linear.x + d * velocity.angular.z;
  // Reverse and brake logic
  if (velRight < 0.0)
  {
    velRight = velRight * (-1);
    rightReverseOut = 1;
    digitalWrite(rightReverse, LOW);
  }else
  {
    rightReverseOut = 0;
    digitalWrite(rightReverse, HIGH);
  }

  if (velLeft < 0.0)
  {
    velLeft = velLeft * (-1);
    leftReverseOut = 1;
    digitalWrite(leftReverse, LOW);
  }else
  {
    leftReverseOut = 0;
    digitalWrite(leftReverse, HIGH);
  }

  BrakeLogic(velRight,velLeft);

  delay(10);
  // Outputing the channels to ESC
  analogWrite(leftMotor, velLeft * maxSpeed);
  analogWrite(rightMotor, velRight * maxSpeed);
}

// Publisher
std_msgs::String list_msg;
ros::Publisher pub("wheelticks", &list_msg);


// Subscriber
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , CmdVelCallback);
ros::Subscriber<std_msgs::Bool> lidar("obj_detection", LidarCallback);

void RemoteControl()
{
  // Reading certain channels
  channel3 = readChannel(2, -100, 100, 0);
  channel2 = readChannel(1, -100, 100, 0);
  channel1 = readChannel(0, -100, 100, 0);
  channel4 = readChannel(3, -100, 100, 0);
  channel5 = readChannel(4, -100, 100, 0);

  
  velLeft = channel2 + channel1+1;
  velRight = channel2 - channel1+1;

  // 100 = Teleoperation
  if (channel5 == 100)
  {
    if (reached)
    {
      Leds(1);
      BrakeLogic(0.0, 0.0);
    }else
    {
      BrakeLogic(0.1, 0.1);
      Leds(3);
    }

  // 0 = Neutral
  }else if (channel5 == 0)
  {
    Leds(2);
    BrakeLogic(0.0, 0.0);
  }
  // -100 = Remote control
  else
  {
    //problem, engine kaput, channel 5: -100 = puldi reziim, 100 = teleop/serial reziim
    if (velRight < 0.0)
    {
      velRight = velRight * (-1);
      rightReverseOut = 1;
      digitalWrite(rightReverse,LOW);
    }else
    {
      rightReverseOut = 0;
      digitalWrite(rightReverse,HIGH);
    }
    if (velLeft < 0.0)
    {
       velLeft = velLeft * (-1);
       leftReverseOut = 1;
      digitalWrite(leftReverse,LOW);
    }else
    {
      leftReverseOut = 0;
      digitalWrite(leftReverse,HIGH);
    }

    BrakeLogic(velRight,velLeft);

    // Outputing the channels to ESC
    analogWrite(leftMotor, velLeft * maxSpeed);
    analogWrite(rightMotor, velRight * maxSpeed);
    Leds(4);
  }
}

void setup()
{
  // Put your setup code here, to run once:
  pinMode(hallRightPin1, INPUT);
  pinMode(hallRightPin2, INPUT);
  pinMode(hallRightPin3, INPUT);
  pinMode(hallLeftPin1, INPUT);
  pinMode(hallLeftPin2, INPUT);
  pinMode(hallLeftPin3, INPUT);
  pinMode(rightReverse, OUTPUT);
  pinMode(leftReverse, OUTPUT);
  pinMode(rightBrake, OUTPUT);
  pinMode(leftBrake, OUTPUT);
  pinMode(leftMotor, OUTPUT);
  pinMode(rightMotor, OUTPUT);
  pinMode(leftReverseOut, OUTPUT);
  pinMode(rightReverseOut, OUTPUT);

  // Adding LEDstrip
  FastLED.addLeds<WS2812B, ledPin, GRB>(leds, numLeds).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(50);

  // Initializing node and subscribing to it
  nh.initNode();
  nh.subscribe(sub);
  nh.subscribe(lidar);
  nh.advertise(pub);

  // Starting IBUS
  debugSerial.begin(57600);
  ibusRC.begin(ibusRcSerial);
}

// Reading the channels
int readChannel(byte channelInput, int minLimit, int maxLimit, int defaultValue)
{
  uint16_t ch = ibusRC.readChannel(channelInput);
  if (ch < 100) return defaultValue;
  
  return map(ch, 1000, 2000, minLimit, maxLimit);
}


void loop() 
{
  RemoteControl();
  String str = String(digitalRead(hallRightPin1)) + String(digitalRead(hallRightPin2)) + String(digitalRead(hallRightPin3)) + " " + String(rightReverseOut)+ " " +
              String(digitalRead(hallLeftPin1)) + String(digitalRead(hallLeftPin2)) + String(digitalRead(hallLeftPin3)+ " " + String(leftReverseOut)); 
  list_msg.data = str.c_str();

  pub.publish(&list_msg);
  nh.spinOnce();
  delay(10);
}





