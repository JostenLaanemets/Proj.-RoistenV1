#include <IBusBM.h>
#include <ros.h>
#include <FastLED.h>
#include <geometry_msgs/Twist.h>
#define  PI 3.14159
#define numLeds 10
#define ledPin 53
CRGB leds[numLeds];

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

// d = Wheel diameter
const float d = 0.25;

// Variables for right and left velocities       
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
// Adding a limit to our output
const float maxSpeed=0.7;


void CmdVelCallback( const geometry_msgs::Twist& velocity)
{
  velLeft = velocity.linear.x - d * velocity.angular.z;
  velRight = velocity.linear.x + d * velocity.angular.z;
  Leds(1);
  // Reverse and brake logic
  if (velRight < 0.0){
    velRight = velRight * (-1);
    digitalWrite(rightReverse, LOW);
  }else{
    digitalWrite(rightReverse, HIGH);
  }

  if (velLeft < 0.0){
    velLeft = velLeft * (-1);
    digitalWrite(leftReverse, LOW);
  }else{
    digitalWrite(leftReverse, HIGH);
  }

  BrakeLogic(velRight,velLeft);

  delay(10);
  // Outputing the channels to ESC
  analogWrite(6, velLeft * maxSpeed);
  analogWrite(7 ,velRight * maxSpeed);
}

void RemoteControl()
{
  // Reading certain channels
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
    Leds(2);
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

    BrakeLogic(velRight,velLeft);
    delay(10);
    // Outputing the channels to ESC
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
  // Adding LEDstrip
  FastLED.addLeds<WS2812B, ledPin, GRB>(leds, numLeds).setCorrection(TypicalLEDStrip);
  FastLED.setBrightness(50);
  // Initializing node and subscribing to it
  nh.initNode();
  nh.subscribe(sub);
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
  Odometry();
  delay(10);
}

