#include <ros.h>
#include <geometry_msgs/Twist.h>
#define PI 3.1415926535897932384626433832795
ros::NodeHandle nh;
//15 tick on t2isring
float d = 0.25;
float wheel_radius = 0.125;
float L= 0.52;
int ticks_per_revolution = 15;

float vel_left = 0;
float vel_right = 0;

int ticks_right=0;
int ticks_left =0;
float prev_tick_left=0.0;
float prev_tick_right=0.0;

float Left_Rotation=0;
float Right_Rotation=0;

float L_Distance=0;
float R_Distance=0;

float Delta_A=0;
float Delta_null=0;
float Delta_x = 0;
float Delta_y=0;

const int Hall_Right_PIN = 2;
const int Hall_Left_PIN = 3;

int previousState_Right = LOW;
int previousState_Left = LOW;

void CmdVelCallback( const geometry_msgs::Twist& velocity){
    vel_left=velocity.linear.x - d*velocity.angular.z;
    vel_right=velocity.linear.x + d*velocity.angular.z;
    if (vel_right < 0.0)
    {
      vel_right= vel_right*(-1);
      digitalWrite(8,LOW);
      Serial.println("-------------ON LOW-----------");
    }else{
      digitalWrite(8,HIGH);
    }
    if (vel_left < 0.0)
    {
      vel_left= vel_left*(-1);
      digitalWrite(9,LOW);
    }else{
      digitalWrite(9,HIGH);
    }
    if (vel_right == 0.0 & vel_left == 0.0)
    {
      digitalWrite(10,LOW);
      digitalWrite(11,LOW);
    }else{
      digitalWrite(10,HIGH);
      digitalWrite(11,HIGH);
    }
    delay(100);
    Serial.println(vel_right);
    analogWrite(5,vel_left);
    analogWrite(6,vel_right);
}

void Odometry(){
  int Hall_Right = digitalRead(Hall_Right_PIN);
  int Hall_Left = digitalRead(Hall_Left_PIN);
  if (Hall_Right == HIGH && previousState_Right == LOW) {
    ticks_right = ticks_right + Hall_Right;
  }
  if (Hall_Left == HIGH && previousState_Left == LOW) {
    ticks_left = ticks_left + Hall_Left;
  }

  previousState_Right = Hall_Right;
  previousState_Left = Hall_Left;

  Left_Rotation= ticks_left * ((2*PI)/ticks_per_revolution);
  Right_Rotation= ticks_right * ((2*PI)/ticks_per_revolution);

  L_Distance= wheel_radius*Left_Rotation;
  R_Distance= wheel_radius*Right_Rotation;

  Delta_A=(R_Distance+L_Distance)/2;
  Delta_null= (R_Distance-L_Distance)/L;
  Delta_x=Delta_A*cos(Delta_null);
  Delta_y=Delta_A*sin(Delta_null);
  
  Serial.print("Left_Rotation: ");
  Serial.println(Left_Rotation);
  Serial.print("Right_Rotation: ");
  Serial.println(Right_Rotation);
  Serial.print("L_Distance: ");
  Serial.println(L_Distance);
  Serial.print("R_Distance: ");
  Serial.println(R_Distance);
}

ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , CmdVelCallback);
void setup() {
    // put your setup code here, to run once:
    pinMode(Hall_Right_PIN, INPUT);
    pinMode(Hall_Left_PIN, INPUT);
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(10, OUTPUT);
    pinMode(11, OUTPUT);
    nh.initNode();
    nh.subscribe(sub);
}
void loop() {
    Odometry();
    nh.spinOnce();
    delay(10);
}