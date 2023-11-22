#include <ros.h>
#include <geometry_msgs/Twist.h>
#define PI 3.1415926535897932384626433832795
ros::NodeHandle nh;

float vel_left = 0;
float vel_right = 0;

float prev_tick_left=0.0;
float prev_tick_right=0.0;
float d = 0.25;
int sum_right=0;





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
  int hall_right = 0;
  int hallB_right=0;
  int hall_left = 0;

  float wheel_radius = 0.125;
  float L= 0.52;
  
  hall_right = digitalRead(2);
  hallB_right = digitalRead(3);
  sum_right = sum_right + hall_right;
  /*Serial.println("sum right");
  Serial.println(sum_right);
  Serial.println("HALL A right");
  Serial.println(hallB_right);*/

  prev_tick_left = hall_left;
  prev_tick_right = sum_right;
    
  int ticks_per_revolution = 4;
  float alpha = 2 * PI / ticks_per_revolution;

  float ticks_left = hall_left;
  float ticks_right = sum_right;

  float delta_ticks_left = ticks_left - prev_tick_left;
  float delta_ticks_right = ticks_right - prev_tick_right;

  float rotation_wheel_left = alpha * delta_ticks_left;
  float rotation_wheel_right = alpha * delta_ticks_right;
  
  prev_tick_left = ticks_left;
  prev_tick_right = ticks_right;


  float distance_left = wheel_radius * rotation_wheel_left;
  float distance_right = wheel_radius * rotation_wheel_right;

  float distance_traveled = (distance_left + distance_right) / 2;     
  float delta_theta = (distance_right - distance_left) / L;

  /*Serial.print("hall right: ");
  Serial.println(hall_right);
  Serial.print("alpha: ");
  Serial.println(alpha);
  Serial.print("delta ticks right: ");
  Serial.println(alpha);
  Serial.print("distance right: ");
  Serial.println(delta_ticks_right);
  Serial.print("rotation wheel right: ");
  Serial.println(rotation_wheel_right);*/
  
}





ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel" , CmdVelCallback);
void setup() {
    // put your setup code here, to run once:
    nh.initNode();
    nh.subscribe(sub);
    pinMode(8, OUTPUT);
    pinMode(9, OUTPUT);
   

}
void loop() {
    // put your main code here, to run repeatedly:
    Odometry();
    
    nh.spinOnce();
    delay(10);
}