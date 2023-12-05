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



float wheel_radius = 0.125;
float L= 0.52;
int ticks_per_revolution = 15;

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