int ticksRight = 0;
int ticksLeft = 0;

float prevTicksLeft = 0.0;
float prevTicksRight = 0.0;

float leftRotation = 0;
float rightRotation = 0;

float leftDistance = 0;
float rightDistance = 0;

float distanceTraveled = 0;
float degreesTurned = 0;
float deltaX = 0;
float deltaY = 0;

// Wheel radius
float wheelRadius = 0.125;
// DIstance between 2 wheels
float L = 0.52;
// HALL sensor full cycle is 15 ticks
int ticksPerRevolution = 15;

void Odometry()
{
  // Read HALL sensors
  int hallRight = digitalRead(hallRightPin);
  int hallLeft = digitalRead(hallLeftPin);

  // Check if right and left wheel have moved, if so, then add ticks to variable
  if (hallRight == HIGH && previousStateRight == LOW) 
  {
    ticksRight = ticksRight + hallRight;
  }
  if (hallLeft == HIGH && previousStateLeft == LOW) 
  {
    ticksLeft = ticksLeft + hallLeft;
  }

  previousStateRight = hallRight;
  previousStateLeft = hallLeft;

  // Calculate rotations of each wheel
  leftRotation = ticksLeft * ((2 * PI) / ticksPerRevolution);
  rightRotation = ticksRight * ((2 * PI) / ticksPerRevolution);

  // Calculate distance traveled of each wheel
  leftDistance = wheelRadius * leftRotation;
  rightDistance = wheelRadius * rightRotation;

  distanceTraveled = (rightDistance+leftDistance) / 2;
  degreesTurned = (rightDistance-leftDistance) / L;

  deltaX = distanceTraveled*cos(degreesTurned);
  deltaY = distanceTraveled*sin(degreesTurned);
}