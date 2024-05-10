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
// Hall sensor previous state
int previousStateRight = LOW;
int previousStateLeft = LOW;
// Wheel radius
float wheelRadius = 0.125;
// Distance between 2 wheels
float L = 0.52;
// HALL sensor full cycle is 15 ticks
int ticksPerRevolution = 15;