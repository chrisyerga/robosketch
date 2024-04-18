//=================================================================
// RoboSketch
//
// Drives an Etch-A-Sketch via 2 stepper motors
//=================================================================

#include <AFMotor.h>

//-----------------------------------------------------------------
// Axis class maintains state for each axis of the Etch A Sketch.
//-----------------------------------------------------------------
class Axis
{
public:
  Axis(int motorPort, int fudge, bool inversePolarity) : 
      motor(48, motorPort)
  {
    // Last step dir is undefined
    lastStepDirection = -1;
    
    reverseFudge = fudge;
    polarity = inversePolarity;
    
    stepCount = 0;
    
    // Initialize motor RPM
    motor.setSpeed(240);
  }
  
  // Perform a single step of the axis's motor in the given direction
  void Step(int direction)
  {
    if (polarity)
    {
      if (direction == FORWARD)
      {
        direction = BACKWARD;
      }
      else
      {
        direction = FORWARD;
      }
    }
    
    // If this is the first step we've taken then initialize direction
    // to the dir we're going in
    if (lastStepDirection == -1)
    {
      lastStepDirection = direction;
    }
  
    // If we're reversing direction from the last movement, then perform
    // an additional step. The Etch A Sketch movement has a bit of slop
    // and the pen won't actually move for the first 0.2mm of movement
    // so by doing an extra step we get the head primed to move so things
    // align correctly.
    if (lastStepDirection != direction)
    {
      motor.step(reverseFudge, direction, DOUBLE);
    }
    
    motor.step(1, direction, DOUBLE);
    lastStepDirection = direction;
    ++stepCount;
  }
  
  void Release()
  {
    motor.release(); 
  }
  
  void PrintStepCount()
  {
    Serial.print(stepCount);
    stepCount = 0;
  }
  
private:
  AF_Stepper motor;
  int lastStepDirection;
  int stepCount;
  int reverseFudge;
  bool polarity;
};

// Create the 2 axes
Axis xAxis(1, 9, false);
Axis yAxis(2, 9, true);

// Mapping from stepper rotation to millimeters
#define STEPS_PER_MM 5
float distancePerStep = 1.0 / (float)STEPS_PER_MM;

// Current position in mm. We can resolve positions up to a single
// step of the motors or 0.2mm (better than .01 inches)
float penX = 0;
float penY = 0;


void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
}

// Do a Bresenham-type line algorithm to move to a new position on the 
// Etch A Sketch
void MovePenTo(float x, float y)
{
  float deltaX = x - penX;
  float deltaY = y - penY;
  
  if (false)
  {
  Serial.println(" ");
  Serial.print("MoveTo("); Serial.print(x); Serial.print(", "); Serial.print(y); Serial.println(")");
  Serial.print("   currentX="); Serial.print(penX); Serial.println(" ");
  Serial.print("     deltaX="); Serial.print(deltaX); Serial.println(" ");
  Serial.print("   currentY="); Serial.print(penY); Serial.println(" ");
  Serial.print("     deltaY="); Serial.print(deltaY); Serial.println(" ");
  }
  
  // If not enough motion to move, then just bail
  if (deltaX == 0 && deltaY == 0)
  {
    return;
  }
  
  Axis *pMajorAxis;
  Axis *pMinorAxis;
  float majorAxisDistance;
  float minorAxisDistance;
  long majorAxisSteps;
  float minorAxisFraction;
  float majorAxisFraction;
  int majorAxisDirection;
  int minorAxisDirection;
  float fraction = 0;
  
  // Determine the longer axis
  if (abs(deltaX) > abs(deltaY))
  {
    // X is the longer (major) axis
    pMajorAxis = &xAxis;
    pMinorAxis = &yAxis;
    
    // Get distance along x axis
    majorAxisDistance = deltaX;
    minorAxisDistance = deltaY;
    
    // minor axis fractional steps
    fraction = penY * STEPS_PER_MM;
    fraction = fraction - (int)fraction;
    
    // major axis fractional steps
    majorAxisFraction = penX * STEPS_PER_MM;
    majorAxisFraction = majorAxisFraction - (int)majorAxisFraction;
  }
  else
  {
    // Y is the longer (major) axis
    pMajorAxis = &yAxis;
    pMinorAxis = &xAxis;
    
    // Get distance along y axis
    majorAxisDistance = deltaY;
    minorAxisDistance = deltaX;
    fraction = penX * STEPS_PER_MM;
    fraction = fraction - (int)fraction;
    
    majorAxisFraction = penY * STEPS_PER_MM;
    majorAxisFraction = majorAxisFraction - (int)majorAxisFraction;    
  }
  
  // Determine direction of major axis stepping
  if (majorAxisDistance < 0)
  {
     majorAxisDirection = FORWARD;
     majorAxisDistance = -majorAxisDistance;
  }
  else
  {
     majorAxisDirection = BACKWARD; 
  }
   
  majorAxisSteps = majorAxisDistance / distancePerStep;
  minorAxisFraction = minorAxisDistance*STEPS_PER_MM / majorAxisSteps;
  if (minorAxisFraction < 0)
  {
     minorAxisDirection = FORWARD;
     minorAxisFraction = -minorAxisFraction;
  }
  else
  {
    minorAxisDirection = BACKWARD;
  }
  
  if (false)
  {
  Serial.print("majorAxisSteps="); Serial.print(majorAxisSteps); Serial.println();
  Serial.print("minorAxisFraction="); Serial.print(minorAxisFraction); Serial.println();
  Serial.print("majorAxisDir="); Serial.println(majorAxisDirection);
  Serial.print("minorAxisDir="); Serial.println(minorAxisDirection);
  }
  
  int steps;
  
  for (steps=0; steps<majorAxisSteps; ++steps)
  {
    pMajorAxis->Step(majorAxisDirection);
    fraction += minorAxisFraction;
    if (fraction >= 1.0)
    {
      pMinorAxis->Step(minorAxisDirection);
      fraction = fraction - 1.0;
    }
  }
  
  // Handle leftover
  float leftover = majorAxisDistance - (majorAxisSteps * distancePerStep);
  leftover = leftover * STEPS_PER_MM;
  if (majorAxisFraction + leftover >= 1.0)
  {
    Serial.println("Leftover step for MAJOR AXIS");
    pMajorAxis->Step(majorAxisDirection);
  }
  fraction = fraction + (leftover * minorAxisFraction);
  if (fraction >= 1.0)
  {
    Serial.println("Leftover step for minor axis");
    pMinorAxis->Step(minorAxisDirection);
  }
  
  
  penX = x;
  penY = y;
}

class SerialCommunications
{
public:
  SerialCommunications()
  {
    parseState = parseState_awaitCommand;
    jumpMode = false;
    Serial.println("A");
  }
  
  void Process()
  {
    switch (parseState)
    {
      case parseState_awaitCommand:
        if (Serial.available() > 0)
        {
          int command = Serial.read();
          
          if (command == 'M')
          {
            // Move to coordinate
            parseState = parseState_awaitXCoord;
          }
          else if (command = 'R')
          {
            // Release motors
            xAxis.Release();
            yAxis.Release();
            Serial.println("A");
          }
          else if (command = 'J')
          {
            // Jump to coordinate. That is, set our internal
            // state to be at this coordinate without moving
            // the Etch-A-Sketch
            parseState = parseState_awaitXCoord;
            jumpMode = true;
          }
          else
          {
            Serial.print("Error: command unknown ");
            Serial.println(command); 
          }
        }
        break;
        
      case parseState_awaitXCoord:
        if (Serial.available() >= 2)
        {
          int x = (Serial.read() << 8) | Serial.read();
          xCoord = (float)x / 100.0;
                      
          parseState = parseState_awaitYCoord;
        }
        break;
        
      case parseState_awaitYCoord:
        if (Serial.available() >= 2)
        {
          int y = (Serial.read() << 8) | Serial.read();
          yCoord = (float)y / 100.0;
            
          if (true)
{          
          Serial.print("MovePenTo(");
          Serial.print(xCoord);
          Serial.print(",");
          Serial.print(yCoord);
          Serial.print(") -- ");
}         
          
          parseState = parseState_awaitCommand;
          
          if (jumpMode)
          {
            Serial.print("JUMP MODE ");
            penX = xCoord;
            penY = yCoord;
            jumpMode = false;
          }
          else
          {
            Serial.print("move ");
            MovePenTo(xCoord, yCoord);
          }
          
            Serial.print("x:"); xAxis.PrintStepCount(); Serial.print(", y:"); yAxis.PrintStepCount(); Serial.print("    ");
            Serial.println("A");
        }
        break;
      }
  }
private:
  enum ParseState
  {
    parseState_awaitCommand,
    parseState_awaitXCoord,
    parseState_awaitYCoord
  } parseState;
  
  float xCoord;
  float yCoord;
  
  bool jumpMode;
};

SerialCommunications comms;

void loop() 
{

  comms.Process();
  
  if (false)
  {
  penX = 34.5;
  penY = 7.5;
  
  MovePenTo(34.5313,7.50003);
  MovePenTo(45.3125,34.375);
  MovePenTo(
  40.3125,10.7813 );
  MovePenTo(
  48.4375,17.5 );
  MovePenTo(
  48.2812,6.40625 );
  MovePenTo(
  49.375,14.6875 );
  MovePenTo(
  50,5 );
  MovePenTo(
  50,5 );
  MovePenTo(
  50.625,14.6875 );
  MovePenTo(
  51.7187,6.40625 );
  MovePenTo(
  51.5625,17.5 );
  
  
  MovePenTo(59.6875,10.7813 );
  MovePenTo( 
  54.6875,34.375); 
  MovePenTo(65.4687,7.50003); MovePenTo(65.4687,7.50003); MovePenTo( 56.4531,35.0172); MovePenTo( 67.7875,13.7297); MovePenTo( 69.6937,24.0984); MovePenTo( 76.7047,15.5); MovePenTo( 72.2203,22.5469); MovePenTo( 78.925,15.5281);
  MovePenTo( 78.925,15.5281); MovePenTo( 73.1765,23.3516); MovePenTo( 79.3375,17.7094); MovePenTo( 72.0875,26.1078); MovePenTo( 82.6297,26.1844);
  MovePenTo( 63.6344,41.0438); MovePenTo( 89.1687,27.386); MovePenTo( 89.1687,27.386); MovePenTo( 64.5734,42.6703); MovePenTo( 86.9406,33.65);
  MovePenTo( 81.7344,42.8172); MovePenTo( 92.6328,40.7375); MovePenTo( 84.6672,43.2531); MovePenTo( 94.3156,42.1859); MovePenTo( 94.3156,42.1859); MovePenTo( 84.8844,44.4828); MovePenTo( 93.2297,44.1234);
  MovePenTo( 82.2781,45.8953); MovePenTo( 90.3047,52.7297); MovePenTo( 66.2015,51.9031); MovePenTo( 94.5406,57.8531); MovePenTo( 94.5406,57.8531); MovePenTo( 65.875,53.7531); MovePenTo( 88.8078,61.2203); MovePenTo( 78.9265,64.8969);
  MovePenTo( 88.6125,70.3078); MovePenTo( 80.8937,67.1156); MovePenTo( 88.9719,72.5); MovePenTo( 88.9719,72.5); MovePenTo( 80.2687,68.1969); MovePenTo( 86.8938,73.2859); MovePenTo( 77.3641,67.6031); MovePenTo( 79.1203,77.9985); MovePenTo( 61.1875,61.8719);
  MovePenTo( 79.0719,84.6469); MovePenTo( 79.0719,84.6469); MovePenTo( 59.7484,63.0797); MovePenTo( 72.5172,83.5406); MovePenTo( 62.5844,80.0063); MovePenTo( 66.525,90.3766); MovePenTo( 62.6656,82.9687); MovePenTo( 65.3906,92.286); MovePenTo( 65.3906,92.286);
  MovePenTo( 61.4906,83.3969); MovePenTo( 63.2953,91.553); MovePenTo( 59.6469,81.075); MovePenTo( 54.3109,90.1672); MovePenTo( 50.9391,66.2859 ); MovePenTo(50,95.2281); MovePenTo( 50,95.2281); MovePenTo( 49.0609,66.2859); MovePenTo( 45.6891,90.1672); MovePenTo( 40.3531,81.075 ); MovePenTo(36.7047,91.5531); MovePenTo( 38.5094,83.3969 ); MovePenTo(34.6094,92.286); MovePenTo( 34.6094,92.286 ); MovePenTo(37.3344,82.9687); MovePenTo( 33.475,90.3766 ); MovePenTo(37.4156,80.0063);
  MovePenTo( 27.4828,83.5406); MovePenTo( 40.2516,63.0797); MovePenTo( 20.9281,84.6469); MovePenTo( 20.9281,84.6469); MovePenTo( 38.8125,61.8719 ); MovePenTo(20.8797,77.9985); MovePenTo( 22.6359,67.6031); MovePenTo( 13.1062,73.2859); MovePenTo( 19.7313,68.1969); MovePenTo( 11.0281,72.5 ); MovePenTo(11.0281,72.5); MovePenTo( 19.1063,67.1156);
  MovePenTo( 11.3875,70.3078); MovePenTo( 21.0734,64.8969); MovePenTo( 11.1922,61.2203); MovePenTo( 34.125,53.7531); MovePenTo( 5.45939,57.8531); MovePenTo( 5.45939,57.8531); MovePenTo( 33.7985,51.9031); MovePenTo( 9.69533,52.7297); MovePenTo( 17.7219,45.8953); MovePenTo( 6.77033,44.1219); MovePenTo( 15.1156,44.4828); MovePenTo( 5.68439,42.1859); MovePenTo( 5.68439,42.1859); MovePenTo( 15.3328,43.2531); MovePenTo( 7.3672,40.7375); MovePenTo( 18.2656,42.8172); MovePenTo( 13.0594,33.65); MovePenTo( 35.4266,42.6703); MovePenTo( 10.8313,27.386); MovePenTo( 10.8313,27.386); MovePenTo( 36.3656,41.0438); MovePenTo( 17.3703,26.1844 ); MovePenTo(27.9125,26.1078); MovePenTo( 20.6625,17.7094 ); MovePenTo(26.8235,23.3516); MovePenTo( 21.075,15.5281); MovePenTo( 21.075,15.5281); MovePenTo( 27.7797,22.5469); MovePenTo( 23.2953,15.5); MovePenTo( 30.3063,24.0984); MovePenTo( 32.2125,13.7297 ); MovePenTo(43.5469,35.0172);

  MovePenTo(34.5313,7.50003);

/*
<polygon class="fil1 str0" points="34.5313,7.50003 45.3125,34.375 40.3125,10.7813 48.4375,17.5 48.2812,6.40625 49.375,14.6875 50,5 50,5 50.625,14.6875 51.7187,6.40625 51.5625,17.5 59.6875,10.7813 54.6875,34.375 65.4687,7.50003 65.4687,7.50003 56.4531,35.0172 67.7875,13.7297 69.6937,24.0984 76.7047,15.5 72.2203,22.5469 78.925,15.5281 78.925,15.5281 73.1765,23.3516 79.3375,17.7094 72.0875,26.1078 82.6297,26.1844 63.6344,41.0438 89.1687,27.386 89.1687,27.386 64.5734,42.6703 86.9406,33.65 81.7344,42.8172 92.6328,40.7375 84.6672,43.2531 94.3156,42.1859 94.3156,42.1859 84.8844,44.4828 93.2297,44.1234 82.2781,45.8953 90.3047,52.7297 66.2015,51.9031 94.5406,57.8531 94.5406,57.8531 65.875,53.7531 88.8078,61.2203 78.9265,64.8969 88.6125,70.3078 80.8937,67.1156 88.9719,72.5 88.9719,72.5 80.2687,68.1969 86.8938,73.2859 77.3641,67.6031 79.1203,77.9985 61.1875,61.8719 79.0719,84.6469 79.0719,84.6469 59.7484,63.0797 72.5172,83.5406 62.5844,80.0063 66.525,90.3766 62.6656,82.9687 65.3906,92.286 65.3906,92.286 61.4906,83.3969 63.2953,91.5531 59.6469,81.075 54.3109,90.1672 50.9391,66.2859 50,95.2281 50,95.2281 49.0609,66.2859 45.6891,90.1672 40.3531,81.075 36.7047,91.5531 38.5094,83.3969 34.6094,92.286 34.6094,92.286 37.3344,82.9687 33.475,90.3766 37.4156,80.0063 27.4828,83.5406 40.2516,63.0797 20.9281,84.6469 20.9281,84.6469 38.8125,61.8719 20.8797,77.9985 22.6359,67.6031 13.1062,73.2859 19.7313,68.1969 11.0281,72.5 11.0281,72.5 19.1063,67.1156 11.3875,70.3078 21.0734,64.8969 11.1922,61.2203 34.125,53.7531 5.45939,57.8531 5.45939,57.8531 33.7985,51.9031 9.69533,52.7297 17.7219,45.8953 6.77033,44.1219 15.1156,44.4828 5.68439,42.1859 5.68439,42.1859 15.3328,43.2531 7.3672,40.7375 18.2656,42.8172 13.0594,33.65 35.4266,42.6703 10.8313,27.386 10.8313,27.386 36.3656,41.0438 17.3703,26.1844 27.9125,26.1078 20.6625,17.7094 26.8235,23.3516 21.075,15.5281 21.075,15.5281 27.7797,22.5469 23.2953,15.5 30.3063,24.0984 32.2125,13.7297 43.5469,35.0172 "/>
*/
  
  // Release motors
  xAxis.Release();
  yAxis.Release();
  while (true)
  {
    ;
  }
  // Calibration test
  MovePenTo(25, 25);
  MovePenTo(25, -25);
  MovePenTo(-25, -25);
  MovePenTo(-25, 25);
  MovePenTo(0, 0);
  
  
  long x;
  int xSteps = 100;
  float radiansStep = (2 * 3.14159) / xSteps;
  float rad = 0;
  
  for (x=0; x<xSteps; ++x)
  {
    float y = 10 * sin(rad);
    
    Serial.print("sin(");
    Serial.print(rad);
    Serial.print(") = ");
    Serial.println(y);
    
    MovePenTo((float)x, (float)y);
    
    rad += radiansStep;
  }
  }
}
