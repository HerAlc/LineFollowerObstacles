/* This example makes use of
   LineFollower Example of Zumo and MazeSolver.

   As well as the function tu turn found at https://github.com/pvcraven/zumo_32u4_examples/blob/master/TurnExample/TurnExample.ino
*/

#include <Wire.h>
#include <Zumo32U4.h>
#include "TurnSensor.h"

// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.
const uint16_t maxSpeed = 300;
const uint16_t calibrationSpeed = 200;
const uint16_t obstacleSpeed = 400;

Zumo32U4Buzzer buzzer;
Zumo32U4LineSensors lineSensors;
Zumo32U4ProximitySensors proxSensors;
Zumo32U4Motors motors;
Zumo32U4ButtonA buttonA;
Zumo32U4LCD lcd;
L3G gyro;

int16_t lastError = 0;

bool proxFrontActive;

#define NUM_SENSORS 5
unsigned int lineSensorValues[NUM_SENSORS];

// Sets up special characters in the LCD so that we can display
// bar graphs.
void loadCustomCharacters()
{
  static const char levels[] PROGMEM = {
    0, 0, 0, 0, 0, 0, 0, 63, 63, 63, 63, 63, 63, 63
  };
  lcd.loadCustomCharacter(levels + 0, 0);  // 1 bar
  lcd.loadCustomCharacter(levels + 1, 1);  // 2 bars
  lcd.loadCustomCharacter(levels + 2, 2);  // 3 bars
  lcd.loadCustomCharacter(levels + 3, 3);  // 4 bars
  lcd.loadCustomCharacter(levels + 4, 4);  // 5 bars
  lcd.loadCustomCharacter(levels + 5, 5);  // 6 bars
  lcd.loadCustomCharacter(levels + 6, 6);  // 7 bars
}

// Takes calibrated readings of the lines sensors and stores them
// in lineSensorValues.  Also returns an estimation of the line
// position.
uint16_t readSensors()
{
  return lineSensors.readLine(lineSensorValues);
}

// Calibrates the line sensors by turning left and right, then
// displays a bar graph of calibrated sensor readings on the LCD.
// Returns after the user presses A.
static void lineSensorSetup()
{
  lcd.clear();
  lcd.print(F("Line cal"));

  // Delay so the robot does not move while the user is still
  // touching the button.
  delay(1000);

  // We use the gyro to turn so that we don't turn more than
  // necessary, and so that if there are issues with the gyro
  // then you will know before actually starting the robot.

  turnSensorReset();

  // Turn to the left 90 degrees.
  motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
  while ((int32_t)turnAngle < turnAngle45 * 2)
  {
    lineSensors.calibrate();
    turnSensorUpdate();
  }

  // Turn to the right 90 degrees.
  motors.setSpeeds(calibrationSpeed, -calibrationSpeed);
  while ((int32_t)turnAngle > -turnAngle45 * 2)
  {
    lineSensors.calibrate();
    turnSensorUpdate();
  }

  // Turn back to center using the gyro.
  motors.setSpeeds(-calibrationSpeed, calibrationSpeed);
  while ((int32_t)turnAngle < 0)
  {
    lineSensors.calibrate();
    turnSensorUpdate();
  }

  // Stop the motors.
  motors.setSpeeds(0, 0);

  // Show the line sensor readings on the LCD until button A is
  // pressed.
  lcd.clear();
  while (!buttonA.getSingleDebouncedPress())
  {
    readSensors();

    lcd.gotoXY(0, 0);
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
      uint8_t barHeight = map(lineSensorValues[i], 0, 1000, 0, 8);
      printBar(barHeight);
    }
  }

  lcd.clear();
}

void printBar(uint8_t height)
{
  if (height > 8) {
    height = 8;
  }
  const char barChars[] = {' ', 0, 1, 2, 3, 4, 5, 6, 255};
  lcd.print(barChars[height]);
}

/***
 * Turn Zumo the desires degrees
 */
void turn(int degrees) {
  turnSensorReset();
  motors.setSpeeds(-obstacleSpeed, obstacleSpeed);
  int angle = 0;
  do {
    delay(1);
    turnSensorUpdate();
    angle = (((int32_t)turnAngle >> 16) * 360) >> 16;
    lcd.gotoXY(0, 0);
    lcd.print(angle);
    lcd.print(" ");
  } while (angle < degrees);
  motors.setSpeeds(0, 0);
}


// Displays a bar graph of sensor readings on the LCD.
// Returns after the user presses A.
void showReadings()
{
  lcd.clear();

  while (!buttonA.getSingleDebouncedPress())
  {
    lineSensors.readCalibrated(lineSensorValues);

    lcd.gotoXY(0, 0);
    for (uint8_t i = 0; i < NUM_SENSORS; i++)
    {
      uint8_t barHeight = map(lineSensorValues[i], 0, 1000, 0, 8);
      printBar(barHeight);
    }
  }
}

// Prints a line with all the sensor readings to the serial
// monitor.
void printReadingsToSerial()
{
  static char buffer[80];
  sprintf(buffer, "%6d %6d %6d\n",
          proxSensors.countsFrontWithLeftLeds(),
          proxSensors.countsFrontWithRightLeds(),
          proxFrontActive
         );
  Serial.print(buffer);
}

void setup()
{
  /* Configuration 2:
     - 5 line sensors (1, 2, 3, 4, 5)
     - 1 proximity sensor (front)

     For this configuration to work, jumpers on the front sensor
     array must be installed in order to connect pin 4 to DN4 and
     pin 20 to DN2.  This is a good configuration for a line
     follower or maze solver. */
  lineSensors.initFiveSensors();
  proxSensors.initFrontSensor();
  // Wait for button A to be pressed and released.
  lcd.clear();
  lcd.print(F("Press A"));
  lcd.gotoXY(0, 1);
  lcd.print(F("to calib"));
  buttonA.waitForButton();
  turnSensorSetup();
  lineSensorSetup();
  loadCustomCharacters();

  // Play a little welcome song
  buzzer.play(">g32>>c32");

  showReadings();
  // Play music and wait for it to finish before we start driving.
  lcd.clear();
  lcd.print(F("Go!"));
  buzzer.play("L16 cdegreg4");
  while (buzzer.isPlaying());
}

void loop()
{
  // Get the position of the line.  Note that we *must* provide
  // the "lineSensorValues" argument to readLine() here, even
  // though we are not interested in the individual sensor
  // readings.
  int16_t position = lineSensors.readLine(lineSensorValues);

  proxSensors.read();

  printReadingsToSerial();

  lcd.clear();
  
  int l = proxSensors.countsFrontWithLeftLeds();
  int r = proxSensors.countsFrontWithRightLeds();

  // Our "error" is how far we are away from the center of the
  // line, which corresponds to position 2000.
  int16_t error = position - 2000;

  // Get motor speed difference using proportional and derivative
  // PID terms (the integral term is generally not very useful
  // for line following).  Here we are using a proportional
  // constant of 1/4 and a derivative constant of 6, which should
  // work decently for many Zumo motor choices.  You probably
  // want to use trial and error to tune these constants for your
  // particular Zumo and line course.
  int16_t speedDifference = error / 1 + 1 * (error - lastError);

  lastError = error;

  // Get individual motor speeds.  The sign of speedDifference
  // determines if the robot turns left or right.
  int16_t leftSpeed = (int16_t)maxSpeed + speedDifference;
  int16_t rightSpeed = (int16_t)maxSpeed - speedDifference;

  // Constrain our motor speeds to be between 0 and maxSpeed.
  // One motor will always be turning at maxSpeed, and the other
  // will be at maxSpeed-|speedDifference| if that is positive,
  // else it will be stationary.  For some applications, you
  // might want to allow the motor speed to go negative so that
  // it can spin in reverse.
  leftSpeed = constrain(leftSpeed, 0, (int16_t)maxSpeed);
  rightSpeed = constrain(rightSpeed, 0, (int16_t)maxSpeed);

  motors.setSpeeds(leftSpeed, rightSpeed);
  if (l == 6 && r == 6) {
    buzzer.play("L5ere");
    motors.setSpeeds(0, 0);
    delay(3000);
    proxSensors.read();
    int l = proxSensors.countsFrontWithLeftLeds();
    int r = proxSensors.countsFrontWithRightLeds();

    if (l == 6 && r == 6) {
      buzzer.play("L5o3b");
      while(buzzer.isPlaying());
      turn(90);
      turn(90);
    }
  }
}
