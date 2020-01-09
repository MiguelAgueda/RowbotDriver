#include <Arduino.h>
#include <Wire.h>

int loop_counter = 0;
unsigned long time_now = 0;

// Bytes coming from RockPi or a drive-manager device.
#define ADDR 0x04 // Address for this device over I2C.
#define BAUD 9600

// Pinouts
const int DriveEn = 3;
const int SteerEn = 2;
const int DriveDir = 5;
const int SteerDir = 4;
const int DrivePWM = 11;
const int SteerPWM = 10;
const int SteerPot = A0;

// Variables for motor control.
const unsigned long DriveAcceleration = 250 ; // Delay (ms) for controlling acceleration.
const unsigned long SteerAcceleration = 5;  // Delay (ms) for controlling acceleration.
int RightTurnPWM = 0;
int LeftTurnPWM = 0;
int ForwardPWM = 0;
int BackwardPWM = 0;
int SteerPotValue = 882;          // potentiometer reading from steering column.
const int MinSteerAngle = 666;    // Lower limit for steering pot value.
const int MidSteerAngle = 882;    // Value when wheels are pointed straight.
const int MaxSteerAngle = 1000;    // Upper limit for steering pot value.
const int SteeringDeviation = 10; // Deviation allowed from center of steering.

// Byte communication code.
int STEERING_INSTR;
int DRIVE_INSTR;
#define FORWARD_CODE 2
#define BACKWARD_CODE 3
#define STOP_CODE 4
#define LEFT_CODE 5
#define RIGHT_CODE 6
#define CENTER_CODE 7
#define TERMINATE_CODE 254

bool FORWARD_FLAG = false;
bool BACKWARD_FLAG = false;
bool STOP_FLAG = false;
bool RIGHT_FLAG = false;
bool LEFT_FLAG = false;
bool CENTER_FLAG = false;
bool PROGRAM_STOPPED = true; // Start with program waiting for 'go.'

void initOutputs()
{
  // Set motor driver pinouts as output.
  pinMode(SteerEn, OUTPUT);
  pinMode(SteerDir, OUTPUT);
  pinMode(SteerPWM, OUTPUT);
  pinMode(DriveEn, OUTPUT);
  pinMode(DriveDir, OUTPUT);
  pinMode(DrivePWM, OUTPUT);
  // Analog pin as input. Not necessary, just as reference.
  pinMode(SteerPot, INPUT);
}

int increment(int returnVal, int incrementLimit)  // Increment and return any number less than 254.
{
  if (returnVal < incrementLimit)
    return ++returnVal;
  else
    return returnVal;
  
}

void handleIncrementOf(char varToIncrement)
{
  // Increment parameter value by one.
  switch (varToIncrement)
  {
  case 'f': // Increment ForwardPWM.
  {
    BackwardPWM = 0;
    ForwardPWM = increment(ForwardPWM, 65);
    break;
  }
  case 'b':
  {
    ForwardPWM = 0;
    BackwardPWM = increment(BackwardPWM, 100);
    break;
  }
  case 'l': // Increment LeftTurnPWM.
  {
    RightTurnPWM = 0;
    LeftTurnPWM = increment(LeftTurnPWM, 255);
    break;
    // LeftTurnPWM = incrementor(LeftTurnPWM);
  }
  case 'r':
  {
    LeftTurnPWM = 0;
    RightTurnPWM = increment(RightTurnPWM, 255);
    break;
  }
  }  // End switch-statement.
}

void stopSteering()
{
  digitalWrite(SteerEn, LOW);
  digitalWrite(SteerDir, LOW);
  analogWrite(SteerPWM, 0);
}

void moveForwards()
{
  // Forward signal to motor driver.
  digitalWrite(DriveEn, HIGH);
  digitalWrite(DriveDir, HIGH);
  // Check if enough time has passed to increment PWM value.
  if (millis() > time_now + DriveAcceleration)
  {
    time_now = millis();
    handleIncrementOf('f');
  }

  analogWrite(DrivePWM, ForwardPWM);
}

void moveBackwards()
{
  // Backward signal to motor driver.
  digitalWrite(DriveEn, HIGH);
  digitalWrite(DriveDir, LOW);
  // Check if enough time has passed to increment PWM value.
  if (millis() > time_now + DriveAcceleration)
  {
    time_now = millis();
    handleIncrementOf('b');
  }

  analogWrite(DrivePWM, BackwardPWM);
}

void turnRight()
{
  // Right turn signal to motor driver.
  if (analogRead(SteerPot) <= MaxSteerAngle) // If wheels not at max:
  {
    Serial.println("Wheels not at max, turning right!");
    digitalWrite(SteerEn, HIGH);
    digitalWrite(SteerDir, HIGH);
    // Check if enough time has passed to increment PWM value.
    if (millis() > time_now + SteerAcceleration)
    {
      time_now = millis();
      handleIncrementOf('r');
    }

    analogWrite(SteerPWM, RightTurnPWM);

  } // End if-statement.
  else
    stopSteering();

}

void turnLeft()
{
  // Left turn signal to motor driver.
  if (analogRead(SteerPot) >= MinSteerAngle) // If wheels not at min:
  {
    Serial.println("Wheels not at min, turning left!");
    digitalWrite(SteerEn, HIGH);
    digitalWrite(SteerDir, LOW);
    // Check if enough time has passed to increment PWM value.
    if (millis() > time_now + SteerAcceleration)
    {
      time_now = millis();
      handleIncrementOf('l');
    }

    analogWrite(SteerPWM, LeftTurnPWM);
  } // End if-statement.
  else
    stopSteering();
  
}

void resetSteering()
{
  // Center wheels when not turning left or right.
  int SteerPotReading = analogRead(SteerPot);
  if (SteerPotReading < MidSteerAngle - SteeringDeviation)
    turnRight();
  else if (SteerPotReading > MidSteerAngle + SteeringDeviation)
    turnLeft();

  else // Stop steering.
  {
    Serial.println("Steering Centered, Now Stopping.");
    digitalWrite(SteerEn, LOW);
    digitalWrite(SteerDir, LOW);
    analogWrite(SteerPWM, 0);
  }

  Serial.println("(STEERING RESET)");
}

void resetDrive()
{
  // Stop drive motor when not going forwards or backwards.
  digitalWrite(DriveEn, LOW);
  digitalWrite(DriveDir, LOW);
  analogWrite(DrivePWM, 0);
}

void emergencyStop()
{
  digitalWrite(DriveEn, LOW);
  digitalWrite(SteerEn, LOW);
  digitalWrite(DriveDir, LOW);
  digitalWrite(SteerDir, LOW);
  analogWrite(DrivePWM, 0);
  analogWrite(SteerPWM, 0);
}

void handleIncomingByte()
{

  // SteerPotValue = analogRead(SteerPot);  // Save current potentiometer reading.
  // Use truth values from handleIncomingByte to control motor driver.
  if (DRIVE_INSTR == FORWARD_CODE)
  {
    Serial.println("Going Forwards");
    moveForwards();
  }
  else if (DRIVE_INSTR == BACKWARD_CODE)
  {
    Serial.println("Going Backwards");
    moveBackwards();
  }
  else if (DRIVE_INSTR == STOP_CODE)
  {
    Serial.println("Resetting Engine");
    resetDrive();
  }

  if (STEERING_INSTR == LEFT_CODE)
  {
    Serial.println("Turning Left");
    turnLeft();
  }
  else if (STEERING_INSTR == RIGHT_CODE)
  {
    Serial.println("Turning Right");
    turnRight();
  }
  else if (STEERING_INSTR == CENTER_CODE)
  {
    Serial.println("Resetting Steering");
    resetSteering();
  }

  if ((STEERING_INSTR == TERMINATE_CODE) && (DRIVE_INSTR == TERMINATE_CODE))
    Serial.println("Emergency Stop Initiated");
  emergencyStop();
}

void readInInstr(int numBytes)
{
  if (numBytes == 3)
  {
    if (Wire.read() == 0)
    {                               // First byte should be 0,
      STEERING_INSTR = Wire.read(); // followed by steering byte,
      DRIVE_INSTR = Wire.read();    // then drive byte.
      Serial.print("STEERING_INSTR: ");
      Serial.println(STEERING_INSTR);
    }
  }
}

void receiveEvent(int numBytes)
{
  readInInstr(numBytes);
  // If terminate signal is not received;
  if ((STEERING_INSTR == TERMINATE_CODE) || (DRIVE_INSTR == TERMINATE_CODE))
  {
    Serial.println("\nTermination Signal Received.");
    emergencyStop();
    FORWARD_FLAG = false;
    BACKWARD_FLAG = false;
    STOP_FLAG = false;
    RIGHT_FLAG = false;
    LEFT_FLAG = false;
    CENTER_FLAG = false;
    PROGRAM_STOPPED = true;
  }
  else
  {
    PROGRAM_STOPPED = false;
    if (STEERING_INSTR == RIGHT_CODE)
    {
      RIGHT_FLAG = true;
      LEFT_FLAG = false;
      CENTER_FLAG = false;
    }
    else if (STEERING_INSTR == LEFT_CODE)
    {
      RIGHT_FLAG = false;
      LEFT_FLAG = true;
      CENTER_FLAG = false;
    }
    else if (STEERING_INSTR == CENTER_CODE)
    {
      RIGHT_FLAG = false;
      LEFT_FLAG = false;
      CENTER_FLAG = true;
    }

    if (DRIVE_INSTR == FORWARD_CODE)
    {
      FORWARD_FLAG = true;
      BACKWARD_FLAG = false;
      STOP_FLAG = false;
    }
    else if (DRIVE_INSTR == BACKWARD_CODE)
    {
      FORWARD_FLAG = false;
      BACKWARD_FLAG = true;
      STOP_FLAG = false;
    }
    else if (DRIVE_INSTR == STOP_CODE)
    {
      FORWARD_FLAG = false;
      BACKWARD_FLAG = false;
      STOP_FLAG = true;
    }
  }
}

void handlePinOutputs()
{
  if (FORWARD_FLAG)
    moveForwards();
  else if (BACKWARD_FLAG)
    moveBackwards();
  else if (STOP_FLAG)
    resetDrive();

  if (LEFT_FLAG)
    turnLeft();
  else if (RIGHT_FLAG)
    turnRight();
  else if (CENTER_FLAG)
    resetSteering();
}

void printTruthValues()
{
  Serial.print("FWD: ");
  Serial.println(FORWARD_FLAG);
  Serial.print("BWD: ");
  Serial.println(BACKWARD_FLAG);
  Serial.print("CNT: ");
  Serial.println(CENTER_FLAG);
}

void setup()
{
  Serial.begin(BAUD);
  initOutputs();    // Initiate output pins.
  Wire.begin(ADDR); // Join I2C bus on address ADDR.
  Serial.println("Connected \nProgram Ready");
  Wire.onReceive(receiveEvent); // Function to call when byte is received.
}

void loop()
{
  // handleIncomingByte();
  while (!PROGRAM_STOPPED)
  {
    handlePinOutputs();
    Serial.print("Potentiometer Value: ");
    Serial.println(analogRead(SteerPot));
    // printTruthValues();
  }
}
