#include <Driver.h>

// Define Pins.
const int L_DIR = 2;
const int L_PWM = 3;
const int L_EN = 4;
const int R_DIR = 5;
const int R_PWM = 6;
const int R_EN = 7;
int L_PWM_VAL = 0;
int R_PWM_VAL = 0;

// Define drive codes.
const int8_t STOP_FLAG = 0;
const int8_t FWD_FLAG = 1;
const int8_t BWD_FLAG = 2;

void calculatePWM(int8_t joystick_x, int8_t joystick_y)
{
    if (joystick_y >= 0)
    {
        R_PWM_VAL = joystick_y - (joystick_x / 4);
        L_PWM_VAL = joystick_y + (joystick_x / 4);
        ;
    }
    else // Steering switches when moving backwards.
    {
        R_PWM_VAL = joystick_y + (joystick_x / 4);
        L_PWM_VAL = joystick_y - (joystick_x / 4);
    }
}

uint8_t isPositive(int i)
{
    if (i >= 0)
        return 1;
    else
        return 0;
}

void calculatePinouts()
{
    /*
    Perform proper pinout operations depending on calculated PWM values.
    For positive PWM values, the direction is meant to be forward.
    For negative PWM values, the direction is meant to be backward.
    The DIR value follows the following key; 0: Backwards, 1: Forwards.
    */
    uint8_t right_side_dir = isPositive(R_PWM_VAL);
    uint8_t left_side_dir = isPositive(L_PWM_VAL);

    // Debugging.
    Serial.print("Left DIR: ");
    Serial.print(left_side_dir);
    Serial.print("\tRight DIR: ");
    Serial.println(right_side_dir);
    R_PWM_VAL = abs(R_PWM_VAL);
    L_PWM_VAL = abs(L_PWM_VAL);
    // Handle outputs for right wheels.
    digitalWrite(R_EN, HIGH);            // En pin will always be high when in motion.
    digitalWrite(R_DIR, right_side_dir); // R_DIR depends on sign of PWM.
    analogWrite(R_PWM, abs(R_PWM_VAL));  // Write absolute value of PWM, no negatives.
    // Handle outputs for left wheels.
    digitalWrite(L_EN, HIGH);           // En pin will always be high when in motion.
    digitalWrite(L_DIR, left_side_dir); // L_DIR depends on sign of PWM.
    analogWrite(L_PWM, abs(L_PWM_VAL)); // Write absolute value of PWM, no negatives.
}

void move_forward()
{
    digitalWrite(R_EN, HIGH);
    digitalWrite(R_DIR, HIGH);
    digitalWrite(L_EN, HIGH);
    digitalWrite(L_DIR, HIGH);
    analogWrite(R_PWM, R_PWM_VAL);
    analogWrite(L_PWM, L_PWM_VAL);
    //   analogWrite(R_PWM, 255);
    //   analogWrite(L_PWM, 255);
}

void move_backward()
{
    digitalWrite(R_EN, HIGH);
    digitalWrite(R_DIR, LOW);
    digitalWrite(L_EN, HIGH);
    digitalWrite(L_DIR, LOW);
    analogWrite(R_PWM, R_PWM_VAL);
    analogWrite(L_PWM, L_PWM_VAL);
}

void stop()
{
    digitalWrite(R_EN, LOW);
    digitalWrite(R_DIR, LOW);
    digitalWrite(L_EN, LOW);
    digitalWrite(L_DIR, LOW);
    analogWrite(R_PWM, 0);
    analogWrite(L_PWM, 0);
}

void drive(int8_t joystick_x, int8_t joystick_y)
{
    calculatePWM(joystick_x, joystick_y);
    calculatePinouts();
}