// #include "../../include/header.h"
#include <Arduino.h>

// extern int8_t DRIVE_INSTR;
// extern int8_t THETA;

void drive(int8_t, int8_t);
// void calculateSteering(uint8_t, uint8_t);
// void move_forward();
// void move_backward();
void stop();

struct Drive_Instruction
{
  uint8_t speed;
  uint8_t theta;
};
