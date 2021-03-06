#include <Arduino.h>
#include <Wire.h>

struct I2C_OUT_PACK
{
  bool DriveAutonomously = false;
  bool SaveScanData = false;
  bool Shutdown = false;
  uint8_t joystick_x = 0;
  uint8_t joystick_y = 0;
};

struct I2C_IN_PACK
{
  uint8_t virt_joystick_x = 0;
  uint8_t virt_joystick_y = 0;
};

extern I2C_IN_PACK GLOBAL_IN_PACK;
extern I2C_OUT_PACK GLOBAL_OUT_PACK;

void setup_i2c();