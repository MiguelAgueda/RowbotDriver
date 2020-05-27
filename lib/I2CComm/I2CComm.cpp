#include <I2CComm.h>

const int ADDR = 0x04; // Address for this device over I2C.
I2C_IN_PACK GLOBAL_IN_PACK;
I2C_OUT_PACK GLOBAL_OUT_PACK;

void readI2CData(int numBytes)
{
  if (numBytes == sizeof(I2C_IN_PACK))
  {
    GLOBAL_IN_PACK.virt_joystick_x = Wire.read();
    GLOBAL_IN_PACK.virt_joystick_y = Wire.read();
  }
  else
  {
    Serial.println("Bytes recieved do not match data structure.");
  }
}

void returnI2CData()
{
  // int8_t buf_to_write[5] = {0};
  // buf_to_write[0] = GLOBAL_OUT_PACK.DriveAutonomously;
  // buf_to_write[1] = GLOBAL_OUT_PACK.SaveScanData;
  // buf_to_write[2] = GLOBAL_OUT_PACK.Shutdown;
  // buf_to_write[3] = GLOBAL_OUT_PACK.joystick_x;
  // buf_to_write[4] = GLOBAL_OUT_PACK.joystick_y;
  Wire.write((byte)GLOBAL_OUT_PACK.DriveAutonomously);
  Wire.write((byte)GLOBAL_OUT_PACK.SaveScanData);
  Wire.write((byte)GLOBAL_OUT_PACK.Shutdown);
  Wire.write((int)GLOBAL_OUT_PACK.joystick_x);
  Wire.write((byte)GLOBAL_OUT_PACK.joystick_y);
  // Wire.write((byte)127);
}

void setup_i2c()
{
  Serial.begin(9600);
  Wire.begin(ADDR);
  Wire.onReceive(readI2CData);
  Wire.onRequest(returnI2CData);
}