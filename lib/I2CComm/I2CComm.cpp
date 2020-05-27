#include <I2CComm.h>

const int ADDR = 0x04; // Address for this device over I2C.

void setup_i2c()
{
  Serial.begin(9600);
  Wire.begin(ADDR);
  Wire.onReceive(readI2CData);
  Wire.onRequest(returnI2CData);
}

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
  Wire.write(GLOBAL_OUT_PACK.DriveAutonomously);
  Wire.write(GLOBAL_OUT_PACK.SaveScanData);
  Wire.write(GLOBAL_OUT_PACK.Shutdown);
  Wire.write(GLOBAL_OUT_PACK.joystick_x);
  Wire.write(GLOBAL_OUT_PACK.joystick_y);
}