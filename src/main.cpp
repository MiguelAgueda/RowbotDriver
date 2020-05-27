#include "header.h"
#include <Driver.h>
#include <RemoteControl.h>
#include <I2CComm.h>

const int BAUD = 9600;

void setup()
{
    Serial.begin(BAUD);
    setup_i2c();
    setup_radio();
    Serial.println("Program Ready\n");
}

void loop()
{
    Radio_Package current_data = read_radio(); // Get latest data from radio.
    if (current_data.tSwitch1)                 // Drive using remote control.
    {
        GLOBAL_OUT_PACK.DriveAutonomously = false;
        // GLOBAL_OUT_PACK.joystick_x = current_data.j1PotX;
        // GLOBAL_OUT_PACK.joystick_y = current_data.j1PotY;
        // Serial.print("Auto: ");
        // Serial.println(GLOBAL_OUT_PACK.DriveAutonomously);
        int8_t joystick_x = map(current_data.j1PotX, 0, 255, -127, 127);
        int8_t joystick_y = map(current_data.j1PotY, 0, 255, -127, 127);
        GLOBAL_OUT_PACK.joystick_x = joystick_x;
        GLOBAL_OUT_PACK.joystick_y = joystick_y;

        if (current_data.tSwitch2) // Save scan data.
            GLOBAL_OUT_PACK.SaveScanData = false;
        else
            GLOBAL_OUT_PACK.SaveScanData = true;

        // GLOBAL_OUT_PACK.DriveAutonomously = false;
        drive(joystick_x, joystick_y); // Drive using joystick position.
    }
    else // Drive autonomously.
    {
        GLOBAL_OUT_PACK.DriveAutonomously = true;
    }
}
