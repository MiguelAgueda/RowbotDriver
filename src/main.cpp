#include 'header.h'
#include <Driver.h>
#include <RemoteControl.h>
#include <I2CComm.h>

const int BAUD = 9600;

void setup()
{
    Serial.begin(BAUD);
    Serial.println("Program Ready\n");

    setup_radio();
}

void loop()
{
    Radio_Package current_data = read_radio(); // Get latest data from radio.
    if (current_data.tSwitch1)                 // Drive using remote control.
    {
        int8_t joystick_x = map(current_data.j1PotX, 0, 255, -127, 127);
        int8_t joystick_y = map(current_data.j1PotY, 0, 255, -127, 127);

        if (current_data.tSwitch2) // Save scan data.
        {
            GLOBAL_OUT_PACK.SaveScanData = true;
            GLOBAL_OUT_PACK.joystick_x = joystick_x;
            GLOBAL_OUT_PACK.joystick_y = joystick_y;
        }
        else
        {
            GLOBAL_OUT_PACK.SaveScanData = false;
        }

        GLOBAL_OUT_PACK.DriveAutonomously = false;
        drive(joystick_x, joystick_y); // Drive using joystick position.
    }
    else // Drive autonomously.
    {
        GLOBAL_OUT_PACK.DriveAutonomously = true;
    }
}
