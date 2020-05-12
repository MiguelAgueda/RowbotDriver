#include <Wire.h>
#include <header.h>
#include <Driver.h>
#include <RemoteControl.h>

#define ADDR 0x04 // Address for this device over I2C.
const int BAUD = 9600;
// Radio_Package data;

void handleIncomingByte(int numBytes)
{
    Serial.println("Bytes Received.");
    if (numBytes == 3)
    {
        if (Wire.read() == 0) // First byte sent should be 0.
        {
            // THETA = Wire.read();
            // DRIVE_INSTR = Wire.read();
            // Serial.println(DRIVE_INSTR);
            // Serial.println(THETA);
        }
        else
            stop();
    }
}

void setup()
{
    Serial.begin(BAUD);
    Serial.println("Program Ready\n");
    Wire.begin(ADDR);
    Wire.onReceive(handleIncomingByte);
    setup_radio();
}

void loop()
{
    Radio_Package current_data = read_radio(); // Get latest data from radio.
    if (current_data.tSwitch1)                 // Drive using remote control.
    {
        int8_t joystick_x = map(current_data.j1PotX, 0, 255, -127, 127);
        int8_t joystick_y = map(current_data.j1PotY, 0, 255, -127, 127);

        drive(joystick_x, joystick_y); // Drive using joystick position.
    }
    else // Drive autonomously.
    {
        // Send autonomous flag to RPi.
        // Begin accepting driving commands based on LiDAR readings.
        // drive(current_data.j1PotX, current_data.j1PotY);
    }
}
