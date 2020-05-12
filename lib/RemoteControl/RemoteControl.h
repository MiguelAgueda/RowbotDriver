#include <Arduino.h>
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

struct Radio_Package // Package for communication with remote control.
{
  uint8_t j1PotX;
  uint8_t j1PotY;
  uint8_t j1Button;
  uint8_t j2PotX;
  uint8_t j2PotY;
  uint8_t j2Button;
  uint8_t pot1;
  uint8_t pot2;
  uint8_t tSwitch1;
  uint8_t tSwitch2;
  uint8_t button1;
  uint8_t button2;
  uint8_t button3;
  uint8_t button4;
};

void setup_radio();
void resetData();
Radio_Package read_radio();
