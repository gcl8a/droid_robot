#ifndef __COMM_H
#define __COMM_H

#include <RFM69.h>
#include <vector.h>

#define ivector TVector<int16_t>

#define DEBUG_SERIAL Serial1

String debugString;
boolean CheckDebugSerial(void) //returns true upon newline, not just a character
{
  while(DEBUG_SERIAL.available()) 
  {
    // get incoming byte:
    char inChar = DEBUG_SERIAL.read();
    debugString += inChar;
 
    if (inChar == '\n') return true;
    else return false; 
  }
  
  return false;
}

RFM69 radio(10, 5, false, digitalPinToInterrupt(5));

void InitRadio(void)
{
  radio.initialize(RF69_915MHZ, 0, 180);
  radio.setHighPower(); // Always use this for RFM69HCW
  radio.setPowerLevel(0); //lowers the power, which is fine for the short distances here
}

bool CheckRadio(void)
{
  bool retVal = false;

  DEBUG_SERIAL.println("radio");

  if(radio.receiveDone()) retVal = true;
  
  DEBUG_SERIAL.println("done");

  return retVal;
}

#define CMD_JOYSTICK 0x01

ivector HandleRadio(void)
{
  ivector retVector;

  //copy radio data to a local array
  uint8_t length = radio.DATALEN;
  uint8_t data[length];
  for(int i = 0; i < length; i++)
  {
    data[i] = radio.DATA[i];
  }
  
  if(length >=2) //valid packet -- at least a command
  {
    uint16_t cmd_type;
    memcpy(&cmd_type, data, sizeof(cmd_type));

    switch(cmd_type)
    {
      case CMD_JOYSTICK:
        if(length != 8) return retVector;  //not the right sized packet: return zero length vector
        retVector = ivector(6);
        memcpy(&retVector[0], &data[2], 6);
        return retVector;
    }
  }

  return retVector;
}

#endif
