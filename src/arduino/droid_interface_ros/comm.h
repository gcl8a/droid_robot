#ifndef __COMM_H
#define __COMM_H

#include <RFM69.h>

#define Serial SerialUSB
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

RFM69 radio(10, 4, false, 4);
void InitRadio(void)
{
  radio.initialize(RF69_915MHZ, 0, 180);
  radio.setHighPower(); // Always use this for RFM69HCW
  radio.setPowerLevel(0); //lowers the power, which is fine for the short distances here
}

bool CheckRadio(void)
{
  bool retVal = false;
  if(radio.receiveDone())
  {
    if(radio.DATALEN == 6) //joystick packet
    {
      
    }
  }
}

void HandleRadio(void)
{
  
}

#endif
