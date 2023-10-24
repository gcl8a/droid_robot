#ifndef __COMM_H
#define __COMM_H

#define DEBUG_SERIAL Serial1
//#define Serial SerialUSB

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

#endif
