#ifndef __SABERTOOTH_H
#define __SABERTOOTH_H

#define EMERGENCY_PIN 3

#include "motor_driver.h"
#include "wiring_private.h" 

#define RC_LOW    100
#define RC_CENTER 185
#define RC_HIGH   270

#define ADDRESS_BYTE 128

enum COMM_METHOD {COMM_NONE, COMM_RC, COMM_PACKET_SERIAL, COMM_PWM};

//declare a UART SERCOM for communicating through packet serial
Uart Serial2 (&sercom2, 3, 2, SERCOM_RX_PAD_1, UART_TX_PAD_2);
void SERCOM2_Handler()
{
  Serial2.IrqHandler();
}

class Sabertooth : public MotorDriver
{
protected:
  uint8_t commMode = COMM_NONE;
  
public:
  Sabertooth(void)
  {
    //don't set pins or registers here since this gets called before standard Arduino setup
    //use SetCommMode instead
  }

  void Init(uint8_t mode)
  {
    DEBUG_SERIAL.println("Sabertooth::Init");
    MotorDriver::Init();
    commMode = mode;
 
    if(commMode == COMM_RC)
    {}

    else if(commMode == COMM_PACKET_SERIAL)
    {
      Serial2.begin(9600);
  
      //Assign pin 2 SERCOM functionality -- why does this have to be after begin() -- who dropped the ball on that one?
      pinPeripheral(2, PIO_SERCOM);
    }

    else commMode = COMM_NONE;
    DEBUG_SERIAL.println("/Sabertooth::Init");
  }

  void EmergencyStop(void)
  {
    if(commMode == COMM_PACKET_SERIAL)
    {
      //in packet mode, we can disable via PE6
//      pinMode(EMERGENCY_PIN, OUTPUT);
//      digitalWrite(EMERGENCY_PIN, LOW);
      //DDRE |= (1 << PE6); //make it an output
      //PORTE &= ~(1 << PE6); //bring it low to stop sabertooth
    }

    MotorDriver::EmergencyStop();
  }

protected:  
  void SendPowers(int16_t powerA, int16_t powerB)
  {   
    if(commMode == COMM_RC)
    {}

    else if(commMode == COMM_PACKET_SERIAL)
    {
      uint8_t valueA = (powerA > 0) ? powerA : -powerA;
      if(valueA > 127) valueA = 127;
      
      uint8_t cmdA = 0;
      cmdA = (powerA > 0) ? 4 : 5;

      uint8_t valueB = (powerB > 0) ? powerB : -powerB;
      if(valueB > 127) valueB = 127;
      
      uint8_t cmdB = 0;
      cmdB = (powerB > 0) ? 0 : 1;

      Serial2.write(ADDRESS_BYTE);
      Serial2.write(cmdA);
      Serial2.write(valueA);
      Serial2.write((ADDRESS_BYTE + cmdA + valueA) & 0x7F);
      
      Serial2.write(ADDRESS_BYTE);
      Serial2.write(cmdB);
      Serial2.write(valueB);
      Serial2.write((ADDRESS_BYTE + cmdB + valueB) & 0x7F);
    }
  }

  uint16_t PowerToRC(int16_t power)
  {
    uint16_t rc = RC_CENTER + power / 4;
    return constrain(rc, RC_LOW, RC_HIGH);
  }
};

#endif

 
