/* 
 * File:   robot.h
 * Author: greg
 *
 * Created on 3. november 2012, 22:53
 */

#ifndef __ROBOT_H
#define __ROBOT_H

#include "comm.h"
#include "sabertooth.h"
//#include "imu.h"

#include "controller.h"

 /*
 * Calculation for RADIANS_PER_TICK:
 * 7 magnets = 28 ticks / rotation
 * Gear ratio of 71:1 -> 1988 ticks / rotation 
 * or 2pi / 1988 = 0.00316 rad / tick
 * 
 */

//#define RADIANS_PER_TICK (0.00316)
//#define RADIUS_WHEEL (0.081)
//const uint16_t TICKS_PER_METER = 3906;
//const float ROBOT_RADIUS = 0.227;
//const float ROBOT_RADIUS_IN_TICKS = 887; //(ROBOT_RADIUS * TICKS_PER_METER)

/*
 * Holds all the kinematics, both pose and velocities
 */
struct Pose
{
  float x = 0; //ticks
  float y = 0; //ticks
  float cos_theta = 1; //unitless
  float sin_theta = 0; //unitless

  float u = 0; //ticks / period
  float omega = 0; //rad / period
};

class UGV
{  
protected:
  CMD_SRC cmdSource = CMD_SRC_ROS; //start out using ROS
  
  ivector motorSpeeds;
  ivector effort;
  
  Sabertooth driver;
  MotionController controller;

//  float base_radius = 0.227;
//
//  Pose currPose;

  uint8_t prevZbutton = 1;

public:
  UGV(void) : motorSpeeds(2), effort(2) //sensorL(REFLECTANCE_L), sensorR(REFLECTANCE_R)
  {}
  
  virtual void Init(void)
  {
    DEBUG_SERIAL.println("UGV::Init");

    //opMode = IDLE;
    driver.Init(COMM_PACKET_SERIAL);
    controller.Init();

    DEBUG_SERIAL.println("/UGV::Init");
  }

  void Idle(void)
  {
    driver.FullStop();
  }

  virtual void MainLoop(void)
  {  
    if(readyToPID) 
    {
      ProcessPID();
      readyToPID = 0;
    }

    if(CheckRadio())
    {
      if(HandleRadio()) lastRadioRec = millis();
    }

    if(cmdSource == CMD_SRC_RADIO)
    {
      if(millis() - lastRadioRec > RADIO_TIMEOUT)
      {
        Idle();  
      }
    }
  }

  virtual void ProcessPID(void)
  {
//    DEBUG_SERIAL.print("readyToPID");
//    DEBUG_SERIAL.print('\n');
      //////////!!!!!!!!!
      motorSpeeds = controller.CalcMotorSpeeds(); //wheel velocity is ticks / period

      DEBUG_SERIAL.print(millis());
      DEBUG_SERIAL.print('\t');

      DEBUG_SERIAL.print(motorSpeeds[0]);
      DEBUG_SERIAL.print('\t');
      DEBUG_SERIAL.print(motorSpeeds[1]);
      DEBUG_SERIAL.print('\t');

      //UpdateKinematics(wheelSpeeds[0], wheelSpeeds[1]);

//      if(cmdMode == CMD_VEL)
      {
        effort = controller.CalcEffort();
        CommandMotors(effort);
  
        DEBUG_SERIAL.print(effort[0]);
        DEBUG_SERIAL.print('\t');
        DEBUG_SERIAL.print(effort[1]);
        DEBUG_SERIAL.print('\n');
      }
  }
  
//  void SetTargetWheelSpeeds(float left, float right) //in m / s
//  {
//    //do calculations in m/s
//    float speedLeft = left;
//    float speedRight = right;
//
//    //and convert to ticks per frame (use floating point math, but speeds are integers)
//    speedLeft *= (float)TICKS_PER_METER / (float)LOOP_RATE;
//    speedRight *= (float)TICKS_PER_METER / (float)LOOP_RATE;
//
//    //integer vector -- speeds are in integral numbers of ticks -- ignore the digitization error for now...
//    ivector speed(2); 
//    speed[0] = speedLeft;
//    speed[1] = speedRight;
//
////    DEBUG_SERIAL.print(speedLeft);
////    DEBUG_SERIAL.print('\t');
////    DEBUG_SERIAL.print(speedLeft);
////    DEBUG_SERIAL.print('\n');
//    
//        
//    controller.SetTarget(speed);
//  }

  void SetTargetMotorSpeeds(float left, float right) //in ticks / period
  {
    //integer vector -- speeds are in integral numbers of ticks -- ignore the digitization error for now...
    ivector speed(2); 
    speed[0] = left;
    speed[1] = right;
        
    controller.SetTarget(speed);
  }

//  void SetTargetSpeed(float u, float omega) //in m / s
//  {
//    //do calculations in m/s
//    float speedLeft = u - omega * ROBOT_RADIUS;
//    float speedRight = u + omega * ROBOT_RADIUS;
//
//    SetTargetWheelSpeeds(speedLeft, speedRight);
//  }

//  dvector EstimateWheelSpeeds(void)
//  {
//    dvector x(2);
//    x[0] = (wheelSpeeds[0] * LOOP_RATE) / (float)(TICKS_PER_METER);
//    x[1] = (wheelSpeeds[1] * LOOP_RATE) / (float)(TICKS_PER_METER);
//
//    return x;
//  }

  ivector CommandMotors(const ivector& effort) //actuators, generically
  {
    driver.SetPowers(effort[0], effort[1]);
    
    return effort;
  }

  /*
   * Updates pose based on encoder ticks in this period.
   */
//  Pose UpdateKinematics(int16_t ticksL, int16_t ticksR) //wheel speeds in ticks / period
//  {
//    currPose.u = (ticksL + ticksR) / 2.0; //ticks / period
//    currPose.omega = (ticksR - ticksL)/ (2.0 * ROBOT_RADIUS_IN_TICKS); //rad / period
//
//    currPose.x += currPose.u * currPose.cos_theta; //ticks
//    currPose.y += currPose.u * currPose.sin_theta; //ticks
//
//    currPose.cos_theta -= currPose.omega * currPose.sin_theta; //unitless
//    currPose.sin_theta += currPose.omega * currPose.cos_theta; //unitless
//
//    float normalizer = sqrt(currPose.cos_theta * currPose.cos_theta + currPose.sin_theta * currPose.sin_theta);
//    currPose.cos_theta /= normalizer;
//    currPose.sin_theta /= normalizer;
//
//    return currPose;
//  }

  uint16_t HandleRadio(void);
  CMD_SRC SetSource(uint16_t newSource) {return cmdSource = (CMD_SRC)newSource;}
};

uint16_t UGV::HandleRadio(void)
{
  uint16_t retVal = 0;

  //copy radio data to a local array
  uint8_t length = radio.DATALEN;
  uint8_t data[length];
  for(int i = 0; i < length; i++)
  {
    data[i] = radio.DATA[i];
  }
  
  if(length >= 2) //valid packet -- at least a command
  {
    uint16_t cmd_type; //commands are 16 bits to keep everything as integers
    memcpy(&cmd_type, data, sizeof(cmd_type));

    switch(cmd_type)
    {
      case CMD_JOYSTICK:
        if(length != 8) break;  //not the right sized packet: return false
        ivector joystickVector(3);
        memcpy(&joystickVector[0], &data[2], 6); //x,y.z-button

        if(cmdSource == CMD_SRC_RADIO) //if we're receiving commands from the radio, set the new speed
        {
          SetTargetMotorSpeeds(joystickVector[0], -joystickVector[1]);
        }

        if(prevZbutton == 1 && joystickVector[2] == 0)
        {
          if(cmdSource == CMD_SRC_ROS) SetSource(CMD_SRC_RADIO);
          else if(cmdSource == CMD_SRC_RADIO) SetSource(CMD_SRC_ROS);
        }

        prevZbutton = joystickVector[2];

        retVal = CMD_JOYSTICK;
        
        break;
    }
  }

  return retVal;
}


#endif	/* ROBOT_H */

