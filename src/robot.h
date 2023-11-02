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

#include "estimator.h"

#define DESTINATION_TOLERANCE 0.1  //m
#define DEFAULT_SPEED (0.4) //m/s
#define DEFAULT_ROTATION_DIFFERENTIAL 0.15
#define MAX_SPEED 0.6

#define REFLECTANCE_L A0
#define REFLECTANCE_R A1

#define IDLE 0
#define DEAD_RECKONING 1
#define LINE_FOLLOWING 2
#define SET_PI 3
#define SET_PD_LF 4

//#define BASE_SPEED 20
unsigned K_pLF = 0;
unsigned K_dLF = 0;
int baseSpeed = 0;


  /*
 * Calculation for RADIANS_PER_TICK:
 * 7 magnets = 28 ticks / rotation
 * Gear ratio of 71:1 -> 1988 ticks / rotation 
 * or 2pi / 1988 = 0.00316 rad / tick
 * 
 */
 
#define RADIANS_PER_TICK (0.00316)
#define RADIUS_WHEEL (0.081)
#define TICKS_PER_METER 3906
#define RADIUS_ROBOT (0.2)
//  int16_t radius = 780;
#define LOOP_RATE 50   //Hz

class UGV
{  
protected:
  ivector estimate;
  ivector effort;
  
  Sabertooth driver;
  MotionController controller;

public:
  UGV(void) : estimate(2), effort(2) //sensorL(REFLECTANCE_L), sensorR(REFLECTANCE_R)
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
  }

  virtual void ProcessPID(void)
  {
//    DEBUG_SERIAL.print("readyToPID");
//    DEBUG_SERIAL.print('\n');
      //////////!!!!!!!!!
      estimate = controller.CalcEstimate(); //wheel velocity is ticks/period

      DEBUG_SERIAL.print(estimate[0]);
      DEBUG_SERIAL.print('\t');
      DEBUG_SERIAL.print(estimate[1]);
      DEBUG_SERIAL.print('\t');

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
  
  void SetTargetWheelSpeeds(float left, float right)
  {
    //do calculations in m/s
    float speedLeft = left;
    float speedRight = right;

    //and convert to ticks per frame (use floating point math, but speeds are integers)
    speedLeft *= (float)TICKS_PER_METER / (float)LOOP_RATE;
    speedRight *= (float)TICKS_PER_METER / (float)LOOP_RATE;

    //integer vector -- speeds are in integral numbers of ticks -- ignore the digitization error for now...
    ivector speed(2); 
    speed[0] = speedLeft;
    speed[1] = speedRight;
        
    controller.SetTarget(speed);
  }

  dvector EstimateWheelSpeeds(void)
  {
    dvector x(2);
    x[0] = (estimate[0] * LOOP_RATE) / (float)(TICKS_PER_METER);
    x[1] = (estimate[1] * LOOP_RATE) / (float)(TICKS_PER_METER);

    return x;
  }

  ivector CommandMotors(const ivector& effort) //actuators, generically
  {
    driver.SetPowers(effort[0], effort[1]);
    
    return effort;
  }
};

#endif	/* ROBOT_H */

