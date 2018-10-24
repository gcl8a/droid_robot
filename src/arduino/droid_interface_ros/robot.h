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
//#include "reflectance.h"
#include "imu.h"

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
  
//  nav_msgs::Odometry estimate; 
//  nav_msgs::Odometry target; 

  //uint8_t opMode;

  Sabertooth driver;
  MotionController controller;

//  ReflectanceSensorAnalog sensorL;
//  ReflectanceSensorAnalog sensorR;
  
//  Timer reflectanceTimer;

//  nav_msgs::Odometry odometryData; //we'll just use the velocity for now?

  //MinIMU9v3 imu;

public:
  UGV(void) : estimate(2), effort(2) //sensorL(REFLECTANCE_L), sensorR(REFLECTANCE_R)
  {}
  
  virtual void Init(void)
  {
    DEBUG_SERIAL.println("UGV::Init");

    //opMode = IDLE;
    driver.Init(COMM_PACKET_SERIAL);
    controller.Init();

    //imu.Init();

    DEBUG_SERIAL.println("/UGV::Init");
  }

  void Idle(void)
  {
    driver.FullStop();
  }


//  void CalibrateSensors(uint32_t period)
//  {
//    sensorL.Calibrate(period);
//    sensorR.Calibrate(period);
//  }

//  droid_bot::DroidDatum HandleReflectanceTimer(void)
//  {
//    droid_bot::DroidDatum datum; 
//
//    uint16_t refL = sensorL.ReadScaled();    
//    uint16_t refR = sensorR.ReadScaled();
//
//    DEBUG_SERIAL.print(refL);
//    DEBUG_SERIAL.print('\t');
//    DEBUG_SERIAL.print(refR);
//    DEBUG_SERIAL.print('\t');
//
//    datum.val0 = refL;
//    datum.val1 = refR;
//
//    return datum;
//  }

//  void ProcessMotorCommand(const droid_bot::DroidDatum& md)
//  {
//    if(md.cmd == FULL_STOP)
//    {
//      driver.FullStop();
//    }
//    
//    if(md.cmd == VEL_CTRL) 
//    {
//      driver.SetSpeeds(md.val0, md.val1); //    driver.EnableSpeedControl();
//    }
//  
//    if(md.cmd == PWR_CTRL) 
//    {
//      driver.SetPowers(md.val0, md.val1);
//    }
//  }
  
//  void ProcessCommand(const droid_bot::DroidDatum& command)
//  {
//    if(command.cmd == IDLE)
//    {
////      encoderTimer.Cancel();
//      reflectanceTimer.Cancel();
//
//      driver.FullStop();
//    }
//
//    if(command.cmd == DEAD_RECKONING)
//    {
//      opMode = DEAD_RECKONING;
//      
//      reflectanceTimer.Cancel();
////      encoderTimer.Start(ENC_INTERVAL);
//
//      driver.FullStop();
//    }
//    
//    if(command.cmd == LINE_FOLLOWING)
//    {
//      opMode = LINE_FOLLOWING;
//
//      driver.FullStop();
////      encoderTimer.Cancel();
//      
//      CalibrateSensors(command.val0);
//      baseSpeed = command.val1;
//      
//      reflectanceTimer.Start(REF_INTERVAL);
//      
//      driver.SetSpeeds(0, 0);
//    }
//    
//    if(command.cmd == SET_PD_LF)  //line following gains
//    {
//      K_pLF = command.val0;
//      K_dLF = command.val1;
//    }
//    
//    if(command.cmd == SET_PI)
//    {
//      driver.SetPIDCoeffs(command.val0, command.val1);
//    }
//  }

  virtual void MainLoop(void)
  {  
//    if(reflectanceTimer.CheckExpired())
//    {
//      reflectanceTimer.Restart();
//      reflectance_data = HandleReflectanceTimer();
//
////      ref_data.mode = opMode;
////      ref_data.a = refDatum.values[0];
////      ref_data.b = refDatum.values[1];
//
//      static int16_t prevError = 0;
//      int16_t error = reflectance_data.val1 - reflectance_data.val0;
//      int16_t dError = error - prevError;
//      
//      int16_t speedA = baseSpeed + (K_pLF * error + K_dLF * dError) / 64;
//      int16_t speedB = baseSpeed - (K_pLF * error + K_dLF * dError) / 64;
//
//      prevError = error;
//      
////      driver.SetSpeeds(speedA, speedB);
//
//      readyToPID = 1;
//      
//      pubReflectance.publish(&reflectance_data);
//    }

    if(readyToPID) 
    {
      ProcessPID();
      readyToPID = 0;
    }

//    if(imu.ReadIMU()) //returns true if new data is available
//    {
//      //process the IMU here
//    }
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
  
  void SetTwistSpeed(float vel, float ang_vel)
  {
    //do calculations in m/s
    float speedLeft = vel - ang_vel * RADIUS_ROBOT;
    float speedRight = vel + ang_vel * RADIUS_ROBOT;

    //and convert to ticks per frame
    speedLeft *= (float)TICKS_PER_METER / (float)LOOP_RATE;
    speedRight *= (float)TICKS_PER_METER / (float)LOOP_RATE;

    //integer vector
    ivector speed(2); 
    speed[0] = speedLeft;
    speed[1] = speedRight;
        
    controller.SetTarget(speed);
  }

  void SetWheelSpeeds(float left, float right)
  {
    //do calculations in m/s
    float speedLeft = left;
    float speedRight = right;

    //and convert to ticks per frame
    speedLeft *= (float)TICKS_PER_METER / (float)LOOP_RATE;
    speedRight *= (float)TICKS_PER_METER / (float)LOOP_RATE;

    //integer vector -- speeds are in integral numbers of ticks -- ignore the digitization error for now...
    ivector speed(2); 
    speed[0] = speedLeft;
    speed[1] = speedRight;
        
    controller.SetTarget(speed);
  }

  dvector EstimateVehicleSpeed(void)
  {
    dvector x(2);
    x[0] = ((estimate[0] + estimate[1]) * LOOP_RATE) / (float)(2 * TICKS_PER_METER);
    x[1] = ((-estimate[0] + estimate[1]) * LOOP_RATE) / (RADIUS_ROBOT * (float)(2 * TICKS_PER_METER));

    return x;
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

