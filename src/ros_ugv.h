#ifndef __ROS_UGV_H
#define __ROS_UGV_H

#include "robot.h"

#include <ros.h>

//#include <geometry_msgs/Twist.h>
#include <droid/MotorData.h>

//motor callback
void MotorCallback(const droid::MotorData& motorCmd);
ros::Subscriber<droid::MotorData> subMotorCmd("motor_targets", MotorCallback);

#define CMD_MOT_IDLE  0
#define CMD_MOT_VEL   1

class ROSUGV : public UGV
{
protected:
  //node handler
  ros::NodeHandle nh;

  droid::MotorData odomData;
  ros::Publisher pubOdom;//("odom", &odomData);

public:
  ROSUGV(void) : pubOdom("encData", &odomData) //not sure why this works...odomData is a member datum
  {}
  
  void Init(void)
  {
    UGV::Init();

    nh.initNode();
    nh.advertise(pubOdom);
    nh.subscribe(subMotorCmd);
  }

  void MainLoop(void)
  {
    nh.spinOnce();
    UGV::MainLoop();
  }

  void ProcessMotorCommand(const droid::MotorData& motor_cmd)
  {
    if(motor_cmd.mode == CMD_MOT_IDLE)
      Idle();
    if(motor_cmd.mode == CMD_MOT_VEL)
      SetTargetWheelSpeeds(motor_cmd.left, motor_cmd.right);
  }

  void ProcessPID(void)
  {
    UGV::ProcessPID();

    //get the odometry data (ignoring covariance for now...bit of a waste)
    dvector x = EstimateWheelSpeeds();

    odomData.mode  = 1;
    odomData.left  = x[0];
    odomData.right = x[1];

    pubOdom.publish(&odomData);
  }
};

#endif
