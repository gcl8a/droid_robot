#ifndef __ROS_UGV_H
#define __ROS_UGV_H

#include "robot.h"

#include <ros.h>

#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>

void TwistCallback(const geometry_msgs::Twist& cmd_vel);

#define CMD_MOT_IDLE  0
#define CMD_MOT_VEL   1

class ROSUGV : public UGV
{
protected:
  //node handler
  ros::NodeHandle nh;

  nav_msgs::Odometry odom; 
  ros::Publisher odom_pub;

  tf::TransformBroadcaster odom_broadcaster;
  
  ros::Subscriber<geometry_msgs::Twist> subCmdVel;

public:
  ROSUGV(void) : odom_pub("odom", &odom), subCmdVel("cmd_vel", TwistCallback)
  {}
  
  void Init(void)
  {
    UGV::Init();

    nh.initNode();
    nh.advertise(odom_pub);

    nh.subscribe(subCmdVel);
    odom_broadcaster.init(nh);
  }

  void MainLoop(void)
  {
    nh.spinOnce();
    UGV::MainLoop();
  }

  void ProcessPID(void)
  {
    UGV::ProcessPID();

    //first, we'll publish the transform over tf                
    geometry_msgs::Quaternion odom_quat;
      
    odom_quat.x = 0.0;
    odom_quat.y = 0.0;

    //crap...this is a waste -- can I save cos_theta/2 ?
    float theta = atan2(currPose.cos_theta, currPose.sin_theta);
    odom_quat.z = sin(theta / 2.0);
    odom_quat.w = cos(theta / 2.0);
      
    geometry_msgs::TransformStamped odom_trans;
    odom_trans.header.stamp = nh.now();//now; I don't like this...
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_link";
      
    odom_trans.transform.translation.x = currPose.x / TICKS_PER_METER;
    odom_trans.transform.translation.y = currPose.y / TICKS_PER_METER;
    odom_trans.transform.translation.z = 0.0;
    odom_trans.transform.rotation = odom_quat;
      
    //send the transform
    odom_broadcaster.sendTransform(odom_trans);

    nav_msgs::Odometry odom;
    odom.header.stamp = nh.now();
    odom.header.frame_id = "odom";
      
    //set the position
    odom.pose.pose.position.x = currPose.x / TICKS_PER_METER;
    odom.pose.pose.position.y = currPose.y / TICKS_PER_METER;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
      
    //set the velocity
    odom.child_frame_id = "base_link";
    odom.twist.twist.linear.x = currPose.u / TICKS_PER_METER;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.angular.z = currPose.omega;
      
    //publish the message
    odom_pub.publish(&odom);
  }
};


#endif
