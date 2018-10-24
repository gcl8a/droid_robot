/*
 * Main program for ROS-driven Droid UGV
 * Serves as the interface between ROS nodes on the main board and motor drivers, etc.
 */
 
#define USE_USBCON

#include "ros_ugv.h"

//goes in ros_ugv.cpp if I ever get my act together...
ROSUGV robot;

//void TwistCallback(const geometry_msgs::Twist& twist)
//{
//  robot.ProcessTwist(twist);
//}

void MotorCallback(const droid::MotorData& motor_cmd)
{
  robot.ProcessMotorCommand(motor_cmd);
}

//N.B.: No need to start SerialUSB manually in ROS, as the constructors take care of that for us
void setup()
{
  DEBUG_SERIAL.begin(115200);
  delay(500);
  DEBUG_SERIAL.println("setup");

  robot.Init();
 
  DEBUG_SERIAL.println("/setup");
}

void loop(void)
{
  robot.MainLoop();

  if(CheckDebugSerial())
  {
    //all in m/s, rad/s
    float vel = debugString.toFloat();
    uint8_t comma = debugString.indexOf(',');
    float ang = debugString.substring(comma+1).toFloat();

    DEBUG_SERIAL.print("Setting u = ");
    DEBUG_SERIAL.println(vel);
    DEBUG_SERIAL.print("Setting omega = ");
    DEBUG_SERIAL.println(ang);

    robot.SetTwistSpeed(vel, ang);

    debugString = "";
  }
}


/////obsolete stuff nedenfor

//callback for commands...
//void CommandCallback(const droid_bot::DroidDatum& cmd)
//{
//  robot.ProcessCommand(cmd);
//}
//
////...and the subscriber for commands
//ros::Subscriber<droid_bot::DroidDatum> subRobotCmd("robotCmd", CommandCallback);


