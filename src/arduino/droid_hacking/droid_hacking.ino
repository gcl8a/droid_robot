/*
 * Main program for ROS-driven Droid UGV
 * Serves as the interface between ROS nodes on the main board and motor drivers, etc.
 */
 
#define USE_USBCON

#include "ros_ugv.h"

//goes in ros_ugv.cpp if I ever get my act together...
ROSUGV robot;

void MotorCallback(const droid::MotorData& motor_cmd)
{
  robot.ProcessMotorCommand(motor_cmd);
}

void TargetSpeedCallback(const geometry_msgs::Vector3& targetSpeeds)
{
  robot.SetTargetWheelSpeeds(targetSpeeds.x, targetSpeeds.y);
}

//N.B.: No need to start SerialUSB manually in ROS, as the constructors take care of that for us
void setup()
{
  DEBUG_SERIAL.begin(115200);
  delay(100);
  DEBUG_SERIAL.println("setup");

  robot.Init();
 
  DEBUG_SERIAL.println("/setup");
}

void loop(void)
{
  robot.MainLoop();

  if(CheckRadio())
  {
//    ivector speeds = HandleRadio();
//    if(speeds.Length() == 3)
//    {
//      robot.SetTargetWheelSpeeds(speeds[0], speeds[1]);
//      
//    DEBUG_SERIAL.print("Setting left = ");
//    DEBUG_SERIAL.println(speeds[0]);
//    DEBUG_SERIAL.print("Setting right = ");
//    DEBUG_SERIAL.println(speeds[1]);
//
//    }
    DEBUG_SERIAL.println("radio");

  }
  
  if(CheckDebugSerial())
  {
    //all in m/s, rad/s
    float left = debugString.toFloat();
    uint8_t comma = debugString.indexOf(',');
    float right = debugString.substring(comma+1).toFloat();

    DEBUG_SERIAL.print("Setting left = ");
    DEBUG_SERIAL.println(left);
    DEBUG_SERIAL.print("Setting right = ");
    DEBUG_SERIAL.println(right);

    robot.SetTargetWheelSpeeds(left, right);

    debugString = "";
  }
}

