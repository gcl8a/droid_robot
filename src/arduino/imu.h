/*
 * TODO: make reading accelerometer and gyroscope more efficient:
 *  tie I2C in more closely; read status and values at once, dropping if status is false
 *  make gyro and accel independent (really, switch to a 9-DOF chip)
 */

#include <Wire.h>

#include <L3G.h>
#include <LSM303.h>

class IMU
{
  
};

class MinIMU9v3 : public IMU
{
protected:
  LSM303 accelerometer;
  L3G gyroscope;

public:

  void Init(void)
  {
    accelerometer.init(LSM303::device_D, LSM303::sa0_high);
    accelerometer.enableDefault();
    switch (accelerometer.getDeviceType())
    {
      case LSM303::device_D:
        accelerometer.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
        accelerometer.writeReg(LSM303::CTRL1, 0x77);
        break;
      case LSM303::device_DLHC:
        accelerometer.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
        break;
      default: // DLM, DLH
        accelerometer.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
    }

    gyroscope.init();
    gyroscope.writeReg(L3G::CTRL_REG4, 0x20); // 2000 dps full scale
    gyroscope.writeReg(L3G::CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
  }

  uint8_t ReadIMU(void)
  {
//    Wire.beginTransmission(acc_address);
//    Wire.write(OUT_X_L_A | 0x08);
//    Wire.endTransmission();
//    
//    Wire.requestFrom(acc_address, (byte)6);
//
//    unsigned int millis_start = millis();
//    while (Wire.available() < 6) {
//      if (io_timeout > 0 && ((unsigned int)millis() - millis_start) > io_timeout)
//      {
//        did_timeout = true;
//        return;
//      }
//  }
//
//  byte xla = Wire.read();
//  byte xha = Wire.read();
//  byte yla = Wire.read();
//  byte yha = Wire.read();
//  byte zla = Wire.read();
//  byte zha = Wire.read();
//
//  // combine high and low bytes
//  // This no longer drops the lowest 4 bits of the readings from the DLH/DLM/DLHC, which are always 0
//  // (12-bit resolution, left-aligned). The D has 16-bit resolution
//  a.x = (int16_t)(xha << 8 | xla);
//  a.y = (int16_t)(yha << 8 | yla);
//  a.z = (int16_t)(zha << 8 | zla);
//}

    uint8_t accelAvailable = accelerometer.readAccReg(LSM303::STATUS_A) & 0x08;
    if(!accelAvailable) return 0;

    uint8_t gyroAvailable = gyroscope.readReg(L3G::STATUS_REG) & 0x08;
    if(!gyroAvailable) return 0;

    //we have both new accelerometer and gyroscope data, so let's read them
    accelerometer.read();
    gyroscope.read();

    return 1;
  }
};

#define GYRO_FACTOR (0.07 * 0.017453) // 70mdps/LSB => rad/s
#define ACCEL_FACTOR (0.000244 * 9.81) //0.244mg/LSB => m/s^2

#define ACCEL_NOISE 0.001 //(m/s^2)^2
#define GYRO_NOISE 0.0001 //(rad/s)^2

#define ACCEL_BIAS_COV 0.001
#define GYRO_BIAS_COV 0.001

//#define IMU_INTERVAL 50
//
//void Accel_Init(void)//LSM303& accelerometer)
//{
//  accelerometer.init(LSM303::device_D, LSM303::sa0_high);
//  accelerometer.enableDefault();
//  switch (accelerometer.getDeviceType())
//  {
//    case LSM303::device_D:
//      accelerometer.writeReg(LSM303::CTRL2, 0x18); // 8 g full scale: AFS = 011
//      accelerometer.writeReg(LSM303::CTRL1, 0x77);
//      break;
//    case LSM303::device_DLHC:
//      accelerometer.writeReg(LSM303::CTRL_REG4_A, 0x28); // 8 g full scale: FS = 10; high resolution output mode
//      break;
//    default: // DLM, DLH
//      accelerometer.writeReg(LSM303::CTRL_REG4_A, 0x30); // 8 g full scale: FS = 11
//  }
//}

//void Gyro_Init(void)//L3G& gyro)
//{
//  gyro.init();
//  gyro.writeReg(L3G::CTRL_REG4, 0x20); // 2000 dps full scale
//  gyro.writeReg(L3G::CTRL_REG1, 0x0F); // normal power mode, all axes enabled, 100 Hz
//}


//uint8_t ReadIMU(void)//LSM303& accelerometer, L3G& gyro)
//{
//  //SerialUSB.print("Reading accelerometer...");
//  uint8_t dataAvailable = accelerometer.readAccReg(LSM303::STATUS_A) & 0x08;
//
//  if(!dataAvailable) return 0;
//  
//  accelerometer.read();
//
////  SerialUSB.print(accelerometer.a.x * ACCEL_FACTOR);
////  SerialUSB.print(":");
////  SerialUSB.print(accelerometer.a.y * ACCEL_FACTOR);
////  SerialUSB.print(":");
//  
//  while((gyro.readReg(L3G::STATUS_REG) & 0x08) == false)
//  {} 
//    
//  gyro.read();
//
////  SerialUSB.print(gyro.g.z * GYRO_FACTOR);
////  SerialUSB.print('\n');
//
//  return 0;
//}

