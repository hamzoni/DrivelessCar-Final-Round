#include "mpu.h"


float getDir(float y, float x){
    float PI = 3.14;
  float angle = (float)std::atan2(y, x);
//  float deg = -0.019; //-(1+7/60)*PI/180  -1*7'
    float dec = (1+ 43 /60)*PI/180;
  angle += dec;
  if(angle < 0) angle += 2* PI;
  if(angle > 2 * PI) angle -= 2* PI;

  float deg = angle * 180 / PI;
  return deg;
// if((deg < 22.5)  || (deg > 337.5 ))  return "North";
// if((deg > 22.5)  && (deg < 67.5 ))   return "North-East";
// if((deg > 67.5)  && (deg < 112.5 ))  return "East";
// if((deg > 112.5) && (deg < 157.5 ))  return "South-East";
// if((deg > 157.5) && (deg < 202.5 ))  return "South";
// if((deg > 202.5) && (deg < 247.5 ))  return "SOuth-West";
// if((deg > 247.5) && (deg < 292.5 ))  return "West";
// if((deg > 292.5) && (deg < 337.5 ))  return "North-West";

}


void MPU::init(){

    this->settings = new RTIMUSettings("RTIMULib");
    this->imu = RTIMU::createIMU(&settings);
    if ((imu == NULL) || (imu->IMUType() == RTIMU_TYPE_NULL)) {
        printf("No IMU found\n");
        //exit(1);
    }

    imu->IMUInit();
    imu->setSlerpPower(0.02);
    imu->setGyroEnable(true);
    imu->setAccelEnable(false);
    imu->setCompassEnable(false);
}

double MPU::getPose(){

        if (imu->IMURead()) {
            RTIMU_DATA imuData = imu->getIMUData();
                return imuData.fusionPose.z()*180/3.14;

        }
        return 999;
    }

namespace Getmpu {
  MPU mpu;
}
