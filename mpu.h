#ifndef MPU__
#define MPU__

#include <RTIMULib.h>
#include <iostream>

float getDir(float y, float x);


class MPU{
    public:
	RTIMUSettings *settings;
	RTIMU *imu;
	void init();
	double getPose();
};

namespace Getmpu {
 extern MPU mpu;
}


#endif
