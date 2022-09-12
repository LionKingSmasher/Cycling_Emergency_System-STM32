#include <math.h>
#include <stdint.h>

extern float invSampleFreq2;

void madgwickUpdate(float gx, float gy, float gz,
					float ax, float ay, float az,
					float mx, float my, float mz,
					float dt);

void computeAngles();

float getYaw();
float getPitch();
float getRoll();
void getGravity();
void getLinearAccel(int16_t* linearAccel, int16_t* accel);
