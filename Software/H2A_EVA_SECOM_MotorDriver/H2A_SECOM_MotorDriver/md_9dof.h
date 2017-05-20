/*
 * md_9dof.h
 *
 * Created: 6/26/2016 11:32:51 PM
 *  Author: bakker
 */ 

#ifndef MD_9DOF_H_
#define MD_9DOF_H_

void InitIMU(void);
void ProcessIMU(void);
void PrintCSV_IMU(FILE *fd);

float IMUGetTemperature(void);
float IMUGetMagnetic(uint8_t X_Y_Z);
float IMUGetAcceleration(uint8_t X_Y_Z);
float IMUGetGyro(uint8_t X_Y_Z);

#endif
