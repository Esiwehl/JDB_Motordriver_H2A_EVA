/*
 * md_9dof.h
 *
 * Created: 6/26/2016 11:32:51 PM
 *  Author: bakker
 */ 

#ifndef MD_9DOF_H_
#define MD_9DOF_H_

void Init9DOF(void);
void Process9DOF(void);
void PrintCSV_9DOF(FILE *fd);

float GyroGetTemp(void);
float GyroGetMagnetic(uint8_t X_Y_Z);
float GyroGetAcceleration(uint8_t X_Y_Z);
float GyroGetGyro(uint8_t X_Y_Z);

#endif
