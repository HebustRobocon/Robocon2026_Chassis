#ifndef _PILOT_CALLBACK__H_
#define _PILOT_CALLBACK__H_

#include "Vector.h"
#include "AutoPilot.h"

#define Pos_Vel_Kp 0.5

void SetRobotPos_Callback(Vector3D pos);
void SetRobotVel_Callback(Vector3D vel);
void SetRobotAcc_Callback(Vector3D acc);
void Finished_Callback(AutopilotState state,AutoPilotReq_t *req,void* user_data);

#endif
