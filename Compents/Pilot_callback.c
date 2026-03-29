#include "Pilot_callback.h"
#include "ForceChassis.h"
#include "JY61.h"
#include "Task_Init.h"

extern JY61_Typedef JY61;
extern Chassis_t chassis;
extern ChassisMode chassis_mode;
// 世界坐标系到车体坐标系的转换
void WorldToBodyFrame(Vector3D world, Vector3D *body, float yaw)
{
    float cos_yaw = cosf(yaw);
    float sin_yaw = sinf(yaw);
    
    // 旋转矩阵：世界坐标系 -> 车体坐标系
    body->x = world.x * cos_yaw + world.y * sin_yaw;
    body->y = -world.x * sin_yaw + world.y * cos_yaw;
    body->z = world.z; // 角速度保持不变
}

void SetRobotPos_Callback(Vector3D pos)
{
    Vector3D relative_pos;
    relative_pos.x = Pos_Vel_Kp * (pos.x - chassis.cur_pos.x);
    relative_pos.y = Pos_Vel_Kp * (pos.y - chassis.cur_pos.y);
    relative_pos.z = pos.z;
    WorldToBodyFrame(relative_pos, &chassis.exp_pos, JY61.Angle.Yaw);
}

void SetRobotVel_Callback(Vector3D vel)
{
    WorldToBodyFrame(vel, &chassis.exp_pilot_vel, JY61.Angle.Yaw);
}

void SetRobotAcc_Callback(Vector3D acc)
{
    WorldToBodyFrame(acc, &chassis.exp_acc, JY61.Angle.Yaw);
}

void Finished_Callback(AutopilotState state,AutoPilotReq_t *req,void* user_data)
{
    if(state == AUTOPILOT_STAGE_IDEL)//自动导航断开，切换成遥控器
    {
        chassis_mode = REMOTE;
    }

    if(state == AUTOPILOT_STAGE_RUNNING)
    {
        
    }

    if(state == AUTOPILOT_STAGE_FINISH)
    {
        //填入下一个导航点

        //切换成遥控状态

        //停止运动

    }

    if(state == AUTOPILOT_STAGE_FINISH)
    {
        //停止运动
        
    }

}

