#include "Task_Init.h"
#include "JY61.h"
#include "encoder.h"
#include "AS5047.h"
#include "spi.h"

SteeringWheel steeringWheelArray[3];
Wheel_t wheelArray[3];
Chassis_t chassis;

TaskHandle_t Wheel_Handles[3];
TaskHandle_t Move_Task_Handle;
TaskHandle_t Can_Send_Handle;

uint8_t usart4_dma_buff[30];
uint8_t usart5_dma_buff[104];
UART_DataPack RemoteData;  //将串口接收的数据存到这里
Remote_Handle_t Remote_Control; //取出遥控器数据

extern SemaphoreHandle_t remote_semaphore;
extern SemaphoreHandle_t Jy61_semaphore;

void Task_Init(void)
{
		//遥控器
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart4, usart4_dma_buff, sizeof(usart4_dma_buff));
	
    steeringWheelArray[0].Key_GPIO_Port = GPIOA;
    steeringWheelArray[0].Key_GPIO_Pin = GPIO_PIN_7;
    steeringWheelArray[1].Key_GPIO_Port = GPIOA;
    steeringWheelArray[1].Key_GPIO_Pin = GPIO_PIN_6;
    steeringWheelArray[2].Key_GPIO_Port = GPIOC;
    steeringWheelArray[2].Key_GPIO_Pin = GPIO_PIN_4;

    wheelArray[0].pos.x =  0.52996f;
    wheelArray[0].pos.y =  0.0f; 
    wheelArray[0].pos.z =  PI/2;
    wheelArray[1].pos.x =  0.0;
    wheelArray[1].pos.y = -0.52932f;
    wheelArray[1].pos.z = -PI/2;
    wheelArray[2].pos.x =  0.0f;
    wheelArray[2].pos.y =  0.52996f;
    wheelArray[2].pos.z =  0;

    for(int i = 0; i < 3; i++)
    {
        wheelArray[i].user_data = &steeringWheelArray[i];
        wheelArray[i].set_target_cb = SetWheelTarget_Callback;
        wheelArray[i].reset_cb = WheelReset_Callback;
        wheelArray[i].state_cb = WheelState_Callback;
        wheelArray[i].get_vel_cb = GetWheelVelocity_Callback;
        chassis.wheel[i] = &wheelArray[i];
    }
		
    Vector2D barycenter = {0, 0};
    chassis.wheel_err_cb = WheelError_Callback;
    ChassisInit(&chassis, wheelArray, 3, barycenter, 10.0f, 1.25f, 0.00001f, 2, 400, 4);
    
		xTaskCreate(Wheel_Task, "wheel_task1", 256, &wheelArray[0], 4, &Wheel_Handles[0]);
		xTaskCreate(Wheel_Task, "wheel_task2", 256, &wheelArray[1], 4, &Wheel_Handles[1]);
		xTaskCreate(Wheel_Task, "wheel_task3", 256, &wheelArray[2], 4, &Wheel_Handles[2]);
		
    xTaskCreate(Move_Task, "Move_Task", 200, NULL, 5, &Move_Task_Handle);//遥控器任务
		xTaskCreate(Can_Send, "Can_Send", 200, NULL, 4, &Can_Send_Handle);
		
}

void Wheel_Task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    Wheel_t *wheel=(Wheel_t *)pvParameters;
    SteeringWheel *swheel = (SteeringWheel *)wheel->user_data;

    swheel->Steering_Vel_PID.Kp = 10.0f;
    swheel->Steering_Vel_PID.Ki = 0.0f;
    swheel->Steering_Vel_PID.Kd = 0.0f;
    swheel->Steering_Vel_PID.limit = 10000.0f;
    swheel->Steering_Vel_PID.output_limit = 10000.0f;

    swheel->Steering_Dir_PID.Kp = 200.0f;
    swheel->Steering_Dir_PID.Ki = 0.0f;
    swheel->Steering_Dir_PID.Kd = 3.3f;
    swheel->Steering_Dir_PID.limit = 10.0f;
    swheel->Steering_Dir_PID.output_limit = 10000.0f;

    swheel->Driver_Vel_PID.Kp = 0.6f;
    swheel->Driver_Vel_PID.Ki = 0.003f;
    swheel->Driver_Vel_PID.Kd = 2.0f;
    swheel->Driver_Vel_PID.limit = 10000.0f;
    swheel->Driver_Vel_PID.output_limit = 50.0f;

    swheel->offset = 0.0f;
    swheel->maxRotateAngle = 350.0f;
    swheel->floatRotateAngle = 340.0f;
    swheel->ready_edge_flag = 0;
	  swheel->expextForce = 0.0f;
		
    for(;;)
    {
				UpdateAngle(swheel);
				PID_Control2(swheel->currentDirection, swheel->putoutDirection, &swheel->Steering_Dir_PID);//角度环
				PID_Control2(swheel->SteeringMotor.Speed, swheel->Steering_Dir_PID.pid_out, &swheel->Steering_Vel_PID);//速度环

				PID_Control_d(swheel->DriveMotor.epm / 20.0f, swheel->putoutVelocity / wheel_radius / (2.0f * PI) * 60.0f, &swheel->Driver_Vel_PID);

				vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
    }
}

int16_t motorCurrentBuf[3] = {0};
float driveCurrentBuf[3] = {0};

void Can_Send(void *pvParameters)
{
		TickType_t last_wake_time = xTaskGetTickCount();
		
		steeringWheelArray[0].DriveMotor.hcan = &hcan1;
		steeringWheelArray[0].DriveMotor.motor_id = 0x01;
		steeringWheelArray[1].DriveMotor.hcan = &hcan1;
		steeringWheelArray[1].DriveMotor.motor_id = 0x02;
    steeringWheelArray[2].DriveMotor.hcan = &hcan1;
		steeringWheelArray[2].DriveMotor.motor_id = 0x03;
		for(;;)
		{
			motorCurrentBuf[0] = steeringWheelArray[0].Steering_Vel_PID.pid_out;
			motorCurrentBuf[1] = steeringWheelArray[1].Steering_Vel_PID.pid_out;
			motorCurrentBuf[2] = steeringWheelArray[2].Steering_Vel_PID.pid_out;
			
			MotorSend(&hcan2, 0x200, motorCurrentBuf);
			
			driveCurrentBuf[0] = steeringWheelArray[0].Driver_Vel_PID.pid_out;
			driveCurrentBuf[1] = steeringWheelArray[1].Driver_Vel_PID.pid_out;
			driveCurrentBuf[2] = steeringWheelArray[2].Driver_Vel_PID.pid_out;
			
      VESC_SetCurrent(&steeringWheelArray[0].DriveMotor, driveCurrentBuf[0]);
      VESC_SetCurrent(&steeringWheelArray[1].DriveMotor, driveCurrentBuf[1]);
      VESC_SetCurrent(&steeringWheelArray[2].DriveMotor, driveCurrentBuf[2]);
			
			vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
		}
}

ChassisMode chassis_mode = REMOTE;
Vector3D cur_pos;
Vector3D exp_pos;
Vector3D exp_vel;
PID2 Pos_PID_x;
PID2 Pos_PID_y;
PID2 Pos_PID_z;
extern JY61_Typedef JY61;
uint8_t Gyro_Rx_Buffer[22];

void Move_Task(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();

    for(;;)
    {
        if(xSemaphoreTake(remote_semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
        {
            memcpy(&RemoteData, usart4_dma_buff, sizeof(RemoteData));
            UpdateKey(&Remote_Control);
            Remote_Control.Ex =-RemoteData.rocker[1];
            Remote_Control.Ey = RemoteData.rocker[0];
            Remote_Control.Eomega = RemoteData.rocker[2];
            Remote_Control.mode = RemoteData.rocker[3];
            Remote_Control.Key_Control = &RemoteData.Key;
        }else{
            Remote_Control.Ex = 0;
            Remote_Control.Ey = 0;
            Remote_Control.Eomega = 0;
            Remote_Control.mode = 0;
            //按键状态清零
            memset(&RemoteData.Key, 0, sizeof(hw_key_t));
            Remote_Control.Key_Control = &RemoteData.Key;
        }

				if(xSemaphoreTake(Jy61_semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
				{
						JY61_Receive(&JY61, usart5_dma_buff, sizeof(JY61));
				}
				
        if(chassis_mode == AUTO)
        {
            PID_Control2(cur_pos.x, exp_pos.x, &Pos_PID_x);
            PID_Control2(cur_pos.y, exp_pos.y, &Pos_PID_y);
            PID_Control2(cur_pos.z, exp_pos.z, &Pos_PID_z);

            Vector3D exp_vel_world;
            exp_vel_world.x = Pos_PID_x.pid_out + exp_vel.x;
            exp_vel_world.y = Pos_PID_y.pid_out + exp_vel.y;
            exp_vel_world.z = Pos_PID_z.pid_out + exp_vel.z;

            float c = arm_cos_f32(cur_pos.z);
            float s = arm_sin_f32(cur_pos.z);
            chassis.exp_vel.x = exp_vel_world.x * c + exp_vel_world.y * s;
            chassis.exp_vel.y = exp_vel_world.y * c - exp_vel_world.x * s;
            chassis.exp_vel.z = exp_vel_world.z;
        }else if(chassis_mode == REMOTE)
        {
            chassis.exp_vel.x = Remote_Control.Ex / 2047.0f * MAX_ROBOT_VEL; //遥控器控制
            chassis.exp_vel.y = Remote_Control.Ey / 2047.0f * MAX_ROBOT_VEL;
            chassis.exp_vel.z = Remote_Control.Eomega / 2047.0f * MAX_ROBOT_OMEGA;//弧度/s
        }
    }
}

void UpdateKey(Remote_Handle_t * xx) { //遥控器数据更新
    xx->Second = xx->First;
    xx->First = *xx->Key_Control;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint8_t Recv[8] = {0};
	uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
	VESC_ReceiveHandler(&steeringWheelArray[0].DriveMotor, hcan, ID,Recv);
	VESC_ReceiveHandler(&steeringWheelArray[1].DriveMotor, hcan, ID,Recv);
	VESC_ReceiveHandler(&steeringWheelArray[2].DriveMotor, hcan, ID,Recv);
}

void HAL_CAN_RxFifo1MsgPendingCallback(CAN_HandleTypeDef *hcan) // 接收2006的反馈
{
	uint8_t Recv[8] = {0};
	uint32_t ID = CAN_Receive_DataFrame(hcan, Recv);
	
  if (hcan->Instance == CAN2)
  {
    if (ID == 0x201) // 左上，象限2
    {
      M2006_Receive(&steeringWheelArray[0].SteeringMotor, Recv);
    }
    else if (ID == 0x202) // 右上(象限1)
    {
      M2006_Receive(&steeringWheelArray[1].SteeringMotor, Recv);
    }
    else if (ID == 0x203) // 左下(象限3)
    {
      M2006_Receive(&steeringWheelArray[2].SteeringMotor, Recv);
    }
  }
}
