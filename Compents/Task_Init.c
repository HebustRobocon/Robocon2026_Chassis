#include "Task_Init.h"
#include "JY61.h"
#include "encoder.h"

#include "comm_stm32_hal_middle.h"
#include "dataFrame.h"
#include "comm.h"

#include "AutoPilot.h"
#include "Pilot_callback.h"
#include "Action_Config.h"
#include "Action.h"

SteeringWheel steeringWheelArray[3];
Wheel_t wheelArray[3];
Chassis_t chassis;

//句柄
TaskHandle_t Wheel_Handles[3];
TaskHandle_t Can_Send_Handle;
TaskHandle_t Remote_Analysis_Handle;

//遥控器数据
uint8_t usart4_dma_buff[30];
uint8_t usart5_dma_buff[60];
Remote_Handle_t Remote_Control;
extern SemaphoreHandle_t Remote_semaphore;

//任务
void Can_Send(void *pvParameters);
void Wheel_Task(void *pvParameters);
void Remote_Analysis_Task(void *pvParameters);

ChassisMode chassis_mode = REMOTE;

void Task_Init(void)
{
		//JY61
    __HAL_UART_ENABLE_IT(&huart4, UART_IT_IDLE);
    HAL_UART_Receive_DMA(&huart4, usart4_dma_buff, sizeof(usart4_dma_buff));
		//遥控器
		__HAL_UART_ENABLE_IT(&huart5, UART_IT_IDLE);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff, sizeof(usart5_dma_buff));
		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
    
    steeringWheelArray[0].Key_GPIO_Port = GPIOE;
    steeringWheelArray[0].Key_GPIO_Pin = GPIO_PIN_11;
    steeringWheelArray[1].Key_GPIO_Port = GPIOE;
    steeringWheelArray[1].Key_GPIO_Pin = GPIO_PIN_9;
    steeringWheelArray[2].Key_GPIO_Port = GPIOA;
    steeringWheelArray[2].Key_GPIO_Pin = GPIO_PIN_5;

    wheelArray[0].pos.x =  0.3725f;
    wheelArray[0].pos.y =  0.3925f; 
    wheelArray[0].pos.z =  PI/2;
    wheelArray[1].pos.x =  0.3725;
    wheelArray[1].pos.y =  -0.3925f;
    wheelArray[1].pos.z =  PI/2;
    wheelArray[2].pos.x =  -0.30733f;
    wheelArray[2].pos.y =  0.0f;
    wheelArray[2].pos.z =  PI/2;

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
    ChassisInit(&chassis, wheelArray, 3, barycenter, 25.2f, 1.25f, 0.00001f, 2, 600, 4);
    
		xTaskCreate(Wheel_Task, "wheel_task1", 300, &wheelArray[0], 4, &Wheel_Handles[0]);
		xTaskCreate(Wheel_Task, "wheel_task2", 300, &wheelArray[1], 4, &Wheel_Handles[1]);
		xTaskCreate(Wheel_Task, "wheel_task3", 300, &wheelArray[2], 4, &Wheel_Handles[2]);
		
		xTaskCreate(Can_Send, "Can_Send", 400, NULL, 4, &Can_Send_Handle);
		
		xTaskCreate(Remote_Analysis_Task, "Remote_Analysis_Task", 128, NULL, 4, &Remote_Analysis_Handle);
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

    swheel->Driver_Vel_PID.Kp = 1.1f;
    swheel->Driver_Vel_PID.Ki = 0.005f;
    swheel->Driver_Vel_PID.Kd = 3.0f;
    swheel->Driver_Vel_PID.limit = 50000.0f;
    swheel->Driver_Vel_PID.output_limit = 45.0f;

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

PackControl_t recv_pack;
Pack_TransRemote_t trans_pack;
uint8_t recv_buff[20] = {0};
float rocker_filter[4] = {0};
static void Key_Parse(uint32_t key, hw_key_t *out)
{
    out->Right_Switch_Up     = (key & KEY_Right_Switch_Up)     ? 1 : 0;
    out->Right_Switch_Down   = (key & KEY_Right_Switch_Down)   ? 1 : 0;

    out->Right_Key_Up        = (key & KEY_Right_Key_Up)        ? 1 : 0;
    out->Right_Key_Down      = (key & KEY_Right_Key_Down)      ? 1 : 0;
    out->Right_Key_Left      = (key & KEY_Right_Key_Left)      ? 1 : 0;
    out->Right_Key_Right     = (key & KEY_Right_Key_Right)     ? 1 : 0;

    out->Right_Broadside_Key = (key & KEY_Right_Broadside_Key) ? 1 : 0;

    out->Left_Switch_Up      = (key & KEY_Left_Switch_Up)      ? 1 : 0;
    out->Left_Switch_Down    = (key & KEY_Left_Switch_Down)    ? 1 : 0;

    out->Left_Key_Up         = (key & KEY_Left_Key_Up)         ? 1 : 0;
    out->Left_Key_Down       = (key & KEY_Left_Key_Down)       ? 1 : 0;
    out->Left_Key_Left       = (key & KEY_Left_Key_Left)       ? 1 : 0;
    out->Left_Key_Right      = (key & KEY_Left_Key_Right)      ? 1 : 0;

    out->Left_Broadside_Key  = (key & KEY_Left_Broadside_Key)  ? 1 : 0;
}

void Remote_Analysis()
{
    if(xSemaphoreTake(Remote_semaphore, pdMS_TO_TICKS(200)) == pdTRUE)
    {
      /* 1. 保存上一帧 */
      Remote_Control.Second = Remote_Control.First;
			
      /* 2. 解析当前按键 */
      Key_Parse(recv_pack.Key, &Remote_Control.First);

      Remote_Control.Ex = - recv_pack.rocker[1] / 1597.0f *MAX_ROBOT_VEL;
      Remote_Control.Ey = recv_pack.rocker[0] / 1597.0f *MAX_ROBOT_VEL;
      Remote_Control.Eomega = recv_pack.rocker[2] / 1597.0f * MAX_ROBOT_OMEGA;
    }else {
      Remote_Control.Ex = 0;
      Remote_Control.Ey = 0;
      Remote_Control.Eomega = 0;

      memset(&Remote_Control.First, 0, sizeof(Remote_Control.First));
    }
}

void MyRecvCallback(uint8_t *src, uint16_t size, void *user_data)
{
    memcpy(&recv_buff, src, size);
    memcpy(&recv_pack, recv_buff, sizeof(recv_pack));
    xSemaphoreGive(Remote_semaphore);
}
CommPackRecv_Cb  recv_cb = MyRecvCallback;

//can发送
int16_t motorCurrentBuf[4] = {0};
float driveCurrentBuf[3] = {0};
float expected = 0.0f;
float Up_L = 0.0f;
float final =0.0f;
Motor3508Ex_t Lift_Motor;
int32_t RAMP_self( int32_t final, int32_t now, int32_t ramp );
void Can_Send(void *pvParameters)
{
		TickType_t last_wake_time = xTaskGetTickCount();
				
		Lift_Motor.ID = 0x204;
	  Lift_Motor.hcan = &hcan2;
	
		Lift_Motor.pos_pid.Kp = 0.1f;
	  Lift_Motor.pos_pid.Ki = 0.0f;
  	Lift_Motor.pos_pid.Kd = 0.0f;
	  Lift_Motor.pos_pid.limit = 10000.0f;
    Lift_Motor.pos_pid.output_limit = 9006.3f;

	  Lift_Motor.vel_pid.Kp = 12.0f;
    Lift_Motor.vel_pid.Ki = 0.01f;
    Lift_Motor.vel_pid.Kd = 0.0f;
    Lift_Motor.vel_pid.limit = 10000.0f;
    Lift_Motor.vel_pid.output_limit = 16384.0f;

		steeringWheelArray[0].DriveMotor.hcan = &hcan1;
		steeringWheelArray[0].DriveMotor.motor_id = 0x01;
		steeringWheelArray[1].DriveMotor.hcan = &hcan1;
		steeringWheelArray[1].DriveMotor.motor_id = 0x02;
    steeringWheelArray[2].DriveMotor.hcan = &hcan1;
		steeringWheelArray[2].DriveMotor.motor_id = 0x03;
		
		g_comm_handle = Comm_Init(&huart5);
    RemoteCommInit(NULL);
    register_comm_recv_cb(recv_cb, 0x01, &recv_pack);
	
		for(;;)
		{
			trans_pack.Key_Data = recv_pack.Key;
			
			if(Remote_Control.First.Left_Key_Left)
				Up_L = 200.0f;
			else if(Remote_Control.First.Left_Key_Down)
				Up_L = 0.0f;
			
			chassis.exp_vel.x = Remote_Control.Ex;
			chassis.exp_vel.y = Remote_Control.Ey;
			chassis.exp_vel.z = Remote_Control.Eomega;
			
			expected = -1*Up_L / (2 * M_PI * 20.63) * 19.0f * 8192.0f;
			final= RAMP_self(expected, final, 520);
			PID_Control2(Lift_Motor.actual_pos,final,&Lift_Motor.pos_pid);
			PID_Control2(Lift_Motor.motor.Speed,Lift_Motor.pos_pid.pid_out,&Lift_Motor.vel_pid); 

			motorCurrentBuf[0] = steeringWheelArray[0].Steering_Vel_PID.pid_out;
			motorCurrentBuf[1] = steeringWheelArray[1].Steering_Vel_PID.pid_out;
			motorCurrentBuf[2] = steeringWheelArray[2].Steering_Vel_PID.pid_out;
			motorCurrentBuf[3] = Lift_Motor.vel_pid.pid_out;
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

void Remote_Analysis_Task(void *pvParameters)
{
	trans_pack.head = 0xAB;
	trans_pack.tail = 0xBA;
	while(1)
	{
		Remote_Analysis();
	}
}

//中断
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
    }else if(ID == 0x204)
		{
			Motor3508Recv(&Lift_Motor,hcan, ID, Recv);
		}
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(&huart5);
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_dma_buff,size);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff,sizeof(usart5_dma_buff));
   	__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	}
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart == &huart2)
	{
			HAL_UART_Transmit_DMA(&huart2,(uint8_t *)&trans_pack,sizeof(Pack_TransRemote_t));
	}
	if(huart == &huart6)
	{
			HAL_UART_Transmit_DMA(&huart6,(uint8_t *)&trans_pack,sizeof(Pack_TransRemote_t));
	}
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == UART5)
	{
		HAL_UART_DMAStop(huart);
		// 重置HAL状态
		huart->ErrorCode = HAL_UART_ERROR_NONE;
		huart->RxState = HAL_UART_STATE_READY;
		huart->gState = HAL_UART_STATE_READY;
		
		// 然后清除错误标志 - 按照STM32F4参考手册要求的顺序
		uint32_t isrflags = READ_REG(huart->Instance->SR);
		
		// 按顺序处理各种错误标志，必须先读SR再读DR来清除错误
		if (isrflags & (USART_SR_ORE | USART_SR_NE | USART_SR_FE)) 
		{
				// 对于ORE、NE、FE错误，需要先读SR再读DR
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
				volatile uint32_t temp_dr = READ_REG(huart->Instance->DR); // 这个读取会清除ORE、NE、FE        

		if (isrflags & USART_SR_PE)
		{
				volatile uint32_t temp_sr = READ_REG(huart->Instance->SR);
		}
	}
		Comm_UART_IRQ_Handle(g_comm_handle, &huart5, usart5_dma_buff, 0);
		HAL_UARTEx_ReceiveToIdle_DMA(&huart5, usart5_dma_buff,sizeof(usart5_dma_buff));
		__HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
	}
}
int32_t RAMP_self( int32_t final, int32_t now, int32_t ramp )
{
    float buffer = final - now;

    if (buffer > 0)
    {
        if (buffer > ramp)  
                now += ramp;  
        else
                now += buffer;
    }		
    else
    {
        if (buffer < -ramp)
                now += -ramp;
        else
                now += buffer;
    }
    return now;
}
