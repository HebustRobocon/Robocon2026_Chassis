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

TaskHandle_t Wheel_Handles[3];
TaskHandle_t Can_Send_Handle;

uint8_t usart4_dma_buff[30];
uint8_t usart5_dma_buff[30];
Remote_Handle_t Remote_Control; //取出遥控器数据
extern SemaphoreHandle_t Remote_semaphore;

void Can_Send(void *pvParameters);
void Wheel_Task(void *pvParameters);

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
    steeringWheelArray[0].Key_GPIO_Pin = GPIO_PIN_9;
    steeringWheelArray[1].Key_GPIO_Port = GPIOA;
    steeringWheelArray[1].Key_GPIO_Pin = GPIO_PIN_7;
    steeringWheelArray[2].Key_GPIO_Port = GPIOE;
    steeringWheelArray[2].Key_GPIO_Pin = GPIO_PIN_11;

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
		
		xTaskCreate(Can_Send, "Can_Send", 300, NULL, 4, &Can_Send_Handle);
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

//路径规划任务
AutoPilot_t autopilot;
AutoPilotReq_t req;
MoveDest_t dest;
MathSolver_t solver = AUTOPILOT_DEFAULT_SOLVER_PARAM();
TaskHandle_t AutoAPP_handle;
AutoPilotCallback_t callbacks = {
        .setPos_cb = SetRobotPos_Callback,
        .setVel_cb = SetRobotVel_Callback,
        .setAcc_cb = SetRobotAcc_Callback
    };

void AutoPilot_APP(void *pvParameters)
{
    TickType_t last_wake_time = xTaskGetTickCount();
    dest.finish_cb = Finished_Callback;
    AutoPilotInit(&autopilot, &callbacks, 5, 5, 10, AutoAPP_handle);
    for(;;)
    {
        
        vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
    }
}
//can发送和遥控器任务
int16_t motorCurrentBuf[3] = {0};
float driveCurrentBuf[3] = {0};
ChassisMode chassis_mode = REMOTE;
extern uint32_t id1;

PackControl_t recv_pack;
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

      Remote_Control.Ex = recv_pack.rocker[0] / 1847.0f *MAX_ROBOT_VEL;
      Remote_Control.Ey = recv_pack.rocker[1] / 1847.0f *MAX_ROBOT_VEL;
      Remote_Control.Eomega = recv_pack.rocker[2] / 1847.0f * MAX_ROBOT_OMEGA;
    }else {
      Remote_Control.Ex = 0;
      Remote_Control.Ey = 0;
      Remote_Control.Eomega = 0;

      memset(&Remote_Control.First, 0, sizeof(Remote_Control.First));
    }
}

void Rocker_Filter(PackControl_t *data)
{
    float alpha = 0.6f;

    for(int i = 0; i < 4; i++)
    {
        rocker_filter[i] = alpha * data->rocker[i] +
                          (1.0f - alpha) * rocker_filter[i];

        data->rocker[i] = rocker_filter[i];
    }
}
void MyRecvCallback(uint8_t *src, uint16_t size, void *user_data)
{
    memcpy(&recv_buff, src, size);
    memcpy(&recv_pack, recv_buff, sizeof(recv_pack));
    Rocker_Filter(&recv_pack);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(Remote_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
CommPackRecv_Cb  recv_cb = MyRecvCallback;

void Can_Send(void *pvParameters)
{
		TickType_t last_wake_time = xTaskGetTickCount();
		
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
			
			Remote_Analysis();
			/* 单次触发 */
			if (Remote_Control.First.Left_Key_Up == 1 && Remote_Control.Second.Left_Key_Up  == 0)
			{
				chassis_mode = REMOTE;
			}
			if (Remote_Control.First.Left_Key_Down == 1 && Remote_Control.Second.Left_Key_Down  == 0)
			{
				chassis_mode = AUTO;
			}
      
      if(chassis_mode == REMOTE)
      {
        chassis.exp_vel.x = Remote_Control.Ex;
        chassis.exp_vel.y = Remote_Control.Ey;
        chassis.exp_vel.z = Remote_Control.Eomega;
      }else if(chassis_mode == AUTO)
      {
        
      }

			vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(2));
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
