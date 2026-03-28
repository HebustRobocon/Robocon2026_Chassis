#include "Action_Config.h"

uint8_t one, two, action_callback;
// 动作执行函数
void ActionOne(void *user_data) {
	for(int i = 0; i < 10; i++)
	{
		one++;
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

void ActionTwo(void *user_data) {
	for(int i = 0; i < 10; i++)
	{
		two++;
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

// 动作完成回调
void ActionCompleteCallback(void *user_data) {
    action_callback++;
}
uint32_t id1, id2;
TaskHandle_t TestTask_handle;
void TestTask(void *pvParameters)
{
	ActionManagerInit();// 初始化动作管理器
	id1 = ActionSendInterruptable(ActionOne, ActionCompleteCallback, NULL);
	id2 = ActionSendUninterruptable(ActionTwo, ActionCompleteCallback, NULL);
	while(1)
	{
		
		vTaskDelay(pdMS_TO_TICKS(5000));
	}
}

