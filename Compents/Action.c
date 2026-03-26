#include "Action_Config.h"

uint8_t one, two, three, four, action_callback;
// 动作执行函数
void ActionOne(void *user_data) {
    while(1)
    {
        one++;
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void ActionTwo(void *user_data) {
    while (1)
    {
        two++;
        vTaskDelay(pdMS_TO_TICKS(1500));
    }
}

void ActionThree(void *user_data) {
    while(1)
    {
        three++;
        vTaskDelay(pdMS_TO_TICKS(800));
    }
    
}

void ActionFour(void *user_data) {
    while(1)
    {
        four++;
        vTaskDelay(pdMS_TO_TICKS(1200)); 
    }
}

// 动作完成回调
void ActionCompleteCallback(void *user_data) {
    action_callback++;
}
uint32_t id1, id2;
void TestTask(void *pvParameters)
{
    // 初始化动作管理器
    ActionManagerInit();
    while (1)
    {
        id1 = ActionSendInterruptable(ActionOne, ActionCompleteCallback, NULL);
        id2 = ActionSendUninterruptable(ActionThree, ActionCompleteCallback, NULL);
    }
}

