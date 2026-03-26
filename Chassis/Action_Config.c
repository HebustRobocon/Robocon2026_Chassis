#include "Action_Config.h"

// 全局变量
ActionManager_t action_manager;
TaskHandle_t ActionDealTask_handle;

void ActionDealTask(void *pvParameters);
void ActionExecuteTask(void *pvParameters);
void ActionManagerInit(void)
{
    action_manager.action_queue = xQueueCreate(10, sizeof(Action_t*));//初始化动作队列长度为10，最大支持5个任务并行

     // 初始化动作数组
    for (int i = 0; i < 10; i++) {
        action_manager.actions[i] = NULL;
    }

    action_manager.action_count = 0;
    action_manager.next_action_id = 1;

    action_manager.mutex = xSemaphoreCreateMutex();//初始化互斥锁
    xTaskCreate(ActionDealTask, "ActionDealTask", 256, NULL, 3, &ActionDealTask_handle);//创建动作处理任务
}

void ActionDealTask(void *pvParameters)
{
    while (1)
    {
        Action_t *action = NULL;//等待新动作
        if(xQueueReceive(action_manager.action_queue, &action, portMAX_DELAY) == pdPASS)
        {
            xSemaphoreTake(action_manager.mutex, portMAX_DELAY);
            if(action_manager.action_count < 10)
            {
                int idx = -1;//找到空闲位置
                for(int i = 0; i < 10; i++)
                {
                    if(action_manager.actions[i] == NULL)
                    {
                        idx = i;
                        break;
                    }
                }
                if(idx != -1)
                {
                    action->id = action_manager.next_action_id++;//设置动作ID和状态
                    action->state = ACTION_STATE_EXECUTING;
                    action_manager.actions[idx] = action;//添加到动作数组
                    action_manager.action_count++;

                    xTaskCreate(ActionExecuteTask, "ActionTask", 128, action, 2, &action->task_handle);// 创建动作执行任务
                }
            }
            xSemaphoreGive(action_manager.mutex);
        }
    }
}

void ActionExecuteTask(void *pvParameters)
{
    Action_t *action = (Action_t *)pvParameters;
    // 检查动作是否已被中断
    if (action->state == ACTION_STATE_INTERRUPTED) {
        // 从动作数组中移除
        xSemaphoreTake(action_manager.mutex, portMAX_DELAY);
        for (int i = 0; i < 10; i++) {
            if (action_manager.actions[i] == action) {
                action_manager.actions[i] = NULL;
                action_manager.action_count--;
                break;
            }
        }
        xSemaphoreGive(action_manager.mutex);
        
        vTaskDelete(NULL);
        return;
    }

    if(action->execute)
    {
        action->execute(action->user_data);
    }
    // 更新状态
    action->state = ACTION_STATE_COMPLETED;

    // 调用完成回调
    if (action->complete) {
        action->complete(action->user_data);
    }
    xSemaphoreTake(action_manager.mutex, portMAX_DELAY);/// 从动作数组中移除
    for (int i = 0; i < 10; i++) {
        if (action_manager.actions[i] == action) {
            action_manager.actions[i] = NULL;
            action_manager.action_count--;
            break;
        }
    }
    xSemaphoreGive(action_manager.mutex);
    vTaskDelete(NULL);// 任务完成
}

// 发送动作
uint32_t ActionSend(Action_t *action) {
    // 发送到队列
    xQueueSend(action_manager.action_queue, &action, portMAX_DELAY);
    
    return action->id; // ID会在ActionDealTask中设置
}

// 发送不可打断动作
uint32_t ActionSendUninterruptable(void (*execute)(void *user_data),
                                 void (*complete)(void *user_data),
                                 void *user_data) {
    // 创建动作
    Action_t *action = pvPortMalloc(sizeof(Action_t));
    if (action != NULL) {
        action->type = ACTION_TYPE_UNINTERRUPTABLE;
        action->state = ACTION_STATE_IDLE;
        action->execute = execute;
        action->complete = complete;
        action->user_data = user_data;
        action->task_handle = NULL;
        
        // 发送到队列
        ActionSend(action);
        return action->id;
    }
    return 0;
}

// 发送可打断动作
uint32_t ActionSendInterruptable(void (*execute)(void *user_data),
                               void (*complete)(void *user_data),
                               void *user_data) {
    // 创建动作
    Action_t *action = pvPortMalloc(sizeof(Action_t));
    if (action != NULL) {
        action->type = ACTION_TYPE_INTERRUPTABLE;
        action->state = ACTION_STATE_IDLE;
        action->execute = execute;
        action->complete = complete;
        action->user_data = user_data;
        action->task_handle = NULL;
        
        // 发送到队列
        ActionSend(action);
        return action->id;
    }
    return 0;
}

/**
 * @brief 中断指定的可打断动作
 * @param action_id 要中断的动作ID，0表示中断所有可打断动作
 * @return 成功中断的动作数量
 */
uint8_t ActionInterruptSpecificInterruptable(uint8_t action_id)
{
    uint8_t interrupted_count = 0;

    xSemaphoreTake(action_manager.mutex, portMAX_DELAY);
    for (int i = 0; i < 10; i++) {
        Action_t *action = action_manager.actions[i];
        if (action != NULL && action->type == ACTION_TYPE_INTERRUPTABLE) {
            // 检查是否是要中断的动作
            if (action_id == 0 || action->id == action_id) {
                // 标记为已打断
                action->state = ACTION_STATE_INTERRUPTED;
                
                // 删除任务
                if (action->task_handle != NULL) {
                    vTaskDelete(action->task_handle);
                    action->task_handle = NULL;
                }
                
                // 调用完成回调
                if (action->complete) {
                    action->complete(action->user_data);
                }
                
                // 从数组中移除
                action_manager.actions[i] = NULL;
                action_manager.action_count--;
                interrupted_count++;
                
                // 如果是指定ID的动作，找到后可以提前退出
                if (action_id != 0) {
                    break;
                }
            }
        }
    }
    xSemaphoreGive(action_manager.mutex);
    return interrupted_count;
}

// 获取当前动作数量
uint32_t ActionGetCount(void) {
    xSemaphoreTake(action_manager.mutex, portMAX_DELAY);
    uint32_t count = action_manager.action_count;
    xSemaphoreGive(action_manager.mutex);
    return count;
}

// 检查是否有动作在执行
bool ActionIsExecuting(void) {
    return ActionGetCount() > 0;
}
