#ifndef __ACTION_CONFIG_H__
#define __ACTION_CONFIG_H__

#include "stdint.h"
#include "stdbool.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

// 动作类型定义
typedef enum {
    ACTION_TYPE_UNINTERRUPTABLE,  // 不可打断动作
    ACTION_TYPE_INTERRUPTABLE     // 可打断动作
} ActionType_t;

// 动作状态定义
typedef enum {
    ACTION_STATE_IDLE,          // 空闲
    ACTION_STATE_EXECUTING,     // 执行中
    ACTION_STATE_COMPLETED,     // 已完成
    ACTION_STATE_INTERRUPTED,   // 已打断
    ACTION_STATE_ERROR          // 错误
} ActionState_t;

// 动作基本结构
typedef struct {
    ActionType_t type;              // 动作类型
    ActionState_t state;            // 动作状态
    uint32_t id;                    // 动作ID
    void (*execute)(void *user_data); // 执行函数
    void (*complete)(void *user_data); // 完成回调
    void *user_data;                // 用户数据
    TaskHandle_t task_handle;       // 任务句柄
} Action_t;

// 动作管理器结构
typedef struct {
    QueueHandle_t action_queue;     // 动作队列
    Action_t *actions[10];          // 最大5个并行动作
    uint32_t action_count;          // 当前动作数量
    uint32_t next_action_id;        // 下一个动作ID
    SemaphoreHandle_t mutex;        // 互斥锁
} ActionManager_t;


void ActionManagerInit(void);//初始化动作管理
uint32_t ActionSendUninterruptable(void (*execute)(void *user_data),
                                 void (*complete)(void *user_data),
                                 void *user_data);//发送不可打断动作
uint32_t ActionSendInterruptable(void (*execute)(void *user_data),
                               void (*complete)(void *user_data),
                               void *user_data);//发送可打断动作
uint8_t ActionInterruptSpecificInterruptable(uint8_t action_id);//中断指定的可打断动作
uint32_t ActionGetCount(void);
bool ActionIsExecuting(void);

#endif
