#pragma once
#include <stdint.h>
#include <stddef.h>
typedef void *osThreadId_t;
typedef int osThreadState_t;
typedef int osPriority_t;
typedef uint32_t TickType_t;
enum { osThreadError = -1, osPriorityNormal = 0 };
typedef struct { const char *name; uint32_t stack_size; osPriority_t priority; } osThreadAttr_t;
osThreadState_t osThreadGetState(osThreadId_t thread);
osThreadId_t osThreadNew(void (*task)(void *), void *arg, const osThreadAttr_t *attr);
TickType_t xTaskGetTickCount(void);
void vTaskDelayUntil(TickType_t *last, TickType_t period);
#define pdMS_TO_TICKS(ms) (ms)
#define portMAX_DELAY UINT32_MAX
