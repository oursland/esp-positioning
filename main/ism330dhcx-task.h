#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"

extern QueueHandle_t ism330dhcx_interrupt_queue;

void ism330dhcx_task_func(void *args);
