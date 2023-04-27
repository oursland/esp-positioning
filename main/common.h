#pragma once

enum {
    EVENT_RANGE_MEASUREMENT,
    EVENT_IMU_MEASUREMENT,
} event_type_t;

typedef struct {

} range_measurement_t;

typedef struct {

} imu_measurement_t;

typedef struct {
    uint32_t type;
    union {
        range_measurement_t range_measurement;
        imu_measurement_t imu_measurement;
    };
} event_t;

typedef struct {
    BaseType_t task_handle;
    QueueHandle_t task_queue;
    QueueHandle_t main_queue;
} task_t;
