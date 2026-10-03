#ifndef __MOTOR_PROTECT_H
#define __MOTOR_PROTECT_H

#include "main.h"

typedef struct {
    float max_tor;      // 最大力矩
    float max_vel;      // 最大速度
    float max_pos;      // 最大位置
    float min_pos;      // 最小位置
} motor_limit_t;

extern const motor_limit_t motor_limits[6];
extern volatile uint8_t global_system_halt_flag;
void motor_protect_run(void);
uint8_t motor_speed_limited(void);
void motor_disable_detect(void);


#endif

