#include "dm4310_ctrl.h"
#include "dm4310_drv.h"

#include "fsm.h"
#include "math.h"
#include "motor_function.h"
#include "gravity.h"
#include "motor_protect.h"

uint8_t lock_flag;
static uint8_t motor_pos_init_reset_request = 1;
static uint8_t motor_pos_run_first_run = 1;

/* motor_pos_init 复位目标位置(单位: rad)，按关节0~5顺序配置 */
static const float motor_reset_target_pos_init[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
/* motor_pos_run 复位目标位置(单位: rad)，按关节0~5顺序配置 */
static const float motor_reset_target_pos_run[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};

void motor_pos_run_reset(void)
{
    motor_pos_run_first_run = 1;
}

void motor_pos_init_reset(void)
{
    motor_pos_init_reset_request = 1;
}

void motor_wait_feedback_update(void)
{
    for (int i = 0; i < motor_num; i++) {
        motor[i].cmd.pos_set = motor[i].para.pos;
        motor[i].cmd.vel_set = 0.0f;
        motor[i].cmd.tor_set = 0.0f;
        motor[i].cmd.kp_set = 0.0f;
        motor[i].cmd.kd_set = 0.0f;
    }
    ctrl_set();
    ctrl_send();
}

void motor_pos_reset(void) 
{
    uint8_t mode_id = 0;// 0: MIT模式   1: 位置速度模式   2: 速度模式
    for (int i = 0; i < 3; i++) {
        save_pos_zero(&hcan1, motor[i].id, mode_id);
    }
    for (int i = 3; i < 6; i++) {
        save_pos_zero(&hcan2, motor[i].id, mode_id);
    }
}


void motor_lock(void) //柔性锁定
{ 
    static float motor_lock_pos[6];
    static float motor_lock_kp[6];
    float gravity_torque[6];
    if (!motor_feedback_ready()) {
        global_system_halt_flag = 1;
        return;
    }
    Gravity_Arm_Calculate(gravity_torque);
    
    if (lock_flag == 0) {
        for(int i = 0; i < motor_num; i++) {
            motor_lock_pos[i] = motor[i].para.pos;
            motor_lock_kp[i] = MOTOR_LOCK_KP_MIN;
        }
        lock_flag = 1;
    }
    
    for(int i = 0; i < motor_num; i++) {
        float target_kp = MOTOR_LOCK_KP_MIN
                        + MOTOR_LOCK_KP_PER_TORQUE * fabsf(gravity_torque[i]);
        if (target_kp > MOTOR_LOCK_KP_MAX) target_kp = MOTOR_LOCK_KP_MAX;
        motor_lock_kp[i] += MOTOR_LOCK_KP_SMOOTHING * (target_kp - motor_lock_kp[i]);
        Set_MIT_PVT(&motor[i], motor_lock_pos[i], 0.0f, gravity_torque[i]);
        Set_MIT_PD(&motor[i], motor_lock_kp[i], MOTOR_LOCK_KD);
    }
		//motor_par_send();
}


static float home_position[motor_num];
static uint32_t home_start_ms;
static uint32_t home_last_ms;
static uint32_t home_stable_ms;
static uint8_t home_stable;

// 开始回零：检查目标范围，并记录当前实际位置作为起点。
static uint8_t motor_home_start(const float target[motor_num], uint32_t now)
{
    for (int i = 0; i < motor_num; i++)
    {
        if (target[i] < motor_limits[i].min_pos ||
            target[i] > motor_limits[i].max_pos)
        {
            global_system_halt_flag = 1;
            return 0;
        }

        if (motor[i].para.pos < motor_limits[i].min_pos ||
            motor[i].para.pos > motor_limits[i].max_pos)
        {
            global_system_halt_flag = 1;
            return 0;
        }

        home_position[i] = motor[i].para.pos;
    }

    home_start_ms = now;
    home_last_ms = now;
    home_stable = 0;
    return 1;
}

// 全部关节实际到位、速度足够小，并持续稳定 200 ms 才算完成。
static uint8_t motor_home_arrived(const float target[motor_num], uint32_t now)
{
    for (int i = 0; i < motor_num; i++)
    {
        float position_error = target[i] - motor[i].para.pos;
        float trajectory_error = target[i] - home_position[i];

        if (fabsf(position_error) > 0.05f)
        {
            home_stable = 0;
            return 0;
        }
        if (fabsf(motor[i].para.vel) > 0.1f)
        {
            home_stable = 0;
            return 0;
        }
        if (fabsf(trajectory_error) > 0.001f)
        {
            home_stable = 0;
            return 0;
        }
    }

    if (home_stable == 0)
    {
        home_stable = 1;
        home_stable_ms = now;
    }

    if (now - home_stable_ms >= 200U)
    {
        return 1;
    }
    return 0;
}

// 每周期调用：检查状态，向目标移动一步，然后检查是否到位。
static uint8_t motor_home_run(const float target[motor_num], uint8_t restart,
                              float speed, float kp)
{
    uint32_t now = HAL_GetTick();

    if (global_system_halt_flag != 0 || motor_feedback_ready() == 0)
    {
        global_system_halt_flag = 1;
        return 0;
    }

    if (restart != 0)
    {
        if (motor_home_start(target, now) == 0)
        {
            return 0;
        }
    }

    // 回零总时间不能超过 10 s，两次调用间隔不能超过 100 ms。
    uint32_t elapsed = now - home_start_ms;
    uint32_t step_ms = now - home_last_ms;
    home_last_ms = now;
    if (elapsed > 10000U || step_ms > 100U)
    {
        global_system_halt_flag = 1;
        return 0;
    }

    // 前 500 ms 逐渐增加速度和增益，避免开始回零时突然拉动。
    float ramp = (float)elapsed / 500.0f;
    if (ramp > 1.0f)
    {
        ramp = 1.0f;
    }
    float step = speed * ((float)step_ms * 0.001f) * ramp;

    for (int i = 0; i < motor_num; i++)
    {
        // 实际位置跟不上内部目标时停止，避免误差继续积累。
        if (fabsf(home_position[i] - motor[i].para.pos) > 0.5f)
        {
            global_system_halt_flag = 1;
            return 0;
        }

        float error = target[i] - home_position[i];
        if (motor_speed_limited() == 0)
        {
            if (error > step)
            {
                home_position[i] += step;
            }
            else if (error < -step)
            {
                home_position[i] -= step;
            }
            else
            {
                home_position[i] = target[i];
            }
        }

        motor[i].cmd.pos_set = home_position[i];
        motor[i].cmd.vel_set = 0.0f;
        motor[i].cmd.kp_set = kp * ramp;
        motor[i].cmd.kd_set = ramp;
    }

    return motor_home_arrived(target, now);
}

uint8_t motor_pos_init(void)
{
    uint8_t finished = motor_home_run(motor_reset_target_pos_init,
                                      motor_pos_init_reset_request, 0.6f, 15.0f);
    if (global_system_halt_flag == 0)
    {
        motor_pos_init_reset_request = 0;
    }
    return finished;
}

uint8_t motor_pos_run(void)
{
    uint8_t finished = motor_home_run(motor_reset_target_pos_run,
                                      motor_pos_run_first_run, 0.5f, 9.0f);
    if (global_system_halt_flag == 0)
    {
        motor_pos_run_first_run = 0;
    }
    return finished;
}
