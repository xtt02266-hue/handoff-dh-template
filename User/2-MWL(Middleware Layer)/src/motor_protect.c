#include "dm4310_ctrl.h"
#include "dm4310_drv.h"

#include "fsm.h"
#include "math.h"
#include "motor_protect.h"

void motor_disable_detect(void);
static void motor_pos_protect(void);
static void motor_vel_protect(void);
static void motor_tor_protect(void);
static void motor_fault_detect(void);
static void motor_safe_mode(void);


#define PROTECT_POS_HARD_MARGIN        0.20f
#define PROTECT_VEL_HARD_SCALE         1.50f

volatile uint8_t global_system_halt_flag = 0; // 全局系统停机标志位

const motor_limit_t motor_limits[6] = {
    {.max_tor = 0.6f, .max_vel = 10, .max_pos = 2.0f, .min_pos = -2.0f},
    {.max_tor = 2.9f, .max_vel = 10, .max_pos = 1.0f, .min_pos = -2.4f},
    {.max_tor = 2.3f, .max_vel = 10, .max_pos = 0.8f, .min_pos = -2.2f},
    {.max_tor = 1.2f, .max_vel = 10, .max_pos = 3.0f, .min_pos = -3.0f},
    {.max_tor = 1.2f, .max_vel = 10, .max_pos = 2.0f, .min_pos = -2.0f},
    {.max_tor = 0.6f, .max_vel = 10, .max_pos = 3.3f, .min_pos = -3.2f},
};

/* 每周期检查硬故障，再处理位置、前馈和速度；故障当周期覆盖输出。 */
void motor_protect_run(void)
{
    motor_fault_detect();
    motor_disable_detect();
    // 如果系统已经被确诊熔断，则直接持续维持安全模式，屏蔽其他一切控制和保护逻辑
    if (global_system_halt_flag) {
        motor_safe_mode();
        return;
    }

    // 强制每周期运行保护函数：
    // 否则一旦实际位置/速度退回正常区间，恢复控制权（例如解除刚度墙）的逻辑就不会执行，导致电机一直卡在保护的粘滞状态。
    motor_pos_protect();
    motor_tor_protect();
    motor_vel_protect();
    motor_fault_detect();
    if (global_system_halt_flag) {
        motor_safe_mode();
    }
}

/* 限制目标位置，实测越界时施加朝向边界的恢复力。增益仅覆盖本周期。 */
static void motor_pos_protect(void)
{
    for (int i = 0; i < motor_num; i++) {
        float max_p = motor_limits[i].max_pos;
        float min_p = motor_limits[i].min_pos;

        // 1. 无条件钳制目标指令：防止上层下发越界的目标位置
        if (motor[i].cmd.pos_set > max_p) {
            motor[i].cmd.pos_set = max_p;
        } else if (motor[i].cmd.pos_set < min_p) {
            motor[i].cmd.pos_set = min_p;
        }

        // 2. 对于物理超过限位的情况（比如外力推动），增加虚拟墙阻力
        if (motor[i].para.pos > max_p) {

            motor[i].cmd.pos_set = max_p;
            motor[i].cmd.vel_set = 0;
            if (motor[i].cmd.kp_set < 10.0f) motor[i].cmd.kp_set = 10.0f;
            if (motor[i].cmd.kd_set < 2.0f) motor[i].cmd.kd_set = 2.0f;
        }
        else if (motor[i].para.pos < min_p) {

            motor[i].cmd.pos_set = min_p;
            motor[i].cmd.vel_set = 0;
            if (motor[i].cmd.kp_set < 10.0f) motor[i].cmd.kp_set = 10.0f;
            if (motor[i].cmd.kd_set < 2.0f) motor[i].cmd.kd_set = 2.0f;
        }

    }
}

/* 超速时使用零目标速度制动；低于退出阈值后由当前模式重新提供增益。 */
static uint8_t in_vel_limit[motor_num] = {0};

static void motor_vel_protect(void)
{
    for (int i = 0; i < motor_num; i++) {
        float speed = fabsf(motor[i].para.vel);
        if (speed > motor_limits[i].max_vel) {
            in_vel_limit[i] = 1;
        } else if (speed < motor_limits[i].max_vel * 0.8f) {
            in_vel_limit[i] = 0;
        }

        if (in_vel_limit[i]) {
            motor[i].cmd.vel_set = 0.0f;
            if (motor[i].cmd.kd_set < 2.0f) {
                motor[i].cmd.kd_set = 2.0f;
            }

            float drive = motor[i].cmd.kp_set *
                (motor[i].cmd.pos_set - motor[i].para.pos)
                + motor[i].cmd.tor_set;
            if (drive * motor[i].para.vel > 0.0f) {
                motor[i].cmd.tor_set = 0.0f;
                if ((motor[i].cmd.pos_set - motor[i].para.pos) *
                    motor[i].para.vel > 0.0f) {
                    motor[i].cmd.kp_set = 0.0f;
                }
            }
        }
    }
}

uint8_t motor_speed_limited(void)
{
    for (int i = 0; i < motor_num; i++) {
        if (in_vel_limit[i]) {
            return 1;
        }
    }
    return 0;
}

/* 仅限制前馈力矩，不代表限制电机内部 P/D 与前馈的总力矩。 */
static void motor_tor_protect(void)
{

    for (int i = 0; i < motor_num; i++) {
        float max_t = motor_limits[i].max_tor;
        // 直接限幅下发的前馈力矩指令（对目标指令进行限幅，而不是依赖反馈力矩）
        // 否则容易导致外界推力大时产生错误的正反馈粘滞力
        if (motor[i].cmd.tor_set > max_t) {
            motor[i].cmd.tor_set = max_t;
        }
        else if (motor[i].cmd.tor_set < -max_t) {
            motor[i].cmd.tor_set = -max_t;
        }
    }
}

/* 检查反馈，不自动重新使能被停用的电机。 */
void motor_disable_detect(void)
{
    if (!motor_feedback_ready()) {
        global_system_halt_flag = 1;
    }
}

/* 严重越界或硬超速时立即停止正常控制。 */
static void motor_fault_detect(void)
{
    for (int i = 0; i < motor_num; i++)
    {
        float hard_max_vel = motor_limits[i].max_vel * PROTECT_VEL_HARD_SCALE;
        float hard_max_pos = motor_limits[i].max_pos + PROTECT_POS_HARD_MARGIN;
        float hard_min_pos = motor_limits[i].min_pos - PROTECT_POS_HARD_MARGIN;

        // 1. 正转或反转速度超过上限。
        if (motor[i].para.vel > hard_max_vel ||
            motor[i].para.vel < -hard_max_vel)
        {
            global_system_halt_flag = 1;
        }

        // 2. 实际位置超过上边界及允许裕量。
        if (motor[i].para.pos > hard_max_pos)
        {
            global_system_halt_flag = 1;
        }

        // 3. 实际位置超过下边界及允许裕量。
        if (motor[i].para.pos < hard_min_pos)
        {
            global_system_halt_flag = 1;
        }
    }
}

static void motor_safe_mode(void)
{
    for (int i = 0; i < motor_num; i++) {
        Set_MIT_PVT(&motor[i], 0, 0, 0);
        Set_MIT_PD(&motor[i], 0, 2);
    }
}
