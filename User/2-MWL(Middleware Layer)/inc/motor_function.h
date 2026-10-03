#ifndef __MOTOR_FUNCTION_H
#define __MOTOR_FUNCTION_H

/*
 * 锁定模式的动态刚度参数：
 *
 * 目标 P 增益计算公式：
 * target_kp = MOTOR_LOCK_KP_MIN
 *           + MOTOR_LOCK_KP_PER_TORQUE * fabsf(当前重力补偿力矩);
 * 随后受 MOTOR_LOCK_KP_MAX 限制，并经平滑系数逐周期更新。
 */

/* 最小 P 增益（基础锁定刚度）。即使补偿力矩为 0，P 也不会低于该值。 */
#define MOTOR_LOCK_KP_MIN         0.5f

/* 力矩到 P 增益的比例系数。补偿力矩绝对值每增加 1 N·m，目标 P 增加 0.35。 */
#define MOTOR_LOCK_KP_PER_TORQUE  0.35f

/* P 增益上限，防止重力补偿力矩较大时锁定刚度过高。 */
#define MOTOR_LOCK_KP_MAX         12.0f

/* P 增益平滑系数，范围通常为 0~1。0.10 表示每周期走完目标差值的 10%。 */
#define MOTOR_LOCK_KP_SMOOTHING   0.30f

/* 锁定模式的 D 增益（阻尼），用于抑制速度和振荡；所有关节当前使用同一数值。 */
#define MOTOR_LOCK_KD             0.02f

extern uint8_t lock_flag;
void motor_pos_reset(void);
void motor_lock(void);
uint8_t motor_pos_init(void);
uint8_t motor_pos_run(void) ;
void motor_pos_init_reset(void);
void motor_pos_run_reset(void);
void motor_wait_feedback_update(void);

#endif
