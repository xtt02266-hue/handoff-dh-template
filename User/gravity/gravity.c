#include "gravity.h"
#include "gravity_config.h"
#include "dm4310_ctrl.h"
#include "motor_protect.h"
#include <math.h>

/*
 * 将参考工程的重力补偿模型接入本工程。
 *
 * 说明：
 *   - 输入使用 motor[i].para.pos（电机反馈关节角，单位 rad）
 *   - 使用 encoder_zero / position_sign 映射，参考姿态模型角为零，
 *     由 gravity_config.h 中的 GRAVITY_JOINT_MAP 决定
 *   - 输出直接写入 motor[i].cmd.tor_set，后续由 ctrl_set() 拷贝并发送
 */
void Gravity_Arm_Calculate(float torque_cmd[6])
{
    float encoder_angle[ROBOT_DOF];

    for (uint8_t i = 0; i < ROBOT_DOF; i++)
    {
        encoder_angle[i] = motor[i].para.pos;
    }

    GravityComp_Calculate(encoder_angle, torque_cmd);

    // 任何一轴算出无效力矩，就停止控制并清空全部输出。
    for (uint8_t i = 0; i < ROBOT_DOF; i++)
    {
        if (!isfinite(torque_cmd[i]))
        {
            global_system_halt_flag = 1;
            for (uint8_t j = 0; j < ROBOT_DOF; j++)
            {
                torque_cmd[j] = 0.0f;
            }
            return;
        }
    }
}

void Gravity_Arm_Run(void)
{
    float gravity_torque_cmd[ROBOT_DOF];
    Gravity_Arm_Calculate(gravity_torque_cmd);

    for (uint8_t i = 0; i < ROBOT_DOF; i++)
    {
        motor[i].cmd.tor_set = gravity_torque_cmd[i];
    }
}
