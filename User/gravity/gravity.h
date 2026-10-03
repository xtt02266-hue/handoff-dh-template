#ifndef __GRAVITY_H
#define __GRAVITY_H

/*
 * 重力补偿与现有工程的接入层。
 *
 * Gravity_Arm_Run() 内部：
 *   1. 读取 6 个电机的实际关节角 motor[i].para.pos
 *   2. 调用 GravityComp_Calculate() 计算补偿力矩 (N·m)
 *   3. 写入 motor[i].cmd.tor_set
 *
 * 发送、限幅、通讯仍由现有控制链路负责。
 */
void Gravity_Arm_Run(void);

/* Compute fresh motor-command compensation torques (six values, N.m),
 * without changing commands or sending CAN. Includes configured gain.
 */
void Gravity_Arm_Calculate(float torque_cmd[6]);

#endif
