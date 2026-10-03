#include "dm4310_drv.h"
#include "dm4310_ctrl.h"
#include "can_driver.h"
#include "key_modlue.h"
#include "string.h"
#include "math.h"
#include "motor_protect.h"


motor_t motor[motor_num];
static volatile uint32_t feedback_tick[motor_num];
static volatile uint8_t feedback_seen[motor_num];

uint8_t motor_feedback_ready(void)
{
    uint32_t now = HAL_GetTick();
    for (int i = 0; i < motor_num; i++) {
        if (!feedback_seen[i] || (uint32_t)(now - feedback_tick[i]) > 100U ||
            motor[i].para.state != 1) {
            return 0;
        }
    }
    return 1;
}


int8_t motor_id = 1;

/**
************************************************************************
* @brief:      	dm4310_motor_init: DM4310电机初始化函数
* @param:      	void
* @retval:     	void
* @details:    	初始化三个DM4310型号的电机，设置默认参数和控制模式。
*               分别初始化Motor1和Motor2，设置ID、控制模式和命令模式等信息。
************************************************************************
**/ //60 1
void dm4310_motor_init(void)
{
	// 初始化Motor1和Motor2的电机结构
	memset(&motor[Motor1], 0, sizeof(motor[Motor1]));
	memset(&motor[Motor2], 0, sizeof(motor[Motor2]));
	memset(&motor[Motor3], 0, sizeof(motor[Motor3]));
	memset(&motor[Motor4], 0, sizeof(motor[Motor4]));
	memset(&motor[Motor5], 0, sizeof(motor[Motor5]));
	memset(&motor[Motor6], 0, sizeof(motor[Motor6]));

	// 设置Motor1的电机信息
	motor[Motor1].id = 1;
	motor[Motor1].ctrl.mode = 0;		// 0: MIT模式   1: 位置速度模式   2: 速度模式
	motor[Motor1].ctrl.kd_set = 0;
	motor[Motor1].ctrl.kp_set = 0;
	motor[Motor1].cmd.kd_set =	0;
	motor[Motor1].cmd.mode = 0;

	// 设置Motor2的电机信息
	motor[Motor2].id = 2;
	motor[Motor2].ctrl.mode = 0;
	motor[Motor2].ctrl.kd_set = 0;
	motor[Motor2].ctrl.kp_set = 0;
	motor[Motor2].cmd.kd_set = 0;
	motor[Motor2].cmd.mode = 0;
	
	// 设置Motor3的电机信息
	motor[Motor3].id = 3;
	motor[Motor3].ctrl.mode = 0;
	motor[Motor3].ctrl.kd_set = 0;
	motor[Motor3].ctrl.kp_set = 0;
	motor[Motor3].cmd.kd_set = 0;
	motor[Motor3].cmd.mode = 0;

	// 设置Motor4的电机信息
	motor[Motor4].id = 4;
	motor[Motor4].ctrl.mode = 0;
	motor[Motor4].ctrl.kd_set = 0;
	motor[Motor4].ctrl.kp_set = 0;
	motor[Motor4].cmd.kd_set = 0;
	motor[Motor4].cmd.mode = 0;

	// 设置Motor2的电机信息
	motor[Motor5].id = 5;
	motor[Motor5].ctrl.mode = 0;
	motor[Motor5].ctrl.kd_set = 0;
	motor[Motor5].ctrl.kp_set = 0;
	motor[Motor5].cmd.kd_set = 0;
	motor[Motor5].cmd.mode = 0;

	// 设置Motor2的电机信息
	motor[Motor6].id = 6;
	motor[Motor6].ctrl.mode = 0;
	motor[Motor6].ctrl.kd_set = 0;
	motor[Motor6].ctrl.kp_set = 0;
	motor[Motor6].cmd.kd_set = 0;
	motor[Motor6].cmd.mode = 0;
}
/**
************************************************************************
* @brief:      	motor_para_add: 修改电机参数函数
* @param[in]:   motor:   指向motor_t结构的指针，包含电机参数信息
* @retval:     	void
* @details:    	根据当前LCD标志位（lcd_flag），修改指定电机的参数信息。
*               根据lcd_flag的不同，可修改电机ID、控制模式、位置设定、速度设定、
*               扭矩设定、比例增益和微分增益等参数。
************************************************************************
**/
void motor_para_add(motor_t *motor)
{
	switch(lcd_flag)
	{
		case 0:
			motor_id += 1;
			if (motor_id > motor_num)
				motor_id = 1;
			break;
		case 1:
			motor->ctrl.mode += 1;
			if (motor->ctrl.mode > 2)
				motor->ctrl.mode = 0;
			break;
		case 2:
			motor->cmd.pos_set += 1.0f;
			break;
		case 3:
			motor->cmd.vel_set += 1.0f;
			break;
		case 4:
			motor->cmd.tor_set += 1.0f;
			break;
		case 5:
			motor->cmd.kp_set += 1.0f;
			break;
		case 6:
			motor->cmd.kd_set += 0.5f;
			break;
	}
}
/**
************************************************************************
* @brief:      	motor_para_minus: 减小电机参数函数
* @param[in]:   motor:   指向motor_t结构的指针，包含电机参数信息
* @retval:     	void
* @details:    	根据当前LCD标志位（lcd_flag），减小指定电机的参数信息。
*               根据lcd_flag的不同，可减小电机ID、控制模式、位置设定、速度设定、
*               扭矩设定、比例增益和微分增益等参数。
************************************************************************
**/
void motor_para_minus(motor_t *motor)
{
	switch(lcd_flag)
	{
		case 0:
			motor_id -= 1;
			if (motor_id < 1)
				motor_id = motor_num;
			break;
		case 1:
			motor->ctrl.mode -= 1;
			if (motor->ctrl.mode < 0)
				motor->ctrl.mode = 2;
			break;
		case 2:
			motor->cmd.pos_set -= 1.0f;
			break;
		case 3:
			motor->cmd.vel_set -= 1.0f;
			break;
		case 4:
			motor->cmd.tor_set -= 1.0f;
			break;
		case 5:
			motor->cmd.kp_set -= 1.0f;
			break;
		case 6:
			motor->cmd.kd_set -= 0.5f;
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_enable: 启用电机控制函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），启用对应的电机控制。
*               设置指定电机的启动标志，并调用dm4310_enable函数启用电机。
************************************************************************
**/
void ctrl_enable(void)
{
    if (global_system_halt_flag) {
        return;
    }
	for (int i = 0; i < 3; i++)
	{
		motor[i].start_flag = 1;
		dm4310_enable(&hcan1, &motor[i]);
	}
	for (int i = 3; i < 6; i++)
	{
		motor[i].start_flag = 1;
		dm4310_enable(&hcan2, &motor[i]);//dm4310_enable(&hcan2, &motor[i]);
	}
}
/**
************************************************************************
* @brief:      	ctrl_disable: 禁用电机控制函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），禁用对应的电机控制。
*               设置指定电机的启动标志为0，并调用dm4310_disable函数禁用电机。
************************************************************************
**/
void ctrl_disable(void)
{
	switch(motor_id)
	{
		case 1:
			// 禁用Motor1的电机控制
			motor[Motor1].start_flag = 0;
			dm4310_disable(&hcan1, &motor[Motor1]);
			break;
		case 2:
			// 禁用Motor2的电机控制
			motor[Motor2].start_flag = 0;
			dm4310_disable(&hcan2, &motor[Motor2]);
			break;
		case 3:
			// 禁用Motor2的电机控制
			motor[Motor3].start_flag = 0;
			dm4310_disable(&hcan1, &motor[Motor3]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_set: 设置电机参数函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），设置对应电机的参数。
*               调用dm4310_set函数设置指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_set(void)
{
	// 设置Motor1的电机参数
	dm4310_set(&motor[Motor1]);
	// 设置Motor2的电机参数
	dm4310_set(&motor[Motor2]);
	// 设置Motor3的电机参数
	dm4310_set(&motor[Motor3]);
	//设置Motor4的电机参数
	dm4310_set(&motor[Motor4]);
	//设置Motor5的电机参数
	dm4310_set(&motor[Motor5]);
	//设置Motor6的电机参数
	dm4310_set(&motor[Motor6]);

}
/**
************************************************************************
* @brief:      	ctrl_clear_para: 清除电机参数函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），清除对应电机的参数。
*               调用dm4310_clear函数清除指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_clear_para(void)
{
	switch(motor_id)
	{
		case 1:
			// 清除Motor1的电机参数
			dm4310_clear_para(&motor[Motor1]);
			break;
		case 2:
			// 清除Motor2的电机参数
			dm4310_clear_para(&motor[Motor2]);
			break;
		case 3:
			// 清除Motor2的电机参数
			dm4310_clear_para(&motor[Motor3]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_clear_err: 清除电机错误信息
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），清除对应电机的参数。
*               调用dm4310_clear函数清除指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_clear_err(void)
{
	switch(motor_id)
	{
		case 1:
			// 清除Motor1的电机错误参数
			dm4310_clear_err(&hcan1, &motor[Motor1]);
			break;
		case 2:
			// 清除Motor2的电机错误参数
			dm4310_clear_err(&hcan2, &motor[Motor2]);
			break;
		case 3:
			// 清除Motor3的电机错误参数
			dm4310_clear_err(&hcan1, &motor[Motor3]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_add: 增加电机参数函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），增加对应电机的参数。
*               调用motor_para_add函数增加指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_add(void)
{
	switch(motor_id)
	{
		case 1:
			// 增加Motor1的电机参数
			motor_para_add(&motor[Motor1]);
			break;
		case 2:
			// 增加Motor2的电机参数
			motor_para_add(&motor[Motor2]);
			break;
		case 3:
			// 增加Motor3的电机参数
			motor_para_add(&motor[Motor3]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_minus: 减少电机参数函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），减少对应电机的参数。
*               调用motor_para_minus函数减少指定电机的参数，以响应外部命令。
************************************************************************
**/
void ctrl_minus(void)
{
	switch(motor_id)
	{
		case 1:
			// 减少Motor1的电机参数
			motor_para_minus(&motor[Motor1]);
			break;
		case 2:
			// 减少Motor2的电机参数
			motor_para_minus(&motor[Motor2]);
			break;
		case 3:
			// 减少Motor3的电机参数
			motor_para_minus(&motor[Motor3]);
			break;
	}
}
/**
************************************************************************
* @brief:      	ctrl_send: 发送电机控制命令函数
* @param:      	void
* @retval:     	void
* @details:    	根据当前电机ID（motor_id），向对应电机发送控制命令。
*               调用dm4310_ctrl_send函数向指定电机发送控制命令，以响应外部命令。
************************************************************************
**/
void ctrl_send(void)
{
	for (int i = 0; i < 3; i++)
	{
		motor[i].start_flag = 1;
		dm4310_ctrl_send(&hcan1, &motor[i]);
	}
	for (int i = 3; i < 6; i++)
	{
		motor[i].start_flag = 1;
		dm4310_ctrl_send(&hcan2, &motor[i]);//dm4310_ctrl_send(&hcan2, &motor[i]);
	}
}
/**
************************************************************************
* @brief:      	can1_rx_callback: CAN1接收回调函数
* @param:      	void
* @retval:     	void
* @details:    	处理CAN1接收中断回调，根据接收到的ID和数据，执行相应的处理。
*               当接收到ID为0时，调用dm4310_fbdata函数更新Motor的反馈数据。
************************************************************************
**/
void can1_rx_callback(void)
{
	uint16_t rec_id = 0;
	uint8_t rx_data[8] = {0};
    if (canx_receive_data(&hcan1, &rec_id, rx_data) != 8) { return; }
	switch (rec_id)
	{
		case 0x11:
			dm4310_fbdata(&motor[Motor1], rx_data);
            feedback_tick[Motor1] = HAL_GetTick();
            feedback_seen[Motor1] = 1;
			break;
		case 0x12:
			dm4310_fbdata(&motor[Motor2], rx_data);
            feedback_tick[Motor2] = HAL_GetTick();
            feedback_seen[Motor2] = 1;
			break;
		case 0x13:
			dm4310_fbdata(&motor[Motor3], rx_data);
            feedback_tick[Motor3] = HAL_GetTick();
            feedback_seen[Motor3] = 1;
			break;

	}
}
/**
************************************************************************
* @brief:      	can1_rx_callback: CAN2接收回调函数
* @param:      	void
* @retval:     	void
* @details:    	处理CAN1接收中断回调，根据接收到的ID和数据，执行相应的处理。
*               当接收到ID为0时，调用dm4310_fbdata函数更新Motor的反馈数据。
************************************************************************
**/
void can2_rx_callback(void)
{
    uint16_t rec_id = 0;
	
	uint8_t rx_data[8] = {0};
    if (canx_receive_data(&hcan2, &rec_id, rx_data) != 8) { return; }
	switch (rec_id)
	{
		case 0x14:
			dm4310_fbdata(&motor[Motor4], rx_data);
            feedback_tick[Motor4] = HAL_GetTick();
            feedback_seen[Motor4] = 1;
			break;
		case 0x15:
			dm4310_fbdata(&motor[Motor5], rx_data);
            feedback_tick[Motor5] = HAL_GetTick();
            feedback_seen[Motor5] = 1;
			break;
		case 0x16:
			dm4310_fbdata(&motor[Motor6], rx_data);
            feedback_tick[Motor6] = HAL_GetTick();
            feedback_seen[Motor6] = 1;
			break;

	}
}



