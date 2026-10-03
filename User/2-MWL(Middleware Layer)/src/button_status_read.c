#include "button_status_read.h"
#include "main.h"
#include "fsm.h"

uint8_t lock_button_flag; // 0 表示开关在低位，1 表示开关在高位

static void lock_button_init(void) 
{
    if(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET) {
        lock_button_flag = 1; 
    } else {
        lock_button_flag = 0; 
    }
}

uint8_t lock_button_enable(void) 
{
    static uint8_t init_flag = 0;
    if (init_flag == 0) {
        lock_button_init();
        init_flag = 1;
    }
    if(lock_button_flag&& (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_RESET)) {
        return 1; // 允许进入工作态
    } else if (!lock_button_flag && (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET)) {
        return 1; // 允许进入工作态
    }
    else {
        return 0; // 继续等待
    }
}

uint8_t lock_button_judge(void) 
{

        if(lock_button_flag == 0) {
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET) {
                return 1; 
            } else {
                return 0; 
            }
        } else {
            if (HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_9) == GPIO_PIN_SET) {
                return 0; 
            } else {
                return 1; 
            }
        }
    
}

void Switch_Callback(fsm_t * fsm_button_callback)
{
    fsm_button_callback->to_manipulator_data.param.Button_state[0] =
		(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8) == GPIO_PIN_SET) ? 0xFF : 0x00;
    fsm_button_callback->to_manipulator_data.param.Button_state[1] =
		(HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_7) == GPIO_PIN_SET) ? 0xFF : 0x00;
}




