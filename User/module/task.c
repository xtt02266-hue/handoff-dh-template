#include "task.h"
#include "scheduler.h"
//#include "vofa.h"
#include "adc_modlue.h"
#include "drivers.h"
#include "dm4310_ctrl.h"
#include "display.h"
#include "lcd.h"
#include "dm4310_drv.h"
#include "usart.h"
#include "remote_control.h"


/* 创建任务函数 */
void task_init(void)
{
	create_task(task_1ms, 1);
	create_task(task_2ms, 2);
	create_task(task_5ms, 5);
	create_task(task_10ms, 10);
	create_task(task_35ms, 35);
	create_task(task_100ms, 100);
	create_task(task_500ms, 500);
}

/* 1ms 任务函数 */
void task_1ms(void)
{
	// key_process();
	// get_key_adc();
}
/* 2ms 任务函数 */
void task_2ms(void)
{

}
/* 5ms 任务函数 */
void task_5ms(void)
{

}
/* 10ms 任务函数 */
void task_10ms(void)
{

}
/* 35ms 任务函数 */
void task_35ms(void)
{

}
/* 100ms 任务函数 */
void task_100ms(void)
{
	//	display();
}
/* 500ms 任务函数 */

void task_500ms(void)
{

}
