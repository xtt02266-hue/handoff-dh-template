#ifndef __FSM_H
#define __FSM_H
#include "main.h"
#include "dm4310_ctrl.h"
typedef enum
{
    fsm_pos_init,
    fsm_lock,
    fsm_geforce_off,
    fsm_judge,
	fsm_protect,
	wait_switch,
	fsm_test,
} fsm_state_t;

typedef union
{
	uint8_t data[30];
	struct
	{
		struct {
			float num;
		} __attribute__((packed)) motor[motor_num]; 
		uint8_t Button_state[2];//Button_num
	}__attribute__((packed)) param;
		
}fsm_param_t;

typedef struct
{
    fsm_state_t state;
    fsm_param_t to_manipulator_data;
		int TEST;
} fsm_t;

extern float arm[6];
extern float q[6];
extern float q_ikine[6];



void fsm_run(fsm_t* fsm);
void fsm_run_test(fsm_t* fsm); 


#endif
