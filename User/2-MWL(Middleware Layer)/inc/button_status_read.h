#ifndef __BUTTON_STATUS_READ_H
#define __BUTTON_STATUS_READ_H

#include "main.h"
#include "fsm.h"

uint8_t lock_button_enable(void);
uint8_t lock_button_judge(void);
void Switch_Callback(fsm_t * fsm_button_callback);

#endif
