#include "remote_control.h"
#include "CRC8_CRC16.h"
#include "string.h"
#include "dm4310_ctrl.h"
#include "dm4310_drv.h"
#include "task.h" 
#include <stdlib.h>

//仅作用在该.C文件中(private)custom_robot_data_t *Tx_data
static void image_transimission_packet_handle(custom_robot_data_t *Tx_data,uint16_t data_length,uint16_t cmd_id,uint8_t CRC16_Length,const uint8_t * data);
//定义自定义控制器与机器人交互数据
custom_robot_data_t CC_Tx_data;
//定义键鼠与机器人交互数据
custom_robot_data_t CCKM_Tx_data;


/**
 * @brief 串级处理，数据拼接函数，将帧头、命令码、数据段、帧尾头拼接成一个数组
 * @param Tx_data 要拼接的目标数组指针
 * @param data_lenth 数据段长度
 * @param cmd_id 数据段的命令码
 * @param data 数据段的数据数组指针
 * @param CRC16_Length 数据段的CRC验证位长度
 */
static void image_transimission_packet_handle(custom_robot_data_t *Tx_data,uint16_t data_length,uint16_t cmd_id,uint8_t CRC16_Length,const uint8_t * data)
{
    //定义包序号
    static uint8_t seq=0;
    //帧头数据
    Tx_data->frame_header.SOF=0xA5;  
    Tx_data->frame_header.data_length=data_length;
    Tx_data->frame_header.seq=seq++;
    // 添加帧头 CRC8 校验位
    append_CRC8_check_sum((uint8_t *)&(Tx_data->frame_header),CRC8_FORMER_LENGTH); 
    //命令码ID
    Tx_data->cmd_id = cmd_id;
	    // 数据段
    memcpy(Tx_data->data,data, data_length);
    // 帧尾CRC16，整包校验
    append_CRC16_check_sum((uint8_t *)(Tx_data),CRC16_Length);
}


/**
 * @brief 自定义控制器/键鼠交互发送数据函数
 * @param *HUART 串口地址
  * @param controller_mode CC/CCKM_mode模式 
  * @param *data_src1/2 待处理数据源
 */

void image_transimission_link(UART_HandleTypeDef *HUART,enum controller_mode_t controller_mode,const uint8_t * data)
{
  // 定义自定义控制器数据
  static uint8_t CC_data_temp[CC_DATA_LENGTH] = {0};
  // 定义键鼠控制器数据
  static uint8_t CCKM_data_temp[CCKM_DATA_LENGTH] = {0};
  if (controller_mode == CC_mode)
  {
    // 将data传入CC_data
    memcpy(CC_data_temp, data, CC_DATA_LENGTH);
    // 串级处理&CC_Tx_data,
    image_transimission_packet_handle(&CC_Tx_data, CC_DATA_LENGTH, CC_CONTROLLER_CMD_ID, CC_CRC16_FORMER_LENGTH, CC_data_temp);
    // 串口一发送数据
    HAL_UART_Transmit_DMA(HUART, (uint8_t *)(&CC_Tx_data), CC_DATA_FRAME_LENGTH);
  }
  // CCKM模式
  else if (controller_mode == CCKM_mode)
  {

    //		    //将data传入CCKM_data
    //        memcpy(CCKM_data_temp,data,CCKM_DATA_LENGTH);
    //        //串级处理&CCKM_Tx_data,
    //        image_transimission_packet_handle(&CCKM_Tx_data,CCKM_DATA_LENGTH,CCKM_CONTROLLER_CMD_ID,CCKM_CRC16_FORMER_LENGTH);
    //        //串口一发送数据
    //        HAL_UART_Transmit_DMA(HUART, (uint8_t *)(&CCKM_Tx_data), CCKM_DATA_FRAME_LENGTH);
  }
}




