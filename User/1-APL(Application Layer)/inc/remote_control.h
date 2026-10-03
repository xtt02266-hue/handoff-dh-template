#ifndef __REMOTE_CONTROL_H
#define __REMOTE_CONTROL_H


#include "main.h"
#include "dm4310_ctrl.h"
#include "dm4310_drv.h"

//数据长度
#define FRAME_HEADER_LENGTH 5                               // 帧头数据长度
#define CMD_ID_LENGTH 2                                     // 命令码ID数据长度
//数据段长度
#define CC_DATA_LENGTH 30                                   // 自定义控制器数据段长度
#define CCKM_DATA_LENGTH 12                                 // 键鼠控制器数据段长度
//帧尾数据长度
#define FRAME_TAIL_LENGTH 2                                 // 帧尾数据长度
// 整个数据帧的长度
#define CC_DATA_FRAME_LENGTH (FRAME_HEADER_LENGTH + CMD_ID_LENGTH + CC_DATA_LENGTH + FRAME_TAIL_LENGTH) // 整个CC数据帧的长度
#define CCKM_DATA_FRAME_LENGTH (FRAME_HEADER_LENGTH + CMD_ID_LENGTH + CCKM_DATA_LENGTH + FRAME_TAIL_LENGTH) // 整个CCKM数据帧的长度
//命令码
#define CC_CONTROLLER_CMD_ID    0x0302                      //自定义控制器命令码
#define CCKM_CONTROLLER_CMD_ID    0x0306                    //自动控制器上的模拟键鼠遥控数据命令码
//CRC数据长度
#define CRC8_FORMER_LENGTH FRAME_HEADER_LENGTH              //CRC8数据和校验的长度
#define CC_CRC16_FORMER_LENGTH CC_DATA_FRAME_LENGTH         //自定义控制器CRC16数据和校验的长度
#define CCKM_CRC16_FORMER_LENGTH CCKM_DATA_FRAME_LENGTH     //键鼠控制器CRC16数据和校验的长度


//控制模式
enum controller_mode_t
{
    CC_mode=1,      //自定义控制器(customized controller)
    CCKM_mode       //键鼠控制器(the key and mouse of constomized controller)
};


/* 键鼠遥控数据，固定 30Hz 频率发送 0x0304 */
typedef  struct
{
		uint16_t key_value; 
	 uint16_t x_position:12; 
	 uint16_t mouse_left:4; 
	 uint16_t y_position:12;
	 uint16_t mouse_right:4; 
	 uint16_t reserved; 
}__attribute__((packed)) cutom_CCKM_data_t;



//custom_robot_data数据传输模版
typedef  struct
{
     struct 
    {
       uint8_t SOF;                  //数据帧起始字节，固定值为0xA5
       uint16_t data_length;    //数据帧中data
       uint8_t seq;                  //包序号
       uint8_t CRC8;                 //帧头CRC8校验
    }__attribute__((packed)) frame_header;                   //帧头
    uint16_t cmd_id;                 //命令码ID
    uint8_t data[30];                //数据内容
    uint16_t frame_tail;             //CRC16整包验证
}__attribute__((packed)) custom_robot_data_t;

//定义自定义控制器与机器人交互数据
extern custom_robot_data_t CC_Tx_data;


//自定义控制器/键鼠交互发送数据函数
void image_transimission_link(UART_HandleTypeDef *HUART,enum controller_mode_t controller_mode,const uint8_t * data);


#endif