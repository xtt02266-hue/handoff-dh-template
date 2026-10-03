#ifndef ROBOT_PARAMETERS_H
#define ROBOT_PARAMETERS_H

/* 在此填入实测参数。长度 m，角度 rad，质量 kg。J0 为第一个电机。 */
/* 全部填完并核对后改为 1；不要用零值冒充未测量参数。 */
#define ROBOT_PARAMETERS_READY 0
#define GRAVITY_COMP_GAIN 0.0f

/* J0：LINK=-1 为底座，0~5 为所随动的连杆。MASS 必须实测/确认。 */
#define J0_A FILL_J0_A                       // DH连杆长度，沿X轴，单位m
#define J0_ALPHA FILL_J0_ALPHA               // DH连杆扭转角，绕X轴，单位rad
#define J0_D FILL_J0_D                       // DH连杆偏距，沿Z轴，单位m
#define J0_THETA_OFFSET FILL_J0_THETA_OFFSET // DH固定角偏置，绕Z轴，单位rad
#define J0_ENCODER_ZERO FILL_J0_ENCODER_ZERO // 参考姿态的编码器读数，单位rad
#define J0_POSITION_SIGN FILL_J0_POSITION_SIGN // 编码器到模型的角度方向，填+1或-1
#define J0_TORQUE_SIGN FILL_J0_TORQUE_SIGN   // 模型到电机的力矩方向，填+1或-1
#define J0_MASS FILL_J0_MASS                 // 第一个电机的质量，单位kg
#define J0_LINK FILL_J0_LINK                 // 质量所属刚体：-1底座，0~5关节后的连杆
#define J0_COM_X FILL_J0_COM_X               // 质心在所属连杆坐标系的X坐标，单位m
#define J0_COM_Y FILL_J0_COM_Y               // 质心在所属连杆坐标系的Y坐标，单位m
#define J0_COM_Z FILL_J0_COM_Z               // 质心在所属连杆坐标系的Z坐标，单位m

/* J1：LINK=-1 为底座，0~5 为所随动的连杆。MASS 必须实测/确认。 */
#define J1_A FILL_J1_A
#define J1_ALPHA FILL_J1_ALPHA
#define J1_D FILL_J1_D
#define J1_THETA_OFFSET FILL_J1_THETA_OFFSET
#define J1_ENCODER_ZERO FILL_J1_ENCODER_ZERO
#define J1_POSITION_SIGN FILL_J1_POSITION_SIGN
#define J1_TORQUE_SIGN FILL_J1_TORQUE_SIGN
#define J1_MASS FILL_J1_MASS
#define J1_LINK FILL_J1_LINK
#define J1_COM_X FILL_J1_COM_X
#define J1_COM_Y FILL_J1_COM_Y
#define J1_COM_Z FILL_J1_COM_Z

/* J2：LINK=-1 为底座，0~5 为所随动的连杆。MASS 必须实测/确认。 */
#define J2_A FILL_J2_A
#define J2_ALPHA FILL_J2_ALPHA
#define J2_D FILL_J2_D
#define J2_THETA_OFFSET FILL_J2_THETA_OFFSET
#define J2_ENCODER_ZERO FILL_J2_ENCODER_ZERO
#define J2_POSITION_SIGN FILL_J2_POSITION_SIGN
#define J2_TORQUE_SIGN FILL_J2_TORQUE_SIGN
#define J2_MASS FILL_J2_MASS
#define J2_LINK FILL_J2_LINK
#define J2_COM_X FILL_J2_COM_X
#define J2_COM_Y FILL_J2_COM_Y
#define J2_COM_Z FILL_J2_COM_Z

/* J3：LINK=-1 为底座，0~5 为所随动的连杆。MASS 必须实测/确认。 */
#define J3_A FILL_J3_A
#define J3_ALPHA FILL_J3_ALPHA
#define J3_D FILL_J3_D
#define J3_THETA_OFFSET FILL_J3_THETA_OFFSET
#define J3_ENCODER_ZERO FILL_J3_ENCODER_ZERO
#define J3_POSITION_SIGN FILL_J3_POSITION_SIGN
#define J3_TORQUE_SIGN FILL_J3_TORQUE_SIGN
#define J3_MASS FILL_J3_MASS
#define J3_LINK FILL_J3_LINK
#define J3_COM_X FILL_J3_COM_X
#define J3_COM_Y FILL_J3_COM_Y
#define J3_COM_Z FILL_J3_COM_Z

/* J4：LINK=-1 为底座，0~5 为所随动的连杆。MASS 必须实测/确认。 */
#define J4_A FILL_J4_A
#define J4_ALPHA FILL_J4_ALPHA
#define J4_D FILL_J4_D
#define J4_THETA_OFFSET FILL_J4_THETA_OFFSET
#define J4_ENCODER_ZERO FILL_J4_ENCODER_ZERO
#define J4_POSITION_SIGN FILL_J4_POSITION_SIGN
#define J4_TORQUE_SIGN FILL_J4_TORQUE_SIGN
#define J4_MASS FILL_J4_MASS
#define J4_LINK FILL_J4_LINK
#define J4_COM_X FILL_J4_COM_X
#define J4_COM_Y FILL_J4_COM_Y
#define J4_COM_Z FILL_J4_COM_Z

/* J5：LINK=-1 为底座，0~5 为所随动的连杆。MASS 必须实测/确认。 */
#define J5_A FILL_J5_A
#define J5_ALPHA FILL_J5_ALPHA
#define J5_D FILL_J5_D
#define J5_THETA_OFFSET FILL_J5_THETA_OFFSET
#define J5_ENCODER_ZERO FILL_J5_ENCODER_ZERO
#define J5_POSITION_SIGN FILL_J5_POSITION_SIGN
#define J5_TORQUE_SIGN FILL_J5_TORQUE_SIGN
#define J5_MASS FILL_J5_MASS
#define J5_LINK FILL_J5_LINK
#define J5_COM_X FILL_J5_COM_X
#define J5_COM_Y FILL_J5_COM_Y
#define J5_COM_Z FILL_J5_COM_Z

#endif
