#ifndef GRAVITY_CONFIG_H
#define GRAVITY_CONFIG_H

#include <stdint.h>
#define ROBOT_DOF 6
#define MOTOR_MASS_COUNT 6
#define GRAVITY_ACCEL 9.8f
#define DEG2RAD(x) ((x) * 0.017453292519943295f)

typedef struct { float x; float y; float z; } Vec3;
typedef struct { float m[4][4]; } Mat4;

/* Craig 改进 DH：Rx(alpha) Tx(a) Rz(q+theta_offset) Tz(d)。 */
typedef struct
{
    float a;
    float alpha;
    float d;
    float theta_offset;
} DHParam;

/* 电机质量由提供者确认；质心相对于所属连杆坐标系。
 * link_index=-1 为底座，0 为经过 J0 后的坐标系 {1}，依此类推。
 */
typedef struct
{
    float mass;
    int link_index;
    Vec3 com_local;
} MotorMassParam;

/* q = position_sign * (encoder - encoder_zero)，参考姿态 q=0。
 * torque_sign 只在模型力矩转电机力矩时使用，必须独立核对。
 */
typedef struct
{
    float encoder_zero;
    float position_sign;
    float torque_sign;
} JointMap;

#include "robot_parameters.h"
#if ROBOT_PARAMETERS_READY != 1
#error "Fill robot_parameters.h from measurements before building firmware."
/* 仅供编译器继续检查类型，避免未填参数引起大量未声明报错。 */
extern const DHParam GRAVITY_DH[ROBOT_DOF];
extern const JointMap GRAVITY_JOINT_MAP[ROBOT_DOF];
extern const MotorMassParam GRAVITY_MOTOR_MASS[MOTOR_MASS_COUNT];
#else

static const DHParam GRAVITY_DH[ROBOT_DOF] =
{
    {J0_A, J0_ALPHA, J0_D, J0_THETA_OFFSET},
    {J1_A, J1_ALPHA, J1_D, J1_THETA_OFFSET},
    {J2_A, J2_ALPHA, J2_D, J2_THETA_OFFSET},
    {J3_A, J3_ALPHA, J3_D, J3_THETA_OFFSET},
    {J4_A, J4_ALPHA, J4_D, J4_THETA_OFFSET},
    {J5_A, J5_ALPHA, J5_D, J5_THETA_OFFSET},
};

static const JointMap GRAVITY_JOINT_MAP[ROBOT_DOF] =
{
    {J0_ENCODER_ZERO, J0_POSITION_SIGN, J0_TORQUE_SIGN},
    {J1_ENCODER_ZERO, J1_POSITION_SIGN, J1_TORQUE_SIGN},
    {J2_ENCODER_ZERO, J2_POSITION_SIGN, J2_TORQUE_SIGN},
    {J3_ENCODER_ZERO, J3_POSITION_SIGN, J3_TORQUE_SIGN},
    {J4_ENCODER_ZERO, J4_POSITION_SIGN, J4_TORQUE_SIGN},
    {J5_ENCODER_ZERO, J5_POSITION_SIGN, J5_TORQUE_SIGN},
};

static const MotorMassParam GRAVITY_MOTOR_MASS[MOTOR_MASS_COUNT] =
{
    {J0_MASS, J0_LINK, {J0_COM_X, J0_COM_Y, J0_COM_Z}},
    {J1_MASS, J1_LINK, {J1_COM_X, J1_COM_Y, J1_COM_Z}},
    {J2_MASS, J2_LINK, {J2_COM_X, J2_COM_Y, J2_COM_Z}},
    {J3_MASS, J3_LINK, {J3_COM_X, J3_COM_Y, J3_COM_Z}},
    {J4_MASS, J4_LINK, {J4_COM_X, J4_COM_Y, J4_COM_Z}},
    {J5_MASS, J5_LINK, {J5_COM_X, J5_COM_Y, J5_COM_Z}},
};
#endif

Mat4 GravityMath_Identity(void);
Mat4 GravityMath_DH(float a, float alpha, float d, float theta);
Mat4 GravityMath_Mat4Mul(Mat4 A, Mat4 B);
Vec3 GravityMath_TransformPoint(Mat4 T, Vec3 p);
Vec3 GravityMath_GetTranslation(Mat4 T);
Vec3 GravityMath_GetZAxis(Mat4 T);
Vec3 GravityMath_Sub(Vec3 a, Vec3 b);
Vec3 GravityMath_Scale(Vec3 v, float s);
Vec3 GravityMath_Cross(Vec3 a, Vec3 b);
float GravityMath_Dot(Vec3 a, Vec3 b);

int GravityComp_ModelValid(void);
void GravityComp_EncoderToModelQ(const float encoder[ROBOT_DOF], float q[ROBOT_DOF]);
/* 只算几何，输入为模型角。frames[i] 是坐标系 {i+1} 相对底座的位姿。 */
void GravityComp_ForwardKinematics(const float q[ROBOT_DOF], Mat4 frames[ROBOT_DOF]);
/* 不含增益/电机方向，可在零补偿增益时观察理论模型力矩。 */
int GravityComp_ModelTorque(const float q[ROBOT_DOF], float torque[ROBOT_DOF]);
void GravityComp_Calculate(const float encoder[ROBOT_DOF], float torque[ROBOT_DOF]);
#endif
