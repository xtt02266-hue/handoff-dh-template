#include "gravity_config.h"
#include <math.h>

/* 参数有效性不能证明几何正确，只拦住非法数值和索引。 */
int GravityComp_ModelValid(void)
{
    if (!isfinite(GRAVITY_COMP_GAIN) || GRAVITY_COMP_GAIN < 0.0f ||
        GRAVITY_COMP_GAIN > 1.0f)
    {
        return 0;
    }
    for (int i = 0; i < ROBOT_DOF; i++)
    {
        if (!isfinite(GRAVITY_DH[i].a) || !isfinite(GRAVITY_DH[i].alpha) ||
            !isfinite(GRAVITY_DH[i].d) || !isfinite(GRAVITY_DH[i].theta_offset))
        {
            return 0;
        }
        if (!isfinite(GRAVITY_JOINT_MAP[i].encoder_zero))
        {
            return 0;
        }
        if (GRAVITY_JOINT_MAP[i].position_sign != 1.0f &&
            GRAVITY_JOINT_MAP[i].position_sign != -1.0f)
        {
            return 0;
        }
        if (GRAVITY_JOINT_MAP[i].torque_sign != 1.0f &&
            GRAVITY_JOINT_MAP[i].torque_sign != -1.0f)
        {
            return 0;
        }
    }
    for (int i = 0; i < MOTOR_MASS_COUNT; i++)
    {
        if (GRAVITY_MOTOR_MASS[i].link_index < -1 ||
            GRAVITY_MOTOR_MASS[i].link_index >= ROBOT_DOF)
        {
            return 0;
        }
        if (!isfinite(GRAVITY_MOTOR_MASS[i].mass) ||
            GRAVITY_MOTOR_MASS[i].mass < 0.0f ||
            !isfinite(GRAVITY_MOTOR_MASS[i].com_local.x) ||
            !isfinite(GRAVITY_MOTOR_MASS[i].com_local.y) ||
            !isfinite(GRAVITY_MOTOR_MASS[i].com_local.z))
        {
            return 0;
        }
    }
    return 1;
}

void GravityComp_EncoderToModelQ(const float encoder[ROBOT_DOF], float q[ROBOT_DOF])
{
    for (int i = 0; i < ROBOT_DOF; i++)
    {
        q[i] = GRAVITY_JOINT_MAP[i].position_sign *
            (encoder[i] - GRAVITY_JOINT_MAP[i].encoder_zero);
    }
}

void GravityComp_ForwardKinematics(const float q[ROBOT_DOF], Mat4 frames[ROBOT_DOF])
{
    Mat4 current = GravityMath_Identity();
    for (int i = 0; i < ROBOT_DOF; i++)
    {
        float theta = q[i] + GRAVITY_DH[i].theta_offset;
        Mat4 local = GravityMath_DH(GRAVITY_DH[i].a, GRAVITY_DH[i].alpha,
                                    GRAVITY_DH[i].d, theta);
        current = GravityMath_Mat4Mul(current, local);
        frames[i] = current;
    }
}

int GravityComp_ModelTorque(const float q[ROBOT_DOF], float torque[ROBOT_DOF])
{
    for (int i = 0; i < ROBOT_DOF; i++)
    {
        torque[i] = 0.0f;
    }
    if (GravityComp_ModelValid() == 0)
    {
        return 0;
    }
    for (int i = 0; i < ROBOT_DOF; i++)
    {
        if (!isfinite(q[i]))
        {
            return 0;
        }
    }

    Mat4 frames[ROBOT_DOF];
    GravityComp_ForwardKinematics(q, frames);
    Vec3 gravity = {0.0f, 0.0f, -GRAVITY_ACCEL};

    for (int point = 0; point < MOTOR_MASS_COUNT; point++)
    {
        int link = GRAVITY_MOTOR_MASS[point].link_index;
        if (link == -1)
        {
            continue;
        }

        Vec3 center = GravityMath_TransformPoint(frames[link],
                                GRAVITY_MOTOR_MASS[point].com_local);
        Vec3 force = GravityMath_Scale(gravity, GRAVITY_MOTOR_MASS[point].mass);

        /* 该质量点由 J0 到所属连杆的关节承担；不会影响更下游的关节。 */
        for (int joint = 0; joint <= link; joint++)
        {
            Vec3 origin = GravityMath_GetTranslation(frames[joint]);
            Vec3 axis = GravityMath_GetZAxis(frames[joint]);
            Vec3 lever = GravityMath_Sub(center, origin);
            Vec3 moment = GravityMath_Cross(lever, force);
            torque[joint] -= GravityMath_Dot(axis, moment);
        }
    }
    return 1;
}

void GravityComp_Calculate(const float encoder[ROBOT_DOF], float torque[ROBOT_DOF])
{
    float q[ROBOT_DOF];
    GravityComp_EncoderToModelQ(encoder, q);
    if (GravityComp_ModelTorque(q, torque) == 0)
    {
        /* gravity.c 的计算出口会识别失败并锁存故障，不把失败冒充零补偿。 */
        for (int i = 0; i < ROBOT_DOF; i++)
        {
            torque[i] = NAN;
        }
        return;
    }

    for (int i = 0; i < ROBOT_DOF; i++)
    {
        torque[i] = torque[i] * GRAVITY_COMP_GAIN * GRAVITY_JOINT_MAP[i].torque_sign;
    }
}
