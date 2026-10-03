#include "gravity_config.h"

#include <math.h>

Mat4 GravityMath_Identity(void)
{
    Mat4 T = {0};

    T.m[0][0] = 1.0f;
    T.m[1][1] = 1.0f;
    T.m[2][2] = 1.0f;
    T.m[3][3] = 1.0f;

    return T;
}

Mat4 GravityMath_DH(
    float a,
    float alpha,
    float d,
    float theta)
{
    Mat4 T = {0};

    const float ct = cosf(theta);
    const float st = sinf(theta);

    const float ca = cosf(alpha);
    const float sa = sinf(alpha);

    T.m[0][0] = ct;
    T.m[0][1] = -st;
    T.m[0][2] = 0.0f;
    T.m[0][3] = a;

    T.m[1][0] = st * ca;
    T.m[1][1] = ct * ca;
    T.m[1][2] = -sa;
    T.m[1][3] = -sa * d;

    T.m[2][0] = st * sa;
    T.m[2][1] = ct * sa;
    T.m[2][2] = ca;
    T.m[2][3] = ca * d;

    T.m[3][0] = 0.0f;
    T.m[3][1] = 0.0f;
    T.m[3][2] = 0.0f;
    T.m[3][3] = 1.0f;

    return T;
}

Mat4 GravityMath_Mat4Mul(
    Mat4 A,
    Mat4 B)
{
    Mat4 C = {0};

    for (uint8_t r = 0; r < 4; r++)
    {
        for (uint8_t c = 0; c < 4; c++)
        {
            for (uint8_t k = 0; k < 4; k++)
            {
                C.m[r][c] +=
                    A.m[r][k]
                    *
                    B.m[k][c];
            }
        }
    }

    return C;
}

Vec3 GravityMath_TransformPoint(
    Mat4 T,
    Vec3 p)
{
    Vec3 out;

    out.x =
        T.m[0][0] * p.x
        + T.m[0][1] * p.y
        + T.m[0][2] * p.z
        + T.m[0][3];

    out.y =
        T.m[1][0] * p.x
        + T.m[1][1] * p.y
        + T.m[1][2] * p.z
        + T.m[1][3];

    out.z =
        T.m[2][0] * p.x
        + T.m[2][1] * p.y
        + T.m[2][2] * p.z
        + T.m[2][3];

    return out;
}

Vec3 GravityMath_GetTranslation(
    Mat4 T)
{
    Vec3 out =
    {
        .x = T.m[0][3],
        .y = T.m[1][3],
        .z = T.m[2][3]
    };

    return out;
}

Vec3 GravityMath_GetZAxis(
    Mat4 T)
{
    Vec3 out =
    {
        .x = T.m[0][2],
        .y = T.m[1][2],
        .z = T.m[2][2]
    };

    return out;
}

Vec3 GravityMath_Sub(
    Vec3 a,
    Vec3 b)
{
    Vec3 out =
    {
        .x = a.x - b.x,
        .y = a.y - b.y,
        .z = a.z - b.z
    };

    return out;
}

Vec3 GravityMath_Scale(
    Vec3 v,
    float s)
{
    Vec3 out =
    {
        .x = v.x * s,
        .y = v.y * s,
        .z = v.z * s
    };

    return out;
}

Vec3 GravityMath_Cross(
    Vec3 a,
    Vec3 b)
{
    Vec3 out =
    {
        .x = a.y * b.z - a.z * b.y,
        .y = a.z * b.x - a.x * b.z,
        .z = a.x * b.y - a.y * b.x
    };

    return out;
}

float GravityMath_Dot(
    Vec3 a,
    Vec3 b)
{
    return
        a.x * b.x
        + a.y * b.y
        + a.z * b.z;
}
