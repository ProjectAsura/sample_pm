//-------------------------------------------------------------------------------------------------
// File : r3d_camera.h
// Desc : Camera.
// Copyright(c) Project Asura. All right reserved.
//-------------------------------------------------------------------------------------------------
#pragma once

//-------------------------------------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------------------------------------
#include <r3d_math.h>


///////////////////////////////////////////////////////////////////////////////////////////////////
// Camera class
///////////////////////////////////////////////////////////////////////////////////////////////////
class Camera
{
public:
    Camera
    (
        const Vector3&  position,
        const Vector3&  dir,
        const Vector3&  upward,
        double          fov,
        double          aspect,
        double          znear
    )
    {
        pos         = position;
        axis_x      = normalize(cross(dir, upward)) * fov * aspect;
        axis_y      = normalize(cross(dir, axis_x)) * fov;
        axis_z      = dir;
        near_clip   = znear;
    }

    inline Ray emit(double x, double y) const
    {
        auto d = axis_x * x + axis_y * y + axis_z;
        auto p = pos + d * near_clip;
        return Ray(p, normalize(d));
    }

private:
    Vector3 pos;        //!< 位置座標です.
    Vector3 axis_x;     //!< 基底ベクトル(X軸)
    Vector3 axis_y;     //!< 基底ベクトル(Y軸)
    Vector3 axis_z;     //!< 基底ベクトル(Z軸).
    double  near_clip;  //!< ニア平面までの距離.

};
