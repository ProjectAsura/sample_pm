//-------------------------------------------------------------------------------------------------
// File : r3d_math.h
// Desc : Math 
// Copyright(c) Project Asura. All right reserved.
//-------------------------------------------------------------------------------------------------
#pragma once

#define NOMINMAX

//-------------------------------------------------------------------------------------------------
// Includes
//-------------------------------------------------------------------------------------------------
#include <cstdint>
#include <cmath>


//-------------------------------------------------------------------------------------------------
// Constant Values
//-------------------------------------------------------------------------------------------------
constexpr double   D_HIT_MAX   = 1e20;                                  //!< 交差判定上限値.
constexpr double   D_HIT_MIN   = 1e-6;                                  //!< 交差判定下限値.
constexpr double   D_PI        = 3.1415926535897932384626433832795;     //!< πです.
constexpr double   D_2PI       = 6.283185307179586476925286766559;      //!< 2πです.
constexpr double   D_1DIVPI    = 0.31830988618379067153776752674503;    //!< 1/πです.
constexpr double   D_1DIV2PI   = 0.15915494309189533576888376337251;    //!< 1/2πです.
constexpr double   D_PIDIV2    = 1.5707963267948966192313216916398;     //!< π/2です.
constexpr double   D_PIDIV3    = 1.0471975511965977461542144610932;     //!< π/3です.
constexpr double   D_PIDIV4    = 0.78539816339744830961566084581988;    //!< π/4です.
constexpr double   D_MAX       = 1.7976931348623158e+308;               //!< double型の最大値です.
constexpr double   D_MIN       = 2.2250738585072014e-308;               //!< double型の最小値です.

inline double max(double lhs, double rhs)
{ return (lhs > rhs) ? lhs : rhs; }

inline double min(double lhs, double rhs)
{ return (lhs < rhs) ? lhs : rhs; }

inline double clamp(double value, double mini, double maxi)
{ return max(mini, min(maxi, value)); }

inline double saturate(double value)
{ return clamp(value, 0, 1); }

///////////////////////////////////////////////////////////////////////////////////////////////////
// Vector2 structure
///////////////////////////////////////////////////////////////////////////////////////////////////
struct Vector2
{
    union
    {
        struct
        {
            double x;
            double y;
        };
        double a[2];
    };

    Vector2()
    { /* DO_NOTHING */ }

    Vector2(double nx, double ny)
    : x(nx), y(ny)
    { /* DO_NOTHING */ }

    inline Vector2 operator - () const
    { return Vector2(-x, -y); }

    inline Vector2 operator + (const Vector2& value) const
    { return Vector2(x + value.x, y + value.y); }

    inline Vector2 operator - (const Vector2& value) const
    { return Vector2(x - value.x, y - value.y); }

    inline Vector2 operator * (const Vector2& value) const
    { return Vector2(x * value.x, y * value.y); }

    inline Vector2 operator * (double value) const
    { return Vector2(x * value, y * value); }

    inline Vector2 operator / (double value) const
    { return Vector2(x / value, y / value); }

    inline Vector2& operator += (const Vector2& value)
    { x += value.x; y += value.y; return *this; }

    inline Vector2& operator -= (const Vector2& value)
    { x -= value.x; y -= value.y; return *this; }

    inline Vector2& operator *= (const Vector2& value)
    { x *= value.x; y *= value.y; return *this; }

    inline Vector2& operator *= (double value)
    { x *= value; y *= value; return *this; }

    inline Vector2& operator /= (double value)
    { x /= value; y /= value; return *this; }
};


///////////////////////////////////////////////////////////////////////////////////////////////////
// Vector3 structure
///////////////////////////////////////////////////////////////////////////////////////////////////
struct Vector3
{
    union
    {
        struct
        {
            double x;
            double y;
            double z;
        };
        double a[3];
    };

    Vector3()
    { /* DO_NOTHING */ }

    Vector3(double nx, double ny, double nz)
    : x(nx), y(ny), z(nz)
    { /* DO_NOTHING */ }

    inline Vector3 operator - () const
    { return Vector3(-x, -y, -z); }

    inline Vector3 operator + (const Vector3& value) const
    { return Vector3(x + value.x, y + value.y, z + value.z); }

    inline Vector3 operator - (const Vector3& value) const
    { return Vector3(x - value.x, y - value.y, z - value.z); }

    inline Vector3 operator * (const Vector3& value) const
    { return Vector3(x * value.x, y * value.y, z * value.z); }

    inline Vector3 operator * (double value) const
    { return Vector3(x * value, y * value, z * value); }

    inline Vector3 operator / (double value) const
    { return Vector3(x / value, y / value, z / value); }

    inline Vector3& operator += (const Vector3& value)
    { x += value.x; y += value.y; z += value.z; return *this; }

    inline Vector3& operator -= (const Vector3& value)
    { x -= value.x; y -= value.y; z -= value.z; return *this; }

    inline Vector3& operator *= (const Vector3& value)
    { x *= value.x; y *= value.y; z *= value.z; return *this; }

    inline Vector3& operator *= (double value)
    { x *= value; y *= value; z *= value; return *this; }

    inline Vector3& operator /= (double value)
    { x /= value; y /= value; z /= value; return *this; }
};


///////////////////////////////////////////////////////////////////////////////////////////////////
// Ray structure
///////////////////////////////////////////////////////////////////////////////////////////////////
struct Ray
{
    Vector3 pos;
    Vector3 dir;

    Ray()
    { /* DO_NOTHING */ }

    Ray(const Vector3& p, const Vector3& d)
    : pos(p), dir(d)
    { /* DO_NOTHING */ }
};


///////////////////////////////////////////////////////////////////////////////////////////////////
// BoundingBox structure
///////////////////////////////////////////////////////////////////////////////////////////////////
struct BoundingBox
{
    Vector3 mini;
    Vector3 maxi;

    inline void merge(const Vector3& value)
    {
        mini.x = min(value.x, mini.x);
        mini.y = min(value.y, mini.y);
        mini.z = min(value.z, mini.z);

        maxi.x = max(value.x, maxi.x);
        maxi.y = max(value.y, maxi.y);
        maxi.z = max(value.z, maxi.z);
    }

    inline void reset()
    {
        mini = Vector3( D_MAX,  D_MAX,  D_MAX);
        maxi = Vector3(-D_MAX, -D_MAX, -D_MAX);
    }

    inline Vector3 centroid() const
    {
        return Vector3(
            (maxi.x + mini.x) * 0.5,
            (maxi.y + mini.y) * 0.5,
            (maxi.z + mini.z) * 0.5);
    }

    inline int longest_axis() const
    {
        auto axis_x = maxi.x - mini.x;
        auto axis_y = maxi.y - mini.y;
        auto axis_z = maxi.z - mini.z;
        if (axis_x < axis_y)
        { return (axis_y < axis_z) ? 2 : 1; }
        else
        { return (axis_x < axis_z) ? 2 : 0; }
    }
};


///////////////////////////////////////////////////////////////////////////////////////////////////
// Random class
///////////////////////////////////////////////////////////////////////////////////////////////////
class Random
{
public:
    Random(uint32_t seed)
    { set_seed( 123456789 ); }

    inline void set_seed(uint32_t seed)
    {
        a = 123456789;
        b = 362436069;
        c = 521288629;
        d = ( seed <= 0 ) ? 88675123 : seed;
    }

    inline uint32_t get()
    {
        auto t = a ^ (a << 11);
        a = b;
        b = c;
        c = d;
        d = (d ^ (d >> 19)) ^ (t ^ (t >> 8));
        return d;
    }

    inline double get_as_double()
    { return static_cast<double>(get()) / 0xffffffffui32; }

    inline float get_as_float()
    { return static_cast<float>(get()) /  0xffffffffui32; }

private:
    uint32_t a;
    uint32_t b;
    uint32_t c;
    uint32_t d;
};

inline Vector2 operator * (double lhs, const Vector2& rhs)
{ return Vector2(lhs * rhs.x, lhs * rhs.y); }

inline Vector3 operator * (double lhs, const Vector3& rhs)
{ return Vector3(lhs * rhs.x, lhs * rhs.y, lhs * rhs.z); }

inline double length(const Vector2& value)
{ return sqrt(value.x * value.x + value.y * value.y); }

inline double length(const Vector3& value)
{ return sqrt(value.x * value.x + value.y * value.y + value.z * value.z); }

inline double dot(const Vector2& lhs, const Vector2& rhs)
{ return lhs.x * rhs.x + lhs.y * rhs.y; }

inline double dot(const Vector3& lhs, const Vector3& rhs)
{ return lhs.x * rhs.x + lhs.y * rhs.y + lhs.z * rhs.z; }

inline Vector2 normalize(const Vector2& value)
{ return value / length(value); }

inline Vector3 normalize(const Vector3& value)
{ return value / length(value); }

inline Vector2 reflect(const Vector2& i, const Vector2& n)
{ return i - n * 2.0 * dot(i, n); }

inline Vector3 reflect(const Vector3& i, const Vector3& n)
{ return i - n * 2.0 * dot(i, n); }

inline Vector3 cross(const Vector3& lhs, const Vector3& rhs)
{
    return Vector3(
        (lhs.y * rhs.z) - (lhs.z * rhs.y),
        (lhs.z * rhs.x) - (lhs.x * rhs.z),
        (lhs.x * rhs.y) - (lhs.y * rhs.x));
}


