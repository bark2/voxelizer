#pragma once

#include <cmath>
#include <iostream>
#include <stdlib.h>

using f32 = float;

struct vec2 {
    typedef std::size_t size_type;

    union {
        f32 v[2];
        struct {
            f32 x, y;
        };
    };

    vec2() {};
    vec2(f32 x, f32 y)
    {
        v[0] = x;
        v[1] = y;
    }

    inline size_type
    size() const
    {
        return 2;
    };

    inline const vec2&
    operator+() const
    {
        return *this;
    }
    inline vec2
    operator-() const
    {
        return vec2(-v[0], -v[1]);
    }
    inline f32 operator[](int i) const { return v[i]; }
    inline f32& operator[](int i) { return v[i]; }

    inline vec2& operator+=(const vec2& v2);
    inline vec2& operator-=(const vec2& v2);
    inline vec2& operator*=(const vec2& v2);
    inline vec2& operator/=(const vec2& v2);
    inline vec2& operator*=(const f32 t);
    inline vec2& operator/=(const f32 t);

    inline f32
    length() const
    {
        return sqrt(v[0] * v[0] + v[1] * v[1]);
    }
    inline f32
    squared_length() const
    {
        return v[0] * v[0] + v[1] * v[1];
    }
    inline void normalize();

    inline std::string
    to_string() const
    {
        using std::to_string;
        return "( " + to_string(x) + ", " + to_string(y) + " )";
    }
};

inline bool
operator==(const vec2& v1, const vec2& v2)
{
    return (v1.x == v2.x && v1.y == v2.y);
}

inline vec2
operator+(const vec2& v1, f32 a)
{
    return vec2(v1.v[0] + a, v1.v[1] + a);
}

inline vec2
operator+(const vec2& v1, const vec2& v2)
{
    return vec2(v1.v[0] + v2.v[0], v1.v[1] + v2.v[1]);
}

inline vec2
operator-(const vec2& v1, const vec2& v2)
{
    return vec2(v1.v[0] - v2.v[0], v1.v[1] - v2.v[1]);
}

inline vec2 operator*(const vec2& v1, const vec2& v2)
{
    return vec2(v1.v[0] * v2.v[0], v1.v[1] * v2.v[1]);
}

inline vec2
operator/(const vec2& v1, const vec2& v2)
{
    return vec2(v1.v[0] / v2.v[0], v1.v[1] / v2.v[1]);
}

inline vec2
operator/(const vec2& v, const f32 a)
{
    return vec2(v[0] / a, v[1] / a);
}

inline f32
dot(const vec2& v1, const vec2& v2)
{
    return v1.x * v2.x + v1.y * v2.y;
}

inline vec2
unit_vector(vec2 v)
{
    return v / v.length();
}

inline vec2 operator*(const f32 a, const vec2& v) { return vec2(a * v.v[0], a * v.v[1]); }
