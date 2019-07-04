#pragma once

#include "vec2.h"
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdlib.h>

using f32 = float;

struct vec3 {
    union {
        f32 v[3];
        struct {
            f32 x, y, z;
        };
    };

    typedef std::size_t size_type;
    typedef f32* iterator_type;

    iterator_type
    begin()
    {
        return v;
    }

    iterator_type
    end()
    {
        return v + size();
    }

    vec3() {};
    vec3(f32 v0, f32 v1, f32 v2)
    {
        v[0] = v0;
        v[1] = v1;
        v[2] = v2;
    }

    vec3(f32 a)
    {
        v[0] = a;
        v[1] = a;
        v[2] = a;
    }

    inline operator vec2() const { return { x, y }; };
    inline size_type
    size() const
    {
        return 3;
    };

    inline const vec3&
    operator+() const
    {
        return *this;
    }
    inline vec3
    operator-() const
    {
        return vec3(-v[0], -v[1], -v[2]);
    }
    inline f32 operator[](int i) const { return v[i]; }
    inline f32& operator[](int i) { return v[i]; }

    inline vec3& operator*=(const f32 t);

    inline vec3&
    operator+=(f32 t)
    {
        for (auto& i : v) i += t;
        return *this;
    }

    inline vec3&
    operator-=(f32 t)
    {
        for (auto& i : v) i -= t;
        return *this;
    }
    inline vec3&
    operator/=(f32 t)
    {
        assert(t != 0.0f);
        for (auto& i : v) i /= t;
        return *this;
    }

    inline f32
    length() const
    {
        return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }
    inline f32
    squared_length() const
    {
        return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    }
    inline void normalize();

    inline std::string
    to_string() const
    {
        using std::to_string;
        return "( " + to_string(x) + ", " + to_string(y) + ", " + to_string(z) + " )";
    }
};

inline vec3
operator+(const vec3& v1, f32 a)
{
    return vec3(v1.v[0] + a, v1.v[1] + a, v1.v[2] + a);
}

inline vec3
operator-(const vec3& v1, f32 a)
{
    return vec3(v1.v[0] - a, v1.v[1] - a, v1.v[2] - a);
}

inline vec3 operator*(const vec3& v1, f32 a) { return vec3(v1.v[0] * a, v1.v[1] * a, v1.v[2] * a); }

inline vec3
operator/(const vec3& v1, f32 a)
{
    return vec3(v1.v[0] / a, v1.v[1] / a, v1.v[2] / a);
}

inline vec3
operator+(const vec3& v1, const vec3& v2)
{
    return vec3(v1.v[0] + v2.v[0], v1.v[1] + v2.v[1], v1.v[2] + v2.v[2]);
}

inline vec3
operator-(const vec3& v1, const vec3& v2)
{
    return vec3(v1.v[0] - v2.v[0], v1.v[1] - v2.v[1], v1.v[2] - v2.v[2]);
}

inline vec3 operator*(const vec3& v1, const vec3& v2)
{
    return vec3(v1.v[0] * v2.v[0], v1.v[1] * v2.v[1], v1.v[2] * v2.v[2]);
}

inline vec3
operator/(const vec3& v1, const vec3& v2)
{
    return vec3(v1.v[0] / v2.v[0], v1.v[1] / v2.v[1], v1.v[2] / v2.v[2]);
}

inline f32
dot(const vec3& v1, const vec3& v2)
{
    return v1.x * v2.x + v1.y * v2.y + v1.z * v2.z;
}

inline vec3
cross(const vec3& v1, const vec3& v2)
{
    return vec3(v1.y * v2.z - v1.z * v2.y, -(v1.x * v2.z - v1.z * v2.x), v1.x * v2.y - v1.y * v2.x);
}

inline vec3
unit_vector(vec3 v)
{
    return v / v.length();
}

inline vec3 operator*(const f32 a, const vec3& v) { return vec3(a * v.v[0], a * v.v[1], a * v.v[2]); }
