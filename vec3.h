#pragma once

#include "vec2.h"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <stdlib.h>

namespace IVoxelizer {

using f64 = double;

struct vec3 {
    union {
        f64 v[3];
        struct {
            f64 x, y, z;
        };
    };

    typedef std::size_t size_type;
    typedef f64* iterator_type;

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

    vec3() = default;
    constexpr vec3(f64 v0, f64 v1, f64 v2) : x(v0), y(v1), z(v2) {};
    constexpr explicit vec3(f64 a) : x(a), y(a), z(a) {};

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
    inline f64 operator[](int i) const { return v[i]; }
    inline f64& operator[](int i) { return v[i]; }

    inline vec3&
    operator*=(const f64 t)
    {
        for (auto& i : v) i *= t;
        return *this;
    }

    inline vec3&
    operator+=(f64 t)
    {
        for (auto& i : v) i += t;
        return *this;
    }

    inline vec3&
    operator-=(f64 t)
    {
        for (auto& i : v) i -= t;
        return *this;
    }

    inline vec3&
    operator/=(f64 t)
    {
        assert(t != 0.0f);
        for (auto& i : v) i /= t;
        return *this;
    }

    inline vec3&
    operator-=(const vec3& l)
    {
        for (int i = 0; i < 3; i++) v[i] -= l[i];
        return *this;
    }

    inline vec3&
    operator+=(const vec3& l)
    {
        for (int i = 0; i < 3; i++) v[i] += l[i];
        return *this;
    }

    inline f64
    length() const
    {
        return sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    }
    inline f64
    squared_length() const
    {
        return v[0] * v[0] + v[1] * v[1] + v[2] * v[2];
    }
    inline void normalize();

    inline std::string
    to_string() const
    {
        using std::to_string;
        return "(" + to_string(x) + ", " + to_string(y) + ", " + to_string(z) + ")";
    }
};

inline bool
operator==(const vec3& r, const vec3& l)
{
    return r[0] == l[0] && r[1] == l[1] && r[2] == l[2];
}

inline bool
operator!=(const vec3& r, const vec3& l)
{
    return !(r == l);
}

inline vec3
operator+(const vec3& v1, f64 a)
{
    return vec3(v1.v[0] + a, v1.v[1] + a, v1.v[2] + a);
}

inline vec3
operator-(const vec3& v1, f64 a)
{
    return vec3(v1.v[0] - a, v1.v[1] - a, v1.v[2] - a);
}

inline vec3 operator*(const vec3& v1, f64 a) { return vec3(v1.v[0] * a, v1.v[1] * a, v1.v[2] * a); }

inline vec3
operator/(const vec3& v1, f64 a)
{
    return vec3(v1.v[0] / a, v1.v[1] / a, v1.v[2] / a);
}

inline vec3
operator/(f64 a, const vec3& v1)
{
    return vec3(a / v1.v[0], a / v1.v[1], a / v1.v[2]);
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

// inline vec3 operator*(const vec3& v1, const vec3& v2)
// {
    // return vec3(v1.v[0] * v2.v[0], v1.v[1] * v2.v[1], v1.v[2] * v2.v[2]);
// }

inline vec3
operator/(const vec3& v1, const vec3& v2)
{
    // assert(v2[0] && v2[1] && v2[2]); //FIXME
    return vec3(v1.v[0] / v2.v[0], v1.v[1] / v2.v[1], v1.v[2] / v2.v[2]);
}

inline f64
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
unit(vec3 v)
{
    return v / v.length();
}

inline vec3 operator*(const f64 a, const vec3& v) { return vec3(a * v.v[0], a * v.v[1], a * v.v[2]); }

}
