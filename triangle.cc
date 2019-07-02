#include "types.h"

vec3
normal(const Triangle& t)
{
    vec3 result = cross(t[1] - t[0], t[2] - t[0]);
    return unit_vector(result);
}

