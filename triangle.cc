#include "types.h"

vec3
Triangle::normal() const
{
    vec3 v1 = this->at(0);
    vec3 v2 = this->at(1);
    vec3 v3 = this->at(2);
    vec3 result = glm::cross(v2 - v1, v3 - v1);
    return glm::normalize(result);
}
