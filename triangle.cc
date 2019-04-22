#include "types.h"

// vec3 Triangle::operator[](u32 i) const { return vertices[i]; }

vec3 Triangle::normal() const {
  vec3 result = { glm::cross(this->at(2) - this->at(1), this->at(1) - this->at(0)) };
    return result / length(result);
}
