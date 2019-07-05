#include "math.h"
#include "obj.h"
#include "types.h"
#include <algorithm>
#include <cassert>
#include <csignal>
#include <cstdio>
#include <cstring>
#include <limits>
#include <tuple>
#include <utility>

#define aprintf(x, ...)                                                                                 \
    if ((x)) { printf(__VA_ARGS__); }

// bool
// test_triangle_box_collision(bool verbose, Triangle t, array<vec3, 2> box)
// {
//     bool result = false;
//     f32 min_coll = std::numeric_limits<f32>::max(), max_coll = -std::numeric_limits<f32>::max();
//     auto option = Intersection_Option::RETURN_INTERSECTOIN_POINT;
//     f32 delta = 0.1f;

//     // should be checking only for squares that are in the triangles's aabb
//     while (!aabb_collision(box, triangle_aabb(t))) {
//         if (t[0].x >= 1.0f) break;
//         for (auto& v : t) v.x += delta;
//     }

//     for (; aabb_collision(box, triangle_aabb(t));) {
//         result = true;
//         auto res = triangle_aabb_collision(t, box, triangle_square_fconservative_collision, option);
//         min_coll = std::min(min_coll, res.second);
//         max_coll = std::max(max_coll, res.second);
//         printf("min triangle aabb = %f, collision t = %f\n", min_coll, res.second);

//         for (auto& v : t) v.x += delta;
//     }

//     printf("%f %f\n", min_coll, max_coll);
//     // if (result) assert(min_coll == 0.0f && max_coll == 1.0f);

//     return result;
// }

char**
get_cmd(char** begin, char** end, const std::string& option)
{
    char** itr = std::find(begin, end, option);
    if (itr != end) return itr;
    return nullptr;
}

int
main(int argc, char* argv[])
{
    return 0;
}
