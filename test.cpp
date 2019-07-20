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

void
test_line_triangle_intersection_fast(bool verbose = false)
{
    Triangle tri;
    tri[0] = { 0.0f, 1.0f, 0.0f };
    tri[1] = { 1.0f, 0.0f, 0.0f };
    tri[2] = { -1.0f, 0.0f, 0.0f };

    array<vec3, 2> line;
    line[0] = { -1.5f, 0.5f, 0.0f };
    line[1] = { 1.5f, 0.5f, 0.0f };
    f64 line_length = (line[1] - line[0]).length();
    vec3 edges[3];
    for (int i = 0; i < 3; i++) edges[i] = tri[(i + 1) % 3] - tri[i];

    vec3 min, max;
    auto coll = line_triangle_intersection_mt(line, line_length, tri, edges);

    // for (auto coll = line_triangle_intersection_mt(line, line_length, tri); !coll.first;)
    std::cout << coll.first << " " << lerp(line, coll.second).to_string() << std::endl;
}

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
    bool verbose = get_cmd(argv, argv + argc, "--verbose");
    test_line_triangle_intersection_fast(verbose);

    return 0;
}
