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

#define aprintf(x, ...) if ((x)) { printf(__VA_ARGS__); }

inline std::pair<bool, f32>
edge_edge_collision(vec2 a, vec2 b, vec2 c, vec2 d)
{
    f32 denominator = ((b.x - a.x) * (d.y - c.y)) - ((b.y - a.y) * (d.x - c.x));
    f32 numerator1 = ((a.y - c.y) * (d.x - c.x)) - ((a.x - c.x) * (d.y - c.y));
    f32 numerator2 = ((a.y - c.y) * (b.x - a.x)) - ((a.x - c.x) * (b.y - a.y));

    if (denominator == 0) return { numerator1 == 0 && numerator2 == 0, std::numeric_limits<f32>::max() };

    f32 r = numerator1 / denominator;
    f32 s = numerator2 / denominator;

    if ((r >= 0 && r <= 1) && (s >= 0 && s <= 1)) return { true, r };
    return { false, 0.0f };
}

void
test_edge_edge_collision(bool verbose)
{
  vec2 e1[] = { { -1.0f, 0.0f }, {1.0f, 0.0f} };
  vec2 e2[] = { { 0.0f, 1.0f }, { 0.0f, -1.0f } };
  auto intersection = edge_edge_collision(e1[0], e1[1], e2[0], e2[1]);
  assert(intersection.first && intersection.second == 0.5);
  aprintf(verbose, "%f\n", intersection.second);

  vec2 e3[] = { { -1.0f, -1.0f }, { 1.0f, 1.0f } };
  vec2 e4[] = { { -1.0f, 1.0f }, { 1.0f, -1.0f } };
  intersection = edge_edge_collision(e3[0], e3[1], e4[0], e4[1]);
  assert(intersection.first && intersection.second == 0.5);
  aprintf(verbose, "%f\n", intersection.second);

  vec2 e5[] = { { -1.0f, -1.0f }, { -1.0f, 1.0f } };
  vec2 e6[] = { { 1.0f, -1.0f }, { 1.0f, 1.0f } };
  intersection = edge_edge_collision(e5[0], e5[1], e6[0], e6[1]);
  assert(!intersection.first);
  aprintf(verbose, "%f\n", intersection.first? intersection.second : -1.0f);

  vec2 e7[] = { { 0.0f, 0.0f }, { 2.0f, 2.0f } };
  vec2 e8[] = { { 1.0f, 0.0f }, { 0.0f, 1.0f } };
  intersection = edge_edge_collision(e7[0], e7[1], e8[0], e8[1]);
  assert(intersection.first);
  aprintf(verbose, "%f\n", intersection.first? intersection.second : -1.0f);
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
    test_edge_edge_collision(verbose);

    return 0;
}
