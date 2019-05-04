#include "obj.h"
#include "types.h"
#include <algorithm>
#include <utility>

using Bitmask = std::array<bool, 8>;

struct Node {
    uint16_t far_and_child_id;
    Bitmask valid;
    Bitmask leaf;

    std::array<bool, 15> child_id();
    bool far();
};
