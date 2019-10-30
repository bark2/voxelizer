#include "vox_common.h"

namespace IVoxelizer {

std::string
strtok(const std::string& s, const std::string& subs, size_t& start)
{
    std::string result;
    size_t i = start;
    if (subs.find(s.at(start)) != std::string::npos) {
        while (i < s.length() && subs.find(s.at(i)) != std::string::npos) { i++; }
        start = i;
    }

    while (i < s.length() && subs.find(s.at(i)) == std::string::npos) i++;

    assert(start != i || start == s.length());
    if (start == i && i >= s.length())
        result = {};
    else
        result = std::string(s, start, i - start);
    return result;
}

std::string
strtok_update(const std::string& s, const std::string& subs, size_t& start)
{
    std::string result = strtok(s, subs, start);
    start += result.length();
    return result;
}

// todo: add a vector print function

}
