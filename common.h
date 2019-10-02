#pragma once

#include "types.h"
#include <cassert>
#include <ostream>
#include <string>
#include <algorithm>

std::string strtok(const std::string& s, const std::string& subs,
    size_t& start);
std::string strtok_update(const std::string& s, const std::string& subs,
    size_t& start);

inline char**
get_cmd(char** begin, char** end, const std::string& option)
{
    char** itr = std::find(begin, end, option);
    if (itr != end) return itr;
    return nullptr;
}
