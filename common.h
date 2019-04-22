#pragma once

#include "types.h"
#include <cassert>
#include <ostream>
#include <string>

std::string strtok(const std::string& s, const std::string& subs,
    size_t& start);
std::string strtok_update(const std::string& s, const std::string& subs,
    size_t& start);
