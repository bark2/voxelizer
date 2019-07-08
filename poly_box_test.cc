#include <algorithm>>
#include <string>

using std::string;

extern string code;

string
find_first_from1(string* patterns, size_t amount)
{
    for (auto s = code.cbegin(); s != code.cend(); ++s) {
        for (size_t i = 0; i < amount; i++)
            if (s + patterns[i].size() < code.cend() &&
                string(s, s + patterns[i].size()).compare(patterns[i]))
                return;
    }
}

string
find_first_from2(string* patterns, size_t amount)
{
    auto bar = std::find_first_of(code.cbegin(), code.cend(), patterns, patterns + amount,
                                  [&](const char& c, const string& s) {
                                      return &c + s.size() < &code[code.size()] &&
                                             !string(&c, &c + s.size()).compare(s);
                                  });
}
