#include <algorithm>
#include <string>

using std::string;

string code = {};

string
GetNextWord(size_t amount, string patterns[amount])
{
  for (char* first = &code[0]; first != &code[code.size()]; ++first) {
      for (size_t i = 0; i < amount; i++)
          if (first. compare(patterns[i]))
              ;
    }

    return last;
}
