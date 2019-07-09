#include <algorithm>>
#include <string>

void
rec(int n, int* i)
{
    if (!n) return;

    *i = *i + 1;
    return rec(n - 1, i);
}

foo(2) { return 1 + foo(1); }
