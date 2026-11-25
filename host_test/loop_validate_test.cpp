#include <cassert>
#include <cstdio>

#include "../components/control/loop_validate.c"

int main() {
    loop_validate_t s;
    loop_validate_reset(&s, 20000);
    for (int i = 0; i < 50; ++i) {
        loop_validate_record(&s, 12000); // pass duration under period
    }
    assert(loop_validate_ok(&s, 0.25, 0.05));
    for (int i = 0; i < 10; ++i) {
        loop_validate_record(&s, 25000);
    }
    // 10/60 ≈ 0.167 > 0.05
    assert(!loop_validate_ok(&s, 0.25, 0.05));
    std::puts("loop_validate_test OK");
    return 0;
}
