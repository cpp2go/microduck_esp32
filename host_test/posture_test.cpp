#include <cassert>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "../components/posture/posture.c"

int main() {
    float upright[3] = {0.f, 0.f, -1.f};
    posture_reading_t r = posture_classify(upright, nullptr);
    assert(r.posture == POSTURE_STANDING);

    float side[3] = {1.f, 0.f, 0.f};
    r = posture_classify(side, nullptr);
    assert(r.posture == POSTURE_LYING);

    float joints[15];
    std::memset(joints, 0, sizeof(joints));
    joints[2] = joints[3] = joints[12] = joints[13] = 1.2f;
    r = posture_classify(upright, joints);
    assert(r.posture == POSTURE_SEATED);

    assert(std::strcmp(posture_name(POSTURE_LYING), "lying") == 0);
    std::puts("posture_test OK");
    return 0;
}
