// Host test: fall predictor + safety clamp vs pollen-robotics/microduck duck-control.
#include <cassert>
#include <cmath>
#include <cstdio>

#include "../components/safety/safety.c"

static void tipping(float tilt, float pitch_rate, float gyro[3], float grav[3]) {
    gyro[0] = 0.f;
    gyro[1] = pitch_rate;
    gyro[2] = 0.f;
    grav[0] = std::sin(tilt);
    grav[1] = 0.f;
    grav[2] = -std::cos(tilt);
}

int main() {
    fall_predictor_t p;
    fall_predictor_init(&p, nullptr);
    float gyro[3], grav[3];
    const float dt = 0.02f;

    // Analytic rate: pitch 1 rad/s at 30° → ġz = sin(30°) = 0.5
    tipping(static_cast<float>(M_PI / 6.0), 1.f, gyro, grav);
    const float rate = fall_gravity_z_rate(gyro, grav);
    assert(std::fabs(rate - 0.5f) < 1e-5f);

    // Arm by standing upright
    for (int i = 0; i < 15; ++i) {
        tipping(0.f, 0.f, gyro, grav);
        assert(!fall_predictor_observe(&p, gyro, grav, dt));
    }

    // Recovering lean must never fire
    for (int i = 0; i < 50; ++i) {
        tipping(0.9f, -4.f, gyro, grav);
        assert(!fall_predictor_observe(&p, gyro, grav, dt));
    }

    // Re-arm
    fall_predictor_reset(&p);
    for (int i = 0; i < 15; ++i) {
        tipping(0.f, 0.f, gyro, grav);
        assert(!fall_predictor_observe(&p, gyro, grav, dt));
    }

    // Footfall on upright — tilt gate refuses
    for (int i = 0; i < 10; ++i) {
        tipping(0.14f, 3.f, gyro, grav);
        assert(!fall_predictor_observe(&p, gyro, grav, dt));
    }

    float targets[3] = {100.f, NAN, -100.f};
    const int lim = safety_clamp_targets(targets, 3);
    assert(lim == 3);
    assert(targets[0] <= SAFETY_ACTUATOR_MAX + 1e-5f);
    assert(targets[1] == 0.f);
    assert(targets[2] >= SAFETY_ACTUATOR_MIN - 1e-5f);

    safety_state_t s;
    safety_init(&s, nullptr);
    for (int i = 0; i < 15; ++i) {
        (void)safety_observe_gravity(&s, 0.f, dt); // past -0.5
    }
    assert(safety_fallen(&s));

    std::puts("safety_fall_test OK");
    return 0;
}
