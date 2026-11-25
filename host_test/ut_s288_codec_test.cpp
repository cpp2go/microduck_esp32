// Host-side Unitree S288 codec checks (no hardware).
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>

extern "C" {
#include "ut_s288_codec.h"
}

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main()
{
    int fails = 0;

    uint8_t pkt[UT_S288_CMD_LEN];
    ut_s288_build_control(0, UT_S288_MODE_FOC, true, 0.0f, 0.0f, 0.0f, 10.0f, 0.5f, pkt);
    if (pkt[0] != 0xFE || pkt[1] != 0xEE) {
        std::printf("FAIL header\n");
        ++fails;
    }
    // mode_byte: id=0, mode=1, timeout=1 → 0x90
    if (pkt[2] != 0x90 || pkt[3] != 0x00) {
        std::printf("FAIL mode_byte %02x\n", pkt[2]);
        ++fails;
    }
    const uint32_t crc = (uint32_t)pkt[16] | ((uint32_t)pkt[17] << 8) | ((uint32_t)pkt[18] << 16) |
                         ((uint32_t)pkt[19] << 24);
    if (crc != ut_s288_crc32(pkt, 16)) {
        std::printf("FAIL embedded CRC\n");
        ++fails;
    }
    // Golden from official table algorithm (pos=0, Kp=10, Kd=0.5, id0 FOC timeout)
    if (crc != 0xe6990ec6u) {
        std::printf("FAIL golden CRC got %08x\n", crc);
        ++fails;
    }

    // Round-trip: build feedback-like buffer by parsing requires real fbk;
    // check position encode magnitude for small angle.
    ut_s288_build_control(1, UT_S288_MODE_FOC, false, 0.0f, 0.0f, 0.1f, 1.0f, 0.1f, pkt);
    if ((pkt[2] & 0x0F) != 1 || ((pkt[2] >> 4) & 7) != 1 || (pkt[2] & 0x80)) {
        std::printf("FAIL id/mode/timeout packing\n");
        ++fails;
    }
    int32_t pos_raw = (int32_t)((uint32_t)pkt[8] | ((uint32_t)pkt[9] << 8) | ((uint32_t)pkt[10] << 16) |
                                ((uint32_t)pkt[11] << 24));
    float back = 2.0f * (float)M_PI * (float)pos_raw / 32768.0f / UT_S288_RATIO;
    if (std::fabs(back - 0.1f) > 1e-3f) {
        std::printf("FAIL pos encode/decode %f\n", (double)back);
        ++fails;
    }

    // CRC length rule: only full 4-byte words
    uint8_t fifteen[15] = {0};
    if (ut_s288_crc32(fifteen, 15) != ut_s288_crc32(fifteen, 12)) {
        std::printf("FAIL CRC truncation rule\n");
        ++fails;
    }

    if (fails) {
        std::printf("ut_s288_codec: %d FAIL\n", fails);
        return 2;
    }
    std::printf("ut_s288_codec: PASS (Unitree S288)\n");
    return 0;
}
