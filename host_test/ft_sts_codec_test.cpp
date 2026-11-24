// Host-side Feetech STS / HD1910 codec checks (no hardware).
#include <cmath>
#include <cstdint>
#include <cstdio>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

static uint8_t checksum(const uint8_t *body, int len)
{
    uint8_t sum = 0;
    for (int i = 0; i < len; ++i) sum = (uint8_t)(sum + body[i]);
    return (uint8_t)(~sum);
}

static int32_t decode_signed(uint16_t raw)
{
    if (raw & 0x8000) return -(int32_t)(raw & 0x7FFF);
    return (int32_t)(raw & 0x7FFF);
}

static float pos_to_rad(uint16_t raw)
{
    return (float)(2.0 * M_PI * (raw & 0x0FFF) / 4096.0 - M_PI);
}

static uint16_t rad_to_pos(float rad)
{
    return (uint16_t)std::lround(4096.0f * ((float)M_PI + rad) / (2.0f * (float)M_PI));
}

int main()
{
    int fails = 0;
    // Wiki example: READ pos len2 → FF FF 01 04 02 38 02 BE
    uint8_t body[] = {0x01, 0x04, 0x02, 0x38, 0x02};
    if (checksum(body, 5) != 0xBE) {
        std::printf("FAIL checksum BE\n");
        ++fails;
    }
    // Reply FF FF 01 04 00 00 08 F2 → position 2048
    if (std::fabs(pos_to_rad(2048) - 0.0f) > 1e-5f || rad_to_pos(0.0f) != 2048) {
        std::printf("FAIL center encoding\n");
        ++fails;
    }
    if (decode_signed(0x800A) != -10 || decode_signed(10) != 10) {
        std::printf("FAIL signed BIT15\n");
        ++fails;
    }
    if (fails) {
        std::printf("ft_sts_codec: %d FAIL\n", fails);
        return 2;
    }
    std::printf("ft_sts_codec: PASS (Feetech STS/HD1910)\n");
    return 0;
}
