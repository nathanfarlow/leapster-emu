#pragma once
#include <bit>
#include <cstdint>

namespace leapster {

using u8 = std::uint8_t;
using u16 = std::uint16_t;
using u32 = std::uint32_t;
using u64 = std::uint64_t;
using s16 = std::int16_t;
using s32 = std::int32_t;
using s64 = std::int64_t;

constexpr u32 bits(u32 word, int hi, int lo) { return word << (31 - hi) >> (31 - hi + lo); }

constexpr bool bit(u32 word, int n) { return word >> n & 1; }

constexpr u32 sext(u32 value, int width) { return u32(s32(value << (32 - width)) >> (32 - width)); }

constexpr u32 ones(u32 bytes) { return u32((1ull << 8 * bytes) - 1); }

constexpr u32 little(const u8 *p, u32 bytes) {
    return bytes == 4   ? u32(p[0] | p[1] << 8 | p[2] << 16) | u32(p[3]) << 24
           : bytes == 2 ? u32(p[0] | p[1] << 8)
                        : p[0];
}

}  // namespace leapster
