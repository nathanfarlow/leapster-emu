#pragma once
#include <algorithm>
#include <array>
#include <optional>
#include <utility>
#include <vector>

#include "registers.h"

namespace leapster {

namespace lfc {

using Record = std::array<s16, 32>;
using Filter = std::array<s16, 10>;

constexpr s16 mul(s16 a, s16 b) { return s16((a * b + 16384) >> 15); }

constexpr s16 add(s16 a, s16 b) { return s16(std::clamp(a + b, -32768, 32767)); }

std::vector<s16> decode(auto &&byte, u32 at) {
    auto rom = [&](u32 a) { return u32(byte(a) | byte(a + 1) << 8); };
    auto table = [&](u32 a, u32 i) { return s16(rom(a + 2 * i)); };
    auto half = [&](u32 i) { return rom(at + 2 * i); };
    auto field = [&](u32 i, u32 from, u32 width) { return half(i + from / 16) >> from % 16 & ((1u << width) - 1); };
    u32 n = 0;
    auto take = [&](u32 count) { return (n += count) - count; };
    auto coefficients = [&](u32 i) {
        Filter f;
        constexpr u32 width[]{5, 4, 4, 3, 3, 3, 3, 3, 2, 2};
        for (u32 j = 0, from = 0, k = 0x1180; j < 10; k += 2u << width[j], from += width[j++])
            f[j] = table(k, field(i, from, width[j]));
        return f;
    };
    const u32 word = half(take(1)), level = std::min(word & 3, 2u);
    auto record = [&](u32 i, u32 width) {
        constexpr u32 place[][8]{{11, 20, 24, 28}, {11, 22, 27, 32, 37, 42, 48, 53}};
        constexpr u32 sign[]{16, 17, 18, 19, 20, 21, 47, 63};
        constexpr u32 shape[]{0x900, 0xA80, 0xD00};
        Record s{};
        const s16 loudness = table(0x1140, field(i, 8, 5)), gain = table(0x1080 + 64 * level, field(i, 0, 5));
        for (u32 j = 0; width == 1 && j < 32; j++)
            s[j] = mul(field(i, 13, 1) ? s16(-loudness) : loudness, table(0x1800 + 64 * field(i, 0, 8), j));
        for (u32 p = 0; width > 1 && p < 4 + 2 * level; p++) {
            const s16 a = p ? mul(gain, table(shape[level] + 2 * (3 + 2 * level) * field(i, 5, 6), p - 1)) : gain;
            s[field(i, place[level > 0][p], level || !p ? 5 : 4) << (!level && p)] = field(i, sign[p], 1) ? s16(-a) : a;
        }
        return s;
    };

    std::vector<std::pair<Record, Filter>> played(1);
    auto continues = [](u32 m) { return m != 0x0FC0; };
    auto present = [](u32 m) { return m & 0x3F; };
    auto header = [&] {
        const u32 m = half(take(1));
        return std::pair{m, present(m) ? coefficients(take(2)) : Filter{}};
    };
    auto frame = [&](u32 m, Filter f) {
        for (u32 j = 0; j < 6; j++) {
            const u32 width = m >> (11 - j) & 1 ? 2 + level : m >> (5 - j) & 1;
            played.emplace_back(record(take(width), width), f);
        }
    };
    auto [mask, filter] = header();
    if (!present(mask) && continues(mask)) frame(mask, Filter{});
    u32 frames = 0;
    for (; continues(mask) && frames < 1 << 16; frames++) {
        const auto [ahead, next] = header();
        frame(mask, present(mask) ? filter : played.back().second);
        mask = ahead, filter = next;
    }
    if (frames) played.insert(played.end(), 6, {Record{}, played.back().second});

    std::vector<s16> out;
    std::array<s16, 11> b{};
    for (const auto &[excitation, ks] : played)
        for (s16 f : excitation) {
            for (u32 j = 10; j--;) {
                f = add(f, mul(ks[j], b[j]));
                b[j + 1] = add(b[j], mul(s16(-ks[j]), f));
            }
            out.push_back(b[0] = f);
        }
    if (word & 4) {
        for (u32 i = 1; i < out.size(); i += 2) out[i / 2] = out[i];
        out.resize(out.size() / 2);
    }
    return out;
}

}  // namespace lfc

constexpr u64 sample_clocks = 2000;

struct Audio {
    struct Voice {
        u64 phase{};
        std::vector<s16> speech;
    };

    static constexpr u32 voice_bit[]{0x80, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40};
    const std::vector<u8> &bios, &cart;
    const Registers &registers;

    Audio(const std::vector<u8> &b, const std::vector<u8> &c, const Registers &r) : bios{b}, cart{c}, registers{r} {}

    std::array<Voice, 8> voices;
    u32 playing{};
    bool recording{};
    std::vector<s16> sound;
    u64 next_sample{sample_clocks};

    static constexpr s32 alaw(u32 byte) {
        const u32 b = byte ^ 0x55, exponent = b >> 4 & 7, mantissa = (b & 15) << 4;
        const s32 v = s32(exponent ? (mantissa + 0x108) << (exponent - 1) : mantissa + 8);
        return b & 0x80 ? -v : v;
    }

    u8 rom(u32 at) const {
        if (at >> 30 == 1 || at < 0x80000) return bios.empty() ? u8(0x55) : bios[at & 0x1FFFFF];
        return at >> 30 == 2 && !cart.empty() ? cart[at & 0x7FFFFF] : u8(0x55);
    }

    enum : u32 {
        START = 0xC4,
        END = 0x104,
        INTERVAL = 0x15C,
        GAIN = 0x1A4,
        MASTER = 0x1C4
    };  // the voice file's fields

    u32 half(u32 offset) const { return registers.half(voice_file, offset); }

    u32 start(u32 v) const { return half(START + 8 * v) << 16 | half(START + 4 + 8 * v); }

    u32 high(u32 v) const { return half(END + 8 * v); }

    u32 low(u32 v) const { return half(END + 4 + 8 * v); }

    u32 command(u32 value, u64 now) {
        u32 lines = mix_until_now(now);
        if (!playing) next_sample = (now / sample_clocks + 1) * sample_clocks;
        const u32 v = value & 7;
        Voice &voice = voices[v];
        if ((value & 0xF8) == 0x80) {
            voice.phase = v < 5 ? u64(start(v) & 0xFFFF) << 32 : 0;
            if (v == 7) voice.speech = lfc::decode([&](u32 at) { return rom(at); }, start(v));
            playing |= voice_bit[v];
        } else if (value & 0xF8 && playing & voice_bit[v]) playing &= ~voice_bit[v], lines |= 1u << 16;
        return lines | 1u << 18;
    }

    u32 mix_until_now(u64 now) {
        u32 lines = 0;
        for (; next_sample <= now && (playing || recording); next_sample += sample_clocks) lines |= mix();
        return lines;
    }

    u32 mix() {
        s32 sum = 0;
        u32 lines = 0;
        for (u32 v = 0; v < 8; ++v) {
            if (!(playing & voice_bit[v])) continue;
            Voice &voice = voices[v];
            u64 at = voice.phase >> 32;
            const u32 base = v < 5 ? start(v) & 0xFFFF0000 : start(v), end = high(v) << 16 | low(v);
            if (v >= 5 && at >= (v == 7 ? voice.speech.size() * 6 : end - std::min(end, base))) {
                playing &= ~voice_bit[v], lines |= 1u << 16;
                continue;
            }
            if (v < 5 && at >= high(v)) at = low(v), voice.phase = at << 32 | u32(voice.phase);
            const s32 gain = s32(half(GAIN + 4 * v));
            if (v == 7) {
                if (recording) sum += s32(s64{voice.speech[at / 6]} * gain >> 15);
                voice.phase += 1ull << 32;
                continue;
            }
            if (recording) {
                const u64 following =
                    v < 5 ? (at + 1 >= high(v) ? low(v) : at + 1) : std::min(at + 1, u64{end - base - 1});
                const s32 first = alaw(rom(base + u32(at))), second = alaw(rom(base + u32(following)));
                const s32 between = std::clamp(second - first, -32768, 32767) * s32(voice.phase >> 16 & 0xFFFF) >> 16;
                sum += s32(s64{first + between} * gain >> 15);
            }
            const u64 interval = half(INTERVAL + 4 * v);
            voice.phase += v < 5 ? ((interval << 32) + 98304) / 196608 : ((8000ull << 32) + 24000) / 48000;
        }
        const s32 master = s32(half(MASTER));
        if (recording) sound.push_back(s16(std::clamp(sum, -32768, 32767) * (master ? master : 0x8000) >> 15));
        return lines;
    }
};

}  // namespace leapster
