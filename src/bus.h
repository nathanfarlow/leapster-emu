#pragma once
#include <algorithm>
#include <array>
#include <utility>

#include "bits.h"

namespace leapster {

enum Region { dram, sram, csn, unmapped, timeout, csn_regs, sdram_regs, peripheral_regs, audio_regs, power_regs };

constexpr Region region(u32 at) {
    constexpr Region page[]{peripheral_regs, csn_regs,        audio_regs, peripheral_regs, audio_regs, unmapped,
                            unmapped,        unmapped,        audio_regs, unmapped,        sdram_regs, power_regs,
                            peripheral_regs, peripheral_regs, unmapped,   unmapped};
    if (at >> 26 == 0x0F) return dram;
    if (at >> 18 == 0xC0) return sram;
    if (at >> 30 || at < 0x80000 || (at >= 0x0C000000 && at < 0x28000000)) return csn;
    if (at >> 18 == 0x3F) return timeout;
    if (at >> 16 != 0x0180) return unmapped;
    if (at >> 12 == 0x0180D && (at >> 8 & 15) - 5 < 2) return audio_regs;
    if (at >> 12 == 0x0180B && at & 0xF00) return unmapped;
    return page[at >> 12 & 15];
}

struct Cost { u32 core, fixed; };

struct Cache {
    static constexpr u32 dirty = 1u << 29;
    std::array<std::array<u32, 4>, 512> line{};
    std::array<u8, 512> victim{};

    static u32 set(u32 at) { return at >> 4 & 511; }

    static u32 tag(u32 at) { return at >> 4 | 1u << 28; }

    u32 *find(u32 at) {
        for (u32 &l : line[set(at)])
            if ((l & ~dirty) == tag(at)) return &l;
        return nullptr;
    }

    bool has(u32 at) { return find(at); }

    bool fill(u32 at) { return std::exchange(line[set(at)][victim[set(at)]++ & 3], tag(at)) & dirty; }

    void advance(u32 at) { ++victim[set(at)]; }

    void mark_dirty(u32 at) { *find(at) |= dirty; }

    void drop(u32 at) {
        if (u32 *l = find(at)) *l = 0;
    }

    void clear() { line = {}; }

    void clean() {
        for (auto &ways : line)
            for (u32 &l : ways) l &= ~dirty;
    }
};

struct Timing {
    bool measured{true};
    Cache code, data;
    u32 asked{};
    const u32 *divider{}, *chip_select[12]{}, *sdram_config{};

    u32 shift() const { return divider ? std::min(*divider, 5u) : 0; }

    u32 chip(u32 n) const { return chip_select[n] ? *chip_select[n] : 0x56; }

    u32 field() const { return sdram_config ? *sdram_config & 15 : 2; }

    u32 scaled(u32 clocks) const { return clocks * std::array{24u, 18u, 12u, 6u, 36u, 8u, 24u, 24u}[field() & 7] / 12; }

    u32 sdram(u32 clocks) const { return fixed(scaled(clocks) + (field() & 8 ? 3 : 0)); }

    static constexpr Cost uncached_load[]{{10, 50}, {5, 44}, {8, 13}, {4, 0},  {256, 0},
                                          {7, 11},  {7, 17}, {6, 37}, {6, 43}, {0, 8 * 2930}};
    static constexpr Cost uncached_store[]{{6, 20}, {0, 24}, {0, 27}, {6, 0},  {259, 0},
                                           {5, 4},  {2, 5},  {0, 24}, {0, 30}, {0, 8 * 2930}};
    static constexpr Cost fill_first[]{{15, 62}, {10, 43}, {11, 13}, {7, 0}};
    static constexpr Cost fill_line[]{{0, 81}, {9, 68}, {12, 15}, {19, 0}};
    static constexpr Cost fill_jump[]{{9, 74}, {11, 63}, {12, 14}, {12, 158}};
    static constexpr Cost fill_next[]{{18, 55}, {8, 59}, {12, 14}, {12, 158}};
    static constexpr u32 word_bus = 69, write_back = 42, row_open = 90, row_recovered = 117, activate = 6;
    static constexpr u32 odd_period_hold = 49;
    static constexpr u32 replayed = 34, hit_under_fill = 9, merged = 20;
    static constexpr u32 odd_half = 4, odd_fresh = 24;

    std::array<u64, 4> busy{};
    u64 next_read{}, row_grant{}, read_hold{};
    u32 prefetching{~0u}, newest{~0u};
    u64 prefetched{}, filled_code{};
    std::array<u64, 2> queue{};
    u32 filling{~0u};
    u64 filled{}, completed{}, posted{};

  private:
    template <class M>
    friend struct Cpu;

    u32 fixed(u32 clocks) const { return clocks >> shift(); }

    u32 cycles(Cost c) const { return c.core + fixed(c.fixed); }

    u32 cycles(Cost c, Region r) const { return c.core + (r == dram ? sdram(c.fixed) : fixed(c.fixed)); }

    static u32 select(u32 at) {
        if (at >> 30) return at >> 30 == 3 ? 2 + (at >> 29 & 1) : (at >> 30) - 1;
        const u32 n = at < 0x80000 ? 0 : (at >> 26) + 1;
        return n + (n >= 6);
    }

    u32 strobes(u32 at, u32 n) const {
        const u32 word = chip(select(at)), w = bits(word, 4, 2), r = bits(word, 6, 5) == 0;
        n <<= !bit(word, 1);
        return 3 * n * ((w || r ? w : 4) + 1) + 3 * r * (n - 1);
    }

    u32 cycles(const Cost *table, u32 at, u32 n, bool fill = true) const {
        const Region r = region(at);
        return cycles(table[fill ? std::min(r, unmapped) : r], r) + (r == csn ? fixed(strobes(at, n)) : 0);
    }

    u64 &port(u32 at) { return busy[std::min(region(at), unmapped)]; }

    u32 fetch(u32 at, u64 now, bool narrow) {
        if (!measured) return 0;
        u64 done = now;
        if ((at & ~15u) == prefetching) {
            done = filled_code = std::max(now, prefetched);
            newest = std::exchange(prefetching, ~0u);
        } else if (!code.has(at)) {
            code.fill(at), asked = newest = at & ~15u;
            done = filled_code = next_read = port(at) = std::max({now, port(at), next_read}) + cycles(fill_jump, at, 8);
        }
        if (at & 8 && !code.has(at + 16)) {
            code.fill(at + 16), asked = prefetching = (at + 16) & ~15u;
            prefetched = next_read = port(at) = std::max({done, port(at), next_read}) + cycles(fill_next, at, 8);
        }
        if (narrow && (at & 2) && (at & ~15u) == newest && done < filled_code + fixed(odd_fresh)) done += odd_half;
        return u32(done - now);
    }

    u64 admit(u64 now) const { return std::max(now, std::min(queue[0], queue[1])); }

    u64 access(u32 at, bool store, bool uncached, u64 now) {
        if (!measured) return now;
        const Region r = region(at);
        const bool own = (at & ~15u) == filling, posting = store && uncached, word = uncached && !store && r == dram;
        u32 cost = 0, line = 0;
        bool evicts = false;
        if (posting) cost = cycles(uncached_store[r], r) + (r == csn && !bit(chip(select(at)), 1) ? fixed(18) : 0);
        else if (uncached) cost = cycles(uncached_load, at, 2, false);
        else if (!data.has(at))
            cost = cycles(fill_first, at, 2), line = cycles(fill_line, at, 8), evicts = data.fill(at);
        else if (store || !own || filled <= now) data.advance(at);
        if (store && !uncached) data.mark_dirty(at);
        if (!cost) {
            if (store && own && now < filled) posted = filled + fixed(merged);
            const u64 answered = own     ? std::max(filled + fixed(replayed), completed)
                                 : store ? now
                                         : std::max({filled + fixed(hit_under_fill), completed, posted});
            if (answered <= now) return now;
            return queue[queue[0] > queue[1]] = answered;
        }
        u64 grant = std::max({now, port(at), posting ? 0 : next_read, word ? read_hold : 0});
        bool open = false;
        if (r == dram) {
            open = grant <= row_grant + sdram(row_open);
            if (!open) grant = std::max(grant, row_grant + sdram(row_recovered));
            row_grant = grant;
            if (word && fixed(scaled(12)) == 6) read_hold = grant + odd_period_hold;
        }
        const u64 done = grant + cost - (open ? sdram(activate) : 0);
        port(at) = (line        ? grant + line
                    : r == dram ? grant + sdram(word_bus)
                                : done) +
                   (evicts ? fixed(write_back) : 0);
        if (!posting) next_read = port(at);
        if (r == power_regs && store) busy.fill(done);
        if (line) filling = at & ~15u, filled = done, completed = done + line - cost;
        return queue[queue[0] > queue[1]] = done;
    }
};

}  // namespace leapster
