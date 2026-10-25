#pragma once
#include <map>
#include <vector>

#include "bits.h"

namespace leapster {

enum class Write { stored, clears, sets };

struct Reg {
    u32 first, last, decoded, mask;
    bool whole = false, half = false;
    Write write = Write::stored;
    bool mirrors = true;
    u32 reset = 0;

    constexpr u32 folded(u32 at) const { return (at >> 2 & decoded) << 2; }
};

constexpr u32 all = 0xFFFFFFFF;

constexpr Reg word(u32 at, u32 mask = all, u32 reset = 0) {
    return {at, at, all, mask, false, false, Write::stored, true, reset};
}

constexpr Reg run(u32 first, u32 last, u32 decoded = all, u32 mask = all, u32 reset = 0) {
    return {first, last, decoded, mask, false, false, Write::stored, true, reset};
}

constexpr Reg system_bytes{0x01800000, 0x0180000C, all, 0x000000FF, true};
constexpr Reg delay_port = run(0x01800010, 0x0180001C, all, 0);
constexpr Reg spi_command = word(0x01800030), spi_strobe = word(0x01800034, 0),
              spi_data = run(0x01800038, 0x0180003C, all, 0);
constexpr Reg gio_out = word(0x01800040), gio_direction = word(0x01800048), gio_input = word(0x0180004C, 0);
constexpr Reg pic_pending{0x01800080, 0x01800080, all, 0x00000200, false, false, Write::clears},
    pic_enable = word(0x01800084);
constexpr Reg adc_config = run(0x01800090, 0x0180009C), adc_fifo = word(0x018000A0), adc_control = word(0x018000A4),
              adc_status{0x018000A8, 0x018000A8, all, 0x0000000F, false, false, Write::sets};
constexpr Reg csn_ctrl = run(0x01801000, 0x0180102C, ~0x30u, all, 2),
              csn_strap = run(0x01801018, 0x01801018, ~0x30u, 0, 4);
constexpr Reg audio_command = run(0x01802070, 0x01802070, ~0x3C0u, 0), voices_playing = word(0x01802078, 0);
constexpr Reg audio_control{0x018020E0, 0x018020E0, ~0x200u, all, true, true};
constexpr Reg audio_enable{0x018020F4, 0x018020F4, ~0x200u, 0xFDFF, true, true};
constexpr Reg buffer_feed{0x01803000, 0x0180301C, ~0x38u, all, false, false, Write::stored, false};
constexpr Reg voice_file{0x01804000, 0x018043FF, ~0x300u, all, true, true};
constexpr Reg frc_id = word(0x01808000, 0, 0x19850103), frc_config = run(0x01808004, 0x0180802C, all, 0),
              frc_power = word(0x01808020, 0);
constexpr Reg lcd_status = word(0x01808080, 0x000F0000), frame_buffer = run(0x01808088, 0x0180808C, all, 0x0007FFFF),
              pixel_format = word(0x01808090, 0x8000071F), raster = word(0x01808098, 0);
constexpr Reg ramp = run(0x01808180, 0x018081BC), ramp_alias = run(0x018081C0, 0x018081FC, all, 0);
constexpr Reg blit_command = word(0x01808800, 0), blit_source = word(0x01808804), blit_longs = word(0x01808808, 0xFFFF),
              blit_lines = word(0x0180880C, 0xFFFF), blit_destination = word(0x01808810, 0x0001FFFF),
              blit_status = word(0x01808818, 0);
constexpr Reg chip_id = word(0x01809000, 0, 2), straps = word(0x01809004, 0), clock_speed = word(0x01809008, 7),
              fetch_address = word(0x01809038, 0), sdram_config = word(0x01809080, all, 0x1102);
constexpr Reg sdram_ctrl = run(0x0180A000, 0x0180A00C, ~0x3Cu);
constexpr Reg analog_switch = run(0x0180C000, 0x0180C010, all, 0xFF),
              analog_enable = run(0x0180C014, 0x0180C01C, all, 1),
              analog_level = run(0x0180C020, 0x0180C024, all, 0xFF);
constexpr Reg rtc_control = word(0x0180B000, ~0x100u), rtc_status = word(0x0180B004, 0, 0x20008000),
              rtc_count = word(0x0180B020, 0),
              rtc_alarm{0x0180B024, 0x0180B024, all, 0x3FFFFFFF, true, false, Write::stored, true, 0x3FFFFFFF};
constexpr Reg timer_count[]{word(0x0180D084, 0), word(0x0180D400, 0), word(0x0180D800, 0)};
constexpr Reg timer_control[]{{0x0180D088, 0x0180D088, all, 7, true},
                              {0x0180D404, 0x0180D404, all, 7, true},
                              {0x0180D804, 0x0180D804, all, 7, true}};
constexpr Reg timer_limit[]{{0x0180D08C, 0x0180D08C, all, all, true},
                            {0x0180D408, 0x0180D408, all, all, true},
                            {0x0180D808, 0x0180D808, all, all, true}};
constexpr Reg serial_control[]{run(0x0180D514, 0x0180D514, ~0x38u, 0x47), run(0x0180D614, 0x0180D614, ~0x38u, 0x47)};
constexpr Reg other[]{
    // words that store, or read a fixed value, and nothing here reads
    word(0x01800044),
    run(0x01800050, 0x01800054),
    word(0x01800060),
    word(0x01800088),
    run(0x01801030, 0x0180103C, ~0x30u, 0),
    {0x018020E4, 0x018020E4, ~0x200u, 0x0001, true, true},
    {0x018020E8, 0x018020E8, ~0x200u, 0x001F, true, true},
    {0x018020EC, 0x018020F0, ~0x200u, 0x00BF, true, true},
    word(0x018020F8, 0),
    word(0x01808030, 0x01FF01FF),
    word(0x01808034, 0xE1FF01FF),
    word(0x01808038, 0x01FF01FF),
    word(0x0180803C, 0xE1FF01FF),
    word(0x01808084, 0x01000003),
    word(0x01808094, 0x01FF01FF),
    word(0x0180809C, 7),
    word(0x01809020),
    word(0x01809034),
    word(0x01809040),
    word(0x0180B00C, all, 0x3FFF0000),
    run(0x0180D500, 0x0180D500, ~0x38u, 0, 1),
    run(0x0180D504, 0x0180D504, ~0x38u, 0, 0xFE),
    run(0x0180D508, 0x0180D508, ~0x38u, 0, 2),
    run(0x0180D50C, 0x0180D510, ~0x38u, 0),
    run(0x0180D518, 0x0180D51C, ~0x38u, 0xFF),
    run(0x0180D600, 0x0180D600, ~0x38u, 0, 1),
    run(0x0180D604, 0x0180D604, ~0x38u, 0, 0xFE),
    run(0x0180D608, 0x0180D608, ~0x38u, 0, 2),
    run(0x0180D60C, 0x0180D610, ~0x38u, 0),
    run(0x0180D618, 0x0180D61C, ~0x38u, 0xFF),
    run(0x0180D700, 0x0180D700, ~0x3Fu),
};
// clang-format off
constexpr const Reg *map[]{
    &system_bytes, &delay_port, &spi_command, &spi_strobe, &spi_data, &gio_out, &gio_direction, &gio_input, &pic_pending,
    &pic_enable, &adc_config, &adc_fifo, &adc_control, &adc_status, &csn_ctrl, &csn_strap, &audio_command, &voices_playing,
    &audio_control, &audio_enable, &buffer_feed, &voice_file, &frc_id, &frc_config, &frc_power, &lcd_status, &frame_buffer,
    &pixel_format, &raster, &ramp, &ramp_alias, &blit_command, &blit_source, &blit_longs, &blit_lines, &blit_destination,
    &blit_status, &chip_id, &straps, &clock_speed, &fetch_address, &sdram_config, &sdram_ctrl, &analog_switch,
    &analog_enable, &analog_level, &rtc_control, &rtc_status,
    &rtc_count, &rtc_alarm, &timer_count[0], &timer_count[1], &timer_count[2], &timer_control[0], &timer_control[1],
    &timer_control[2], &timer_limit[0], &timer_limit[1], &timer_limit[2], &serial_control[0], &serial_control[1],
};
// clang-format on

inline std::vector<const Reg *> rows() {
    std::vector<const Reg *> all_rows{std::begin(map), std::end(map)};
    for (const Reg &r : other) all_rows.push_back(&r);
    return all_rows;
}

inline const Reg *rule(u32 at) {  // a later row takes a word from an earlier one
    static const auto by_word = [] {
        std::vector<const Reg *> held(1 << 14);
        for (u32 n = 0; n < held.size(); ++n)
            for (const Reg *r : rows())
                if (const u32 f = r->folded(0x01800000 + 4 * n); f >= r->first && f <= r->last) held[n] = r;
        return held;
    }();
    return at >> 16 == 0x0180 ? by_word[at >> 2 & 0x3FFF] : nullptr;
}

inline u32 fold(u32 at) { return rule(at) ? rule(at)->folded(at) : at & ~3u; }

struct Registers {
    std::map<u32, u32> words;

    Registers() {
        for (const Reg *r : rows())
            for (u32 at = r->first; r->reset && at <= r->last; at += 4) words[at] = r->reset;
    }

    explicit Registers(const std::map<u32, u32> &declared) {
        for (auto [at, value] : declared) words[fold(at)] = value;
    }

    u32 read(u32 at, u32 bytes) const {
        const auto it = words.find(fold(at));
        const u32 word = it == words.end() ? 0 : it->second;
        return (rule(at) && rule(at)->half ? (word & 0xFFFF) * 0x10001 : word) >> 8 * (at & 3) & ones(bytes);
    }

    u32 operator[](const Reg &r) const { return read(r.first, 4); }

    u32 word(const Reg &r, u32 n) const { return read(r.first + 4 * n, 4); }

    u32 half(const Reg &r, u32 offset) const { return read(r.first + offset, 2); }

    void write(u32 at, u32 bytes, u32 value) {
        const Reg *r = rule(at);
        if (!r || (!r->mirrors && r->folded(at) != (at & ~3u))) return;
        const u32 bus = std::rotl(value, int(8 * (at & 3))),
                  reach = r->mask & (r->whole ? all : ones(bytes) << 8 * (at & 3));
        u32 &word = words[fold(at)];
        word = r->write == Write::clears ? word & ~(bus & reach)
               : r->write == Write::sets ? word | (bus & reach)
                                         : (word & ~reach) | (bus & reach);
    }
};

}  // namespace leapster
