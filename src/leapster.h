#pragma once
#include <array>
#include <deque>
#include <map>
#include <vector>

#include "audio.h"
#include "cpu.h"
#include "registers.h"

namespace leapster {

constexpr u64 line_clocks = 2100, frame_clocks = 160 * line_clocks, count_clocks = 6;
constexpr u32 display = 0x03000000;

struct Board {
    static std::vector<u8> tiled(const std::vector<u8> &part, u32 size) {
        std::vector<u8> whole(part.empty() ? 0 : size);
        for (u32 n = 0; n < whole.size(); ++n) whole[n] = part[n % part.size()];
        return whole;
    }

    Board(Timing &t, const std::vector<u8> &b, const std::vector<u8> &c)
        : eeprom{std::vector<u8>(cartridge_eeprom(c), 0xFF), std::vector<u8>(256, 0xFF)}, timing{t},
          bios{tiled(b, 0x200000)}, cart{tiled(c, 0x800000)} {
        t.divider = &registers.words[clock_speed.first];
        t.sdram_config = &registers.words[sdram_config.first];
        for (u32 n = 0; n < 12; ++n) t.chip_select[n] = &registers.words[csn_ctrl.first + 4 * n];
    }

    u64 now{};

    std::array<std::vector<u8>, 4> eeprom;
    u32 buttons{};
    std::optional<std::array<u16, 2>> touch;

    void stylus(u32 x, u32 y) {
        const auto w = [&](u32 n) { return eeprom[1].size() < 36 ? 0.0 : double(s32(little(&eeprom[1][4 * n], 4))); };
        const double a = w(3), b = w(4), c = w(5), d = w(6), e = w(7), f = w(8), det = a * e - b * d;
        const double X = (10 * x + 5) * w(2) - c, Y = (10 * y + 5) * w(2) - f;
        const auto raw = [](double v) { return u16(std::clamp(v, 0.0, 65535.0)); };
        if (det == 0) touch = {{u16(0xC0 + x * 0x680 / 159), u16(0xC0 + y * 0x680 / 159)}};
        else touch = {{raw(2 * (e * X - b * Y) / det), raw(2 * (a * Y - d * X) / det)}};
    }

    std::array<u16, 160 * 160> panel() const {
        const u32 fb = registers[frame_buffer] & 0xFFFF;
        std::array<u16, 160 * 160> pixels;
        for (u32 at = 0; at < pixels.size(); at += 2) {
            const auto b = [&](u32 n) { return u16(sram[(fb + at / 2 * 3 + n) & 0xFFFF]); };
            pixels[at] = u16((b(1) >> 4) << 8 | b(0)), pixels[at + 1] = u16((b(1) & 15) << 8 | b(2));
        }
        return pixels;
    }

    u32 load(u32 at, u32 bytes) {
        if (const u8 *p = memory(at & -bytes)) return little(p, bytes);
        if (mmio(at)) {
            const std::optional<u32> word = live(at & ~3u);
            return word ? *word >> 8 * (at & 3) & ones(bytes) : registers.read(at, bytes);
        }
        return at >= 0xC0000000 || (at >= 0x0C000000 && at < 0x28000000) ? ones(bytes) : 0;
    }

    void store(u32 at, u32 bytes, u32 value) {
        if (mmio(at)) return write(at, bytes, value);
        if (at >> 26 == 0x0F || at >> 18 == 0xC0)
            for (u8 *p = memory(at & -bytes); bytes--; value >>= 8) *p++ = u8(value);
    }

    u32 requests{};

    void advance(u64 to) {
        if ((now = to) >= next) events();
    }

  private:
    Timing &timing;
    std::vector<u8> bios, cart, dram = std::vector<u8>(0x200000), sram = std::vector<u8>(0x10000);
    Registers registers;

  public:
    Audio audio{bios, cart, registers};

    std::vector<s16> &samples() { return pending |= audio.mix_until_now(now), audio.sound; }

  private:
    u8 *memory(u32 at) {
        if (at >> 30 == 1 || at < 0x80000) return bios.empty() ? nullptr : &bios[at & 0x1FFFFF];
        if (at >> 30 == 2) return cart.empty() ? nullptr : &cart[at & 0x7FFFFF];
        if (at >> 26 == 0x0F) return &dram[at & 0x1FFFFF];
        if (at >> 18 == 0xC0) return &sram[at & 0xFFFF];
        return nullptr;
    }

    static bool mmio(u32 at) { return at >> 20 == 0x018; }

    u32 pending{};

    u32 lines() const { return pending | u32{!adc.empty()} << 8; }

    void request() {
        constexpr std::pair<u32, u32> wired[]{{8, 16}, {14, 18}, {16, 21}, {18, 29}};
        const u32 active = lines() & registers[pic_enable];
        requests = 0;
        for (auto [line, vector] : wired) requests |= (active >> line & 1) << vector;
        for (u32 n = 0; n < 3; ++n) requests |= u32{raised[n]} << counter_vector[n];
        if ((registers[serial_control[0]] | registers[serial_control[1]]) & 0x40) requests |= 1u << 27;
    }

    static constexpr u32 counter_vector[]{25, 26, 30};
    std::array<u32, 3> seed{};
    std::array<u64, 3> started{};
    std::array<bool, 3> raised{}, written{};

    u32 count(u32 n) const { return now < started[n] ? seed[n] : seed[n] + u32((now - started[n]) / count_clocks); }

    static constexpr u64 tick_clocks = 96000000;
    u32 clock{};
    u64 clock_set{};

    u32 seconds() const { return clock >> 31 ? clock + u32(now / tick_clocks - clock_set / tick_clocks) : clock; }

    static u32 cartridge_eeprom(const std::vector<u8> &cart) {  // the EEpromMPI resource's fields, in order
        for (std::size_t at = 0; at + 24 <= cart.size(); at += 4) {
            const u8 *p = &cart[at];
            if (little(p, 4) == 0x100 && p[4] == 1 && p[5] == 0 && little(p + 12, 4) == 0x2000 &&
                little(p + 16, 4) == 100 && little(p + 20, 4) - 1 < 0x800)
                return little(p + 20, 4);
        }
        return 0;
    }

    std::array<u64, 4> port_busy{}, writing{};
    u64 eeprom_ready{};
    u32 eeprom_byte{};

    u64 epoch{}, blit_start{}, blit_done{};
    u32 blit_flags{}, lcd_flags{5};

    u32 scan() const {
        const u64 in_frame = (now - epoch) % frame_clocks;
        return u32(in_frame % line_clocks * 2 / 3) << 16 | u32(in_frame / line_clocks);
    }

    u16 touched(u32 mux) const {
        if (!touch) return 0x7FF;
        const u32 out = registers[gio_out] & 0xF00000, direction = registers[gio_direction] & 0xF00000;
        if (direction == 0xC00000 && out == 0x800000 && mux == 3) return 0x100;
        if (direction == 0x300000 && out == 0x200000 && mux == 1) return 0x100;
        if (direction == 0xA00000 && out == 0x800000 && mux == 2) return (*touch)[0];
        if (direction == 0x500000 && out == 0x400000 && mux == 1) return (*touch)[1];
        return 0x7FF;
    }

    std::deque<u32> adc;
    std::array<u64, 2> adc_due{};

    bool channel(u32 n, bool fast) const {
        const u32 config = registers.word(adc_config, n), rate = config >> 16 & 3;
        return config & 0x8000 && rate != 3 && (rate != 0) == fast;
    }

    bool converting(bool fast) const {
        return channel(0, fast) || channel(1, fast) || channel(2, fast) || channel(3, fast);
    }

    void convert(bool fast) {
        constexpr u16 sample[]{0, 0, 1100, 990};
        for (u32 n = 0; n < 4; ++n) {
            const u32 mux = registers.word(adc_config, n) & 7;
            if (channel(n, fast) && adc.size() < 64 && registers[adc_control] & 0xFF)
                adc.push_back(mux << 24 | n << 16 | u32(n ? sample[n] : touched(mux)) << 5);
        }
        pending |= 1u << 9;
    }

    u64 after(u64 t, bool fast) const {
        if (fast) return (t / 24576 + 1) * 24576;
        const u64 base = t - (t - epoch) % frame_clocks;
        for (u64 line : {39u, 78u, 117u, 156u})
            if (base + line * line_clocks > t) return base + line * line_clocks;
        return base + frame_clocks + 39 * line_clocks;
    }

    std::optional<u32> live(u32 at) {
        for (u32 n = 0; n < 3; ++n)
            if (at == timer_count[n].first) return count(n);
        if (at >> 8 == 0x0180D5 || at >> 8 == 0x0180D6) {
            if ((at & 0x1C) == 0x10) return 0;
            if ((at & 0x1C) == 0x14) return registers.read(at, 4) | 0xA0;
        }
        if (at >= delay_port.first && at <= delay_port.last) return now < port_busy[at / 4 & 3] ? 0x80 : 0;
        if (at >= ramp_alias.first && at <= ramp_alias.last) return registers[ramp];
        switch (at) {
        case gio_input.first: return ~buttons;
        case pic_pending.first: return lines();
        case adc_fifo.first:
            return adc.empty() ? 0 : [&] {
                const u32 word = adc.front();
                return adc.pop_front(), next = 0, word;
            }();
        case adc_status.first: return registers[adc_status] | (adc.empty() ? 0x100 : 0);
        case spi_data.first: return eeprom_byte;
        case lcd_status.first: return registers[lcd_status] | lcd_flags;
        case voices_playing.first: return audio.playing;
        case raster.first: return scan();
        case rtc_count.first: return seconds();
        case blit_status.first: return blit_flags | (blit_done > now && !blit_start ? 1 : 0);
        case straps.first: return 0x0BFFFFFF;
        case fetch_address.first: return timing.asked;
        default: return std::nullopt;
        }
    }

    void write(u32 at, u32 bytes, u32 value) {
        if (fold(at) == audio_enable.first && value & 0x8000 && !(registers[audio_enable] & 0x8000))
            pending |= 1u << 18;
        registers.write(at, bytes, value);
        const u32 word = at & ~3u, lanes = value << 8 * (at & 3);
        for (u32 n = 0; n < 3; ++n) {
            if (word == timer_count[n].first) seed[n] = value, started[n] = now + 25, written[n] = true;
            if (word == timer_control[n].first) raised[n] = false;
        }
        if (word >= delay_port.first && word <= delay_port.last)
            port_busy[word / 4 & 3] = std::max(port_busy[word / 4 & 3], now) + 66 * 96;
        if (word == spi_strobe.first && value) {
            const u32 command = registers[spi_command], offset = command >> 16 & 0x7FF;
            const u32 port = std::array{0u, 2u, 3u, 1u}[command >> 9 & 3];
            std::vector<u8> &chip = eeprom[port];
            if (value == 2 && offset < chip.size()) chip[offset] = u8(command);
            if (value != 2) eeprom_byte = offset < chip.size() ? chip[offset] : 0xFF;
            eeprom_ready = std::max(now, writing[port]) + 408 * 96;
            if (value == 2) writing[port] = eeprom_ready + 5000 * 96;
        }
        if (word == pic_pending.first) pending &= ~lanes;
        if (word == blit_status.first) blit_flags &= ~(lanes & 0x0E);
        if (word == lcd_status.first) lcd_flags &= ~(lanes & 5), pending |= 1u << 12;
        if (word == rtc_count.first) clock = value, clock_set = now;
        if (word == frc_power.first && value >> 31 && !(registers[pixel_format] >> 31)) epoch = now;
        if (word == blit_command.first && (value & 0xFF) == 0x1B) {
            blit_start = now - (now - epoch) % frame_clocks + frame_clocks;
            blit_done = blit_start + registers[blit_lines] * line_clocks;
        }
        if (fold(at) == audio_command.first) pending |= audio.command(value, now);
        next = 0;
    }

    u64 next{};

    void events() {
        if (blit_start && now >= blit_start) {
            const u32 from = registers[blit_source], to = display + (registers[blit_destination] & 0xFFFF);
            const u32 length = registers[blit_longs] * 4 * registers[blit_lines];
            for (u32 n = 0; n < length; ++n) sram[(to + n) & 0xFFFF] = memory(from + n) ? *memory(from + n) : 0;
            blit_start = 0;
        }
        if (blit_done && !blit_start && now >= blit_done) blit_done = 0, blit_flags |= 0x0C, pending |= 1u << 14;
        if (eeprom_ready && now >= eeprom_ready) eeprom_ready = 0, pending |= 1u << 7;
        for (const bool fast : {false, true}) {
            u64 &due = adc_due[fast];
            if (!converting(fast)) due = 0;
            else if (!due) due = after(now, fast);
            else if (now >= due) convert(fast), due = after(due, fast);
        }
        pending |= audio.mix_until_now(now);
        next = ~0ull;
        for (u32 n = 0; n < 3; ++n) {
            const u32 limit = registers[timer_limit[n]];
            if (!limit) continue;
            const u64 counted = now < started[n] ? started[n] : now - (now - started[n]) % count_clocks;
            if (written[n] && seed[n] > limit && count(n) >= seed[n]) {
                next = std::min(next, counted + ((u64{1} << 32) - count(n)) * count_clocks);
                continue;
            }
            if (count(n) > limit)
                seed[n] -= limit + 1, raised[n] |= registers[timer_control[n]] & 1, written[n] = false;
            next = std::min(next, count(n) > limit ? now : counted + (u64{limit} + 1 - count(n)) * count_clocks);
        }
        if (clock >> 31 && (seconds() & 0x3FFFFFFF) == registers[rtc_alarm]) pending |= 1u << 4;
        if (clock >> 31) next = std::min(next, (now / tick_clocks + 1) * tick_clocks);
        for (u64 t :
             {blit_start, blit_done, eeprom_ready, adc_due[0], adc_due[1], audio.playing ? audio.next_sample : 0})
            if (t) next = std::min(next, t);
        request();
    }
};

struct Leapster {
    Timing timing;
    Board board;
    Cpu<Board> cpu;

    Leapster(const std::vector<u8> &bios, const std::vector<u8> &cart, const State &start = {})
        : board{timing, bios, cart}, cpu{board, timing, start} {}

    void step() {
        const u64 before = cpu.cycle();
        cpu.step();
        board.advance(board.now + ((cpu.cycle() - before) << timing.shift()));
    }
};

}  // namespace leapster
