#pragma once
#include <algorithm>
#include <array>
#include <bit>
#include <map>
#include <optional>
#include <string_view>
#include <utility>
#include <vector>

#include "bus.h"

namespace leapster {

inline constexpr u32 H = 1u, E1 = 1u << 1, E2 = 1u << 2;                    // STATUS32: halted, levels enabled
inline constexpr u32 V = 1u << 8, C = 1u << 9, N = 1u << 10, Z = 1u << 11;  // STATUS32 flags
inline constexpr u32 SAT = 1u << 4, OVER = 1u << 5, NEG = 1u << 6, ZERO = 1u << 7, ACC = 1u << 8;  // MACMODE
inline constexpr u32 FRACTIONAL = 1u << 2, HIGH_HALF = 1u << 3, SATURATE_32 = 1u << 14;  // MACMODE, both channels
inline constexpr u32 BH = 1u << 29, SH = 1u << 30;                                       // DEBUG

enum Register : u32 { gp = 26, fp, sp, ilink1, ilink2, blink, acc1 = 56, acc2, mlo, mhi, lp_count, limm = 62, pcl };

struct State {
    std::array<u32, 64> r{};
    u32 pc{}, status32{}, lp_start{}, lp_end{}, status32_l1{}, status32_l2{};
    u32 irq_lv12{}, irq_lev{}, irq_hint{}, int_vector_base{}, semaphore{}, debug{};
    u32 macmode{}, xmac0{}, xmac1{}, xmac2{};
    u32 count0{}, control0{}, limit0{~0u};
};

constexpr u32 kept_flags = Z | N | C | V | E1 | E2;

// clang-format off
struct Word { std::string_view name; u32 State::*field; u32 number, mask; };
constexpr Word state_words[]{
    {"pc", &State::pc, 0, 0}, {"status32", &State::status32, 0x0A, 0}, {"lp_start", &State::lp_start, 0x02, 0},
    {"lp_end", &State::lp_end, 0x03, 0}, {"status32_l1", &State::status32_l1, 0x0B, kept_flags},
    {"status32_l2", &State::status32_l2, 0x0C, kept_flags}, {"irq_lv12", &State::irq_lv12, 0x43, 0},
    {"irq_lev", &State::irq_lev, 0x200, ~7u}, {"irq_hint", &State::irq_hint, 0x201, 0},
    {"int_vector_base", &State::int_vector_base, 0x25, ~0x3FFu}, {"semaphore", &State::semaphore, 0x01, 0xF},
    {"debug", &State::debug, 0x05, 0}, {"macmode", &State::macmode, 0x41, 0}, {"xmac0", &State::xmac0, 0x44, 0},
    {"xmac1", &State::xmac1, 0x45, 0}, {"xmac2", &State::xmac2, 0x46, 0}, {"count0", &State::count0, 0x21, 0},
    {"control0", &State::control0, 0x22, ~0u}, {"limit0", &State::limit0, 0x23, ~0u}};
// clang-format on

// clang-format off
enum class Op : u32 {
    add = 0x400, adc, sub, sbc, and_, or_, bic, xor_, max, min, mov, tst, cmp, rcmp, rsub, bset,
    bclr, btst, bxor, bmsk, add1, add2, add3, sub1, sub2, sub3,
    j = 0x420, j_d, jl, jl_d, lp = 0x428, flag, lr, sr,
    asl = 0x500, lsr, asr, ror, mul64, mulu64, adds, subs, divaw, asls = 0x50A, asrs,
    muldw = 0x50C, mulrdw = 0x50E, macdw = 0x510, macudw, msubdw = 0x514,
    asl1 = 0x480, asr1, lsr1, ror1, rrc, sexb, sexw, extb, extw, abs, not_, rlc, zero_operand = 0x4BF,
    swap = 0x580, norm, sat16, rnd16, abssw, abss, negsw, negs, normw,
};
// clang-format on

namespace alu {
struct Result {
    u32 value{}, flags{}, mask{};
    bool kept = true, known = true, saturated = false;
};

constexpr Result test(Result r) { return r.kept = false, r; }

constexpr u32 zn(u32 v) { return (v == 0 ? Z : 0) | (v >> 31 ? N : 0); }

constexpr Result logic(u32 v) { return {v, zn(v), Z | N}; }

constexpr Result moved(u32 v, bool out) { return {v, zn(v) | (out ? C : 0), Z | N | C}; }

constexpr Result add(u32 b, u32 c, u32 carry) {
    const u64 sum = u64{b} + c + carry;
    const u32 v = u32(sum);
    return {v, zn(v) | (sum >> 32 ? C : 0) | ((~(b ^ c) & (b ^ v)) >> 31 ? V : 0), Z | N | C | V};
}

constexpr Result sub(u32 b, u32 c, u32 borrow) {
    const u32 v = b - c - borrow;
    return {v, zn(v) | (u64{c} + borrow > b ? C : 0) | (((b ^ c) & (b ^ v)) >> 31 ? V : 0), Z | N | C | V};
}

constexpr Result pick(u32 b, u32 c, bool second) {
    return {second ? c : b, (sub(b, c, 0).flags & ~C) | (second ? C : 0), Z | N | C | V};
}

constexpr Result saturated(s64 exact, int bits = 32) {
    const s64 clamped = std::clamp(exact, -(s64{1} << (bits - 1)), (s64{1} << (bits - 1)) - 1);
    const u32 v = u32(clamped);
    return {v, zn(v) | (clamped != exact ? V : 0), Z | N | V, true, true, clamped != exact};
}

constexpr u32 norm(u32 c) { return u32(std::countl_zero(s32(c) < 0 ? ~c : c)) - 1; }

constexpr Result signed_as(Result r, u32 operand) { return r.flags = (r.flags & ~N) | (operand >> 31 ? N : 0), r; }

constexpr s64 scaled(u32 b, s64 left) {
    return left < 0 ? s64{s32(b)} >> std::min<s64>(-left, 31) : s64{s32(b)} << std::min<s64>(left, 32);
}

constexpr Result compute(Op op, u32 b, u32 c, u32 carry) {
    const u32 n = c & 31;
    const bool out = n && bit(b, int(n) - 1);
    if (op >= Op::add1 && op <= Op::add3) return add(b, c << (u32(op) - u32(Op::add1) + 1), 0);
    if (op >= Op::sub1 && op <= Op::sub3) return sub(b, c << (u32(op) - u32(Op::sub1) + 1), 0);
    switch (op) {
    case Op::add: return add(b, c, 0);
    case Op::adc: return add(b, c, carry);
    case Op::sub: return sub(b, c, 0);
    case Op::sbc: return sub(b, c, carry);
    case Op::and_: return logic(b & c);
    case Op::or_: return logic(b | c);
    case Op::bic: return logic(b & ~c);
    case Op::xor_: return logic(b ^ c);
    case Op::max: return pick(b, c, s32(c) >= s32(b));
    case Op::min: return pick(b, c, s32(c) <= s32(b));
    case Op::mov: return logic(c);
    case Op::tst: return test(logic(b & c));
    case Op::cmp: return test(sub(b, c, 0));
    case Op::rcmp: return test(sub(c, b, 0));
    case Op::rsub: return sub(c, b, 0);
    case Op::bset: return logic(b | 1u << n);
    case Op::bclr: return logic(b & ~(1u << n));
    case Op::btst: return test(logic(b & 1u << n));
    case Op::bxor: return logic(b ^ 1u << n);
    case Op::bmsk: return logic(b & ((2u << n) - 1));
    case Op::asl: return moved(b << n, n && bit(b, 32 - int(n)));
    case Op::lsr: return moved(b >> n, out);
    case Op::asr: return moved(u32(s32(b) >> n), out);
    case Op::ror: return moved(std::rotr(b, int(n)), out);
    case Op::adds: return saturated(s64{s32(b)} + s32(c));
    case Op::subs: return saturated(s64{s32(b)} - s32(c));
    case Op::divaw: return {b == 0 ? 0 : ((b << 1) - c) >> 31 ? b << 1 : ((b << 1) - c) | 1, 0, Z | N | C | V};
    case Op::asls: return saturated(scaled(b, s32(c)));
    case Op::asrs: return saturated(scaled(b, -s64{s32(c)}));
    case Op::asl1: return add(c, c, 0);
    case Op::asr1: return moved(u32(s32(c) >> 1), c & 1);
    case Op::lsr1: return moved(c >> 1, c & 1);
    case Op::ror1: return moved(std::rotr(c, 1), c & 1);
    case Op::rrc: return moved(c >> 1 | carry << 31, c & 1);
    case Op::rlc: return add(c, c, carry);
    case Op::sexb: return logic(sext(c, 8));
    case Op::sexw: return logic(sext(c, 16));
    case Op::extb: return logic(c & 0xFF);
    case Op::extw: return logic(c & 0xFFFF);
    case Op::abs: return bit(c, 31) ? sub(0, c, 0) : Result{c, zn(c), Z | N | C | V};
    case Op::not_: return logic(~c);
    case Op::swap: return logic(c << 16 | c >> 16);
    case Op::norm: return Result{norm(c), zn(c), Z | N};
    case Op::normw: return Result{norm(sext(c, 16)) - 16, zn(sext(c, 16)), Z | N};
    case Op::sat16: return saturated(s32(c), 16);
    case Op::rnd16: return saturated((s64{s32(c)} + 0x8000) >> 16, 16);
    case Op::abssw: return signed_as(saturated(std::abs(s64{s16(c)}), 16), sext(c, 16));
    case Op::negsw: return saturated(-s64{s16(c)}, 16);
    case Op::negs: return saturated(-s64{s32(c)});
    case Op::abss: return signed_as(saturated(std::abs(s64{s32(c)})), c);
    default:
        if (u32(op) >> 8 != 4) return {.kept = false, .known = false};
        if (u32(op) & 0x80) return {~c, zn(~c) | C, Z | N | C, op != Op::zero_operand, false};
        return {.flags = zn(b ^ c), .mask = Z | N, .kept = false, .known = false};
    }
}
}  // namespace alu

template <class M>
struct Cpu : State {
    Cpu(M &m, Timing &t, const State &start) : State{start}, memory{m}, timing{t}, count_zero{0 - u64{start.count0}} {}

    Cpu(const Cpu &) = delete;
    std::map<u32, u32> auxiliary{{0x11, 0}, {0x48, 0}};

    u64 cycle() const { return slots + drift; }

    void step() {
        const u32 requests = (memory.requests | 1u << irq_hint | (timer() ? 8u : 0u)) & ~7u;
        const u32 level2 = status32 & E2 ? requests & irq_lev : 0;
        const u32 level1 = status32 & E1 ? requests & ~irq_lev : 0;
        if (slots >= unguarded) {
            if (level2 || level1) slots += pc + width(half(pc)) == lp_end && across(pc);
            if (erring) return interrupt(2, 2);
            if (level2) return interrupt(u32(std::bit_width(level2)) - 1, 2);
            if (level1) return interrupt(u32(std::bit_width(level1)) - 1, 1);
        }
        if (asleep) return advance();
        const u32 first = half(pc), w = width(first);
        fetch(pc, w == 2);
        if (w == 4) fetch(pc + 2);
        if (!dual(Op{first >> 11 << 8 | bits(first, 5, 0)})) wait(mac_flags);
        slot = std::exchange(delayed, std::nullopt);
        following = slot ? slot->target : pc + w;
        immediate.reset();
        redirected = false;
        bool round = pc + w == lp_end && count_down();
        if (w == 4) wide(first << 16 | half(pc + 2));
        else narrow(first);
        if (slot) slots += u64(s64{slot->dead - (immediate && slot->dead > 0 ? 1 : 0)});
        if (immediate) {
            advance();
            round |= following == lp_end && count_down();
        }
        if (!redirected) pc = following;
        if (round && across(pc)) ++slots;
        advance();
    }

  private:
    M &memory;
    Timing &timing;

    struct Branch {
        u32 target;
        bool compared;
        s32 dead;
    };

    u32 following{};
    std::optional<Branch> delayed, slot;
    std::optional<u32> immediate;
    bool redirected{}, asleep{}, erring{};
    u64 unguarded{};

    void guard(u64 n) { unguarded = std::max(unguarded, slots + n); }

    struct Late {
        u32 *to, value;
        u64 due, from;
        bool loaded, landed = false;
        u32 was;
    };

    std::vector<Late> unsettled;

    Late &arrive(u32 &to, u32 value, u64 after) {
        return unsettled.emplace_back(&to, value, slots + after, slots, false);
    }

    void redirect(u64 after) {
        for (Late &w : unsettled) {
            if (!w.loaded) continue;
            if (w.landed) *w.to = w.was, w.landed = false;
            w.due = (slots <= w.due ? w.from + 10 : slots) + after;
        }
    }

    void advance() {
        ++slots;
        if (unsettled.empty()) return;
        for (Late &w : unsettled)
            if (!w.landed && (w.due < slots || status32 & H)) w.was = std::exchange(*w.to, w.value), w.landed = true;
        std::erase_if(unsettled, [&](const Late &w) { return w.landed && !(w.loaded && slots <= w.due + 2); });
    }

    bool count_down() {
        if (--r[lp_count] == 0 || slot) return false;
        following = lp_start;
        return true;
    }

    u64 slots{}, drift{};
    std::array<u64, 64> ready{};
    u64 mac_flags{}, flagged{};

    void wait(u64 until) {
        while (cycle() < until)
            if (unsettled.empty()) slots = until - drift;
            else advance();
    }

    void hold(u64 until) { drift = until > cycle() ? until - slots : drift; }

    void fetch(u32 word, bool narrow = false) { wait(cycle() + timing.fetch(word, cycle(), narrow)); }

    void interrupt(u32 vector, u32 level) {
        redirect(2);
        drift += asleep ? 3 : 2;
        r[level == 1 ? ilink1 : ilink2] = pc;
        (level == 1 ? status32_l1 : status32_l2) = status32;
        status32 &= level == 1 ? ~E1 : ~(E1 | E2);
        irq_lv12 |= level;
        pc = int_vector_base + 8 * vector;
        asleep = erring = false;
    }

    u64 count_zero;

    bool timer() const { return bit(control0, 0) && s64(cycle() - count_zero) >= s64(limit0) + 2; }

    u32 counted(u64 cycle) const { return u32((cycle - count_zero) % (u64{limit0} + 1)); }

    void halt(u32 why) {
        status32 |= H, debug |= why;
        const u32 shown = counted(cycle() + 3);
        count0 = shown == limit0 ? 0 : shown;
    }

    u32 half(u32 at) { return memory.load(at, 2); }

    static u32 word(u32 at) { return at >> 2 << 2; }

    static u32 width(u32 first) { return first >> 11 < 0x08 ? 4 : 2; }

    bool across(u32 at) { return bit(at, 1) && width(half(at)) == 4; }

    u32 long_immediate() {
        if (!immediate) {
            const u32 from = slot && slot->compared ? pc + width(half(pc)) : std::exchange(following, following + 4);
            fetch(from), fetch(from + 2);
            immediate = half(from) << 16 | half(from + 2);
        }
        return *immediate;
    }

    static u32 cell(u32 n) { return n < 32 || n >= acc1 ? n : bit(n, bit(n, 4) ? 2 : 3) ? 40 : 32; }

    enum class Way { alu, early, late };
    std::array<u32, 2> glimpse{};

    u32 read(u32 n) { return wait(ready[n]), r[n]; }

    u32 get(u32 n, Way way = Way::alu) {
        if (n == limm) return long_immediate();
        if (n == pcl) return word(pc);
        n = cell(n);
        n < 32 || n == mlo || n == mhi ? wait(ready[n]) : hold(ready[n]);
        const bool is_cell = n == 32 || n == 40;
        if (is_cell && way == Way::early) return glimpse[n == 40];
        u32 value = r[n];
        for (const Late &w : unsettled)
            if (w.to == &r[n] && w.due == slots) value = w.value;
        if (is_cell && way == Way::alu) glimpse[n == 40] = value;
        return value;
    }

    void set(u32 n, u32 value, bool loaded = false) {
        if (n < 32 || (loaded && n < mlo && n >= acc1)) {
            if (!loaded) wait(ready[n]);
            r[n] = value;
            for (Late &w : unsettled)
                if (w.loaded && w.due == slots + (immediate ? 4 : 3)) --w.due, ++drift;
        } else if (n < acc1 || n == lp_count) {
            if (!loaded && ready[cell(n)] > cycle()) wait(ready[cell(n)] - 1);
            Late &write = arrive(r[cell(n)], value, loaded ? 8 : 3);
            write.loaded = loaded;
            for (const Late &w : unsettled)
                if (w.to == write.to) write.due = std::max(write.due, w.due);
        } else if (n < mlo) arrive(r[n], value, 2);
    }

    void jump(u32 target, bool delay, bool link, bool taken = true, u32 bubbles = 1, u64 after = 0) {
        guard(1 + bubbles);
        if (link) wait(ready[blink]);
        if (!taken) return;
        redirect(after ? after : 5 - bubbles);
        const bool waits = delay && target != following;
        const s32 dead = s32(bubbles) + across(target) - waits - immediate.has_value();
        if (link) r[blink] = delay ? following + width(half(following)) : following;
        if (waits) delayed = Branch{target, bubbles == 2, dead};
        else slots += u64(s64{dead}), pc = target, redirected = true;
    }

    void branch(u32 cc, u32 offset, bool link = false) {
        if (cc) behind_flags();
        jump(word(pc) + offset, false, link, holds(cc));
    }

    static u32 address(u32 aa, u32 zz, u32 base, u32 offset) {
        return aa == 2 ? base : base + (aa == 3 ? offset << (zz == 0 ? 2 : zz == 2 ? 1 : 0) : offset);
    }

    void load_to(u32 a, u32 fb, u32 offset, u32 aa, u32 zz, bool extend = false, bool uncached = false) {
        const u32 base = get(fb, Way::early), at = address(aa, zz, base, offset);
        const u32 bytes = zz == 0 ? 4 : zz == 1 ? 1 : 2, aligned = at & ~(bytes - 1);
        if (a < 32) wait(ready[a]);
        wait(timing.admit(cycle()));
        ready[cell(a)] = timing.access(aligned, false, uncached, cycle()) + 3 + (a >= 32);
        const u32 value = memory.load(aligned, bytes);
        if (aa == 1 || aa == 2) set(fb, base + offset);
        set(a, extend ? sext(value, int(8 * bytes)) : value, true);
    }

    void store_from(u32 value, u32 fb, u32 offset, u32 aa, u32 zz, bool uncached = false) {
        const u32 base = get(fb, Way::early), at = address(aa, zz, base, offset);
        const u32 bytes = zz == 1 ? 1 : zz == 2 ? 2 : 4, aligned = at & ~(bytes - 1);
        wait(timing.admit(cycle()));
        timing.access(aligned, true, uncached, cycle());
        memory.store(aligned, bytes, std::rotl(value, int(8 * (at - aligned))));
        if (aa == 1 || aa == 2) set(fb, base + offset);
    }

    void multiply(u64 product) {
        wait(ready[mlo]);
        r[mlo] = u32(product), r[mhi] = u32(product >> 32);
        ready[mlo] = ready[mhi] = cycle() + 4;
    }

    void flags(const alu::Result &out) {
        flagged = cycle();
        status32 = (status32 & ~out.mask) | out.flags;
        if (out.saturated) macmode |= SAT * 0x21;
    }

    void behind_flags() { drift += flagged + 1 == cycle(); }

    void operate(Op op, u32 dest, u32 b, u32 c, bool f, bool taken = true) {
        const alu::Result out = alu::compute(op, b, c, status32 & C ? 1 : 0);
        const u32 n = cell(dest);
        if (taken && f) flags(out);
        if (taken && out.kept) set(dest, out.value);
        else if (out.kept && n < 32) wait(ready[n]);
        else if (out.kept && (n == 32 || n == lp_count) && ready[n] > cycle()) wait(ready[n] - 1);
        if (out.known) return;
        const bool consumed = u32(op) >> 8 == 4 && u32(op) & 0x80 && op != Op::zero_operand;
        if (!consumed && std::exchange(immediate, std::nullopt)) following -= 4;
        instruction_error(following);
    }

    void instruction_error(u32 return_to) { pc = slot ? slot->target : return_to, erring = redirected = true; }

    bool holds(u32 cc) const {
        const bool z = status32 & Z, n = status32 & N, c = status32 & C, v = status32 & V;
        const bool conditions[]{true,         z,      !z,     !n,          n,        c,      !c,      v, !v,
                                n == v && !z, n == v, n != v, z || n != v, !c && !z, c || z, !n && !z};
        if (cc < 0x10) return conditions[cc];
        constexpr u32 flags[]{SAT, ACC, 0, 0, ZERO, NEG, OVER, 0};
        const u32 asked = flags[(cc >> 1) & 7] * 0x21;
        return asked && bool(macmode & asked) != bool(cc & 1);
    }

    struct Fields { u32 a, b, c, h, u, o, z, x; };

    using Row = std::pair<std::string_view, void (*)(Cpu &, Fields)>;

    static Fields fields(std::string_view pattern, u32 op) {
        std::array<u32, 128> v{};
        for (u32 i = 0; i < 16; ++i) v[u8(pattern[i])] = v[u8(pattern[i])] << 1 | bit(op, int(15 - i));
        const int s = int(std::ranges::count(pattern, 's'));
        const auto reg = [](u32 field) { return field > 3 ? field + 8 : field; };
        return {reg(v['a']), reg(v['b']), reg(v['c']), v['H'] << 3 | v['h'], s ? sext(v['s'], s) : v['u'],
                v['o'],      v['z'],      v['x']};
    }

    static bool fits(std::string_view pattern, u32 op) {
        for (u32 i = 0; i < 16; ++i)
            if (pattern[i] == char('0' + !bit(op, int(15 - i)))) return false;
        return true;
    }

    void narrow(u32 op) {
        using enum Op;
        using F = Fields;
        static constexpr Op arith[]{add, sub, asl, asr}, shifts[]{asl, lsr, asr, sub, bset, bclr, bmsk, btst};
        // clang-format off
        static constexpr Op general[32]{
            Op{}, Op{}, sub,  Op{}, and_, or_,  bic,  xor_, Op{}, Op{}, Op{}, tst,  Op{}, sexb, sexw, extb,
            extw, abs,  not_, sub,  add1, add2, add3, Op{}, asl,  lsr,  asr,  asl1, asr1, lsr1, Op{}, Op{}};
        // clang-format on
        static constexpr u32 conditions[]{0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x06, 0x05, 0x0E};
        static constexpr Row rows[]{
            {"01100bbbccc11aaa", [](Cpu &m, F f) { m.operate(add, f.a, m.read(f.b), m.read(f.c), false); }},
            {"01100bbbccczzaaa", [](Cpu &m, F f) { m.load_to(f.a, f.b, m.read(f.c), 0, f.z); }},
            {"01101bbbcccoouuu", [](Cpu &m, F f) { m.operate(arith[f.o], f.c, m.read(f.b), f.u, false); }},
            {"01110bbbhhh00HHH",
             [](Cpu &m, F f) {
                 const u32 b = m.read(f.b);
                 m.operate(add, f.b, b, m.get(f.h), false);
             }},
            {"01110bbbhhh01HHH", [](Cpu &m, F f) { m.set(f.b, m.get(f.h)); }},
            {"01110bbbhhh10HHH",
             [](Cpu &m, F f) {
                 const u32 b = m.read(f.b);
                 m.operate(cmp, f.b, b, m.get(f.h), true);
             }},
            {"01110bbbhhh11HHH", [](Cpu &m, F f) { m.set(f.h, m.read(f.b)); }},
            {"01111bbb0zx00000", [](Cpu &m, F f) { m.jump(m.read(f.b), f.x, f.z); }},
            {"01111bbb11000000",
             [](Cpu &m, F f) { m.holds(2) ? m.operate(sub, f.b, m.read(f.b), m.read(f.b), 0) : void(); }},
            {"0111100011100000", [](Cpu &, F) {}},
            {"011111zz11100000",
             [](Cpu &m, F f) { m.jump(m.read(blink), f.z == 3, false, f.z > 1 || m.holds(f.z + 1)); }},
            {"01111bbbccc01100", [](Cpu &m, F f) { m.multiply(u64(s64{s32(m.read(f.b))} * s32(m.read(f.c)))); }},
            {"01111------11111", [](Cpu &m, F) { m.halt(BH), m.redirected = true; }},
            {"01111bbbccc10011", [](Cpu &m, F f) { m.operate(sub, f.b, 0, m.read(f.c), false); }},
            {"01111bbbcccooooo",
             [](Cpu &m, F f) {
                 if (general[f.o] == Op{}) return m.instruction_error(m.following);
                 m.operate(general[f.o], f.b, m.read(f.b), m.read(f.c), f.o == 0x0B);
             }},
            {"100zzbbbcccuuuuu", [](Cpu &m, F f) { m.load_to(f.c, f.b, f.u, 3, std::min(f.z, 2u), f.z == 3); }},
            {"10111bbbooouuuuu", [](Cpu &m, F f) { m.operate(shifts[f.o], f.b, m.read(f.b), f.u, f.o == 7); }},
            {"101zzbbbcccuuuuu", [](Cpu &m, F f) { m.store_from(m.read(f.c), f.b, f.u, 3, f.z); }},
            {"11000bbb00zuuuuu", [](Cpu &m, F f) { m.load_to(f.b, sp, f.u << 2, 0, f.z); }},
            {"11000bbb01zuuuuu", [](Cpu &m, F f) { m.store_from(m.read(f.b), sp, f.u << 2, 0, f.z); }},
            {"11000bbb100uuuuu", [](Cpu &m, F f) { m.operate(add, f.b, m.read(sp), f.u << 2, false); }},
            {"11000000101uuuuu", [](Cpu &m, F f) { m.operate(add, sp, m.read(sp), f.u << 2, false); }},
            {"11000---101uuuuu", [](Cpu &m, F f) { m.operate(sub, sp, m.read(sp), f.u << 2, false); }},
            {"11000bbb1100----", [](Cpu &m, F f) { m.load_to(f.b, sp, 4, 2, 0); }},
            {"11000---1101---x", [](Cpu &m, F f) { m.load_to(f.x ? u32{blink} : 0, sp, 4, 2, 0); }},
            {"11000---1111---1", [](Cpu &m, F) { m.store_from(m.read(blink), sp, u32(-4), 1, 0); }},
            {"11000bbb111-----", [](Cpu &m, F f) { m.store_from(m.read(f.b), sp, u32(-4), 1, 0); }},
            {"1100111sssssssss", [](Cpu &m, F f) { m.operate(add, 0, m.read(gp), f.u << 2, false); }},
            {"11001zzsssssssss", [](Cpu &m, F f) { m.load_to(0, gp, f.u, 3, f.z); }},
            {"11010bbbuuuuuuuu", [](Cpu &m, F f) { m.load_to(f.b, pcl, f.u, 3, 0); }},
            {"11011bbbuuuuuuuu", [](Cpu &m, F f) { m.set(f.b, f.u); }},
            {"11100bbb0uuuuuuu", [](Cpu &m, F f) { m.operate(add, f.b, m.read(f.b), f.u, false); }},
            {"11100bbb1uuuuuuu", [](Cpu &m, F f) { m.operate(cmp, f.b, m.read(f.b), f.u, true); }},
            {"11101bbbxsssssss",
             [](Cpu &m, F f) {
                 ++m.drift;
                 m.jump(word(m.pc) + (f.u << 1), false, false, (m.read(f.b) == 0) != f.x, 2);
             }},
            {"1111011ooossssss", [](Cpu &m, F f) { m.branch(conditions[f.o], f.u << 1); }},
            {"11110zzsssssssss", [](Cpu &m, F f) { m.branch(f.z, f.u << 1); }},
            {"11111sssssssssss", [](Cpu &m, F f) { m.branch(0, f.u << 2, true); }},
        };
        static_assert(std::ranges::all_of(rows, [](const Row &row) { return row.first.size() == 16; }));
        static const auto decoded = [] {
            std::vector<std::pair<const Row *, Fields>> all(1 << 16);
            for (u32 n = 0; n < all.size(); ++n) {
                const auto matches = [&](const Row &row) { return fits(row.first, n); };
                if (const Row *row = std::ranges::find_if(rows, matches); row != std::end(rows))
                    all[n] = {row, fields(row->first, n)};
            }
            return all;
        }();
        const auto &[row, f] = decoded[op];
        row ? row->second(*this, f) : instruction_error(following);
    }

    void wide(u32 word) {
        switch (word >> 27) {
        case 0: return far_branch(word);
        case 1: return bit(word, 16) ? compare_branch(word) : far_branch(word);
        case 2:
            return load_to(bits(word, 5, 0), field_b(word), offset9(word), bits(word, 10, 9), bits(word, 8, 7),
                           bit(word, 6), bit(word, 11));
        case 3: {
            const u32 fc = bits(word, 11, 6);
            return store_from(get(fc, bit(word, 18) && cell(fc) == 32 ? Way::early : Way::late), field_b(word),
                              offset9(word), bits(word, 4, 3), bits(word, 2, 1), bit(word, 5));
        }
        default: return general(word);
        }
    }

    static u32 field_b(u32 word) { return bits(word, 14, 12) << 3 | bits(word, 26, 24); }

    static u32 offset9(u32 word) { return sext(bits(word, 15, 15) << 8 | bits(word, 23, 16), 9); }

    void general(u32 word) {
        const u32 p = bits(word, 23, 22), sub = bits(word, 21, 16);
        if (word >> 27 == 4 && sub >> 3 == 6 && (sub & 6) != 6 && (sub & 7) != 1)
            return load_to(bits(word, 5, 0), field_b(word), get(bits(word, 11, 6), Way::early), p, sub >> 1 & 3,
                           bit(sub, 0), bit(word, 15));
        const bool single = sub == 0x2F;
        const Op op{word >> 27 << 8 | (single ? 0x80 | bits(word, 5, 0) : sub)};
        const u32 fb = field_b(word), fc = bits(word, 11, 6), fa = bits(word, 5, 0);
        const bool f = bit(word, 15) && !(op == Op::zero_operand && (fb == 2 || fb == 1));
        const bool c_is_register = p == 0 || (p == 3 && !bit(word, 5));
        const bool early = op >= Op::j && op <= Op::sr;
        const u32 c = c_is_register ? get(fc, early ? Way::early : Way::alu) : p == 2 ? sext(fc | fa << 6, 12) : fc;
        auto skipped = [&] { return p == 3 && !holds(bits(word, 4, 0)); };
        if (op >= Op::j && op <= Op::jl_d) {
            const bool link = op >= Op::jl, delay = (op == Op::j_d || op == Op::jl_d) && !immediate, taken = !skipped();
            if (taken && f && !link && (fc == ilink1 || fc == ilink2)) {
                status32 = fc == ilink1 ? status32_l1 : status32_l2;
                jump(c, delay, false, true, 2, 5);
                return void((++unguarded, --drift));
            }
            if (taken && f && !link) status32 = (status32 & ~(Z | N)) | alu::zn(r[fb] ^ c);
            if (p == 3) behind_flags();
            return jump(c, delay, link, taken);
        }
        if (op == Op::zero_operand && fb == 1 && p < 2) return void(asleep = true);
        if (op == Op::lp) {
            const u32 end = this->word(pc) + (c << 1);
            if (skipped()) return jump(end, false, false);
            arrive(lp_start, following, 1);
            arrive(lp_end, end, 1);
            return;
        }
        if (op == Op::flag) {
            if (skipped()) return;
            if (c & H) halt(SH);
            else status32 = c & (Z | N | C | V | E1 | E2), guard(3);
            return;
        }
        if (op == Op::sr) return aux(c & 0xFFFF, get(fb, Way::early));
        if (op == Op::lr) {
            if (fb == limm) long_immediate();
            if (fb < 32) wait(ready[fb]);
            return set(fb, aux(c & 0xFFFF));
        }
        const bool unary = single || op == Op::mov;
        const u32 b = unary ? u32{} : get(fb);
        const bool taken = !skipped();
        if (op == Op::mul64 || op == Op::mulu64) {
            if (taken) multiply(op == Op::mul64 ? u64(s64{s32(b)} * s32(c)) : u64{b} * c);
            return;
        }
        if (dual(op)) {
            if (taken) dual_word(op, p < 2 ? fa : fb, b, c, f);
            else if (f) mac_flags = cycle() + 3;
            return;
        }
        operate(op, p < 2 && !unary ? fa : fb, b, c, f, taken);
        if (single && word >> 27 == 4 && p >= 2) instruction_error(following);
        if (op == Op::zero_operand && fb == 1) ++drift;
    }

    void far_branch(u32 word) {
        const bool link = word >> 27 == 1, far = bit(word, link ? 17 : 16);
        const u32 low = link ? bits(word, 26, 18) << 2 : bits(word, 26, 17) << 1;
        const u32 offset = far ? sext(bits(word, 3, 0) << 21 | bits(word, 15, 6) << 11 | low, 25)
                               : sext(bits(word, 15, 6) << 11 | low, 21);
        const bool taken = far || holds(bits(word, 4, 0));
        if (!far) behind_flags();
        jump(this->word(pc) + offset, bit(word, 5), link, taken);
    }

    void compare_branch(u32 word) {
        const u32 b = get(field_b(word)), c = bit(word, 4) ? bits(word, 11, 6) : get(bits(word, 11, 6));
        const u32 cc = bits(word, 3, 0), n = c & 31;
        const bool taken[]{b == c, b != c, s32(b) < s32(c), s32(b) >= s32(c),
                           b < c,  b >= c, !bit(b, int(n)), bit(b, int(n))};
        if (cc >= 6 && cc < 14) return instruction_error(following);
        ++drift;
        const u32 offset = sext(bits(word, 15, 15) << 8 | bits(word, 23, 17) << 1, 9);
        jump(this->word(pc) + offset, bit(word, 5), false, taken[cc < 6 ? cc : cc - 8], 2);
    }

    static bool dual(Op op) {
        return op == Op::muldw || op == Op::mulrdw || op == Op::macdw || op == Op::macudw || op == Op::msubdw;
    }

    s64 accumulator(bool upper) const {
        const u32 guard = upper ? xmac0 >> 16 : xmac0;
        return s64(u64{sext(guard, 8)} << 32 | (upper ? xmac1 : xmac2));
    }

    void accumulator(bool upper, s64 value) {
        const int half = upper ? 16 : 0;
        (upper ? xmac1 : xmac2) = u32(value);
        xmac0 = (xmac0 & ~(0xFFFFu << half)) | (sext(u32(value >> 32), 8) & 0xFFFF) << half;
    }

    void dual_word(Op op, u32 dest, u32 b, u32 c, bool f) {
        const bool is_unsigned = op == Op::macudw, fresh = op == Op::muldw || op == Op::mulrdw;
        auto within = [&](s64 v, int bits) {
            const s64 top = s64{1} << (is_unsigned ? bits : bits - 1);
            return std::clamp(v, is_unsigned ? 0 : -top, top - 1);
        };
        auto wrapped = [&](s64 v) { return is_unsigned ? v & 0xFF'FFFF'FFFF : s64(u64(v) << 24) >> 24; };
        const bool kept = dest != limm;
        u32 value = 0, flags = macmode & ~((OVER | NEG | ZERO) * 0x21);
        for (const bool upper : {true, false}) {
            const int half = upper ? 16 : 0, up = upper ? 5 : 0;
            const s64 x = is_unsigned ? s64{u16(b >> half)} : s64{s16(b >> half)},
                      y = is_unsigned ? s64{u16(c >> half)} : s64{s16(c >> half)};
            s64 product = macmode & FRACTIONAL ? wrapped(is_unsigned ? u32(x * y << 1) : x * y << 1) : x * y;
            const s64 held = fresh ? 0 : wrapped(accumulator(upper));
            bool over = false, stuck;
            if (macmode & SATURATE_32) over = product != within(product, 32), product = within(product, 32);
            const s64 exact = op == Op::msubdw ? held - product : held + product;
            s64 next = within(exact, 40);
            if (macmode & SATURATE_32) {
                const s64 word =
                    (is_unsigned ? s64{u32(held)} : s64{s32(held)}) + (op == Op::msubdw ? -product : product);
                stuck = word != within(word, 32);
                next = stuck ? wrapped(s32(within(word, 32))) : wrapped(exact);
            } else stuck = next != exact;
            s64 result = stuck && next < 0 && wrapped(exact) >= 0 ? 0x7FFF0000 : within(next, 32);
            over |= stuck || result != next;
            if (op == Op::mulrdw) result = within(result + 0x8000, 32) & ~s64{0xFFFF};
            u32 part = u32(result >> 16);
            if (!(macmode & HIGH_HALF) && !stuck) {
                over |= next != within(next, 16);
                part = next != within(next, 16) ? u32(within(next, 16)) : u32(next) + (op == Op::mulrdw ? 0x8000 : 0);
            }
            value |= (part & 0xFFFF) << half;
            flags |= (over ? OVER << up : 0) | (next < 0 ? NEG << up : 0) | (next == 0 ? ZERO << up : 0) |
                     (stuck ? ACC << up : 0);
            flags |= over && kept ? SAT << up : 0;
            accumulator(upper, next);
            r[upper ? acc1 : acc2] = u32(result);
        }
        ready[acc1] = ready[acc2] = cycle() + 3;
        if (f) macmode = flags, mac_flags = cycle() + 3;
        if (f && kept) {
            const u32 now =
                (value == 0 ? Z : 0) | (!is_unsigned && value & 0x8000'8000 ? N : 0) | (flags & (OVER * 0x21) ? V : 0);
            status32 = (status32 & ~(Z | N | V)) | now;
        }
        set(dest, value);
    }

    // clang-format off
    static constexpr std::array<u32, 57> build_configuration{0, 1, 0, 1, 1, 0xFC0001, 0x0D010112, 0x0D010112,
        0x0D010112, 0x0D010112, 0x0D010112, 0x0D010112, 0x0D010112, 0x0D010112, 0x6201, 0, 0, 0x302, 0, 0x16401, 0,
        0x1203, 0x203, 1, 1, 2, 0, 2};
    // clang-format on

    u32 aux(u32 number) {
        if (number == 0x00)
            return (status32 & (Z | N | C | V)) << 20 | (status32 & (E1 | E2)) << 25 | (aux(0x06) >> 2 & 0xFFFFFF);
        if (number == 0x06) return pc + width(half(pc));
        if (number == 0x21) return counted(cycle() + 2);
        for (const Word &s : state_words)
            if (s.number == number) {
                u32 value = this->*s.field;
                for (const Late &w : unsettled) value = w.to == &(this->*s.field) ? w.value : value;
                return value;
            }
        if (number - 0x64 < build_configuration.size()) return build_configuration[number - 0x64];
        if (number >= 0x1B && number <= 0x1D) drift += 3;
        if (number >= 0x59 && number <= 0x5B) drift += 4;
        return auxiliary.contains(number) ? auxiliary.at(number) : 0x0D010112;
    }

    void aux(u32 number, u32 value) {
        switch (number) {
        case 0x02: return void(arrive(lp_start, value & ~1u, 2));
        case 0x03: return void(arrive(lp_end, value & ~1u, 2));
        case 0x10: return timing.code.clear();
        case 0x12: return void(arrive(r[mhi], value, 2));
        case 0x19: return timing.code.drop(value);
        case 0x21: return void(count_zero = cycle() + 3 - value);
        case 0x41:
            return void(macmode = value & 3 ? macmode & ~(value & 1 ? ACC * 0x21 : 0) & ~(value & 2 ? SAT * 0x21 : 0)
                                            : value & 0x7FFC);
        case 0x43: return void(irq_lv12 &= ~value);
        case 0x44: return void(xmac0 = (sext(value >> 16, 8) & 0xFFFF) << 16 | (sext(value, 8) & 0xFFFF));
        case 0x45: return accumulator(true, s32(value));
        case 0x46: return accumulator(false, s32(value));
        case 0x47: return timing.data.clear();
        case 0x4A: return wait(timing.completed), timing.data.drop(value);
        case 0x4B: return void(timing.data.clean());
        case 0x201: return void(arrive(irq_hint, value & 0x1F, 3));
        }
        for (const Word &s : state_words)
            if (s.number == number && s.mask) return void(this->*s.field = value & s.mask);
        if (const auto it = auxiliary.find(number); it != auxiliary.end()) it->second = value;
    }
};

}  // namespace leapster
