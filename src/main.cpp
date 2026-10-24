#include <SDL2/SDL.h>

#include <algorithm>
#include <fstream>
#include <map>
#include <print>
#include <string>

#include "leapster.h"

using namespace leapster;

namespace {

constexpr const char *usage = "usage: leapster BIOS CART [--eeprom FILE] [--save FILE] [--flash FILE] [--scale N]";

constexpr u8 calibration[]{0x00, 0x01, 0x00, 0x00, 0x40, 0x06, 0x40, 0x06, 0x52, 0x2B, 0x06, 0x00, 0xF8, 0xB0,
                           0xF2, 0xFF, 0xB8, 0xD9, 0xFF, 0xFF, 0xB8, 0x5D, 0x4D, 0x2E, 0x68, 0x10, 0x00, 0x00,
                           0x38, 0xDD, 0x0D, 0x00, 0x58, 0xAD, 0x26, 0xF7, 0x05, 0x00, 0x00, 0x00, 0x76};

std::vector<u8> read(const std::string &path) {
    std::ifstream in{path, std::ios::binary};
    if (!in) throw std::runtime_error{"cannot open " + path};
    return {std::istreambuf_iterator<char>{in}, {}};
}

void write(const std::string &path, const std::vector<u8> &bytes) {
    std::ofstream{path, std::ios::binary}.write(reinterpret_cast<const char *>(bytes.data()), std::ssize(bytes));
}

struct OutputStage {
    static constexpr double gain = 2.1565e-02;
    static constexpr double sections[3][3]{// b1, a1, a2 (b0 = b2 = a0 = 1)
                                           {-9.122555018165416e-01, -1.6544416859072675e+00, 7.473407005845932e-01},
                                           {-1.5033339309766456e+00, -1.6819320713852317e+00, 8.745731889355856e-01},
                                           {-1.6163860839782422e+00, -1.715375001187219e+00, 9.651086154754862e-01}};
    double z[3][2]{};

    s16 operator()(s16 in) {
        double y = in * gain;
        for (u32 k = 0; k < 3; ++k) {
            const auto &[b1, a1, a2] = sections[k];
            const double w = y - a1 * z[k][0] - a2 * z[k][1];
            y = w + b1 * z[k][0] + z[k][1];
            z[k][1] = z[k][0], z[k][0] = w;
        }
        return s16(std::clamp(y, -32768.0, 32767.0));
    }
};

u32 button(SDL_Keycode key) {
    switch (key) {
    case SDLK_UP: return 1u << 31;
    case SDLK_DOWN: return 1u << 8;
    case SDLK_LEFT: return 1u << 9;
    case SDLK_RIGHT: return 1u << 7;
    case SDLK_z: return 1u << 14;
    case SDLK_x: return 1u << 13;
    case SDLK_h: return 1u << 28;
    case SDLK_RETURN: return 1u << 29;
    case SDLK_p: return 1u << 26;
    default: return 0;
    }
}

int run(int argc, char **argv) {
    if (argc < 3) return std::println(stderr, "{}", usage), 2;
    std::map<std::string, std::string> option;
    for (int i = 3; i < argc; i += 2) {
        const std::string name = argv[i];
        if (i + 1 >= argc || (name != "--eeprom" && name != "--save" && name != "--flash" && name != "--scale"))
            return std::println(stderr, "{}", usage), 2;
        option[name.substr(2)] = argv[i + 1];
    }
    const int scale = option.contains("scale") ? std::stoi(option["scale"]) : 4;
    const std::string &eeprom = option["eeprom"], &save = option["save"], &flash = option["flash"];
    Leapster console{read(argv[1]), read(argv[2])};
    Board &board = console.board;
    board.eeprom[1].assign(256, 0xFF);
    std::ranges::copy(calibration, board.eeprom[1].begin());
    std::ranges::copy(calibration, board.eeprom[1].begin() + 0x30);
    if (!eeprom.empty() && std::ifstream{eeprom}) board.eeprom[1] = read(eeprom);
    if (!save.empty() && std::ifstream{save}) board.eeprom[0] = read(save);
    if (!flash.empty() && std::ifstream{flash}) board.flash = read(flash), board.flash.resize(0x80000, 0xFF);
    board.audio.recording = true;

    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) != 0) throw std::runtime_error{SDL_GetError()};
    SDL_Window *window =
        SDL_CreateWindow("Leapster", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, 160 * scale, 160 * scale, 0);
    SDL_Renderer *renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED);
    SDL_Texture *panel = SDL_CreateTexture(renderer, SDL_PIXELFORMAT_RGB444, SDL_TEXTUREACCESS_STREAMING, 160, 160);
    SDL_AudioSpec want{}, got{};
    want.freq = 48000, want.format = AUDIO_S16SYS, want.channels = 1, want.samples = 1024;
    const SDL_AudioDeviceID speaker = SDL_OpenAudioDevice(nullptr, 0, &want, &got, 0);
    SDL_PauseAudioDevice(speaker, 0);

    constexpr u64 setpoint = 48000 * 90 / 1000, slice = 96'000'000 / 30;
    const u64 wall_hz = SDL_GetPerformanceFrequency();
    u64 wall_base = SDL_GetPerformanceCounter(), clock_base = board.now, presented = 0;
    OutputStage stage;
    for (bool running = true; running;) {
        for (SDL_Event e; SDL_PollEvent(&e);) {
            if (e.type == SDL_QUIT || (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_ESCAPE)) running = false;
            if (e.type == SDL_KEYDOWN) board.buttons |= button(e.key.keysym.sym);
            if (e.type == SDL_KEYUP) board.buttons &= ~button(e.key.keysym.sym);
        }
        int x, y;
        const bool down = SDL_GetMouseState(&x, &y) & SDL_BUTTON(SDL_BUTTON_LEFT);
        board.touch.reset();
        if (down) board.stylus(u32(std::clamp(x / scale, 0, 159)), u32(std::clamp(y / scale, 0, 159)));

        u64 target = board.now;
        if (speaker) {
            const u64 queued = SDL_GetQueuedAudioSize(speaker) / sizeof(s16);
            if (queued < setpoint) target += (setpoint - queued) * sample_clocks;
        } else {
            const u64 owed = clock_base + u64(double(SDL_GetPerformanceCounter() - wall_base) / double(wall_hz) * 96e6);
            if (owed > board.now + 96'000'000) wall_base = SDL_GetPerformanceCounter(), clock_base = board.now;
            else if (owed > board.now) target = owed;
        }
        target = std::min(target, board.now + slice);
        if (target > board.now)
            while (board.now < target && !(console.cpu.status32 & H)) console.step();
        else SDL_Delay(1);
        std::vector<s16> &sound = board.samples();
        for (s16 &sample : sound) sample = stage(sample);
        SDL_QueueAudio(speaker, sound.data(), u32(sound.size() * sizeof(s16)));
        sound.clear();

        const u64 now = SDL_GetPerformanceCounter();
        if (now - presented >= wall_hz / 60) {
            presented = now;
            const auto pixels = board.panel();
            SDL_UpdateTexture(panel, nullptr, pixels.data(), 160 * sizeof(u16));
            SDL_RenderCopy(renderer, panel, nullptr, nullptr);
            SDL_RenderPresent(renderer);
        }
    }
    if (!eeprom.empty()) write(eeprom, board.eeprom[1]);
    if (!save.empty() && !board.eeprom[0].empty()) write(save, board.eeprom[0]);
    if (!flash.empty()) write(flash, board.flash);
    SDL_Quit();
    return 0;
}

}  // namespace

int main(int argc, char **argv) {
    try {
        return run(argc, argv);
    } catch (const std::exception &e) { return std::println(stderr, "leapster: {}", e.what()), 1; }
}
