// Servo rig firmware for a Raspberry Pi Pico or Pico 2 (Pico SDK).
// Seven SG92R servos on GP0..GP6, driven over USB serial with line commands:
//   PING                  -> OK servo-rig 1
//   SET <i> <us>          move servo i (0-6) to a pulse width, approaching from below
//   MOVE <us0> ... <us6>  move all seven (use - to leave one where it is)
//   OFF [i]               stop pulses (one servo or all) so they don't buzz or hum
//   GET                   -> OK <us0> ... <us6> (0 = off)
// Every command answers one line: "OK ..." when done (motion settled) or "ERR ...".
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

namespace {

constexpr int kNumServos = 7;
constexpr uint kPins[kNumServos] = {0, 1, 2, 3, 4, 5, 6};
constexpr int kUsMin = 500, kUsMax = 2500;
// Approach every target from below by this much so gear slack always sits on the same side.
constexpr int kOvershootUs = 60;
// SG92R: about 0.1 s per 60 degrees, roughly 0.15 ms per microsecond of pulse change.
constexpr float kSettleMsPerUs = 0.2f;
constexpr int kSettleMsMin = 150;
constexpr uint16_t kPeriodUs = 20000;

int current[kNumServos] = {};

void Write(int i, int us) {
    pwm_set_gpio_level(kPins[i], static_cast<uint16_t>(us));
    current[i] = us;
}

void Settle(int distance_us) {
    sleep_ms(std::max(kSettleMsMin, static_cast<int>(distance_us * kSettleMsPerUs)));
}

// targets[i] < 0 leaves servo i alone. Moves together, each finishing on an upward approach.
bool Move(const int* targets, char* error) {
    for (int i = 0; i < kNumServos; i++)
        if (targets[i] >= 0 && (targets[i] < kUsMin || targets[i] > kUsMax)) {
            sprintf(error, "servo %d to %d us out of range", i, targets[i]);
            return false;
        }
    // Phase 1: anything that must come down (or is off) goes below its target.
    int dist = 0;
    bool any_low = false;
    for (int i = 0; i < kNumServos; i++) {
        if (targets[i] < 0 || (current[i] != 0 && current[i] < targets[i])) continue;
        int low = std::max(kUsMin, targets[i] - kOvershootUs);
        dist = std::max(dist, std::abs((current[i] ? current[i] : kUsMax) - low));
        Write(i, low);
        any_low = true;
    }
    if (any_low) Settle(dist);
    // Phase 2: everything rises onto its target.
    dist = 0;
    for (int i = 0; i < kNumServos; i++) {
        if (targets[i] < 0) continue;
        dist = std::max(dist, std::abs(current[i] - targets[i]));
        Write(i, targets[i]);
    }
    Settle(dist);
    return true;
}

void Off(int only) {
    for (int i = 0; i < kNumServos; i++)
        if (only < 0 || only == i) Write(i, 0);
}

void Handle(char* line) {
    char* argv[1 + kNumServos + 1];
    int argc = 0;
    for (char* tok = strtok(line, " \t"); tok && argc < 1 + kNumServos + 1;
         tok = strtok(nullptr, " \t"))
        argv[argc++] = tok;
    if (argc == 0) return;
    for (char* c = argv[0]; *c; c++) *c = static_cast<char>(toupper(*c));
    char error[64] = "";
    int targets[kNumServos];
    std::fill(targets, targets + kNumServos, -1);

    if (!strcmp(argv[0], "PING") && argc == 1) {
        puts("OK servo-rig 1");
    } else if (!strcmp(argv[0], "SET") && argc == 3) {
        int i = atoi(argv[1]);
        if (i < 0 || i >= kNumServos) return (void)puts("ERR bad servo index");
        targets[i] = atoi(argv[2]);
        if (Move(targets, error)) puts("OK");
        else printf("ERR %s\n", error);
    } else if (!strcmp(argv[0], "MOVE") && argc == 1 + kNumServos) {
        for (int i = 0; i < kNumServos; i++)
            if (strcmp(argv[1 + i], "-")) targets[i] = atoi(argv[1 + i]);
        if (Move(targets, error)) puts("OK");
        else printf("ERR %s\n", error);
    } else if (!strcmp(argv[0], "OFF") && argc <= 2) {
        Off(argc == 2 ? atoi(argv[1]) : -1);
        puts("OK");
    } else if (!strcmp(argv[0], "GET") && argc == 1) {
        printf("OK");
        for (int us : current) printf(" %d", us);
        printf("\n");
    } else {
        puts("ERR unknown command");
    }
}

} // namespace

int main() {
    stdio_init_all();
    // 1 MHz PWM counter, so the level is the pulse width in microseconds.
    float div = static_cast<float>(clock_get_hz(clk_sys)) / 1e6f;
    for (uint pin : kPins) {
        gpio_set_function(pin, GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(pin);
        pwm_config cfg = pwm_get_default_config();
        pwm_config_set_clkdiv(&cfg, div);
        pwm_config_set_wrap(&cfg, kPeriodUs - 1);
        pwm_init(slice, &cfg, true);
        pwm_set_gpio_level(pin, 0);
    }

    char line[128];
    size_t len = 0;
    for (;;) {
        int c = getchar_timeout_us(100000);
        if (c == PICO_ERROR_TIMEOUT) continue;
        if (c == '\n' || c == '\r') {
            line[len] = '\0';
            Handle(line);
            len = 0;
        } else if (len < sizeof(line) - 1) {
            line[len++] = static_cast<char>(c);
        }
    }
}
