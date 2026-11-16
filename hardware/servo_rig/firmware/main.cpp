// Servo rig firmware for a Raspberry Pi Pico or Pico 2 (Pico SDK).
// Seven SG92R servos on GP0..GP6, driven over USB serial with line commands:
//   PING                  -> OK servo-rig 2
//   AT <i> <us>           declare where servo i physically is; starts holding it there
//   SET <i> <us>          ramp servo i (0-6) to a pulse width, finishing from below
//   MOVE <us0> ... <us6>  ramp all seven together (use - to leave one where it is)
//   SPEED <us_per_s>      ramp speed (default 150)
//   OFF [i]               stop pulses (one servo or all); positions are remembered
//   GET                   -> OK <us0> ... <us6> (0 = position unknown)
// Every command answers one line: "OK ..." when done (motion settled) or "ERR ...".
//
// A hobby servo jumps to any new pulse width at full speed and can't report where it is,
// so smooth motion has to come from here: every 20 ms each moving servo's pulse advances
// a little, with acceleration and deceleration. After power-up a servo won't move until
// AT says where it is, so it can never snap from an unknown position into a pot's end stop.
#include <algorithm>
#include <cctype>
#include <cmath>
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
constexpr uint16_t kPeriodUs = 20000;
constexpr int kTickMs = 20;           // one servo frame
constexpr float kAccel = 600.f;       // us/s^2: reaches full speed in a fraction of a second
constexpr int kSettleMs = 150;        // after arriving, before answering OK

float speed = 150.f;                  // us/s
float pos[kNumServos] = {};           // last pulse width (0 = unknown)
float vel[kNumServos] = {};
bool on[kNumServos] = {};

void Pulse(int i) {
    pwm_set_gpio_level(kPins[i], on[i] ? static_cast<uint16_t>(lroundf(pos[i])) : 0);
}

// Ramp every servo with a target >= 0 to it, all together, trapezoidal velocity.
void Ramp(const float* target) {
    const float dt = kTickMs / 1000.f;
    absolute_time_t next = get_absolute_time();
    for (;;) {
        bool moving = false;
        for (int i = 0; i < kNumServos; i++) {
            if (target[i] < 0) continue;
            float remaining = target[i] - pos[i];
            if (fabsf(remaining) < 0.5f && fabsf(vel[i]) < kAccel * dt) {
                pos[i] = target[i];
                vel[i] = 0.f;
                Pulse(i);
                continue;
            }
            moving = true;
            // Fastest speed that can still stop at the target, capped at the cruise speed.
            float vmax = std::min(speed, sqrtf(2.f * kAccel * fabsf(remaining)));
            float want = remaining > 0 ? vmax : -vmax;
            float dv = std::clamp(want - vel[i], -kAccel * dt, kAccel * dt);
            vel[i] += dv;
            float step = vel[i] * dt;
            pos[i] = fabsf(step) >= fabsf(remaining) ? target[i] : pos[i] + step;
            Pulse(i);
        }
        if (!moving) break;
        next = delayed_by_ms(next, kTickMs);
        sleep_until(next);
    }
    sleep_ms(kSettleMs);
}

// targets[i] < 0 leaves servo i alone. Moves together, each finishing on an upward approach.
bool Move(const int* targets, char* error) {
    for (int i = 0; i < kNumServos; i++) {
        if (targets[i] < 0) continue;
        if (targets[i] < kUsMin || targets[i] > kUsMax) {
            sprintf(error, "servo %d to %d us out of range", i, targets[i]);
            return false;
        }
        if (pos[i] == 0.f) {
            sprintf(error, "servo %d position unknown; send AT %d <us> first", i, i);
            return false;
        }
    }
    float low[kNumServos], final_[kNumServos];
    bool any_low = false;
    for (int i = 0; i < kNumServos; i++) {
        low[i] = final_[i] = -1.f;
        if (targets[i] < 0) continue;
        on[i] = true;
        final_[i] = static_cast<float>(targets[i]);
        if (pos[i] >= targets[i]) {
            low[i] = static_cast<float>(std::max(kUsMin, targets[i] - kOvershootUs));
            any_low = true;
        }
    }
    if (any_low) Ramp(low);
    Ramp(final_);
    return true;
}

void Off(int only) {
    for (int i = 0; i < kNumServos; i++)
        if (only < 0 || only == i) {
            on[i] = false;
            vel[i] = 0.f;
            Pulse(i);
        }
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
        puts("OK servo-rig 2");
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
    } else if (!strcmp(argv[0], "AT") && argc == 3) {
        int i = atoi(argv[1]), us = atoi(argv[2]);
        if (i < 0 || i >= kNumServos || us < kUsMin || us > kUsMax)
            return (void)puts("ERR bad servo index or pulse width");
        pos[i] = static_cast<float>(us);
        vel[i] = 0.f;
        on[i] = true;
        Pulse(i);
        puts("OK");
    } else if (!strcmp(argv[0], "SPEED") && argc == 2) {
        float v = static_cast<float>(atof(argv[1]));
        if (v < 10.f || v > 2000.f) return (void)puts("ERR speed must be 10-2000 us/s");
        speed = v;
        puts("OK");
    } else if (!strcmp(argv[0], "OFF") && argc <= 2) {
        Off(argc == 2 ? atoi(argv[1]) : -1);
        puts("OK");
    } else if (!strcmp(argv[0], "GET") && argc == 1) {
        printf("OK");
        for (float us : pos) printf(" %d", static_cast<int>(lroundf(us)));
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
