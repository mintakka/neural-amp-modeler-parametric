// Servo rig firmware for a Raspberry Pi Pico or Pico 2 (Pico SDK).
// Seven TowerPro SG92R servos on GP0..GP6 turn the amp's pots through 36:20 printed gears.
//
// Commands (one per line over USB serial; an optional trailing "@<tag>" is echoed back so
// the host can match each reply to its command):
//   PING                     -> OK servo-rig 3
//   AT <i> <us>              declare servo i is physically at <us> and start holding it
//                            there. Only for re-homing with the gear UNMESHED: a wrong
//                            value makes the servo jump to it at full speed.
//   LIMITS <i> <lo> <hi>     per-servo safe range (measured in calibration), saved to flash
//   SET <i> <us>             one smooth ramp of servo i to <us>
//   MOVE <us0> ... <us6>     ramp each listed servo in turn ("-" = leave it alone)
//   SPEED <us_per_s>         ramp cruise speed, 10-400 (default 225)
//   OFF [i]                  stop pulses (servo goes limp, held by friction)
//   GET                      -> OK <pos0> ... <pos6>     (0 = position unknown)
//   GETLIM                   -> OK <lo0> <hi0> ... <lo6> <hi6>
// Every command answers one line: "OK ..." when finished, or "ERR ...".
//
// Why it works this way (SG92R datasheet + measurements, see servo_rig/README.md):
// - 1000-2000 us is about 180 deg (~5.5 us/deg). Pulses outside that drive the servo into
//   its internal stops, so they are refused outright.
// - An analog servo drives at full speed toward every pulse; it has no speed control and
//   can't report its position. Slow, knob-like motion therefore comes only from here: each
//   move is ONE continuous ramp, the pulse advancing a few us every 20 ms frame with gentle
//   acceleration, starting from the servo's known position.
// - So the firmware must always know the true position. It saves positions to flash, marks
//   a servo "moving" before each ramp and clears that afterwards. A servo whose ramp was
//   interrupted (power loss) boots as unknown and refuses to move until re-homed with AT.
// - Pots are never commanded outside the servo's LIMITS, so nothing pushes into a pot stop.
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "hardware/clocks.h"
#include "hardware/flash.h"
#include "hardware/pwm.h"
#include "pico/flash.h"
#include "pico/stdlib.h"

namespace {

constexpr int kNumServos = 7;
constexpr uint kPins[kNumServos] = {0, 1, 2, 3, 4, 5, 6};
constexpr int kUsMin = 1000, kUsMax = 2000; // SG92R datasheet range: never exceeded
constexpr uint16_t kPeriodUs = 20000;
constexpr int kTickMs = 20;          // one servo frame
constexpr float kAccel = 400.f;      // us/s^2
constexpr float kSpeedMin = 10.f, kSpeedMax = 400.f;
// Final approach always from the low-pulse side, so gear slack sits the same way.
// ~2x the printed gears' backlash; small enough not to visibly wiggle the knob.
constexpr int kBacklashUs = 20;
constexpr int kSettleMs = 150;       // after arriving, before answering OK

float speed = 225.f;                 // us/s, about 2.4 dial numbers per second
float pos[kNumServos] = {};          // current pulse width; 0 = unknown
float lo[kNumServos], hi[kNumServos];
bool on[kNumServos] = {};

// ---- Saved state: an append-only log of 256-byte records in the last flash sector, so a
// sector erase happens only every 16 saves.
constexpr uint32_t kSectorOffset = PICO_FLASH_SIZE_BYTES - FLASH_SECTOR_SIZE;
constexpr int kSlots = FLASH_SECTOR_SIZE / FLASH_PAGE_SIZE;
constexpr uint32_t kMagic = 0x53524733u; // "SRG3"

struct Record {
    uint32_t magic;
    uint32_t seq;
    float pos[kNumServos];
    float lo[kNumServos], hi[kNumServos];
    uint8_t moving[kNumServos];
};
static_assert(sizeof(Record) <= FLASH_PAGE_SIZE);

uint8_t moving[kNumServos] = {};
uint32_t seq = 0;
int next_slot = 0;

const Record* Slot(int k) {
    return reinterpret_cast<const Record*>(XIP_BASE + kSectorOffset + k * FLASH_PAGE_SIZE);
}

struct FlashJob {
    bool erase;
    uint32_t offset;
    const uint8_t* page;
};

void DoFlash(void* arg) {
    auto* job = static_cast<FlashJob*>(arg);
    if (job->erase) flash_range_erase(kSectorOffset, FLASH_SECTOR_SIZE);
    flash_range_program(job->offset, job->page, FLASH_PAGE_SIZE);
}

void Save() {
    static uint8_t page[FLASH_PAGE_SIZE];
    Record r{kMagic, ++seq, {}, {}, {}, {}};
    std::copy(pos, pos + kNumServos, r.pos);
    std::copy(lo, lo + kNumServos, r.lo);
    std::copy(hi, hi + kNumServos, r.hi);
    std::copy(moving, moving + kNumServos, r.moving);
    std::fill(page, page + sizeof(page), 0xFF);
    memcpy(page, &r, sizeof(r));
    FlashJob job{next_slot == 0, kSectorOffset + next_slot * FLASH_PAGE_SIZE, page};
    flash_safe_execute(DoFlash, &job, 1000);
    next_slot = (next_slot + 1) % kSlots;
}

void Load() {
    for (int i = 0; i < kNumServos; i++) lo[i] = kUsMin, hi[i] = kUsMax;
    int best = -1;
    for (int k = 0; k < kSlots; k++)
        if (Slot(k)->magic == kMagic && (best < 0 || Slot(k)->seq > Slot(best)->seq)) best = k;
    if (best < 0) return; // nothing saved: every position unknown
    const Record& r = *Slot(best);
    seq = r.seq;
    next_slot = (best + 1) % kSlots;
    for (int i = 0; i < kNumServos; i++) {
        bool limits_ok = r.lo[i] >= kUsMin && r.hi[i] <= kUsMax && r.lo[i] < r.hi[i];
        if (limits_ok) lo[i] = r.lo[i], hi[i] = r.hi[i];
        // A ramp that never finished leaves the servo somewhere unknown.
        bool pos_ok = !r.moving[i] && r.pos[i] >= lo[i] && r.pos[i] <= hi[i];
        pos[i] = pos_ok ? r.pos[i] : 0.f;
    }
}

// ---- Motion

void Pulse(int i) {
    pwm_set_gpio_level(kPins[i], on[i] ? static_cast<uint16_t>(lroundf(pos[i])) : 0);
}

// One continuous ramp of servo i to `target`: trapezoidal velocity, updated every frame.
void Ramp(int i, float target) {
    const float dt = kTickMs / 1000.f;
    float vel = 0.f;
    absolute_time_t next = get_absolute_time();
    for (;;) {
        float remaining = target - pos[i];
        if (fabsf(remaining) < 0.5f && fabsf(vel) <= kAccel * dt) break;
        // Fastest speed that can still stop at the target, capped at the cruise speed.
        float vmax = std::min(speed, sqrtf(2.f * kAccel * fabsf(remaining)));
        float want = remaining > 0 ? vmax : -vmax;
        vel += std::clamp(want - vel, -kAccel * dt, kAccel * dt);
        float step = vel * dt;
        pos[i] = fabsf(step) >= fabsf(remaining) ? target : pos[i] + step;
        Pulse(i);
        next = delayed_by_ms(next, kTickMs);
        sleep_until(next);
    }
    pos[i] = target;
    Pulse(i);
}

// Smoothly move servo i to `us`. Pulses start at the known position, so switching on
// doesn't move it; the move itself is one ramp, with a short final approach from below.
bool MoveOne(int i, int us, char* error) {
    if (pos[i] == 0.f) {
        sprintf(error, "servo %d position unknown; re-home it (AT) first", i);
        return false;
    }
    if (us < lo[i] || us > hi[i]) {
        sprintf(error, "servo %d: %d us is outside its limits %d-%d", i, us,
                static_cast<int>(lo[i]), static_cast<int>(hi[i]));
        return false;
    }
    on[i] = true;
    Pulse(i);
    if (fabsf(pos[i] - us) < 0.5f) return true;
    moving[i] = 1;
    Save();
    if (us < pos[i]) Ramp(i, std::max(lo[i], static_cast<float>(us - kBacklashUs)));
    Ramp(i, static_cast<float>(us));
    moving[i] = 0;
    Save();
    sleep_ms(kSettleMs);
    return true;
}

void Off(int only) {
    for (int i = 0; i < kNumServos; i++)
        if (only < 0 || only == i) {
            on[i] = false;
            Pulse(i);
        }
}

// ---- Commands

bool ParseServo(const char* s, int* i) {
    *i = atoi(s);
    return *i >= 0 && *i < kNumServos;
}

void Handle(char* line) {
    char* argv[2 + 2 * kNumServos];
    int argc = 0;
    for (char* tok = strtok(line, " \t"); tok && argc < 2 + 2 * kNumServos;
         tok = strtok(nullptr, " \t"))
        argv[argc++] = tok;
    const char* tag = "";
    if (argc > 0 && argv[argc - 1][0] == '@') tag = argv[--argc];
    if (argc == 0) return;
    for (char* c = argv[0]; *c; c++) *c = static_cast<char>(toupper(*c));
    const char* cmd = argv[0];
    char error[96] = "";
    char reply[160] = "OK";
    int i = 0;
    bool ok = true;

    if (!strcmp(cmd, "PING") && argc == 1) {
        strcpy(reply, "OK servo-rig 3");
    } else if (!strcmp(cmd, "AT") && argc == 3 && ParseServo(argv[1], &i)) {
        int us = atoi(argv[2]);
        if (us < kUsMin || us > kUsMax) {
            ok = false;
            sprintf(error, "AT %d us is outside %d-%d", us, kUsMin, kUsMax);
        } else {
            pos[i] = static_cast<float>(us);
            moving[i] = 0;
            on[i] = true;
            Pulse(i);
            Save();
        }
    } else if (!strcmp(cmd, "LIMITS") && argc == 4 && ParseServo(argv[1], &i)) {
        int a = atoi(argv[2]), b = atoi(argv[3]);
        if (a < kUsMin || b > kUsMax || a >= b) {
            ok = false;
            sprintf(error, "limits must satisfy %d <= lo < hi <= %d", kUsMin, kUsMax);
        } else {
            lo[i] = static_cast<float>(a);
            hi[i] = static_cast<float>(b);
            Save();
        }
    } else if (!strcmp(cmd, "SET") && argc == 3 && ParseServo(argv[1], &i)) {
        ok = MoveOne(i, atoi(argv[2]), error);
    } else if (!strcmp(cmd, "MOVE") && argc == 1 + kNumServos) {
        // One servo at a time: smooth, and within the USB port's current budget.
        for (int k = 0; k < kNumServos && ok; k++)
            if (strcmp(argv[1 + k], "-")) ok = MoveOne(k, atoi(argv[1 + k]), error);
    } else if (!strcmp(cmd, "SPEED") && argc == 2) {
        float v = static_cast<float>(atof(argv[1]));
        if (v < kSpeedMin || v > kSpeedMax) {
            ok = false;
            sprintf(error, "speed must be %d-%d us/s", static_cast<int>(kSpeedMin),
                    static_cast<int>(kSpeedMax));
        } else {
            speed = v;
        }
    } else if (!strcmp(cmd, "OFF") && argc <= 2) {
        if (argc == 2 && !ParseServo(argv[1], &i)) {
            ok = false;
            strcpy(error, "bad servo index");
        } else {
            Off(argc == 2 ? i : -1);
            Save();
        }
    } else if (!strcmp(cmd, "GET") && argc == 1) {
        int n = 2;
        for (float us : pos) n += sprintf(reply + n, " %d", static_cast<int>(lroundf(us)));
    } else if (!strcmp(cmd, "GETLIM") && argc == 1) {
        int n = 2;
        for (int k = 0; k < kNumServos; k++)
            n += sprintf(reply + n, " %d %d", static_cast<int>(lo[k]), static_cast<int>(hi[k]));
    } else {
        ok = false;
        strcpy(error, "unknown or malformed command");
    }

    if (ok) printf("%s%s%s\n", reply, *tag ? " " : "", tag);
    else printf("ERR %s%s%s\n", error, *tag ? " " : "", tag);
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
        pwm_set_gpio_level(pin, 0); // no pulses until a command asks for them
    }
    Load();

    char line[160];
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
