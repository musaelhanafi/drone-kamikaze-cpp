// test_joystick.cpp — minimal joystick RC override test
//
// Passes joystick axes (ch1–ch5) straight through to the FC via
// RC_CHANNELS_OVERRIDE.  Button 4 toggles the flight mode directly
// via MAV_CMD_DO_SET_MODE:
//   STABILIZE (ArduPlane custom_mode 2)
//   AUTOTUNE  (ArduPlane custom_mode 8)
// ch6 PWM mirrors the toggle (1000 / 2000) for reference.
//
// Usage:
//   ./test_joystick [--connection STR] [--joystick N]
//   Defaults: udpin:0.0.0.0:14560   joystick 0

#include "MavlinkConn.hpp"

#include <SDL2/SDL.h>

#include <ardupilotmega/mavlink.h>

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <stdexcept>
#include <string>
#include <thread>

static std::atomic<bool> g_stop{false};
static void sig_handler(int) { g_stop.store(true); }

// ── PWM helpers ───────────────────────────────────────────────────────────────

static int axis_pwm(Sint16 raw, bool invert = false)
{
    float v = raw / 32767.0f;
    if (invert) v = -v;
    int pwm = (int)(1500.0f + v * 500.0f);
    return pwm < 1000 ? 1000 : (pwm > 2000 ? 2000 : pwm);
}

static int thr_pwm(Sint16 raw)
{
    float v = raw / 32767.0f;          // [-1, +1]
    int pwm = (int)(1000.0f + (v + 1.0f) * 500.0f);
    return pwm < 1000 ? 1000 : (pwm > 2000 ? 2000 : pwm);
}

// ── Entry point ───────────────────────────────────────────────────────────────

int main(int argc, char* argv[])
{
    const char* conn_str  = "udpin:0.0.0.0:14560";
    int         joy_index = 0;

    for (int i = 1; i < argc; i++) {
        if (std::strcmp(argv[i], "--connection") == 0 && i+1 < argc)
            conn_str = argv[++i];
        else if (std::strcmp(argv[i], "--joystick") == 0 && i+1 < argc)
            joy_index = std::atoi(argv[++i]);
    }

    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    // ── MAVLink connection ────────────────────────────────────────────────────
    MavlinkConn mav(conn_str);
    try {
        mav.connect();
    } catch (const std::exception& e) {
        fprintf(stderr, "MAVLink connect failed: %s\n", e.what());
        return 1;
    }
    printf("Waiting for heartbeat on %s ...\n", conn_str);
    if (!mav.wait_heartbeat(30.0)) {
        fprintf(stderr, "No heartbeat received in 30 s.\n");
        return 1;
    }
    printf("Heartbeat  sys=%d comp=%d\n",
           mav.target_system, mav.target_component);

    // ── Joystick ──────────────────────────────────────────────────────────────
    if (SDL_InitSubSystem(SDL_INIT_JOYSTICK) < 0) {
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }
    int n_joy = SDL_NumJoysticks();
    if (n_joy == 0) {
        fprintf(stderr, "No joysticks detected.\n");
        SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
        return 1;
    }
    if (joy_index >= n_joy) {
        fprintf(stderr, "Joystick index %d out of range (found %d).\n", joy_index, n_joy);
        SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
        return 1;
    }
    SDL_Joystick* joy = SDL_JoystickOpen(joy_index);
    if (!joy) {
        fprintf(stderr, "SDL_JoystickOpen failed: %s\n", SDL_GetError());
        SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
        return 1;
    }
    printf("Joystick   [%d] %s  axes=%d  buttons=%d\n",
           joy_index, SDL_JoystickName(joy),
           SDL_JoystickNumAxes(joy), SDL_JoystickNumButtons(joy));

    // ── ArduPlane flight mode numbers ─────────────────────────────────────────
    // custom_mode values used in MAV_CMD_DO_SET_MODE / SET_MODE
    constexpr uint32_t MODE_STABILIZE = 2;
    constexpr uint32_t MODE_AUTOTUNE  = 8;

    // ── State ─────────────────────────────────────────────────────────────────
    uint32_t cur_mode  = MODE_STABILIZE;
    int      ch6_pwm   = 1000;   // mirrors mode: 1000=STABILIZE, 2000=AUTOTUNE
    bool     btn4_prev = false;

    // Helper: send MAV_CMD_DO_SET_MODE to switch ArduPlane flight mode
    auto send_set_mode = [&](uint32_t custom_mode) {
        mavlink_message_t msg;
        mavlink_msg_command_long_pack(
            mav.src_system, mav.src_component, &msg,
            mav.target_system, mav.target_component,
            MAV_CMD_DO_SET_MODE,
            0,                                    // confirmation
            MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,    // param1: base_mode
            (float)custom_mode,                   // param2: custom_mode
            0, 0, 0, 0, 0);
        mav.send_message(msg);
    };

    // Set initial mode
    send_set_mode(cur_mode);
    printf("\nMode = STABILIZE  ch6 = 1000\n");
    printf("Press button 4 to toggle.  Ctrl+C to exit.\n\n");

    using Clock  = std::chrono::steady_clock;
    using Ms     = std::chrono::milliseconds;
    auto next_tick = Clock::now();

    while (!g_stop.load()) {
        SDL_JoystickUpdate();

        int n_axes = SDL_JoystickNumAxes(joy);
        int n_btns = SDL_JoystickNumButtons(joy);

        auto axis = [&](int i) -> Sint16 {
            return (i < n_axes) ? SDL_JoystickGetAxis(joy, i) : 0;
        };
        auto btn = [&](int i) -> bool {
            return (i < n_btns) && SDL_JoystickGetButton(joy, i);
        };

        // Rising edge of button 4 → toggle flight mode via MAVLink
        bool btn4_now = btn(4);
        if (btn4_now && !btn4_prev) {
            if (cur_mode == MODE_STABILIZE) {
                cur_mode = MODE_AUTOTUNE;
                ch6_pwm  = 2000;
                printf("Mode → AUTOTUNE  ch6 = 2000\n");
            } else {
                cur_mode = MODE_STABILIZE;
                ch6_pwm  = 1000;
                printf("Mode → STABILIZE  ch6 = 1000\n");
            }
            send_set_mode(cur_mode);
        }
        btn4_prev = btn4_now;

        // Channel mapping (same as JoystickHandler):
        //   axis 0 → ch1 aileron   (centred)
        //   axis 1 → ch2 elevator  (centred, inverted)
        //   axis 2 → ch3 throttle  (full-range)
        //   axis 3 → ch4 rudder    (centred)
        //   button 6/7 → ch5
        int ch1 = axis_pwm(axis(0));
        int ch2 = axis_pwm(axis(1), /*invert=*/true);
        int ch3 = thr_pwm (axis(2));
        int ch4 = axis_pwm(axis(3));
        int ch5 = (btn(6) || btn(7)) ? 2000 : 1000;

        // Send RC_CHANNELS_OVERRIDE
        mavlink_message_t msg;
        mavlink_msg_rc_channels_override_pack(
            mav.src_system, mav.src_component, &msg,
            mav.target_system, mav.target_component,
            (uint16_t)ch1, (uint16_t)ch2,
            (uint16_t)ch3, (uint16_t)ch4,
            (uint16_t)ch5, (uint16_t)ch6_pwm,
            0, 0,
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0);
        mav.send_message(msg);

        next_tick += Ms(20);   // 50 Hz
        std::this_thread::sleep_until(next_tick);
    }

    printf("\nStopped.\n");
    SDL_JoystickClose(joy);
    SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
    mav.close();
    return 0;
}
