#include "JoystickHandler.hpp"

#include <cstdio>
#include <stdexcept>
#include <string>
#include <algorithm>
#include <cmath>

// ── PWM helpers ───────────────────────────────────────────────────────────────

// Centred axis: 1500 ± 500
int JoystickHandler::_axis_pwm(float v, bool invert)
{
    if (invert) v = -v;
    return std::max(1000, std::min(2000, (int)(1500.0f + v * 500.0f)));
}

// Throttle axis: 1000–2000 full range
int JoystickHandler::_thr_pwm(float v, bool invert)
{
    if (invert) v = -v;
    return std::max(1000, std::min(2000, (int)(1000.0f + (v + 1.0f) * 500.0f)));
}

// ── JoystickHandler ───────────────────────────────────────────────────────────

JoystickHandler::JoystickHandler(int index)
    : _index(index)
{}

JoystickHandler::~JoystickHandler()
{
    close();
}

void JoystickHandler::open()
{
#ifndef DRONE_USE_JOYSTICK
    throw std::runtime_error(
        "[Joy] Joystick support not compiled in (build with -DWITH_JOYSTICK=ON)");
#else
    if (SDL_InitSubSystem(SDL_INIT_JOYSTICK) < 0)
        throw std::runtime_error(std::string("[Joy] SDL_Init failed: ") + SDL_GetError());

    int count = SDL_NumJoysticks();
    if (count == 0)
        throw std::runtime_error("[Joy] No joysticks detected");
    if (_index >= count)
        throw std::runtime_error(
            std::string("[Joy] Index ") + std::to_string(_index) +
            " out of range (found " + std::to_string(count) + ")");

    _joy = SDL_JoystickOpen(_index);
    if (!_joy)
        throw std::runtime_error(std::string("[Joy] SDL_JoystickOpen failed: ") + SDL_GetError());

    printf("[Joy] Using [%d] %s  axes=%d  buttons=%d\n",
           _index,
           SDL_JoystickName(_joy),
           SDL_JoystickNumAxes(_joy),
           SDL_JoystickNumButtons(_joy));
#endif
}

void JoystickHandler::close()
{
#ifdef DRONE_USE_JOYSTICK
    if (_joy) {
        SDL_JoystickClose(_joy);
        _joy = nullptr;
    }
    SDL_QuitSubSystem(SDL_INIT_JOYSTICK);
#endif
}

JoyChannels JoystickHandler::read()
{
    JoyChannels ch;

#ifdef DRONE_USE_JOYSTICK
    if (!_joy) return ch;

    SDL_JoystickUpdate();

    int n_axes    = SDL_JoystickNumAxes(_joy);
    int n_buttons = SDL_JoystickNumButtons(_joy);

    auto axis = [&](int i) -> float {
        if (i >= n_axes) return 0.0f;
        return SDL_JoystickGetAxis(_joy, i) / 32767.0f;
    };
    auto btn = [&](int i) -> bool {
        return i < n_buttons && SDL_JoystickGetButton(_joy, i);
    };

    // Channel mapping (matches Python joystick_handler.py):
    //   Axis 0 → CH1 aileron   (centred)
    //   Axis 1 → CH2 elevator  (centred, inverted)
    //   Axis 2 → CH3 throttle  (full-range)
    //   Axis 3 → CH4 rudder    (centred)
    //   Button 6 or 7 → CH5   (2000 pressed, 1000 released)
    //   Button 4/5 or axis 4/5 > 0.5 → CH6 (2000 active, 1000 released)
    ch.ch1 = _axis_pwm(axis(0));
    ch.ch2 = _axis_pwm(axis(1), /*invert=*/true);
    ch.ch3 = _thr_pwm (axis(2));
    ch.ch4 = _axis_pwm(axis(3));

    bool ch5_on = btn(6) || btn(7);
    ch.ch5 = ch5_on ? 2000 : 1000;

    bool ch6_on = btn(4) || btn(5) || axis(4) > 0.5f || axis(5) > 0.5f;
    ch.ch6 = ch6_on ? 2000 : 1000;
#endif

    return ch;
}
