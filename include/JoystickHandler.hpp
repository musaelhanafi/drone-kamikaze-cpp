#pragma once

#ifdef DRONE_USE_JOYSTICK
#include <SDL2/SDL.h>
#endif

// PWM values for all 6 RC channels (1000–2000 µs).
// ch6 == 2000 → switch active, == 1000 → released.
struct JoyChannels {
    int ch1 = 1500;  // aileron
    int ch2 = 1500;  // elevator
    int ch3 = 1000;  // throttle
    int ch4 = 1500;  // rudder
    int ch5 = 1000;  // mode switch (button 6/7)
    int ch6 = 1000;  // arm/ch6   (button 4/5 or trigger axis)
};

class JoystickHandler {
public:
    explicit JoystickHandler(int index = 0);
    ~JoystickHandler();

    // Initialise SDL joystick subsystem and open device.
    // Prints device name and axis/button count.
    void open();

    // Close device and shut down SDL joystick subsystem.
    void close();

    // Pump SDL events and return the current mapped channel values.
    // Must be called from the same thread as open().
    JoyChannels read();

private:
    int _index;

    static int  _axis_pwm(float v, bool invert = false);
    static int  _thr_pwm (float v, bool invert = false);

#ifdef DRONE_USE_JOYSTICK
    SDL_Joystick* _joy = nullptr;
#endif
};
