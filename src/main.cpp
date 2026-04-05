#include "SeekerCtrl.hpp"

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <stdexcept>

// ── Minimal argument parser ───────────────────────────────────────────────────

static void usage(const char* prog)
{
    printf(
        "Usage: %s [options]\n"
        "\n"
        "  --connection STR     MAVLink connection string (default: udpin:0.0.0.0:14560)\n"
        "  --baud N             Baud rate for serial connections (default: 57600)\n"
        "  --source STR         Camera index or video file (default: 0)\n"
        "  --res W H            Request capture resolution (e.g. --res 1280 720)\n"
        "  --crop X Y W H       Crop ROI from each frame (use 0 for W/H to mean 'rest')\n"
        "  --mask-algo ALGO     gaussian|adaptive|inrange|all (default: all)\n"
        "  --histogram          Show calibration histogram window\n"
        "  --mask               Show detection mask window\n"
        "  --no-camshift        Use blob centroid directly (disable CamShift)\n"
        "  --no-box-filter      Accept any blob shape\n"
        "  --no-prediction      Disable PN lead input prediction\n"
        "  --no-hud-pitch       Disable pitch ladder in HUD\n"
        "  --no-hud-yaw         Disable yaw compass in HUD\n"
        "  --debug              Log telemetry to tracking.csv during TRACKING\n"
        "  --record             Record annotated video during TRACKING\n"
        "  --auto               Auto mode: enter TRACKING near target on final WP\n"
        "  --joystick [N]       Enable joystick RC override; N = device index (default 0)\n"
        "  --help               Show this help\n",
        prog);
}

int main(int argc, char* argv[])
{
    SeekerCtrlConfig cfg;

    // Parse argv
    for (int i = 1; i < argc; i++) {
        const char* a = argv[i];

        if (std::strcmp(a, "--help") == 0 || std::strcmp(a, "-h") == 0) {
            usage(argv[0]); return 0;
        } else if (std::strcmp(a, "--connection") == 0 && i+1 < argc) {
            cfg.connection_string = argv[++i];
        } else if (std::strcmp(a, "--baud") == 0 && i+1 < argc) {
            cfg.baud = std::atoi(argv[++i]);
        } else if (std::strcmp(a, "--source") == 0 && i+1 < argc) {
            const char* src = argv[++i];
            // Try to parse as integer; otherwise treat as file path
            char* end;
            long idx = std::strtol(src, &end, 10);
            if (*end == '\0') {
                cfg.source_index = (int)idx;
            } else {
                cfg.source_file  = src;
                cfg.source_index = 0;
            }
        } else if (std::strcmp(a, "--res") == 0 && i+2 < argc) {
            cfg.capture_width  = std::atoi(argv[++i]);
            cfg.capture_height = std::atoi(argv[++i]);
        } else if (std::strcmp(a, "--crop") == 0 && i+4 < argc) {
            int cx = std::atoi(argv[++i]);
            int cy = std::atoi(argv[++i]);
            int cw = std::atoi(argv[++i]);
            int ch = std::atoi(argv[++i]);
            cfg.crop     = {cx, cy, cw, ch};
            cfg.use_crop = true;
        } else if (std::strcmp(a, "--mask-algo") == 0 && i+1 < argc) {
            cfg.mask_algo = argv[++i];
        } else if (std::strcmp(a, "--histogram") == 0) {
            cfg.show_histogram = true;
        } else if (std::strcmp(a, "--mask") == 0) {
            cfg.show_mask = true;
        } else if (std::strcmp(a, "--no-camshift") == 0) {
            cfg.use_camshift = false;
        } else if (std::strcmp(a, "--no-box-filter") == 0) {
            cfg.box_filter = false;
        } else if (std::strcmp(a, "--no-prediction") == 0) {
            cfg.input_prediction = false;
        } else if (std::strcmp(a, "--no-hud-pitch") == 0) {
            cfg.hud_pitch = false;
        } else if (std::strcmp(a, "--no-hud-yaw") == 0) {
            cfg.hud_yaw = false;
        } else if (std::strcmp(a, "--debug") == 0) {
            cfg.debug_log = true;
        } else if (std::strcmp(a, "--record") == 0) {
            cfg.record = true;
        } else if (std::strcmp(a, "--auto") == 0) {
            cfg.auto_mode = true;
        } else if (std::strcmp(a, "--joystick") == 0) {
            cfg.use_joystick = true;
            // Optional numeric argument: --joystick [N]
            if (i+1 < argc) {
                char* end;
                long idx = std::strtol(argv[i+1], &end, 10);
                if (*end == '\0') { cfg.joy_index = (int)idx; i++; }
            }
        } else {
            printf("Unknown option: %s\n", a);
            usage(argv[0]);
            return 1;
        }
    }

    try {
        SeekerCtrl ctrl(cfg);
        ctrl.connect();
        ctrl.run();
    } catch (const std::exception& e) {
        fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }
    return 0;
}
