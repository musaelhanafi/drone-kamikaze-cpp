#pragma once
#include "MavlinkConn.hpp"
#include "Seeker.hpp"
#include "HudDisplay.hpp"
#include "JoystickHandler.hpp"

#include <string>
#include <memory>
#include <fstream>
#include <optional>
#include <atomic>
#include <thread>
#include <mutex>

// ── Tracking control tuning ───────────────────────────────────────────────────
inline constexpr double LATENCY_S          = 0.08;   // pipeline latency to compensate (s)
inline constexpr double PN_LEAD_S          = 0.30;   // proportional navigation lead (s)
inline constexpr double TRK_MAX_DEG        = 30.0;   // must match ArduPlane TRK_MAX_DEG
inline constexpr double TRK_PITCH_OFFSET   = 3.0;    // must match ArduPlane TRK_PITCH_OFFSET
inline constexpr double TRK_TERM_PTCH      = 0.0;    // must match ArduPlane TRK_TERM_PTCH

// Default target position (overridden by params fetched from ArduPlane)
inline constexpr double DEFAULT_TARGET_ALT_MSL = 744.0;
inline constexpr double DEFAULT_TARGET_LAT     = -6.897367724;
inline constexpr double DEFAULT_TARGET_LON     = 107.566559898;

// ArduPlane custom mode numbers
inline constexpr int TRACKING_MODE  = 27;
inline constexpr int LOITER_MODE    = 5;
inline constexpr int AUTO_MODE      = 10;
inline constexpr int STABILIZE_MODE = 2;

// ch6 PWM threshold to consider the switch "active"
inline constexpr int CH6_ACTIVE_PWM = 1400;

// ── SeekerCtrl ───────────────────────────────────────────────────────────────

struct SeekerCtrlConfig {
    std::string connection_string = "udpin:0.0.0.0:14560";
    int         baud              = 57600;

    // Camera source (index or file path)
    int         source_index      = 0;
    std::string source_file;

    int         capture_width     = 0;
    int         capture_height    = 0;
    cv::Rect    crop;
    bool        use_crop          = false;

    bool        show_histogram    = false;
    bool        show_mask         = false;
    bool        debug_log         = false;
    bool        record            = false;
    bool        input_prediction  = true;
    std::string mask_algo         = "all";
    bool        use_camshift      = true;
    bool        box_filter        = true;
    bool        hud_pitch         = true;
    bool        hud_yaw           = true;
    bool        auto_mode         = false;

    bool        use_joystick      = false;
    int         joy_index         = 0;
};

class SeekerCtrl {
public:
    explicit SeekerCtrl(const SeekerCtrlConfig& cfg);

    // Connect to MAVLink, fetch params, open camera.
    void connect();

    // Main loop — runs until 'q' pressed or stream ends.
    void run();

private:
    // ── MAVLink helpers ───────────────────────────────────────────────────────
    void _requestDataStreams();
    void _fetchTrackingParams();
    void _fetchMissionCount();
    void _pollMavlinkState();
    void _pollRC();
    void _pollHeartbeat();
    bool _ch6Active() const;
    void _setMode(int custom_mode);
    void _sendTracking(double errorx, double errory);

    // ── Distance helpers ──────────────────────────────────────────────────────
    double _distToTargetM() const;
    double _distToHomeM()   const;

    // ── CSV logger ────────────────────────────────────────────────────────────
    void _openCsv();
    void _closeCsv();
    void _logRow(double timestamp, double errorx, double errory,
                 bool target_locked, bool terminal);

    // ── Video recorder ────────────────────────────────────────────────────────
    void _openVideo(int w, int h);
    void _closeVideo();
    void _writeFrame(const cv::Mat& frame);

    // ── RC override (joystick → autopilot) ───────────────────────────────────
    void _sendRcOverride(const JoyChannels& ch);

    // ── Mode names ────────────────────────────────────────────────────────────
    static std::string _modeName(int custom_mode);

    // ── Config ───────────────────────────────────────────────────────────────
    SeekerCtrlConfig _cfg;

    // ── MAVLink ───────────────────────────────────────────────────────────────
    std::unique_ptr<MavlinkConn> _mav;

    // ── Seeker ───────────────────────────────────────────────────────────────
    std::unique_ptr<Seeker>          _seeker;
    std::unique_ptr<HudDisplay>      _hud;
    std::unique_ptr<JoystickHandler> _joy;

    // ── RC / mode state ───────────────────────────────────────────────────────
    int    _rc_ch6_pwm       = 0;
    bool   _in_tracking      = false;
    std::string _flight_mode = "?";
    int    _commanded_mode   = -1;
    bool   _prev_ch6_on      = false;
    int    _tracking_entry_count = 0;
    int    _lost_count           = 0;

    // ── MAVLink telemetry state ───────────────────────────────────────────────
    int    _srv1_raw          = 0;
    int    _srv2_raw          = 0;
    double _roll_deg          = 0.0;
    double _pitch_deg         = 0.0;
    double _nav_pitch_deg     = 0.0;
    double _yaw_deg           = 0.0;
    double _roll_rate_dps     = 0.0;
    double _pitch_rate_dps    = 0.0;
    double _lat               = 0.0;
    double _lon               = 0.0;
    double _rel_alt_m         = 0.0;
    double _alt_msl_m         = 0.0;
    double _airspeed_ms       = 0.0;
    int    _throttle_pct      = 0;
    double _home_lat          = 0.0;
    double _home_lon          = 0.0;
    bool   _home_valid        = false;
    int    _current_wp        = 0;
    int    _waypoint_count    = 0;

    // PID_TUNING fields (debug mode only)
    double _pid_roll_P = 0.0, _pid_roll_I = 0.0, _pid_roll_D = 0.0, _pid_roll_des = 0.0;
    double _pid_pitch_P= 0.0, _pid_pitch_I= 0.0, _pid_pitch_D= 0.0, _pid_pitch_des= 0.0;

    // ── Target position (from ArduPlane params) ───────────────────────────────
    double _target_alt_msl    = DEFAULT_TARGET_ALT_MSL;
    double _target_lat        = DEFAULT_TARGET_LAT;
    double _target_lon        = DEFAULT_TARGET_LON;
    double _term_alt          = 0.0;

    // ── Latency prediction state ──────────────────────────────────────────────
    double _prev_errorx       = 0.0;
    double _prev_errory       = 0.0;
    double _prev_err_t        = 0.0;
    double _last_errorx       = 0.0;
    double _last_errory       = 0.0;

    // ── CSV / video ───────────────────────────────────────────────────────────
    std::ofstream         _csv_file;
    cv::VideoWriter       _vwriter;
    bool                  _vwriter_open = false;
};
