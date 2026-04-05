#include "SeekerCtrl.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <chrono>
#include <sstream>
#include <iomanip>
#include <ctime>

using namespace std::chrono;

// ── Monotonic clock ───────────────────────────────────────────────────────────
static double mono_s()
{
    return duration<double>(steady_clock::now().time_since_epoch()).count();
}

// ── Haversine ─────────────────────────────────────────────────────────────────
static double haversine(double lat1_deg, double lon1_deg,
                        double lat2_deg, double lon2_deg)
{
    constexpr double R = 6371000.0;
    double lat1 = lat1_deg * M_PI / 180.0;
    double lat2 = lat2_deg * M_PI / 180.0;
    double dlat = (lat2_deg - lat1_deg) * M_PI / 180.0;
    double dlon = (lon2_deg - lon1_deg) * M_PI / 180.0;
    double a = std::sin(dlat/2)*std::sin(dlat/2) +
               std::cos(lat1)*std::cos(lat2)*std::sin(dlon/2)*std::sin(dlon/2);
    return R * 2.0 * std::asin(std::sqrt(a));
}

// ── Mode names ────────────────────────────────────────────────────────────────
std::string SeekerCtrl::_modeName(int m)
{
    switch (m) {
        case  0: return "MANUAL";     case  1: return "CIRCLE";
        case  2: return "STABILIZE";  case  3: return "TRAINING";
        case  4: return "ACRO";       case  5: return "LOITER";
        case  6: return "FBW_B";      case  7: return "CRUISE";
        case  8: return "AUTOTUNE";   case 10: return "AUTO";
        case 11: return "RTL";        case 12: return "LOITER";
        case 13: return "TAKEOFF";    case 15: return "GUIDED";
        case 17: return "QSTABILIZE"; case 18: return "QHOVER";
        case 19: return "QLOITER";    case 20: return "QLAND";
        case 21: return "QRTL";       case 27: return "TRACKING";
        default:
            return "MODE(" + std::to_string(m) + ")";
    }
}

// ── ctor ──────────────────────────────────────────────────────────────────────

SeekerCtrl::SeekerCtrl(const SeekerCtrlConfig& cfg)
    : _cfg(cfg)
{
    _mav = std::make_unique<MavlinkConn>(cfg.connection_string, cfg.baud);

    _seeker = std::make_unique<Seeker>(
        cfg.source_index,
        cfg.source_file,
        "Seeker",
        cfg.capture_width,
        cfg.capture_height,
        cfg.crop,
        cfg.use_crop,
        CAL_HISTOGRAM_FILE,
        cfg.show_histogram,
        cfg.show_mask,
        cfg.mask_algo,
        cfg.use_camshift,
        cfg.box_filter
    );

    _hud = std::make_unique<HudDisplay>(0, 120, cfg.hud_pitch, cfg.hud_yaw);
}

// ── connect ───────────────────────────────────────────────────────────────────

void SeekerCtrl::connect()
{
    printf("Connecting to %s ...\n", _cfg.connection_string.c_str());
    _mav->connect();
    if (!_mav->wait_heartbeat(30.0))
        throw std::runtime_error("No heartbeat received within 30 s");
    _requestDataStreams();
    _fetchTrackingParams();
    _fetchMissionCount();
}

// ── Stream requests ───────────────────────────────────────────────────────────

void SeekerCtrl::_requestDataStreams()
{
    // MAV_CMD_SET_MESSAGE_INTERVAL = 511
    constexpr int CMD_SET_INTERVAL = 511;

    struct { int msg_id; int rate_hz; } streams[] = {
        { 30, 25 },   // ATTITUDE
        { 36, 25 },   // SERVO_OUTPUT_RAW
        { 33,  5 },   // GLOBAL_POSITION_INT
        { 74, 10 },   // VFR_HUD
        { 62, 25 },   // NAV_CONTROLLER_OUTPUT
        { 65, 10 },   // RC_CHANNELS
    };
    if (_cfg.debug_log) {
        // PID_TUNING will be added via separate call below
    }

    for (auto& s : streams) {
        mavlink_message_t msg;
        mavlink_msg_command_long_pack(
            _mav->src_system, _mav->src_component, &msg,
            _mav->target_system, _mav->target_component,
            CMD_SET_INTERVAL, 0,
            (float)s.msg_id,
            (float)(1000000 / s.rate_hz),  // interval µs
            0, 0, 0, 0, 0);
        _mav->send_message(msg);
    }

    if (_cfg.debug_log) {
        mavlink_message_t msg;
        mavlink_msg_command_long_pack(
            _mav->src_system, _mav->src_component, &msg,
            _mav->target_system, _mav->target_component,
            CMD_SET_INTERVAL, 0,
            98.f, 1000000.f / 25.f, 0, 0, 0, 0, 0);
        _mav->send_message(msg);
    }
}

// ── Parameter fetch ───────────────────────────────────────────────────────────

void SeekerCtrl::_fetchTrackingParams()
{
    const char* names[] = {"TRK_TGT_ALT", "TRK_TGT_LAT", "TRK_TGT_LON", "TRK_TERM_ALT"};
    for (const char* n : names) {
        mavlink_message_t msg;
        char param_id[16] = {};
        std::strncpy(param_id, n, 15);
        mavlink_msg_param_request_read_pack(
            _mav->src_system, _mav->src_component, &msg,
            _mav->target_system, _mav->target_component,
            param_id, -1);
        _mav->send_message(msg);
    }

    int remaining = 4;
    double deadline = mono_s() + 2.0;
    while (remaining > 0 && mono_s() < deadline) {
        mavlink_message_t msg;
        if (!_mav->recv_match(MsgId::PARAM_VALUE, msg, 0.2)) continue;

        mavlink_param_value_t pv;
        mavlink_msg_param_value_decode(&msg, &pv);

        // Null-terminate param_id
        char id[17] = {};
        std::memcpy(id, pv.param_id, 16);

        if (std::strcmp(id, "TRK_TGT_ALT") == 0) {
            _target_alt_msl = pv.param_value;  printf("[Param] TRK_TGT_ALT = %.1f\n", _target_alt_msl);
            remaining--;
        } else if (std::strcmp(id, "TRK_TGT_LAT") == 0) {
            _target_lat = pv.param_value;      printf("[Param] TRK_TGT_LAT = %.7f\n", _target_lat);
            remaining--;
        } else if (std::strcmp(id, "TRK_TGT_LON") == 0) {
            _target_lon = pv.param_value;      printf("[Param] TRK_TGT_LON = %.7f\n", _target_lon);
            remaining--;
        } else if (std::strcmp(id, "TRK_TERM_ALT") == 0) {
            _term_alt = pv.param_value;        printf("[Param] TRK_TERM_ALT = %.1f\n", _term_alt);
            remaining--;
        }
    }
    if (remaining > 0) printf("[Param] %d param(s) not received — using defaults\n", remaining);
}

void SeekerCtrl::_fetchMissionCount()
{
    {
        mavlink_message_t msg;
        mavlink_msg_mission_request_list_pack(
            _mav->src_system, _mav->src_component, &msg,
            _mav->target_system, _mav->target_component,
            MAV_MISSION_TYPE_MISSION);
        _mav->send_message(msg);
    }

    mavlink_message_t msg;
    if (_mav->recv_match(MsgId::MISSION_COUNT, msg, 3.0)) {
        mavlink_mission_count_t mc;
        mavlink_msg_mission_count_decode(&msg, &mc);
        _waypoint_count = mc.count;
        printf("[Mission] %d waypoints\n", _waypoint_count);
    } else {
        printf("[Mission] MISSION_COUNT not received — waypoint check disabled\n");
    }

    {
        mavlink_message_t msg2;
        mavlink_msg_mission_set_current_pack(
            _mav->src_system, _mav->src_component, &msg2,
            _mav->target_system, _mav->target_component, 0);
        _mav->send_message(msg2);
    }
    _current_wp = 0;
    printf("[Mission] Current WP set to 0\n");
}

// ── Poll telemetry ────────────────────────────────────────────────────────────

void SeekerCtrl::_pollMavlinkState()
{
    mavlink_message_t msg;

    if (_mav->get_message(MsgId::SERVO_OUTPUT_RAW, msg)) {
        mavlink_servo_output_raw_t s;
        mavlink_msg_servo_output_raw_decode(&msg, &s);
        _srv1_raw = s.servo1_raw;
        _srv2_raw = s.servo2_raw;
    }

    if (_mav->get_message(MsgId::ATTITUDE, msg)) {
        mavlink_attitude_t a;
        mavlink_msg_attitude_decode(&msg, &a);
        _roll_deg       = a.roll       * 180.0 / M_PI;
        _pitch_deg      = a.pitch      * 180.0 / M_PI;
        _yaw_deg        = std::fmod(a.yaw * 180.0 / M_PI + 360.0, 360.0);
        _roll_rate_dps  = a.rollspeed  * 180.0 / M_PI;
        _pitch_rate_dps = a.pitchspeed * 180.0 / M_PI;
    }

    if (_cfg.debug_log) {
        // Drain all PID_TUNING messages
        while (_mav->get_message(MsgId::PID_TUNING, msg)) {
            mavlink_pid_tuning_t pt;
            mavlink_msg_pid_tuning_decode(&msg, &pt);
            if (pt.axis == 1) {  // roll
                _pid_roll_P   = pt.P;
                _pid_roll_I   = pt.I;
                _pid_roll_D   = pt.D;
                _pid_roll_des = pt.desired;
            } else if (pt.axis == 2) {  // pitch
                _pid_pitch_P   = pt.P;
                _pid_pitch_I   = pt.I;
                _pid_pitch_D   = pt.D;
                _pid_pitch_des = pt.desired;
            }
        }
    }

    if (_mav->get_message(MsgId::GLOBAL_POSITION_INT, msg)) {
        mavlink_global_position_int_t g;
        mavlink_msg_global_position_int_decode(&msg, &g);
        _rel_alt_m = g.relative_alt * 1e-3;
        _alt_msl_m = g.alt * 1e-3;
        _lat       = g.lat * 1e-7;
        _lon       = g.lon * 1e-7;
    }

    if (_mav->get_message(MsgId::VFR_HUD, msg)) {
        mavlink_vfr_hud_t v;
        mavlink_msg_vfr_hud_decode(&msg, &v);
        _airspeed_ms  = v.airspeed;
        _throttle_pct = v.throttle;
    }

    if (_mav->get_message(MsgId::HOME_POSITION, msg)) {
        mavlink_home_position_t h;
        mavlink_msg_home_position_decode(&msg, &h);
        _home_lat   = h.latitude  * 1e-7;
        _home_lon   = h.longitude * 1e-7;
        _home_valid = true;
    }

    if (_mav->get_message(MsgId::MISSION_CURRENT, msg)) {
        mavlink_mission_current_t mc;
        mavlink_msg_mission_current_decode(&msg, &mc);
        _current_wp = mc.seq;
    }

    if (_mav->get_message(MsgId::NAV_CONTROLLER_OUTPUT, msg)) {
        mavlink_nav_controller_output_t nc;
        mavlink_msg_nav_controller_output_decode(&msg, &nc);
        _nav_pitch_deg = nc.nav_pitch;
    }
}

void SeekerCtrl::_pollRC()
{
    mavlink_message_t msg;
    if (_mav->get_message(MsgId::RC_CHANNELS, msg)) {
        mavlink_rc_channels_t rc;
        mavlink_msg_rc_channels_decode(&msg, &rc);
        _rc_ch6_pwm = rc.chan6_raw;
    }
}

void SeekerCtrl::_pollHeartbeat()
{
    mavlink_message_t msg;
    if (_mav->get_message(MsgId::HEARTBEAT, msg)) {
        mavlink_heartbeat_t hb;
        mavlink_msg_heartbeat_decode(&msg, &hb);
        _flight_mode = _modeName(hb.custom_mode);
    }
}

bool SeekerCtrl::_ch6Active() const
{
    return _rc_ch6_pwm >= CH6_ACTIVE_PWM;
}

// ── Mode commands ─────────────────────────────────────────────────────────────

void SeekerCtrl::_setMode(int custom_mode)
{
    if (_commanded_mode == custom_mode) return;
    _commanded_mode = custom_mode;

    mavlink_message_t msg;
    // MAV_CMD_DO_SET_MODE = 176
    // param1 = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED = 1
    mavlink_msg_command_long_pack(
        _mav->src_system, _mav->src_component, &msg,
        _mav->target_system, _mav->target_component,
        176,  // MAV_CMD_DO_SET_MODE
        0,
        1.0f,  // MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
        (float)custom_mode,
        0, 0, 0, 0, 0);
    _mav->send_message(msg);
}

// ── Tracking message ──────────────────────────────────────────────────────────

void SeekerCtrl::_sendTracking(double errorx, double errory)
{
    mavlink_message_t msg;
    uint64_t time_usec = (uint64_t)(mono_s() * 1e6);
    char name[10] = "tracking\0";
    mavlink_msg_debug_vect_pack(
        _mav->src_system, _mav->src_component, &msg,
        name, time_usec,
        (float)errorx, (float)errory, 0.0f);
    _mav->send_message(msg);
}

// ── Distance helpers ──────────────────────────────────────────────────────────

double SeekerCtrl::_distToTargetM() const
{
    if (_lat == 0.0 && _lon == 0.0) return std::numeric_limits<double>::infinity();
    return haversine(_lat, _lon, _target_lat, _target_lon);
}

double SeekerCtrl::_distToHomeM() const
{
    if (!_home_valid || (_lat == 0.0 && _lon == 0.0)) return 0.0;
    return haversine(_lat, _lon, _home_lat, _home_lon);
}

// ── CSV logger ────────────────────────────────────────────────────────────────

void SeekerCtrl::_openCsv()
{
    _csv_file.open("tracking.csv");
    if (!_csv_file.is_open()) { printf("[LOG] Failed to open tracking.csv\n"); return; }
    _csv_file << "timestamp_s,errorx,errory,"
                 "aileron,elevator,"
                 "roll_deg,pitch_deg,roll_rate_dps,pitch_rate_dps,"
                 "pid_roll_desired,pid_roll_P,pid_roll_I,pid_roll_D,"
                 "pid_pitch_desired,pid_pitch_P,pid_pitch_I,pid_pitch_D,"
                 "alt_rel_m,airspeed_ms,throttle_pct,"
                 "nav_pitch_deg,"
                 "target_locked,terminal,"
                 "dist_m\n";
    printf("[LOG] tracking.csv opened\n");
}

void SeekerCtrl::_closeCsv()
{
    if (_csv_file.is_open()) {
        _csv_file.flush();
        _csv_file.close();
        printf("[LOG] tracking.csv closed\n");
    }
}

void SeekerCtrl::_logRow(double ts, double errorx, double errory,
                          bool target_locked, bool terminal)
{
    if (!_csv_file.is_open()) return;
    double aileron  = (_srv1_raw - _srv2_raw) / 700.0;
    double elevator = (_srv1_raw + _srv2_raw - 2700) / 700.0;
    double alt_rel  = _alt_msl_m - _target_alt_msl;
    double dist     = std::hypot(_distToTargetM(), alt_rel);

    _csv_file << std::fixed << std::setprecision(3) << ts << ","
              << std::setprecision(4) << (target_locked ? errorx : std::numeric_limits<double>::quiet_NaN()) << ","
              << (target_locked ? errory : std::numeric_limits<double>::quiet_NaN()) << ","
              << aileron << "," << elevator << ","
              << std::setprecision(3)
              << _roll_deg << "," << _pitch_deg << ","
              << _roll_rate_dps << "," << _pitch_rate_dps << ","
              << std::setprecision(4)
              << _pid_roll_des << "," << _pid_roll_P << "," << _pid_roll_I << "," << _pid_roll_D << ","
              << _pid_pitch_des << "," << _pid_pitch_P << "," << _pid_pitch_I << "," << _pid_pitch_D << ","
              << std::setprecision(2) << alt_rel << ","
              << _airspeed_ms << "," << _throttle_pct << ","
              << std::setprecision(3) << _nav_pitch_deg << ","
              << (target_locked ? 1 : 0) << ","
              << (terminal ? 1 : 0) << ","
              << std::setprecision(1) << dist << "\n";
}

// ── Video recorder ────────────────────────────────────────────────────────────

void SeekerCtrl::_openVideo(int w, int h)
{
    auto now = std::time(nullptr);
    char ts[32];
    std::strftime(ts, sizeof(ts), "%Y%m%d_%H%M%S", std::localtime(&now));
    std::string path = std::string("tracking_") + ts + ".avi";
    _vwriter.open(path, cv::VideoWriter::fourcc('M','J','P','G'), 25.0, {w, h});
    _vwriter_open = _vwriter.isOpened();
    if (_vwriter_open) printf("[REC] recording → %s\n", path.c_str());
    else printf("[REC] Failed to open video writer\n");
}

void SeekerCtrl::_closeVideo()
{
    if (_vwriter_open) {
        _vwriter.release();
        _vwriter_open = false;
        printf("[REC] recording stopped\n");
    }
}

void SeekerCtrl::_writeFrame(const cv::Mat& frame)
{
    if (_vwriter_open) _vwriter.write(frame);
}

// ── run ───────────────────────────────────────────────────────────────────────

void SeekerCtrl::run()
{
    _seeker->open();

    std::deque<double> frame_times;
    double prev_time     = mono_s();
    bool   prev_in_track = false;

    while (true) {
        // ── 1. Grab frame ─────────────────────────────────────────────────────
        cv::Mat frame;
        if (!_seeker->readFrame(frame)) continue;
        if (frame.empty()) { printf("[Ctrl] End of stream.\n"); break; }

        double now = mono_s();
        frame_times.push_back(now - prev_time);
        while (frame_times.size() > 30) frame_times.pop_front();
        prev_time = now;

        double sum_ft = 0;
        for (double t : frame_times) sum_ft += t;
        double fps = frame_times.empty() ? 0.0 : frame_times.size() / sum_ft;

        // ── 2. Track ──────────────────────────────────────────────────────────
        auto [cx, cy] = _seeker->track(frame);
        bool target_locked = (cx >= 0 && cy >= 0);
        double errorx = 0.0, errory = 0.0;
        if (target_locked)
            std::tie(errorx, errory) = _seeker->errorXY(cx, cy, frame.cols, frame.rows);

        // ── 3. Poll MAVLink ───────────────────────────────────────────────────
        _pollRC();
        _pollHeartbeat();
        _pollMavlinkState();
        bool ch6_on  = _ch6Active();
        bool ch6_fell = _prev_ch6_on && !ch6_on;

        // ── 4. Mode management ────────────────────────────────────────────────
        if (_cfg.auto_mode) {
            double dist_to_target = _distToTargetM();
            bool close_enough = dist_to_target < 700.0;
            bool on_last_wp   = (_waypoint_count > 0 &&
                                 _current_wp == _waypoint_count - 1);

            if (ch6_fell) {
                _in_tracking = false;
                _setMode(STABILIZE_MODE);
            } else if (ch6_on) {
                if (close_enough && on_last_wp && target_locked && !_in_tracking) {
                    _setMode(TRACKING_MODE);
                    _in_tracking = true;
                    _tracking_entry_count++;
                } else if (!(close_enough && on_last_wp) && !_in_tracking) {
                    _setMode(AUTO_MODE);
                }
            } else {
                _setMode(STABILIZE_MODE);
            }
        } else {
            if (ch6_fell) {
                _in_tracking = false;
                _setMode(AUTO_MODE);
            } else if (ch6_on) {
                if (target_locked && !_in_tracking) {
                    _setMode(TRACKING_MODE);
                    _in_tracking = true;
                    _tracking_entry_count++;
                }
            }
        }
        _prev_ch6_on = ch6_on;

        // ── 5. CSV / video lifecycle ──────────────────────────────────────────
        if (_in_tracking && !prev_in_track) {
            if (_cfg.debug_log)  _openCsv();
            if (_cfg.record)     _openVideo(frame.cols, frame.rows);
        } else if (!_in_tracking && prev_in_track) {
            _sendTracking(0.0, 0.0);   // zero error on exit
            if (_cfg.debug_log)  _closeCsv();
            if (_cfg.record)     _closeVideo();
        }
        prev_in_track = _in_tracking;

        // ── 6. Feed tracking error ────────────────────────────────────────────
        double alt_dist_m = _alt_msl_m - _target_alt_msl;
        bool in_terminal  = (_term_alt > 0.0 && alt_dist_m <= _term_alt);

        if (_in_tracking && _flight_mode == "TRACKING") {
            if (target_locked) {
                double raw_ex = errorx, raw_ey = errory;
                if (_cfg.input_prediction) {
                    double dt_err = (_prev_err_t > 0.0) ? (now - _prev_err_t) : 0.0;
                    if (dt_err > 0.0 && dt_err < 0.5) {
                        double dx_dt = (raw_ex - _prev_errorx) / dt_err;
                        double dy_dt = (raw_ey - _prev_errory) / dt_err;
                        double lead  = LATENCY_S + PN_LEAD_S;
                        errorx = std::max(-1.0, std::min(1.0, raw_ex + dx_dt * lead));
                        errory = std::max(-1.0, std::min(1.0, raw_ey + dy_dt * lead));
                    }
                }
                _prev_errorx = raw_ex;
                _prev_errory = raw_ey;
                _prev_err_t  = now;
                _last_errorx = errorx;
                _last_errory = errory;
                _lost_count  = 0;
                _sendTracking(errorx, errory);

                double offset_norm = TRK_PITCH_OFFSET / TRK_MAX_DEG;
                if (in_terminal) offset_norm += TRK_TERM_PTCH / TRK_MAX_DEG;
                _logRow(now, errorx, errory - offset_norm, true, in_terminal);
            } else {
                _lost_count++;
                bool committed = _tracking_entry_count >= 3;
                int  limit     = committed ? 30 : 10;
                if (_lost_count >= limit) {
                    _lost_count  = 0;
                    _in_tracking = false;
                    _setMode(AUTO_MODE);
                } else if (committed) {
                    _sendTracking(_last_errorx, _last_errory);
                } else {
                    _sendTracking(0.0, 0.0);
                }
                double lx = committed ? _last_errorx : 0.0;
                double ly = committed ? _last_errory : 0.0;
                _logRow(now, lx, ly, false, in_terminal);
            }
        }

        // ── 7. Annotate display ───────────────────────────────────────────────
        double dist_km   = _distToTargetM() / 1000.0;
        double spd_kmh   = _airspeed_ms * 3.6;
        int    h_frame   = frame.rows;

        char status[256];
        std::snprintf(status, sizeof(status),
            "FPS:%.1f  LOCK:%s  ex=%+.3f ey=%+.3f",
            fps,
            ch6_on ? "ON" : "OFF",
            target_locked ? errorx : 0.0,
            target_locked ? errory : 0.0);

        char desc[256];
        std::snprintf(desc, sizeof(desc),
            "%s:%d, dist %.3fkm, alt %+.0fm, v %.2fkm/h, throttle %.0f%%",
            _flight_mode.c_str(), _current_wp,
            dist_km, alt_dist_m, spd_kmh, (double)_throttle_pct);

        cv::putText(frame, status, {5, h_frame - 40},
                    cv::FONT_HERSHEY_DUPLEX, 0.5, {255, 255, 0}, 2);
        cv::putText(frame, desc,   {5, h_frame - 20},
                    cv::FONT_HERSHEY_DUPLEX, 0.5, {255, 255, 0}, 2);

        _hud->drawHud(true, frame, _lat, _lon, _yaw_deg, _pitch_deg, _roll_deg);

        if (_cfg.record && _in_tracking) _writeFrame(frame);
        cv::imshow(_seeker->windowName(), frame);

        int key = cv::waitKey(1) & 0xFF;
        if (key == 'q') { printf("[Ctrl] Quit.\n"); break; }
        if (key == 'r') {
            _seeker->resetTracker();
            printf("[Ctrl] Tracker reset.\n");
        }
    }

    _closeCsv();
    _closeVideo();
    _seeker->close();
}
