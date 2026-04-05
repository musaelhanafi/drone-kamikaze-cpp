#pragma once
#include <string>
#include <mutex>
#include <condition_variable>
#include <thread>
#include <atomic>
#include <unordered_map>
#include <functional>
#include <cstdint>

// POSIX socket types needed for _peer_addr / _peer_addr_len members
#include <sys/socket.h>

// MAVLink C headers (ardupilotmega dialect)
#include <ardupilotmega/mavlink.h>

// ── MavlinkConn ──────────────────────────────────────────────────────────────
// Manages a single MAVLink link over UDP or serial.
//
// Supported connection strings (same as pymavlink):
//   "udpin:0.0.0.0:14560"   — bind UDP on port 14560, reply to first sender
//   "udp:127.0.0.1:14550"   — connect UDP to remote host:port
//   "udpout:host:port"       — alias for udp:
//   "/dev/ttyUSB0"           — serial, baud from ctor
//   "serial:/dev/ttyUSB0:57600"
//
// A background receive thread parses incoming MAVLink frames and stores the
// most recent message of each msgid in a cache.  Callers poll the cache with
// get_message() or block on recv_match().
// ─────────────────────────────────────────────────────────────────────────────
class MavlinkConn {
public:
    explicit MavlinkConn(const std::string& connection_string, int baud = 57600);
    ~MavlinkConn();

    // Open socket/serial and start the receive thread.
    void connect();
    void close();

    // Block until a HEARTBEAT is received (or timeout expires).
    // Returns true on success.  Sets target_system / target_component.
    bool wait_heartbeat(double timeout_s = 30.0);

    // Send a packed MAVLink message.
    void send_message(const mavlink_message_t& msg);

    // Non-blocking: copy the cached message into *out* and return true,
    // or return false if no message of that ID has arrived yet.
    bool get_message(uint32_t msgid, mavlink_message_t& out);

    // Blocking: wait up to timeout_s for the next message of the given ID.
    // Returns true on success.
    bool recv_match(uint32_t msgid, mavlink_message_t& out, double timeout_s = 1.0);

    // MAVLink addressing
    uint8_t target_system    = 1;
    uint8_t target_component = 1;
    uint8_t src_system       = 255;  // GCS
    uint8_t src_component    = 0;

private:
    enum class ConnType { UDP_IN, UDP_OUT, SERIAL };

    void _parse_connection_string();
    void _open_udp_in();
    void _open_udp_out();
    void _open_serial();
    void _recv_thread_fn();

    // Write raw bytes to the link
    void _send_bytes(const uint8_t* buf, size_t len);

    std::string  _conn_str;
    int          _baud;
    ConnType     _type    = ConnType::UDP_IN;
    std::string  _host;
    int          _port    = 0;
    std::string  _device;

    // Socket / fd
    int          _fd      = -1;

    // For udpin: store the sender's address once received
    bool         _peer_known = false;
    struct sockaddr_storage _peer_addr{};
    socklen_t    _peer_addr_len = 0;

    // Receive thread
    std::thread              _rx_thread;
    std::atomic<bool>        _rx_stop{false};

    // Message cache: msgid → latest message
    mutable std::mutex                           _cache_mtx;
    std::condition_variable                      _cache_cv;
    std::unordered_map<uint32_t, mavlink_message_t> _cache;

    // Track which msgids have been freshly received (for recv_match blocking)
    std::unordered_map<uint32_t, uint64_t> _cache_seq;
    uint64_t _cache_gen = 0;
};

// ── Convenience message-ID constants ────────────────────────────────────────
// Re-export the MAVLink IDs with readable names for SeekerCtrl.
namespace MsgId {
    constexpr uint32_t HEARTBEAT            = MAVLINK_MSG_ID_HEARTBEAT;
    constexpr uint32_t ATTITUDE             = MAVLINK_MSG_ID_ATTITUDE;
    constexpr uint32_t SERVO_OUTPUT_RAW     = MAVLINK_MSG_ID_SERVO_OUTPUT_RAW;
    constexpr uint32_t GLOBAL_POSITION_INT  = MAVLINK_MSG_ID_GLOBAL_POSITION_INT;
    constexpr uint32_t VFR_HUD              = MAVLINK_MSG_ID_VFR_HUD;
    constexpr uint32_t RC_CHANNELS          = MAVLINK_MSG_ID_RC_CHANNELS;
    constexpr uint32_t MISSION_CURRENT      = MAVLINK_MSG_ID_MISSION_CURRENT;
    constexpr uint32_t MISSION_COUNT        = MAVLINK_MSG_ID_MISSION_COUNT;
    constexpr uint32_t NAV_CONTROLLER_OUTPUT= MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT;
    constexpr uint32_t HOME_POSITION        = MAVLINK_MSG_ID_HOME_POSITION;
    constexpr uint32_t PARAM_VALUE          = MAVLINK_MSG_ID_PARAM_VALUE;
    constexpr uint32_t PID_TUNING           = MAVLINK_MSG_ID_PID_TUNING;
    constexpr uint32_t DEBUG_VECT           = MAVLINK_MSG_ID_DEBUG_VECT;
}
