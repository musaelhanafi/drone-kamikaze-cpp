#include "MavlinkConn.hpp"

#include <stdexcept>
#include <cstring>
#include <cstdio>
#include <chrono>
#include <algorithm>

// POSIX
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>

// UDP
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>

// Serial
#include <termios.h>

using namespace std::chrono;

// ── ctor / dtor ───────────────────────────────────────────────────────────────

MavlinkConn::MavlinkConn(const std::string& connection_string, int baud)
    : _conn_str(connection_string), _baud(baud)
{}

MavlinkConn::~MavlinkConn()
{
    close();
}

// ── connect / close ───────────────────────────────────────────────────────────

void MavlinkConn::connect()
{
    _parse_connection_string();
    switch (_type) {
        case ConnType::UDP_IN:  _open_udp_in();  break;
        case ConnType::UDP_OUT: _open_udp_out(); break;
        case ConnType::SERIAL:  _open_serial();  break;
    }
    _rx_stop.store(false);
    _rx_thread = std::thread(&MavlinkConn::_recv_thread_fn, this);
}

void MavlinkConn::close()
{
    _rx_stop.store(true);
    if (_rx_thread.joinable()) {
        _rx_thread.join();
    }
    if (_fd >= 0) {
        ::close(_fd);
        _fd = -1;
    }
}

// ── connection string parser ──────────────────────────────────────────────────

void MavlinkConn::_parse_connection_string()
{
    const auto& s = _conn_str;

    auto split3 = [](const std::string& str, char delim) {
        std::vector<std::string> parts;
        size_t start = 0, pos;
        while ((pos = str.find(delim, start)) != std::string::npos) {
            parts.push_back(str.substr(start, pos - start));
            start = pos + 1;
        }
        parts.push_back(str.substr(start));
        return parts;
    };

    if (s.rfind("udpin:", 0) == 0) {
        _type = ConnType::UDP_IN;
        auto parts = split3(s.substr(6), ':');
        _host = parts.size() > 0 ? parts[0] : "0.0.0.0";
        _port = parts.size() > 1 ? std::stoi(parts[1]) : 14560;
    } else if (s.rfind("udpout:", 0) == 0 || s.rfind("udp:", 0) == 0) {
        _type = ConnType::UDP_OUT;
        size_t off = (s.rfind("udpout:", 0) == 0) ? 7 : 4;
        auto parts = split3(s.substr(off), ':');
        _host = parts.size() > 0 ? parts[0] : "127.0.0.1";
        _port = parts.size() > 1 ? std::stoi(parts[1]) : 14550;
    } else if (s.rfind("serial:", 0) == 0) {
        _type = ConnType::SERIAL;
        auto parts = split3(s.substr(7), ':');
        _device = parts.size() > 0 ? parts[0] : s.substr(7);
        if (parts.size() > 1) _baud = std::stoi(parts[1]);
    } else if (!s.empty() && s[0] == '/') {
        _type   = ConnType::SERIAL;
        _device = s;
    } else {
        throw std::runtime_error("Unknown connection string: " + s);
    }
}

// ── UDP IN ────────────────────────────────────────────────────────────────────

void MavlinkConn::_open_udp_in()
{
    _fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (_fd < 0) throw std::runtime_error("socket() failed");

    int opt = 1;
    ::setsockopt(_fd, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));

    struct sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port   = htons(static_cast<uint16_t>(_port));
    ::inet_pton(AF_INET, _host.c_str(), &addr.sin_addr);

    if (::bind(_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
        throw std::runtime_error("bind() failed on " + _host + ":" + std::to_string(_port));

    // Non-blocking for recv loop
    int flags = ::fcntl(_fd, F_GETFL, 0);
    ::fcntl(_fd, F_SETFL, flags | O_NONBLOCK);

    printf("[MAVLink] UDP IN bound to %s:%d\n", _host.c_str(), _port);
}

// ── UDP OUT ───────────────────────────────────────────────────────────────────

void MavlinkConn::_open_udp_out()
{
    _fd = ::socket(AF_INET, SOCK_DGRAM, 0);
    if (_fd < 0) throw std::runtime_error("socket() failed");

    struct addrinfo hints{}, *res = nullptr;
    hints.ai_family   = AF_INET;
    hints.ai_socktype = SOCK_DGRAM;
    std::string port_str = std::to_string(_port);
    if (::getaddrinfo(_host.c_str(), port_str.c_str(), &hints, &res) != 0)
        throw std::runtime_error("getaddrinfo() failed for " + _host);

    // Store peer address for sends
    _peer_addr_len = res->ai_addrlen;
    std::memcpy(&_peer_addr, res->ai_addr, res->ai_addrlen);
    ::freeaddrinfo(res);
    _peer_known = true;

    int flags = ::fcntl(_fd, F_GETFL, 0);
    ::fcntl(_fd, F_SETFL, flags | O_NONBLOCK);

    printf("[MAVLink] UDP OUT → %s:%d\n", _host.c_str(), _port);
}

// ── Serial ────────────────────────────────────────────────────────────────────

static speed_t baud_to_speed(int baud)
{
    switch (baud) {
        case 9600:   return B9600;
        case 19200:  return B19200;
        case 38400:  return B38400;
        case 57600:  return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        default:     return B57600;
    }
}

void MavlinkConn::_open_serial()
{
    _fd = ::open(_device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (_fd < 0)
        throw std::runtime_error("open() serial failed: " + _device);

    struct termios tty{};
    ::tcgetattr(_fd, &tty);
    speed_t spd = baud_to_speed(_baud);
    ::cfsetispeed(&tty, spd);
    ::cfsetospeed(&tty, spd);
    tty.c_cflag &= ~PARENB;
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CSIZE;
    tty.c_cflag |=  CS8;
    tty.c_cflag &= ~CRTSCTS;
    tty.c_cflag |=  CREAD | CLOCAL;
    tty.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    tty.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL | INLCR);
    tty.c_oflag &= ~OPOST;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 1;
    ::tcsetattr(_fd, TCSANOW, &tty);

    printf("[MAVLink] Serial %s @ %d\n", _device.c_str(), _baud);
}

// ── Receive thread ────────────────────────────────────────────────────────────

void MavlinkConn::_recv_thread_fn()
{
    mavlink_message_t msg;
    mavlink_status_t  status;
    uint8_t buf[1024];

    while (!_rx_stop.load()) {
        ssize_t n;

        if (_type == ConnType::UDP_IN || _type == ConnType::UDP_OUT) {
            struct sockaddr_storage from{};
            socklen_t fromlen = sizeof(from);
            n = ::recvfrom(_fd, buf, sizeof(buf), 0,
                           reinterpret_cast<sockaddr*>(&from), &fromlen);
            if (n > 0 && _type == ConnType::UDP_IN && !_peer_known) {
                std::memcpy(&_peer_addr, &from, fromlen);
                _peer_addr_len = fromlen;
                _peer_known    = true;
            }
        } else {
            n = ::read(_fd, buf, sizeof(buf));
        }

        if (n <= 0) {
            // No data yet — short sleep to avoid busy-spin
            std::this_thread::sleep_for(milliseconds(1));
            continue;
        }

        for (ssize_t i = 0; i < n; i++) {
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                std::unique_lock<std::mutex> lk(_cache_mtx);
                _cache[msg.msgid] = msg;
                _cache_seq[msg.msgid] = ++_cache_gen;
                lk.unlock();
                _cache_cv.notify_all();
            }
        }
    }
}

// ── Send ──────────────────────────────────────────────────────────────────────

void MavlinkConn::_send_bytes(const uint8_t* buf, size_t len)
{
    if (_fd < 0) return;

    if (_type == ConnType::UDP_IN) {
        if (!_peer_known) return;   // haven't received a packet yet
        ::sendto(_fd, buf, len, 0,
                 reinterpret_cast<const sockaddr*>(&_peer_addr),
                 _peer_addr_len);
    } else if (_type == ConnType::UDP_OUT) {
        ::sendto(_fd, buf, len, 0,
                 reinterpret_cast<const sockaddr*>(&_peer_addr),
                 _peer_addr_len);
    } else {
        ::write(_fd, buf, len);
    }
}

void MavlinkConn::send_message(const mavlink_message_t& msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
    _send_bytes(buf, len);
}

// ── Cache accessors ───────────────────────────────────────────────────────────

bool MavlinkConn::get_message(uint32_t msgid, mavlink_message_t& out)
{
    std::lock_guard<std::mutex> lk(_cache_mtx);
    auto it = _cache.find(msgid);
    if (it == _cache.end()) return false;
    out = it->second;
    return true;
}

bool MavlinkConn::recv_match(uint32_t msgid, mavlink_message_t& out, double timeout_s)
{
    auto deadline = steady_clock::now() +
                    duration_cast<steady_clock::duration>(
                        duration<double>(timeout_s));

    std::unique_lock<std::mutex> lk(_cache_mtx);
    uint64_t gen_before = 0;
    {
        auto it = _cache_seq.find(msgid);
        if (it != _cache_seq.end()) gen_before = it->second;
    }

    while (true) {
        auto it_seq = _cache_seq.find(msgid);
        if (it_seq != _cache_seq.end() && it_seq->second > gen_before) {
            out = _cache.at(msgid);
            return true;
        }
        if (_cache_cv.wait_until(lk, deadline) == std::cv_status::timeout)
            return false;
    }
}

// ── wait_heartbeat ────────────────────────────────────────────────────────────

bool MavlinkConn::wait_heartbeat(double timeout_s)
{
    mavlink_message_t msg;
    if (!recv_match(MsgId::HEARTBEAT, msg, timeout_s))
        return false;

    mavlink_heartbeat_t hb;
    mavlink_msg_heartbeat_decode(&msg, &hb);
    target_system    = msg.sysid;
    target_component = msg.compid;
    printf("[MAVLink] Heartbeat from sysid=%d compid=%d\n",
           target_system, target_component);
    return true;
}
