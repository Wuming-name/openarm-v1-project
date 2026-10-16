// OpenArm V1 static-pose torque recorder. Linux / C++17.
// Default: read-only diagnosis. Motion requires a prepared/reviewed plan AND --execute.
// Feedback torque is a motor-reported estimate, not an external torque measurement.
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/error.h>
#include <linux/can/raw.h>

#include <net/if.h>
#include <sys/socket.h>
#include <unistd.h>
#include <fcntl.h>
#include <poll.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstring>
#include <deque>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
#include <filesystem>
#include <cstdlib>
#include <nlohmann/json.hpp>

namespace {

using Clock = std::chrono::steady_clock;
using WallClock = std::chrono::system_clock;
using Vec = std::array<double, 7>;

constexpr int N = 7;
std::string CAN_INTERFACE = "can0";
using Json = nlohmann::json;
Vec TORQUE_ALARM{}, SPEED_ALARM{};
double TRACK_ALARM = 0.16;
constexpr bool CAN_FD = true;

// Keep consistent with the actual motors.
constexpr std::array<uint32_t, N> SEND_ID{1, 2, 3, 4, 5, 6, 7};
constexpr std::array<uint32_t, N> RECV_ID{
    0x11, 0x12, 0x13, 0x14, 0x15, 0x16, 0x17
};

// Protocol encoding ranges, NOT structural safety ratings.
constexpr Vec PMAX{12.5, 12.5, 12.5, 12.5, 12.5, 12.5, 12.5};
constexpr Vec VMAX{45, 45, 10, 10, 30, 30, 30};
constexpr Vec TMAX{54, 54, 28, 28, 10, 10, 10};

Vec QMIN{};
Vec QMAX{};

Vec KP{};
Vec KD{};

// These thresholds are starting values, not calibrated acceptance standards.
constexpr double PERIOD_S = 0.050;
constexpr double MAX_SAMPLE_GAP_S = 0.075;
constexpr double TRANSACTION_LIMIT_S = 0.150;
constexpr double CONTROL_LIMIT_S = 0.150;

constexpr double MAX_KERNEL_AGE_S = 0.025;
constexpr double MAX_JOINT_RX_SKEW_S = 0.020;
constexpr double MAX_CLOCK_OFFSET_CHANGE_S = 0.010;




// Startup / settling quality, separate from emergency limits.

// Crossing quality.


// Allowed difference between |positive speed| and |negative speed|.

volatile std::sig_atomic_t stop_requested = 0;

void signal_handler(int) {
    stop_requested = 1;
}

void check_stop() {
    if (stop_requested)
        throw std::runtime_error("用户请求停止");
}

double elapsed(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double>(a - b).count();
}

double wall_seconds() {
    return std::chrono::duration<double>(
        WallClock::now().time_since_epoch()
    ).count();
}

Clock::duration duration_s(double value) {
    return std::chrono::duration_cast<Clock::duration>(
        std::chrono::duration<double>(value)
    );
}

std::string system_error(const char* operation) {
    return std::string(operation) + ": " + std::strerror(errno);
}

void validate_target(const Vec& q) {
    for (int i = 0; i < N; ++i) {
        if (!std::isfinite(q[i]) ||
            q[i] < QMIN[i] || q[i] > QMAX[i]) {
            throw std::runtime_error(
                "J" + std::to_string(i + 1) +
                " 指令角度越界或无效，q=" + std::to_string(q[i])
            );
        }
    }
}

struct State {
    Vec q{}, dq{}, tau{}, mos{}, coil{};
    Vec rx_time{};
    std::array<int, N> status{};
    double time_s = 0;
    double transaction_ms = 0;
    double rx_skew_ms = 0;
    double max_kernel_age_ms = 0;
    bool sample_valid = false;
};

struct Diagnostics {
    double start_s = 0;
    double prepare_ms = 0;
    double send_ms = 0;
    double wait_ms = 0;
    double max_poll_ms = 0;
    double max_recv_ms = 0;
    double max_process_ms = 0;
    double total_ms = 0;
    double cycle_gap_ms = 0;
    double wake_late_ms = 0;

    uint64_t frames = 0;
    uint64_t accepted = 0;
    uint64_t rejected = 0;
    uint64_t old_frames = 0;
    int retries = 0;

    bool success = false;
    std::string note;
};


// SocketCAN communication, no SDK response counter dependency.
class Arm {
public:
    explicit Arm(const std::string& diagnostic_path)
        : start_(Clock::now()),
          wall_start_(wall_seconds()),
          diagnostic_path_(diagnostic_path) {
        try {
            fd_ = ::socket(
                PF_CAN, SOCK_RAW | SOCK_NONBLOCK | SOCK_CLOEXEC, CAN_RAW
            );
            if (fd_ < 0)
                throw std::runtime_error(system_error("socket"));

            int enabled = 1;
            if (CAN_FD &&
                ::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FD_FRAMES,
                             &enabled, sizeof(enabled)) < 0)
                throw std::runtime_error(system_error("CAN_RAW_FD_FRAMES"));

            if (::setsockopt(fd_, SOL_SOCKET, SO_TIMESTAMPNS,
                             &enabled, sizeof(enabled)) < 0)
                throw std::runtime_error(system_error("SO_TIMESTAMPNS"));

            if (::setsockopt(fd_, SOL_SOCKET, SO_RXQ_OVFL,
                             &enabled, sizeof(enabled)) < 0)
                throw std::runtime_error(system_error("SO_RXQ_OVFL"));

            std::array<can_filter, N> filters{};
            for (int i = 0; i < N; ++i) {
                filters[i].can_id = RECV_ID[i];
                filters[i].can_mask =
                    CAN_SFF_MASK | CAN_EFF_FLAG | CAN_RTR_FLAG;
            }

            if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_FILTER,
                             filters.data(), sizeof(filters)) < 0)
                throw std::runtime_error(system_error("CAN_RAW_FILTER"));

            can_err_mask_t error_mask = CAN_ERR_MASK;
            if (::setsockopt(fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER,
                             &error_mask, sizeof(error_mask)) < 0)
                throw std::runtime_error(system_error("CAN_RAW_ERR_FILTER"));

            ifreq request{};
            if (CAN_INTERFACE.size() >= IFNAMSIZ)
                throw std::runtime_error("CAN 接口名称过长");

            std::strncpy(request.ifr_name, CAN_INTERFACE.c_str(), IFNAMSIZ - 1);
            if (::ioctl(fd_, SIOCGIFINDEX, &request) < 0)
                throw std::runtime_error(system_error("SIOCGIFINDEX"));

            sockaddr_can address{};
            address.can_family = AF_CAN;
            address.can_ifindex = request.ifr_ifindex;

            if (::bind(fd_, reinterpret_cast<sockaddr*>(&address),
                       sizeof(address)) < 0)
                throw std::runtime_error(system_error("bind"));

            diagnostics_.reserve(20000);
            live_.open(diagnostic_path_ + ".jsonl");
            if (!live_) throw std::runtime_error("cannot open live diagnostic log");

        } catch (...) {
            if (fd_ >= 0)
                ::close(fd_);
            fd_ = -1;
            throw;
        }
    }

    ~Arm() {
        // In diagnose mode this flag never becomes true.
        if (may_be_enabled_) {
            bool failed = false;
            // Best effort, bounded number of writes; no acknowledgement claim.
            for (int pass = 0; pass < 3; ++pass) {
                for (int i = 0; i < N; ++i) {
                    try {
                        special(i, 0xFD);
                    } catch (...) {
                        failed = true;
                    }
                }
            }
            std::cerr << (
                failed
                ? "警告：部分失能命令发送失败。\n"
                : "已尝试发送七关节失能命令，未确认电机实际失能。\n"
            );
        }

        if (fd_ >= 0)
            ::close(fd_);

        save_diagnostics();
    }

    Arm(const Arm&) = delete;
    Arm& operator=(const Arm&) = delete;

    double now_s() const {
        return elapsed(Clock::now(), start_);
    }

    State query(bool require_enabled = false) {
        return transaction(nullptr, require_enabled);
    }

    State command(const Vec& target) {
        validate_target(target);
        return transaction(&target, true);
    }

    State enable_at(const Vec& target) {
        validate_target(target);
        may_be_enabled_ = true;
        // Preload current-position MIT targets while still disabled, before 0xFC.
        // Refuse firmware that unexpectedly reports enabled or changed pose.
        const State primed = transaction(&target, false);
        if (!primed.sample_valid) throw std::runtime_error("preload timing invalid");
        for (int i = 0; i < N; ++i) {
            if (primed.status[i] != 0 || std::abs(primed.q[i]-target[i]) > 0.005 ||
                std::abs(primed.dq[i]) > 0.04)
                throw std::runtime_error("preload changed enable state/pose; stopping");
        }
        for (int i = 0; i < N; ++i) {
            check_stop();
            special(i, 0xFC);
        }
        return command(target);
    }


    Json verify_parameters() {
        Json result = Json::array();
        const std::array<int, 7> rids{7, 8, 10, 21, 22, 23, 35};
        for (int i = 0; i < N; ++i) {
            Json row;
            for (int rid : rids) {
                check_stop(); check_clock();
                Frame stale;
                int discarded = 0;
                while (receive(stale)) {
                    if (++discarded > 128) throw std::runtime_error("parameter RX backlog");
                }
                std::array<uint8_t, 8> bytes{};
                bytes[0] = uint8_t(SEND_ID[i]); bytes[2] = 0x33; bytes[3] = uint8_t(rid);
                const double sent = wall_seconds();
                send(0x7FF, bytes);
                const auto start = Clock::now();
                bool found = false;
                while (elapsed(Clock::now(), start) < TRANSACTION_LIMIT_S) {
                    check_stop(); check_clock();
                    Frame frame;
                    if (!receive(frame)) { std::this_thread::sleep_for(std::chrono::milliseconds(1)); continue; }
                    const auto& d = frame.data;
                    if (frame.id != RECV_ID[i] || frame.len != 8 || frame.timestamp < sent ||
                        frame.age > MAX_KERNEL_AGE_S || d[0] != SEND_ID[i] || d[1] != 0 ||
                        d[2] != 0x33 || d[3] != rid) continue;
                    uint32_t raw = uint32_t(d[4]) | (uint32_t(d[5]) << 8) |
                                   (uint32_t(d[6]) << 16) | (uint32_t(d[7]) << 24);
                    double value;
                    if (rid == 21 || rid == 22 || rid == 23) {
                        float number; std::memcpy(&number, &raw, sizeof(number)); value = number;
                    } else value = raw;
                    if (!std::isfinite(value)) throw std::runtime_error("invalid motor parameter");
                    row[std::to_string(rid)] = value; found = true; break;
                }
                if (!found) throw std::runtime_error("J" + std::to_string(i+1) + " parameter timeout RID=" + std::to_string(rid));
            }
            if (row["7"].get<double>() != RECV_ID[i] || row["8"].get<double>() != SEND_ID[i] ||
                row["10"].get<double>() != 1 || row["35"].get<double>() != 9 ||
                std::abs(row["21"].get<double>() - PMAX[i]) > 1e-4 ||
                std::abs(row["22"].get<double>() - VMAX[i]) > 1e-4 ||
                std::abs(row["23"].get<double>() - TMAX[i]) > 1e-4)
                throw std::runtime_error("J" + std::to_string(i+1) + " ID/mode/baud/scaling differs from verified arm");
            result.push_back(row);
        }
        return result;
    }

    void record_cycle(double gap_ms, double wake_ms) {
        if (!diagnostics_.empty()) {
            diagnostics_.back().cycle_gap_ms = gap_ms;
            diagnostics_.back().wake_late_ms = wake_ms;
        }
    }

private:
    struct Frame {
        canid_t id = 0;
        unsigned len = 0;
        std::array<uint8_t, 64> data{};
        double timestamp = 0;
        double age = 0;
    };

    void check_clock() const {
        const double wall_delta = wall_seconds() - wall_start_;
        const double steady_delta = now_s();
        if (std::abs(wall_delta - steady_delta) >
            MAX_CLOCK_OFFSET_CHANGE_S) {
            throw std::runtime_error(
                "系统实时时钟相对单调时钟变化超过 10 ms；"
                "软件时间戳不可继续用于本次实验"
            );
        }
    }

    void send(canid_t id, const std::array<uint8_t, 8>& data) {
        ssize_t result;
        if (CAN_FD) {
            canfd_frame frame{};
            frame.can_id = id;
            frame.len = 8;
            frame.flags = CANFD_BRS;
            std::copy(data.begin(), data.end(), frame.data);
            result = ::write(fd_, &frame, CANFD_MTU);
            if (result != CANFD_MTU)
                throw std::runtime_error(system_error("CAN-FD write"));
        } else {
            can_frame frame{};
            frame.can_id = id;
            frame.can_dlc = 8;
            std::copy(data.begin(), data.end(), frame.data);
            result = ::write(fd_, &frame, CAN_MTU);
            if (result != CAN_MTU)
                throw std::runtime_error(system_error("CAN write"));
        }
    }

    void special(int joint, uint8_t value) {
        std::array<uint8_t, 8> data;
        data.fill(0xFF);
        data.back() = value;
        send(SEND_ID[joint], data);
    }

    void refresh(int joint) {
        std::array<uint8_t, 8> data{};
        data[0] = static_cast<uint8_t>(SEND_ID[joint]);
        data[1] = static_cast<uint8_t>(SEND_ID[joint] >> 8);
        data[2] = 0xCC;
        send(0x7FF, data);
    }

    static uint64_t encode(double value, double low,
                           double high, int bits) {
        if (!std::isfinite(value) || value < low || value > high)
            throw std::runtime_error("MIT 编码值超出协议范围");

        return static_cast<uint64_t>(
            (value - low) / (high - low) *
            static_cast<double>((uint64_t{1} << bits) - 1)
        );
    }

    void mit(int joint, double position) {
        // 64-bit sequence: position16, velocity12, KP12, KD12, torque12.
        uint64_t packed =
            encode(position, -PMAX[joint], PMAX[joint], 16);

        packed = (packed << 12) |
                 encode(0, -VMAX[joint], VMAX[joint], 12);
        packed = (packed << 12) | encode(KP[joint], 0, 500, 12);
        packed = (packed << 12) | encode(KD[joint], 0, 5, 12);
        packed = (packed << 12) |
                 encode(0, -TMAX[joint], TMAX[joint], 12);

        std::array<uint8_t, 8> data{};
        for (int b = 7; b >= 0; --b) {
            data[b] = static_cast<uint8_t>(packed & 0xFF);
            packed >>= 8;
        }
        send(SEND_ID[joint], data);
    }

    bool receive(Frame& out) {
        canfd_frame frame{};
        iovec vector{&frame, sizeof(frame)};

        alignas(cmsghdr) char control[
            CMSG_SPACE(sizeof(timespec)) +
            CMSG_SPACE(sizeof(uint32_t))
        ]{};

        msghdr message{};
        message.msg_iov = &vector;
        message.msg_iovlen = 1;
        message.msg_control = control;
        message.msg_controllen = sizeof(control);

        const ssize_t size =
            ::recvmsg(fd_, &message, MSG_DONTWAIT);

        if (size < 0) {
            if (errno == EAGAIN || errno == EWOULDBLOCK)
                return false;
            if (errno == EINTR) {
                check_stop();
                return false;
            }
            throw std::runtime_error(system_error("recvmsg"));
        }

        if (message.msg_flags & (MSG_TRUNC | MSG_CTRUNC))
            throw std::runtime_error("CAN 报文或辅助时间戳被截断");

        if (size != CAN_MTU && size != CANFD_MTU)
            throw std::runtime_error("CAN 帧长度异常");

        out.id = frame.can_id;
        out.len = frame.len;
        if (out.len > 64)
            throw std::runtime_error("CAN 数据长度异常");

        std::copy(frame.data, frame.data + out.len, out.data.begin());

        bool timestamp_found = false;
        for (cmsghdr* c = CMSG_FIRSTHDR(&message);
             c; c = CMSG_NXTHDR(&message, c)) {

            if (c->cmsg_level != SOL_SOCKET)
                continue;

            if (c->cmsg_type == SCM_TIMESTAMPNS) {
                timespec ts{};
                std::memcpy(&ts, CMSG_DATA(c), sizeof(ts));
                out.timestamp =
                    static_cast<double>(ts.tv_sec) +
                    static_cast<double>(ts.tv_nsec) * 1e-9;
                timestamp_found = true;
            }

            if (c->cmsg_type == SO_RXQ_OVFL) {
                uint32_t dropped = 0;
                std::memcpy(&dropped, CMSG_DATA(c), sizeof(dropped));
                if (dropped != last_drop_count_) {
                    last_drop_count_ = dropped;
                    throw std::runtime_error(
                        "当前 CAN socket 接收队列出现丢包"
                    );
                }
            }
        }

        if (!timestamp_found)
            throw std::runtime_error("没有收到内核软件时间戳");

        out.age = wall_seconds() - out.timestamp;
        if (out.age < -0.005)
            throw std::runtime_error("接收时间戳超前，系统时钟异常");

        if (out.id & CAN_ERR_FLAG)
            throw std::runtime_error(
                "收到 CAN 错误帧，停止本次实验；检查接口统计"
            );

        return true;
    }

    static double decode(unsigned value, double maximum, int bits) {
        const double denominator = (1u << bits) - 1u;
        return -maximum + 2 * maximum * value / denominator;
    }

    bool parse(const Frame& frame, State& state, int& joint) {
        if (frame.id & (CAN_EFF_FLAG | CAN_RTR_FLAG))
            return false;

        joint = -1;
        for (int i = 0; i < N; ++i)
            if ((frame.id & CAN_SFF_MASK) == RECV_ID[i])
                joint = i;

        if (joint < 0 || frame.len != 8)
            return false;

        const auto& d = frame.data;

        if ((d[0] & 0x0F) != SEND_ID[joint])
            return false;

        // Recognize the parameter-response signature seen in prior captures.
        // Do not identify parameter frames from d[2] alone.
        // A position byte can legitimately equal 0x33 or 0x55.
        const bool parameter_signature =
            d[0] == SEND_ID[joint] &&
            d[1] == 0 &&
            (d[2] == 0x33 || d[2] == 0x55);

        if (parameter_signature)
            return false;

        const int status = d[0] >> 4;
        if (status >= 8) {
            throw std::runtime_error(
                "J" + std::to_string(joint + 1) +
                " 电机故障状态码=" + std::to_string(status)
            );
        }
        if (status != 0 && status != 1)
            return false;

        const unsigned position = (unsigned(d[1]) << 8) | d[2];
        const unsigned velocity = (unsigned(d[3]) << 4) | (d[4] >> 4);
        const unsigned torque = (unsigned(d[4] & 15) << 8) | d[5];

        state.q[joint] = decode(position, PMAX[joint], 16);
        state.dq[joint] = decode(velocity, VMAX[joint], 12);
        state.tau[joint] = decode(torque, TMAX[joint], 12);
        state.status[joint] = status;
        state.mos[joint] = d[6];
        state.coil[joint] = d[7];
        state.rx_time[joint] = frame.timestamp - wall_start_;

        return std::isfinite(state.q[joint]) &&
               std::isfinite(state.dq[joint]) &&
               std::isfinite(state.tau[joint]);
    }

    State transaction(const Vec* target, bool require_enabled) {
        Diagnostics diag;
        diag.start_s = now_s();
        const auto transaction_begin = Clock::now();

        try {
            check_stop();
            check_clock();

            // Bounded cleanup. Discard already-queued frames.
            const auto prepare_begin = Clock::now();
            bool queue_empty = false;

            for (int n = 0; n < 128; ++n) {
                check_stop();
                Frame frame;
                if (!receive(frame)) {
                    queue_empty = true;
                    break;
                }
            }

            if (!queue_empty)
                throw std::runtime_error(
                    "接收缓存持续积压，本轮不发送运动指令"
                );

            diag.prepare_ms =
                1000 * elapsed(Clock::now(), prepare_begin);

            State state;
            std::array<bool, N> updated{};
            Vec sent_wall{};

            const auto send_begin = Clock::now();
            for (int i = 0; i < N; ++i) {
                check_stop();

                if (elapsed(Clock::now(), transaction_begin) >
                    TRANSACTION_LIMIT_S)
                    throw std::runtime_error("发送阶段发生严重调度延迟");

                sent_wall[i] = wall_seconds();
                if (target)
                    mit(i, (*target)[i]);
                else
                    refresh(i);
            }

            diag.send_ms = 1000 * elapsed(Clock::now(), send_begin);
            const auto wait_begin = Clock::now();
            auto next_retry = wait_begin + duration_s(0.030);

            while (true) {
                check_stop();
                check_clock();

                if (elapsed(Clock::now(), transaction_begin) >
                    TRANSACTION_LIMIT_S)
                    break;

                pollfd descriptor{};
                descriptor.fd = fd_;
                descriptor.events = POLLIN;

                const auto poll_begin = Clock::now();
                const int result = ::poll(&descriptor, 1, 2);
                diag.max_poll_ms = std::max(
                    diag.max_poll_ms,
                    1000 * elapsed(Clock::now(), poll_begin)
                );

                if (result < 0 && errno != EINTR)
                    throw std::runtime_error(system_error("poll"));
                if (result < 0)
                    continue;

                if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))
                    throw std::runtime_error("CAN socket 状态异常");

                for (int n = 0; n < 64; ++n) {
                    if (elapsed(Clock::now(), transaction_begin) >
                        TRANSACTION_LIMIT_S)
                        break;

                    Frame frame;
                    const auto recv_begin = Clock::now();
                    const bool got = receive(frame);
                    diag.max_recv_ms = std::max(
                        diag.max_recv_ms,
                        1000 * elapsed(Clock::now(), recv_begin)
                    );
                    if (!got)
                        break;

                    ++diag.frames;
                    const auto process_begin = Clock::now();

                    int joint = -1;
                    State decoded = state;
                    const bool valid = parse(frame, decoded, joint);

                    if (!valid) {
                        ++diag.rejected;
                    } else if (frame.timestamp < sent_wall[joint]) {
                        // Timestamp precedes this joint's send operation.
                        ++diag.old_frames;
                    } else {
                        if (require_enabled && decoded.status[joint] != 1)
                            throw std::runtime_error(
                                "J" + std::to_string(joint + 1) +
                                " 未报告使能状态"
                            );

                        state = decoded;
                        updated[joint] = true;
                        ++diag.accepted;

                        state.max_kernel_age_ms = std::max(
                            state.max_kernel_age_ms,
                            1000 * std::max(0.0, frame.age)
                        );
                    }

                    diag.max_process_ms = std::max(
                        diag.max_process_ms,
                        1000 * elapsed(Clock::now(), process_begin)
                    );
                }

                const bool complete =
                    std::all_of(updated.begin(), updated.end(),
                                [](bool x) { return x; });

                const auto now = Clock::now();

                // Reject late completion, even when all frames were collected.
                if (elapsed(now, transaction_begin) >
                    TRANSACTION_LIMIT_S)
                    break;

                if (complete) {
                    const auto bounds = std::minmax_element(
                        state.rx_time.begin(), state.rx_time.end()
                    );

                    state.rx_skew_ms = 1000 * (*bounds.second - *bounds.first);
                    state.time_s = *bounds.second;
                    state.transaction_ms =
                        1000 * elapsed(now, transaction_begin);

                    state.sample_valid =
                        state.transaction_ms <= 1000 * PERIOD_S &&
                        state.rx_skew_ms <= 1000 * MAX_JOINT_RX_SKEW_S &&
                        state.max_kernel_age_ms <=
                            1000 * MAX_KERNEL_AGE_S;

                    diag.wait_ms = 1000 * elapsed(now, wait_begin);
                    diag.total_ms = state.transaction_ms;
                    diag.success = true;
                    diag.note = state.sample_valid
                        ? "ok" : "complete_but_quality_invalid";
                    diagnostics_.push_back(diag);
                    live_ << Json{{"start_s", diag.start_s}, {"total_ms", diag.total_ms},
                                  {"accepted", diag.accepted}, {"retries", diag.retries},
                                  {"success", diag.success}, {"note", diag.note}}.dump() << std::endl;
                    if (!live_) throw std::runtime_error("live log write failed");
                    return state;
                }

                if (now >= next_retry && diag.retries < 2) {
                    for (int i = 0; i < N; ++i) {
                        if (elapsed(Clock::now(), transaction_begin) >
                            TRANSACTION_LIMIT_S)
                            break;

                        if (!updated[i])
                            refresh(i);
                    }
                    ++diag.retries;
                    next_retry = Clock::now() + duration_s(0.040);
                }
            }

            std::ostringstream reason;
            reason << "反馈事务超时，缺少有效状态：";
            bool missing = false;
            for (int i = 0; i < N; ++i) {
                if (!updated[i]) {
                    missing = true;
                    reason << " J" << i + 1;
                }
            }
            if (!missing)
                reason << "无，但事务超过期限";

            throw std::runtime_error(reason.str());

        } catch (const std::exception& error) {
            diag.total_ms =
                1000 * elapsed(Clock::now(), transaction_begin);
            diag.note = error.what();
            diagnostics_.push_back(diag);
                    live_ << Json{{"start_s", diag.start_s}, {"total_ms", diag.total_ms},
                                  {"accepted", diag.accepted}, {"retries", diag.retries},
                                  {"success", diag.success}, {"note", diag.note}}.dump() << std::endl;
                    if (!live_) throw std::runtime_error("live log write failed");

            std::ostringstream message;
            message << error.what()
                    << "；总耗时=" << diag.total_ms
                    << " ms，准备=" << diag.prepare_ms
                    << " ms，发送=" << diag.send_ms
                    << " ms，最长poll=" << diag.max_poll_ms
                    << " ms，最长recvmsg=" << diag.max_recv_ms
                    << " ms，最长解析处理=" << diag.max_process_ms
                    << " ms，报文数=" << diag.frames
                    << "，有效状态数=" << diag.accepted;

            throw std::runtime_error(message.str());
        }
    }

    void save_diagnostics() noexcept {
        try {
            std::ofstream out(diagnostic_path_);
            out << std::fixed << std::setprecision(6);
            out <<
                "start_s,prepare_ms,send_ms,wait_ms,max_poll_ms,"
                "max_recv_ms,max_process_ms,total_ms,cycle_gap_ms,"
                "wake_late_ms,frames,accepted,rejected,old_frames,"
                "retries,success,note\n";

            for (const auto& d : diagnostics_) {
                std::string note = d.note;
                for (char& c : note)
                    if (c == ',' || c == '\n' || c == '\r')
                        c = ' ';

                out << d.start_s << ',' << d.prepare_ms << ','
                    << d.send_ms << ',' << d.wait_ms << ','
                    << d.max_poll_ms << ',' << d.max_recv_ms << ','
                    << d.max_process_ms << ',' << d.total_ms << ','
                    << d.cycle_gap_ms << ',' << d.wake_late_ms << ','
                    << d.frames << ',' << d.accepted << ','
                    << d.rejected << ',' << d.old_frames << ','
                    << d.retries << ',' << d.success << ','
                    << note << '\n';
            }
            out.flush();
            if (!out)
                std::cerr << "诊断 CSV 保存失败。\n";
        } catch (...) {
            std::cerr << "诊断 CSV 保存异常。\n";
        }
    }

    int fd_ = -1;
    uint32_t last_drop_count_ = 0;
    bool may_be_enabled_ = false;

    Clock::time_point start_;
    double wall_start_;
    std::string diagnostic_path_;
    std::vector<Diagnostics> diagnostics_;
    std::ofstream live_;
};

void require_disabled(const State& s) {
    for (int i = 0; i < N; ++i)
        if (s.status[i] != 0)
            throw std::runtime_error(
                "J" + std::to_string(i + 1) +
                " 当前报告使能；请先按既有方式失能，"
                "本程序不会在查询阶段自动改变电机状态"
            );
}

class Cycle {
public:
    void begin() {
        check_stop();
        const auto now = Clock::now();
        gap_ms = have_previous_
            ? 1000 * elapsed(now, previous_) : 0;

        if (have_previous_ && gap_ms > 1000 * CONTROL_LIMIT_S)
            throw std::runtime_error("控制循环间隔超过 150 ms");

        current_ = now;
        previous_ = now;
        have_previous_ = true;
    }

    void finish(Arm& arm) {
        const auto due = current_ + duration_s(PERIOD_S);
        if (Clock::now() < due)
            std::this_thread::sleep_until(due);

        const auto now = Clock::now();
        wake_ms = 1000 * std::max(0.0, elapsed(now, due));
        arm.record_cycle(gap_ms, wake_ms);
        check_stop();

        if (elapsed(now, current_) > CONTROL_LIMIT_S)
            throw std::runtime_error(
                "控制循环/休眠唤醒超过 150 ms；唤醒延迟=" +
                std::to_string(wake_ms) + " ms"
            );
    }

    double gap_ms = 0;
    double wake_ms = 0;

private:
    Clock::time_point current_{}, previous_{};
    bool have_previous_ = false;
};

// Static-pose acquisition replaces all crossing/interpolation code from the old file.
namespace fs = std::filesystem;

Json read_json(const fs::path& path) {
    std::ifstream in(path);
    if (!in) throw std::runtime_error("cannot read " + path.string());
    Json value; in >> value; return value;
}
void write_json(const fs::path& path, const Json& value) {
    std::ofstream out(path); out << value.dump(2) << '\n'; out.flush();
    if (!out) throw std::runtime_error("cannot write " + path.string());
}
Vec vector7(const Json& parent, const std::string& key) {
    const Json& values = parent.at(key);
    if (!values.is_array() || values.size() != N) throw std::runtime_error(key + ": requires 7 numbers");
    Vec out{};
    for (int i = 0; i < N; ++i) {
        if (!values[i].is_number()) throw std::runtime_error(key + ": null/non-number");
        out[i] = values[i].get<double>();
        if (!std::isfinite(out[i])) throw std::runtime_error(key + ": nonfinite");
    }
    return out;
}
double bounded(const Json& cfg, const char* key, double low, double high) {
    double x = cfg.at(key).get<double>();
    if (!std::isfinite(x) || x < low || x > high) throw std::runtime_error(std::string("invalid ") + key);
    return x;
}

struct Settings {
    double speed = 0.04, accel = 0.08, max_temp = 55;
    double settle_timeout = 20, dwell = 3, startup_tolerance = 0.005;
    double position_error = 0.04, still_speed = 0.04, position_spread = 0.003;
    double stable_seconds = 1;
    int min_samples = 40;
    bool actual_static_measurement = false;
    double capture_error = .08;
    double approach_error = .08;
    double return_error = .04;
};
Settings load_settings(const Json& plan) {
    if (plan.at("schema") != "openarm-static-v1" || !plan.at("model_path_checked").get<bool>())
        throw std::runtime_error("Use torque_model.py prepare to create the plan first");
    const auto& cfg = plan.at("config");
    const auto scope=cfg.value("experiment_scope",std::string{});
    const bool multi=scope=="screened_multi_pose_15deg";
    const bool legacy=scope=="legacy_static_50";
    const bool proximal=scope=="proximal_grid_50";
    const bool pilot=scope=="local_pilot_1deg" || multi;
    if (pilot || legacy || proximal) {
        if (!cfg.at("holding_gains_verified").get<bool>() || !cfg.at("fall_protection_confirmed").get<bool>())
            throw std::runtime_error("pilot requires verified holding gains and fall protection");
    } else {
        for (const char* flag : {"motion_limits_reviewed", "geometry_and_mass_reviewed", "fall_protection_confirmed"})
            if (!cfg.at(flag).get<bool>()) throw std::runtime_error(std::string("not confirmed: ") + flag);
    }
    QMIN = vector7(cfg, "q_min_rad"); QMAX = vector7(cfg, "q_max_rad");
    KP = vector7(cfg, "kp"); KD = vector7(cfg, "kd");
    TORQUE_ALARM = vector7(cfg, "torque_alarm_nm"); SPEED_ALARM = vector7(cfg, "speed_alarm_rad_s");
    TRACK_ALARM = bounded(cfg, "tracking_alarm_rad", 0.01, 0.16);
    for (int i = 0; i < N; ++i) {
        if (!(QMIN[i] < QMAX[i] && QMIN[i] > -PMAX[i] && QMAX[i] < PMAX[i] &&
              KP[i] > 0 && KP[i] <= 500 && KD[i] >= 0 && KD[i] <= 5 &&
              TORQUE_ALARM[i] > 0 && TORQUE_ALARM[i] < 0.9*TMAX[i] &&
              SPEED_ALARM[i] > 0 && SPEED_ALARM[i] <= 0.5))
            throw std::runtime_error("invalid safety settings for J" + std::to_string(i+1));
    }
    Settings s;
    s.speed = bounded(cfg, "transit_speed_rad_s", 0.001, 0.05);
    s.accel = bounded(cfg, "transit_accel_rad_s2", 0.001, 0.1);
    s.max_temp = bounded(cfg, "temperature_alarm_c", 25, 70);
    s.settle_timeout = bounded(cfg, "settle_timeout_s", 2, 60);
    s.dwell = bounded(cfg, "sample_seconds", 1, 15);
    s.startup_tolerance = bounded(cfg, "startup_tolerance_rad", 0.0005, 0.01);
    s.position_error = bounded(cfg, "settle_position_error_rad", 0.001, TRACK_ALARM);
    s.return_error=s.position_error;
    s.still_speed = bounded(cfg, "settle_speed_rad_s", 0.001, 0.04);
    s.position_spread = bounded(cfg, "settle_spread_rad", 0.0005, 0.01);
    s.stable_seconds = bounded(cfg, "stable_seconds", 0.5, 3);
    s.min_samples = cfg.at("minimum_valid_samples").get<int>();
    if (s.min_samples < 10 || s.min_samples > s.dwell/PERIOD_S*0.9)
        throw std::runtime_error("minimum_valid_samples incompatible with dwell");
    for (double x : SPEED_ALARM) if (x <= s.speed) throw std::runtime_error("speed alarm must exceed transit speed");
    if (pilot) {
        const Vec origin=vector7(cfg,"pilot_baseline_q_motor_rad");
        const Vec expected=vector7(plan,"baseline_q_motor_rad");
        const Vec verified_kp{35,35,20,20,6,6,6}, verified_kd{4,4,2,2,0.8,0.8,0.8};
        const Vec torque_caps{2,2,1.5,1.5,0.7,0.7,0.7};
        for (int i=0;i<N;++i) {
            if (std::abs(origin[i]-expected[i])>1e-9 || KP[i]!=verified_kp[i] || KD[i]!=verified_kd[i] ||
                TORQUE_ALARM[i]>torque_caps[i] || SPEED_ALARM[i]>0.15 ||
                QMIN[i]<origin[i]-(multi ? 0.350001 : 0.120001) ||
                QMAX[i]>origin[i]+(multi ? 0.350001 : 0.120001))
                throw std::runtime_error("pilot config exceeds verified gain/local envelope/alarm scope");
        }
        if (TRACK_ALARM>0.03 || s.speed>0.005 || s.accel>0.01)
            throw std::runtime_error("pilot control thresholds exceed small-motion scope");
        for (const auto& pose : plan.at("poses")) {
            const Vec q=vector7(pose,"q_motor_rad");
            double total=0;
            for(int i=0;i<N;++i) total+=std::abs(q[i]-origin[i]);
            int changed=0;
            for(int i=0;i<N;++i) if(std::abs(q[i]-origin[i])>1e-6) ++changed;
            if (total>(multi ? 0.261801 : 0.017454) || changed>1)
                throw std::runtime_error("target exceeds screened single-joint angular scope");
        }
        std::cout << (multi ? "SCREENED MULTI-POSE: up to 15deg; low alarms/gains unchanged; actual movement checked.\n" :
                             "LOCAL PILOT: nominal one-degree excursions; unknown mass/zero; exploratory data only.\n");
    }
    if (legacy) {
        const auto policy=cfg.value("measurement_position_policy",std::string("target_tracking"));
        if(policy!="target_tracking" && policy!="actual_static")
            throw std::runtime_error("unknown measurement position policy");
        s.actual_static_measurement=policy=="actual_static";
        if(s.actual_static_measurement) s.capture_error=bounded(cfg,"static_capture_error_rad",.04,std::min(.08,TRACK_ALARM));
        const Vec old_kp{35,35,20,20,6,6,6}, old_kd{4,4,2,2,.8,.8,.8};
        const Vec old_low{-1.396263,-.174533,-1.570796,0,-1.570796,-.785398,-1.570796};
        const Vec old_high{3.490659,3.316125,1.570796,2.443461,1.570796,.785398,1.570796};
        const Vec caps{2,2,1.5,1.5,.7,.7,.7};
        const Vec origin=vector7(plan,"baseline_q_motor_rad");
        const Vec recorded_origin=vector7(cfg,"pilot_baseline_q_motor_rad");
        const double span=bounded(cfg,"max_excursion_rad",10*3.141592653589793/180,30*3.141592653589793/180);
        bounded(cfg,"minimum_distinct_pose_rad",3.141592653589793/180,5*3.141592653589793/180);
        if (cfg.at("requested_pose_count").get<int>()!=50 || plan.at("poses").size()!=50 ||
            s.position_error>.04 || s.startup_tolerance>.01 || s.accel>.08 ||
            bounded(cfg,"path_margin_rad",.04,.2)<.04)
            throw std::runtime_error("invalid legacy 50-pose settings");
        for(int i=0;i<N;++i) {
            if(KP[i]!=old_kp[i] || KD[i]!=old_kd[i] || TORQUE_ALARM[i]>caps[i] ||
               QMIN[i]<old_low[i]-1e-8 || QMAX[i]>old_high[i]+1e-8 ||
               std::abs(origin[i]-recorded_origin[i])>1e-9)
                throw std::runtime_error("50-pose settings exceed supplied legacy profile");
        }
        std::vector<Vec> checked_poses;
        for(const auto& pose:plan.at("poses")) {
            const Vec q=vector7(pose,"q_motor_rad");
            for(int i=0;i<N;++i) if(std::abs(q[i]-origin[i])>span+1e-9)
                throw std::runtime_error("50-pose target exceeds configured local excursion");
            for(const auto& old:checked_poses) {
                double distance=0;
                for(int i=0;i<N;++i) distance=std::max(distance,std::abs(q[i]-old[i]));
                if(distance<5*3.141592653589793/180-1e-9)
                    throw std::runtime_error("50-pose targets are not sufficiently distinct");
            }
            checked_poses.push_back(q);
        }
        std::cout << "LEGACY STATIC 50: supplied KP/KD; up to configured 10-30deg per joint.\n"
                     "Tracking alarm 0.16rad maximum; lower torque alarms retained.\n"
                     "Nominal model paths only: physical zero/mass/clearance remain unverified.\n";
        if(s.actual_static_measurement)
            std::cout << "ACTUAL STATIC measurement: compare at measured angles; capture deviation <= "
                      << s.capture_error << "rad. Initial/return holds retain strict target tolerance.\n";
    }
    if(proximal) {
        constexpr double rad=3.141592653589793/180;
        const Vec old_kp{35,35,20,20,6,6,6},old_kd{4,4,2,2,.8,.8,.8};
        const Vec low{-1.396263,-.174533,-1.570796,0,-1.570796,-.785398,-1.570796};
        const Vec high{3.490659,3.316125,1.570796,2.443461,1.570796,.785398,1.570796};
        const auto torque_profile=cfg.value("torque_profile",std::string("legacy_low"));
        if(torque_profile!="legacy_low" && torque_profile!="proximal_5_3")
            throw std::runtime_error("unknown proximal torque profile");
        const Vec caps=torque_profile=="proximal_5_3" ? Vec{5,5,3,3,.7,.7,.7} : Vec{2,2,1.5,1.5,.7,.7,.7};
        const Vec maxima{60*rad,75*rad,30*rad,90*rad,0,0,0};
        const Vec origin=vector7(plan,"baseline_q_motor_rad"),configured_origin=vector7(cfg,"pilot_baseline_q_motor_rad");
        const Vec extent=vector7(cfg,"max_excursion_per_joint_rad"),offset=vector7(cfg,"approach_offset_rad");
        const int repeats=cfg.at("repetitions").get<int>();
        const bool validation=cfg.value("validation_only",false);
        const int base_count=validation ? 2 : 50;
        if(repeats<1 || repeats>3 || cfg.at("requested_pose_count").get<int>()!=base_count ||
           cfg.at("requested_acquisition_count").get<int>()!=2*base_count*repeats || plan.at("poses").size()!=size_t(2*base_count*repeats) ||
           cfg.value("measurement_position_policy",std::string{})!="actual_static" ||
           s.position_error>.04 || s.startup_tolerance>.01 || s.accel>.08)
            throw std::runtime_error("invalid proximal paired acquisition settings");
        s.actual_static_measurement=true;
        s.capture_error=bounded(cfg,"static_capture_error_rad",.04,std::min(.08,TRACK_ALARM));
        s.approach_error=cfg.contains("approach_position_error_rad") ?
            bounded(cfg,"approach_position_error_rad",.08,std::min(.10,TRACK_ALARM-.04)) : s.capture_error;
        s.return_error=cfg.contains("return_position_error_rad") ?
            bounded(cfg,"return_position_error_rad",.04,std::min(.08,TRACK_ALARM/2)) : s.position_error;
        if(cfg.contains("approach_position_error_rad")) {
            const double minimum=cfg.at("minimum_actual_approach_rad").get<double>();
            for(double x:offset) if(x<s.approach_error+s.capture_error+minimum)
                throw std::runtime_error("approach offset too small for staging and sample deviations");
        }
        bounded(cfg,"minimum_proximal_actual_span_rad",20*rad,60*rad);
        bounded(cfg,"minimum_actual_approach_rad",.5*rad,3*rad);
        bounded(cfg,"minimum_distinct_pose_rad",rad,5*rad);
        bounded(cfg,"path_margin_rad",.04,.2);
        for(int i=0;i<N;++i) {
            if(KP[i]!=old_kp[i] || KD[i]!=old_kd[i] || TORQUE_ALARM[i]>caps[i] ||
               QMIN[i]<low[i]-1e-8 || QMAX[i]>high[i]+1e-8 ||
               extent[i]<0 || extent[i]>maxima[i]+1e-9 || offset[i]<10*rad || offset[i]>15*rad ||
               std::abs(origin[i]-configured_origin[i])>1e-9)
                throw std::runtime_error("proximal profile exceeds declared gain/range/alarm bounds");
        }
        Json bases=Json::object(),names=Json::object(),directions=Json::object();
        const auto direction_policy=cfg.value("approach_direction_policy",std::string{"same_sign"});
        if(direction_policy!="same_sign" && direction_policy!="per_base_opposed_vectors")
            throw std::runtime_error("unknown approach direction policy");
        for(const auto& pose:plan.at("poses")) {
            const Vec q=vector7(pose,"q_motor_rad"),pre=vector7(pose,"pre_q_motor_rad");
            const Vec axis_sign=pose.contains("approach_axis_sign") ? vector7(pose,"approach_axis_sign") : Vec{1,1,1,1,1,1,1};
            if(direction_policy=="per_base_opposed_vectors" && !pose.contains("approach_axis_sign"))
                throw std::runtime_error("missing per-axis approach directions");
            const std::string base=pose.at("base_pose_id").get<std::string>(),name=pose.at("pose_id").get<std::string>();
            const int sign=pose.at("approach_command_sign").get<int>(),rep=pose.at("repetition").get<int>();
            if(base.empty() || names.contains(name) || (sign!=1 && sign!=-1) || rep<1 || rep>repeats)
                throw std::runtime_error("invalid/duplicate paired acquisition identity");
            names[name]=true;
            for(int i=0;i<N;++i) {
                if((axis_sign[i]!=1 && axis_sign[i]!=-1) || (direction_policy=="same_sign" && axis_sign[i]!=1) ||
                   std::abs(q[i]-origin[i])>extent[i]+1e-9 ||
                   std::abs(pre[i]-(q[i]-sign*axis_sign[i]*offset[i]))>1e-9 ||
                   q[i]<QMIN[i] || q[i]>QMAX[i] || pre[i]<QMIN[i] || pre[i]>QMAX[i])
                    throw std::runtime_error("proximal target/staging point exceeds declared path scope");
            }
            if(bases.contains(base)) {
                const Vec prior=vector7(bases[base],"q_motor_rad");
                for(int i=0;i<N;++i) if(std::abs(q[i]-prior[i])>1e-9)
                    throw std::runtime_error("paired targets differ for one base pose");
                if(axis_sign!=vector7(bases[base],"approach_axis_sign"))
                    throw std::runtime_error("paired per-axis approach directions differ");
            } else bases[base]={{"q_motor_rad",q},{"approach_axis_sign",axis_sign}};
            const std::string key=base+"_r"+std::to_string(rep);
            const int flag=sign==1 ? 1 : 2;
            const int mask=directions.value(key,0);
            if(mask&flag) throw std::runtime_error("duplicate acquisition direction");
            directions[key]=mask|flag;
        }
        if(bases.size()!=size_t(base_count) || directions.size()!=size_t(base_count*repeats))
            throw std::runtime_error("proximal plan has an incomplete paired-base count");
        for(const auto& value:directions) if(value.get<int>()!=3)
            throw std::runtime_error("missing direction of a proximal pair");
        std::vector<double> shoulder_levels[2];std::vector<Vec> elbow_levels,base_vectors;
        for(const auto& base:bases) {
            const Vec q=vector7(base,"q_motor_rad");
            for(const auto& old:base_vectors) {
                double gap=0;for(int i=0;i<N;++i) gap=std::max(gap,std::abs(q[i]-old[i]));
                if(gap<5*rad-1e-9) throw std::runtime_error("proximal base targets insufficiently distinct");
            }
            base_vectors.push_back(q);
            for(int i=0;i<2;++i) {
                bool found=false;for(double level:shoulder_levels[i]) found=found || std::abs(level-q[i])<1e-9;
                if(!found) shoulder_levels[i].push_back(q[i]);
            }
            bool found=false;
            for(const auto& old:elbow_levels) {
                double gap=0;for(int i=2;i<N;++i) gap=std::max(gap,std::abs(q[i]-old[i]));
                found=found || gap<1e-9;
            }
            if(!found) elbow_levels.push_back(q);
        }
        const auto layout=cfg.value("proximal_layout",std::string{"grid_5x5x2"});
        if(layout!="grid_5x5x2" && layout!="adaptive_coverage")
            throw std::runtime_error("unknown proximal layout");
        if(validation && layout!="adaptive_coverage")
            throw std::runtime_error("short validation requires adaptive coverage layout");
        if(!validation && layout=="grid_5x5x2" &&
           (shoulder_levels[0].size()!=5 || shoulder_levels[1].size()!=5 || elbow_levels.size()!=2))
            throw std::runtime_error("proximal plan must contain a complete 5x5x2 grid");
        for(const auto& pose:plan.at("poses")) {
            if(pose.contains("via_start") && (!pose.at("via_start").is_boolean() || layout!="adaptive_coverage"))
                throw std::runtime_error("invalid planned baseline transit");
        }
        const double required=cfg.at("minimum_proximal_actual_span_rad").get<double>()+2*s.capture_error;
        for(int i=0;!validation && i<2;++i) {
            const auto bounds=std::minmax_element(shoulder_levels[i].begin(),shoulder_levels[i].end());
            if(*bounds.second-*bounds.first<required-1e-9)
                throw std::runtime_error("nominal shoulder coverage cannot guarantee requested capture-error margin");
        }
        std::cout << (validation ? "SHORT HOLD VALIDATION: 2 bases; NOT a full proximal coverage test.\n" :
                                  "PROXIMAL COVERAGE: 50 bases, paired approaches; J1/J2 actual coverage enforced.\n")
                  << "Gains and motion speed unchanged; review planned ranges and selected alarms.\n"
                  << "TORQUE PROFILE: " << torque_profile << "; software alarms Nm " << Json(TORQUE_ALARM).dump() << '\n';
    }
    return s;
}
Json last_safety_fault;
Json accepted_sample_windows=Json::object();
Json last_quality_event;
Json sample_window_rejections=Json::array();
std::string safety_phase="startup", safety_pose="startup";
Json fault_state(const State& s, const Vec* target) {
    Json result{{"time_s",s.time_s},{"q_rad",s.q},{"dq_rad_s",s.dq},{"tau_feedback_nm",s.tau},
                {"mos_c",s.mos},{"coil_c",s.coil},{"status",s.status},
                {"transaction_ms",s.transaction_ms},{"rx_skew_ms",s.rx_skew_ms},
                {"phase",safety_phase},{"pose_id",safety_pose},
                {"kp_sent",KP},{"kd_sent",KD},{"feedforward_torque_nm",Vec{}},
                {"desired_velocity_rad_s",Vec{}}};
    result["command_q_rad"]=target ? Json(*target) : Json(nullptr);
    return result;
}
void guard(const State& s, const Settings& settings, const Vec* target = nullptr) {
    for (int i = 0; i < N; ++i) {
        Json violations=Json::array();
        if (s.q[i]<QMIN[i] || s.q[i]>QMAX[i]) violations.push_back("position_range");
        if (std::abs(s.dq[i])>SPEED_ALARM[i]) violations.push_back("velocity");
        if (std::abs(s.tau[i])>TORQUE_ALARM[i]) violations.push_back("torque_feedback");
        if (s.mos[i]>=settings.max_temp) violations.push_back("mos_temperature");
        if (s.coil[i]>=settings.max_temp) violations.push_back("coil_temperature");
        if (target && std::abs(s.q[i]-(*target)[i])>TRACK_ALARM) violations.push_back("tracking_error");
        if (!violations.empty()) {
            // Capture the offending frame in memory. Top-level catch persists it
            // only AFTER Arm unwinding has attempted disable; no disk write here.
            last_safety_fault=fault_state(s,target);
            last_safety_fault["kind"]="feedback_threshold";
            last_safety_fault["joint"]=i+1;
            last_safety_fault["violations"]=violations;
            last_safety_fault["limits"]={{"q_min_rad",QMIN[i]},{"q_max_rad",QMAX[i]},
                {"speed_rad_s",SPEED_ALARM[i]},{"torque_feedback_nm",TORQUE_ALARM[i]},
                {"temperature_c",settings.max_temp},{"tracking_error_rad",TRACK_ALARM}};
            std::ostringstream message;
            message << std::fixed << std::setprecision(6)
                    << "safety threshold exceeded J" << i+1 << " " << violations.dump()
                    << "; q=" << s.q[i] << " rad; allowed=[" << QMIN[i] << ',' << QMAX[i] << ']'
                    << "; dq=" << s.dq[i] << " rad/s; limit=" << SPEED_ALARM[i]
                    << "; tau_fb=" << s.tau[i] << " Nm; limit=" << TORQUE_ALARM[i]
                    << "; MOS=" << s.mos[i] << "; coil=" << s.coil[i] << "; temp_limit=" << settings.max_temp;
            if (target) {
                const double error=s.q[i]-(*target)[i];
                message << "; command=" << (*target)[i] << " rad; tracking_error=" << error
                        << " rad (" << error*180.0/3.141592653589793 << " deg); limit=" << TRACK_ALARM << " rad";
            }
            throw std::runtime_error(message.str());
        }
    }
}
void predicted_guard(const State& previous, const Vec& target) {
    for (int i = 0; i < N; ++i) {
        // Advisory PD effort estimate BEFORE sending; this is not a firmware torque clamp.
        const double estimated = KP[i] * (target[i] - previous.q[i]) - KD[i]*previous.dq[i];
        if (std::abs(estimated) > TORQUE_ALARM[i]) {
            last_safety_fault=fault_state(previous,&target);
            last_safety_fault["kind"]="predicted_effort"; last_safety_fault["joint"]=i+1;
            last_safety_fault["estimated_effort_nm"]=estimated;
            last_safety_fault["limit_nm"]=TORQUE_ALARM[i];
            throw std::runtime_error("predicted PD effort exceeds alarm J"+std::to_string(i+1)+
                "; estimate="+std::to_string(estimated)+" Nm; limit="+std::to_string(TORQUE_ALARM[i]));
        }
    }
}
class Recorder {
public:
    explicit Recorder(const fs::path& path) : out_(path) {
        if (!out_) throw std::runtime_error("cannot open samples.csv");
        out_ << std::setprecision(12) << "time_s,stage,pose_id,stable,transaction_ms,rx_skew_ms,max_kernel_age_ms,cycle_gap_ms";
        for (const auto& prefix : {"q", "dq", "tau", "cmd", "rx", "status", "mos", "coil"})
            for (int i = 1; i <= N; ++i) out_ << ',' << prefix << i;
        out_ << ",sample_segment\n"; out_.flush();
    }
    void row(const State& s, const std::string& stage, const std::string& pose, bool valid,
             const Vec& target, double gap, int sample_segment=0) {
        out_ << s.time_s << ',' << stage << ',' << pose << ',' << valid << ',' << s.transaction_ms << ','
             << s.rx_skew_ms << ',' << s.max_kernel_age_ms << ',' << gap;
        for (const auto& values : {s.q, s.dq, s.tau, target, s.rx_time}) for (double x : values) out_ << ',' << x;
        for (int x : s.status) out_ << ',' << x;
        for (const auto& values : {s.mos, s.coil}) for (double x : values) out_ << ',' << x;
        out_ << ',' << sample_segment << '\n'; out_.flush();
        if (!out_) throw std::runtime_error("sample log write failed");
    }
private:
    std::ofstream out_;
};

double smooth(double u) { return u*u*u*(10 + u*(-15 + 6*u)); }
void move_static(Arm& arm, State& current, const Vec& target, const Settings& settings,
                 Recorder& log, Cycle& cycle, const std::string& pose) {
    validate_target(target);
    const Vec start = current.q;
    double distance = 0;
    for (int i=0; i<N; ++i) distance = std::max(distance, std::abs(target[i]-start[i]));
    // Quintic interpolation: maximum normalized speed 1.875, acceleration 5.774.
    const double duration = std::max({PERIOD_S, 1.875*distance/settings.speed,
                                    std::sqrt(5.774*distance/settings.accel)});
    const int count = int(std::ceil(duration/PERIOD_S));
    for (int n=1; n<=count; ++n) {
        safety_phase="transit"; safety_pose=pose;
        cycle.begin();
        Vec cmd{};
        for (int i=0; i<N; ++i) cmd[i] = start[i] + (target[i]-start[i])*smooth(double(n)/count);
        predicted_guard(current, cmd);
        current = arm.command(cmd); guard(current, settings, &cmd);
        log.row(current, "transit", pose, false, cmd, cycle.gap_ms);
        if (!current.sample_valid) throw std::runtime_error("transit feedback timing invalid");
        cycle.finish(arm);
    }
}
void hold_static(Arm& arm, State& current, const Vec& target, const Settings& settings,
                 Recorder& log, Cycle& cycle, const std::string& pose, bool collect,
                 Vec* sampled_mean = nullptr) {
    const auto start = Clock::now();
    std::deque<State> history;
    last_quality_event=Json::object();
    const bool actual_static=collect && settings.actual_static_measurement;
    const double position_capture_limit=actual_static ? settings.capture_error : settings.position_error;
    bool sampling = false;
    int valid_count = 0;
    int segment=0, restarts=0;
    Vec sum{};
    Clock::time_point sample_start{};
    const size_t window = size_t(std::ceil(settings.stable_seconds/PERIOD_S))+1;
    while (true) {
        safety_phase=sampling && collect ? "sample" : "settle"; safety_pose=pose;
        cycle.begin(); predicted_guard(current, target);
        current = arm.command(target); guard(current, settings, &target);
        Json reasons=Json::array();
        Vec feedback_gaps{};
        if(!current.sample_valid) reasons.push_back("feedback_sample_timing");
        if(cycle.gap_ms>1000*MAX_SAMPLE_GAP_S) reasons.push_back("control_sample_gap");
        bool good = current.sample_valid && cycle.gap_ms <= 1000*MAX_SAMPLE_GAP_S;
        for (int i=0; i<N; ++i) {
            if(std::abs(current.q[i]-target[i])>position_capture_limit) {
                reasons.push_back("J"+std::to_string(i+1)+(actual_static ? ":static_capture_error" : ":settle_position_error")); good=false;
            }
            if(std::abs(current.dq[i])>settings.still_speed) {
                reasons.push_back("J"+std::to_string(i+1)+":settle_velocity"); good=false;
            }
            if (!history.empty()) {
                const double dt = current.rx_time[i] - history.back().rx_time[i];
                feedback_gaps[i]=dt;
                if(!(dt>0 && dt<=MAX_SAMPLE_GAP_S)) {
                    reasons.push_back("J"+std::to_string(i+1)+":feedback_sample_gap"); good=false;
                }
            }
        }
        if (!good) history.clear(); else history.push_back(current);
        while (history.size() > window) history.pop_front();
        bool stable = history.size() == window;
        Vec spreads{};
        if (stable) {
            for (int i=0; i<N; ++i) {
                double low = current.q[i], high = low;
                for (const auto& h : history) { low=std::min(low,h.q[i]); high=std::max(high,h.q[i]); }
                spreads[i]=high-low;
                if (high-low > settings.position_spread) {
                    stable=false;
                    reasons.push_back("J"+std::to_string(i+1)+":settle_position_spread");
                }
            }
        }
        if(!reasons.empty()) {
            last_quality_event=fault_state(current,&target);
            last_quality_event["kind"]="sampling_quality";
            last_quality_event["reasons"]=reasons;
            last_quality_event["position_spread_rad"]=spreads;
            last_quality_event["history_size"]=history.size();
            last_quality_event["feedback_gap_s"]=feedback_gaps;
            last_quality_event["sample_valid"]=current.sample_valid;
            last_quality_event["cycle_gap_ms"]=cycle.gap_ms;
            last_quality_event["max_kernel_age_ms"]=current.max_kernel_age_ms;
            last_quality_event["limits"]={{"position_error_rad",position_capture_limit},
                {"velocity_rad_s",settings.still_speed},{"position_spread_rad",settings.position_spread},
                {"sample_gap_ms",1000*MAX_SAMPLE_GAP_S},{"timeout_s",settings.settle_timeout+settings.dwell}};
        }
        if (sampling && !stable) {
            ++restarts;
            if(sample_window_rejections.size()<20) {
                Json event=last_quality_event;
                event["rejected_segment"]=segment; event["restarts"]=restarts;
                sample_window_rejections.push_back(event);
            }
            if(restarts<=3 || restarts%10==0)
                std::cout << pose << ": discard sample segment " << segment << "; " << reasons.dump()
                          << "; holding SAME target, waiting for a new continuous stable window\n";
            // This is a sampling-quality reset, not a new motion attempt. All
            // per-cycle safety guards remain active at the unchanged target.
            sampling=false; valid_count=0; sum=Vec{}; history.clear();
        }
        if (stable && !sampling) {
            sampling=true; ++segment; valid_count=0; sum=Vec{}; sample_start=Clock::now();
        }
        const bool valid = sampling && stable && collect;
        log.row(current, sampling && collect ? "sample" : "settle", pose, valid, target, cycle.gap_ms,segment);
        if (valid) {
            ++valid_count;
            for(int i=0;i<N;++i) sum[i]+=current.q[i];
        }
        cycle.finish(arm);
        if (stable && !collect) return;
        if (sampling && elapsed(Clock::now(), sample_start) >= settings.dwell) {
            if (valid_count < settings.min_samples) throw std::runtime_error("too few valid pose samples");
            if(sampled_mean) for(int i=0;i<N;++i) (*sampled_mean)[i]=sum[i]/valid_count;
            accepted_sample_windows[pose]={{"segment",segment},{"valid_count",valid_count},{"restarts",restarts}};
            std::cout << pose << ": " << valid_count << " stable samples accepted from segment " << segment << '\n'; return;
        }
        if (elapsed(Clock::now(),start)>settings.settle_timeout+settings.dwell) {
            last_safety_fault=fault_state(current,&target);
            last_safety_fault["kind"]="sampling_quality_timeout";
            last_safety_fault["last_quality_event"]=last_quality_event;
            last_safety_fault["restarts"]=restarts;
            throw std::runtime_error("continuous stable sample window timed out at "+pose+
                                     "; last reasons="+last_quality_event.value("reasons",Json::array()).dump());
        }
    }
}
} // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler); std::signal(SIGTERM, signal_handler);
    fs::path plan_path, output_root;
    bool execute = false;
    for (int n=1; n<argc; ++n) {
        const std::string arg=argv[n];
        if (arg=="--version") {
            std::cout << "openarm-static-v1-multi11-return-holdfix\n"; return 0;
        } else if (arg=="--help") {
            std::cout << "openarm_static_torque [--diagnose] [--interface can0] [--output-root DIR]\n"
                         "openarm_static_torque --plan PLAN.json --execute [--interface can0]\n"; return 0;
        } else if (arg=="--diagnose") { /* Default mode. */ }
        else if (arg=="--execute") execute=true;
        else if ((arg=="--plan" || arg=="--interface" || arg=="--output-root") && n+1<argc) {
            const std::string value=argv[++n];
            if (arg=="--plan") plan_path=value;
            else if (arg=="--interface") CAN_INTERFACE=value;
            else output_root=value;
        } else { std::cerr << "Unknown/incomplete argument: " << arg << '\n'; return 2; }
    }
    if (execute && plan_path.empty()) { std::cerr << "--execute requires --plan\n"; return 2; }
    if (!execute && !plan_path.empty()) { std::cerr << "Use prepare for offline validation; --plan requires --execute\n"; return 2; }
    fs::path run_dir;
    Json metadata{{"completed",false},{"mode", execute ? "static_motion" : "diagnose"},
                  {"interface",CAN_INTERFACE},{"period_s",PERIOD_S},
                  {"torque_source","motor reported estimate; not a calibrated external torque sensor"},
                  {"software_version","openarm-static-v1-multi11-return-holdfix"},
                  {"sampling_schema","continuous-window-v1"}};
    metadata["acquisition_completed"]=false;
    metadata["return_completed"]=false;
    metadata["supported_exit_confirmed"]=false;
    try {
        if (output_root.empty()) {
            const char* home=std::getenv("HOME");
            if (!home) throw std::runtime_error("HOME unset; provide --output-root");
            output_root=fs::path(home)/"experiments/torque_compare";
        }
        const auto id=std::chrono::duration_cast<std::chrono::nanoseconds>(WallClock::now().time_since_epoch()).count();
        fs::create_directories(output_root);
        run_dir=output_root/((execute ? "static_" : "diagnose_")+std::to_string(id));
        if (!fs::create_directory(run_dir)) throw std::runtime_error("run directory already exists");
        Json plan;
        Settings settings;
        if (execute) {
            plan=read_json(plan_path); settings=load_settings(plan);
            const fs::path model=plan_path.parent_path()/plan.at("frozen_model_file").get<std::string>();
            fs::copy_file(model,run_dir/"model.mjb");
            write_json(run_dir/"plan.json",plan);
            metadata["plan"]=plan;
        }
        write_json(run_dir/"metadata.json",metadata);
        std::cout << "Logs: " << run_dir << "\nOnly can0 J1-J7 (no J8). Close all other control/query programs.\n";
        Arm arm((run_dir/"timing.csv").string());
        metadata["motor_parameters"]=arm.verify_parameters();
        State current=arm.query(false); require_disabled(current);
        if (!current.sample_valid) throw std::runtime_error("startup feedback timing invalid");
        Recorder log(run_dir/"samples.csv"); Cycle cycle;
        log.row(current,"startup","startup",false,current.q,0);
        const Vec baseline=current.q;
        metadata["start_q_motor_rad"]=baseline;
        write_json(run_dir/"snapshot.json",Json{{"schema","openarm-snapshot-v1"},{"q_motor_rad",baseline},
                   {"interface",CAN_INTERFACE},{"motor_parameters",metadata["motor_parameters"]},
                   {"run_time_ns",id}});
        write_json(run_dir/"metadata.json",metadata);
        if (!execute) {
            for (int n=0; n<200; ++n) {
                cycle.begin(); current=arm.query(false); require_disabled(current);
                log.row(current,"diagnose","diagnose",false,current.q,cycle.gap_ms);
                if (!current.sample_valid) throw std::runtime_error("diagnostic feedback timing invalid");
                cycle.finish(arm);
            }
            metadata["completed"]=true; write_json(run_dir/"metadata.json",metadata);
            std::cout << "Read-only diagnosis complete. No enable/disable/motion packets sent.\n"; return 0;
        }
        guard(current,settings);
        const Vec expected=vector7(plan,"baseline_q_motor_rad");
        const auto poses=plan.at("poses");
        const bool proximal=plan.at("config").value("experiment_scope",std::string{})=="proximal_grid_50";
        const bool validation=proximal && plan.at("config").value("validation_only",false);
        const int expected_bases=validation ? 2 : 50;
        metadata["validation_only"]=validation;
        metadata["full_proximal_coverage_complete"]=false;
        if (!poses.is_array() || poses.empty() || poses.size()>(proximal ? 300u : 100u))
            throw std::runtime_error("prepared acquisition count exceeds profile limit");
        for (const auto& pose : poses) {
            validate_target(vector7(pose,"q_motor_rad"));
            const auto name=pose.at("pose_id").get<std::string>();
            if (name.empty() || name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos)
                throw std::runtime_error("invalid pose name");
        }
        for (int i=0;i<N;++i) if (std::abs(current.q[i]-expected[i])>settings.startup_tolerance)
            throw std::runtime_error("startup differs from prepared model path; prepare a new plan");
        std::cout << "Physical fall protection must cover ALL poses and loss of power.\n"
                     "Measurement poses must have NO external support contact. Ctrl+C/fault will attempt disable.\n"
                     "Type RUN_STATIC to enable and move: " << std::flush;
        std::string answer; std::getline(std::cin,answer);
        if (answer!="RUN_STATIC") throw std::runtime_error("motion not confirmed");
        const State fresh=arm.query(false); require_disabled(fresh); guard(fresh,settings);
        if (!fresh.sample_valid) throw std::runtime_error("pre-enable timing invalid");
        for(int i=0;i<N;++i) if(std::abs(fresh.q[i]-baseline[i])>settings.startup_tolerance ||
                                  std::abs(fresh.q[i]-expected[i])>settings.startup_tolerance ||
                                  std::abs(fresh.dq[i])>settings.still_speed)
            throw std::runtime_error("pose changed/moving during confirmation; re-prepare");
        safety_phase="enable_hold"; safety_pose="initial_hold";
        current=arm.enable_at(fresh.q); guard(current,settings,&fresh.q);
        if (!current.sample_valid) throw std::runtime_error("post-enable timing invalid");
        hold_static(arm,current,fresh.q,settings,log,cycle,"initial_hold",false);
        const Vec held_origin=current.q;
        const bool verify_excursion=plan.at("config").value("experiment_scope",std::string{})=="screened_multi_pose_15deg";
        const bool verify_50=plan.at("config").value("experiment_scope",std::string{})=="legacy_static_50";
        std::vector<Vec> distinct_measured_poses;
        std::vector<std::string> measured_base_names;
        int proximal_acquisitions=0;
        for(const auto& pose : poses) {
            const auto name=pose.at("pose_id").get<std::string>();
            const Vec target=vector7(pose,"q_motor_rad");
            Vec pre_actual{};
            if(proximal) {
                Settings staging=settings;staging.position_error=settings.approach_error;
                if(pose.value("via_start",false)) {
                    move_static(arm,current,expected,settings,log,cycle,name+"_via_start");
                    hold_static(arm,current,expected,staging,log,cycle,name+"_via_start",false);
                }
                const Vec pre=vector7(pose,"pre_q_motor_rad");
                move_static(arm,current,pre,settings,log,cycle,name+"_approach");
                hold_static(arm,current,pre,staging,log,cycle,name+"_approach",false);
                pre_actual=current.q;
            }
            move_static(arm,current,target,settings,log,cycle,name);
            Vec measured_mean{};
            hold_static(arm,current,target,settings,log,cycle,name,true,&measured_mean);
            metadata["accepted_sample_windows"]=accepted_sample_windows;
            metadata["sample_window_rejections"]=sample_window_rejections;
            if(proximal) {
                const auto& cfg=plan.at("config");
                const std::string base=pose.at("base_pose_id").get<std::string>();
                const int sign=pose.at("approach_command_sign").get<int>();
                const Vec axis_sign=pose.contains("approach_axis_sign") ? vector7(pose,"approach_axis_sign") : Vec{1,1,1,1,1,1,1};
                const double minimum=cfg.at("minimum_actual_approach_rad").get<double>();
                Vec signed_motion{};bool passed=true;
                for(int i=0;i<N;++i) {
                    signed_motion[i]=sign*axis_sign[i]*(measured_mean[i]-pre_actual[i]);
                    passed=passed && signed_motion[i]>=minimum;
                }
                Json check{{"pose_id",name},{"base_pose_id",base},{"mean_q_rad",measured_mean},
                           {"pre_actual_q_rad",pre_actual},{"signed_approach_delta_rad",signed_motion},
                           {"approach_command_sign",sign},{"approach_axis_sign",axis_sign},
                           {"repetition",pose.at("repetition")}};
                const bool new_base=std::find(measured_base_names.begin(),measured_base_names.end(),base)==measured_base_names.end();
                if(new_base) {
                    const double distinct=cfg.at("minimum_distinct_pose_rad").get<double>();
                    for(const auto& other:distinct_measured_poses) {
                        double gap=0;for(int i=0;i<N;++i) gap=std::max(gap,std::abs(measured_mean[i]-other[i]));
                        passed=passed && gap>=distinct;
                    }
                }
                check["passed"]=passed;
                if(!metadata.contains("pose_checks")) metadata["pose_checks"]=Json::array();
                metadata["pose_checks"].push_back(check);
                if(!passed) throw std::runtime_error("actual paired approach direction/distinctness not verified at "+name);
                if(new_base) {measured_base_names.push_back(base);distinct_measured_poses.push_back(measured_mean);}
                ++proximal_acquisitions;
                metadata["valid_distinct_pose_count"]=distinct_measured_poses.size();
                metadata["valid_acquisition_count"]=proximal_acquisitions;
                write_json(run_dir/"metadata.json",metadata);
                std::cout << "Verified base " << distinct_measured_poses.size() << '/' << expected_bases << "; directional acquisition "
                          << proximal_acquisitions << '/' << poses.size() << '\n';
            }
            if(verify_50) {
                const double minimum=plan.at("config").at("minimum_distinct_pose_rad").get<double>();
                double nearest=1e100, actual_shift=0, requested_shift=0;
                for(int i=0;i<N;++i) {
                    actual_shift=std::max(actual_shift,std::abs(measured_mean[i]-held_origin[i]));
                    requested_shift=std::max(requested_shift,std::abs(target[i]-expected[i]));
                }
                for(const auto& other:distinct_measured_poses) {
                    double difference=0;
                    for(int i=0;i<N;++i) difference=std::max(difference,std::abs(measured_mean[i]-other[i]));
                    nearest=std::min(nearest,difference);
                }
                const bool passed=nearest>=minimum && actual_shift>=.5*requested_shift;
                Json check{{"pose_id",name},{"mean_q_rad",measured_mean},{"passed",passed},
                           {"actual_shift_rad",actual_shift},{"requested_shift_rad",requested_shift}};
                check["nearest_previous_pose_rad"]=distinct_measured_poses.empty() ? Json(nullptr) : Json(nearest);
                if(!metadata.contains("pose_checks")) metadata["pose_checks"]=Json::array();
                metadata["pose_checks"].push_back(check);
                if(!passed) throw std::runtime_error("50-pose actual movement/distinctness check failed at "+name);
                distinct_measured_poses.push_back(measured_mean);
                metadata["valid_distinct_pose_count"]=distinct_measured_poses.size();
                write_json(run_dir/"metadata.json",metadata);
                std::cout << "Verified distinct measured pose " << distinct_measured_poses.size() << "/50\n";
            }
            if (verify_excursion) {
                Json check{{"pose_id",name},{"passed",true}};
                for(int i=0;i<N;++i) {
                    const double requested=target[i]-expected[i];
                    if (std::abs(requested)>1e-6) {
                        const double measured=current.q[i]-held_origin[i];
                        const bool passed=requested*measured>0 && std::abs(measured)>=0.5*std::abs(requested);
                        check["joint"]=i+1; check["requested_delta_rad"]=requested;
                        check["actual_delta_rad"]=measured; check["passed"]=passed;
                        std::cout << name << ": measured excursion=" << measured*180.0/3.141592653589793 << " deg\n";
                    }
                }
                if (!metadata.contains("movement_checks")) metadata["movement_checks"]=Json::array();
                metadata["movement_checks"].push_back(check);
                if (!check.at("passed").get<bool>())
                    throw std::runtime_error("insufficient actual excursion; this pose must not count as moved");
            }
        }
        if(proximal) {
            Vec low=distinct_measured_poses.front(),high=low,span{};
            for(const auto& q:distinct_measured_poses) for(int i=0;i<N;++i) {
                low[i]=std::min(low[i],q[i]);high[i]=std::max(high[i],q[i]);
            }
            for(int i=0;i<N;++i) span[i]=high[i]-low[i];
            metadata["proximal_measured_span_rad"]=span;
            const double minimum=plan.at("config").at("minimum_proximal_actual_span_rad").get<double>();
            if(distinct_measured_poses.size()!=size_t(expected_bases) || proximal_acquisitions!=plan.at("config").at("requested_acquisition_count").get<int>() ||
               (!validation && (span[0]<minimum || span[1]<minimum)))
                throw std::runtime_error("actual J1/J2 coverage insufficient; cannot declare proximal acquisition complete");
            metadata["full_proximal_coverage_complete"]=!validation;
        }
        metadata["acquisition_completed"]=true;
        write_json(run_dir/"metadata.json",metadata);
        move_static(arm,current,fresh.q,settings,log,cycle,"return_start");
        Settings returning=settings;returning.position_error=settings.return_error;
        hold_static(arm,current,fresh.q,returning,log,cycle,"return_start",false);
        metadata["return_completed"]=true;
        Vec return_error{};for(int i=0;i<N;++i) return_error[i]=current.q[i]-fresh.q[i];
        metadata["return_actual_error_rad"]=return_error;
        metadata["return_position_limit_rad"]=settings.return_error;
        write_json(run_dir/"metadata.json",metadata);
        std::cout << "Acquisition finished; HOLDING near start pose (within configured return tolerance). Install support then type SUPPORTED + Enter.\n";
        std::string stop_line;
        while (true) {
            pollfd stdin_fd{STDIN_FILENO,POLLIN,0};
            if (::poll(&stdin_fd,1,0)>0) {
                if (stdin_fd.revents & POLLIN) {
                    // Nonblocking character reads: an unfinished input line must not stall control.
                    char ch; const ssize_t got=::read(STDIN_FILENO,&ch,1);
                    if (got<=0) throw std::runtime_error("console EOF during hold");
                    if (got==1 && ch=='\n') {
                        if (stop_line=="SUPPORTED") break;
                        stop_line.clear();
                    } else if(got==1 && ch!='\r') {
                        stop_line+=ch;
                        if (stop_line.size()>128) stop_line.clear();
                    }
                } else if(stdin_fd.revents&(POLLHUP|POLLERR)) throw std::runtime_error("console closed during hold");
            }
            safety_phase="end_hold"; safety_pose="end_hold";
            cycle.begin(); predicted_guard(current,fresh.q);
            current=arm.command(fresh.q); guard(current,settings,&fresh.q);
            log.row(current,"end_hold","end_hold",false,fresh.q,cycle.gap_ms);
            if(!current.sample_valid) throw std::runtime_error("end hold feedback timing invalid");
            cycle.finish(arm);
        }
        metadata["supported_exit_confirmed"]=true;
        metadata["completed"]=true; metadata["stop_reason"]="supported_normal_exit";
        metadata["accepted_sample_windows"]=accepted_sample_windows;
        metadata["sample_window_rejections"]=sample_window_rejections;
        write_json(run_dir/"metadata.json",metadata);
        return 0; // Destructor now attempts disable. Physical support is required.
    } catch(const std::exception& e) {
        std::cerr << "STOP: " << e.what() << "\nIf motors were enabled, bounded best-effort disable was attempted.\n"
                     "No acknowledgement/safe stop guarantee: check physical protection and motor state.\n";
        metadata["error"]=e.what();
        metadata["accepted_sample_windows"]=accepted_sample_windows;
        metadata["sample_window_rejections"]=sample_window_rejections;
        if(!last_quality_event.is_null()) metadata["last_sampling_quality_event"]=last_quality_event;
        if (!last_safety_fault.is_null()) metadata["safety_fault"]=last_safety_fault;
        if(!run_dir.empty()) { try { write_json(run_dir/"metadata.json",metadata); } catch(...) {} }
        return 1;
    }
}
