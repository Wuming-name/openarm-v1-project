// OpenArm V1 J1-J7 RL reach bridge. Read-only unless --execute with IPC handshake.
// Motor feedback torque is not an independently calibrated shaft torque measurement.
#include <sys/ioctl.h>
#include <sys/file.h>
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
constexpr double PERIOD_S = 0.020;
constexpr double MAX_SAMPLE_GAP_S = 0.075;
constexpr double TRANSACTION_LIMIT_S = 0.020;
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
    Vec rx_time{}, tx_time{};
    std::array<std::array<unsigned,8>,N> raw{};
    bool command_sent=false;
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

            diagnostics_.reserve(50000);


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
        for(int b=0;b<8;++b) state.raw[joint][b]=d[b];

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
            state.command_sent=target!=nullptr;
            std::array<bool, N> updated{};
            Vec sent_wall{};

            const auto send_begin = Clock::now();
            for (int i = 0; i < N; ++i) {
                check_stop();

                if (elapsed(Clock::now(), transaction_begin) >
                    TRANSACTION_LIMIT_S)
                    throw std::runtime_error("发送阶段发生严重调度延迟");

                sent_wall[i] = wall_seconds();
                state.tx_time[i]=sent_wall[i]-wall_start_;
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
                    if (diagnostics_.size()<50000) diagnostics_.push_back(diag);
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
            if (diagnostics_.size()<50000) diagnostics_.push_back(diag);

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

// New reach bridge: lock, bounded IPC deadline, pre-send timing checks, memory logging.
constexpr const char* VERSION="openarm-reach-supervised-trial-v2-diagnostics-r2";
struct Entry {State s; Vec ref; double gap;};
double vmax=.02,amax=.04,temp=55,radius=.15;
Vec home{};
double effective_reach(double x,double lo,double hi,int bits) {
    const auto n=(1u<<bits)-1;
    return lo+double(uint64_t((x-lo)/(hi-lo)*n))*(hi-lo)/n;
}
void configure(const Json& c,bool motion) {
    if(c.at("schema")!="openarm-reach-sim-v2")throw std::runtime_error("Unknown config");
    QMIN=vector7(c,"q_min");QMAX=vector7(c,"q_max");KP=vector7(c,"kp");KD=vector7(c,"kd");
    TORQUE_ALARM=vector7(c,"torque_alarm");SPEED_ALARM=vector7(c,"speed_alarm");
    TRACK_ALARM=bounded(c,"tracking_alarm",.02,.16);
    vmax=bounded(c,"reference_speed",.005,.05);amax=bounded(c,"reference_accel",.005,.08);
    temp=bounded(c,"temperature_alarm",25,55);radius=bounded(c,"actual_radius",.02,.45);
    const Vec low{-1.396263,-.174532,-1.570796,0,-1.570796,-.785398,-1.570796};
    const Vec high{3.490659,3.316125,1.570796,2.443461,1.570796,.785398,1.570796};
    const Vec kp{35,35,20,20,6,6,6},kd{4,4,2,2,.8,.8,.8},caps{5,5,3,3,.7,.7,.7};
    for(int j=0;j<N;++j)if(QMIN[j]<low[j]-1e-8||QMAX[j]>high[j]+1e-8||QMIN[j]>=QMAX[j]||
        KP[j]!=kp[j]||KD[j]!=kd[j]||TORQUE_ALARM[j]<=0||TORQUE_ALARM[j]>caps[j]||
        SPEED_ALARM[j]<=vmax||SPEED_ALARM[j]>.5)throw std::runtime_error("Config outside inherited limits");
    if(motion){home=vector7(c,"home");validate_target(home);
        if(!c.value("supervised_trial",false))throw std::runtime_error("Explicit trial coordinator required");}
}
void checked(const State& s,const Vec* ref=nullptr) {
    if(!s.sample_valid)throw std::runtime_error("CAN feedback quality invalid");
    for(int j=0;j<N;++j){
        if(!std::isfinite(s.q[j])||!std::isfinite(s.dq[j])||!std::isfinite(s.tau[j])||
           s.q[j]<QMIN[j]||s.q[j]>QMAX[j]||std::abs(s.dq[j])>SPEED_ALARM[j]||
           std::abs(s.tau[j])>TORQUE_ALARM[j]||s.mos[j]>=temp||s.coil[j]>=temp) {
            std::ostringstream message;
            message << std::setprecision(12) << "Feedback threshold J" << j+1 << ": ";
            if(!std::isfinite(s.q[j])) message << "q_nonfinite; ";
            if(!std::isfinite(s.dq[j])) message << "dq_nonfinite; ";
            if(!std::isfinite(s.tau[j])) message << "tau_nonfinite; ";
            if(s.q[j]<QMIN[j]) message << "q_below_min; ";
            if(s.q[j]>QMAX[j]) message << "q_above_max; ";
            if(std::abs(s.dq[j])>SPEED_ALARM[j]) message << "speed_above_limit; ";
            if(std::abs(s.tau[j])>TORQUE_ALARM[j]) message << "torque_above_limit; ";
            if(s.mos[j]>=temp) message << "mos_temperature_at_or_above_limit; ";
            if(s.coil[j]>=temp) message << "coil_temperature_at_or_above_limit; ";
            message << "q=" << s.q[j] << " rad, q_range=[" << QMIN[j] << ',' << QMAX[j]
                    << "]; dq=" << s.dq[j] << " rad/s, abs_limit=" << SPEED_ALARM[j]
                    << "; tau_feedback=" << s.tau[j] << ", abs_limit=" << TORQUE_ALARM[j]
                    << "; mos=" << s.mos[j] << ", coil=" << s.coil[j]
                    << ", temperature_limit=" << temp << "; status=" << s.status[j];
            throw std::runtime_error(message.str());
        }
        if(ref&&(std::abs((*ref)[j]-s.q[j])>TRACK_ALARM||std::abs(s.q[j]-home[j])>radius||s.status[j]!=1))
            throw std::runtime_error("Tracking/envelope/status J"+std::to_string(j+1));
    }
}
void command_check(const State& s,const Vec& next,const Vec& prev,Vec& velocity,const Json& c){
    validate_target(next);
    const double r=bounded(c,"reference_radius",.02,.40);
    for(int j=0;j<N;++j){
        double v=(next[j]-prev[j])/PERIOD_S;
        if(std::abs(v)>vmax+1e-7||std::abs(v-velocity[j])>amax*PERIOD_S+1e-7||
           next[j]<QMIN[j]+.04||next[j]>QMAX[j]-.04||std::abs(next[j]-home[j])>r+1e-8)
            throw std::runtime_error("Reference speed/acceleration/envelope J"+std::to_string(j+1));
        double estimate=effective_reach(KP[j],0,500,12)*(effective_reach(next[j],-12.5,12.5,16)-s.q[j])+
            effective_reach(KD[j],0,5,12)*(effective_reach(0,-VMAX[j],VMAX[j],12)-s.dq[j])+
            effective_reach(0,-TMAX[j],TMAX[j],12);
        if(std::abs(estimate)>TORQUE_ALARM[j])throw std::runtime_error("Predicted effort J"+std::to_string(j+1));
        velocity[j]=v;
    }
}
void pre_send_guard(double gap,double age){
    if(!std::isfinite(gap)||!std::isfinite(age)||gap<0||age<0||gap>.040||age>.040)
        throw std::runtime_error("Pre-send timing/feedback age exceeds 40 ms");
}
void publish(int fd,const State& s,int seq,const Vec& ref){
    const auto msg=Json{{"seq",seq},{"q",s.q},{"dq",s.dq},{"tau",s.tau},{"ref",ref},
        {"time_s",s.time_s},{"status",s.status},{"transaction_ms",s.transaction_ms}}.dump();
    if(::send(fd,msg.data(),msg.size(),MSG_DONTWAIT|MSG_NOSIGNAL)!=(ssize_t)msg.size())
        throw std::runtime_error("IPC output blocked/closed");
}
Json receive_request(int fd,Clock::time_point deadline){
    char buf[4096];
    while(Clock::now()<deadline){
        check_stop();pollfd p{fd,POLLIN,0};int r=::poll(&p,1,1);
        if(r<0&&errno!=EINTR)throw std::runtime_error("IPC poll failed");
        if(r>0){ssize_t n=::recv(fd,buf,sizeof(buf),MSG_DONTWAIT|MSG_TRUNC);
            if(n<=0||n>=(ssize_t)sizeof(buf))throw std::runtime_error("IPC closed/oversized");
            return Json::parse(buf,buf+n);}
    }
    throw std::runtime_error("IPC deadline exceeded; no next motion command sent");
}
void save_entries(const fs::path& dir,const std::vector<Entry>& data){
    std::ofstream f(dir/"samples.csv");f<<"time_s,cycle_gap_ms,transaction_ms,rx_skew_ms,kernel_age_ms,valid";
    for(const char* name:{"q","dq","tau_feedback","reference","status","mos","coil","rx_time","tx_time"})
        for(int j=1;j<=7;++j)f<<','<<name<<j;
    f<<'\n'<<std::setprecision(12);
    for(const auto& x:data){const auto&s=x.s;
        f<<s.time_s<<','<<x.gap<<','<<s.transaction_ms<<','<<s.rx_skew_ms<<','<<s.max_kernel_age_ms<<','<<s.sample_valid;
        for(double v:s.q) { f<<','<<v; }
        for(double v:s.dq) { f<<','<<v; }
        for(double v:s.tau) { f<<','<<v; }
        for(double v:x.ref) { f<<','<<v; }
        for(int v:s.status) { f<<','<<v; }
        for(double v:s.mos) { f<<','<<v; }
        for(double v:s.coil) { f<<','<<v; }
        for(double v:s.rx_time) { f<<','<<v; }
        for(double v:s.tx_time) { f<<','<<v; }
        f<<'\n';
    }
    f.flush();if(!f)throw std::runtime_error("Sample file save failed");
}
} // namespace
int main(int argc,char**argv){
    std::signal(SIGINT,signal_handler);std::signal(SIGTERM,signal_handler);std::signal(SIGPIPE,SIG_IGN);
    fs::path config,output;int ipc=-1,lockfd=-1;bool execute=false,shadow=false,created=false;std::vector<Entry> entries;
    Json meta{{"version",VERSION},{"completed",false},{"disable_confirmed",false}};
    try{
        for(int a=1;a<argc;++a){std::string x=argv[a];
            if(x=="--version"){std::cout<<VERSION<<'\n';return 0;}
            else if(x=="--config"&&a+1<argc)config=argv[++a];
            else if(x=="--output"&&a+1<argc)output=argv[++a];
            else if(x=="--interface"&&a+1<argc)CAN_INTERFACE=argv[++a];
            else if(x=="--fd"&&a+1<argc)ipc=std::stoi(argv[++a]);
            else if(x=="--execute")execute=true;
            else if(x=="--shadow")shadow=true;
            else throw std::runtime_error("Usage: can_bridge --config file --output NEW_DIR [--interface can0] [--execute --fd N]");}
        if(execute&&shadow)throw std::runtime_error("Cannot combine execute and shadow");
        if(config.empty()||output.empty()||fs::exists(output))throw std::runtime_error("Config required; output must not exist");
        fs::create_directories(output);created=true;auto c=read_json(config);configure(c,execute);meta["config"]=c;
        meta["motion_requested"]=execute;
        meta["shadow_requested"]=shadow;
        const char* userhome=std::getenv("HOME");if(!userhome)throw std::runtime_error("HOME missing");
        fs::path lock=fs::path(userhome)/".cache/openarm_static_torque.lock";fs::create_directories(lock.parent_path());
        lockfd=::open(lock.c_str(),O_CREAT|O_RDWR|O_CLOEXEC,0600);
        if(lockfd<0||flock(lockfd,LOCK_EX|LOCK_NB)<0)throw std::runtime_error("Another robot tool holds the lock");
        entries.reserve(45000);
        {
            Arm arm((output/"timing.csv").string());meta["motor_parameters"]=arm.verify_parameters();
            State state=arm.query(false);
            // Retain the first sample BEFORE validation; a rejected startup is evidence too.
            // Only memory writes here; disk output remains outside the control session.
            meta["startup_feedback"]=Json{{"q",state.q},{"dq",state.dq},{"tau_feedback",state.tau},
                {"mos",state.mos},{"coil",state.coil},{"status",state.status},
                {"valid",state.sample_valid},{"raw",state.raw},{"time_s",state.time_s}};
            entries.push_back({state,state.q,0});
            require_disabled(state);checked(state);
            Vec baseline=state.q;
            for(int i=0;i<25;++i){auto t=Clock::now();state=arm.query(false);require_disabled(state);checked(state);
                for(int j=0;j<N;++j)if(std::abs(state.q[j]-baseline[j])>.003||std::abs(state.dq[j])>.04)
                    throw std::runtime_error("Arm not stationary during diagnosis");
                entries.push_back({state,state.q,0});std::this_thread::sleep_until(t+duration_s(PERIOD_S));}
            if(!execute&&!shadow){write_json(output/"snapshot.json",Json{{"q",state.q},{"dq",state.dq},{"status",state.status},
                {"parameters",meta["motor_parameters"]},{"unix_time",wall_seconds()}});meta["completed"]=true;}
            else{
                if(ipc<0)throw std::runtime_error("Motion requires local IPC launcher");
                const double capture=bounded(c,"home_capture_radius",.01,.10);
                for(int j=0;j<N;++j)if(!shadow&&(std::abs(state.q[j]-home[j])>capture||state.q[j]<QMIN[j]+.04||state.q[j]>QMAX[j]-.04))
                    throw std::runtime_error("Start outside reviewed home capture area");
                Vec ref=state.q,velocity{};publish(ipc,state,0,ref);
                auto request=receive_request(ipc,Clock::now()+duration_s(3));
                if(request.at("op")!=(shadow?"read":"enable")||request.at("seq")!=0)throw std::runtime_error("Missing handshake");
                auto expected=vector7(request,"expected_q");
                for(int j=0;j<N;++j)if(std::abs(expected[j]-state.q[j])>.003)throw std::runtime_error("Start moved since path review");
                state=arm.query(false);require_disabled(state);checked(state);
                for(int j=0;j<N;++j)if(std::abs(state.q[j]-ref[j])>.003)throw std::runtime_error("Start moved before enable");
                auto previous=Clock::now();state=shadow?arm.query(false):arm.enable_at(ref);
                if(shadow){require_disabled(state);checked(state);}else checked(state,&ref);int seq=1;
                entries.push_back({state,ref,0});publish(ipc,state,seq,ref);
                while(entries.size()<44000){
                    auto req=receive_request(ipc,previous+duration_s(.040));
                    if(req.at("seq").get<int>()!=seq)throw std::runtime_error("Stale/out-of-order IPC command");
                    const auto op=req.at("op").get<std::string>();
                    if(op=="done"&&shadow){meta["completed"]=true;meta["shadow_only"]=true;break;}
                    if(op=="supported"){
                        for(double v:velocity)if(std::abs(v)>1e-7)throw std::runtime_error("Support exit while reference moving");
                        meta["supported_exit_confirmed"]=true;meta["completed"]=true;break;}
                    if(op!="command")throw std::runtime_error("Stop requested by coordinator");
                    Vec next=vector7(req,"reference");if(!shadow)command_check(state,next,ref,velocity,c);
                    auto due=previous+duration_s(PERIOD_S);std::this_thread::sleep_until(due);check_stop();
                    double gap=elapsed(Clock::now(),previous);
                    // Important: detect delayed wake before sending the NEXT command.
                    pre_send_guard(gap,arm.now_s()-state.time_s);
                    previous=Clock::now();state=shadow?arm.query(false):arm.command(next);ref=next;
                    entries.push_back({state,ref,gap*1000});if(shadow){require_disabled(state);checked(state);}else checked(state,&ref);
                    arm.record_cycle(gap*1000,1000*std::max(0.0,gap-PERIOD_S));publish(ipc,state,++seq,ref);
                }
                if(!meta["completed"].get<bool>())throw std::runtime_error("Bounded session duration reached");
            }
        } // Arm destructor attempts disable BEFORE any bulk disk writes.
        save_entries(output,entries);write_json(output/"metadata.json",meta);
        if(lockfd>=0) { ::close(lockfd); }
        return 0;
    }catch(const std::exception&e){
        meta["error"]=e.what();
        std::cerr<<"STOP: "<<e.what();
        if(execute) {
            std::cerr<<"; if enabled, disable attempted, NOT confirmed.\n";
        } else {
            std::cerr<<"; read-only stage: no enable or MIT motion commands sent.\n";
        }
        try{if(created){save_entries(output,entries);write_json(output/"metadata.json",meta);}}catch(...){}
        if(lockfd>=0) { ::close(lockfd); }
        return 1;
    }
}
