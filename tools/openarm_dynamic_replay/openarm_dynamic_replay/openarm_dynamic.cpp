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
    double speed=.02, accel=.04, max_temp=55;
    double startup_tolerance=.005;
};
constexpr const char* VERSION="openarm-dynamic-replay-v1";
Settings load_settings(const Json& p) {
    if(p.at("schema")!="openarm-dynamic-v1" || !p.at("model_path_checked").get<bool>())
        throw std::runtime_error("Not a screened dynamic plan");
    const auto& c=p.at("config");
    QMIN=vector7(c,"q_min_rad"); QMAX=vector7(c,"q_max_rad");
    KP=vector7(c,"kp"); KD=vector7(c,"kd");
    TORQUE_ALARM=vector7(c,"torque_alarm_nm"); SPEED_ALARM=vector7(c,"speed_alarm_rad_s");
    TRACK_ALARM=bounded(c,"tracking_alarm_rad",.01,.16);
    const Vec low{-1.396263,-.174533,-1.570796,0,-1.570796,-.785398,-1.570796};
    const Vec high{3.490659,3.316125,1.570796,2.443461,1.570796,.785398,1.570796};
    const Vec kp{35,35,20,20,6,6,6},kd{4,4,2,2,.8,.8,.8};
    const auto profile=c.at("torque_profile").get<std::string>();
    if(profile!="legacy_low" && profile!="proximal_5_3") throw std::runtime_error("Unknown torque profile");
    const Vec caps=profile=="proximal_5_3"?Vec{5,5,3,3,.7,.7,.7}:Vec{2,2,1.5,1.5,.7,.7,.7};
    Settings s;
    s.speed=bounded(c,"transit_speed_rad_s",.005,.05);
    s.accel=bounded(c,"transit_accel_rad_s2",.005,.08);
    s.max_temp=bounded(c,"temperature_alarm_c",25,55);
    s.startup_tolerance=bounded(c,"startup_tolerance_rad",.0005,.005);
    for(int j=0;j<N;++j) if(KP[j]!=kp[j] || KD[j]!=kd[j] || QMIN[j]<low[j]-1e-8 || QMAX[j]>high[j]+1e-8 ||
        QMIN[j]>=QMAX[j] || TORQUE_ALARM[j]<=0 || TORQUE_ALARM[j]>caps[j] ||
        SPEED_ALARM[j]<=s.speed || SPEED_ALARM[j]>.5)
        throw std::runtime_error("Settings exceed inherited profile J"+std::to_string(j+1));
    if(c.value("trim_j2",false)) throw std::runtime_error("Dynamic recorder does not support reference trim");
    const Vec base=vector7(p,"baseline_q_motor_rad"); validate_target(base);
    const double amplitude=bounded(c,"amplitude_deg",1,15)*3.141592653589793/180;
    const auto& points=p.at("poses");
    if(!points.is_array() || points.empty() || points.size()>90) throw std::runtime_error("Invalid trajectory size");
    for(const auto& point:points) {
        auto q=vector7(point,"q_motor_rad"); validate_target(q);
        int changed=0;
        for(int j=0;j<N;++j) {
            if(std::abs(q[j]-base[j])>amplitude+1e-8) throw std::runtime_error("Excursion exceeds plan");
            if(std::abs(q[j]-base[j])>1e-8)++changed;
        }
        if(changed>1)throw std::runtime_error("v1 supports single-axis excitation only");
        bounded(point,"hold_s",1,5);
        auto name=point.at("pose_id").get<std::string>();
        if(name.empty() || name.find_first_not_of("abcdefghijklmnopqrstuvwxyzABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789_-")!=std::string::npos)
            throw std::runtime_error("Invalid phase name");
    }
    const auto last=vector7(points.back(),"q_motor_rad");
    for(int j=0;j<N;++j)if(std::abs(last[j]-base[j])>1e-9)throw std::runtime_error("Last target must return to baseline");
    return s;
}
Json last_safety_fault;
Json accepted_sample_windows=Json::object();
Json trim_records=Json::object();
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


double effective(double x,double low,double high,int bits) {
    const auto n=(1u<<bits)-1;
    return low+double(uint64_t((x-low)/(high-low)*n))*(high-low)/n;
}
class Recorder {
    std::ofstream out;
    int rows=0;
public:
    explicit Recorder(const fs::path& p):out(p) {
        if(!out)throw std::runtime_error("Cannot create samples.csv");
        out<<std::setprecision(12)<<"time_s,stage,pose_id,feedback_valid,command_sent,transaction_ms,rx_skew_ms,max_kernel_age_ms,cycle_gap_ms";
        for(const auto prefix:{"q","dq","tau","cmd","rx","tx","status","mos","coil","cmd_effective","kp_effective","kd_effective","vdes_effective","ff_effective"})
            for(int j=1;j<=N;++j)out<<','<<prefix<<j;
        for(int j=1;j<=N;++j)for(int b=0;b<8;++b)out<<",raw"<<j<<'_'<<b;
        out<<'\n';out.flush();
    }
    void row(const State& s,const std::string& stage,const std::string& id,const Vec& cmd,double gap) {
        out<<s.time_s<<','<<stage<<','<<id<<','<<s.sample_valid<<','<<s.command_sent<<','<<s.transaction_ms<<','<<s.rx_skew_ms<<','<<s.max_kernel_age_ms<<','<<gap;
        for(const auto& v:{s.q,s.dq,s.tau,cmd,s.rx_time,s.tx_time})for(double x:v)out<<','<<x;
        for(int x:s.status)out<<','<<x;
        for(const auto& v:{s.mos,s.coil})for(double x:v)out<<','<<x;
        for(int kind=0;kind<5;++kind)for(int j=0;j<N;++j){
            double x=0;
            if(kind==0)x=effective(cmd[j],-PMAX[j],PMAX[j],16);
            if(kind==1)x=effective(KP[j],0,500,12);
            if(kind==2)x=effective(KD[j],0,5,12);
            if(kind==3)x=effective(0,-VMAX[j],VMAX[j],12);
            if(kind==4)x=effective(0,-TMAX[j],TMAX[j],12);
            if(s.command_sent)out<<','<<x;else out<<','; // No fictitious command in diagnosis.
        }
        for(const auto& bytes:s.raw)for(unsigned b:bytes)out<<','<<b;
        out<<'\n';if(++rows%20==0)out.flush();
        if(!out)throw std::runtime_error("Data log write failed");
    }
    void flush(){out.flush();if(!out)throw std::runtime_error("Log flush failed");}
};
void tick(Arm& arm,State& state,const Vec& command,const Settings& s,Recorder& log,Cycle& cycle,
          const std::string& stage,const std::string& id) {
    safety_phase=stage;safety_pose=id;
    cycle.begin();predicted_guard(state,command);
    state=arm.command(command);
    log.row(state,stage,id,command,cycle.gap_ms); // Includes offending received frame before guard.
    guard(state,s,&command);
    if(!state.sample_valid || cycle.gap_ms>75)throw std::runtime_error("Invalid acquisition timing (>75ms gap / stale feedback)");
    cycle.finish(arm);
}
void fixed_hold(Arm& arm,State& state,const Vec& cmd,const Settings& s,Recorder& log,Cycle& cycle,
                double seconds,const std::string& stage,const std::string& id) {
    const auto start=Clock::now();
    do {tick(arm,state,cmd,s,log,cycle,stage,id);}while(elapsed(Clock::now(),start)<seconds);
}
void verified_hold(Arm& arm,State& state,const Vec& cmd,const Settings& s,Recorder& log,Cycle& cycle,
                   double tolerance,const std::string& stage) {
    const auto start=Clock::now();std::deque<State> window;
    while(elapsed(Clock::now(),start)<30) {
        tick(arm,state,cmd,s,log,cycle,stage,stage);
        bool ok=true;
        for(int j=0;j<N;++j)ok=ok && std::abs(state.dq[j])<=.04 && std::abs(state.q[j]-cmd[j])<=tolerance;
        if(!ok)window.clear();else window.push_back(state);
        if(window.size()>21)window.pop_front();
        if(window.size()==21){
            for(int j=0;j<N;++j){double low=state.q[j],high=low;
                for(const auto& x:window){low=std::min(low,x.q[j]);high=std::max(high,x.q[j]);}
                ok=ok && high-low<=.003;
            }
            if(ok)return;
        }
    }
    throw std::runtime_error(stage+": stable initial/return hold not achieved; no gain/limit changes");
}
void ramp(Arm& arm,State& state,Vec& command,const Vec& target,const Settings& s,Recorder& log,Cycle& cycle,const std::string& id) {
    const Vec start=command;double distance=0;
    for(int j=0;j<N;++j)distance=std::max(distance,std::abs(target[j]-start[j]));
    const double duration=std::max({PERIOD_S,1.875*distance/s.speed,std::sqrt(5.774*distance/s.accel)});
    const auto begin=Clock::now();double u=0;
    do {
        u=std::min(1.0,elapsed(Clock::now(),begin)/duration);
        const double blend=u*u*u*(10+u*(-15+6*u));
        for(int j=0;j<N;++j)command[j]=start[j]+(target[j]-start[j])*blend;
        tick(arm,state,command,s,log,cycle,"transit",id);
    }while(u<1);
}
} // namespace
int main(int argc,char**argv) {
    std::signal(SIGINT,signal_handler);std::signal(SIGTERM,signal_handler);
    bool execute=false;fs::path plan_path,root,dir;Json meta{{"completed",false},{"version",VERSION},{"period_s",PERIOD_S}};
    try {
        for(int i=1;i<argc;++i){std::string a=argv[i];
            if(a=="--version"){std::cout<<VERSION<<'\n';return 0;}
            else if(a=="--help"){std::cout<<"openarm_dynamic --diagnose | --execute --plan FILE [--interface can0] [--output-root DIR]\n";return 0;}
            else if(a=="--execute")execute=true;
            else if(a=="--diagnose"){}
            else if((a=="--plan" || a=="--output-root" || a=="--interface") && i+1<argc){
                auto value=std::string(argv[++i]);if(a=="--plan")plan_path=value;else if(a=="--interface")CAN_INTERFACE=value;else root=value;
            }else throw std::runtime_error("Unknown/incomplete option: "+a);
        }
        if(CAN_INTERFACE!="can0")throw std::runtime_error("v1 is configured only for can0; verify the adapter channel");
        if(execute!=!plan_path.empty())throw std::runtime_error("Use --execute with --plan, or --diagnose without plan");
        if(root.empty())root=fs::path(std::getenv("HOME")?std::getenv("HOME"):".")/"experiments/dynamic_replay";
        fs::create_directories(root);
        const auto stamp=std::chrono::duration_cast<std::chrono::nanoseconds>(WallClock::now().time_since_epoch()).count();
        dir=root/((execute?"dynamic_":"diagnose_")+std::to_string(stamp));
        if(!fs::create_directory(dir))throw std::runtime_error("Output already exists");
        meta["mode"]=execute?"dynamic_motion":"diagnose";meta["interface"]=CAN_INTERFACE;
        meta["torque_source"]="motor feedback estimate; output-shaft physical meaning unverified";
        meta["acquisition_completed"]=false;meta["return_completed"]=false;meta["supported_exit_confirmed"]=false;
        Json plan;Settings settings;
        if(execute){plan=read_json(plan_path);settings=load_settings(plan);meta["plan"]=plan;
            fs::copy_file(plan_path.parent_path()/"model.mjb",dir/"model.mjb");write_json(dir/"plan.json",plan);}
        write_json(dir/"metadata.json",meta);
        std::cout<<"Logs: "<<dir<<"\nJ1-J7 only. Close other control/query tools.\n";
        Arm arm((dir/"timing.csv").string());
        meta["motor_parameters"]=arm.verify_parameters();
        State state=arm.query(false);require_disabled(state);
        if(!state.sample_valid)throw std::runtime_error("Initial feedback invalid");
        Recorder log(dir/"samples.csv");Cycle cycle;
        const Vec baseline=state.q;
        log.row(state,"startup","startup",baseline,0);
        write_json(dir/"snapshot.json",Json{{"schema","openarm-snapshot-v1"},{"q_motor_rad",baseline},{"interface",CAN_INTERFACE},{"run_time_ns",stamp},{"motor_parameters",meta["motor_parameters"]}});
        if(!execute){for(int i=0;i<60;++i){cycle.begin();state=arm.query(false);require_disabled(state);
                log.row(state,"diagnose","diagnose",baseline,cycle.gap_ms);
                if(!state.sample_valid)throw std::runtime_error("Diagnosis timing invalid");cycle.finish(arm);}
            log.flush();meta["completed"]=true;write_json(dir/"metadata.json",meta);return 0;}
        guard(state,settings);const Vec expected=vector7(plan,"baseline_q_motor_rad");
        for(int j=0;j<N;++j)if(std::abs(baseline[j]-expected[j])>settings.startup_tolerance)throw std::runtime_error("Baseline changed: re-prepare");
        std::cout<<"MIT position-driven trajectory, not open-loop torque control. No J2 trim.\n"
                 <<"Fall protection must cover the full path and power loss, without loading the arm during acquisition.\n"
                 <<"Ctrl+C/fault attempts disable, without acknowledgement guarantee.\nType RUN_DYNAMIC to enable and move: "<<std::flush;
        std::string answer;std::getline(std::cin,answer);if(answer!="RUN_DYNAMIC")throw std::runtime_error("Motion not confirmed");
        state=arm.query(false);require_disabled(state);guard(state,settings);
        if(!state.sample_valid)throw std::runtime_error("Fresh feedback invalid");
        for(int j=0;j<N;++j)if(std::abs(state.q[j]-expected[j])>settings.startup_tolerance || std::abs(state.dq[j])>.04)
            throw std::runtime_error("Start changed during confirmation; re-prepare");
        Vec command=state.q;
        state=arm.enable_at(command);guard(state,settings,&command);
        verified_hold(arm,state,command,settings,log,cycle,.04,"initial_hold");
        // Align to the exact screened baseline smoothly; subsequent ramps start
        // from previous COMMAND, never reset abruptly to measured position.
        ramp(arm,state,command,expected,settings,log,cycle,"align_baseline");
        fixed_hold(arm,state,command,settings,log,cycle,2,"record_hold","start");
        for(const auto& p:plan.at("poses")){
            const auto id=p.at("pose_id").get<std::string>();
            ramp(arm,state,command,vector7(p,"q_motor_rad"),settings,log,cycle,id);
            fixed_hold(arm,state,command,settings,log,cycle,p.at("hold_s").get<double>(),"record_hold",id);
            std::cout<<id<<": recorded; no precise-arrival requirement\n";
        }
        meta["acquisition_completed"]=true;write_json(dir/"metadata.json",meta);
        verified_hold(arm,state,command,settings,log,cycle,.08,"return_hold");
        meta["return_completed"]=true;write_json(dir/"metadata.json",meta);
        std::cout<<"HOLDING near baseline. Install support then type SUPPORTED + Enter.\n";
        std::string line;
        while(true){pollfd in{STDIN_FILENO,POLLIN,0};
            if(::poll(&in,1,0)>0){if(in.revents&POLLIN){char ch;if(::read(STDIN_FILENO,&ch,1)<=0)throw std::runtime_error("Console EOF");
                if(ch=='\n'){if(line=="SUPPORTED")break;line.clear();}else if(ch!='\r'){line+=ch;if(line.size()>128)line.clear();}}
                else if(in.revents&(POLLHUP|POLLERR))throw std::runtime_error("Console closed");}
            tick(arm,state,command,settings,log,cycle,"end_hold","end_hold");
        }
        log.flush();meta["supported_exit_confirmed"]=true;meta["completed"]=true;
        meta["disable_confirmed"]=false;write_json(dir/"metadata.json",meta);
        return 0; // Arm destructor attempts disable after physical support.
    }catch(const std::exception& e){meta["error"]=e.what();meta["safety_fault"]=last_safety_fault;
        if(!dir.empty())try{write_json(dir/"metadata.json",meta);}catch(...){}
        std::cerr<<"STOP: "<<e.what()<<"\nIf enabled, best-effort disable was attempted; verify physical motor state.\n";return 1;}
}
