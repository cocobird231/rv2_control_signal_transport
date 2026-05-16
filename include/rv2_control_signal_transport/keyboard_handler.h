/**
 * keyboard_handler.h
 *
 * Linux evdev keyboard reader that reads directly from /dev/input/eventX,
 * providing true per-key PRESS/HOLD/RELEASE events with full multi-key support.
 *
 * Unlike the previous terminal raw-mode approach (which read characters from
 * stdin and synthesized RELEASE via timeout), the evdev interface reports every
 * key's press and release independently at the hardware level.  This enables
 * combinations like W+A, W+D, S+A and S+D to work correctly.
 *
 * Permission requirement
 * ──────────────────────
 *   The process must have read access to /dev/input/eventX.
 *   Add the user to the 'input' group:
 *     sudo usermod -aG input $USER   (log out and back in)
 *   Or run as root.
 *
 * Recognized keys
 * ───────────────
 *   W  S  A  D         — movement (also ↑↓←→ arrow keys)
 *   Q  E  R  Z         — auxiliary / command
 *   1 … 7              — numeric commands
 *   SPACE              — stop / zero
 *   TAB                — mode switch
 *   CTRL_C (Ctrl+C)    — quit
 *   Any other key      — UNKNOWN (ignored)
 *
 * Events
 * ──────
 *   KeyEvent::PRESS    — key pressed (hardware down transition)
 *   KeyEvent::HOLD     — key still held (OS auto-repeat)
 *   KeyEvent::RELEASE  — key released (hardware up transition)
 *
 * Usage
 * ─────
 *   // Auto-detect first keyboard device:
 *   KeyboardHandler kb([](Key k, KeyEvent e){ handle(k, e); });
 *
 *   // Specify device explicitly:
 *   KeyboardHandler kb([](Key k, KeyEvent e){ handle(k, e); }, "/dev/input/event3");
 *
 *   // kb destroyed → background thread joined, terminal echo restored
 */

#pragma once

#include <linux/input.h>
#include <poll.h>
#include <fcntl.h>
#include <unistd.h>
#include <dirent.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <cstring>
#include <functional>
#include <stdexcept>
#include <string>
#include <thread>
#include <atomic>
#include <array>
#include <chrono>


namespace rv2_transport
{


/** Key transition event reported by KeyboardHandler. */
enum class KeyEvent
{
    PRESS,    ///< Key pressed (hardware down transition)
    HOLD,     ///< Key still held (OS auto-repeat, value=2 from evdev)
    RELEASE,  ///< Key released (hardware up transition)
};

inline const char * keyEventName(KeyEvent e) noexcept
{
    switch (e) {
    case KeyEvent::PRESS:   return "PRESS";
    case KeyEvent::HOLD:    return "HOLD";
    case KeyEvent::RELEASE: return "RELEASE";
    default:                return "UNKNOWN";
    }
}


enum class Key
{
    W, S, A, D,              // WASD — movement
    Q, E, R, Z,              // auxiliary keys
    K1, K2, K3, K4,
    K5, K6, K7,              // number keys 1-7
    ARROW_UP, ARROW_DOWN,
    ARROW_LEFT, ARROW_RIGHT,
    SPACE,
    TAB,
    CTRL_C,
    UNKNOWN
};

inline const char * keyName(Key k) noexcept
{
    switch (k) {
    case Key::W:            return "W";
    case Key::S:            return "S";
    case Key::A:            return "A";
    case Key::D:            return "D";
    case Key::Q:            return "Q";
    case Key::E:            return "E";
    case Key::R:            return "R";
    case Key::Z:            return "Z";
    case Key::K1:           return "1";
    case Key::K2:           return "2";
    case Key::K3:           return "3";
    case Key::K4:           return "4";
    case Key::K5:           return "5";
    case Key::K6:           return "6";
    case Key::K7:           return "7";
    case Key::ARROW_UP:     return "UP";
    case Key::ARROW_DOWN:   return "DOWN";
    case Key::ARROW_LEFT:   return "LEFT";
    case Key::ARROW_RIGHT:  return "RIGHT";
    case Key::SPACE:        return "SPACE";
    case Key::TAB:          return "TAB";
    case Key::CTRL_C:       return "CTRL_C";
    default:                return "UNKNOWN";
    }
}


class KeyboardHandler
{
public:
    using KeyCb = std::function<void(Key, KeyEvent)>;

    /**
     * @brief Construct and start the background keyboard reader.
     *
     * Reads from evdev by default for true per-key events.
     * If auto-detect fails and stdin is a TTY (e.g. noVNC terminal),
     * it falls back to stdin raw-mode decoding.
     * @param cb          Callback invoked for every key event (PRESS / HOLD / RELEASE).
     * @param devicePath  Path to the evdev keyboard device (e.g. "/dev/input/event3").
     *                    Leave empty (default) to auto-detect.
     *                    Use "stdin" (or "tty") to force terminal input mode.
     */
    explicit KeyboardHandler(KeyCb cb, const std::string & devicePath = "")
        : cb_(std::move(cb))
        , running_(true)
        , mode_(InputMode::EVDEV)
        , fd_(-1)
        , ownsFd_(false)
    {
        // Determine whether we want terminal (stdin) mode or evdev mode.
        const bool wantStdin = _isStdinPath(devicePath);
        const bool wantX11   = devicePath.empty() && _isX11Session();

        if (wantStdin || wantX11) {
            // /dev/tty is the process's controlling terminal and is readable even
            // when stdin/stdout/stderr are redirected by ros2 launch.  Open it so
            // that keyboard events from the noVNC terminal reach us correctly.
            auto [tfd, tOwns] = _openTty();
            if (tfd >= 0) {
                _applyRawMode(tfd);
                mode_    = InputMode::STDIN;
                fd_      = tfd;
                ownsFd_  = tOwns;
            } else if (!wantStdin) {
                // X11 auto-detect, but no controlling terminal — fall through to evdev.
            } else {
                // Explicitly requested "stdin"/"tty" but no terminal is available.
                // Rather than aborting, fall through to evdev auto-detect.
            }
        }

        if (fd_ < 0) {
            // evdev mode: explicit path or auto-detect.
            const bool tryAutoDetect = devicePath.empty() || wantStdin;
            const int efd = tryAutoDetect
                ? _autoDetect()
                : ::open(devicePath.c_str(), O_RDONLY);
            if (efd >= 0) {
                mode_   = InputMode::EVDEV;
                fd_     = efd;
                ownsFd_ = true;
            }
        }

        if (fd_ < 0) {
            throw std::runtime_error(
                std::string("KeyboardHandler: cannot open input source") +
                (devicePath.empty() || wantStdin
                    ? " (no controlling terminal and evdev auto-detect failed)"
                    : (": " + devicePath)));
        }

        thread_ = std::thread([this]() { _run(); });
    }

    /** @brief Stop the background thread and restore terminal settings. */
    ~KeyboardHandler()
    {
        running_.store(false, std::memory_order_relaxed);
        if (thread_.joinable())
            thread_.join();
        if (savedTermios_)
            ::tcsetattr(termiosFd_, TCSAFLUSH, &origTermios_);
        if (ownsFd_ && fd_ >= 0)
            ::close(fd_);
    }

    // Non-copyable / non-movable.
    KeyboardHandler(const KeyboardHandler &)             = delete;
    KeyboardHandler & operator=(const KeyboardHandler &) = delete;

private:
    enum class InputMode
    {
        EVDEV,
        STDIN,
    };

    struct KeyState
    {
        bool down{false};
        std::chrono::steady_clock::time_point lastEvent{};
    };

    // ── Background read loop ─────────────────────────────────────────────────

    void _run()
    {
        if (mode_ == InputMode::EVDEV) {
            _runEvdev();
        } else {
            _runStdin();
        }
    }

    void _runEvdev()
    {
        bool ctrlHeld = false;

        while (running_.load(std::memory_order_relaxed)) {
            // poll() with timeout so we can check running_ periodically.
            struct pollfd pfd { fd_, POLLIN, 0 };
            if (::poll(&pfd, 1, 100) <= 0)
                continue;

            struct input_event ev;
            const ssize_t n = ::read(fd_, &ev, sizeof(ev));
            if (n != static_cast<ssize_t>(sizeof(ev)))
                continue;
            if (ev.type != EV_KEY)
                continue;

            // Track Ctrl modifier for Ctrl+C detection.
            if (ev.code == KEY_LEFTCTRL || ev.code == KEY_RIGHTCTRL) {
                ctrlHeld = (ev.value != 0);
                continue;
            }

            // Ctrl+C — emit as PRESS so the node can call rclcpp::shutdown().
            if (ev.code == KEY_C && ctrlHeld && ev.value == 1) {
                cb_(Key::CTRL_C, KeyEvent::PRESS);
                continue;
            }

            const Key k = _decode(ev.code);
            if (k == Key::UNKNOWN)
                continue;

            KeyEvent e;
            switch (ev.value) {
            case 0:  e = KeyEvent::RELEASE; break;
            case 1:  e = KeyEvent::PRESS;   break;
            case 2:  e = KeyEvent::HOLD;    break;
            default: continue;
            }

            cb_(k, e);
        }
    }

    void _runStdin()
    {
        // Terminal input has no hardware key-up event; synthesize RELEASE
        // when repeats stop for a short interval.
        static constexpr int kPollMs = 25;
        static constexpr std::chrono::milliseconds kReleaseAfter(400);

        std::array<KeyState, static_cast<std::size_t>(Key::UNKNOWN) + 1> state{};

        while (running_.load(std::memory_order_relaxed)) {
            struct pollfd pfd { fd_, POLLIN, 0 };   // fd_ is /dev/tty or STDIN_FILENO
            const int pr = ::poll(&pfd, 1, kPollMs);
            const auto now = std::chrono::steady_clock::now();

            if (pr > 0 && (pfd.revents & POLLIN)) {
                const Key k = _readStdinKey(fd_);
                if (k != Key::UNKNOWN) {
                    auto & ks = state[_idx(k)];
                    cb_(k, ks.down ? KeyEvent::HOLD : KeyEvent::PRESS);
                    ks.down = true;
                    ks.lastEvent = now;
                }
            }

            for (std::size_t i = 0; i < state.size(); ++i) {
                auto & ks = state[i];
                if (!ks.down)
                    continue;
                if (now - ks.lastEvent >= kReleaseAfter) {
                    ks.down = false;
                    cb_(static_cast<Key>(i), KeyEvent::RELEASE);
                }
            }
        }
    }

    static std::size_t _idx(Key k) noexcept
    {
        return static_cast<std::size_t>(k);
    }

    static bool _isStdinPath(const std::string & devicePath) noexcept
    {
        return devicePath == "stdin" || devicePath == "tty";
    }

    /// Returns true when the process is running inside an X11 session
    /// (local display, VNC, noVNC, SSH with X forwarding, etc.).
    /// In such sessions, keyboard events reach the process via the display
    /// server → terminal emulator → the controlling terminal, NOT /dev/input/eventX.
    static bool _isX11Session() noexcept
    {
        const char * display = ::getenv("DISPLAY");
        return display != nullptr && display[0] != '\0';
    }

    /// Open the process's controlling terminal for raw keyboard input.
    /// /dev/tty always refers to the controlling terminal regardless of whether
    /// stdin/stdout/stderr have been redirected (e.g. by ros2 launch).
    /// Returns {fd, ownsFd}; fd is -1 if no terminal is available.
    static std::pair<int, bool> _openTty() noexcept
    {
        const int fd = ::open("/dev/tty", O_RDONLY | O_NOCTTY);
        if (fd >= 0)
            return {fd, true};
        if (::isatty(STDIN_FILENO))
            return {STDIN_FILENO, false};
        return {-1, false};
    }

    /// Suppress terminal echo and disable line-buffering on `fd`.
    /// Saves the original termios so the destructor can restore it.
    void _applyRawMode(int fd) noexcept
    {
        if (!::isatty(fd))
            return;
        if (::tcgetattr(fd, &origTermios_) != 0)
            return;
        savedTermios_ = true;
        termiosFd_    = fd;
        ::termios t = origTermios_;
        // Disable echo and canonical mode.  Keep ISIG so Ctrl+C still sends SIGINT.
        t.c_lflag &= static_cast<::tcflag_t>(~(ECHO | ICANON));
        ::tcsetattr(fd, TCSAFLUSH, &t);
    }

    static bool _readByteWithTimeout(int fd, uint8_t & out, int timeoutMs)
    {
        struct pollfd pfd { fd, POLLIN, 0 };
        if (::poll(&pfd, 1, timeoutMs) <= 0 || !(pfd.revents & POLLIN))
            return false;

        char c = 0;
        if (::read(fd, &c, 1) != 1)
            return false;
        out = static_cast<uint8_t>(c);
        return true;
    }

    static Key _decodeAscii(uint8_t c) noexcept
    {
        switch (c) {
        case 'w': case 'W': return Key::W;
        case 's': case 'S': return Key::S;
        case 'a': case 'A': return Key::A;
        case 'd': case 'D': return Key::D;
        case 'q': case 'Q': return Key::Q;
        case 'e': case 'E': return Key::E;
        case 'r': case 'R': return Key::R;
        case 'z': case 'Z': return Key::Z;
        case '1': return Key::K1;
        case '2': return Key::K2;
        case '3': return Key::K3;
        case '4': return Key::K4;
        case '5': return Key::K5;
        case '6': return Key::K6;
        case '7': return Key::K7;
        case ' ': return Key::SPACE;
        case '\t': return Key::TAB;
        case 0x03: return Key::CTRL_C; // Ctrl+C
        default:   return Key::UNKNOWN;
        }
    }

    static Key _readStdinKey(int fd)
    {
        uint8_t c0 = 0;
        if (!_readByteWithTimeout(fd, c0, 0))
            return Key::UNKNOWN;

        if (c0 != 0x1b)
            return _decodeAscii(c0);

        // Arrow keys typically arrive as ESC [ A/B/C/D.
        uint8_t c1 = 0;
        if (!_readByteWithTimeout(fd, c1, 5))
            return Key::UNKNOWN;
        if (c1 != '[')
            return Key::UNKNOWN;

        uint8_t c2 = 0;
        if (!_readByteWithTimeout(fd, c2, 5))
            return Key::UNKNOWN;

        switch (c2) {
        case 'A': return Key::ARROW_UP;
        case 'B': return Key::ARROW_DOWN;
        case 'C': return Key::ARROW_RIGHT;
        case 'D': return Key::ARROW_LEFT;
        default:  return Key::UNKNOWN;
        }
    }

    // ── Linux key-code → Key mapping ─────────────────────────────────────────

    static Key _decode(uint16_t code) noexcept
    {
        switch (code) {
        case KEY_W:      return Key::W;
        case KEY_S:      return Key::S;
        case KEY_A:      return Key::A;
        case KEY_D:      return Key::D;
        case KEY_Q:      return Key::Q;
        case KEY_E:      return Key::E;
        case KEY_R:      return Key::R;
        case KEY_Z:      return Key::Z;
        case KEY_1:      return Key::K1;
        case KEY_2:      return Key::K2;
        case KEY_3:      return Key::K3;
        case KEY_4:      return Key::K4;
        case KEY_5:      return Key::K5;
        case KEY_6:      return Key::K6;
        case KEY_7:      return Key::K7;
        case KEY_UP:     return Key::ARROW_UP;
        case KEY_DOWN:   return Key::ARROW_DOWN;
        case KEY_LEFT:   return Key::ARROW_LEFT;
        case KEY_RIGHT:  return Key::ARROW_RIGHT;
        case KEY_SPACE:  return Key::SPACE;
        case KEY_TAB:    return Key::TAB;
        default:         return Key::UNKNOWN;
        }
    }

    // ── Auto-detection: find first evdev device with letter keys ─────────────

    static bool _testBit(int bit, const uint8_t * arr) noexcept
    {
        return (arr[bit / 8] >> (bit % 8)) & 1u;
    }

    static int _autoDetect()
    {
        DIR * d = ::opendir("/dev/input");
        if (!d) return -1;

        int result = -1;
        struct dirent * ent;
        while (result < 0 && (ent = ::readdir(d)) != nullptr) {
            if (::strncmp(ent->d_name, "event", 5) != 0)
                continue;

            const std::string path = "/dev/input/" + std::string(ent->d_name);
            const int fd = ::open(path.c_str(), O_RDONLY | O_NONBLOCK);
            if (fd < 0) continue;

            // Check EV_KEY capability.
            uint8_t evBits[(EV_MAX + 7) / 8] = {};
            if (::ioctl(fd, EVIOCGBIT(0, sizeof(evBits)), evBits) >= 0 &&
                _testBit(EV_KEY, evBits))
            {
                // Must have WASD keys — distinguishes keyboards from mice/joysticks.
                uint8_t keyBits[(KEY_MAX + 7) / 8] = {};
                if (::ioctl(fd, EVIOCGBIT(EV_KEY, sizeof(keyBits)), keyBits) >= 0 &&
                    _testBit(KEY_W, keyBits) && _testBit(KEY_A, keyBits))
                {
                    ::close(fd);
                    result = ::open(path.c_str(), O_RDONLY);  // reopen in blocking mode
                    continue;
                }
            }
            ::close(fd);
        }
        ::closedir(d);
        return result;
    }

    // ── Members ──────────────────────────────────────────────────────────────

    KeyCb              cb_;
    std::atomic<bool>  running_;
    InputMode          mode_;
    std::thread        thread_;
    int                fd_;
    bool               ownsFd_;
    int                termiosFd_{-1};  // fd on which raw mode was applied
    bool               savedTermios_{false};
    ::termios          origTermios_{};
};

} // namespace rv2_transport
