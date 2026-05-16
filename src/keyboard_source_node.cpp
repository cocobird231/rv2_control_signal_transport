/**
 * keyboard_source_node.cpp
 *
 * Composable ROS 2 node that reads raw keyboard input and publishes it as
 * a single control signal source (Joy OR Twist) through the
 * ControlSignalManager (CSM) infrastructure.
 *
 * The message type is fixed at startup by the `initial_msg_type` parameter.
 * Exactly one source is registered with the target server's CSM.
 * Use separate launch files (keyboard_source_joy / keyboard_source_twist)
 * to select the type without exposing it as a runtime argument.
 *
 * ── Parameters ────────────────────────────────────────────────────────────────
 *  server_name           (string, "control_server")  — target CSM (ControlServerNode)
 *  csm_name              (string, "keyboard_source")  — this node's own CSM name
 *  channel_name          (string, "keyboard_control") — source channel name
 *  priority              (int64,  50)                 — source priority (1-100)
 *  timeout_ms            (int64,  2000)               — inactivity timeout before TIMEOUT state
 *  disconnect_timeout_ms (int64,  10000)              — ms in TIMEOUT before CSM removes source (0=never)
 *  initial_msg_type      (string, "joy")              — "joy" or "twist" (fixed)
 *  max_setpoint_linear   (double, 1.0)                — maximum linear setpoint (throttle ceiling)
 *  max_setpoint_angular  (double, 1.0)                — maximum angular setpoint (throttle ceiling)
 *  ramp_step_linear      (double, 0.05)               — linear setpoint change per send tick
 *  ramp_step_angular     (double, 0.05)               — angular setpoint change per send tick
 *  send_rate_ms          (int64,  100)                — publish interval (ms)
 *
 * ── Control mode (Throttle) ───────────────────────────────────────────────────
 *  Movement axes ramp toward max_setpoint while the key is held and ramp back
 *  to zero when the key is released.  Command keys (1-7, E, R, Z/SPACE) fire
 *  as one-shot events on PRESS only and are unaffected by throttle logic.
 *
 * ── Key bindings (Joy mode) ───────────────────────────────────────────────────
 *  W / ↑      forward     (axes[0] → +max_setpoint_linear)
 *  S / ↓      backward    (axes[1] → +max_setpoint_linear)
 *  A / ←      turn left   (axes[2] → +max_setpoint_angular)
 *  D / →      turn right  (axes[3] → +max_setpoint_angular)
 *  1          Damp                    (button[0])
 *  2          StandUp                 (button[1])
 *  3          StandDown               (button[2])
 *  4          StopMove                (button[6])
 *  5          SwitchGait 0            (button[7])
 *  6          SwitchGait 1            (button[8])
 *  7          RecoveryStand           (button[9])
 *  E          E-stop                  (buttons[0..3] = -99)
 *  R          Request-active          (buttons[0..3] = 99)
 *  SPACE      Zero all axes/buttons (also clears held movement keys)
 *
 * ── Key bindings (Twist mode) ────────────────────────────────────────────────
 *  W / ↑      forward     (linear.x → +max_setpoint_linear)
 *  S / ↓      backward    (linear.x → -max_setpoint_linear)
 *  A / ←      turn left   (angular.z → +max_setpoint_angular)
 *  D / →      turn right  (angular.z → -max_setpoint_angular)
 *  Q          strafe left  (linear.y → +max_setpoint_linear)
 *  E          strafe right (linear.y → -max_setpoint_linear)
 *  Z          E-stop       (linear.z = angular.x = angular.y = -99)
 *  R          Request-active          (= +99)
 *  SPACE      Zero all (also clears held movement keys)
 *
 * ── Notes ─────────────────────────────────────────────────────────────────────
 *  The node requires component_container_mt (multi-threaded) because
 *  ControlSignalManager::registerSource() blocks until the target responds.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <rv2_interfaces/msg/control_signal_const.hpp>
#include <rv2_interfaces/msg/control_signal_info.hpp>
#include "rv2_server_control/control_signal_manager.h"
#include "rv2_control_signal_transport/keyboard_handler.h"

#include <sensor_msgs/msg/joy.hpp>
#include <geometry_msgs/msg/twist.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <mutex>
#include <set>
#include <thread>
#include <memory>


using Joy   = sensor_msgs::msg::Joy;
using Twist = geometry_msgs::msg::Twist;
namespace CSC = rv2_interfaces::msg;

using rv2_interfaces::ControlSignalManager;
using rv2_interfaces::ControlSignalSource;
using rv2_transport::Key;
using rv2_transport::KeyEvent;
using rv2_transport::KeyboardHandler;


// ── Helpers ───────────────────────────────────────────────────────────────────

static constexpr std::size_t JOY_AXES    = 4;   // [fwd, bck, left, right]
static constexpr std::size_t JOY_BUTTONS = 12;  // index up to [11]


class KeyboardSourceNode : public rclcpp::Node
{
public:
    explicit KeyboardSourceNode(const rclcpp::NodeOptions & options)
        : Node("keyboard_source", options)
    {
        // ── Declare & read parameters ─────────────────────────────────────────
        declare_parameter<std::string>("server_name",                 "control_server");
        declare_parameter<std::string>("csm_name",                     "keyboard_source");
        declare_parameter<std::string>("channel_name",                 "keyboard_control");
        declare_parameter<int64_t>    ("priority",                     50LL);
        declare_parameter<int64_t>    ("timeout_ms",                   2000LL);
        declare_parameter<int64_t>    ("disconnect_timeout_ms",        10000LL);
        declare_parameter<std::string>("initial_msg_type",             "joy");
        declare_parameter<double>     ("max_setpoint_linear",          1.0);
        declare_parameter<double>     ("max_setpoint_angular",         1.0);
        declare_parameter<double>     ("ramp_step_linear",             0.05);
        declare_parameter<double>     ("ramp_step_angular",            0.05);
        declare_parameter<int64_t>    ("send_rate_ms",                 100LL);
        declare_parameter<int64_t>    ("csm_status_timer_interval_ms", 1000LL);
        declare_parameter<std::string>("keyboard_device",              "");  // "" = auto-detect, "stdin"/"tty" = force stdin, "/dev/input/eventX" = force evdev

        serverName_         = get_parameter("server_name").as_string();
        csmName_            = get_parameter("csm_name").as_string();
        priority_           = static_cast<int8_t>(get_parameter("priority").as_int());
        timeoutNs_          = get_parameter("timeout_ms").as_int() * 1'000'000LL;
        disconnectTimeoutNs_= get_parameter("disconnect_timeout_ms").as_int() * 1'000'000LL;
        maxSetpointLinear_  = get_parameter("max_setpoint_linear").as_double();
        maxSetpointAngular_ = get_parameter("max_setpoint_angular").as_double();
        rampStepLinear_     = get_parameter("ramp_step_linear").as_double();
        rampStepAngular_    = get_parameter("ramp_step_angular").as_double();
        sendRateMs_         = get_parameter("send_rate_ms").as_int();
        const int64_t csmStatusTimerMs = get_parameter("csm_status_timer_interval_ms").as_int();
        keyboardDevice_     = get_parameter("keyboard_device").as_string();

        // Derive send_freq_hz from send_rate_ms (0 disables frequency checking).
        sendFreqHz_ = (sendRateMs_ > 0) ? (1000.0f / static_cast<float>(sendRateMs_)) : 0.0f;

        {
            const std::string t = get_parameter("initial_msg_type").as_string();
            isJoy_ = (t != "twist");
        }

        channel_ = get_parameter("channel_name").as_string();

        // ── CSM (creates service servers immediately) ─────────────────────────
        csm_ = std::make_unique<ControlSignalManager>(this, csmName_, csmStatusTimerMs);

        // ── One-shot init thread ───────────────────────────────────────────────
        // registerSource() blocks on a service call; it must run AFTER the
        // executor has started spinning AND must NOT block any executor thread
        // (otherwise the response callback can never be dispatched).
        // We use a 500 ms timer to ensure the executor is spinning, then move
        // the blocking work to a dedicated std::thread.
        initTimer_ = create_wall_timer(
            std::chrono::milliseconds(500),
            [this]() {
                initTimer_->cancel();
                initThread_ = std::thread([this]() { _init(); });
            });
    }

    ~KeyboardSourceNode()
    {
        // Restore terminal before ROS shuts down.
        keyboard_.reset();
        // Wait for the init thread to finish (it may be mid-registration).
        if (initThread_.joinable())
            initThread_.join();
    }

private:
    // ── Deferred initialisation ───────────────────────────────────────────────

    void _init()
    {
        const std::string type = isJoy_
            ? CSC::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY
            : CSC::ControlSignalConst::CONTROL_SIGNAL_TYPE_TWIST;

        if (!_registerSource(channel_, type)) {
            RCLCPP_ERROR(get_logger(),
                "[KBD] Failed to register %s source with server '%s'. Aborting.",
                type.c_str(), serverName_.c_str());
            return;
        }

        // Periodic send timer.
        sendTimer_ = create_wall_timer(
            std::chrono::milliseconds(sendRateMs_),
            [this]() { _sendTimerCb(); });

        // Start keyboard reader — must be after other setup.
        keyboard_ = std::make_unique<KeyboardHandler>(
            [this](Key k, KeyEvent e) { _onKey(k, e); },
            keyboardDevice_);

        RCLCPP_INFO(get_logger(),
            "[KBD] Ready. Mode: %s  channel: '%s'  input: %s",
            isJoy_ ? "Joy" : "Twist", channel_.c_str(),
            keyboardDevice_.empty() ? "(auto-detect)" : keyboardDevice_.c_str());
        _printHelp();
    }

    bool _registerSource(const std::string & channel, const std::string & type)
    {
        CSC::ControlSignalInfo info;
        info.target_csm_name        = serverName_;
        info.control_signal_mode    = CSC::ControlSignalConst::CONTROL_SIGNAL_MODE_TOPIC;
        info.control_signal_type    = type;
        info.channel_name           = channel;
        info.send_freq_hz           = sendFreqHz_;
        info.timeout_ns             = timeoutNs_;
        info.disconnect_timeout_ns  = disconnectTimeoutNs_;
        info.use_keep_alive         = false;
        info.keep_alive_interval_ns = 0;
        info.priority               = priority_;

        const bool ok = csm_->registerSource(info, /*timeoutMs=*/5000);
        if (ok)
            RCLCPP_INFO(get_logger(),
                "[KBD] Registered %-6s source: ch='%s' → server='%s'",
                type.c_str(), channel.c_str(), serverName_.c_str());
        else
            RCLCPP_WARN(get_logger(),
                "[KBD] Could not register %s source with server '%s'",
                type.c_str(), serverName_.c_str());
        return ok;
    }

    // ── Periodic send ─────────────────────────────────────────────────────────

    // Helper: ramp `current` toward `target` by `step` (works for any sign).
    static double rampToward(double current, double target, double step)
    {
        if (current < target) return std::min(current + step, target);
        if (current > target) return std::max(current - step, target);
        return current;
    }

    void _sendTimerCb()
    {
        bool cmdOk = false;
        auto base = csm_->getSource(channel_);
        if (!base) return;

        if (isJoy_) {
            auto * src = dynamic_cast<ControlSignalSource<Joy>*>(base.get());
            if (!src) return;

            Joy msg;
            {
                std::lock_guard<std::mutex> lk(stateMtx_);

                // Throttle ramp: each axis ramps toward max when key held, toward 0 when released.
                const auto maxL = static_cast<float>(maxSetpointLinear_);
                const auto maxA = static_cast<float>(maxSetpointAngular_);
                const auto stepL = static_cast<float>(rampStepLinear_);
                const auto stepA = static_cast<float>(rampStepAngular_);

                joyAxes_[0] = static_cast<float>(rampToward(joyAxes_[0], fwdPressed_   ? maxL : 0.0f, stepL));
                joyAxes_[1] = static_cast<float>(rampToward(joyAxes_[1], bckPressed_   ? maxL : 0.0f, stepL));
                joyAxes_[2] = static_cast<float>(rampToward(joyAxes_[2], leftPressed_  ? maxA : 0.0f, stepA));
                joyAxes_[3] = static_cast<float>(rampToward(joyAxes_[3], rightPressed_ ? maxA : 0.0f, stepA));

                msg.axes.assign(joyAxes_.begin(), joyAxes_.end());
                if (joyHeldCmdKey_ != Key::UNKNOWN) {
                    // Command held — keep sending the button state unchanged.
                    msg.buttons.assign(joyButtons_.begin(), joyButtons_.end());
                } else {
                    msg.buttons.assign(JOY_BUTTONS, 0);
                }
            }
            src->send(msg, cmdOk);

        } else {
            auto * src = dynamic_cast<ControlSignalSource<Twist>*>(base.get());
            if (!src) return;

            Twist msg;
            {
                std::lock_guard<std::mutex> lk(stateMtx_);

                // Compute per-axis targets from held movement keys.
                const double targetLinX =
                    (fwdPressed_ ? maxSetpointLinear_ : 0.0) -
                    (bckPressed_ ? maxSetpointLinear_ : 0.0);
                const double targetLinY =
                    (qPressed_           ? maxSetpointLinear_ : 0.0) -
                    (strafeRightPressed_ ? maxSetpointLinear_ : 0.0);
                const double targetAngZ =
                    (leftPressed_  ? maxSetpointAngular_ : 0.0) -
                    (rightPressed_ ? maxSetpointAngular_ : 0.0);

                twistLinX_ = rampToward(twistLinX_, targetLinX, rampStepLinear_);
                twistLinY_ = rampToward(twistLinY_, targetLinY, rampStepLinear_);
                twistAngZ_ = rampToward(twistAngZ_, targetAngZ, rampStepAngular_);

                msg.linear.x  = twistLinX_;
                msg.linear.y  = twistLinY_;
                msg.angular.z = twistAngZ_;
                if (twistHeldCmdKey_ != Key::UNKNOWN) {
                    // Command held — keep sending the sentinel values unchanged.
                    const double v = (twistHeldCmdKey_ == Key::Z) ? -99.0 : 99.0;
                    msg.linear.z = msg.angular.x = msg.angular.y = v;
                }
            }
            src->send(msg, cmdOk);
        }
    }

    // ── Key dispatch ──────────────────────────────────────────────────────────

    void _onKey(Key k, KeyEvent e)
    {
        std::lock_guard<std::mutex> lk(stateMtx_);

        if (k == Key::CTRL_C && e == KeyEvent::PRESS) {
            rclcpp::shutdown();
            return;
        }

        if (isJoy_)
            _handleJoyKey(k, e);
        else
            _handleTwistKey(k, e);
    }

    // ── Joy key handler ───────────────────────────────────────────────────────
    // Called with stateMtx_ held.

    void _handleJoyKey(Key k, KeyEvent e)
    {
        // ── Movement keys: update pressed state (throttle ramp driven by send timer) ──
        switch (k) {
        case Key::W:
        case Key::ARROW_UP:    fwdPressed_   = (e != KeyEvent::RELEASE); return;
        case Key::S:
        case Key::ARROW_DOWN:  bckPressed_   = (e != KeyEvent::RELEASE); return;
        case Key::A:
        case Key::ARROW_LEFT:  leftPressed_  = (e != KeyEvent::RELEASE); return;
        case Key::D:
        case Key::ARROW_RIGHT: rightPressed_ = (e != KeyEvent::RELEASE); return;
        default: break;
        }

        // ── Command keys: hold = keep sending; release = back to axes ──────────
        const bool pressing = (e != KeyEvent::RELEASE);
        switch (k) {
        case Key::K1:
        case Key::K2:
        case Key::K3:
        case Key::K4:
        case Key::K5:
        case Key::K6:
        case Key::K7: {
            if (pressing) {
                const std::size_t btnIdx =
                    (k == Key::K1) ? 0u : (k == Key::K2) ? 1u : (k == Key::K3) ? 2u :
                    (k == Key::K4) ? 6u : (k == Key::K5) ? 7u : (k == Key::K6) ? 8u : 9u;
                if (joyHeldCmdKey_ != k)
                    _setJoyCommand(btnIdx, k);
            } else if (joyHeldCmdKey_ == k) {
                joyHeldCmdKey_ = Key::UNKNOWN;
                joyButtons_.fill(0);
            }
            break;
        }
        case Key::E:
            if (pressing) {
                if (joyHeldCmdKey_ != Key::E) {
                    joyButtons_.fill(0);
                    joyButtons_[0] = joyButtons_[1] = joyButtons_[2] = joyButtons_[3] = -99;
                    joyHeldCmdKey_ = Key::E;
                    fwdPressed_ = bckPressed_ = leftPressed_ = rightPressed_ = false;
                    joyAxes_.fill(0.0f);
                    RCLCPP_WARN(get_logger(), "[KBD] Joy E-STOP held");
                }
            } else if (joyHeldCmdKey_ == Key::E) {
                joyHeldCmdKey_ = Key::UNKNOWN;
                joyButtons_.fill(0);
                RCLCPP_INFO(get_logger(), "[KBD] Joy E-STOP released");
            }
            break;
        case Key::R:
            if (pressing) {
                if (joyHeldCmdKey_ != Key::R) {
                    joyButtons_.fill(0);
                    joyButtons_[0] = joyButtons_[1] = joyButtons_[2] = joyButtons_[3] = 99;
                    joyHeldCmdKey_ = Key::R;
                    RCLCPP_INFO(get_logger(), "[KBD] Joy REQUEST-ACTIVE held");
                }
            } else if (joyHeldCmdKey_ == Key::R) {
                joyHeldCmdKey_ = Key::UNKNOWN;
                joyButtons_.fill(0);
                RCLCPP_INFO(get_logger(), "[KBD] Joy REQUEST-ACTIVE released");
            }
            break;
        case Key::SPACE:
            if (pressing) {
                fwdPressed_ = bckPressed_ = leftPressed_ = rightPressed_ = false;
                joyAxes_.fill(0.0f);
                joyButtons_.fill(0);
                joyHeldCmdKey_ = Key::UNKNOWN;
                RCLCPP_INFO(get_logger(), "[KBD] Joy STOP");
            }
            break;
        default:
            break;
        }
    }

    // Helper: set a single command button and record it as the held command.
    void _setJoyCommand(std::size_t buttonIdx, Key cmdKey)
    {
        joyButtons_.fill(0);
        if (buttonIdx < JOY_BUTTONS)
            joyButtons_[buttonIdx] = 1;
        joyHeldCmdKey_ = cmdKey;
    }

    // ── Twist key handler ─────────────────────────────────────────────────────
    // Called with stateMtx_ held.

    void _handleTwistKey(Key k, KeyEvent e)
    {
        // ── Movement keys: update pressed state (throttle ramp driven by send timer) ──
        switch (k) {
        case Key::W:
        case Key::ARROW_UP:    fwdPressed_         = (e != KeyEvent::RELEASE); return;
        case Key::S:
        case Key::ARROW_DOWN:  bckPressed_         = (e != KeyEvent::RELEASE); return;
        case Key::A:
        case Key::ARROW_LEFT:  leftPressed_        = (e != KeyEvent::RELEASE); return;
        case Key::D:
        case Key::ARROW_RIGHT: rightPressed_       = (e != KeyEvent::RELEASE); return;
        case Key::Q:           qPressed_           = (e != KeyEvent::RELEASE); return;
        case Key::E:           strafeRightPressed_ = (e != KeyEvent::RELEASE); return;
        default: break;
        }

        // ── Command keys: hold = keep sending; release = back to axes ──────────
        const bool pressing = (e != KeyEvent::RELEASE);
        switch (k) {
        case Key::Z:
            if (pressing) {
                if (twistHeldCmdKey_ != Key::Z) {
                    twistHeldCmdKey_ = Key::Z;
                    fwdPressed_ = bckPressed_ = leftPressed_ = rightPressed_ =
                        qPressed_ = strafeRightPressed_ = false;
                    twistLinX_ = twistLinY_ = twistAngZ_ = 0.0;
                    RCLCPP_WARN(get_logger(), "[KBD] Twist E-STOP held");
                }
            } else if (twistHeldCmdKey_ == Key::Z) {
                twistHeldCmdKey_ = Key::UNKNOWN;
                RCLCPP_INFO(get_logger(), "[KBD] Twist E-STOP released");
            }
            break;
        case Key::R:
            if (pressing) {
                if (twistHeldCmdKey_ != Key::R) {
                    twistHeldCmdKey_ = Key::R;
                    RCLCPP_INFO(get_logger(), "[KBD] Twist REQUEST-ACTIVE held");
                }
            } else if (twistHeldCmdKey_ == Key::R) {
                twistHeldCmdKey_ = Key::UNKNOWN;
                RCLCPP_INFO(get_logger(), "[KBD] Twist REQUEST-ACTIVE released");
            }
            break;
        case Key::SPACE:
            if (pressing) {
                fwdPressed_ = bckPressed_ = leftPressed_ = rightPressed_ =
                    qPressed_ = strafeRightPressed_ = false;
                twistLinX_ = twistLinY_ = twistAngZ_ = 0.0;
                twistHeldCmdKey_ = Key::UNKNOWN;
                RCLCPP_INFO(get_logger(), "[KBD] Twist STOP");
            }
            break;
        default:
            break;
        }
    }

    // ── Help text ─────────────────────────────────────────────────────────────

    void _printHelp()
    {
        auto L = get_logger();
        if (isJoy_) {
            RCLCPP_INFO(L, "┌────────────────────────────────────────────────────┐");
            RCLCPP_INFO(L, "│  Keyboard Source [Joy mode]  — Throttle Control    │");
            RCLCPP_INFO(L, "├────────────────────────────────────────────────────┤");
            RCLCPP_INFO(L, "│  Hold key → ramp to max setpoint                   │");
            RCLCPP_INFO(L, "│  Release  → ramp back to zero                      │");
            RCLCPP_INFO(L, "│  Space    Zero all / clear held keys               │");
            RCLCPP_INFO(L, "│  Ctrl+C   Quit                                     │");
            RCLCPP_INFO(L, "├────────────────────────────────────────────────────┤");
            RCLCPP_INFO(L, "│  W/↑  S/↓   forward / backward     (vx)           │");
            RCLCPP_INFO(L, "│  A/←  D/→   turn left / right      (vyaw)         │");
            RCLCPP_INFO(L, "│  1  Damp        2  StandUp      3  StandDown       │");
            RCLCPP_INFO(L, "│  4  StopMove    5  Gait0        6  Gait1           │");
            RCLCPP_INFO(L, "│  7  RecoveryStand                                  │");
            RCLCPP_INFO(L, "│  E  E-stop (btns=-99)       R  Request-active      │");
            RCLCPP_INFO(L, "└────────────────────────────────────────────────────┘");
        } else {
            RCLCPP_INFO(L, "┌────────────────────────────────────────────────────┐");
            RCLCPP_INFO(L, "│  Keyboard Source [Twist mode] — Throttle Control   │");
            RCLCPP_INFO(L, "├────────────────────────────────────────────────────┤");
            RCLCPP_INFO(L, "│  Hold key → ramp to max setpoint                   │");
            RCLCPP_INFO(L, "│  Release  → ramp back to zero                      │");
            RCLCPP_INFO(L, "│  Space    Zero all / clear held keys               │");
            RCLCPP_INFO(L, "│  Ctrl+C   Quit                                     │");
            RCLCPP_INFO(L, "├────────────────────────────────────────────────────┤");
            RCLCPP_INFO(L, "│  W/↑  S/↓   forward / backward  (linear.x)        │");
            RCLCPP_INFO(L, "│  A/←  D/→   turn left / right   (angular.z)       │");
            RCLCPP_INFO(L, "│  Q  strafe left   E  strafe right (linear.y)       │");
            RCLCPP_INFO(L, "│  Z  E-stop (lin.z/ang.x/y=-99)  R  Req-active      │");
            RCLCPP_INFO(L, "└────────────────────────────────────────────────────┘");
        }
    }

    // ── Parameters ────────────────────────────────────────────────────────────

    std::string serverName_;
    std::string csmName_;
    std::string channel_;      // active channel (joy or twist, fixed at startup)
    bool        isJoy_{true};  // fixed at startup from initial_msg_type
    int8_t      priority_{50};
    int64_t     timeoutNs_{2'000'000'000LL};
    int64_t     disconnectTimeoutNs_{10'000'000'000LL};
    float       sendFreqHz_{10.0f};   // derived from send_rate_ms at startup
    double      maxSetpointLinear_{1.0};
    double      maxSetpointAngular_{1.0};
    double      rampStepLinear_{0.05};
    double      rampStepAngular_{0.05};
    int64_t     sendRateMs_{100};
    std::string keyboardDevice_;  // "" = auto-detect; "stdin"/"tty" = force stdin; evdev path = force evdev

    // ── State (all protected by stateMtx_) ────────────────────────────────────

    std::mutex stateMtx_;

    // Throttle: which movement keys are currently held.
    bool fwdPressed_{false};          // W / ARROW_UP
    bool bckPressed_{false};          // S / ARROW_DOWN
    bool leftPressed_{false};         // A / ARROW_LEFT
    bool rightPressed_{false};        // D / ARROW_RIGHT
    bool qPressed_{false};            // Q (Twist strafe-left)
    bool strafeRightPressed_{false};  // E (Twist strafe-right)

    // Joy: axes (ramped), command button state, held command key.
    std::array<float,   JOY_AXES>    joyAxes_{};
    std::array<int32_t, JOY_BUTTONS> joyButtons_{};
    Key                              joyHeldCmdKey_{Key::UNKNOWN};  // != UNKNOWN while cmd held

    // Twist: axes (ramped), held command key.
    double twistLinX_{0}, twistLinY_{0}, twistAngZ_{0};
    Key    twistHeldCmdKey_{Key::UNKNOWN};  // Key::Z = e-stop, Key::R = req-active

    // ── ROS 2 infrastructure ──────────────────────────────────────────────────

    std::unique_ptr<ControlSignalManager> csm_;
    rclcpp::TimerBase::SharedPtr          initTimer_;
    rclcpp::TimerBase::SharedPtr          sendTimer_;
    std::unique_ptr<KeyboardHandler>      keyboard_;
    std::thread                           initThread_;
};

RCLCPP_COMPONENTS_REGISTER_NODE(KeyboardSourceNode)
