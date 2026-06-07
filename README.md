# rv2_control_signal_transport

ROS 2 package that provides the **ControlSignalManager / ControlSignalSource / ControlSignalSink** transport layer and a ready-to-use **KeyboardSourceNode** composable node.

---

## Package contents

| Path | Description |
|---|---|
| `include/rv2_control_signal_transport/control_signal_transport.h` | Core transport primitives: `ControlSignalSource`, `ControlSignalSink`, base classes, factories |
| `include/rv2_control_signal_transport/control_signal_manager.h` | `ControlSignalManager` — multi-source/sink registry with auto-disconnect |
| `include/rv2_control_signal_transport/keyboard_handler.h` | Linux evdev keyboard reader (`KeyboardHandler`) |
| `src/keyboard_source_node.cpp` | `KeyboardSourceNode` composable node |
| `config/keyboard_source.yaml` | Default parameter file (generic) |
| `config/keyboard_source_joy.yaml` | Default parameter file for Joy mode |
| `config/keyboard_source_twist.yaml` | Default parameter file for Twist mode |
| `launch/keyboard_source.launch.py` | Launch with generic config |
| `launch/keyboard_source_joy.launch.py` | Launch in Joy mode |
| `launch/keyboard_source_twist.launch.py` | Launch in Twist mode |

---

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Source node (e.g. KeyboardSourceNode)                      │
│                                                             │
│  ControlSignalManager (Source CSM)                          │
│  └── ControlSignalSource<Joy|Twist>  ──── topic pub ───►    │
└────────────────────────────────────────────────────────────┘
                                │  /channel_name (topic)
                                ▼
┌────────────────────────────────────────────────────────────┐
│  Server node (e.g. ControlServerNode)                      │
│                                                            │
│  ControlSignalManager (Server CSM)                         │
│  └── ControlSignalSink<Joy|Twist>  ◄──── topic sub         │
│                                                            │
│  ControlServer                                             │
│  └── active-sink selection + output callbacks              │
└────────────────────────────────────────────────────────────┘
```

Registration flow:
1. Source CSM calls `<server_name>/control_signal_reg` (service).
2. Server CSM creates a matching `ControlSignalSink` and starts monitoring it.
3. Source CSM creates the local `ControlSignalSource` and begins publishing.

---

## Transport primitives

### `ControlSignalSource<msgT, srvT>`

Wraps a ROS 2 **publisher** (topic mode, default) or **service client** (service mode).

- `send(msg, cmdSuccess)` — publishes or calls the service.
- `getState()` — passive timeout check; returns `UNKNOWN / ACTIVE / LOW_FREQ / TIMEOUT / DISCONNECTED`.
- Topic-mode sources have no send-side feedback; state stays `UNKNOWN` unless keep-alive is enabled.

### `ControlSignalSink<msgT, srvT>`

Wraps a ROS 2 **subscription** (topic mode, default) or **service server** (service mode).

- `getState()` — passive timeout check against `timeout_ns` from construction info.
- `setMsgCallback(cb)` — optional callback fired on every received message.
- State transitions: `UNKNOWN → ACTIVE → LOW_FREQ → TIMEOUT → DISCONNECTED`.
  - `UNKNOWN` transitions to `TIMEOUT` after `timeout_ns` even if no message was ever received (ensures auto-disconnect works for idle sources).

### `ControlSignalManager`

Owns maps of Sources and Sinks, keyed by `channel_name`.

- `registerSource(info, timeoutMs)` — validates config, calls the target CSM's `control_signal_reg` service, then stores a local Source. Blocking; must be called after the executor is spinning.
- `setSinkMsgCallback<msgT>(cb)` — type-safe callback applied to all current and future Sinks of type `msgT`.
- **Status timer** (period = `statusTimerIntervalMs`) — walks Sources and Sinks; removes any entry that has been continuously in `TIMEOUT` for longer than its `disconnect_timeout_ns`.

---

## `ControlSignalInfo` fields

| Field | Type | Description |
|---|---|---|
| `target_csm_name` | string | Name of the target ControlSignalManager (server) |
| `control_signal_mode` | string | `"topic"` or `"service"` |
| `control_signal_type` | string | `"joy"`, `"twist"`, or `"string"` |
| `channel_name` | string | Topic/service name used for the transport |
| `send_freq_hz` | float | Expected publish frequency (0 = unchecked) |
| `timeout_ns` | int64 | Nanoseconds before TIMEOUT (0 = disabled) |
| `disconnect_timeout_ns` | int64 | ns in TIMEOUT before auto-remove (0 = never) |
| `priority` | int8 | Source priority (1–100; higher = preferred) |
| `use_keep_alive` | bool | Enable keep-alive heartbeat |
| `keep_alive_interval_ns` | int64 | Heartbeat interval (ns) |

Validation rules (checked by `validateControlSignalInfo()`):
- `timeout_ns` > 0 required when `disconnect_timeout_ns` > 0.
- `disconnect_timeout_ns` > `timeout_ns` (cannot disconnect before timing out).
- `1 / send_freq_hz` < `timeout_ns` (send period must be shorter than timeout).

---

## KeyboardSourceNode

Composable node (`plugin: KeyboardSourceNode`) that reads a Linux evdev keyboard device and publishes control signals.

### Parameters

| Parameter | Type | Default | Description |
|---|---|---|---|
| `server_name` | string | `"control_server"` | Target CSM name |
| `csm_name` | string | `"keyboard_source"` | This node's CSM name |
| `channel_name` | string | `"keyboard_control"` | Source channel name |
| `priority` | int64 | `50` | Source priority (1–100) |
| `timeout_ms` | int64 | `2000` | Inactivity timeout (ms) |
| `disconnect_timeout_ms` | int64 | `10000` | ms in TIMEOUT before CSM removes source (0 = never) |
| `initial_msg_type` | string | `"joy"` | `"joy"` or `"twist"` (fixed at startup) |
| `max_setpoint_linear` | double | `1.0` | Maximum linear setpoint |
| `max_setpoint_angular` | double | `1.0` | Maximum angular setpoint |
| `ramp_step_linear` | double | `0.05` | Linear setpoint change per send tick |
| `ramp_step_angular` | double | `0.05` | Angular setpoint change per send tick |
| `send_rate_ms` | int64 | `100` | Publish interval (ms) |
| `keyboard_device` | string | `""` | evdev path (e.g. `/dev/input/event3`); `""` = auto-detect; `"stdin"` = noVNC terminal |
| `csm_status_timer_interval_ms` | int64 | `1000` | CSM status/disconnect timer period (ms) |

### Key bindings — Joy mode

| Key | Action | Signal |
|---|---|---|
| W / ↑ | Forward | `axes[0]` → `+max_setpoint_linear` |
| S / ↓ | Backward | `axes[1]` → `+max_setpoint_linear` |
| A / ← | Turn left | `axes[2]` → `+max_setpoint_angular` |
| D / → | Turn right | `axes[3]` → `+max_setpoint_angular` |
| 1 | Damp | `buttons[0]` |
| 2 | StandUp | `buttons[1]` |
| 3 | StandDown | `buttons[2]` |
| 4 | StopMove | `buttons[6]` |
| 5 | SwitchGait 0 | `buttons[7]` |
| 6 | SwitchGait 1 | `buttons[8]` |
| 7 | RecoveryStand | `buttons[9]` |
| E | E-stop | `buttons[0..3]` = −99 |
| R | Request active | `buttons[0..3]` = 99 |
| SPACE | Zero all | axes and buttons cleared |

### Key bindings — Twist mode

| Key | Action | Signal |
|---|---|---|
| W / ↑ | Forward | `linear.x` → `+max_setpoint_linear` |
| S / ↓ | Backward | `linear.x` → `−max_setpoint_linear` |
| A / ← | Turn left | `angular.z` → `+max_setpoint_angular` |
| D / → | Turn right | `angular.z` → `−max_setpoint_angular` |
| Q | Strafe left | `linear.y` → `+max_setpoint_linear` |
| E | Strafe right | `linear.y` → `−max_setpoint_linear` |
| Z | E-stop | `linear.z = angular.x = angular.y` = −99 |
| R | Request active | = 99 |
| SPACE | Zero all | all fields cleared |

Movement axes ramp toward the setpoint while a key is held and ramp back to zero on release. Command keys (1–7, E, R, Z/SPACE) fire as one-shot events on PRESS only.

### Keyboard device setup

The node reads directly from `/dev/input/eventX` (Linux evdev).  
Add your user to the `input` group so no `sudo` is needed:

```bash
sudo usermod -aG input $USER
# log out and back in, then verify:
groups | grep input
```

Leave `keyboard_device` empty to auto-detect the first keyboard, or set it explicitly:

```yaml
keyboard_device: "/dev/input/event3"
```

Set `keyboard_device: "stdin"` for noVNC / terminal environments that cannot open evdev directly.

---

## Usage

### Quickstart

```bash
# Joy mode (default config)
ros2 launch rv2_control_signal_transport keyboard_source_joy.launch.py

# Twist mode
ros2 launch rv2_control_signal_transport keyboard_source_twist.launch.py

# Custom config
ros2 launch rv2_control_signal_transport keyboard_source.launch.py \
    config_file:=/path/to/my_keyboard_source.yaml
```

### Using as a library (header-only)

```cpp
#include "rv2_control_signal_transport/control_signal_manager.h"

// Source side
auto csm = std::make_unique<rv2_interfaces::ControlSignalManager>(node, "my_csm", 1000);

rv2_interfaces::msg::ControlSignalInfo info;
info.target_csm_name       = "control_server";
info.control_signal_mode   = rv2_interfaces::msg::ControlSignalConst::CONTROL_SIGNAL_MODE_TOPIC;
info.control_signal_type   = rv2_interfaces::msg::ControlSignalConst::CONTROL_SIGNAL_TYPE_JOY;
info.channel_name          = "my_channel";
info.send_freq_hz          = 20.0f;
info.timeout_ns            = 2'000'000'000LL;
info.disconnect_timeout_ns = 10'000'000'000LL;
info.priority              = 50;

csm->registerSource(info, 5000);  // blocks; node must be spinning

// Send
auto base = csm->getSource("my_channel");
auto* src = dynamic_cast<rv2_interfaces::ControlSignalSource<sensor_msgs::msg::Joy>*>(base.get());
bool ok;
src->send(joy_msg, ok);
```

### Building

```bash
cd ~/ros2_ws
colcon build --packages-select rv2_control_signal_transport
source install/setup.bash
```

---

## Dependencies

- `rclcpp`, `rclcpp_components`
- `sensor_msgs`, `geometry_msgs`, `std_msgs`
- `rv2_interfaces`
