/**
 * control_signal_factory.h
 *
 * Compile-time registration and runtime factory for ControlSignalSource /
 * ControlSignalSink.
 *
 * Design summary
 * ──────────────
 *  ControlSignalFactory             — singleton registry + runtime CreateSource/CreateSink
 *  REGISTER_CONTROL_SIGNAL(...)     — macro that inserts a static auto-registrar
 *
 * To add a new control-signal type:
 *  1. In one .cpp file, include the message and (optional) service headers.
 *  2. Call REGISTER_CONTROL_SIGNAL(UniqueId, "name", MsgType, SrvType).
 *  No factory code changes are required anywhere else.
 */

#pragma once

#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <typeindex>
#include <typeinfo>
#include <unordered_map>

#include "rv2_control_signal_transport/control_signal_transport.h"

namespace rv2_interfaces
{

// ============================================================
//  ControlSignalFactory
// ============================================================

/**
 * @brief Singleton factory for creating ControlSignalSource and ControlSignalSink
 *        instances by runtime type-string.
 *
 * Lookup path:
 * @code
 *   string control_type
 *       ↓
 *   Factory lookup
 *       ↓
 *   (MsgT, SrvT)
 *       ↓
 *   ControlSignalSource<MsgT, SrvT>  or  ControlSignalSink<MsgT, SrvT>
 * @endcode
 *
 * Types are registered at program startup via REGISTER_CONTROL_SIGNAL().
 * CreateSource() / CreateSink() throw std::runtime_error for unknown names.
 */
class ControlSignalFactory
{
public:
    using SourceCreator = std::function<
        std::unique_ptr<BaseControlSignalSource>(
            rclcpp::Node*,
            const msg::ControlSignalInfo&)>;

    using SinkCreator = std::function<
        std::unique_ptr<BaseControlSignalSink>(
            rclcpp::Node*,
            const msg::ControlSignalInfo&)>;

private:
    struct Entry
    {
        SourceCreator sourceCreator;
        SinkCreator   sinkCreator;
    };

    std::unordered_map<std::string, Entry>           registry_;
    std::unordered_map<std::type_index, std::string> reverseRegistry_;

    ControlSignalFactory() = default;

public:
    /// Returns the process-wide singleton instance.
    /// Defined in control_signal_factory.cpp (shared library) so all consumers
    /// share one registry regardless of how many translation units include this header.
    static ControlSignalFactory& Instance();

    ControlSignalFactory(const ControlSignalFactory&)            = delete;
    ControlSignalFactory& operator=(const ControlSignalFactory&) = delete;

    /**
     * @brief Register ControlSignalSource<MsgT, SrvT> and ControlSignalSink<MsgT, SrvT>
     *        under @p name.
     *
     * Called automatically by REGISTER_CONTROL_SIGNAL() during static initialization.
     * Re-registration under the same name silently overwrites the previous entry.
     *
     * @tparam MsgT  ROS 2 message type.
     * @tparam SrvT  ROS 2 service type, or void for topic-only.
     * @param  name  Control-type string (e.g. "joy").
     */
    template<typename MsgT, typename SrvT = void>
    void Register(const std::string& name)
    {
        registry_[name] = Entry{
            [](rclcpp::Node* node, const msg::ControlSignalInfo& info)
                -> std::unique_ptr<BaseControlSignalSource>
            {
                return std::make_unique<ControlSignalSource<MsgT, SrvT>>(node, info);
            },
            [](rclcpp::Node* node, const msg::ControlSignalInfo& info)
                -> std::unique_ptr<BaseControlSignalSink>
            {
                return std::make_unique<ControlSignalSink<MsgT, SrvT>>(node, info);
            }
        };
        reverseRegistry_[std::type_index(typeid(MsgT))] = name;
    }

    /**
     * @brief Create a ControlSignalSource for the registered @p name.
     *
     * @throws std::runtime_error if @p name was not registered.
     */
    std::unique_ptr<BaseControlSignalSource>
    CreateSource(
        const std::string& name,
        rclcpp::Node* node,
        const msg::ControlSignalInfo& info)
    {
        auto it = registry_.find(name);
        if (it == registry_.end())
            throw std::runtime_error(
                "ControlSignalFactory: unknown control type '" + name + "'");
        return it->second.sourceCreator(node, info);
    }

    /**
     * @brief Create a ControlSignalSink for the registered @p name.
     *
     * @throws std::runtime_error if @p name was not registered.
     */
    std::unique_ptr<BaseControlSignalSink>
    CreateSink(
        const std::string& name,
        rclcpp::Node* node,
        const msg::ControlSignalInfo& info)
    {
        auto it = registry_.find(name);
        if (it == registry_.end())
            throw std::runtime_error(
                "ControlSignalFactory: unknown control type '" + name + "'");
        return it->second.sinkCreator(node, info);
    }

    /**
     * @brief Reverse lookup: returns the registered type-string for @p msgTypeIndex,
     *        or an empty string if the type has not been registered.
     */
    std::string typeKey(std::type_index msgTypeIndex) const
    {
        auto it = reverseRegistry_.find(msgTypeIndex);
        return (it != reverseRegistry_.end()) ? it->second : std::string{};
    }
};


// ============================================================
//  REGISTER_CONTROL_SIGNAL macro
// ============================================================

/**
 * @brief Register a control-signal type with the ControlSignalFactory.
 *
 * Place this macro in exactly one translation unit (.cpp) after including
 * the required message and service headers.
 *
 * Parameters:
 *   UniqueId  — a bare identifier unique within the translation unit (used
 *               to name the internal static object; no quotes, no colons).
 *   name_str  — the runtime type string (e.g. "joy").
 *   MsgType   — the ROS 2 message type (e.g. sensor_msgs::msg::Joy).
 *   SrvType   — the ROS 2 service type, or `void` for topic-only types.
 *
 * Usage:
 *   @code
 *   // Joy — supports both topic and service transport:
 *   REGISTER_CONTROL_SIGNAL(
 *       Joy,
 *       "joy",
 *       sensor_msgs::msg::Joy,
 *       rv2_interfaces::srv::ControlSignalJoy);
 *
 *   // Twist — supports both topic and service transport:
 *   REGISTER_CONTROL_SIGNAL(
 *       Twist,
 *       "twist",
 *       geometry_msgs::msg::Twist,
 *       rv2_interfaces::srv::ControlSignalTwist);
 *
 *   // String — topic-only (no service binding); pass void for SrvType:
 *   REGISTER_CONTROL_SIGNAL(
 *       String,
 *       "string",
 *       std_msgs::msg::String,
 *       void);
 *   @endcode
 *
 * Internally creates a static object whose constructor calls
 * ControlSignalFactory::Instance().Register<MsgType, SrvType>(name_str)
 * at program startup — no explicit call is required.
 */
#define REGISTER_CONTROL_SIGNAL(UniqueId, name_str, MsgType, SrvType)       \
    namespace {                                                              \
    struct _ControlSignalAutoRegister_##UniqueId {                          \
        _ControlSignalAutoRegister_##UniqueId() {                           \
            ::rv2_interfaces::ControlSignalFactory::Instance()              \
                .Register<MsgType, SrvType>(name_str);                      \
        }                                                                    \
    };                                                                       \
    static _ControlSignalAutoRegister_##UniqueId                            \
        _rv2_cs_instance_##UniqueId; /* NOLINT(cert-err58-cpp) */           \
    } // anonymous namespace


} // namespace rv2_interfaces
