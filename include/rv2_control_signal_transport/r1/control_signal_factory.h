/**
 * @file control_signal_factory.h
 * @brief r1::ControlSignalFactory — runtime type-string to compile-time
 *        (MsgT, SrvT) registry (design draft §7), plus the
 *        R1_REGISTER_CONTROL_SIGNAL registration macro.
 *
 * Differences from the proven rv2 registry (§7.1): creators return
 * shared_ptr (enable_shared_from_this needs shared ownership), duplicate
 * Register() logs and refuses instead of silently overwriting, and
 * CreateSource/CreateSink never throw — they return nullptr with the reason
 * in the errOut parameter.
 */

#ifndef RV2_CONTROL_SIGNAL_TRANSPORT_R1_CONTROL_SIGNAL_FACTORY_H
#define RV2_CONTROL_SIGNAL_TRANSPORT_R1_CONTROL_SIGNAL_FACTORY_H

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <typeindex>

#include <rclcpp/rclcpp.hpp>

#include "rv2_control_signal_transport/r1/control_signal_info.h"
#include "rv2_control_signal_transport/r1/control_signal_sink.h"
#include "rv2_control_signal_transport/r1/control_signal_source.h"

namespace rv2_interfaces
{
namespace r1
{

class ControlSignalFactory
{
public:
    /// Defined in control_signal_factory.cpp: the singleton lives once in
    /// the shared library so registrations are visible across every TU and
    /// consumer in the process (§7.2, F5).
    static ControlSignalFactory& Instance();

    /// Registers name -> (MsgT, SrvT). SrvT = void marks a topic-only type
    /// (its service-mode creation is refused with an error string).
    /// @return false = the name already exists (logged and refused, §7.1).
    template <typename MsgT, typename SrvT = void> bool Register(const std::string& name)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        if (entries_.count(name))
        {
            RCLCPP_ERROR(rclcpp::get_logger("r1.control_signal_factory"),
                         "duplicate control-signal type registration refused: '%s'",
                         name.c_str());
            return false;
        }
        Entry e;
        e.msgType = std::type_index(typeid(MsgT));
        e.serviceCapable = !std::is_void_v<SrvT>;
        e.sourceCreator =
            [](rclcpp::Node* node, const InfoT& info, int64_t rateWindowNs) -> std::shared_ptr<BaseControlSignalSource>
        {
            return std::shared_ptr<ControlSignalSource<MsgT, SrvT>>(
                new ControlSignalSource<MsgT, SrvT>(node, info, rateWindowNs));
        };
        e.sinkCreator =
            [](rclcpp::Node* node, const InfoT& info, int64_t rateWindowNs) -> std::shared_ptr<BaseControlSignalSink>
        {
            auto sink = std::shared_ptr<ControlSignalSink<MsgT, SrvT>>(
                new ControlSignalSink<MsgT, SrvT>(node, info, rateWindowNs));
            sink->_bindTransport();
            return sink;
        };
        entries_.emplace(name, std::move(e));
        return true;
    }

    /// @param rateWindowNs RateRecorder window, normally
    ///        ManagerOptions.rateWindowNs (extension over the §7.2 signature:
    ///        the option lives in the Manager, defaulted for direct use).
    std::shared_ptr<BaseControlSignalSource> CreateSource(const std::string& type,
                                                          rclcpp::Node* node,
                                                          const InfoT& info,
                                                          std::string* errOut = nullptr,
                                                          int64_t rateWindowNs = 1'000'000'000) noexcept
    {
        return _create<BaseControlSignalSource>(type, node, info, errOut, rateWindowNs, /*source=*/true);
    }

    std::shared_ptr<BaseControlSignalSink> CreateSink(const std::string& type,
                                                      rclcpp::Node* node,
                                                      const InfoT& info,
                                                      std::string* errOut = nullptr,
                                                      int64_t rateWindowNs = 1'000'000'000) noexcept
    {
        return _create<BaseControlSignalSink>(type, node, info, errOut, rateWindowNs, /*source=*/false);
    }

    /// Reverse lookup of the registered type key; unregistered -> "".
    std::string typeKey(std::type_index msgType) const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        for (const auto& [name, e] : entries_)
            if (e.msgType == msgType)
                return name;
        return "";
    }

    bool has(const std::string& type) const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return entries_.count(type) != 0;
    }

    /// True when the type exists and supports service mode (SrvT != void).
    bool serviceCapable(const std::string& type) const
    {
        std::lock_guard<std::mutex> lk(mtx_);
        const auto it = entries_.find(type);
        return it != entries_.end() && it->second.serviceCapable;
    }

private:
    ControlSignalFactory() = default;

    struct Entry
    {
        std::type_index msgType{typeid(void)};
        bool serviceCapable{false};
        std::function<std::shared_ptr<BaseControlSignalSource>(rclcpp::Node*, const InfoT&, int64_t)> sourceCreator;
        std::function<std::shared_ptr<BaseControlSignalSink>(rclcpp::Node*, const InfoT&, int64_t)> sinkCreator;
    };

    template <typename BaseT>
    std::shared_ptr<BaseT> _create(const std::string& type,
                                   rclcpp::Node* node,
                                   const InfoT& info,
                                   std::string* errOut,
                                   int64_t rateWindowNs,
                                   bool source) noexcept
    {
        try
        {
            Entry entry;
            {
                std::lock_guard<std::mutex> lk(mtx_);
                const auto it = entries_.find(type);
                if (it == entries_.end())
                {
                    if (errOut)
                        *errOut = "unregistered control-signal type '" + type + "'";
                    return nullptr;
                }
                entry = it->second;
            }
            if (info.mode == ControlSignalInfo::MODE_SERVICE && !entry.serviceCapable)
            {
                if (errOut)
                    *errOut = "type '" + type +
                              "' is topic-only (no service type "
                              "registered, §7.2)";
                return nullptr;
            }
            if constexpr (std::is_same_v<BaseT, BaseControlSignalSource>)
                return entry.sourceCreator(node, info, rateWindowNs);
            else
                return entry.sinkCreator(node, info, rateWindowNs);
        }
        catch (const std::exception& ex)
        {
            if (errOut)
                *errOut = std::string("creation failed: ") + ex.what();
            return nullptr;
        }
        catch (...)
        {
            if (errOut)
                *errOut = "creation failed: unknown exception";
            return nullptr;
        }
    }

    mutable std::mutex mtx_;
    std::map<std::string, Entry> entries_;
};

/// Header-safe registration macro (§7.2): expands to a TU-local static whose
/// initializer performs the registration; the anonymous namespace prevents
/// ODR clashes when used from headers.
#define R1_REGISTER_CONTROL_SIGNAL(UniqueId, name_str, MsgType, SrvType)                                               \
    namespace                                                                                                          \
    {                                                                                                                  \
    const bool r1_registered_##UniqueId =                                                                              \
        ::rv2_interfaces::r1::ControlSignalFactory::Instance().Register<MsgType, SrvType>(name_str);                   \
    }

}  // namespace r1
}  // namespace rv2_interfaces

#endif  // RV2_CONTROL_SIGNAL_TRANSPORT_R1_CONTROL_SIGNAL_FACTORY_H
