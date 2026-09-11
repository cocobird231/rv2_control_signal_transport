/**
 * @file control_signal_handles.h
 * @brief r1::SourceHandle / r1::SinkHandle — the user's only holdables
 *        (design draft §10). Copyable lightweight values; weak binding, so a
 *        Handle never extends slot / endpoint lifetime and expires the
 *        moment the Manager removes them (no zombies, §0.1).
 */

#ifndef RV2_CONTROL_SIGNAL_TRANSPORT_R1_CONTROL_SIGNAL_HANDLES_H
#define RV2_CONTROL_SIGNAL_TRANSPORT_R1_CONTROL_SIGNAL_HANDLES_H

#include <cassert>
#include <memory>
#include <optional>
#include <string>

#include "rv2_control_signal_transport/r1/control_signal_info.h"
#include "rv2_control_signal_transport/r1/control_signal_sink.h"
#include "rv2_control_signal_transport/r1/source_registration.h"

namespace rv2_interfaces
{
namespace r1
{

class SourceHandle
{
public:
    SourceHandle() = default;  // empty handle: every operation is inert

    /// True while the logical intent is still owned by the Manager: the weak
    /// slot locks AND desired is still set (§10.3). A slot momentarily kept
    /// alive by a tick snapshot cannot cause a false positive because
    /// unregister clears desired first.
    bool valid() const
    {
        const auto slot = slot_.lock();
        if (!slot)
            return false;
        std::shared_lock<std::shared_mutex> lk(slot->slotMtx);
        return slot->desired;
    }

    /// valid() plus a REGISTERED endpoint currently present.
    bool ready() const
    {
        const auto slot = slot_.lock();
        if (!slot)
            return false;
        std::shared_lock<std::shared_mutex> lk(slot->slotMtx);
        return slot->desired && slot->phase == RegistrationPhase::REGISTERED && slot->endpoint != nullptr;
    }

    const std::string& controllerName() const { return controllerName_; }

    SendResult send(const void* msg) = delete;  // type-safe version only:

    /// Forwards to the current endpoint. PENDING / RETRY_WAIT (intent alive,
    /// endpoint absent) -> RETRYING; slot gone or undesired -> DISCONNECTED.
    /// msgType mismatch: assert in debug, SendResult::NO_TRANSPORT in
    /// release (§10.2, H3).
    template <typename msgT> SendResult send(const msgT& msg)
    {
        const auto slot = slot_.lock();
        if (!slot)
            return SendResult::DISCONNECTED;
        std::shared_ptr<BaseControlSignalSource> ep;
        {
            std::shared_lock<std::shared_mutex> lk(slot->slotMtx);
            if (!slot->desired)
                return SendResult::DISCONNECTED;
            if (!slot->endpoint || slot->phase != RegistrationPhase::REGISTERED)
                return SendResult::RETRYING;
            ep = slot->endpoint;
        }
        if (ep->msgType() != std::type_index(typeid(msgT)))
        {
            assert(false && "SourceHandle::send<msgT>: message type mismatch");
            return SendResult::NO_TRANSPORT;
        }
        return ep->sendErased(&msg);
    }

    /// Local liveness of the current endpoint; no endpoint (incl. RETRY_WAIT
    /// or an expired handle) -> nullopt — deliberately NOT DISCONNECTED,
    /// which is a terminal transition, not "no endpoint" (§10.3).
    std::optional<ControlSignalState> state() const
    {
        const auto slot = slot_.lock();
        if (!slot)
            return std::nullopt;
        std::shared_ptr<BaseControlSignalSource> ep;
        {
            std::shared_lock<std::shared_mutex> lk(slot->slotMtx);
            if (!slot->desired || !slot->endpoint || slot->phase != RegistrationPhase::REGISTERED)
                return std::nullopt;
            ep = slot->endpoint;
        }
        return ep->getState();
    }

    /// Readable even while PENDING / RETRY_WAIT (the intent's descriptor).
    std::optional<ControlSignalInfo> info() const
    {
        const auto slot = slot_.lock();
        if (!slot)
            return std::nullopt;
        std::shared_lock<std::shared_mutex> lk(slot->slotMtx);
        if (!slot->desired)
            return std::nullopt;
        return slot->info;
    }

private:
    friend class ControlSignalManager;
    SourceHandle(std::weak_ptr<SourceRegistrationSlot> slot, std::string controller) :
        slot_(std::move(slot)),
        controllerName_(std::move(controller))
    {
    }

    std::weak_ptr<SourceRegistrationSlot> slot_;  // type at namespace scope (§8.2)
    std::string controllerName_;
};

class SinkHandle
{
public:
    SinkHandle() = default;

    bool valid() const { return !endpoint_.expired(); }

    template <typename msgT> bool read(msgT& out) const
    {
        const auto ep = endpoint_.lock();
        if (!ep)
            return false;  // expired: inert semantics (§10.3)
        if (ep->msgType() != std::type_index(typeid(msgT)))
        {
            assert(false && "SinkHandle::read<msgT>: message type mismatch");
            return false;
        }
        return ep->readErased(&out);
    }

    /// Forwards Sink::waitForMessage; expired handle -> immediate false.
    /// Blocking: never call inside a ROS callback (§2.6).
    template <typename msgT> bool waitForMessage(msgT& out, int64_t timeoutNs = 0) const
    {
        const auto ep = endpoint_.lock();
        if (!ep)
            return false;
        if (ep->msgType() != std::type_index(typeid(msgT)))
        {
            assert(false && "SinkHandle::waitForMessage<msgT>: message type mismatch");
            return false;
        }
        return ep->_waitForMessageErased(&out, timeoutNs);
    }

    std::optional<ControlSignalState> state() const
    {
        const auto ep = endpoint_.lock();
        if (!ep)
            return std::nullopt;
        return ep->getState();
    }

    std::optional<ControlSignalInfo> info() const
    {
        const auto ep = endpoint_.lock();
        if (!ep)
            return std::nullopt;
        return ep->getInfo();
    }

    // Callback registration goes through Manager::registerCallback (type
    // level); the Handle offers none to avoid lifetime coupling (§10.2).

private:
    friend class ControlSignalManager;
    explicit SinkHandle(std::weak_ptr<BaseControlSignalSink> ep) :
        endpoint_(std::move(ep))
    {
    }

    std::weak_ptr<BaseControlSignalSink> endpoint_;
};

}  // namespace r1
}  // namespace rv2_interfaces

#endif  // RV2_CONTROL_SIGNAL_TRANSPORT_R1_CONTROL_SIGNAL_HANDLES_H
