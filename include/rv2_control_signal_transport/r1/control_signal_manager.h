/**
 * @file control_signal_manager.h
 * @brief r1::ControlSignalManager — the CSM itself (design draft §8).
 *
 * Single status timer does all periodic work (D8, §2.6): snapshot/calc,
 * non-terminal commit, terminal seal + same-tick deregistration, status
 * publish + master heartbeat, retry/maintenance. The tick is the only state
 * writer; management services and master clients live in a Reentrant group,
 * the tick in its own MutuallyExclusive group.
 */

#ifndef RV2_CONTROL_SIGNAL_TRANSPORT_R1_CONTROL_SIGNAL_MANAGER_H
#define RV2_CONTROL_SIGNAL_TRANSPORT_R1_CONTROL_SIGNAL_MANAGER_H

#include <algorithm>
#include <atomic>
#include <cassert>
#include <deque>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <shared_mutex>
#include <sstream>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <r1_interfaces/msg/entry_status.hpp>
#include <r1_interfaces/msg/manager_status.hpp>
#include <r1_interfaces/srv/control_signal_info_req.hpp>
#include <r1_interfaces/srv/control_signal_manage.hpp>
#include <r1_interfaces/srv/csm_heartbeat.hpp>
#include <r1_interfaces/srv/csm_notify.hpp>
#include <r1_interfaces/srv/csm_register.hpp>

#include "rv2_control_signal_transport/r1/control_signal_factory.h"
#include "rv2_control_signal_transport/r1/control_signal_handles.h"
#include "rv2_control_signal_transport/r1/control_signal_info.h"
#include "rv2_control_signal_transport/r1/control_signal_sink.h"
#include "rv2_control_signal_transport/r1/control_signal_source.h"
#include "rv2_control_signal_transport/r1/source_registration.h"

namespace rv2_interfaces
{
namespace r1
{

namespace detail
{
/// Thread-local marker set inside every Manager-owned ROS callback; the
/// synchronous public APIs check it as their precondition (§2.6/§8.3, M14).
inline thread_local bool tlInManagerCallback = false;

struct CallbackGuard
{
    CallbackGuard() { tlInManagerCallback = true; }
    ~CallbackGuard() { tlInManagerCallback = false; }
};

inline std::string makeUuid()
{
    static thread_local std::mt19937_64 rng{std::random_device{}()};
    std::ostringstream os;
    os << std::hex << rng() << rng();
    return os.str();
}
} // namespace detail

struct RetryPolicy
{
    /// v1.1.0: no implicit defaults — deployments give explicit values until
    /// D7 freezes them. Recommended() carries the D7 proposal (§12 #6).
    RetryPolicy(int64_t initialDelayMs_, int64_t maxDelayMs_, double jitterRatio_,
                uint32_t maxInitialAttempts_, uint32_t maxInFlight_)
        : initialDelayMs(initialDelayMs_)
        , maxDelayMs(maxDelayMs_)
        , jitterRatio(jitterRatio_)
        , maxInitialAttempts(maxInitialAttempts_)
        , maxInFlight(maxInFlight_)
    {}

    /// D7 proposal (recorded in r1_todo.md; to be frozen back into the
    /// design doc): 200ms initial, 5s cap, 20% jitter, 3 initial attempts,
    /// 4 concurrent attempts, quarantine after 3 never-active rebuilds.
    static RetryPolicy Recommended()
    {
        RetryPolicy p(200, 5000, 0.2, 3, 4);
        return p;
    }

    int64_t initialDelayMs;       // D7 pending
    int64_t maxDelayMs;           // D7 pending; exponential backoff cap
    double  jitterRatio;          // D7 pending; avoids multi-CSM retry storms
    uint32_t maxInitialAttempts;  // caps only the never-yet-successful initial retry
    uint32_t maxInFlight;         // bounded async attempts per tick
    /// D7 flag ownership proposal: initial-failure auto-retry lives on the
    /// policy (global), not per-Info.
    bool autoRetryInitial{true};
    /// v1.2.1 oscillation damping: consecutive never-ACTIVE rebuilds before
    /// quarantine backoff + SUSPECTED_DATA_PATH_FAULT event.
    uint32_t quarantineThreshold{3};

    bool valid() const
    {
        return initialDelayMs > 0 && maxDelayMs >= initialDelayMs &&
               jitterRatio >= 0.0 && jitterRatio <= 1.0 && maxInFlight >= 1;
    }
};

struct ManagerOptions
{
    explicit ManagerOptions(RetryPolicy policy) : retryPolicy(std::move(policy)) {}

    int64_t     statusIntervalMs = 200;    // one tick: progression + status + heartbeat
    int64_t     pendingTtlMs     = 10000;
    int64_t     maxRegisterTimeoutMs = 5000;   // legal upper bound of registerSource timeoutMs
    int64_t     rateWindowNs     = 1'000'000'000;
    std::string masterName       = "csm_master";
    RetryPolicy retryPolicy;               // D7 values / flag ownership: §12 #6
    int64_t     csmTimeoutNs           = 600'000'000;     // sent to master via CsmRegister
    int64_t     csmDisconnectTimeoutNs = 6'000'000'000;   // recommended >= 10x csmTimeoutNs

    /// §8.2: the options validate themselves; Info's six rules stay separate.
    /// CSM thresholds follow the §2.5.2 shape, and the heartbeat deadline
    /// (2 x statusInterval) must stay below csmTimeoutNs / 2 (v1.2.1).
    std::string validate() const
    {
        if (statusIntervalMs <= 0) return "statusIntervalMs must be > 0";
        if (pendingTtlMs <= 0) return "pendingTtlMs must be > 0";
        if (maxRegisterTimeoutMs <= 0) return "maxRegisterTimeoutMs must be > 0";
        if (rateWindowNs <= 0) return "rateWindowNs must be > 0";
        if (masterName.empty()) return "masterName must not be empty";
        if (!retryPolicy.valid()) return "retryPolicy invalid";
        if (csmTimeoutNs < 0 || csmDisconnectTimeoutNs < 0)
            return "CSM thresholds must be >= 0";
        if (csmTimeoutNs > 0 && csmDisconnectTimeoutNs > 0 &&
            csmDisconnectTimeoutNs <= csmTimeoutNs)
            return "csmDisconnectTimeoutNs must be strictly greater than csmTimeoutNs";
        if (csmTimeoutNs > 0 && statusIntervalMs * 1'000'000 >= csmTimeoutNs / 2)
            return "statusIntervalMs must fit below csmTimeoutNs / 2 so the "
                   "heartbeat deadline can obey the v1.2.1 rule";
        return "";
    }
};

class ControlSignalManager
{
public:
    using InfoT = ControlSignalInfo;
    using EntryStatusT = r1_interfaces::msg::EntryStatus;
    using ManagerStatusT = r1_interfaces::msg::ManagerStatus;
    using ManageSrv = r1_interfaces::srv::ControlSignalManage;
    using InfoReqSrv = r1_interfaces::srv::ControlSignalInfoReq;
    using CsmRegisterSrv = r1_interfaces::srv::CsmRegister;
    using CsmHeartbeatSrv = r1_interfaces::srv::CsmHeartbeat;
    using CsmNotifySrv = r1_interfaces::srv::CsmNotify;

    using RegisterError = r1::RegisterError;   // defined in source_registration.h

    struct RegisterResult
    {
        RegisterError code;
        SourceHandle handle;
    };

    enum class EventKind : uint8_t
    {
        PEER_STATE, PEER_CSM_TIMEOUT, PEER_CSM_ACTIVE,
        PEER_DISCONNECTED, PAIR_MISSING,
        RETRY_STARTED, RETRY_FAILED, RETRY_SUCCEEDED,
        LOCAL_DISCONNECTED,
        SUSPECTED_DATA_PATH_FAULT   // v1.2.1 oscillation damping (§8.3 stage 5)
    };
    struct NotificationEvent
    {
        EventKind kind;
        std::vector<EntryStatusT> entries;
        std::string controllerName;
        RegisterError result;
        uint64_t attemptGeneration;
        std::string reason;
    };
    using NotificationCb = std::function<void(const NotificationEvent&)>;

    ControlSignalManager(rclcpp::Node* node, const std::string& name,
                         const ManagerOptions& opt)
        : node_(node), name_(name), opt_(opt)
        , csmInstanceId_(detail::makeUuid())
    {
        const std::string err = opt_.validate();
        if (!err.empty())
            throw std::invalid_argument("ManagerOptions: " + err);

        mgmtGroup_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        tickGroup_ = node_->create_callback_group(
            rclcpp::CallbackGroupType::MutuallyExclusive);

        manageSrv_ = node_->create_service<ManageSrv>(
            name_ + "/control_signal_manage",
            [this, life = life_](const std::shared_ptr<ManageSrv::Request> req,
                   std::shared_ptr<ManageSrv::Response> res) {
                if (!life->alive.load()) return;
                life->active.fetch_add(1);
                if (!life->alive.load()) { life->active.fetch_sub(1); return; }
                struct Drop { LifeToken* t; ~Drop() { t->active.fetch_sub(1); } }
                    drop{life.get()};
                detail::CallbackGuard g;
                _onManage(*req, *res);
            },
            rclcpp::ServicesQoS(), mgmtGroup_);
        infoReqSrv_ = node_->create_service<InfoReqSrv>(
            name_ + "/control_signal_info_req",
            [this, life = life_](const std::shared_ptr<InfoReqSrv::Request>,
                   std::shared_ptr<InfoReqSrv::Response> res) {
                if (!life->alive.load()) return;
                life->active.fetch_add(1);
                if (!life->alive.load()) { life->active.fetch_sub(1); return; }
                struct Drop { LifeToken* t; ~Drop() { t->active.fetch_sub(1); } }
                    drop{life.get()};
                detail::CallbackGuard g;
                _onInfoReq(*res);
            },
            rclcpp::ServicesQoS(), mgmtGroup_);
        notifySrv_ = node_->create_service<CsmNotifySrv>(
            name_ + "/get_notifications",
            [this, life = life_](const std::shared_ptr<CsmNotifySrv::Request> req,
                   std::shared_ptr<CsmNotifySrv::Response> res) {
                if (!life->alive.load()) return;
                life->active.fetch_add(1);
                if (!life->alive.load()) { life->active.fetch_sub(1); return; }
                struct Drop { LifeToken* t; ~Drop() { t->active.fetch_sub(1); } }
                    drop{life.get()};
                detail::CallbackGuard g;
                _onGetNotifications(*req, *res);
            },
            rclcpp::ServicesQoS(), mgmtGroup_);

        statusPub_ = node_->create_publisher<ManagerStatusT>(name_ + "/status",
                                                             rclcpp::QoS(10));
        masterRegClient_ = node_->create_client<CsmRegisterSrv>(
            opt_.masterName + "/register", rclcpp::ServicesQoS(), mgmtGroup_);
        heartbeatClient_ = node_->create_client<CsmHeartbeatSrv>(
            opt_.masterName + "/heartbeat", rclcpp::ServicesQoS(), mgmtGroup_);

        tickTimer_ = node_->create_wall_timer(
            std::chrono::milliseconds(opt_.statusIntervalMs),
            [this, life = life_] {
                if (!life->alive.load()) return;
                life->active.fetch_add(1);
                if (!life->alive.load()) { life->active.fetch_sub(1); return; }
                struct Drop { LifeToken* t; ~Drop() { t->active.fetch_sub(1); } }
                    drop{life.get()};
                // User state/notification callbacks run on this thread; the
                // guard makes the sync-API precondition observable (M14).
                detail::CallbackGuard g;
                _tick();
            }, tickGroup_);
    }

    ControlSignalManager(const ControlSignalManager&) = delete;
    ControlSignalManager& operator=(const ControlSignalManager&) = delete;

    ~ControlSignalManager()
    {
        life_->alive.store(false);
        tickTimer_->cancel();
        while (life_->active.load() != 0)
            std::this_thread::yield();
        std::vector<std::shared_ptr<BaseControlSignalSource>> srcs;
        std::vector<std::shared_ptr<BaseControlSignalSink>> snks;
        {
            std::vector<std::shared_ptr<SourceRegistrationSlot>> slots;
            {
                std::unique_lock<std::shared_mutex> lk(sourceMtx_);
                for (auto& [c, slot] : sources_)
                    slots.push_back(slot);
                sources_.clear();
            }
            for (auto& slot : slots)
            {
                std::unique_lock<std::shared_mutex> sl(slot->slotMtx);
                slot->desired = false;
                if (slot->endpoint)
                    srcs.push_back(std::move(slot->endpoint));
            }
        }
        {
            std::unique_lock<std::shared_mutex> lk(sinkMtx_);
            for (auto& [c, e] : sinks_)
                if (e.endpoint)
                    snks.push_back(std::move(e.endpoint));
            sinks_.clear();
        }
        for (auto& s : srcs) s->shutdown();
        for (auto& s : snks) s->shutdown();
    }

    // ── user API ──────────────────────────────────────────────────────────

    /// Two-phase registration (§2.4/§8.3): PENDING placeholder under the map
    /// lock, synchronous REGISTER outside it, endpoint + REGISTERED only
    /// after re-validating desired/phase/identity under the slot lock.
    /// Forbidden inside ROS callbacks and before the executor spins (§2.6).
    RegisterResult registerSource(const InfoT& info, int64_t timeoutMs = 5000)
    {
        if (detail::tlInManagerCallback || !executorObserved_.load())
            return {RegisterError::INVALID_CONTEXT, {}};
        if (timeoutMs <= 0 || timeoutMs > opt_.maxRegisterTimeoutMs)
            return {RegisterError::INVALID_CONTEXT, {}};

        const InfoValidation v = validateControlSignalInfo(info);
        if (!v.valid)
            return {RegisterError::INVALID_INFO, {}};
        if (!_allowed(info.controller_name))
            return {RegisterError::FILTERED, {}};
        auto& factory = ControlSignalFactory::Instance();
        if (!factory.has(info.type) ||
            (info.mode == InfoT::MODE_SERVICE && !factory.serviceCapable(info.type)))
            return {RegisterError::TYPE_UNSUPPORTED, {}};

        // Atomic duplicate check + PENDING slot insertion (§1.3: emplace,
        // never operator[]; M2: local duplicates never reach the wire).
        std::shared_ptr<SourceRegistrationSlot> slot;
        {
            std::unique_lock<std::shared_mutex> lk(sourceMtx_);
            for (const auto& [c, s] : sources_)
                if (c == info.controller_name ||
                    s->info.channel_name == info.channel_name)
                    return {RegisterError::DUPLICATE, {}};
            slot = std::make_shared<SourceRegistrationSlot>();
            slot->info = info;
            slot->registrationId = detail::makeUuid();
            slot->attemptGeneration = 1;
            sources_.emplace(info.controller_name, slot);
        }
        SourceHandle handle(slot, info.controller_name);

        const RegistrationIdentity identity{csmInstanceId_, slot->registrationId, 1};
        auto req = std::make_shared<ManageSrv::Request>();
        req->op = ManageSrv::Request::OP_REGISTER;
        req->source_manager_name = name_;
        req->source_csm_instance_id = identity.sourceCsmInstanceId;
        req->registration_id = identity.registrationId;
        req->attempt_generation = identity.attemptGeneration;
        req->info = info;

        auto client = _manageClient(info.target_manager_name);
        if (!client->service_is_ready())
            return _initialFailure(slot, identity, RegisterError::TARGET_UNREACHABLE,
                                   RetryReason::INITIAL_UNREACHABLE,
                                   "target manage service not ready", handle);

        auto future = client->async_send_request(req);
        if (future.wait_for(std::chrono::milliseconds(timeoutMs)) !=
            std::future_status::ready)
        {
            // Outcome unknown: rollback with a matching-generation
            // best-effort UNREGISTER, then drop the slot (§8.3, M6).
            client->remove_pending_request(future);
            _sendBestEffortUnregister(info.target_manager_name,
                                      info.controller_name, identity);
            _dropSlot(slot, info.controller_name);
            return {RegisterError::TIMEOUT_UNKNOWN, {}};
        }

        const auto res = future.get();
        switch (res->response)
        {
            case ManageSrv::Response::RESPONSE_SUCCESS:
            case ManageSrv::Response::RESPONSE_ALREADY_APPLIED:
                break;
            case ManageSrv::Response::RESPONSE_RETRYABLE_CONFLICT:
                return _initialFailure(slot, identity, RegisterError::RETRYABLE_CONFLICT,
                                       RetryReason::OLD_GENERATION_CONFLICT,
                                       res->reason, handle);
            case ManageSrv::Response::RESPONSE_TEMPORARY_UNAVAILABLE:
                return _initialFailure(slot, identity, RegisterError::TARGET_UNREACHABLE,
                                       RetryReason::INITIAL_UNREACHABLE,
                                       res->reason, handle);
            default:   // PERMANENT_REJECTION / STALE / ERROR: never retried
                _dropSlot(slot, info.controller_name);
                return {RegisterError::REJECTED, {}};
        }

        std::string err;
        auto endpoint = factory.CreateSource(info.type, node_, info, &err,
                                             opt_.rateWindowNs);
        if (!endpoint)
        {
            _sendBestEffortUnregister(info.target_manager_name,
                                      info.controller_name, identity);
            _dropSlot(slot, info.controller_name);
            return {RegisterError::TYPE_UNSUPPORTED, {}};
        }
        _installAllStateCbs(endpoint);   // global per-state slots apply (M18)
        {
            std::unique_lock<std::shared_mutex> sl(slot->slotMtx);
            if (!slot->desired || slot->phase != RegistrationPhase::PENDING ||
                slot->registrationId != identity.registrationId ||
                slot->attemptGeneration != identity.attemptGeneration)
            {
                sl.unlock();
                endpoint->shutdown();
                // The remote side accepted; roll the orphan back (M7's
                // in-flight unregister crossing exactly this window).
                _sendBestEffortUnregister(info.target_manager_name,
                                          info.controller_name, identity);
                return {RegisterError::TIMEOUT_UNKNOWN, {}};
            }
            slot->endpoint = std::move(endpoint);
            slot->phase = RegistrationPhase::REGISTERED;
            slot->wasActiveSinceEndpoint = false;
        }
        return {RegisterError::OK, handle};
    }

    /// M7: desired drops at once (handle invalidates immediately); endpoints
    /// are terminally removed by the next tick; RETRY_WAIT slots vanish now.
    /// A matching-generation best-effort UNREGISTER always goes out.
    bool unregisterSource(const SourceHandle& h, int64_t timeoutMs = 5000)
    {
        (void)timeoutMs;
        if (detail::tlInManagerCallback || !executorObserved_.load())
            return false;
        auto slot = h.slot_.lock();
        if (!slot)
            return false;

        RegistrationIdentity identity;
        std::string target, controller;
        bool hasEndpoint = false;
        {
            std::unique_lock<std::shared_mutex> sl(slot->slotMtx);
            if (!slot->desired)
                return false;   // already unregistered
            slot->desired = false;
            identity = {csmInstanceId_, slot->registrationId, slot->attemptGeneration};
            target = slot->info.target_manager_name;
            controller = slot->info.controller_name;
            hasEndpoint = slot->endpoint != nullptr;
            if (hasEndpoint)
                slot->pendingRemoval = RemovalReason::EXPLICIT_UNREGISTER;
        }
        {
            std::lock_guard<std::mutex> lk(retryMtx_);
            retryTable_.erase(slot->registrationId);
        }
        if (!hasEndpoint)
            _dropSlot(slot, controller);   // RETRY_WAIT / PENDING: gone now
        _sendBestEffortUnregister(target, controller, identity);
        return true;
    }

    SourceHandle getSource(const std::string& controllerName) const
    {
        std::shared_lock<std::shared_mutex> lk(sourceMtx_);
        const auto it = sources_.find(controllerName);
        if (it == sources_.end())
            return {};
        return SourceHandle(it->second, controllerName);
    }

    SinkHandle getSink(const std::string& controllerName) const
    {
        std::shared_lock<std::shared_mutex> lk(sinkMtx_);
        const auto it = sinks_.find(controllerName);
        if (it == sinks_.end() || !it->second.endpoint)
            return {};
        return SinkHandle(it->second.endpoint);
    }

    /// No entry -> nullopt (replaces rv2's ambiguous UNKNOWN, §2.3).
    std::optional<ControlSignalState> getSourceState(const std::string& controllerName) const
    {
        std::shared_ptr<SourceRegistrationSlot> slot;
        {
            std::shared_lock<std::shared_mutex> lk(sourceMtx_);
            const auto it = sources_.find(controllerName);
            if (it == sources_.end())
                return std::nullopt;
            slot = it->second;
        }
        std::shared_lock<std::shared_mutex> sl(slot->slotMtx);
        if (!slot->endpoint)
            return std::nullopt;
        return slot->endpoint->getState();
    }

    std::optional<ControlSignalState> getSinkState(const std::string& controllerName) const
    {
        std::shared_lock<std::shared_mutex> lk(sinkMtx_);
        const auto it = sinks_.find(controllerName);
        if (it == sinks_.end() || !it->second.endpoint)
            return std::nullopt;
        return it->second.endpoint->getState();
    }

    std::vector<InfoT> getSourceInfoList() const
    {
        std::vector<std::shared_ptr<SourceRegistrationSlot>> slots;
        {
            std::shared_lock<std::shared_mutex> lk(sourceMtx_);
            for (const auto& [c, slot] : sources_)
                slots.push_back(slot);
        }
        std::vector<InfoT> out;
        for (const auto& slot : slots)
        {
            std::shared_lock<std::shared_mutex> sl(slot->slotMtx);
            if (slot->phase == RegistrationPhase::REGISTERED && slot->endpoint)
                out.push_back(slot->info);
        }
        return out;
    }

    std::vector<InfoT> getSinkInfoList() const
    {
        std::vector<InfoT> out;
        std::shared_lock<std::shared_mutex> lk(sinkMtx_);
        for (const auto& [c, e] : sinks_)
            if (e.phase == RegistrationPhase::REGISTERED && e.endpoint)
                out.push_back(e.endpoint->getInfo());
        return out;
    }

    void setNotificationCallback(NotificationCb cb)
    {
        std::lock_guard<std::mutex> lk(notifyCbMtx_);
        notifyCb_ = std::move(cb);
    }

    /// Per-state transition callbacks for ALL managed sources / sinks; one
    /// slot per state, later registration overwrites, nullptr clears; always
    /// fired on the tick thread (§8.2, M18). Applied to existing and future
    /// endpoints.
    void registerSourceStateCallback(ControlSignalState state, StateCb cb)
    {
        {
            std::lock_guard<std::mutex> lk(stateCbMtx_);
            sourceStateCbs_[static_cast<size_t>(state)] = cb;
        }
        std::vector<std::shared_ptr<SourceRegistrationSlot>> slots;
        {
            std::shared_lock<std::shared_mutex> lk(sourceMtx_);
            for (auto& [c, slot] : sources_)
                slots.push_back(slot);
        }
        for (auto& slot : slots)
        {
            std::shared_ptr<BaseControlSignalSource> ep;
            {
                std::shared_lock<std::shared_mutex> sl(slot->slotMtx);
                ep = slot->endpoint;
            }
            if (ep)
                _installSourceStateCb(ep, state, cb);
        }
    }

    void registerSinkStateCallback(ControlSignalState state, StateCb cb)
    {
        {
            std::lock_guard<std::mutex> lk(stateCbMtx_);
            sinkStateCbs_[static_cast<size_t>(state)] = cb;
        }
        std::shared_lock<std::shared_mutex> lk(sinkMtx_);
        for (auto& [c, e] : sinks_)
            if (e.endpoint)
                _installSinkStateCb(e.endpoint, state, cb);
    }

    /// Sink message callbacks keyed by factory type string; one per type,
    /// later registration overwrites; both orders work (M12).
    template<typename msgT>
    bool registerCallback(std::function<void(const msgT&, const InfoT&)> cb)
    {
        const std::string type =
            ControlSignalFactory::Instance().typeKey(std::type_index(typeid(msgT)));
        if (type.empty())
            return false;
        return registerCallback(type,
            [cb = std::move(cb)](const void* m, const InfoT& i) {
                cb(*static_cast<const msgT*>(m), i);
            });
    }

    bool registerCallback(const std::string& type,
                          BaseControlSignalSink::ErasedMsgCb cb)
    {
        if (!ControlSignalFactory::Instance().has(type))
            return false;
        {
            std::lock_guard<std::mutex> lk(typedCbMtx_);
            typedCbs_[type] = cb;
        }
        _applyTypedCb(type, cb);
        return true;
    }

    void unregisterCallback(const std::string& type)
    {
        {
            std::lock_guard<std::mutex> lk(typedCbMtx_);
            typedCbs_.erase(type);
        }
        _applyTypedCb(type, nullptr);
    }

    /// Black/whitelist keyed by controller_name, applied on BOTH the
    /// registerSource and the _onManage side (M11). An enabled empty
    /// whitelist blocks everything.
    void enableControllerWhitelist(const std::vector<std::string>& names)
    {
        std::lock_guard<std::mutex> lk(filterMtx_);
        whitelist_.emplace(names.begin(), names.end());
    }
    void disableControllerWhitelist()
    {
        std::lock_guard<std::mutex> lk(filterMtx_);
        whitelist_.reset();
    }
    void enableControllerBlacklist(const std::vector<std::string>& names)
    {
        std::lock_guard<std::mutex> lk(filterMtx_);
        blacklist_.emplace(names.begin(), names.end());
    }
    void disableControllerBlacklist()
    {
        std::lock_guard<std::mutex> lk(filterMtx_);
        blacklist_.reset();
    }

    const std::string& getName() const { return name_; }
    const std::string& getCsmInstanceId() const { return csmInstanceId_; }
    bool isDegraded() const { return degraded_.load(); }

private:
    friend class SourceHandle;
    friend struct ManagerTestAccess;

    struct SinkEntry
    {
        RegistrationIdentity identity;
        std::string sourceManagerName;
        RegistrationPhase phase{RegistrationPhase::PENDING};
        std::shared_ptr<BaseControlSignalSink> endpoint;
        int64_t pendingSinceNs{0};
        EntityStatus lastStatus{ControlSignalState::INITIAL, 0.f};
        PeerHealth peerHealth{PeerHealth::UNKNOWN};
        std::optional<RemovalReason> pendingRemoval;
    };

    struct PendingRegister
    {
        std::weak_ptr<SourceRegistrationSlot> slot;
        RetryReason reason;
        uint32_t attempts{0};
        int64_t nextTryNs{0};
        bool inFlight{false};
        int64_t inFlightDeadlineNs{0};
        uint64_t inFlightGeneration{0};   // stale completions must not clear newer attempts
        bool everSucceeded{false};   // initial cap applies only before first success
    };

    // ── helpers ───────────────────────────────────────────────────────────

    bool _allowed(const std::string& controller) const
    {
        std::lock_guard<std::mutex> lk(filterMtx_);
        if (whitelist_ && whitelist_->count(controller) == 0)
            return false;
        if (blacklist_ && blacklist_->count(controller) != 0)
            return false;
        return true;
    }

    rclcpp::Client<ManageSrv>::SharedPtr _manageClient(const std::string& target)
    {
        std::lock_guard<std::mutex> lk(clientMtx_);
        auto& c = manageClients_[target];
        if (!c)
            c = node_->create_client<ManageSrv>(target + "/control_signal_manage",
                                                rclcpp::ServicesQoS(), mgmtGroup_);
        return c;
    }

    void _dropSlot(const std::shared_ptr<SourceRegistrationSlot>& slot,
                   const std::string& controller)
    {
        {
            std::unique_lock<std::shared_mutex> sl(slot->slotMtx);
            slot->desired = false;
        }
        std::unique_lock<std::shared_mutex> lk(sourceMtx_);
        const auto it = sources_.find(controller);
        if (it != sources_.end() && it->second == slot)
            sources_.erase(it);
    }

    /// Initial-registration failure of a retryable class (§8.3/§2.4):
    /// a typed D3 conflict ALWAYS enters RETRY_WAIT (mandatory, waits the
    /// old generation out); only the initial-unreachable case is gated by
    /// the D7 policy — disabled, the slot goes away with the raw error.
    RegisterResult _initialFailure(const std::shared_ptr<SourceRegistrationSlot>& slot,
                                   const RegistrationIdentity& identity,
                                   RegisterError code, RetryReason reason,
                                   const std::string& why, SourceHandle handle)
    {
        const bool d7Gated = reason == RetryReason::INITIAL_UNREACHABLE;
        if (d7Gated && (!opt_.retryPolicy.autoRetryInitial ||
                        opt_.retryPolicy.maxInitialAttempts == 0))
        {
            _dropSlot(slot, slot->info.controller_name);
            (void)why;
            return {code, {}};
        }
        {
            std::unique_lock<std::shared_mutex> sl(slot->slotMtx);
            if (!slot->desired)
                return {code, {}};
            slot->phase = RegistrationPhase::RETRY_WAIT;
        }
        _enqueueRetry(slot, identity.registrationId, reason, /*everSucceeded=*/false);
        return {RegisterError::RETRY_SCHEDULED, handle};
    }

    void _enqueueRetry(const std::shared_ptr<SourceRegistrationSlot>& slot,
                       const std::string& registrationId, RetryReason reason,
                       bool everSucceeded)
    {
        // Quarantine check outside retryMtx_ (no nested locks): rebuilds that
        // keep terminating without ever turning ACTIVE switch to the long
        // backoff and raise SUSPECTED_DATA_PATH_FAULT (v1.2.1, §8.3 stage 5).
        uint32_t neverActive = 0;
        std::string controller;
        uint64_t generation = 0;
        {
            std::shared_lock<std::shared_mutex> sl(slot->slotMtx);
            neverActive = slot->neverActiveTerminations;
            controller = slot->info.controller_name;
            generation = slot->attemptGeneration;
        }
        const bool quarantined =
            neverActive >= opt_.retryPolicy.quarantineThreshold;
        const int64_t delayMs = quarantined ? opt_.retryPolicy.maxDelayMs
                                            : opt_.retryPolicy.initialDelayMs;
        {
            std::lock_guard<std::mutex> lk(retryMtx_);
            auto [it, inserted] = retryTable_.try_emplace(registrationId);
            if (inserted)
            {
                it->second.slot = slot;
                it->second.reason = reason;
                it->second.attempts = 0;
                it->second.nextTryNs = steadyNowNs() + delayMs * 1'000'000;
                it->second.everSucceeded = everSucceeded;
            }
            // Deduplicated by registrationId: a master lifecycle event landing
            // after a local RESPONSE_FAILURE must not add a second entry (§8.3).
        }
        if (quarantined)
            _notify(EventKind::SUSPECTED_DATA_PATH_FAULT, controller,
                    RegisterError::OK, generation,
                    "endpoint keeps terminating without ever turning ACTIVE");
    }

    void _sendBestEffortUnregister(const std::string& target,
                                   const std::string& controller,
                                   const RegistrationIdentity& identity)
    {
        auto client = _manageClient(target);
        if (!client->service_is_ready())
            return;
        auto req = std::make_shared<ManageSrv::Request>();
        req->op = ManageSrv::Request::OP_UNREGISTER;
        req->source_manager_name = name_;
        req->source_csm_instance_id = identity.sourceCsmInstanceId;
        req->registration_id = identity.registrationId;
        req->attempt_generation = identity.attemptGeneration;
        req->info.controller_name = controller;
        client->async_send_request(req,
            [](rclcpp::Client<ManageSrv>::SharedFuture) {});   // best-effort
    }

    void _notify(EventKind kind, const std::string& controller,
                 RegisterError result, uint64_t generation,
                 const std::string& reason,
                 std::vector<EntryStatusT> entries = {})
    {
        NotificationCb cb;
        {
            std::lock_guard<std::mutex> lk(notifyCbMtx_);
            cb = notifyCb_;
        }
        if (!cb)
            return;
        NotificationEvent ev;
        ev.kind = kind;
        ev.entries = std::move(entries);
        ev.controllerName = controller;
        ev.result = result;
        ev.attemptGeneration = generation;
        ev.reason = reason;
        cb(ev);
    }

    void _installSourceStateCb(const std::shared_ptr<BaseControlSignalSource>& ep,
                               ControlSignalState state, const StateCb& cb);
    void _installSinkStateCb(const std::shared_ptr<BaseControlSignalSink>& ep,
                             ControlSignalState state, const StateCb& cb);
    void _installAllStateCbs(const std::shared_ptr<BaseControlSignalSource>& ep);
    void _installAllStateCbs(const std::shared_ptr<BaseControlSignalSink>& ep);

    void _applyTypedCb(const std::string& type,
                       BaseControlSignalSink::ErasedMsgCb cb)
    {
        std::vector<std::shared_ptr<BaseControlSignalSink>> eps;
        {
            std::shared_lock<std::shared_mutex> lk(sinkMtx_);
            for (auto& [c, e] : sinks_)
                if (e.endpoint && e.endpoint->getInfo().type == type)
                    eps.push_back(e.endpoint);
        }
        for (auto& ep : eps)
            ep->setErasedMsgCallback(cb);
    }

    // ── service handlers (Reentrant mgmtGroup_) ───────────────────────────

    void _onManage(const ManageSrv::Request& req, ManageSrv::Response& res)
    {
        if (req.op == ManageSrv::Request::OP_REGISTER)
            _onManageRegister(req, res);
        else if (req.op == ManageSrv::Request::OP_UNREGISTER)
            _onManageUnregister(req, res);
        else
        {
            res.response = ManageSrv::Response::RESPONSE_ERROR;
            res.reason = "unknown op";
        }
    }

    void _onManageRegister(const ManageSrv::Request& req, ManageSrv::Response& res)
    {
        // Validate control identity, Info and filter (§8.3).
        if (req.source_manager_name.empty() || req.source_csm_instance_id.empty() ||
            req.registration_id.empty() || req.attempt_generation == 0)
        {
            res.response = ManageSrv::Response::RESPONSE_PERMANENT_REJECTION;
            res.reason = "incomplete control identity";
            return;
        }
        const InfoValidation v = validateControlSignalInfo(req.info);
        if (!v.valid)
        {
            res.response = ManageSrv::Response::RESPONSE_PERMANENT_REJECTION;
            res.reason = "invalid info: " + v.error;
            return;
        }
        if (req.info.target_manager_name != name_)
        {
            res.response = ManageSrv::Response::RESPONSE_PERMANENT_REJECTION;
            res.reason = "mis-routed: target '" + req.info.target_manager_name +
                         "' is not '" + name_ + "'";
            return;
        }
        if (!_allowed(req.info.controller_name))
        {
            res.response = ManageSrv::Response::RESPONSE_PERMANENT_REJECTION;
            res.reason = "filtered by white/blacklist";
            return;
        }
        auto& factory = ControlSignalFactory::Instance();
        if (!factory.has(req.info.type) ||
            (req.info.mode == InfoT::MODE_SERVICE &&
             !factory.serviceCapable(req.info.type)))
        {
            res.response = ManageSrv::Response::RESPONSE_PERMANENT_REJECTION;
            res.reason = "unsupported type/mode";
            return;
        }

        const RegistrationIdentity identity{req.source_csm_instance_id,
                                            req.registration_id,
                                            req.attempt_generation};
        // Duplicate check + idempotency + PENDING emplace under one lock.
        {
            std::unique_lock<std::shared_mutex> lk(sinkMtx_);
            const auto it = sinks_.find(req.info.controller_name);
            if (it != sinks_.end())
            {
                if (it->second.identity == identity)
                {
                    // Idempotent resend of an applied transaction (D3).
                    res.response =
                        it->second.phase == RegistrationPhase::REGISTERED
                            ? ManageSrv::Response::RESPONSE_ALREADY_APPLIED
                            : ManageSrv::Response::RESPONSE_TEMPORARY_UNAVAILABLE;
                    res.reason = "";
                    return;
                }
                // Same name, different identity: always refused, whatever
                // the local state; retry waits the old generation out (D3).
                res.response = ManageSrv::Response::RESPONSE_RETRYABLE_CONFLICT;
                res.reason = "existing endpoint with different identity";
                return;
            }
            SinkEntry e;
            e.identity = identity;
            e.sourceManagerName = req.source_manager_name;
            e.phase = RegistrationPhase::PENDING;
            e.pendingSinceNs = steadyNowNs();
            sinks_.emplace(req.info.controller_name, std::move(e));
        }

        // Build the Sink outside the lock (§8.3).
        std::string err;
        auto endpoint = factory.CreateSink(req.info.type, node_, req.info, &err,
                                           opt_.rateWindowNs);
        if (!endpoint)
        {
            std::unique_lock<std::shared_mutex> lk(sinkMtx_);
            const auto it = sinks_.find(req.info.controller_name);
            // Re-confirm before erasing: a concurrent UNREGISTER + newer
            // REGISTER may own this key by now (§8.3 single-lock contention).
            if (it != sinks_.end() && it->second.identity == identity &&
                it->second.phase == RegistrationPhase::PENDING &&
                !it->second.endpoint)
                sinks_.erase(it);
            res.response = ManageSrv::Response::RESPONSE_PERMANENT_REJECTION;
            res.reason = "sink creation failed: " + err;
            return;
        }
        // Install the typed callback + state callbacks before going live.
        {
            std::lock_guard<std::mutex> lk(typedCbMtx_);
            const auto cbIt = typedCbs_.find(req.info.type);
            if (cbIt != typedCbs_.end())
                endpoint->setErasedMsgCallback(cbIt->second);
        }
        _installAllStateCbs(endpoint);

        // Re-confirm phase and identity under the unique lock; the PENDING
        // TTL reaper and matching UNREGISTERs race through this same lock.
        {
            std::unique_lock<std::shared_mutex> lk(sinkMtx_);
            const auto it = sinks_.find(req.info.controller_name);
            if (it == sinks_.end() || it->second.identity != identity ||
                it->second.phase != RegistrationPhase::PENDING ||
                it->second.pendingRemoval.has_value())
            {
                lk.unlock();
                endpoint->shutdown();
                res.response = ManageSrv::Response::RESPONSE_STALE;
                res.reason = "transaction superseded";
                return;
            }
            it->second.endpoint = std::move(endpoint);
            it->second.phase = RegistrationPhase::REGISTERED;
        }
        res.response = ManageSrv::Response::RESPONSE_SUCCESS;
        res.reason = "";
    }

    void _onManageUnregister(const ManageSrv::Request& req, ManageSrv::Response& res)
    {
        const RegistrationIdentity identity{req.source_csm_instance_id,
                                            req.registration_id,
                                            req.attempt_generation};
        std::unique_lock<std::shared_mutex> lk(sinkMtx_);
        const auto it = sinks_.find(req.info.controller_name);
        if (it == sinks_.end())
        {
            // Same identity already removed -> idempotent ALREADY_APPLIED;
            // anything else is stale (§8.3).
            const auto done = completedRemovals_.find(identity.registrationId);
            if (done != completedRemovals_.end() &&
                done->second == identity.attemptGeneration)
            {
                res.response = ManageSrv::Response::RESPONSE_ALREADY_APPLIED;
                return;
            }
            res.response = ManageSrv::Response::RESPONSE_STALE;
            res.reason = "no such entry";
            return;
        }
        if (it->second.identity != identity)
        {
            res.response = ManageSrv::Response::RESPONSE_STALE;
            res.reason = "identity mismatch";
            return;
        }
        // Only queue the removal; state callback / shutdown / erase belong
        // to the tick (§8.3). PENDING entries are simply reverted here.
        if (it->second.phase == RegistrationPhase::PENDING && !it->second.endpoint)
        {
            completedRemovals_[identity.registrationId] = identity.attemptGeneration;
            sinks_.erase(it);
        }
        else
        {
            it->second.pendingRemoval = RemovalReason::EXPLICIT_UNREGISTER;
        }
        res.response = ManageSrv::Response::RESPONSE_SUCCESS;
        res.reason = "";
    }

    void _onInfoReq(InfoReqSrv::Response& res)
    {
        res.response = InfoReqSrv::Response::RESPONSE_SUCCESS;
        res.source_list = getSourceInfoList();
        res.sink_list = getSinkInfoList();
    }

    void _onGetNotifications(const CsmNotifySrv::Request& req,
                             CsmNotifySrv::Response& res)
    {
        if (req.target_csm_instance_id != csmInstanceId_)
        {
            res.response = CsmNotifySrv::Response::RESPONSE_STALE;
            res.reason = "wrong incarnation";
            return;
        }
        {
            std::lock_guard<std::mutex> lk(eventMtx_);
            if (appliedEvents_.count(req.event_id))
            {
                res.response = CsmNotifySrv::Response::RESPONSE_ALREADY_APPLIED;
                return;
            }
        }

        bool anyStale = false;
        switch (req.kind)
        {
            case CsmNotifySrv::Request::KIND_STATE:
                // Observation only: never touches table or local state.
                _notify(EventKind::PEER_STATE, "", RegisterError::OK, 0, "",
                        req.entries);
                break;
            case CsmNotifySrv::Request::KIND_CSM_TIMEOUT:
            {
                const bool healthy =
                    req.peer_csm_health == CsmNotifySrv::Request::PEER_HEALTH_ACTIVE;
                anyStale = !_updatePeerHealth(req.entries,
                                              healthy ? PeerHealth::ACTIVE
                                                      : PeerHealth::TIMEOUT);
                if (!anyStale)
                    _notify(healthy ? EventKind::PEER_CSM_ACTIVE
                                    : EventKind::PEER_CSM_TIMEOUT,
                            "", RegisterError::OK, 0, "", req.entries);
                break;
            }
            case CsmNotifySrv::Request::KIND_DISCONNECTED:
            case CsmNotifySrv::Request::KIND_PAIR_MISSING:
            {
                const RemovalReason reason =
                    req.kind == CsmNotifySrv::Request::KIND_DISCONNECTED
                        ? RemovalReason::PEER_DISCONNECTED
                        : RemovalReason::PAIR_MISSING;
                anyStale = !_queueMatchingRemovals(req.entries, reason);
                break;
            }
            default:
                res.response = CsmNotifySrv::Response::RESPONSE_REJECTED;
                res.reason = "unknown kind";
                return;
        }

        if (anyStale)
        {
            res.response = CsmNotifySrv::Response::RESPONSE_STALE;
            res.reason = "no matching identity";
            return;
        }
        {
            std::lock_guard<std::mutex> lk(eventMtx_);
            if (appliedEvents_.insert(req.event_id).second)
            {
                appliedOrder_.push_back(req.event_id);
                if (appliedOrder_.size() > 4096)   // FIFO eviction: oldest first
                {
                    appliedEvents_.erase(appliedOrder_.front());
                    appliedOrder_.pop_front();
                }
            }
        }
        res.response = CsmNotifySrv::Response::RESPONSE_APPLIED;
        res.reason = "";
    }

    /// Applies peer health to matching identities only; returns true when at
    /// least one entry matched (stale generations -> STALE, like removals).
    /// Map and slot locks are never held together (§8.3 lock rules).
    bool _updatePeerHealth(const std::vector<EntryStatusT>& entries, PeerHealth ph)
    {
        bool matched = false;
        for (const auto& e : entries)
        {
            std::shared_ptr<SourceRegistrationSlot> slot;
            {
                std::shared_lock<std::shared_mutex> lk(sourceMtx_);
                const auto it = sources_.find(e.controller_name);
                if (it != sources_.end())
                    slot = it->second;
            }
            if (slot)
            {
                std::unique_lock<std::shared_mutex> sl(slot->slotMtx);
                if (slot->registrationId == e.registration_id &&
                    slot->attemptGeneration == e.attempt_generation)
                {
                    slot->peerHealth = ph;
                    matched = true;
                }
                continue;
            }
            const RegistrationIdentity identity{e.source_csm_instance_id,
                                                e.registration_id,
                                                e.attempt_generation};
            std::unique_lock<std::shared_mutex> lk(sinkMtx_);
            const auto it = sinks_.find(e.controller_name);
            if (it != sinks_.end() && it->second.identity == identity)
            {
                it->second.peerHealth = ph;
                matched = true;
            }
        }
        return matched;
    }

    /// Queues one removal per matching identity; returns true if at least
    /// one entry matched a live identity (M8: stale generations -> STALE).
    bool _queueMatchingRemovals(const std::vector<EntryStatusT>& entries,
                                RemovalReason reason)
    {
        bool matched = false;
        for (const auto& e : entries)
        {
            const RegistrationIdentity identity{e.source_csm_instance_id,
                                                e.registration_id,
                                                e.attempt_generation};
            std::shared_ptr<SourceRegistrationSlot> slot;
            {
                std::shared_lock<std::shared_mutex> lk(sourceMtx_);
                const auto it = sources_.find(e.controller_name);
                if (it != sources_.end())
                    slot = it->second;
            }
            if (slot)
            {
                std::unique_lock<std::shared_mutex> sl(slot->slotMtx);
                if (slot->registrationId == identity.registrationId &&
                    slot->attemptGeneration == identity.attemptGeneration &&
                    csmInstanceId_ == identity.sourceCsmInstanceId)
                {
                    if (!slot->pendingRemoval)
                        slot->pendingRemoval = reason;
                    matched = true;
                }
                continue;
            }
            std::unique_lock<std::shared_mutex> lk(sinkMtx_);
            const auto it = sinks_.find(e.controller_name);
            if (it != sinks_.end() && it->second.identity == identity)
            {
                if (!it->second.pendingRemoval)
                    it->second.pendingRemoval = reason;
                matched = true;
            }
        }
        return matched;
    }

    // ── the tick (§8.3, five phases) ──────────────────────────────────────

    void _tick()
    {
        if (tickRunning_.exchange(true))
            return;   // idempotent guard against re-entry
        executorObserved_.store(true);
        const int64_t now = steadyNowNs();

        _tickCalcAndCommit(now);
        _tickPublishAndHeartbeat(now);
        _tickRetryAndMaintenance(now);

        firstTickDone_.store(true);
        tickRunning_.store(false);
    }

    struct SourceWork
    {
        std::string controller;
        std::shared_ptr<SourceRegistrationSlot> slot;
        std::shared_ptr<BaseControlSignalSource> endpoint;
        RegistrationIdentity identity;
        std::optional<RemovalReason> pendingRemoval;
        EntityDecision decision{};
    };
    struct SinkWork
    {
        std::string controller;
        std::shared_ptr<BaseControlSignalSink> endpoint;
        RegistrationIdentity identity;
        std::optional<RemovalReason> pendingRemoval;
        EntityDecision decision{};
    };

    void _tickCalcAndCommit(int64_t now)
    {
        // Phase 1: snapshot under short shared locks, calc outside any lock.
        std::vector<SourceWork> sw;
        {
            std::vector<std::pair<std::string, std::shared_ptr<SourceRegistrationSlot>>> slots;
            {
                std::shared_lock<std::shared_mutex> lk(sourceMtx_);
                for (const auto& [c, s] : sources_)
                    slots.emplace_back(c, s);
            }
            for (auto& [c, s] : slots)
            {
                SourceWork w;
                w.controller = c;
                w.slot = s;
                {
                    std::shared_lock<std::shared_mutex> sl(s->slotMtx);
                    if (s->phase != RegistrationPhase::REGISTERED || !s->endpoint)
                    {
                        if (s->pendingRemoval || !s->desired)
                        {
                            w.pendingRemoval = s->pendingRemoval;
                            w.identity = {csmInstanceId_, s->registrationId,
                                          s->attemptGeneration};
                            sw.push_back(std::move(w));
                        }
                        continue;
                    }
                    w.endpoint = s->endpoint;
                    w.identity = {csmInstanceId_, s->registrationId,
                                  s->attemptGeneration};
                    w.pendingRemoval = s->pendingRemoval;
                }
                w.decision = w.endpoint->_calcStatus(now);
                sw.push_back(std::move(w));
            }
        }
        std::vector<SinkWork> kw;
        {
            std::vector<SinkWork> snap;
            {
                std::shared_lock<std::shared_mutex> lk(sinkMtx_);
                for (const auto& [c, e] : sinks_)
                {
                    if (e.phase != RegistrationPhase::REGISTERED || !e.endpoint)
                        continue;
                    SinkWork w;
                    w.controller = c;
                    w.endpoint = e.endpoint;
                    w.identity = e.identity;
                    w.pendingRemoval = e.pendingRemoval;
                    snap.push_back(std::move(w));
                }
            }
            for (auto& w : snap)
            {
                w.decision = w.endpoint->_calcStatus(now);
                kw.push_back(std::move(w));
            }
        }

        // Phase 2: commit non-terminal results.
        for (auto& w : sw)
        {
            if (!w.endpoint || w.pendingRemoval ||
                w.decision.status.state == ControlSignalState::DISCONNECTED)
                continue;
            bool ok = false;
            {
                std::unique_lock<std::shared_mutex> sl(w.slot->slotMtx);
                if (w.slot->desired &&
                    w.slot->phase == RegistrationPhase::REGISTERED &&
                    w.slot->endpoint == w.endpoint &&
                    w.slot->attemptGeneration == w.identity.attemptGeneration)
                {
                    w.slot->lastStatus = w.decision.status;
                    if (w.decision.status.state == ControlSignalState::ACTIVE)
                    {
                        w.slot->wasActiveSinceEndpoint = true;
                        w.slot->neverActiveTerminations = 0;
                    }
                    ok = true;
                }
            }
            if (ok)
                w.endpoint->_applyStatus(w.decision);
        }
        for (auto& w : kw)
        {
            if (w.pendingRemoval ||
                w.decision.status.state == ControlSignalState::DISCONNECTED)
                continue;
            bool ok = false;
            {
                std::unique_lock<std::shared_mutex> lk(sinkMtx_);
                const auto it = sinks_.find(w.controller);
                if (it != sinks_.end() && it->second.identity == w.identity &&
                    it->second.endpoint == w.endpoint && !it->second.pendingRemoval)
                {
                    it->second.lastStatus = w.decision.status;
                    ok = true;
                }
            }
            if (ok)
                w.endpoint->_applyStatus(w.decision);
        }

        // Phase 3: terminal commit + same-tick deregistration.
        for (auto& w : sw)
        {
            const bool localTerminal =
                w.endpoint &&
                w.decision.status.state == ControlSignalState::DISCONNECTED &&
                !w.pendingRemoval;
            if (!localTerminal && !w.pendingRemoval)
                continue;
            _terminalCommitSource(w, now, localTerminal);
        }
        for (auto& w : kw)
        {
            const bool localTerminal =
                w.decision.status.state == ControlSignalState::DISCONNECTED &&
                !w.pendingRemoval;
            if (!localTerminal && !w.pendingRemoval)
                continue;
            _terminalCommitSink(w, localTerminal);
        }
    }

    void _terminalCommitSource(SourceWork& w, int64_t now, bool localTerminal)
    {
        (void)now;
        RemovalReason reason;
        if (localTerminal)
        {
            reason = w.decision.cause == LivenessCause::RESPONSE_FAILURE
                         ? RemovalReason::RESPONSE_FAILURE
                         : RemovalReason::LOCAL_DISCONNECT;
            if (!w.endpoint->_trySealLocalTerminal(w.decision))
                return;   // evidence changed after the computation: cancel
        }
        else
        {
            reason = *w.pendingRemoval;
            if (w.endpoint)
                w.endpoint->_sealTerminal();
        }

        const bool keepIntent = reason == RemovalReason::RESPONSE_FAILURE ||
                                reason == RemovalReason::PEER_DISCONNECTED ||
                                reason == RemovalReason::PAIR_MISSING;
        bool neverActive = false;
        {
            std::unique_lock<std::shared_mutex> sl(w.slot->slotMtx);
            if (w.slot->endpoint != w.endpoint ||
                w.slot->attemptGeneration != w.identity.attemptGeneration)
                return;   // a newer generation took over: leave it alone
            w.slot->lastStatus = {ControlSignalState::DISCONNECTED,
                                  w.decision.status.rateHz};
            w.slot->phase = RegistrationPhase::REMOVING;
            if (!keepIntent)
                w.slot->desired = false;
            neverActive = !w.slot->wasActiveSinceEndpoint;
            w.slot->pendingRemoval.reset();
        }
        if (w.endpoint)
        {
            EntityDecision d = w.decision;
            d.status.state = ControlSignalState::DISCONNECTED;
            w.endpoint->_applyStatus(d);   // state callback before removal
            w.endpoint->shutdown();        // outside every lock
        }

        bool retainSlot;
        {
            std::unique_lock<std::shared_mutex> sl(w.slot->slotMtx);
            w.slot->endpoint.reset();
            retainSlot = keepIntent && w.slot->desired;
            w.slot->phase = retainSlot ? RegistrationPhase::RETRY_WAIT
                                       : RegistrationPhase::REMOVING;
            if (retainSlot && neverActive)
                ++w.slot->neverActiveTerminations;
        }
        if (!retainSlot)
        {
            std::unique_lock<std::shared_mutex> lk(sourceMtx_);
            const auto it = sources_.find(w.controller);
            if (it != sources_.end() && it->second == w.slot)
                sources_.erase(it);
        }
        else
        {
            _enqueueRetry(w.slot, w.identity.registrationId,
                          reason == RemovalReason::RESPONSE_FAILURE
                              ? RetryReason::RESPONSE_FAILURE
                              : (reason == RemovalReason::PAIR_MISSING
                                     ? RetryReason::PAIR_MISSING
                                     : RetryReason::PEER_DISCONNECTED),
                          /*everSucceeded=*/true);
        }
        // Every Source terminal sends a matching-generation best-effort
        // UNREGISTER to the target (§8.3); retry may start before its ACK.
        _sendBestEffortUnregister(w.slot->info.target_manager_name, w.controller,
                                  w.identity);

        const EventKind kind =
            reason == RemovalReason::PEER_DISCONNECTED ? EventKind::PEER_DISCONNECTED
            : reason == RemovalReason::PAIR_MISSING    ? EventKind::PAIR_MISSING
                                                       : EventKind::LOCAL_DISCONNECTED;
        _notify(kind, w.controller, RegisterError::OK,
                w.identity.attemptGeneration, "terminal removal");
    }

    void _terminalCommitSink(SinkWork& w, bool localTerminal)
    {
        if (localTerminal)
        {
            if (!w.endpoint->_trySealLocalTerminal(w.decision))
                return;   // a fresh message arrived: cancel this round
        }
        else
        {
            w.endpoint->_sealTerminal();
        }
        {
            std::unique_lock<std::shared_mutex> lk(sinkMtx_);
            const auto it = sinks_.find(w.controller);
            if (it == sinks_.end() || it->second.identity != w.identity ||
                it->second.endpoint != w.endpoint)
                return;
            it->second.lastStatus = {ControlSignalState::DISCONNECTED,
                                     w.decision.status.rateHz};
            it->second.phase = RegistrationPhase::REMOVING;
        }
        EntityDecision d = w.decision;
        d.status.state = ControlSignalState::DISCONNECTED;
        w.endpoint->_applyStatus(d);
        w.endpoint->shutdown();
        {
            std::unique_lock<std::shared_mutex> lk(sinkMtx_);
            const auto it = sinks_.find(w.controller);
            if (it != sinks_.end() && it->second.identity == w.identity)
            {
                completedRemovals_[w.identity.registrationId] =
                    w.identity.attemptGeneration;
                if (completedRemovals_.size() > 4096)
                    completedRemovals_.erase(completedRemovals_.begin());
                sinks_.erase(it);
            }
        }
        _notify(EventKind::LOCAL_DISCONNECTED, w.controller, RegisterError::OK,
                w.identity.attemptGeneration, "sink terminal removal");
    }

    void _tickPublishAndHeartbeat(int64_t now)
    {
        // Phase 4: full snapshot publish (§2.5.1) + master control plane.
        ManagerStatusT msg;
        msg.manager_name = name_;
        msg.csm_instance_id = csmInstanceId_;
        msg.snapshot_seq = ++snapshotSeq_;
        msg.snapshot_ready = firstTickDone_.load();
        msg.stamp = node_->get_clock()->now();
        std::vector<std::shared_ptr<SourceRegistrationSlot>> slotSnap;
        {
            std::shared_lock<std::shared_mutex> lk(sourceMtx_);
            slotSnap.reserve(sources_.size());
            for (const auto& [c, s] : sources_)
                slotSnap.push_back(s);
        }
        {
            for (const auto& s : slotSnap)
            {
                std::shared_lock<std::shared_mutex> sl(s->slotMtx);
                EntryStatusT e;
                e.manager_name = name_;
                e.csm_instance_id = csmInstanceId_;
                e.source_manager_name = name_;
                e.source_csm_instance_id = csmInstanceId_;
                e.target_manager_name = s->info.target_manager_name;
                e.registration_id = s->registrationId;
                e.attempt_generation = s->attemptGeneration;
                e.controller_name = s->info.controller_name;
                e.channel_name = s->info.channel_name;
                e.type = s->info.type;
                e.mode = s->info.mode;
                e.is_source = true;
                e.registration_phase = static_cast<int8_t>(s->phase);
                e.endpoint_present = s->endpoint != nullptr;
                e.state = static_cast<int8_t>(s->lastStatus.state);
                e.data_rate_hz = s->lastStatus.rateHz;
                e.priority = s->info.priority;
                msg.sources.push_back(std::move(e));
            }
        }
        {
            std::shared_lock<std::shared_mutex> lk(sinkMtx_);
            for (const auto& [c, en] : sinks_)
            {
                EntryStatusT e;
                e.manager_name = name_;
                e.csm_instance_id = csmInstanceId_;
                e.source_manager_name = en.sourceManagerName;
                e.source_csm_instance_id = en.identity.sourceCsmInstanceId;
                e.target_manager_name = name_;
                e.registration_id = en.identity.registrationId;
                e.attempt_generation = en.identity.attemptGeneration;
                e.controller_name = c;
                if (en.endpoint)
                {
                    const auto& info = en.endpoint->getInfo();
                    e.channel_name = info.channel_name;
                    e.type = info.type;
                    e.mode = info.mode;
                    e.priority = info.priority;
                }
                e.is_source = false;
                e.registration_phase = static_cast<int8_t>(en.phase);
                e.endpoint_present = en.endpoint != nullptr;
                e.state = static_cast<int8_t>(en.lastStatus.state);
                e.data_rate_hz = en.lastStatus.rateHz;
                msg.sinks.push_back(std::move(e));
            }
        }
        statusPub_->publish(msg);

        // Master registration: single in-flight async request; no blocking.
        if (!masterRegistered_.load() && masterRegClient_->service_is_ready())
        {
            std::lock_guard<std::mutex> lk(heartbeatMtx_);
            if (!regInFlight_)
            {
                regInFlight_ = true;
                auto req = std::make_shared<CsmRegisterSrv::Request>();
                req->csm_name = name_;
                req->csm_instance_id = csmInstanceId_;
                req->csm_timeout_ns = opt_.csmTimeoutNs;
                req->csm_disconnect_timeout_ns = opt_.csmDisconnectTimeoutNs;
                req->status_interval_ns = opt_.statusIntervalMs * 1'000'000;
                req->registration_grace_ns =
                    (opt_.maxRegisterTimeoutMs + 2 * opt_.statusIntervalMs) * 1'000'000;
                masterRegClient_->async_send_request(req,
                    [this, life = life_](rclcpp::Client<CsmRegisterSrv>::SharedFuture f) {
                if (!life->alive.load()) return;
                life->active.fetch_add(1);
                if (!life->alive.load()) { life->active.fetch_sub(1); return; }
                struct Drop { LifeToken* t; ~Drop() { t->active.fetch_sub(1); } }
                    drop{life.get()};
                        detail::CallbackGuard g;
                        std::lock_guard<std::mutex> lk2(heartbeatMtx_);
                        regInFlight_ = false;
                        const auto r = f.get();
                        if (r->response == CsmRegisterSrv::Response::RESPONSE_SUCCESS)
                        {
                            masterRegistered_.store(true);
                            degraded_.store(false);
                        }
                    });
            }
        }

        // Heartbeat: at most one in flight; the slot frees only past its
        // deadline; a late completion carries its generation and must not
        // clear a newer attempt (M17).
        if (masterRegistered_.load())
        {
            std::lock_guard<std::mutex> lk(heartbeatMtx_);
            if (heartbeatInFlight_ && now > heartbeatDeadlineNs_)
            {
                heartbeatInFlight_ = false;   // expired: release + degraded
                degraded_.store(true);
            }
            if (!heartbeatInFlight_)
            {
                heartbeatInFlight_ = true;
                const uint64_t gen = ++heartbeatGeneration_;
                // v1.2.1: the in-flight deadline must stay < csmTimeoutNs/2
                // so one stalled attempt cannot occupy the whole window.
                int64_t deadline = 2 * opt_.statusIntervalMs * 1'000'000;
                if (opt_.csmTimeoutNs > 0)
                    deadline = std::min(deadline, opt_.csmTimeoutNs / 2 - 1);
                heartbeatDeadlineNs_ = now + deadline;
                auto req = std::make_shared<CsmHeartbeatSrv::Request>();
                req->csm_name = name_;
                req->csm_instance_id = csmInstanceId_;
                heartbeatClient_->async_send_request(req,
                    [this, gen, life = life_](rclcpp::Client<CsmHeartbeatSrv>::SharedFuture f) {
                if (!life->alive.load()) return;
                life->active.fetch_add(1);
                if (!life->alive.load()) { life->active.fetch_sub(1); return; }
                struct Drop { LifeToken* t; ~Drop() { t->active.fetch_sub(1); } }
                    drop{life.get()};
                        detail::CallbackGuard g;
                        std::lock_guard<std::mutex> lk2(heartbeatMtx_);
                        if (gen != heartbeatGeneration_)
                            return;   // stale completion: newer attempt owns the slot
                        heartbeatInFlight_ = false;
                        const auto r = f.get();
                        if (r->response == CsmHeartbeatSrv::Response::RESPONSE_SUCCESS)
                        {
                            degraded_.store(false);
                        }
                        else
                        {
                            // STALE_INSTANCE / UNKNOWN_CSM: schedule re-register.
                            masterRegistered_.store(false);
                        }
                    });
            }
        }
    }

    void _tickRetryAndMaintenance(int64_t now)
    {
        // Phase 5a: drain completions written by response callbacks.
        std::vector<RetryCompletion> completions;
        {
            std::lock_guard<std::mutex> lk(retryMtx_);
            completions.assign(retryCompletions_.begin(), retryCompletions_.end());
            retryCompletions_.clear();
        }
        for (auto& c : completions)
            _applyRetryCompletion(c, now);

        // Phase 5b: launch due attempts, bounded by maxInFlight.
        std::vector<std::pair<std::string, std::shared_ptr<SourceRegistrationSlot>>> due;
        std::vector<std::pair<std::string, std::shared_ptr<SourceRegistrationSlot>>> exhausted;
        uint32_t inFlightCount = 0;
        {
            std::lock_guard<std::mutex> lk(retryMtx_);
            for (auto& [rid, pr] : retryTable_)
            {
                if (pr.inFlight && now > pr.inFlightDeadlineNs)
                    pr.inFlight = false;   // lost response: give the slot back
                if (pr.inFlight)
                    ++inFlightCount;
            }
            for (auto it = retryTable_.begin(); it != retryTable_.end();)
            {
                auto& pr = it->second;
                if (pr.inFlight || now < pr.nextTryNs ||
                    inFlightCount + due.size() >= opt_.retryPolicy.maxInFlight)
                {
                    ++it;
                    continue;
                }
                auto slot = pr.slot.lock();
                if (!slot)
                {
                    it = retryTable_.erase(it);
                    continue;
                }
                // D7 initial cap: ONLY the optional never-succeeded
                // initial-unreachable retry is bounded; D3 conflicts and
                // established intents retry until success/unregister/
                // permanent error (§8.3 stage 5). Exhausted entries
                // terminate loudly instead of starving (no zombies).
                if (!pr.everSucceeded &&
                    pr.reason == RetryReason::INITIAL_UNREACHABLE &&
                    pr.attempts >= opt_.retryPolicy.maxInitialAttempts)
                {
                    exhausted.emplace_back(it->first, slot);
                    it = retryTable_.erase(it);
                    continue;
                }
                pr.inFlight = true;
                pr.inFlightDeadlineNs =
                    now + opt_.maxRegisterTimeoutMs * 1'000'000;
                ++pr.attempts;
                due.emplace_back(it->first, slot);
                ++it;
            }
        }
        for (auto& [rid, slot] : exhausted)
        {
            uint64_t gen = 0;
            {
                std::shared_lock<std::shared_mutex> sl(slot->slotMtx);
                gen = slot->attemptGeneration;
            }
            _dropSlot(slot, slot->info.controller_name);
            _notify(EventKind::RETRY_FAILED, slot->info.controller_name,
                    RegisterError::TARGET_UNREACHABLE, gen,
                    "initial retry attempts exhausted (D7 cap)");
        }
        for (auto& [rid, slot] : due)
            _launchRetryAttempt(rid, slot, now);

        // Phase 5c: PENDING TTL (sink side; §2.4).
        {
            std::unique_lock<std::shared_mutex> lk(sinkMtx_);
            for (auto it = sinks_.begin(); it != sinks_.end();)
            {
                if (it->second.phase == RegistrationPhase::PENDING &&
                    !it->second.endpoint &&
                    now - it->second.pendingSinceNs > opt_.pendingTtlMs * 1'000'000)
                    it = sinks_.erase(it);
                else
                    ++it;
            }
        }
    }

    void _applyRetryCompletion(const RetryCompletion& c, int64_t now)
    {
        std::shared_ptr<SourceRegistrationSlot> slot;
        bool everSucceeded = false;
        {
            std::lock_guard<std::mutex> lk(retryMtx_);
            const auto it = retryTable_.find(c.identity.registrationId);
            if (it == retryTable_.end())
                return;   // cancelled: a late success must not revive it (M7)
            if (it->second.inFlightGeneration != c.identity.attemptGeneration)
                return;   // stale completion: a newer attempt owns the slot
            it->second.inFlight = false;
            slot = it->second.slot.lock();
            everSucceeded = it->second.everSucceeded;
            if (!slot)
            {
                retryTable_.erase(it);
                return;
            }
        }

        if (c.result == RegisterError::OK)
        {
            std::string err;
            std::shared_ptr<BaseControlSignalSource> endpoint;
            InfoT info;
            {
                std::shared_lock<std::shared_mutex> sl(slot->slotMtx);
                // Endpoint replacement only while desired and the identity
                // still matches (§8.3 phase 5).
                if (!slot->desired ||
                    slot->attemptGeneration != c.identity.attemptGeneration ||
                    slot->registrationId != c.identity.registrationId)
                    return;
                info = slot->info;
            }
            endpoint = ControlSignalFactory::Instance().CreateSource(
                info.type, node_, info, &err, opt_.rateWindowNs);
            if (!endpoint)
                return;
            bool committed = false;
            {
                std::unique_lock<std::shared_mutex> sl(slot->slotMtx);
                if (slot->desired &&
                    slot->attemptGeneration == c.identity.attemptGeneration)
                {
                    slot->endpoint = endpoint;
                    slot->phase = RegistrationPhase::REGISTERED;
                    slot->wasActiveSinceEndpoint = false;
                    committed = true;
                }
            }
            if (!committed)
            {
                endpoint->shutdown();
                return;
            }
            _installAllStateCbs(endpoint);
            {
                std::lock_guard<std::mutex> lk(retryMtx_);
                retryTable_.erase(c.identity.registrationId);
            }
            _notify(EventKind::RETRY_SUCCEEDED, slot->info.controller_name,
                    RegisterError::OK, c.identity.attemptGeneration, c.reason);
            return;
        }

        const bool permanent = c.result == RegisterError::REJECTED;
        if (permanent)
        {
            {
                std::lock_guard<std::mutex> lk(retryMtx_);
                retryTable_.erase(c.identity.registrationId);
            }
            _dropSlot(slot, slot->info.controller_name);
            _notify(EventKind::RETRY_FAILED, slot->info.controller_name,
                    c.result, c.identity.attemptGeneration, c.reason);
            return;
        }

        // Retryable failure: exponential backoff + jitter; no attempt cap
        // once the intent has ever succeeded (D2-D4). Quarantine backoff
        // kicks in after quarantineThreshold never-active rebuilds (v1.2.1).
        uint32_t quarantined = 0;
        {
            std::shared_lock<std::shared_mutex> sl(slot->slotMtx);
            quarantined = slot->neverActiveTerminations;
        }
        {
            std::lock_guard<std::mutex> lk(retryMtx_);
            const auto it = retryTable_.find(c.identity.registrationId);
            if (it == retryTable_.end())
                return;
            // A typed conflict reclassifies the intent as a D3 conflict:
            // it escapes the initial-unreachable cap and retries until the
            // old generation leaves (§2.4/§8.3 stage 5).
            if (c.result == RegisterError::RETRYABLE_CONFLICT)
                it->second.reason = RetryReason::OLD_GENERATION_CONFLICT;
            const uint32_t attempts = it->second.attempts;
            int64_t delayMs = opt_.retryPolicy.initialDelayMs;
            for (uint32_t i = 1; i < attempts && delayMs < opt_.retryPolicy.maxDelayMs; ++i)
                delayMs *= 2;
            delayMs = std::min(delayMs, opt_.retryPolicy.maxDelayMs);
            if (quarantined >= opt_.retryPolicy.quarantineThreshold)
                delayMs = opt_.retryPolicy.maxDelayMs;   // long backoff (v1.2.1)
            static thread_local std::mt19937 jrng{std::random_device{}()};
            std::uniform_real_distribution<double> dist(
                1.0 - opt_.retryPolicy.jitterRatio,
                1.0 + opt_.retryPolicy.jitterRatio);
            it->second.nextTryNs =
                now + static_cast<int64_t>(delayMs * dist(jrng)) * 1'000'000;
        }
        (void)everSucceeded;
        _notify(EventKind::RETRY_FAILED, slot->info.controller_name, c.result,
                c.identity.attemptGeneration, c.reason);
    }

    void _launchRetryAttempt(const std::string& registrationId,
                             const std::shared_ptr<SourceRegistrationSlot>& slot,
                             int64_t now)
    {
        (void)now;
        RegistrationIdentity identity;
        InfoT info;
        {
            std::unique_lock<std::shared_mutex> sl(slot->slotMtx);
            if (!slot->desired)
                return;
            ++slot->attemptGeneration;   // a new attempt generation (§8.3)
            identity = {csmInstanceId_, registrationId, slot->attemptGeneration};
            info = slot->info;
        }
        {
            // Stamp the generation so late completions of older attempts are
            // recognizably stale (mirrors the heartbeat guard, §8.3 phase 4).
            std::lock_guard<std::mutex> lk(retryMtx_);
            const auto it = retryTable_.find(registrationId);
            if (it != retryTable_.end())
                it->second.inFlightGeneration = identity.attemptGeneration;
        }
        auto req = std::make_shared<ManageSrv::Request>();
        req->op = ManageSrv::Request::OP_REGISTER;
        req->source_manager_name = name_;
        req->source_csm_instance_id = identity.sourceCsmInstanceId;
        req->registration_id = identity.registrationId;
        req->attempt_generation = identity.attemptGeneration;
        req->info = info;
        auto client = _manageClient(info.target_manager_name);
        _notify(EventKind::RETRY_STARTED, info.controller_name, RegisterError::OK,
                identity.attemptGeneration, "");
        if (!client->service_is_ready())
        {
            std::lock_guard<std::mutex> lk(retryMtx_);
            RetryCompletion c{identity, RegisterError::TARGET_UNREACHABLE,
                              "manage service not ready"};
            retryCompletions_.push_back(std::move(c));
            return;
        }
        client->async_send_request(req,
            [this, identity, life = life_](rclcpp::Client<ManageSrv>::SharedFuture f) {
                if (!life->alive.load()) return;
                life->active.fetch_add(1);
                if (!life->alive.load()) { life->active.fetch_sub(1); return; }
                struct Drop { LifeToken* t; ~Drop() { t->active.fetch_sub(1); } }
                    drop{life.get()};
                detail::CallbackGuard g;
                RegisterError result;
                std::string reason;
                const auto r = f.get();
                reason = r->reason;
                switch (r->response)
                {
                    case ManageSrv::Response::RESPONSE_SUCCESS:
                    case ManageSrv::Response::RESPONSE_ALREADY_APPLIED:
                        result = RegisterError::OK;
                        break;
                    case ManageSrv::Response::RESPONSE_RETRYABLE_CONFLICT:
                        result = RegisterError::RETRYABLE_CONFLICT;
                        break;
                    case ManageSrv::Response::RESPONSE_TEMPORARY_UNAVAILABLE:
                        result = RegisterError::TARGET_UNREACHABLE;
                        break;
                    default:
                        result = RegisterError::REJECTED;
                        break;
                }
                // Response callbacks only write the completion queue (§2.6);
                // the next tick commits the lifecycle change.
                std::lock_guard<std::mutex> lk(retryMtx_);
                retryCompletions_.push_back({identity, result, reason});
            });
    }

    // ── members ───────────────────────────────────────────────────────────

    rclcpp::Node* node_;
    const std::string name_;
    const ManagerOptions opt_;
    const std::string csmInstanceId_;

    mutable std::shared_mutex sourceMtx_;
    std::map<std::string, std::shared_ptr<SourceRegistrationSlot>> sources_;
    mutable std::shared_mutex sinkMtx_;
    std::map<std::string, SinkEntry> sinks_;

    std::mutex typedCbMtx_;
    std::map<std::string, BaseControlSignalSink::ErasedMsgCb> typedCbs_;
    mutable std::mutex filterMtx_;
    std::optional<std::set<std::string>> whitelist_;
    std::optional<std::set<std::string>> blacklist_;

    std::mutex stateCbMtx_;
    std::array<StateCb, 4> sourceStateCbs_;
    std::array<StateCb, 4> sinkStateCbs_;

    std::mutex notifyCbMtx_;
    NotificationCb notifyCb_;

    std::mutex retryMtx_;
    std::map<std::string, PendingRegister> retryTable_;
    std::deque<RetryCompletion> retryCompletions_;

    std::mutex eventMtx_;
    std::set<std::string> appliedEvents_;
    std::deque<std::string> appliedOrder_;   // FIFO for dedup-cache eviction
    std::map<std::string, uint64_t> completedRemovals_;   // registrationId -> generation

    std::mutex clientMtx_;
    std::map<std::string, rclcpp::Client<ManageSrv>::SharedPtr> manageClients_;

    rclcpp::CallbackGroup::SharedPtr mgmtGroup_;
    rclcpp::CallbackGroup::SharedPtr tickGroup_;
    rclcpp::Service<ManageSrv>::SharedPtr manageSrv_;
    rclcpp::Service<InfoReqSrv>::SharedPtr infoReqSrv_;
    rclcpp::Service<CsmNotifySrv>::SharedPtr notifySrv_;
    rclcpp::Publisher<ManagerStatusT>::SharedPtr statusPub_;
    rclcpp::Client<CsmRegisterSrv>::SharedPtr masterRegClient_;
    rclcpp::Client<CsmHeartbeatSrv>::SharedPtr heartbeatClient_;
    rclcpp::TimerBase::SharedPtr tickTimer_;

    // Destruction fence: async lambdas capture life_ (shared) and bail out
    // once alive reads false; the destructor stores false first, then drains
    // active so no callback body can outlive member destruction. Flag and
    // counter live in the SAME shared block so a preempted lambda can never
    // touch freed Manager memory.
    struct LifeToken
    {
        std::atomic<bool> alive{true};
        std::atomic<int> active{0};
    };
    std::shared_ptr<LifeToken> life_{std::make_shared<LifeToken>()};

    std::atomic<bool> tickRunning_{false};
    std::atomic<bool> executorObserved_{false};
    std::atomic<bool> firstTickDone_{false};
    std::atomic<bool> masterRegistered_{false};
    std::atomic<bool> degraded_{false};
    uint64_t snapshotSeq_{0};   // tick thread only

    std::mutex heartbeatMtx_;
    bool regInFlight_{false};
    bool heartbeatInFlight_{false};
    uint64_t heartbeatGeneration_{0};
    int64_t heartbeatDeadlineNs_{0};
};

// ── out-of-line helpers needing complete Manager type ─────────────────────

inline void ControlSignalManager::_installSourceStateCb(
    const std::shared_ptr<BaseControlSignalSource>& ep, ControlSignalState state,
    const StateCb& cb)
{
    ep->_setStateCallbackErased(state, cb);
}

inline void ControlSignalManager::_installSinkStateCb(
    const std::shared_ptr<BaseControlSignalSink>& ep, ControlSignalState state,
    const StateCb& cb)
{
    ep->_setStateCallbackErased(state, cb);
}

inline void ControlSignalManager::_installAllStateCbs(
    const std::shared_ptr<BaseControlSignalSource>& ep)
{
    std::lock_guard<std::mutex> lk(stateCbMtx_);
    for (size_t i = 0; i < sourceStateCbs_.size(); ++i)
        if (sourceStateCbs_[i])
            ep->_setStateCallbackErased(static_cast<ControlSignalState>(i),
                                        sourceStateCbs_[i]);
}

inline void ControlSignalManager::_installAllStateCbs(
    const std::shared_ptr<BaseControlSignalSink>& ep)
{
    std::lock_guard<std::mutex> lk(stateCbMtx_);
    for (size_t i = 0; i < sinkStateCbs_.size(); ++i)
        if (sinkStateCbs_[i])
            ep->_setStateCallbackErased(static_cast<ControlSignalState>(i),
                                        sinkStateCbs_[i]);
}

} // namespace r1
} // namespace rv2_interfaces

#endif // RV2_CONTROL_SIGNAL_TRANSPORT_R1_CONTROL_SIGNAL_MANAGER_H
