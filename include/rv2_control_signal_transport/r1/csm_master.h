/**
 * @file csm_master.h
 * @brief r1::CsmMaster — centralized notification architecture (design §9):
 *        sole status subscriber, pairing + reconciliation, CSM-level
 *        dual-threshold polling, reliable control notifications.
 *
 * Lock rules (§9.3): shared csmMtx_ locks only immutable queries; heartbeat,
 * snapshot replacement, reconciliation levels and pending-event mutation use
 * the unique lock. Inside locks only immutable work items are computed; ROS
 * calls, logging and response handling happen after release.
 */

#ifndef RV2_CONTROL_SIGNAL_TRANSPORT_R1_CSM_MASTER_H
#define RV2_CONTROL_SIGNAL_TRANSPORT_R1_CSM_MASTER_H

#include <algorithm>
#include <atomic>
#include <cstdint>
#include <deque>
#include <map>
#include <memory>
#include <mutex>
#include <optional>
#include <random>
#include <set>
#include <shared_mutex>
#include <sstream>
#include <string>
#include <thread>
#include <tuple>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include <r1_interfaces/msg/entry_status.hpp>
#include <r1_interfaces/msg/manager_status.hpp>
#include <r1_interfaces/srv/csm_heartbeat.hpp>
#include <r1_interfaces/srv/csm_notify.hpp>
#include <r1_interfaces/srv/csm_register.hpp>

#include "rv2_control_signal_transport/r1/control_signal_info.h"
#include "rv2_control_signal_transport/r1/source_registration.h"

namespace rv2_interfaces
{
namespace r1
{

/// Identity of one logical pair; the target incarnation is part of the
/// reconciliation key so a target restart always resets the grace (§9.1).
struct PairIdentity
{
    RegistrationIdentity registration;
    std::string controllerName;
    std::string sourceCsmName;
    std::string targetCsmName;
    std::string targetCsmInstanceId;

    bool operator<(const PairIdentity& o) const
    {
        return std::tie(registration.sourceCsmInstanceId,
                        registration.registrationId,
                        registration.attemptGeneration, controllerName,
                        sourceCsmName, targetCsmName, targetCsmInstanceId) <
               std::tie(o.registration.sourceCsmInstanceId,
                        o.registration.registrationId,
                        o.registration.attemptGeneration, o.controllerName,
                        o.sourceCsmName, o.targetCsmName, o.targetCsmInstanceId);
    }
    bool operator==(const PairIdentity& o) const
    {
        return !(*this < o) && !(o < *this);
    }
};

struct NotificationRetryPolicy
{
    NotificationRetryPolicy(int64_t initialDelayMs_, int64_t maxDelayMs_,
                            double jitterRatio_, uint32_t maxInFlight_)
        : initialDelayMs(initialDelayMs_)
        , maxDelayMs(maxDelayMs_)
        , jitterRatio(jitterRatio_)
        , maxInFlight(maxInFlight_)
    {}
    int64_t initialDelayMs;
    int64_t maxDelayMs;
    double jitterRatio;
    uint32_t maxInFlight;   // correctness-critical events have NO attempt cap
};

struct MasterOptions
{
    explicit MasterOptions(NotificationRetryPolicy retry)
        : notificationRetry(std::move(retry))
    {}
    std::string masterName    = "csm_master";
    int64_t tickIntervalMs    = 200;
    int64_t pairGraceMs       = 1000;   // minimum grace; PENDING suspends the clock
    NotificationRetryPolicy notificationRetry;
};

class CsmMaster
{
public:
    using EntryStatusT = r1_interfaces::msg::EntryStatus;
    using ManagerStatusT = r1_interfaces::msg::ManagerStatus;
    using CsmRegisterSrv = r1_interfaces::srv::CsmRegister;
    using CsmHeartbeatSrv = r1_interfaces::srv::CsmHeartbeat;
    using CsmNotifySrv = r1_interfaces::srv::CsmNotify;

    CsmMaster(rclcpp::Node* node, const MasterOptions& opt)
        : node_(node), opt_(opt)
    {
        group_ = node_->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        regSrv_ = node_->create_service<CsmRegisterSrv>(
            opt_.masterName + "/register",
            [this, life = life_](const std::shared_ptr<CsmRegisterSrv::Request> rq,
                                 std::shared_ptr<CsmRegisterSrv::Response> rs) {
                if (!life->alive.load()) return;
                life->active.fetch_add(1);
                if (!life->alive.load()) { life->active.fetch_sub(1); return; }
                struct Drop { LifeToken* t; ~Drop() { t->active.fetch_sub(1); } }
                    drop{life.get()};
                _onRegister(*rq, *rs);
            },
            rclcpp::ServicesQoS(), group_);
        hbSrv_ = node_->create_service<CsmHeartbeatSrv>(
            opt_.masterName + "/heartbeat",
            [this, life = life_](const std::shared_ptr<CsmHeartbeatSrv::Request> rq,
                                 std::shared_ptr<CsmHeartbeatSrv::Response> rs) {
                if (!life->alive.load()) return;
                life->active.fetch_add(1);
                if (!life->alive.load()) { life->active.fetch_sub(1); return; }
                struct Drop { LifeToken* t; ~Drop() { t->active.fetch_sub(1); } }
                    drop{life.get()};
                _onHeartbeat(*rq, *rs);
            },
            rclcpp::ServicesQoS(), group_);
        tick_ = node_->create_wall_timer(
            std::chrono::milliseconds(opt_.tickIntervalMs),
            [this, life = life_] {
                if (!life->alive.load()) return;
                life->active.fetch_add(1);
                if (!life->alive.load()) { life->active.fetch_sub(1); return; }
                struct Drop { LifeToken* t; ~Drop() { t->active.fetch_sub(1); } }
                    drop{life.get()};
                _tick();
            },
            group_);
    }

    CsmMaster(const CsmMaster&) = delete;
    CsmMaster& operator=(const CsmMaster&) = delete;

    ~CsmMaster()
    {
        life_->alive.store(false);
        tick_->cancel();
        while (life_->active.load() != 0)
            std::this_thread::yield();
    }

    /// CSM-name white/blacklist, mirroring the CSM's controller filters (§9.1).
    void enableCsmWhitelist(const std::vector<std::string>& names)
    {
        std::lock_guard<std::mutex> lk(filterMtx_);
        whitelist_.emplace(names.begin(), names.end());
    }
    void disableCsmWhitelist()
    {
        std::lock_guard<std::mutex> lk(filterMtx_);
        whitelist_.reset();
    }
    void enableCsmBlacklist(const std::vector<std::string>& names)
    {
        std::lock_guard<std::mutex> lk(filterMtx_);
        blacklist_.emplace(names.begin(), names.end());
    }
    void disableCsmBlacklist()
    {
        std::lock_guard<std::mutex> lk(filterMtx_);
        blacklist_.reset();
    }

    bool hasCsm(const std::string& name) const
    {
        std::shared_lock<std::shared_mutex> lk(csmMtx_);
        return csms_.count(name) != 0;
    }

private:
    enum class CsmHealth : uint8_t { INITIAL, ACTIVE, TIMEOUT, DISCONNECTED };

    /// CSM health recovers from TIMEOUT via a same-incarnation heartbeat, so
    /// the entity terminal FSM (§4) is deliberately NOT reused (§9.2).
    struct HeartbeatTracker
    {
        int64_t lastSeenNs{0};
        CsmHealth state{CsmHealth::INITIAL};
    };

    struct IdentityKey
    {
        std::string sourceCsmInstanceId;
        std::string registrationId;
        uint64_t attemptGeneration;
        bool isSource;
        bool operator<(const IdentityKey& o) const
        {
            return std::tie(sourceCsmInstanceId, registrationId,
                            attemptGeneration, isSource) <
                   std::tie(o.sourceCsmInstanceId, o.registrationId,
                            o.attemptGeneration, o.isSource);
        }
    };

    struct CsmRecord
    {
        rclcpp::Subscription<ManagerStatusT>::SharedPtr statusSub;
        rclcpp::Client<CsmNotifySrv>::SharedPtr notifyCli;
        std::string instanceId;
        HeartbeatTracker heartbeat;
        int64_t csmTimeoutNs{0};
        int64_t csmDisconnectTimeoutNs{0};
        int64_t statusIntervalNs{0};
        int64_t registrationGraceNs{0};
        uint64_t lastSnapshotSeq{0};
        bool snapshotReady{false};
        std::set<std::string> retiredInstanceIds;
        std::map<IdentityKey, EntryStatusT> entries;   // latest full snapshot
    };

    enum class PendingKind : uint8_t { CSM_TIMEOUT, DISCONNECTED, PAIR_MISSING };

    struct PendingNotification
    {
        std::string eventId;
        PendingKind kind;
        int8_t peerHealth{0};
        std::string targetCsmName;
        std::string targetInstanceId;
        std::vector<EntryStatusT> entries;
        std::string ownerCsm;   // whose health/pairing produced this event
        uint32_t attempts{0};
        int64_t nextTryNs{0};
        bool inFlight{false};
        int64_t inFlightDeadlineNs{0};
        bool acknowledged{false};   // level event queued at receiver, awaiting proof
        int64_t acknowledgedNs{0};
        std::optional<PairIdentity> pair;   // set for PAIR_MISSING levels
    };

    struct NotifyCompletion
    {
        std::string eventId;
        int8_t response;
        bool transportOk;
    };

    static int64_t _steadyNowNs()
    {
        return std::chrono::duration_cast<std::chrono::nanoseconds>(
                   std::chrono::steady_clock::now().time_since_epoch())
            .count();
    }

    static std::string _uuid()
    {
        static thread_local std::mt19937_64 rng{std::random_device{}()};
        std::ostringstream os;
        os << std::hex << rng() << rng();
        return os.str();
    }

    bool _allowed(const std::string& csmName) const
    {
        std::lock_guard<std::mutex> lk(filterMtx_);
        if (whitelist_ && whitelist_->count(csmName) == 0)
            return false;
        if (blacklist_ && blacklist_->count(csmName) != 0)
            return false;
        return true;
    }

    // ── registration (§9.3) ───────────────────────────────────────────────

    void _onRegister(const CsmRegisterSrv::Request& rq,
                     CsmRegisterSrv::Response& rs)
    {
        if (!_allowed(rq.csm_name))
        {
            rs.response = CsmRegisterSrv::Response::RESPONSE_INVALID_CONFIG;
            rs.reason = "csm filtered by white/blacklist";
            return;
        }
        if (rq.csm_name.empty() || rq.csm_instance_id.empty() ||
            rq.csm_timeout_ns < 0 || rq.csm_disconnect_timeout_ns < 0 ||
            (rq.csm_timeout_ns > 0 && rq.csm_disconnect_timeout_ns > 0 &&
             rq.csm_disconnect_timeout_ns <= rq.csm_timeout_ns) ||
            rq.status_interval_ns <= 0 ||
            rq.registration_grace_ns < 2 * rq.status_interval_ns)
        {
            rs.response = CsmRegisterSrv::Response::RESPONSE_INVALID_CONFIG;
            rs.reason = "threshold validation failed (§2.5.2)";
            return;
        }

        // Build transport endpoints first (outside the lock), commit after.
        auto sub = node_->create_subscription<ManagerStatusT>(
            rq.csm_name + "/status", rclcpp::QoS(10),
            [this, life = life_, csm = rq.csm_name](const ManagerStatusT& m) {
                if (!life->alive.load()) return;
                life->active.fetch_add(1);
                if (!life->alive.load()) { life->active.fetch_sub(1); return; }
                struct Drop { LifeToken* t; ~Drop() { t->active.fetch_sub(1); } }
                    drop{life.get()};
                _onStatus(csm, m);
            });
        rclcpp::SubscriptionOptions subOpt;
        auto cli = node_->create_client<CsmNotifySrv>(
            rq.csm_name + "/get_notifications", rclcpp::ServicesQoS(), group_);
        (void)subOpt;

        {
            std::unique_lock<std::shared_mutex> lk(csmMtx_);
            const auto found = csms_.find(rq.csm_name);
            if (found != csms_.end() &&
                found->second.retiredInstanceIds.count(rq.csm_instance_id))
            {
                rs.response = CsmRegisterSrv::Response::RESPONSE_STALE_INSTANCE;
                rs.reason = "instance already retired";
                return;
            }
            auto& rec = csms_[rq.csm_name];
            const bool sameInstance = rec.instanceId == rq.csm_instance_id;
            if (sameInstance &&
                rec.heartbeat.state == CsmHealth::DISCONNECTED)
            {
                // v1.2.1: explicit re-register of a same-instance record that
                // was declared dead = full rebuild, not an idempotent no-op.
                // The snapshot_seq fence is NOT reset: the incarnation (and
                // its strictly-increasing numbering) continues.
                rec.entries.clear();
                rec.snapshotReady = false;
                rec.heartbeat = {_steadyNowNs(), CsmHealth::INITIAL};
            }
            else if (sameInstance)
            {
                // Idempotent update: refresh thresholds only.
            }
            else
            {
                if (!rec.instanceId.empty())
                {
                    rec.retiredInstanceIds.insert(rec.instanceId);
                    _dropEventsForInstanceLocked(rq.csm_name, rec.instanceId);
                }
                rec.instanceId = rq.csm_instance_id;
                rec.entries.clear();
                rec.lastSnapshotSeq = 0;
                rec.snapshotReady = false;
                rec.heartbeat = {_steadyNowNs(), CsmHealth::INITIAL};
            }
            rec.csmTimeoutNs = rq.csm_timeout_ns;
            rec.csmDisconnectTimeoutNs = rq.csm_disconnect_timeout_ns;
            rec.statusIntervalNs = rq.status_interval_ns;
            rec.registrationGraceNs = rq.registration_grace_ns;
            if (!rec.statusSub)
                rec.statusSub = sub;
            if (!rec.notifyCli)
                rec.notifyCli = cli;
        }
        rs.response = CsmRegisterSrv::Response::RESPONSE_SUCCESS;
        rs.reason = "";
    }

    /// Invalidate pending events aimed at a replaced incarnation (locked).
    void _dropEventsForInstanceLocked(const std::string& csmName,
                                      const std::string& instanceId)
    {
        for (auto it = pendingNotifications_.begin();
             it != pendingNotifications_.end();)
        {
            if (it->second.targetCsmName == csmName &&
                it->second.targetInstanceId == instanceId)
                it = pendingNotifications_.erase(it);
            else
                ++it;
        }
    }

    void _onHeartbeat(const CsmHeartbeatSrv::Request& rq,
                      CsmHeartbeatSrv::Response& rs)
    {
        std::unique_lock<std::shared_mutex> lk(csmMtx_);
        const auto it = csms_.find(rq.csm_name);
        if (it == csms_.end() || it->second.instanceId.empty())
        {
            rs.response = CsmHeartbeatSrv::Response::RESPONSE_UNKNOWN_CSM;
            rs.reason = "not registered";
            return;
        }
        auto& rec = it->second;
        if (rec.instanceId != rq.csm_instance_id ||
            rec.retiredInstanceIds.count(rq.csm_instance_id))
        {
            rs.response = CsmHeartbeatSrv::Response::RESPONSE_STALE_INSTANCE;
            rs.reason = "old incarnation";
            return;
        }
        if (rec.heartbeat.state == CsmHealth::DISCONNECTED)
        {
            // v1.2.1: a record declared dead only revives through an explicit
            // re-register; matching-instance heartbeats answer STALE_INSTANCE.
            rs.response = CsmHeartbeatSrv::Response::RESPONSE_STALE_INSTANCE;
            rs.reason = "record disconnected; re-register required";
            return;
        }
        rec.heartbeat.lastSeenNs = _steadyNowNs();
        rs.response = CsmHeartbeatSrv::Response::RESPONSE_SUCCESS;
        rs.reason = "";
    }

    // ── status intake (§9.3) ──────────────────────────────────────────────

    void _onStatus(const std::string& csmName, const ManagerStatusT& msg)
    {
        struct StateEdge
        {
            std::string toCsm;          // notification target CSM name
            std::string toInstance;
            EntryStatusT entry;
        };
        std::vector<StateEdge> edges;

        {
            std::unique_lock<std::shared_mutex> lk(csmMtx_);
            const auto it = csms_.find(csmName);
            if (it == csms_.end())
                return;
            auto& rec = it->second;
            if (msg.manager_name != csmName ||
                msg.csm_instance_id != rec.instanceId ||
                rec.retiredInstanceIds.count(msg.csm_instance_id))
                return;   // old instance: drop whole snapshot
            if (rec.heartbeat.state == CsmHealth::DISCONNECTED)
                return;   // v1.2.1: stale data must not revive a dead record;
                          // only an explicit register performs the rebuild
            if (!msg.snapshot_ready)
                return;   // not a valid baseline yet (§2.5.1)
            // The seq fence spans the whole incarnation (§2.5.1) — it also
            // survives a same-instance full rebuild, so replayed pre-rebuild
            // snapshots can never become the new baseline.
            if (msg.snapshot_seq <= rec.lastSnapshotSeq)
                return;   // duplicate / out-of-order: drop whole snapshot

            std::map<IdentityKey, EntryStatusT> fresh;
            for (const auto& e : msg.sources)
                fresh.emplace(IdentityKey{e.source_csm_instance_id,
                                          e.registration_id,
                                          e.attempt_generation, true},
                              e);
            for (const auto& e : msg.sinks)
                fresh.emplace(IdentityKey{e.source_csm_instance_id,
                                          e.registration_id,
                                          e.attempt_generation, false},
                              e);

            const bool baseline = !rec.snapshotReady;
            // Edge detection old vs new; absent-but-was-REGISTERED entries
            // synthesize a STATE(DISCONNECTED) observation tombstone.
            if (!baseline)
            {
                for (const auto& [key, oldE] : rec.entries)
                {
                    const auto f = fresh.find(key);
                    if (f == fresh.end())
                    {
                        if (oldE.registration_phase == EntryStatusT::PHASE_REGISTERED)
                        {
                            EntryStatusT tomb = oldE;
                            tomb.state = EntryStatusT::STATE_DISCONNECTED;
                            tomb.endpoint_present = false;
                            _appendPairEdgesLocked(csmName, tomb, edges);
                        }
                    }
                    else if (f->second.state != oldE.state)
                    {
                        _appendPairEdgesLocked(csmName, f->second, edges);
                    }
                }
                for (const auto& [key, newE] : fresh)
                    if (!rec.entries.count(key) &&
                        newE.state != EntryStatusT::STATE_INITIAL)
                        _appendPairEdgesLocked(csmName, newE, edges);
            }
            rec.entries = std::move(fresh);   // atomic full replacement
            rec.lastSnapshotSeq = msg.snapshot_seq;
            rec.snapshotReady = true;

            // Track referenced-but-unregistered names (absence clock input,
            // v1.2.1); registered names clear their clock.
            const int64_t now = _steadyNowNs();
            for (const auto& [key, e] : rec.entries)
            {
                for (const std::string& n :
                     {e.source_manager_name, e.target_manager_name})
                {
                    if (n.empty())
                        continue;
                    if (csms_.count(n))
                        absenceSince_.erase(n);
                    else
                        absenceSince_.emplace(n, now);
                }
            }
        }

        // Best-effort one-shot STATE notifications, outside every lock.
        for (const auto& e : edges)
            _sendStateOneShot(e.toCsm, e.toInstance, e.entry);
    }

    /// Adds STATE edge notifications for both members of the entry's pair
    /// (locked; reads other records immutably).
    template<typename EdgeVec>
    void _appendPairEdgesLocked(const std::string& ownerCsm,
                                const EntryStatusT& entry, EdgeVec& edges)
    {
        // The owner itself observes the change...
        const auto ownIt = csms_.find(ownerCsm);
        if (ownIt != csms_.end())
            edges.push_back({ownerCsm, ownIt->second.instanceId, entry});
        // ...and so does the paired CSM on the other side (CM3).
        const std::string peer = entry.is_source ? entry.target_manager_name
                                                 : entry.source_manager_name;
        if (peer.empty() || peer == ownerCsm)
            return;
        const auto peerIt = csms_.find(peer);
        if (peerIt == csms_.end())
            return;
        edges.push_back({peer, peerIt->second.instanceId, entry});
    }

    void _sendStateOneShot(const std::string& csmName,
                           const std::string& instanceId,
                           const EntryStatusT& entry)
    {
        rclcpp::Client<CsmNotifySrv>::SharedPtr cli;
        {
            std::shared_lock<std::shared_mutex> lk(csmMtx_);
            const auto it = csms_.find(csmName);
            if (it == csms_.end() || it->second.instanceId != instanceId)
                return;
            cli = it->second.notifyCli;
        }
        if (!cli || !cli->service_is_ready())
            return;   // STATE is best-effort: may be lost (CM9)
        auto rq = std::make_shared<CsmNotifySrv::Request>();
        rq->kind = CsmNotifySrv::Request::KIND_STATE;
        rq->event_id = _uuid();
        rq->target_csm_instance_id = instanceId;
        rq->entries = {entry};
        cli->async_send_request(rq,
            [](rclcpp::Client<CsmNotifySrv>::SharedFuture) {});
    }

    // ── tick: polling + reconciliation + notification pump (§9.3) ─────────

    void _tick()
    {
        const int64_t now = _steadyNowNs();
        _drainCompletions(now);
        _pollHeartbeats(now);
        _reconcile(now);
        _pumpNotifications(now);
    }

    void _pollHeartbeats(int64_t now)
    {
        struct HealthEdge
        {
            PendingKind kind;
            int8_t peerHealth;
            std::string owner;   // the CSM whose health changed
        };
        std::vector<HealthEdge> healthEdges;
        {
            std::unique_lock<std::shared_mutex> lk(csmMtx_);
            for (auto& [name, rec] : csms_)
            {
                if (rec.instanceId.empty() ||
                    rec.heartbeat.state == CsmHealth::DISCONNECTED)
                    continue;
                const int64_t elapsed = now - rec.heartbeat.lastSeenNs;
                // Strict >; disconnect judged first, so a single scan may go
                // straight to DISCONNECTED without a phantom TIMEOUT edge.
                if (rec.csmDisconnectTimeoutNs > 0 &&
                    elapsed > rec.csmDisconnectTimeoutNs)
                {
                    rec.heartbeat.state = CsmHealth::DISCONNECTED;
                    healthEdges.push_back({PendingKind::DISCONNECTED, 0, name});
                }
                else if (rec.csmTimeoutNs > 0 && elapsed > rec.csmTimeoutNs)
                {
                    if (rec.heartbeat.state != CsmHealth::TIMEOUT)
                    {
                        rec.heartbeat.state = CsmHealth::TIMEOUT;
                        healthEdges.push_back(
                            {PendingKind::CSM_TIMEOUT,
                             CsmNotifySrv::Request::PEER_HEALTH_TIMEOUT, name});
                    }
                }
                else if (rec.heartbeat.state == CsmHealth::TIMEOUT)
                {
                    rec.heartbeat.state = CsmHealth::ACTIVE;
                    healthEdges.push_back(
                        {PendingKind::CSM_TIMEOUT,
                         CsmNotifySrv::Request::PEER_HEALTH_ACTIVE, name});
                }
                else if (rec.heartbeat.state == CsmHealth::INITIAL)
                {
                    rec.heartbeat.state = CsmHealth::ACTIVE;   // silent edge
                }
            }
            for (const auto& e : healthEdges)
                _queueHealthEventsLocked(e.owner, e.kind, e.peerHealth, now);
        }
    }

    /// For every entry of the affected CSM, queue a reliable event to the
    /// paired peer (locked). Latest level replaces an older pending event
    /// for the same target+kind (§9.3).
    void _queueHealthEventsLocked(const std::string& owner, PendingKind kind,
                                  int8_t peerHealth, int64_t now)
    {
        const auto ownIt = csms_.find(owner);
        if (ownIt == csms_.end())
            return;
        // Group affected entries per peer CSM.
        std::map<std::string, std::vector<EntryStatusT>> perPeer;
        for (const auto& [key, e] : ownIt->second.entries)
        {
            const std::string peer = e.is_source ? e.target_manager_name
                                                 : e.source_manager_name;
            if (peer.empty() || peer == owner || !csms_.count(peer))
                continue;
            perPeer[peer].push_back(e);
        }
        for (auto& [peer, entries] : perPeer)
        {
            const auto& peerRec = csms_.at(peer);
            if (peerRec.instanceId.empty())
                continue;
            // Latest level replaces an older pending of the same class FROM
            // THE SAME owner only — another CSM's undelivered reliable event
            // toward the same peer must survive (§9.3).
            for (auto it = pendingNotifications_.begin();
                 it != pendingNotifications_.end();)
            {
                if (it->second.targetCsmName == peer &&
                    it->second.kind == kind &&
                    it->second.ownerCsm == owner &&
                    !it->second.pair.has_value())
                    it = pendingNotifications_.erase(it);
                else
                    ++it;
            }
            PendingNotification pn;
            pn.eventId = _uuid();
            pn.kind = kind;
            pn.peerHealth = peerHealth;
            pn.targetCsmName = peer;
            pn.targetInstanceId = peerRec.instanceId;
            pn.entries = std::move(entries);
            pn.ownerCsm = owner;
            pn.nextTryNs = now;
            pendingNotifications_.emplace(pn.eventId, std::move(pn));
        }
    }

    void _reconcile(int64_t now)
    {
        struct NewLevel
        {
            PairIdentity pair;
            std::string liveCsm;
            std::string liveInstance;
            EntryStatusT entry;
        };
        std::vector<NewLevel> newLevels;
        {
            std::unique_lock<std::shared_mutex> lk(csmMtx_);

            // Absence clocks for referenced-but-unregistered names (v1.2.1):
            // beyond grace their pairs skip the ready-snapshot gate.
            std::set<std::string> absenceExpired;
            for (const auto& [name, since] : absenceSince_)
            {
                if (csms_.count(name))
                    continue;
                if (now - since > opt_.pairGraceMs * 1'000'000)
                    absenceExpired.insert(name);
            }

            // Pairing sweep across every ready record's entries.
            std::set<PairIdentity> pairedNow;
            std::set<PairIdentity> pendingNow;
            std::map<PairIdentity, std::pair<std::string, EntryStatusT>> single;
            for (const auto& [name, rec] : csms_)
            {
                if (rec.instanceId.empty() || !rec.snapshotReady ||
                    rec.heartbeat.state == CsmHealth::DISCONNECTED)
                    continue;
                for (const auto& [key, e] : rec.entries)
                {
                    PairIdentity pid;
                    pid.registration = {e.source_csm_instance_id,
                                        e.registration_id, e.attempt_generation};
                    pid.controllerName = e.controller_name;
                    pid.sourceCsmName = e.source_manager_name;
                    pid.targetCsmName = e.target_manager_name;
                    // Target incarnation from the target record when known.
                    const auto tIt = csms_.find(e.target_manager_name);
                    pid.targetCsmInstanceId =
                        tIt != csms_.end() ? tIt->second.instanceId : "";

                    if (e.registration_phase == EntryStatusT::PHASE_PENDING)
                    {
                        pendingNow.insert(pid);   // suspends the grace clock
                        continue;
                    }
                    const bool live =
                        e.registration_phase == EntryStatusT::PHASE_REGISTERED &&
                        e.endpoint_present;
                    if (!live)
                        continue;   // RETRY_WAIT is an intent, not an endpoint
                    const auto sIt = single.find(pid);
                    if (sIt == single.end())
                        single.emplace(pid, std::make_pair(name, e));
                    else if (sIt->second.second.is_source != e.is_source)
                    {
                        pairedNow.insert(pid);
                        single.erase(sIt);
                    }
                }
            }

            // Paired or pending: reset clocks and clear matching levels.
            for (auto it = unpairedSinceNs_.begin();
                 it != unpairedSinceNs_.end();)
            {
                if (pairedNow.count(it->first) || pendingNow.count(it->first) ||
                    !single.count(it->first))
                    it = unpairedSinceNs_.erase(it);
                else
                    ++it;
            }
            for (auto it = pendingNotifications_.begin();
                 it != pendingNotifications_.end();)
            {
                if (it->second.pair &&
                    (pairedNow.count(*it->second.pair) ||
                     !single.count(*it->second.pair)))
                    it = pendingNotifications_.erase(it);   // condition gone
                else
                    ++it;
            }

            // Singles: gate on both sides' ready snapshots unless the absent
            // side's absence clock expired; then run the grace clock.
            for (const auto& [pid, ownerEntry] : single)
            {
                if (pendingNow.count(pid))
                    continue;
                const std::string& owner = ownerEntry.first;
                const EntryStatusT& e = ownerEntry.second;
                const std::string other = e.is_source ? pid.targetCsmName
                                                      : pid.sourceCsmName;
                const auto oIt = csms_.find(other);
                const bool otherDead =
                    oIt != csms_.end() &&
                    oIt->second.heartbeat.state == CsmHealth::DISCONNECTED;
                const bool otherReady =
                    oIt != csms_.end() && !oIt->second.instanceId.empty() &&
                    oIt->second.snapshotReady && !otherDead;
                // A DISCONNECTED peer that never re-registers must not hold
                // the gate closed forever — the grace clock runs.
                if (!otherReady && !otherDead && oIt != csms_.end() &&
                    !absenceExpired.count(other))
                    continue;   // ready gate holds while the peer is coming up
                if (oIt == csms_.end() && !absenceExpired.count(other))
                    continue;   // absence clock still running

                int64_t graceNs = opt_.pairGraceMs * 1'000'000;
                const auto rIt = csms_.find(owner);
                if (rIt != csms_.end())
                    graceNs = std::max(
                        {graceNs, 2 * rIt->second.statusIntervalNs,
                         rIt->second.registrationGraceNs});
                if (oIt != csms_.end())
                    graceNs = std::max(
                        {graceNs, 2 * oIt->second.statusIntervalNs,
                         oIt->second.registrationGraceNs});

                const auto uIt = unpairedSinceNs_.emplace(pid, now).first;
                if (now - uIt->second <= graceNs)
                    continue;

                // Level exists already? keep it (same event id).
                bool exists = false;
                for (const auto& [id, pn] : pendingNotifications_)
                    if (pn.pair && *pn.pair == pid)
                    {
                        exists = true;
                        break;
                    }
                if (exists)
                    continue;
                newLevels.push_back({pid, owner,
                                     rIt != csms_.end() ? rIt->second.instanceId
                                                        : "",
                                     e});
            }

            for (auto& lv : newLevels)
            {
                PendingNotification pn;
                pn.eventId = _uuid();
                pn.kind = PendingKind::PAIR_MISSING;
                pn.targetCsmName = lv.liveCsm;
                pn.targetInstanceId = lv.liveInstance;
                pn.entries = {lv.entry};
                pn.nextTryNs = now;
                pn.pair = lv.pair;
                pendingNotifications_.emplace(pn.eventId, std::move(pn));
            }
        }
    }

    void _pumpNotifications(int64_t now)
    {
        struct Launch
        {
            std::string eventId;
            PendingKind kind;
            int8_t peerHealth;
            std::string target;
            std::string targetInstance;
            std::vector<EntryStatusT> entries;
        };
        std::vector<Launch> launches;
        {
            std::unique_lock<std::shared_mutex> lk(csmMtx_);
            uint32_t inFlight = 0;
            for (const auto& [id, pn] : pendingNotifications_)
                if (pn.inFlight)
                    ++inFlight;
            for (auto& [id, pn] : pendingNotifications_)
            {
                if (pn.inFlight && now > pn.inFlightDeadlineNs)
                {
                    pn.inFlight = false;   // lost response: release the slot
                    if (inFlight > 0)
                        --inFlight;
                }
                if (inFlight + launches.size() >=
                    opt_.notificationRetry.maxInFlight)
                    break;
                if (pn.inFlight || now < pn.nextTryNs)
                    continue;
                if (pn.acknowledged)
                {
                    // delivered-awaiting-observation (PAIR_MISSING): resend
                    // only if the condition persists past the apply grace.
                    if (now - pn.acknowledgedNs <=
                        opt_.pairGraceMs * 1'000'000)
                        continue;
                    pn.acknowledged = false;
                }
                const auto rIt = csms_.find(pn.targetCsmName);
                if (rIt == csms_.end() ||
                    rIt->second.instanceId != pn.targetInstanceId)
                    continue;   // fixed target incarnation: never re-aimed
                pn.inFlight = true;
                pn.inFlightDeadlineNs =
                    now + opt_.notificationRetry.maxDelayMs * 1'000'000;
                ++pn.attempts;
                launches.push_back({id, pn.kind, pn.peerHealth,
                                    pn.targetCsmName, pn.targetInstanceId,
                                    pn.entries});
            }
        }
        for (auto& l : launches)
            _launchNotification(l.eventId, l.kind, l.peerHealth, l.target,
                                l.targetInstance, l.entries);
    }

    void _launchNotification(const std::string& eventId, PendingKind kind,
                             int8_t peerHealth, const std::string& target,
                             const std::string& targetInstance,
                             const std::vector<EntryStatusT>& entries)
    {
        rclcpp::Client<CsmNotifySrv>::SharedPtr cli;
        {
            std::shared_lock<std::shared_mutex> lk(csmMtx_);
            const auto it = csms_.find(target);
            if (it != csms_.end())
                cli = it->second.notifyCli;
        }
        if (!cli || !cli->service_is_ready())
        {
            std::lock_guard<std::mutex> lk(completionMtx_);
            completions_.push_back({eventId, 0, false});
            return;
        }
        auto rq = std::make_shared<CsmNotifySrv::Request>();
        switch (kind)
        {
            case PendingKind::CSM_TIMEOUT:
                rq->kind = CsmNotifySrv::Request::KIND_CSM_TIMEOUT;
                break;
            case PendingKind::DISCONNECTED:
                rq->kind = CsmNotifySrv::Request::KIND_DISCONNECTED;
                break;
            case PendingKind::PAIR_MISSING:
                rq->kind = CsmNotifySrv::Request::KIND_PAIR_MISSING;
                break;
        }
        rq->peer_csm_health = peerHealth;
        rq->event_id = eventId;
        rq->target_csm_instance_id = targetInstance;
        rq->entries = entries;
        cli->async_send_request(rq,
            [this, life = life_, eventId](
                rclcpp::Client<CsmNotifySrv>::SharedFuture f) {
                if (!life->alive.load()) return;
                life->active.fetch_add(1);
                if (!life->alive.load()) { life->active.fetch_sub(1); return; }
                struct Drop { LifeToken* t; ~Drop() { t->active.fetch_sub(1); } }
                    drop{life.get()};
                std::lock_guard<std::mutex> lk(completionMtx_);
                completions_.push_back({eventId, f.get()->response, true});
            });
    }

    void _drainCompletions(int64_t now)
    {
        std::vector<NotifyCompletion> done;
        {
            std::lock_guard<std::mutex> lk(completionMtx_);
            done.assign(completions_.begin(), completions_.end());
            completions_.clear();
        }
        if (done.empty())
            return;
        std::unique_lock<std::shared_mutex> lk(csmMtx_);
        for (const auto& c : done)
        {
            const auto it = pendingNotifications_.find(c.eventId);
            if (it == pendingNotifications_.end())
                continue;
            auto& pn = it->second;
            pn.inFlight = false;
            const bool settled =
                c.transportOk &&
                (c.response == CsmNotifySrv::Response::RESPONSE_APPLIED ||
                 c.response == CsmNotifySrv::Response::RESPONSE_ALREADY_APPLIED ||
                 c.response == CsmNotifySrv::Response::RESPONSE_STALE);
            if (settled)
            {
                if (pn.kind == PendingKind::PAIR_MISSING)
                {
                    // ACK only proves the command was queued; the level stays
                    // until a snapshot proves removal (§9.3).
                    pn.acknowledged = true;
                    pn.acknowledgedNs = now;
                }
                else
                {
                    pendingNotifications_.erase(it);
                }
                continue;
            }
            // Transport loss / REJECTED: exponential backoff + jitter,
            // no attempt cap (correctness-critical, §9.2).
            int64_t delayMs = opt_.notificationRetry.initialDelayMs;
            for (uint32_t i = 1;
                 i < pn.attempts && delayMs < opt_.notificationRetry.maxDelayMs;
                 ++i)
                delayMs *= 2;
            delayMs = std::min(delayMs, opt_.notificationRetry.maxDelayMs);
            static thread_local std::mt19937 jrng{std::random_device{}()};
            std::uniform_real_distribution<double> dist(
                1.0 - opt_.notificationRetry.jitterRatio,
                1.0 + opt_.notificationRetry.jitterRatio);
            pn.nextTryNs =
                now + static_cast<int64_t>(delayMs * dist(jrng)) * 1'000'000;
        }
    }

    // ── members ───────────────────────────────────────────────────────────

    struct LifeToken
    {
        std::atomic<bool> alive{true};
        std::atomic<int> active{0};
    };

    rclcpp::Node* node_;
    const MasterOptions opt_;
    std::shared_ptr<LifeToken> life_{std::make_shared<LifeToken>()};

    mutable std::shared_mutex csmMtx_;   // guards the three maps + records
    std::map<std::string, CsmRecord> csms_;
    std::map<PairIdentity, int64_t> unpairedSinceNs_;
    std::map<std::string, PendingNotification> pendingNotifications_;
    std::map<std::string, int64_t> absenceSince_;   // per-name absence clock

    std::mutex completionMtx_;
    std::deque<NotifyCompletion> completions_;

    mutable std::mutex filterMtx_;
    std::optional<std::set<std::string>> whitelist_;
    std::optional<std::set<std::string>> blacklist_;

    rclcpp::CallbackGroup::SharedPtr group_;
    rclcpp::Service<CsmRegisterSrv>::SharedPtr regSrv_;
    rclcpp::Service<CsmHeartbeatSrv>::SharedPtr hbSrv_;
    rclcpp::TimerBase::SharedPtr tick_;
};

} // namespace r1
} // namespace rv2_interfaces

#endif // RV2_CONTROL_SIGNAL_TRANSPORT_R1_CSM_MASTER_H
