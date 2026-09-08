/**
 * @file test_csm_master.cpp
 * @brief CM1-CM13 unit tests for r1::CsmMaster (design §9.4). Mock CSMs are
 *        bare nodes (status publisher + get_notifications server +
 *        register/heartbeat clients) — no real ControlSignalManager.
 */

#include <atomic>
#include <chrono>
#include <functional>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <gtest/gtest.h>
#include <rclcpp/rclcpp.hpp>

#include "r1_test_utils.h"
#include "rv2_control_signal_transport/r1/csm_master.h"

namespace
{

using namespace std::chrono_literals;
using rv2_interfaces::r1::CsmMaster;
using rv2_interfaces::r1::MasterOptions;
using rv2_interfaces::r1::NotificationRetryPolicy;
using CsmRegisterSrv = r1_interfaces::srv::CsmRegister;
using CsmHeartbeatSrv = r1_interfaces::srv::CsmHeartbeat;
using CsmNotifySrv = r1_interfaces::srv::CsmNotify;
using ManagerStatusT = r1_interfaces::msg::ManagerStatus;
using EntryStatusT = r1_interfaces::msg::EntryStatus;

constexpr int64_t kMs = 1'000'000;

bool waitFor(const std::function<bool()>& cond, int64_t timeoutMs = 5000)
{
    const auto deadline =
        std::chrono::steady_clock::now() + std::chrono::milliseconds(timeoutMs);
    while (std::chrono::steady_clock::now() < deadline)
    {
        if (cond())
            return true;
        std::this_thread::sleep_for(10ms);
    }
    return cond();
}

/// Bare-node mock CSM (§9.4).
struct MockCsm
{
    MockCsm(rclcpp::Node* node, const std::string& masterName,
            const std::string& name, const std::string& instance)
        : node_(node), name_(name), instance_(instance)
    {
        regCli_ = node->create_client<CsmRegisterSrv>(masterName + "/register");
        hbCli_ = node->create_client<CsmHeartbeatSrv>(masterName + "/heartbeat");
        statusPub_ = node->create_publisher<ManagerStatusT>(name + "/status",
                                                            rclcpp::QoS(10));
        makeNotifyService();
    }

    void makeNotifyService()
    {
        notifySrv_ = node_->create_service<CsmNotifySrv>(
            name_ + "/get_notifications",
            [this](const std::shared_ptr<CsmNotifySrv::Request> rq,
                   std::shared_ptr<CsmNotifySrv::Response> rs) {
                std::lock_guard<std::mutex> lk(mtx_);
                notifications_.push_back(*rq);
                rs->response = scriptedResponse_;
            });
    }
    void killNotify() { notifySrv_.reset(); }

    int8_t doRegister(int64_t timeoutNs = 400 * kMs,
                      int64_t disconnectNs = 1'200 * kMs,
                      int64_t intervalNs = 100 * kMs,
                      int64_t graceNs = 400 * kMs,
                      const std::string& instanceOverride = "")
    {
        auto rq = std::make_shared<CsmRegisterSrv::Request>();
        rq->csm_name = name_;
        rq->csm_instance_id =
            instanceOverride.empty() ? instance_ : instanceOverride;
        rq->csm_timeout_ns = timeoutNs;
        rq->csm_disconnect_timeout_ns = disconnectNs;
        rq->status_interval_ns = intervalNs;
        rq->registration_grace_ns = graceNs;
        if (!regCli_->wait_for_service(3s))
            return -1;
        auto f = regCli_->async_send_request(rq);
        if (f.wait_for(3s) != std::future_status::ready)
            return -1;
        return f.get()->response;
    }

    int8_t heartbeat(const std::string& instanceOverride = "")
    {
        auto rq = std::make_shared<CsmHeartbeatSrv::Request>();
        rq->csm_name = name_;
        rq->csm_instance_id =
            instanceOverride.empty() ? instance_ : instanceOverride;
        auto f = hbCli_->async_send_request(rq);
        if (f.wait_for(3s) != std::future_status::ready)
            return -1;
        return f.get()->response;
    }

    void startHeartbeatLoop(int64_t periodMs = 100)
    {
        stopHb_.store(false);
        hbThread_ = std::thread([this, periodMs] {
            while (!stopHb_.load())
            {
                (void)heartbeat();
                std::this_thread::sleep_for(std::chrono::milliseconds(periodMs));
            }
        });
    }
    void stopHeartbeatLoop()
    {
        stopHb_.store(true);
        if (hbThread_.joinable())
            hbThread_.join();
    }

    void publish(const std::vector<EntryStatusT>& sources,
                 const std::vector<EntryStatusT>& sinks, bool ready = true,
                 std::optional<uint64_t> seqOverride = {},
                 const std::string& instanceOverride = "")
    {
        ManagerStatusT m;
        m.manager_name = name_;
        m.csm_instance_id =
            instanceOverride.empty() ? instance_ : instanceOverride;
        m.snapshot_seq = seqOverride ? *seqOverride : ++seq_;
        m.snapshot_ready = ready;
        m.sources = sources;
        m.sinks = sinks;
        statusPub_->publish(m);
    }

    int notifyCount(int8_t kind)
    {
        std::lock_guard<std::mutex> lk(mtx_);
        int n = 0;
        for (const auto& q : notifications_)
            if (q.kind == kind)
                ++n;
        return n;
    }
    std::vector<CsmNotifySrv::Request> notifications()
    {
        std::lock_guard<std::mutex> lk(mtx_);
        return notifications_;
    }

    ~MockCsm() { stopHeartbeatLoop(); }

    rclcpp::Node* node_;
    std::string name_;
    std::string instance_;
    uint64_t seq_{0};
    std::mutex mtx_;
    std::vector<CsmNotifySrv::Request> notifications_;
    int8_t scriptedResponse_{CsmNotifySrv::Response::RESPONSE_APPLIED};
    std::atomic<bool> stopHb_{false};
    std::thread hbThread_;
    rclcpp::Client<CsmRegisterSrv>::SharedPtr regCli_;
    rclcpp::Client<CsmHeartbeatSrv>::SharedPtr hbCli_;
    rclcpp::Publisher<ManagerStatusT>::SharedPtr statusPub_;
    rclcpp::Service<CsmNotifySrv>::SharedPtr notifySrv_;
};

class CsmMasterTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        uid_ = "cm" + std::to_string(counter_++);
        masterNode_ = std::make_shared<rclcpp::Node>("master_" + uid_);
        csmNode_ = std::make_shared<rclcpp::Node>("mockcsm_" + uid_);
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
            rclcpp::ExecutorOptions(), 4);
        executor_->add_node(masterNode_);
        executor_->add_node(csmNode_);
        spin_ = std::thread([this] { executor_->spin(); });
        while (!executor_->is_spinning())
            std::this_thread::sleep_for(1ms);

        NotificationRetryPolicy retry(100, 500, 0.1, 8);
        MasterOptions opt(retry);
        opt.masterName = masterName();
        opt.tickIntervalMs = 100;
        opt.pairGraceMs = 400;
        master_ = std::make_unique<CsmMaster>(masterNode_.get(), opt);

        a_ = std::make_unique<MockCsm>(csmNode_.get(), masterName(),
                                       "A_" + uid_, "ai1");
        b_ = std::make_unique<MockCsm>(csmNode_.get(), masterName(),
                                       "B_" + uid_, "bi1");
    }

    void TearDown() override
    {
        a_.reset();
        b_.reset();
        master_.reset();
        executor_->cancel();
        if (spin_.joinable())
            spin_.join();
        executor_.reset();
        csmNode_.reset();
        masterNode_.reset();
    }

    std::string masterName() const { return "cmaster_" + uid_; }

    /// Mirrored source/sink pair entries for controller X.
    EntryStatusT sourceEntry(const std::string& tag, int8_t state,
                             uint64_t gen = 1) const
    {
        EntryStatusT e;
        e.manager_name = a_->name_;
        e.csm_instance_id = a_->instance_;
        e.source_manager_name = a_->name_;
        e.source_csm_instance_id = a_->instance_;
        e.target_manager_name = b_->name_;
        e.registration_id = "rid_" + uid_ + "_" + tag;
        e.attempt_generation = gen;
        e.controller_name = "ctrl_" + uid_ + "_" + tag;
        e.channel_name = uid_ + "/" + tag;
        e.type = "joy";
        e.mode = "topic";
        e.is_source = true;
        e.registration_phase = EntryStatusT::PHASE_REGISTERED;
        e.endpoint_present = true;
        e.state = state;
        e.data_rate_hz = 10.f;
        e.priority = 50;
        return e;
    }

    EntryStatusT sinkEntry(const std::string& tag, int8_t state,
                           uint64_t gen = 1) const
    {
        EntryStatusT e = sourceEntry(tag, state, gen);
        e.manager_name = b_->name_;
        e.csm_instance_id = b_->instance_;
        e.is_source = false;
        return e;
    }

    void registerBoth()
    {
        ASSERT_EQ(a_->doRegister(), CsmRegisterSrv::Response::RESPONSE_SUCCESS);
        ASSERT_EQ(b_->doRegister(), CsmRegisterSrv::Response::RESPONSE_SUCCESS);
        a_->startHeartbeatLoop();
        b_->startHeartbeatLoop();
    }

    static int counter_;
    std::string uid_;
    rclcpp::Node::SharedPtr masterNode_, csmNode_;
    std::shared_ptr<rclcpp::executors::MultiThreadedExecutor> executor_;
    std::thread spin_;
    std::unique_ptr<CsmMaster> master_;
    std::unique_ptr<MockCsm> a_, b_;
};
int CsmMasterTest::counter_ = 0;

// CM1: register + heartbeat — matching heartbeat updates the record; a new
// instance retires the old ID whose heartbeat and late register answer STALE.
TEST_F(CsmMasterTest, CM1_RegisterHeartbeatRetire)
{
    ASSERT_EQ(a_->doRegister(), CsmRegisterSrv::Response::RESPONSE_SUCCESS);
    EXPECT_EQ(a_->heartbeat(), CsmHeartbeatSrv::Response::RESPONSE_SUCCESS);
    EXPECT_EQ(a_->heartbeat("ghost"),
              CsmHeartbeatSrv::Response::RESPONSE_STALE_INSTANCE);
    EXPECT_EQ(b_->heartbeat(), CsmHeartbeatSrv::Response::RESPONSE_UNKNOWN_CSM);

    // Idempotent same-instance re-register.
    EXPECT_EQ(a_->doRegister(), CsmRegisterSrv::Response::RESPONSE_SUCCESS);
    // Replacement: new instance retires ai1.
    EXPECT_EQ(a_->doRegister(400 * kMs, 1200 * kMs, 100 * kMs, 400 * kMs, "ai2"),
              CsmRegisterSrv::Response::RESPONSE_SUCCESS);
    EXPECT_EQ(a_->heartbeat("ai1"),
              CsmHeartbeatSrv::Response::RESPONSE_STALE_INSTANCE);
    EXPECT_EQ(a_->doRegister(400 * kMs, 1200 * kMs, 100 * kMs, 400 * kMs, "ai1"),
              CsmRegisterSrv::Response::RESPONSE_STALE_INSTANCE);
    EXPECT_EQ(a_->heartbeat("ai2"),
              CsmHeartbeatSrv::Response::RESPONSE_SUCCESS);

    // Invalid thresholds refused; the interval/grace fields are consumed by
    // the same validation (record 內含 interval — observable via its rules).
    EXPECT_EQ(a_->doRegister(500 * kMs, 500 * kMs),
              CsmRegisterSrv::Response::RESPONSE_INVALID_CONFIG);
    EXPECT_EQ(a_->doRegister(400 * kMs, 1200 * kMs, 0),
              CsmRegisterSrv::Response::RESPONSE_INVALID_CONFIG);
    EXPECT_EQ(a_->doRegister(400 * kMs, 1200 * kMs, 100 * kMs, 100 * kMs),
              CsmRegisterSrv::Response::RESPONSE_INVALID_CONFIG);
}

// CM2: black/whitelist — refused registration creates nothing.
TEST_F(CsmMasterTest, CM2_Filters)
{
    master_->enableCsmBlacklist({a_->name_});
    EXPECT_EQ(a_->doRegister(),
              CsmRegisterSrv::Response::RESPONSE_INVALID_CONFIG);
    EXPECT_FALSE(master_->hasCsm(a_->name_));
    std::this_thread::sleep_for(200ms);
    // No status subscription was created for the refused CSM (doc CM2).
    EXPECT_EQ(csmNode_->count_subscribers(a_->name_ + "/status"), 0u);
    master_->disableCsmBlacklist();

    master_->enableCsmWhitelist({});   // enabled empty = block all
    EXPECT_EQ(b_->doRegister(),
              CsmRegisterSrv::Response::RESPONSE_INVALID_CONFIG);
    master_->disableCsmWhitelist();
    EXPECT_EQ(b_->doRegister(), CsmRegisterSrv::Response::RESPONSE_SUCCESS);
}

// CM3: pairing notification — a Source state edge sends one KIND_STATE to
// each side with the new state inside.
TEST_F(CsmMasterTest, CM3_StateEdgeNotifiesBothSides)
{
    registerBoth();
    // Baseline snapshots (no notifications for the first ready snapshot).
    a_->publish({sourceEntry("x", EntryStatusT::STATE_ACTIVE)}, {});
    b_->publish({}, {sinkEntry("x", EntryStatusT::STATE_ACTIVE)});
    std::this_thread::sleep_for(400ms);
    EXPECT_EQ(a_->notifyCount(CsmNotifySrv::Request::KIND_STATE), 0);
    EXPECT_EQ(b_->notifyCount(CsmNotifySrv::Request::KIND_STATE), 0);

    // Edge: X ACTIVE -> TIMEOUT.
    a_->publish({sourceEntry("x", EntryStatusT::STATE_TIMEOUT)}, {});
    ASSERT_TRUE(waitFor([&] {
        return a_->notifyCount(CsmNotifySrv::Request::KIND_STATE) >= 1 &&
               b_->notifyCount(CsmNotifySrv::Request::KIND_STATE) >= 1;
    }));
    bool sawTimeout = false;
    for (const auto& q : b_->notifications())
        for (const auto& e : q.entries)
            if (e.controller_name == "ctrl_" + uid_ + "_x" &&
                e.state == EntryStatusT::STATE_TIMEOUT)
                sawTimeout = true;
    EXPECT_TRUE(sawTimeout);
}

// CM4: one-shot — repeating the same state does not resend; recovery to
// ACTIVE sends exactly one more.
TEST_F(CsmMasterTest, CM4_OneShotEdges)
{
    registerBoth();
    a_->publish({sourceEntry("x", EntryStatusT::STATE_ACTIVE)}, {});
    b_->publish({}, {sinkEntry("x", EntryStatusT::STATE_ACTIVE)});
    std::this_thread::sleep_for(300ms);

    a_->publish({sourceEntry("x", EntryStatusT::STATE_TIMEOUT)}, {});
    ASSERT_TRUE(waitFor([&] {
        return b_->notifyCount(CsmNotifySrv::Request::KIND_STATE) >= 1;
    }));
    const int after1 = b_->notifyCount(CsmNotifySrv::Request::KIND_STATE);

    a_->publish({sourceEntry("x", EntryStatusT::STATE_TIMEOUT)}, {});
    a_->publish({sourceEntry("x", EntryStatusT::STATE_TIMEOUT)}, {});
    std::this_thread::sleep_for(400ms);
    EXPECT_EQ(b_->notifyCount(CsmNotifySrv::Request::KIND_STATE), after1);

    a_->publish({sourceEntry("x", EntryStatusT::STATE_ACTIVE)}, {});
    ASSERT_TRUE(waitFor([&] {
        return b_->notifyCount(CsmNotifySrv::Request::KIND_STATE) == after1 + 1;
    }));   // recovery edge: exactly one more to the peer
    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(b_->notifyCount(CsmNotifySrv::Request::KIND_STATE), after1 + 1);
}

// CM5: CSM-level TIMEOUT warning (D6) — heartbeat silence beyond timeout
// warns the peer (peerHealth TIMEOUT, reliable with resend); recovery sends
// the ACTIVE clearance reliably too.
TEST_F(CsmMasterTest, CM5_CsmTimeoutWarning)
{
    registerBoth();
    a_->publish({sourceEntry("x", EntryStatusT::STATE_ACTIVE)}, {});
    b_->publish({}, {sinkEntry("x", EntryStatusT::STATE_ACTIVE)});
    std::this_thread::sleep_for(200ms);

    // Peer B's notify service is dead first: the warning must survive
    // the RPC loss and arrive after revival.
    b_->killNotify();
    a_->stopHeartbeatLoop();   // silence > 400 ms
    std::this_thread::sleep_for(700ms);
    b_->makeNotifyService();
    ASSERT_TRUE(waitFor([&] {
        return b_->notifyCount(CsmNotifySrv::Request::KIND_CSM_TIMEOUT) >= 1;
    }));
    bool sawWarn = false;
    for (const auto& q : b_->notifications())
        if (q.kind == CsmNotifySrv::Request::KIND_CSM_TIMEOUT &&
            q.peer_csm_health == CsmNotifySrv::Request::PEER_HEALTH_TIMEOUT)
            sawWarn = true;
    EXPECT_TRUE(sawWarn);

    // Recovery: heartbeats resume -> reliable ACTIVE clearance.
    a_->startHeartbeatLoop();
    ASSERT_TRUE(waitFor([&] {
        for (const auto& q : b_->notifications())
            if (q.kind == CsmNotifySrv::Request::KIND_CSM_TIMEOUT &&
                q.peer_csm_health == CsmNotifySrv::Request::PEER_HEALTH_ACTIVE)
                return true;
        return false;
    }));
}

// CM6: CSM-level DISCONNECTED (D6) — the peer receives the matching-
// generation lifecycle event; the dead CSM's old instance cannot revive the
// record; only a register (same or new instance) can.
TEST_F(CsmMasterTest, CM6_CsmDisconnected)
{
    registerBoth();
    a_->publish({sourceEntry("x", EntryStatusT::STATE_ACTIVE)}, {});
    b_->publish({}, {sinkEntry("x", EntryStatusT::STATE_ACTIVE)});
    std::this_thread::sleep_for(200ms);

    a_->stopHeartbeatLoop();
    ASSERT_TRUE(waitFor([&] {
        return b_->notifyCount(CsmNotifySrv::Request::KIND_DISCONNECTED) >= 1;
    }, 8000));
    bool sawIdentity = false;
    for (const auto& q : b_->notifications())
        if (q.kind == CsmNotifySrv::Request::KIND_DISCONNECTED)
            for (const auto& e : q.entries)
                if (e.registration_id == "rid_" + uid_ + "_x" &&
                    e.attempt_generation == 1)
                    sawIdentity = true;
    EXPECT_TRUE(sawIdentity);

    // Old incarnation is fenced out after the verdict (v1.2.1): heartbeat
    // answers STALE and a same-instance status snapshot is dropped whole —
    // no STATE resurrection reaches the peer.
    EXPECT_EQ(a_->heartbeat(),
              CsmHeartbeatSrv::Response::RESPONSE_STALE_INSTANCE);
    const int stateBefore = b_->notifyCount(CsmNotifySrv::Request::KIND_STATE);
    // A CHANGED state: without the DISCONNECTED-record fence this snapshot
    // would be accepted and produce STATE edges to both sides.
    a_->publish({sourceEntry("x", EntryStatusT::STATE_TIMEOUT)}, {});
    std::this_thread::sleep_for(400ms);
    EXPECT_EQ(b_->notifyCount(CsmNotifySrv::Request::KIND_STATE), stateBefore);
    // Same-instance explicit re-register = full rebuild, allowed.
    EXPECT_EQ(a_->doRegister(), CsmRegisterSrv::Response::RESPONSE_SUCCESS);
    EXPECT_EQ(a_->heartbeat(), CsmHeartbeatSrv::Response::RESPONSE_SUCCESS);
}

// CM7: readiness gating — first ready snapshots create no STATE storm; no
// reconciliation before both snapshots are ready, level reconciliation runs
// right after.
TEST_F(CsmMasterTest, CM7_ReadinessGate)
{
    registerBoth();
    // A's baseline already contains a TIMEOUT entry: no STATE notifications.
    a_->publish({sourceEntry("x", EntryStatusT::STATE_TIMEOUT)}, {});
    std::this_thread::sleep_for(300ms);
    EXPECT_EQ(a_->notifyCount(CsmNotifySrv::Request::KIND_STATE), 0);
    EXPECT_EQ(b_->notifyCount(CsmNotifySrv::Request::KIND_STATE), 0);

    // B not ready yet: the single-sided X must NOT trigger PAIR_MISSING.
    std::this_thread::sleep_for(600ms);   // > pairGrace
    EXPECT_EQ(a_->notifyCount(CsmNotifySrv::Request::KIND_PAIR_MISSING), 0);

    // B delivers its ready (empty) snapshot: reconciliation starts at once.
    b_->publish({}, {});
    ASSERT_TRUE(waitFor([&] {
        return a_->notifyCount(CsmNotifySrv::Request::KIND_PAIR_MISSING) >= 1;
    }, 8000));
}

// CM8: fast-restart reconciliation — B's new instance publishes an empty
// snapshot; past the grace A receives PAIR_MISSING, resent until ACK.
TEST_F(CsmMasterTest, CM8_FastRestartPairMissing)
{
    registerBoth();
    a_->publish({sourceEntry("x", EntryStatusT::STATE_ACTIVE)}, {});
    b_->publish({}, {sinkEntry("x", EntryStatusT::STATE_ACTIVE)});
    std::this_thread::sleep_for(300ms);

    // B restarts with a fresh instance and an EMPTY complete snapshot.
    b_->stopHeartbeatLoop();
    b_->instance_ = "bi2";
    b_->seq_ = 0;
    ASSERT_EQ(b_->doRegister(), CsmRegisterSrv::Response::RESPONSE_SUCCESS);
    b_->startHeartbeatLoop();

    // A's notify service down first: the level event survives RPC loss.
    a_->killNotify();
    b_->publish({}, {});
    std::this_thread::sleep_for(700ms);   // > grace while A is unreachable
    a_->makeNotifyService();
    ASSERT_TRUE(waitFor([&] {
        return a_->notifyCount(CsmNotifySrv::Request::KIND_PAIR_MISSING) >= 1;
    }, 8000));

    // Level persists after ACK (delivered-awaiting-observation): with A's
    // snapshot still showing X, the same event resends past the apply grace.
    ASSERT_TRUE(waitFor([&] {
        return a_->notifyCount(CsmNotifySrv::Request::KIND_PAIR_MISSING) >= 2;
    }, 8000));
    const auto notifs = a_->notifications();
    std::string firstId;
    for (const auto& q : notifs)
        if (q.kind == CsmNotifySrv::Request::KIND_PAIR_MISSING)
        {
            if (firstId.empty())
                firstId = q.event_id;
            else
                EXPECT_EQ(q.event_id, firstId);   // same level event ID
        }

    // A proves removal: the level clears, deliveries stop.
    a_->publish({}, {});
    std::this_thread::sleep_for(300ms);
    const int settled = a_->notifyCount(CsmNotifySrv::Request::KIND_PAIR_MISSING);
    std::this_thread::sleep_for(900ms);
    EXPECT_EQ(a_->notifyCount(CsmNotifySrv::Request::KIND_PAIR_MISSING), settled);
}

// CM9: unreachable notification target — STATE may be lost; control events
// are retried non-blockingly and the master stays responsive throughout.
TEST_F(CsmMasterTest, CM9_NonBlockingRetries)
{
    registerBoth();
    a_->publish({sourceEntry("x", EntryStatusT::STATE_ACTIVE)}, {});
    b_->publish({}, {sinkEntry("x", EntryStatusT::STATE_ACTIVE)});
    std::this_thread::sleep_for(200ms);

    b_->killNotify();
    a_->stopHeartbeatLoop();   // drive a CSM_TIMEOUT control event toward B
    std::this_thread::sleep_for(700ms);

    // Master must keep serving while it retries into the void.
    const auto t0 = std::chrono::steady_clock::now();
    EXPECT_EQ(b_->heartbeat(), CsmHeartbeatSrv::Response::RESPONSE_SUCCESS);
    EXPECT_LE(std::chrono::steady_clock::now() - t0, 1s);
    EXPECT_EQ(a_->doRegister(400 * kMs, 1200 * kMs, 100 * kMs, 400 * kMs, "ai9"),
              CsmRegisterSrv::Response::RESPONSE_SUCCESS);

    // Revived target eventually receives the pending control event.
    b_->makeNotifyService();
    ASSERT_TRUE(waitFor([&] {
        return b_->notifyCount(CsmNotifySrv::Request::KIND_CSM_TIMEOUT) >= 1;
    }, 8000));
}

// CM10: omission / sequence rules — seq N+1 omission removes the entry
// (observed as a STATE(DISCONNECTED) tombstone to the peer); duplicate,
// out-of-order and old-instance snapshots never overwrite.
TEST_F(CsmMasterTest, CM10_SnapshotOmissionAndSequence)
{
    registerBoth();
    a_->publish({sourceEntry("x", EntryStatusT::STATE_ACTIVE)}, {});
    b_->publish({}, {sinkEntry("x", EntryStatusT::STATE_ACTIVE)});
    std::this_thread::sleep_for(300ms);

    // Omission: B's next snapshot drops sink X -> tombstone STATE to A.
    b_->publish({}, {});
    ASSERT_TRUE(waitFor([&] {
        for (const auto& q : a_->notifications())
            if (q.kind == CsmNotifySrv::Request::KIND_STATE)
                for (const auto& e : q.entries)
                    if (!e.is_source &&
                        e.state == EntryStatusT::STATE_DISCONNECTED)
                        return true;
        return false;
    }));
    const auto countAfterTombstone =
        a_->notifyCount(CsmNotifySrv::Request::KIND_STATE);

    // Replay of an OLD sequence (still containing X) must be dropped whole.
    b_->publish({}, {sinkEntry("x", EntryStatusT::STATE_ACTIVE)}, true,
                uint64_t{1});
    // Old-instance snapshot equally dropped.
    b_->publish({}, {sinkEntry("x", EntryStatusT::STATE_ACTIVE)}, true,
                uint64_t{99}, "bi_old");
    std::this_thread::sleep_for(400ms);
    EXPECT_EQ(a_->notifyCount(CsmNotifySrv::Request::KIND_STATE),
              countAfterTombstone);   // no resurrection edges
}

// CM11: PENDING transactions suspend the missing grace; completion pairs
// normally; a rollback (entry gone) starts the grace clock.
TEST_F(CsmMasterTest, CM11_PendingSuppressesGrace)
{
    registerBoth();
    a_->publish({sourceEntry("x", EntryStatusT::STATE_ACTIVE)}, {});
    auto pendingSink = sinkEntry("x", EntryStatusT::STATE_INITIAL);
    pendingSink.registration_phase = EntryStatusT::PHASE_PENDING;
    pendingSink.endpoint_present = false;
    b_->publish({}, {pendingSink});

    std::this_thread::sleep_for(1200ms);   // well past every grace
    EXPECT_EQ(a_->notifyCount(CsmNotifySrv::Request::KIND_PAIR_MISSING), 0);

    // Transaction completes: proper pair, still no PAIR_MISSING.
    b_->publish({}, {sinkEntry("x", EntryStatusT::STATE_ACTIVE)});
    std::this_thread::sleep_for(600ms);
    EXPECT_EQ(a_->notifyCount(CsmNotifySrv::Request::KIND_PAIR_MISSING), 0);

    // Rollback: sink vanishes; the grace clock starts NOW — the missing
    // notification must not arrive earlier than the minimum grace.
    const auto tRollback = std::chrono::steady_clock::now();
    b_->publish({}, {});
    ASSERT_TRUE(waitFor([&] {
        return a_->notifyCount(CsmNotifySrv::Request::KIND_PAIR_MISSING) >= 1;
    }, 8000));
    const auto elapsed = std::chrono::steady_clock::now() - tRollback;
    EXPECT_GE(elapsed, 350ms);   // grace = max(400ms, ...) minus margin
}

// CM12: settled ACK classes — ALREADY_APPLIED (and STALE) close the delivery
// of edge events; no infinite resends.
TEST_F(CsmMasterTest, CM12_AckClassesSettle)
{
    registerBoth();
    // Long disconnect: this case must stay in the TIMEOUT/ACTIVE regime —
    // the record may never cross into DISCONNECTED during the settle waits.
    ASSERT_EQ(a_->doRegister(400 * kMs, 30'000 * kMs),
              CsmRegisterSrv::Response::RESPONSE_SUCCESS);
    a_->publish({sourceEntry("x", EntryStatusT::STATE_ACTIVE)}, {});
    b_->publish({}, {sinkEntry("x", EntryStatusT::STATE_ACTIVE)});
    std::this_thread::sleep_for(200ms);

    b_->scriptedResponse_ = CsmNotifySrv::Response::RESPONSE_ALREADY_APPLIED;
    a_->stopHeartbeatLoop();   // CSM_TIMEOUT control event toward B
    ASSERT_TRUE(waitFor([&] {
        return b_->notifyCount(CsmNotifySrv::Request::KIND_CSM_TIMEOUT) >= 1;
    }, 8000));
    const int settled = b_->notifyCount(CsmNotifySrv::Request::KIND_CSM_TIMEOUT);
    std::this_thread::sleep_for(800ms);
    EXPECT_EQ(b_->notifyCount(CsmNotifySrv::Request::KIND_CSM_TIMEOUT), settled);

    // STALE settles edge events the same way: the ACTIVE clearance answered
    // with STALE is not resent forever.
    b_->scriptedResponse_ = CsmNotifySrv::Response::RESPONSE_STALE;
    a_->startHeartbeatLoop();   // recovery -> ACTIVE clearance event
    ASSERT_TRUE(waitFor([&] {
        for (const auto& q : b_->notifications())
            if (q.kind == CsmNotifySrv::Request::KIND_CSM_TIMEOUT &&
                q.peer_csm_health == CsmNotifySrv::Request::PEER_HEALTH_ACTIVE)
                return true;
        return false;
    }, 8000));
    const int settled2 = b_->notifyCount(CsmNotifySrv::Request::KIND_CSM_TIMEOUT);
    std::this_thread::sleep_for(800ms);
    EXPECT_EQ(b_->notifyCount(CsmNotifySrv::Request::KIND_CSM_TIMEOUT), settled2);
}

// CM13: zero thresholds disable individually; disconnect=0 promises no
// bounded auto-convergence for a permanent crash.
TEST_F(CsmMasterTest, CM13_ZeroThresholds)
{
    // A: both disabled — silence forever produces no health events.
    ASSERT_EQ(a_->doRegister(0, 0), CsmRegisterSrv::Response::RESPONSE_SUCCESS);
    // B: timeout only (variant B) — warning yes, disconnect never.
    ASSERT_EQ(b_->doRegister(300 * kMs, 0),
              CsmRegisterSrv::Response::RESPONSE_SUCCESS);
    a_->publish({sourceEntry("x", EntryStatusT::STATE_ACTIVE)}, {});
    b_->publish({}, {sinkEntry("x", EntryStatusT::STATE_ACTIVE)});
    // No heartbeats at all from here on.
    std::this_thread::sleep_for(1500ms);
    EXPECT_EQ(b_->notifyCount(CsmNotifySrv::Request::KIND_DISCONNECTED), 0);
    EXPECT_EQ(a_->notifyCount(CsmNotifySrv::Request::KIND_DISCONNECTED), 0);
    // B (timeout=300ms) went TIMEOUT: A gets the warning; A (thresholds 0)
    // never triggers anything toward B.
    ASSERT_TRUE(waitFor([&] {
        return a_->notifyCount(CsmNotifySrv::Request::KIND_CSM_TIMEOUT) >= 1;
    }));
    EXPECT_EQ(b_->notifyCount(CsmNotifySrv::Request::KIND_CSM_TIMEOUT), 0);

    // Variant C for the CSM level (timeout=0, disconnect>0): the silent A
    // goes straight to DISCONNECTED — no phantom TIMEOUT edge on the way
    // (disconnect judged first, §9.3).
    EXPECT_EQ(a_->doRegister(0, 800 * kMs),
              CsmRegisterSrv::Response::RESPONSE_SUCCESS);
    ASSERT_TRUE(waitFor([&] {
        return b_->notifyCount(CsmNotifySrv::Request::KIND_DISCONNECTED) >= 1;
    }, 8000));
    EXPECT_EQ(b_->notifyCount(CsmNotifySrv::Request::KIND_CSM_TIMEOUT), 0);
}

} // namespace

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    ::testing::AddGlobalTestEnvironment(new rv2_interfaces::r1::RclcppEnv);
    return RUN_ALL_TESTS();
}
