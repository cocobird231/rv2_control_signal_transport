/**
 * @file source_registration.h
 * @brief Registration support types at namespace scope (design draft §8.2,
 *        v1.2.1): RegistrationIdentity, lifecycle enums,
 *        SourceRegistrationSlot and RetryCompletion.
 *
 * These live outside ControlSignalManager on purpose: SourceHandle holds a
 * weak_ptr<SourceRegistrationSlot> while the Manager's RegisterResult holds
 * a SourceHandle by value — nesting the types inside the Manager would make
 * that cycle unorderable (§8.2). By convention only the Manager writes them.
 */

#ifndef RV2_CONTROL_SIGNAL_TRANSPORT_R1_SOURCE_REGISTRATION_H
#define RV2_CONTROL_SIGNAL_TRANSPORT_R1_SOURCE_REGISTRATION_H

#include <memory>
#include <optional>
#include <shared_mutex>
#include <string>

#include "rv2_control_signal_transport/r1/control_signal_info.h"
#include "rv2_control_signal_transport/r1/control_signal_source.h"

namespace rv2_interfaces
{
namespace r1
{

/// Result codes of the user-facing registration API (§8.2; namespace scope
/// because RetryCompletion carries one — the Manager re-exports it).
enum class RegisterError : uint8_t
{
    OK,
    INVALID_INFO,
    INVALID_CONTEXT,
    FILTERED,
    DUPLICATE,
    TARGET_UNREACHABLE,
    TIMEOUT_UNKNOWN,  // TIMEOUT_UNKNOWN = outcome unknown, rollback fired
    RETRYABLE_CONFLICT,
    REJECTED,
    TYPE_UNSUPPORTED,
    RETRY_SCHEDULED
};

/// The identity triple every cross-CSM mutation must match (§1.3.1/§2.4).
struct RegistrationIdentity
{
    std::string sourceCsmInstanceId;
    std::string registrationId;
    uint64_t attemptGeneration{0};

    bool operator==(const RegistrationIdentity& o) const
    {
        return sourceCsmInstanceId == o.sourceCsmInstanceId && registrationId == o.registrationId &&
               attemptGeneration == o.attemptGeneration;
    }
    bool operator!=(const RegistrationIdentity& o) const { return !(*this == o); }
};

/// Registration lifecycle, layer 2 of the three-layer model (§1.3.1).
/// ABSENT is represented by removal from the map, not by an enum value.
enum class RegistrationPhase : uint8_t
{
    PENDING,
    REGISTERED,
    RETRY_WAIT,
    REMOVING
};

/// Layer 3: peer / CSM health as reported by the master — never merged into
/// the local ControlSignalState (§1.3.1 invariant 1).
enum class PeerHealth : uint8_t
{
    UNKNOWN,
    ACTIVE,
    TIMEOUT,
    DISCONNECTED
};

/// Terminal routing key (§8.3 stage 3 / §10.3): INACTIVITY, FORCED and
/// EXPLICIT_UNREGISTER remove the logical slot; RESPONSE_FAILURE,
/// PEER_DISCONNECTED and PAIR_MISSING keep it and enter RETRY_WAIT.
enum class RemovalReason : uint8_t
{
    LOCAL_DISCONNECT,
    FORCED,
    EXPLICIT_UNREGISTER,
    RESPONSE_FAILURE,
    PEER_DISCONNECTED,
    PAIR_MISSING
};

enum class RetryReason : uint8_t
{
    INITIAL_UNREACHABLE,
    OLD_GENERATION_CONFLICT,
    RESPONSE_FAILURE,
    PEER_DISCONNECTED,
    PAIR_MISSING
};

/// Stable slot owning the current Source endpoint (§8.2). The logical
/// registration intent (registrationId) outlives endpoint replacements;
/// SourceHandle binds to the slot, not the endpoint (§10).
struct SourceRegistrationSlot
{
    mutable std::shared_mutex slotMtx;
    ControlSignalInfo info;
    std::string registrationId;  // logical intent, fixed for the slot's life
    uint64_t attemptGeneration{0};
    RegistrationPhase phase{RegistrationPhase::PENDING};
    std::shared_ptr<BaseControlSignalSource> endpoint;
    EntityStatus lastStatus{ControlSignalState::INITIAL, 0.f};
    PeerHealth peerHealth{PeerHealth::UNKNOWN};  // never merged into local state
    std::optional<RemovalReason> pendingRemoval;
    bool desired{true};  // unregister clears atomically; async responses must not revive
    // v1.2.1 oscillation damping: consecutive never-ACTIVE terminations of
    // rebuilt endpoints; reaching the quarantine threshold switches the
    // retry backoff to its long (maxDelayMs-based) form.
    uint32_t neverActiveTerminations{0};
    bool wasActiveSinceEndpoint{false};
};

/// Written by retry response callbacks, drained by tick stage 5 (§8.2/§8.3).
struct RetryCompletion
{
    RegistrationIdentity identity;
    RegisterError result;
    std::string reason;
};

}  // namespace r1
}  // namespace rv2_interfaces

#endif  // RV2_CONTROL_SIGNAL_TRANSPORT_R1_SOURCE_REGISTRATION_H
