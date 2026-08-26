#pragma once

#include <cstddef>
#include <cstdint>

namespace armhat {

constexpr std::size_t MAX_REQUEST_BYTES = 256;
constexpr std::size_t MAX_RESPONSE_BYTES = 4096;
constexpr std::size_t MAX_ARGUMENTS = 6;
constexpr uint8_t MAX_TRACKED_SERVOS = 8;
// Four: three arm joints plus the camera joint, which is driven like any other
// servo but is not part of the kinematic chain. The Pi's own MAX_HELD stays at 3
// until this is flashed, so an un-updated controller is never handed a hold set
// it will reject.
constexpr uint8_t MAX_HOLD_SERVOS = 4;
constexpr uint16_t MAX_TORQUE_OFF_OBLIGATIONS = 254;
constexpr std::size_t MAX_SERVO_JSON_BYTES = 384;
// Grew to carry busNoise: the byte census that says whether a jammed line is
// stuck low (wiring) or carrying wreckage (protocol). Worst case is checked
// against the response buffer by the static_assert below.
constexpr std::size_t MAX_STATUS_FIXED_JSON_BYTES = 960;
constexpr std::size_t MAX_RESPONSE_FRAME_OVERHEAD_BYTES = 32;
constexpr uint32_t WATCHDOG_TIMEOUT_MS = 750;
constexpr uint32_t TORQUE_LEASE_MIN_MS = 100;
constexpr uint32_t TORQUE_LEASE_MAX_MS = 2000;
constexpr int32_t NUDGE_MAX_DELTA_TICKS = 64;
constexpr uint32_t NUDGE_MAX_SPEED = 256;
constexpr uint32_t NUDGE_MAX_ACCEL = 20;
constexpr uint32_t NUDGE_PLANNED_MOTION_BUDGET_MS = 350;
constexpr uint32_t NUDGE_MS_PER_TICK_AT_SPEED_1 = 1004;

// ---- Runtime-configurable policy -------------------------------------------
// The motion numbers are DEFAULTS, settable at runtime with CONFIG, so tuning
// this arm never needs another flash. The CEILINGs below stay compile-time
// because exceeding them is physically unsafe rather than merely aggressive: a
// motion window longer than the watchdog would outlive the fail-safe, and a
// delta past half a turn is ambiguous on a wrapping encoder.
constexpr uint16_t CONFIG_MAX_DELTA_CEILING = 2048;
constexpr uint16_t CONFIG_MOTION_BUDGET_CEILING_MS = 600;
constexpr uint16_t CONFIG_POSITION_TOLERANCE_CEILING = 256;
constexpr uint16_t CONFIG_EXECUTION_TIMEOUT_CEILING_MS = 600;
constexpr uint16_t CONFIG_SPEED_CEILING = 4095;
constexpr uint16_t CONFIG_ACCEL_CEILING = 255;
// Bounded so a host can never disable supervision entirely: at 25 ms per sample
// even the ceiling revokes authority well inside the watchdog window.
constexpr uint16_t CONFIG_SUPERVISION_TOLERANCE_CEILING = 8;
constexpr uint8_t REGISTER_ACCESS_MAX_LENGTH = 16;
constexpr uint32_t PROPOSAL_TTL_MS = 15000;
constexpr uint32_t INTERNAL_NUDGE_LEASE_MS = 650;

// Resolve a wrapping 12-bit encoder sample into the revolution nearest a
// previously trusted counted position. This is the only safe way to recover a
// mode-3 joint after torque cancels an unfinished relative step: register 56
// has stopped being encoder feedback in mode 3, so the servo must first be put
// back in mode 0 and the real encoder read. The reference is sampled often
// enough that the physical joint cannot travel half a motor turn before the
// resync runs.
constexpr int32_t MULTI_TURN_ENCODER_TICKS = 4096;
constexpr int32_t reconcileWrappedPosition(int32_t reference,
                                           uint16_t wrapped) {
  const int32_t difference =
      reference - static_cast<int32_t>(wrapped);
  const int32_t half = MULTI_TURN_ENCODER_TICKS / 2;
  const int32_t revolutions =
      (difference >= 0
           ? difference + half
           : difference - half + 1) /
      MULTI_TURN_ENCODER_TICKS;
  return revolutions * MULTI_TURN_ENCODER_TICKS +
         static_cast<int32_t>(wrapped);
}

// Regression for the reported +140 -> 0 drift. At ratio 4, stopping 1,365
// motor ticks short is almost exactly 30 joint degrees. A wrapped encoder read
// of 911 near the last trustworthy 5,000-count estimate must recover 5,007,
// never the optimistic 6,372-count command ledger.
static_assert(reconcileWrappedPosition(5000, 911) == 5007,
              "multi-turn resync must recover physical, not commanded, pose");
static_assert(reconcileWrappedPosition(-5000, 3185) == -5007,
              "multi-turn resync must work on the negative side of zero");

// Pure truth ledger for an ST3215 relative (mode-3) step. `acceptedPosition`
// is only the endpoint named by an addressed write ACK; it is deliberately not
// exposed as completed position until a live non-zero countdown has been seen
// and subsequently reaches zero. `referencePosition` is the last position the
// wire evidence supports and remains the mode-0 wrap-reconciliation reference
// if authority disappears mid-step.
enum class MultiTurnTruthPhase : uint8_t {
  UNSEEDED,
  COMPLETE,
  AWAITING_COUNTDOWN,
  COUNTDOWN_LIVE,
  RESYNC_REQUIRED,
};

enum class MultiTurnCountdownResult : uint8_t {
  PENDING,
  COMPLETE,
  RESYNC_REQUIRED,
};

struct MultiTurnTruthState {
  int32_t acceptedPosition = 0;
  int32_t referencePosition = 0;
  int32_t remaining = 0;
  MultiTurnTruthPhase phase = MultiTurnTruthPhase::UNSEEDED;
  uint32_t resyncCount = 0;
};

void seedMultiTurnTruth(MultiTurnTruthState& state, int32_t position);
void forgetMultiTurnTruth(MultiTurnTruthState& state);
bool beginAckedMultiTurnStep(MultiTurnTruthState& state, int32_t step);
MultiTurnCountdownResult observeMultiTurnCountdown(
    MultiTurnTruthState& state, int32_t remaining,
    uint16_t arrivedTolerance);
void invalidateMultiTurnTruth(MultiTurnTruthState& state);
bool resyncMultiTurnTruthFromWrappedEncoder(MultiTurnTruthState& state,
                                            int32_t reference,
                                            uint16_t wrapped,
                                            int32_t& actual);
bool multiTurnTruthComplete(const MultiTurnTruthState& state);
bool multiTurnStepOutstanding(const MultiTurnTruthState& state);
bool multiTurnCountdownObserved(const MultiTurnTruthState& state);
bool multiTurnResyncRequired(const MultiTurnTruthState& state);
bool multiTurnDeltaToTarget(const MultiTurnTruthState& state, int32_t target,
                            int32_t& delta);

constexpr uint32_t TORQUE_OFF_RETRY_MS = 25;
constexpr uint32_t LEASE_SUPERVISION_INTERVAL_MS = 25;

static_assert(MAX_STATUS_FIXED_JSON_BYTES +
                      MAX_TRACKED_SERVOS * MAX_SERVO_JSON_BYTES +
                      MAX_RESPONSE_FRAME_OVERHEAD_BYTES <=
                  MAX_RESPONSE_BYTES,
              "Worst-case STATUS response must fit the fixed response buffer");

constexpr uint8_t SCS_INSTRUCTION_PING = 0x01;
constexpr uint8_t SCS_INSTRUCTION_READ = 0x02;
constexpr uint8_t SCS_INSTRUCTION_WRITE = 0x03;
constexpr uint8_t SCS_INSTRUCTION_SYNC_WRITE = 0x83;

enum class Operation : uint8_t {
  HELLO,
  HEARTBEAT,
  STATUS,
  SCAN,
  ASSIGN_ID,
  SET_POSITION_MODE,
  CAPTURE,
  HOLD_SET,
  TORQUE_LEASE,
  TORQUE_OFF,
  PREPARE_NUDGE,
  EXECUTE_NUDGE,
  STOP,
  RESET,
  ODO_ZERO,
  ODO_READ,
  REG_READ,
  REG_WRITE,
  CONFIG,
  MOVE,
  MOVE_SET,
  FOLLOW_SET,
  FOLLOW_READ,
  FAMILY,
  MULTITURN,
};

enum class ParseResult : uint8_t {
  OK,
  BAD_FORMAT,
  BAD_VERSION,
  BAD_SEQUENCE,
  UNKNOWN_OPERATION,
  TOO_MANY_ARGUMENTS,
};

struct ParsedCommand {
  uint32_t sequence = 0;
  Operation operation = Operation::HELLO;
  uint8_t argumentCount = 0;
  char* arguments[MAX_ARGUMENTS] = {};
};

uint8_t scsChecksum(const uint8_t* body, std::size_t length);
std::size_t buildScsInstruction(uint8_t id, uint8_t instruction,
                                const uint8_t* parameters,
                                std::size_t parameterCount, uint8_t* output,
                                std::size_t outputCapacity);

ParseResult parseCommandLine(char* line, std::size_t length,
                             ParsedCommand& command);
const char* operationName(Operation operation);
bool parseUInt32(const char* text, uint32_t& value);
bool parseInt32(const char* text, int32_t& value);
bool validServoId(uint32_t id);
bool validScanRange(uint32_t minimum, uint32_t maximum);
bool validTorqueLeaseMs(uint32_t leaseMs);
bool validNudge(int32_t deltaTicks, uint32_t speed, uint32_t acceleration,
                uint32_t maxDelta = NUDGE_MAX_DELTA_TICKS,
                uint32_t maxSpeed = NUDGE_MAX_SPEED,
                uint32_t maxAccel = NUDGE_MAX_ACCEL);
uint32_t predictedNudgeMotionMs(int32_t deltaTicks, uint32_t speed);
bool validNudgeTiming(int32_t deltaTicks, uint32_t speed,
                      uint32_t budgetMs = NUDGE_PLANNED_MOTION_BUDGET_MS);
bool validProposalToken(const char* token);
int16_t decodeSignedMagnitude16(uint16_t raw);
int16_t decodeServoLoad(uint16_t raw);
bool validNudgeCompletion(uint16_t start, uint16_t target, uint16_t actual,
                          int32_t requestedDelta,
                          uint16_t toleranceTicks = 2);
// `expectedMode` is 0 for every servo except one armed for multi-turn, which
// legitimately runs in step mode (3) while driven and held.
bool validActiveLeaseTelemetry(bool readSucceeded, bool torqueOn,
                               bool operatingModeKnown,
                               uint8_t operatingMode, uint8_t servoFault,
                               uint8_t statusError, bool contractValid,
                               uint8_t expectedMode = 0);
bool analyzePositionSamples4096(const uint16_t* samples, std::size_t count,
                                uint16_t& median, uint16_t& variation);
bool reconcileCompletedScanIds(const uint8_t* existing,
                               std::size_t existingCount, uint8_t minimum,
                               uint8_t maximum, const uint8_t* discovered,
                               std::size_t discoveredCount, uint8_t* output,
                               std::size_t outputCapacity,
                               std::size_t& outputCount);

enum class SafetyEvent : uint8_t {
  NONE,
  LEASE_EXPIRED,
  WATCHDOG_STOP,
  TORQUE_OFF_RETRY,
};

class SafetyStateMachine {
 public:
  void begin(uint32_t nowMs);
  bool bootTorqueOffRequired() const;
  void markBootTorqueOffAttempted();

  void heartbeat(uint32_t nowMs);
  bool heartbeatSeen() const;
  bool heartbeatFresh(uint32_t nowMs) const;
  uint32_t hostAgeMs(uint32_t nowMs) const;

  // Arm this obligation immediately before any torque-enable write. It remains
  // live until addressed register-40 read-back proves that exact servo is off.
  bool armTorqueOffObligation(uint8_t servoId);
  bool queueTorqueOffObligation(uint8_t servoId);
  bool startTorqueLease(uint8_t servoId, uint32_t leaseMs, uint32_t nowMs);
  bool torqueLeaseActive(uint32_t nowMs) const;
  bool setHoldSet(const uint8_t* servoIds, uint8_t servoCount,
                  uint32_t leaseMs, uint32_t nowMs);
  bool holdSetActive(uint32_t nowMs) const;
  bool holdAuthorizedFor(uint8_t servoId, uint32_t nowMs) const;
  uint8_t holdSetCount() const;
  uint8_t holdSetServoId(uint8_t index) const;
  uint32_t holdSetRemainingMs(uint32_t nowMs) const;
  bool removeHoldServo(uint8_t servoId);
  bool torqueAuthorizedFor(uint8_t servoId, uint32_t nowMs) const;
  uint8_t torqueLeaseId() const;
  uint32_t torqueLeaseRemainingMs(uint32_t nowMs) const;
  bool torqueOffPending() const;
  uint16_t torqueOffPendingCount() const;
  bool torqueOffPendingFor(uint8_t servoId) const;
  uint8_t torqueOffServoId() const;
  bool nextUnauthorizedTorqueOff(uint32_t nowMs, uint8_t& servoId) const;
  bool torqueOffRetryDue(uint32_t nowMs) const;
  void noteTorqueOffAttempt(uint8_t servoId, uint32_t nowMs);
  bool markTorqueOffConfirmed(uint8_t servoId);

  SafetyEvent tick(uint32_t nowMs);
  // Automatic faults revoke torque authority without creating an operator
  // latch. Explicit operator or hardware stop requests use latchStop().
  void revokeAuthority();
  void latchStop();
  bool stopped() const;
  bool resetInspected(uint32_t nowMs);

 private:
  bool bootTorqueOffRequired_ = true;
  bool heartbeatSeen_ = false;
  uint32_t lastHeartbeatMs_ = 0;
  bool leaseActive_ = false;
  uint8_t leaseServoId_ = 0;
  uint32_t leaseStartedMs_ = 0;
  uint32_t leaseDurationMs_ = 0;
  bool holdSetActive_ = false;
  uint8_t holdSetIds_[MAX_HOLD_SERVOS] = {};
  uint8_t holdSetCount_ = 0;
  uint32_t holdSetStartedMs_ = 0;
  uint32_t holdSetDurationMs_ = 0;
  uint8_t torqueOffIds_[MAX_TORQUE_OFF_OBLIGATIONS] = {};
  uint16_t torqueOffCount_ = 0;
  uint16_t torqueOffRetryCursor_ = 0;
  bool torqueOffAttemptSeen_ = false;
  uint32_t lastTorqueOffAttemptMs_ = 0;
  bool stopped_ = false;
};

}  // namespace armhat
