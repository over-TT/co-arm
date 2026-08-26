#pragma once

#include <Arduino.h>

#include "ArmHatProtocol.h"

namespace armhat {

constexpr uint8_t ST3215_BROADCAST_ID = 0xFE;
constexpr uint16_t ST3215_POSITION_MAX = 4095;
// Native ST3215 extended-position mode: angle limits 0/0, Phase register bit 4
// set, resolution 1 and operating mode 0. Both the absolute goal and present
// position use little-endian sign-magnitude (bit 15 is the sign). The servo's
// own position controller closes the loop across multiple motor turns.
constexpr int32_t ST3215_MULTI_TURN_MAX = 30719;
constexpr uint16_t ST3215_SIGN_BIT = 0x8000;

// Legacy mode-3 constants remain only for the isolated historical regression
// harness. The sketch never runs that loop and no public command enters mode 3.
constexpr int32_t MULTI_TURN_ARRIVED_TICKS = 32;
constexpr uint32_t MULTI_TURN_STEP_INTERVAL_MS = 25;
// An addressed position write can be ACKed before register 56 publishes the
// new relative-step countdown. Poll briefly instead of treating the first zero
// (previously sampled after only 1.2 ms) as a lost command.
constexpr uint32_t MULTI_TURN_COUNTDOWN_START_GRACE_MS = 50;
constexpr uint32_t MULTI_TURN_COUNTDOWN_POLL_US = 1500;
// A remaining that is nonzero and frozen this long means the joint is blocked;
// stop commanding it. The estimate stays honest either way.
constexpr uint32_t MULTI_TURN_STALL_MS = 700;

// A grouped SYNC_WRITE has no ACK, and a servo can still be finishing that
// broadcast when the first addressed goal-register read arrives. High-current
// Shoulder/Elbow starts can also make the first few reply slots noisy. Allow a
// short bounded verification window, but accept the command only after an exact
// readback; a persistent mismatch still revokes authority and proves torque off.
// The physical 2400/80 Shoulder + Elbow start can keep the first addressed
// reply slot unavailable for longer than the former ~13.5 ms total window.
// Four reads with 5 ms gaps extend only the ambiguous path to ~27.5 ms; a
// normal first-read success is unchanged, and the grouped motion write is
// still issued exactly once.
constexpr uint8_t POSITION_VERIFY_ATTEMPTS = 4;
constexpr uint32_t POSITION_VERIFY_RETRY_DELAY_US = 5000;
// Full telemetry is likewise allowed a few bounded attempts while a servo is
// moving. This mirrors the normal lease supervisor's transient-sample policy
// without ever retrying or duplicating the motion command itself.
constexpr uint8_t FOLLOW_FEEDBACK_ATTEMPTS = 3;
constexpr uint32_t FOLLOW_FEEDBACK_RETRY_DELAY_US = 2000;

enum class BusResult : uint8_t {
  OK,
  TIMEOUT,
  CORRUPT,
  SERVO_ERROR,
  INVALID,
};

enum class TorqueState : uint8_t {
  OFF,
  ON,
  UNKNOWN,
};

// Two servo families share this bus. Packet framing, PING, the ID register and
// the one-byte torque flag are identical, which is why an SC09 scans and re-IDs
// correctly against STS code. Everything else below is where they part.
enum class ServoFamily : uint8_t {
  STS,  // ST3215 and the rest of the STS/SMS line
  SCS,  // SC09 and the rest of the SCS/SCSCL line
};

struct ServoDialect {
  bool bigEndian;           // every 16-bit field is byte-swapped between families
  uint16_t positionMax;     // 4095 over 360 deg, vs 1023 over 300 deg
  uint8_t lockRegister;     // 55 on STS; 48 on SCS, where 55 is not the lock
  uint8_t goalRegister;     // first byte of the contiguous goal block
  uint8_t goalBytes;
  uint8_t telemetryBytes;   // 56.. up to the last register the family defines
  bool hasAcceleration;     // STS reg 41; SCS has no acceleration at all
  bool hasOperatingMode;    // STS reg 33; SCS is structurally position-only
  bool hasCurrent;          // STS regs 69-70; absent on SCS
};

const ServoDialect& dialectFor(ServoFamily family);

// What the line was doing when a drain gave up. The distinction this exists to
// draw: a line held low reads as an unending run of 0x00 (a wiring or power
// fault, and no protocol change can fix it), whereas two servos talking over
// each other leaves recognisable 0xFF-framed wreckage.
struct BusNoise {
  uint32_t bytes = 0;
  uint8_t sample[8] = {};
  uint8_t sampleCount = 0;
  bool allZero = false;
  bool allOnes = false;
};

// In native extended mode the ST3215 publishes one signed absolute coordinate.
// Sample it often enough to keep the dashboard current without flooding the bus;
// ordinary single-turn/SCS joints keep their existing wrapped telemetry path.
constexpr uint32_t ODOMETER_SAMPLE_INTERVAL_MS = 10;
// A full 4096-count coordinate collapse inside this window cannot be a real
// shaft revolution on the commissioned Base. It is the signature of the
// servo's volatile turn origin resetting while the HAT stayed alive.
constexpr uint32_t NATIVE_MULTI_TURN_REVOLUTION_MIN_MS = 250;
// A missed encoder reply makes the observation stale; it does not prove that a
// wrap was crossed. Normal STATUS work can occupy this bus for well over 100 ms,
// so wall-clock gaps must never erase an otherwise valid counted position. A
// real continuity loss is established by controller reboot, a freshly confirmed
// wrong operating mode, an invalid register value, or an unrecoverable step
// state. A missing mode sample is stale evidence, not proof of lost continuity.
constexpr int32_t ODOMETER_WRAP_THRESHOLD_TICKS = 2048;
constexpr int32_t ODOMETER_MAX_REVOLUTIONS = 64;

struct ServoTelemetry {
  uint8_t id = 0;
  bool online = false;
  bool fresh = false;
  bool sampled = false;
  uint16_t rawPosition = 0;
  int16_t speed = 0;
  int16_t load = 0;
  uint8_t voltage = 0;
  uint8_t temperature = 0;
  uint8_t moving = 0;
  uint16_t currentRaw = 0;
  uint8_t error = 0;
  uint8_t statusError = 0;
  bool operatingModeKnown = false;
  uint8_t operatingMode = 0;
  ServoFamily family = ServoFamily::STS;
  TorqueState torque = TorqueState::UNKNOWN;
  uint32_t sampledAtMs = 0;
  // Native signed absolute position split into quotient/remainder for the
  // existing ODO wire shape. `valid` is established only by explicit ODO_ZERO
  // and is cleared by controller/servo continuity loss.
  bool odometerTracking = false;
  bool odometerValid = false;
  int32_t revolutions = 0;
  uint16_t odometerLastRaw = 0;
  uint32_t odometerSampledAtMs = 0;
  // A failed native mode-0 position read opens an observation gap. The next
  // exact whole-turn collapse remains suspicious even after the normal short
  // discontinuity window has elapsed.
  bool nativeSampleMissed = false;
  // Retained wire-compatible state; native absolute mode keeps it COMPLETE at
  // the latest verified encoder coordinate and never exposes countdown phases.
  MultiTurnTruthState multiTurnTruth{};
};

class St3215Bus {
 public:
  explicit St3215Bus(HardwareSerial& serial);

  void begin(uint32_t baud, int8_t rxPin, int8_t txPin);
  bool broadcastTorqueOff();
  // `outcome` distinguishes silence from garbage. Garbage back from a ping is
  // the signature of two servos answering one id.
  bool ping(uint8_t id, uint32_t timeoutUs = 1500, BusResult* outcome = nullptr);
  BusResult readRegisters(uint8_t id, uint8_t address, uint8_t length,
                          uint8_t* output, uint8_t& servoError,
                          uint32_t timeoutUs = 3000);
  bool writeRegister(uint8_t id, uint8_t address, const uint8_t* values,
                     uint8_t valueCount);
  bool setTorque(uint8_t id, bool enabled);
  BusResult readTorque(uint8_t id, bool& enabled, uint8_t& servoError);
  BusResult readTelemetry(uint8_t id, ServoTelemetry& telemetry);
  // Signed, because a multi-turn joint legitimately sits either side of its
  // own zero. A single-turn servo still rejects anything outside 0..positionMax.
  bool writePosition(uint8_t id, int32_t position, uint16_t speed,
                     uint8_t acceleration);
  struct PositionTarget {
    uint8_t id = 0;
    int32_t position = 0;
    uint16_t speed = 1;
    uint8_t acceleration = 1;
  };
  // SYNC_WRITE has no latent register state: every member in one family starts
  // from one broadcast packet. Mixed families require two packets because
  // their goal address/length/endianness differ.
  bool syncWritePositions(ServoFamily family, const PositionTarget* targets,
                          uint8_t targetCount);
  bool verifyPositionCommand(uint8_t id, int32_t position, uint16_t speed,
                             uint8_t acceleration);
  bool writeId(uint8_t oldId, uint8_t newId);
  bool writePositionMode(uint8_t id);
  bool writeOperatingMode(uint8_t id, uint8_t mode);
  BusResult readPositionLimits(uint8_t id, uint16_t& minimum,
                               uint16_t& maximum, uint8_t& servoError);
  // True once a drain gave up with the line still busy. Two servos sharing an
  // id answer together and drive the bus against each other, so bytes never
  // stop arriving; that is a jam, and it is reportable rather than fatal.
  bool jammed() const { return jammed_; }
  const BusNoise& noise() const { return noise_; }
  // A jam is a statement about the line right now, not a permanent verdict. It
  // used to be latched for the life of the boot, so one transient burst made
  // every later report claim a jam that had long since cleared.
  void clearJam();
  // Family is declared by the host, never sniffed off the wire: the model
  // numbers that would tell an SC09 from an ST3215 are undocumented, and a
  // wrong guess writes SCS registers into an arm servo.
  void setFamily(uint8_t id, ServoFamily family);
  ServoFamily familyOf(uint8_t id) const;
  // Multi-turn is a property of how this servo has been configured, so the
  // encoding of every goal depends on it. Declared by the host alongside the
  // angle-limit write that actually switches the servo over.
  void setMultiTurn(uint8_t id, bool enabled);
  bool isMultiTurn(uint8_t id) const;
  const ServoDialect& dialect(uint8_t id) const {
    return dialectFor(familyOf(id));
  }
  bool writeLock(uint8_t id, bool locked);
  BusResult readLock(uint8_t id, bool& locked, uint8_t& servoError);
  BusResult readOperatingMode(uint8_t id, uint8_t& mode,
                              uint8_t& servoError);

 private:
  // Returns false when the line was still busy at the cut-off. Unbounded, this
  // was the hang: it runs inside every send, and a jammed bus never satisfied
  // available() == 0.
  bool discardInput();
  bool sendInstruction(uint8_t id, uint8_t instruction,
                       const uint8_t* parameters, uint8_t parameterCount);
  BusResult receiveStatus(uint8_t expectedId, uint8_t* parameters,
                          uint8_t parameterCapacity, uint8_t& parameterCount,
                          uint8_t& servoError, uint32_t timeoutUs);
  bool readByteUntil(uint8_t& value, uint32_t startedUs, uint32_t timeoutUs);

  HardwareSerial& serial_;
  bool jammed_ = false;
  BusNoise noise_;
  uint8_t scsIds_[MAX_TRACKED_SERVOS] = {};
  uint8_t scsIdCount_ = 0;
  uint8_t multiTurnIds_[MAX_TRACKED_SERVOS] = {};
  uint8_t multiTurnIdCount_ = 0;
};

// Motion policy the host owns at runtime. Boot values are conservative; the Pi
// sets whatever this particular arm needs. Nothing here is a safety fail-safe —
// the watchdog, torque lease, STOP latch and boot torque-off are not settable.
struct MotionPolicy {
  uint16_t maxDeltaTicks = 64;
  uint16_t motionBudgetMs = 350;
  uint16_t positionToleranceTicks = 8;
  uint16_t executionTimeoutMs = 450;
  uint16_t maxSpeed = 256;
  uint16_t maxAccel = 20;
  // Consecutive unhealthy supervision samples tolerated before authority is
  // revoked. A moving servo can return one transient fault/status byte; treating
  // a single blip as a safety event latched STOP on every real move. Sustained
  // faults still revoke, just not on the first sample.
  uint16_t supervisionFaultTolerance = 3;
};

class ArmHatRuntime {
 public:
  ArmHatRuntime(Stream& host, HardwareSerial& servoSerial);

  void begin(uint32_t servoBaud = 1000000, int8_t servoRxPin = 18,
             int8_t servoTxPin = 19);
  void poll();
  void tick();
  // Called from loop(), deliberately not from tick(): tick() also runs before
  // every host command and must stay free of extra bus traffic.
  void sampleOdometers();
  // Historical Mode-3 regression seam only; the production sketch does not call
  // it and no public command can populate its private goal list.
  void stepMultiTurnGoals();

 private:
  struct Proposal {
    bool active = false;
    char token[32] = {};
    uint8_t servoId = 0;
    uint16_t from = 0;
    uint16_t target = 0;
    int16_t delta = 0;
    uint16_t speed = 0;
    uint8_t acceleration = 0;
    uint32_t createdAtMs = 0;
  };

  // A goal in the counted frame, delivered as relative steps (mode 3). Since
  // writes accumulate exactly, sizing every write from completed truth
  // converges in a single command; the loop only watches for stalls after.
  struct MultiTurnGoal {
    uint8_t servoId = 0;
    bool active = false;
    int32_t target = 0;
    uint16_t speed = 1;
    uint8_t acceleration = 1;
    uint32_t lastStepMs = 0;
    int32_t lastRemaining = 0;
    uint32_t lastProgressMs = 0;
  };

  struct HoldGoal {
    uint8_t servoId = 0;
    // The position the hold was captured at, in the servo's own goal frame:
    // wrapping counts for a single-turn joint, wrap-counted for a multi-turn
    // one. Writing the wrapping value back to a multi-turn servo names an
    // absolute position a whole turn away, which commanded the base to unwind
    // one revolution every time torque was taken.
    int32_t holdPosition = 0;
  };

  struct MoveSetGoal {
    uint8_t servoId = 0;
    int32_t goal = 0;
    uint16_t speed = 1;
    uint8_t acceleration = 1;
  };

  void processLine();
  void dispatch(const ParsedCommand& command);
  void handleHello(uint32_t sequence, const ParsedCommand& command);
  void handleHeartbeat(uint32_t sequence, const ParsedCommand& command);
  void handleStatus(uint32_t sequence, const ParsedCommand& command);
  void handleScan(uint32_t sequence, const ParsedCommand& command);
  void handleAssignId(uint32_t sequence, const ParsedCommand& command);
  void handleSetPositionMode(uint32_t sequence,
                             const ParsedCommand& command);
  void handleCapture(uint32_t sequence, const ParsedCommand& command);
  void handleHoldSet(uint32_t sequence, const ParsedCommand& command);
  void handleTorqueLease(uint32_t sequence, const ParsedCommand& command);
  void handleTorqueOff(uint32_t sequence, const ParsedCommand& command);
  void handlePrepareNudge(uint32_t sequence, const ParsedCommand& command);
  void handleExecuteNudge(uint32_t sequence, const ParsedCommand& command);
  void handleStop(uint32_t sequence, const ParsedCommand& command);
  void handleReset(uint32_t sequence, const ParsedCommand& command);
  void handleOdometerZero(uint32_t sequence, const ParsedCommand& command);
  void handleOdometerRead(uint32_t sequence, const ParsedCommand& command);
  void handleRegisterRead(uint32_t sequence, const ParsedCommand& command);
  void handleRegisterWrite(uint32_t sequence, const ParsedCommand& command);
  void handleConfig(uint32_t sequence, const ParsedCommand& command);
  void handleMove(uint32_t sequence, const ParsedCommand& command);
  void handleMoveSet(uint32_t sequence, const ParsedCommand& command);
  void handleFollowSet(uint32_t sequence, const ParsedCommand& command);
  void handleFollowRead(uint32_t sequence, const ParsedCommand& command);
  void handleFamily(uint32_t sequence, const ParsedCommand& command);
  void handleMultiTurn(uint32_t sequence, const ParsedCommand& command);
  // Where a joint actually is, wraps included. Only meaningful for a multi-turn
  // servo whose odometer is armed and unbroken.
  bool multiTurnPositionOf(const ServoTelemetry& servo, int32_t& position) const;
  bool resyncMultiTurnFromEncoder(ServoTelemetry& servo);
  void updateMultiTurnEstimate(ServoTelemetry& servo, int32_t remaining,
                               uint32_t sampledAtMs);
  MultiTurnGoal* findMultiTurnGoal(uint8_t servoId);
  bool setMultiTurnGoal(uint8_t servoId, int32_t target, uint16_t speed,
                        uint8_t acceleration);
  void clearMultiTurnGoal(uint8_t servoId);
  void sendPolicyJson();
  void appendBusNoise();
  void sendOdometerJson(const ServoTelemetry& servo, uint32_t nowMs);

  ServoTelemetry* rememberServo(uint8_t id);
  ServoTelemetry* findServo(uint8_t id);
  bool reconcileCompletedScan(uint8_t minimum, uint8_t maximum,
                              const uint8_t* discovered,
                              uint8_t discoveredCount);
  bool refreshServo(uint8_t id);
  bool refreshFollowServo(uint8_t id);
  bool confirmTorqueOff(uint8_t id);
  bool confirmAllTrackedTorqueOff();
  bool torqueOffAndConfirm(uint8_t id);
  bool resolvePendingTorqueOffFor(uint8_t id);
  bool resolvePendingTorqueOff();
  bool superviseActiveLease(uint32_t nowMs);
  HoldGoal* findHoldGoal(uint8_t servoId);
  const HoldGoal* findHoldGoal(uint8_t servoId) const;
  void removeHoldGoal(uint8_t servoId);
  void clearHoldGoals();
  void failMoveSetClosed(uint32_t sequence, const MoveSetGoal* goals,
                         uint8_t goalCount, const char* phase,
                         uint8_t failedIndex,
                         uint8_t dispatchedFamilyCount = 0);
  bool stopMoveSetMembersAndConfirm(const MoveSetGoal* goals,
                                    uint8_t goalCount);
  void invalidateProposal();
  bool proposalFresh(uint32_t nowMs) const;

  bool startResponse(uint32_t sequence, bool ok, const char* errorCode);
  bool append(const char* format, ...);
  void sendResponse();
  void sendError(uint32_t sequence, const char* code,
                 const char* details = "{}");
  void sendServoFields(const ServoTelemetry& servo, uint32_t nowMs);
  void sendServoJson(const ServoTelemetry& servo, uint32_t nowMs);
  void sendStatusJson(uint32_t nowMs);
  bool telemetryContractValid(const ServoTelemetry& servo,
                              uint32_t nowMs) const;
  const char* aggregateTorqueState() const;
  const char* aggregateServosState(uint32_t nowMs) const;
  const char* busStateName() const;
  const char* motionState(uint32_t nowMs) const;
  const char* stateName(uint32_t nowMs) const;
  bool safetyFaultActive() const;
  bool operatorClearRequired() const;
  const char* safetyStopReasonName() const;
  static const char* torqueName(TorqueState state);
  static const char* parseErrorCode(ParseResult result);
  static bool requireNoArguments(const ParsedCommand& command);

  Stream& host_;
  St3215Bus bus_;
  SafetyStateMachine safety_;
  ServoTelemetry servos_[MAX_TRACKED_SERVOS] = {};
  uint8_t servoCount_ = 0;
  Proposal proposal_;
  HoldGoal holdGoals_[MAX_HOLD_SERVOS] = {};
  uint8_t holdGoalCount_ = 0;
  MultiTurnGoal multiTurnGoals_[MAX_HOLD_SERVOS] = {};
  uint64_t bootId_ = 0;
  uint32_t proposalCounter_ = 0;
  uint32_t evidenceCounter_ = 0;
  char controllerId_[32] = {};
  bool safetyFault_ = false;
  // A MOVE_SET ambiguity requires a human-inspected RESET even if the Pi
  // process reconnects. This remains in HAT RAM for the complete controller
  // boot and is cleared only by a successful strict RESET INSPECTED.
  bool operatorInspectionRequired_ = false;
  // Distinguishes a deliberate host STOP from a controller-detected safety
  // fault. Both survive Pi restart and require the same explicit clear path.
  bool explicitStopLatched_ = false;
  bool bootTorqueOffSent_ = false;
  // Deferred out of begin() so the host link is live before the bus is touched.
  // A controller that cannot be talked to cannot report why it is unhappy.
  bool bootTorqueOffPending_ = true;
  bool inventoryScanned_ = false;
  bool busResponseSeen_ = false;
  bool busDegraded_ = false;
  bool leaseSupervisionSeen_ = false;
  uint32_t lastLeaseSupervisionMs_ = 0;
  uint32_t lastOdometerSampleMs_ = 0;
  MotionPolicy policy_;
  uint16_t supervisionFaults_ = 0;
  char request_[MAX_REQUEST_BYTES] = {};
  std::size_t requestLength_ = 0;
  bool requestOverflow_ = false;
  char response_[MAX_RESPONSE_BYTES] = {};
  std::size_t responseLength_ = 0;
};

}  // namespace armhat
