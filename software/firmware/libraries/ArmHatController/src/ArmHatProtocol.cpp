#include "ArmHatProtocol.h"

#include <climits>
#include <cstring>

namespace armhat {
namespace {

struct OperationEntry {
  const char* name;
  Operation operation;
};

constexpr OperationEntry OPERATIONS[] = {
    {"HELLO", Operation::HELLO},
    {"HEARTBEAT", Operation::HEARTBEAT},
    {"STATUS", Operation::STATUS},
    {"SCAN", Operation::SCAN},
    {"ASSIGN_ID", Operation::ASSIGN_ID},
    {"SET_POSITION_MODE", Operation::SET_POSITION_MODE},
    {"CAPTURE", Operation::CAPTURE},
    {"HOLD_SET", Operation::HOLD_SET},
    {"TORQUE_LEASE", Operation::TORQUE_LEASE},
    {"TORQUE_OFF", Operation::TORQUE_OFF},
    {"PREPARE_NUDGE", Operation::PREPARE_NUDGE},
    {"EXECUTE_NUDGE", Operation::EXECUTE_NUDGE},
    {"STOP", Operation::STOP},
    {"RESET", Operation::RESET},
    {"ODO_ZERO", Operation::ODO_ZERO},
    {"ODO_READ", Operation::ODO_READ},
    {"REG_READ", Operation::REG_READ},
    {"REG_WRITE", Operation::REG_WRITE},
    {"CONFIG", Operation::CONFIG},
    {"MOVE", Operation::MOVE},
    {"MOVE_SET", Operation::MOVE_SET},
    {"FOLLOW_SET", Operation::FOLLOW_SET},
    {"FOLLOW_READ", Operation::FOLLOW_READ},
    {"FAMILY", Operation::FAMILY},
    {"MULTITURN", Operation::MULTITURN},
};

bool isSpace(char character) {
  return character == ' ' || character == '\t';
}

bool nextToken(char*& cursor, char* end, char*& token) {
  while (cursor < end && isSpace(*cursor)) {
    *cursor++ = '\0';
  }
  if (cursor >= end || *cursor == '\0') {
    token = nullptr;
    return false;
  }
  token = cursor;
  while (cursor < end && *cursor != '\0' && !isSpace(*cursor)) {
    ++cursor;
  }
  if (cursor < end) {
    *cursor++ = '\0';
  }
  return true;
}

bool findOperation(const char* name, Operation& operation) {
  for (const auto& entry : OPERATIONS) {
    if (std::strcmp(name, entry.name) == 0) {
      operation = entry.operation;
      return true;
    }
  }
  return false;
}

uint32_t elapsed(uint32_t nowMs, uint32_t thenMs) {
  return nowMs - thenMs;
}

// Encoder wrapping is useful only while comparing read-only CAPTURE samples.
// Motion commands never wrap: their targets and observations use direct signed
// subtraction and must stay inside the physical 0..4095 working interval.
int16_t signedCaptureDelta4096(uint16_t from, uint16_t to) {
  int32_t delta =
      (static_cast<int32_t>(to & 0x0FFFu) -
       static_cast<int32_t>(from & 0x0FFFu)) &
      0x0FFF;
  if (delta >= 2048) {
    delta -= 4096;
  }
  return static_cast<int16_t>(delta);
}

}  // namespace

uint8_t scsChecksum(const uint8_t* body, std::size_t length) {
  uint8_t sum = 0;
  for (std::size_t index = 0; index < length; ++index) {
    sum = static_cast<uint8_t>(sum + body[index]);
  }
  return static_cast<uint8_t>(~sum);
}

std::size_t buildScsInstruction(uint8_t id, uint8_t instruction,
                                const uint8_t* parameters,
                                std::size_t parameterCount, uint8_t* output,
                                std::size_t outputCapacity) {
  if (output == nullptr || parameterCount > 253 ||
      outputCapacity < parameterCount + 6 ||
      (parameterCount > 0 && parameters == nullptr)) {
    return 0;
  }

  output[0] = 0xFF;
  output[1] = 0xFF;
  output[2] = id;
  output[3] = static_cast<uint8_t>(parameterCount + 2);
  output[4] = instruction;
  if (parameterCount > 0) {
    std::memcpy(output + 5, parameters, parameterCount);
  }
  output[parameterCount + 5] =
      scsChecksum(output + 2, parameterCount + 3);
  return parameterCount + 6;
}

ParseResult parseCommandLine(char* line, std::size_t length,
                             ParsedCommand& command) {
  command = ParsedCommand{};
  if (line == nullptr || length == 0 || length >= MAX_REQUEST_BYTES) {
    return ParseResult::BAD_FORMAT;
  }

  while (length > 0 &&
         (line[length - 1] == '\r' || line[length - 1] == '\n')) {
    line[--length] = '\0';
  }
  if (length == 0) {
    return ParseResult::BAD_FORMAT;
  }
  for (std::size_t index = 0; index < length; ++index) {
    const auto value = static_cast<unsigned char>(line[index]);
    if (value < 0x20 || value > 0x7E) {
      return ParseResult::BAD_FORMAT;
    }
  }

  char* cursor = line;
  char* const end = line + length;
  char* token = nullptr;
  if (!nextToken(cursor, end, token) || std::strcmp(token, "A1") != 0) {
    return token == nullptr ? ParseResult::BAD_FORMAT
                            : ParseResult::BAD_VERSION;
  }
  if (!nextToken(cursor, end, token) ||
      !parseUInt32(token, command.sequence)) {
    return ParseResult::BAD_SEQUENCE;
  }
  if (!nextToken(cursor, end, token)) {
    return ParseResult::BAD_FORMAT;
  }
  if (!findOperation(token, command.operation)) {
    return ParseResult::UNKNOWN_OPERATION;
  }

  while (nextToken(cursor, end, token)) {
    if (command.argumentCount >= MAX_ARGUMENTS) {
      return ParseResult::TOO_MANY_ARGUMENTS;
    }
    command.arguments[command.argumentCount++] = token;
  }
  return ParseResult::OK;
}

const char* operationName(Operation operation) {
  for (const auto& entry : OPERATIONS) {
    if (entry.operation == operation) {
      return entry.name;
    }
  }
  return "UNKNOWN";
}

bool parseUInt32(const char* text, uint32_t& value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  uint32_t result = 0;
  for (const char* cursor = text; *cursor != '\0'; ++cursor) {
    if (*cursor < '0' || *cursor > '9') {
      return false;
    }
    const uint32_t digit = static_cast<uint32_t>(*cursor - '0');
    if (result > (UINT32_MAX - digit) / 10u) {
      return false;
    }
    result = result * 10u + digit;
  }
  value = result;
  return true;
}

bool parseInt32(const char* text, int32_t& value) {
  if (text == nullptr || *text == '\0') {
    return false;
  }
  bool negative = false;
  if (*text == '-') {
    negative = true;
    ++text;
  }
  if (*text == '\0') {
    return false;
  }

  uint32_t magnitude = 0;
  if (!parseUInt32(text, magnitude)) {
    return false;
  }
  const uint32_t limit = negative ? 2147483648u : 2147483647u;
  if (magnitude > limit) {
    return false;
  }
  if (negative && magnitude == 2147483648u) {
    value = INT32_MIN;
  } else {
    value = negative ? -static_cast<int32_t>(magnitude)
                     : static_cast<int32_t>(magnitude);
  }
  return true;
}

bool validServoId(uint32_t id) { return id <= 253; }

bool validScanRange(uint32_t minimum, uint32_t maximum) {
  return validServoId(minimum) && validServoId(maximum) && minimum <= maximum;
}

bool validTorqueLeaseMs(uint32_t leaseMs) {
  return leaseMs >= TORQUE_LEASE_MIN_MS && leaseMs <= TORQUE_LEASE_MAX_MS;
}

bool validNudge(int32_t deltaTicks, uint32_t speed, uint32_t acceleration,
                uint32_t maxDelta, uint32_t maxSpeed, uint32_t maxAccel) {
  const int32_t bound = static_cast<int32_t>(maxDelta);
  return deltaTicks != 0 && deltaTicks >= -bound && deltaTicks <= bound &&
         speed >= 1 && speed <= maxSpeed && acceleration >= 1 &&
         acceleration <= maxAccel;
}

uint32_t predictedNudgeMotionMs(int32_t deltaTicks, uint32_t speed) {
  if (speed == 0) {
    return UINT32_MAX;
  }
  const uint32_t magnitude =
      deltaTicks < 0
          ? static_cast<uint32_t>(-static_cast<int64_t>(deltaTicks))
          : static_cast<uint32_t>(deltaTicks);
  const uint64_t numerator =
      static_cast<uint64_t>(magnitude) * NUDGE_MS_PER_TICK_AT_SPEED_1;
  const uint64_t predicted = (numerator + speed - 1u) / speed;
  return predicted > UINT32_MAX ? UINT32_MAX
                                : static_cast<uint32_t>(predicted);
}

bool validNudgeTiming(int32_t deltaTicks, uint32_t speed, uint32_t budgetMs) {
  return deltaTicks != 0 && speed >= 1 &&
         predictedNudgeMotionMs(deltaTicks, speed) <= budgetMs;
}

bool validProposalToken(const char* token) {
  if (token == nullptr || *token == '\0') {
    return false;
  }
  std::size_t length = 0;
  for (; token[length] != '\0'; ++length) {
    const char value = token[length];
    const bool valid = (value >= 'A' && value <= 'Z') ||
                       (value >= 'a' && value <= 'z') ||
                       (value >= '0' && value <= '9') || value == '_' ||
                       value == '-';
    if (!valid || length >= 31) {
      return false;
    }
  }
  return length > 0;
}

int16_t decodeSignedMagnitude16(uint16_t raw) {
  const int16_t magnitude = static_cast<int16_t>(raw & 0x7FFFu);
  return (raw & 0x8000u) != 0 ? static_cast<int16_t>(-magnitude) : magnitude;
}

// ST3215 present load is a 10-bit magnitude with the direction in bit 10, NOT a
// bit-15 signed magnitude. Decoding it the generic way turned any load in the
// negative direction into 1024+, which blew the `abs(load) <= 1000` telemetry
// contract and made lease supervision revoke authority the instant the joint
// moved that way. Proven by moving one servo the same distance both ways: the
// direction with the bit clear completed, the other was cut short every time.
int16_t decodeServoLoad(uint16_t raw) {
  const int16_t magnitude = static_cast<int16_t>(raw & 0x03FFu);
  return (raw & 0x0400u) != 0 ? static_cast<int16_t>(-magnitude) : magnitude;
}

bool validNudgeCompletion(uint16_t start, uint16_t target, uint16_t actual,
                          int32_t requestedDelta, uint16_t toleranceTicks) {
  if (requestedDelta == 0 ||
      static_cast<int32_t>(start) + requestedDelta !=
          static_cast<int32_t>(target)) {
    return false;
  }
  const int32_t measuredDelta =
      static_cast<int32_t>(actual) - static_cast<int32_t>(start);
  if ((requestedDelta > 0 && measuredDelta <= 0) ||
      (requestedDelta < 0 && measuredDelta >= 0)) {
    return false;
  }
  int32_t error = static_cast<int32_t>(actual) - static_cast<int32_t>(target);
  if (error < 0) {
    error = -error;
  }
  return error <= toleranceTicks;
}

bool validActiveLeaseTelemetry(bool readSucceeded, bool torqueOn,
                               bool operatingModeKnown,
                               uint8_t operatingMode, uint8_t servoFault,
                               uint8_t statusError, bool contractValid,
                               uint8_t expectedMode) {
  return readSucceeded && torqueOn && operatingModeKnown &&
         operatingMode == expectedMode && servoFault == 0 &&
         statusError == 0 && contractValid;
}

bool analyzePositionSamples4096(const uint16_t* samples, std::size_t count,
                                uint16_t& median, uint16_t& variation) {
  if (samples == nullptr || count == 0 || count > 16 || (count % 2) == 0) {
    return false;
  }
  int32_t unwrapped[16] = {};
  const uint16_t reference = samples[0];
  if (reference > 4095) {
    return false;
  }
  for (std::size_t index = 0; index < count; ++index) {
    if (samples[index] > 4095) {
      return false;
    }
    unwrapped[index] = static_cast<int32_t>(reference) +
                       signedCaptureDelta4096(reference, samples[index]);
  }
  for (std::size_t left = 0; left < count; ++left) {
    for (std::size_t right = left + 1; right < count; ++right) {
      if (unwrapped[right] < unwrapped[left]) {
        const int32_t temporary = unwrapped[left];
        unwrapped[left] = unwrapped[right];
        unwrapped[right] = temporary;
      }
    }
  }
  variation = static_cast<uint16_t>(unwrapped[count - 1] - unwrapped[0]);
  int32_t wrappedMedian = unwrapped[count / 2] % 4096;
  if (wrappedMedian < 0) {
    wrappedMedian += 4096;
  }
  median = static_cast<uint16_t>(wrappedMedian);
  return true;
}

bool reconcileCompletedScanIds(const uint8_t* existing,
                               std::size_t existingCount, uint8_t minimum,
                               uint8_t maximum, const uint8_t* discovered,
                               std::size_t discoveredCount, uint8_t* output,
                               std::size_t outputCapacity,
                               std::size_t& outputCount) {
  outputCount = 0;
  if (minimum > maximum || output == nullptr ||
      (existingCount > 0 && existing == nullptr) ||
      (discoveredCount > 0 && discovered == nullptr)) {
    return false;
  }

  const auto appendUnique = [&](uint8_t id) -> bool {
    if (!validServoId(id)) {
      return false;
    }
    for (std::size_t index = 0; index < outputCount; ++index) {
      if (output[index] == id) {
        return true;
      }
    }
    if (outputCount >= outputCapacity) {
      return false;
    }
    output[outputCount++] = id;
    return true;
  };

  // A completed partial scan is authoritative only inside its requested range.
  for (std::size_t index = 0; index < existingCount; ++index) {
    const uint8_t id = existing[index];
    if ((id < minimum || id > maximum) && !appendUnique(id)) {
      return false;
    }
  }
  for (std::size_t index = 0; index < discoveredCount; ++index) {
    const uint8_t id = discovered[index];
    if ((id < minimum || id > maximum) || !appendUnique(id)) {
      return false;
    }
  }
  return true;
}

void seedMultiTurnTruth(MultiTurnTruthState& state, int32_t position) {
  const uint32_t resyncCount = state.resyncCount;
  state = MultiTurnTruthState{};
  state.acceptedPosition = position;
  state.referencePosition = position;
  state.phase = MultiTurnTruthPhase::COMPLETE;
  state.resyncCount = resyncCount;
}

void forgetMultiTurnTruth(MultiTurnTruthState& state) {
  const uint32_t resyncCount = state.resyncCount;
  state = MultiTurnTruthState{};
  state.resyncCount = resyncCount;
}

bool beginAckedMultiTurnStep(MultiTurnTruthState& state, int32_t step) {
  if (!multiTurnTruthComplete(state) || step == 0) {
    return false;
  }
  const int64_t accepted =
      static_cast<int64_t>(state.referencePosition) + step;
  if (accepted < INT32_MIN || accepted > INT32_MAX) {
    return false;
  }
  state.acceptedPosition = static_cast<int32_t>(accepted);
  state.remaining = 0;
  state.phase = MultiTurnTruthPhase::AWAITING_COUNTDOWN;
  return true;
}

MultiTurnCountdownResult observeMultiTurnCountdown(
    MultiTurnTruthState& state, int32_t remaining,
    uint16_t arrivedTolerance) {
  // The production caller retains the existing, hardware-proven range check.
  // We deliberately do not add sign or monotonic assumptions here: neither was
  // part of the prior runtime contract. This helper decides only when the
  // observed sequence is sufficient to promote an ACKed endpoint to completed
  // truth.
  state.remaining = remaining;

  if (state.phase == MultiTurnTruthPhase::AWAITING_COUNTDOWN) {
    if (remaining == 0) {
      const int64_t magnitude =
          static_cast<int64_t>(state.acceptedPosition) -
          state.referencePosition;
      const int64_t absoluteMagnitude = magnitude < 0 ? -magnitude : magnitude;
      if (absoluteMagnitude > arrivedTolerance) {
        invalidateMultiTurnTruth(state);
        return MultiTurnCountdownResult::RESYNC_REQUIRED;
      }
      state.referencePosition = state.acceptedPosition;
      state.phase = MultiTurnTruthPhase::COMPLETE;
      return MultiTurnCountdownResult::COMPLETE;
    }

    const int64_t observed =
        static_cast<int64_t>(state.acceptedPosition) - remaining;
    if (observed < INT32_MIN || observed > INT32_MAX) {
      invalidateMultiTurnTruth(state);
      return MultiTurnCountdownResult::RESYNC_REQUIRED;
    }
    state.referencePosition = static_cast<int32_t>(observed);
    state.phase = MultiTurnTruthPhase::COUNTDOWN_LIVE;
    return MultiTurnCountdownResult::PENDING;
  }

  if (state.phase == MultiTurnTruthPhase::COUNTDOWN_LIVE) {
    if (remaining == 0) {
      state.referencePosition = state.acceptedPosition;
      state.phase = MultiTurnTruthPhase::COMPLETE;
      return MultiTurnCountdownResult::COMPLETE;
    }
    const int64_t observed =
        static_cast<int64_t>(state.acceptedPosition) - remaining;
    if (observed < INT32_MIN || observed > INT32_MAX) {
      invalidateMultiTurnTruth(state);
      return MultiTurnCountdownResult::RESYNC_REQUIRED;
    }
    state.referencePosition = static_cast<int32_t>(observed);
    return MultiTurnCountdownResult::PENDING;
  }

  if (state.phase == MultiTurnTruthPhase::COMPLETE && remaining == 0) {
    return MultiTurnCountdownResult::COMPLETE;
  }

  // A countdown with no step owned by this boot, or any observation after an
  // already-ambiguous command, cannot make the frame true again.
  invalidateMultiTurnTruth(state);
  return MultiTurnCountdownResult::RESYNC_REQUIRED;
}

void invalidateMultiTurnTruth(MultiTurnTruthState& state) {
  state.phase = MultiTurnTruthPhase::RESYNC_REQUIRED;
}

bool resyncMultiTurnTruthFromWrappedEncoder(MultiTurnTruthState& state,
                                            int32_t reference,
                                            uint16_t wrapped,
                                            int32_t& actual) {
  if (wrapped >= MULTI_TURN_ENCODER_TICKS) {
    return false;
  }
  actual = reconcileWrappedPosition(reference, wrapped);
  seedMultiTurnTruth(state, actual);
  ++state.resyncCount;
  return true;
}

bool multiTurnTruthComplete(const MultiTurnTruthState& state) {
  return state.phase == MultiTurnTruthPhase::COMPLETE;
}

bool multiTurnStepOutstanding(const MultiTurnTruthState& state) {
  return state.phase == MultiTurnTruthPhase::AWAITING_COUNTDOWN ||
         state.phase == MultiTurnTruthPhase::COUNTDOWN_LIVE;
}

bool multiTurnCountdownObserved(const MultiTurnTruthState& state) {
  return state.phase == MultiTurnTruthPhase::COUNTDOWN_LIVE;
}

bool multiTurnResyncRequired(const MultiTurnTruthState& state) {
  return state.phase == MultiTurnTruthPhase::RESYNC_REQUIRED;
}

bool multiTurnDeltaToTarget(const MultiTurnTruthState& state, int32_t target,
                            int32_t& delta) {
  if (!multiTurnTruthComplete(state)) {
    return false;
  }
  const int64_t difference =
      static_cast<int64_t>(target) - state.referencePosition;
  if (difference < INT32_MIN || difference > INT32_MAX) {
    return false;
  }
  delta = static_cast<int32_t>(difference);
  return true;
}

void SafetyStateMachine::begin(uint32_t) {
  bootTorqueOffRequired_ = true;
  heartbeatSeen_ = false;
  lastHeartbeatMs_ = 0;
  leaseActive_ = false;
  leaseServoId_ = 0;
  leaseStartedMs_ = 0;
  leaseDurationMs_ = 0;
  holdSetActive_ = false;
  std::memset(holdSetIds_, 0, sizeof(holdSetIds_));
  holdSetCount_ = 0;
  holdSetStartedMs_ = 0;
  holdSetDurationMs_ = 0;
  std::memset(torqueOffIds_, 0, sizeof(torqueOffIds_));
  torqueOffCount_ = 0;
  torqueOffRetryCursor_ = 0;
  torqueOffAttemptSeen_ = false;
  lastTorqueOffAttemptMs_ = 0;
  stopped_ = false;
}

bool SafetyStateMachine::bootTorqueOffRequired() const {
  return bootTorqueOffRequired_;
}

void SafetyStateMachine::markBootTorqueOffAttempted() {
  bootTorqueOffRequired_ = false;
}

void SafetyStateMachine::heartbeat(uint32_t nowMs) {
  heartbeatSeen_ = true;
  lastHeartbeatMs_ = nowMs;
}

bool SafetyStateMachine::heartbeatSeen() const { return heartbeatSeen_; }

bool SafetyStateMachine::heartbeatFresh(uint32_t nowMs) const {
  return heartbeatSeen_ && elapsed(nowMs, lastHeartbeatMs_) < WATCHDOG_TIMEOUT_MS;
}

uint32_t SafetyStateMachine::hostAgeMs(uint32_t nowMs) const {
  return heartbeatSeen_ ? elapsed(nowMs, lastHeartbeatMs_) : UINT32_MAX;
}

bool SafetyStateMachine::armTorqueOffObligation(uint8_t servoId) {
  return queueTorqueOffObligation(servoId);
}

bool SafetyStateMachine::queueTorqueOffObligation(uint8_t servoId) {
  if (!validServoId(servoId)) {
    return false;
  }
  for (uint16_t index = 0; index < torqueOffCount_; ++index) {
    if (torqueOffIds_[index] == servoId) {
      return true;
    }
  }
  if (torqueOffCount_ >= MAX_TORQUE_OFF_OBLIGATIONS) {
    return false;
  }
  torqueOffIds_[torqueOffCount_++] = servoId;
  if (torqueOffCount_ == 1) {
    torqueOffRetryCursor_ = 0;
    torqueOffAttemptSeen_ = false;
    lastTorqueOffAttemptMs_ = 0;
  }
  return true;
}

bool SafetyStateMachine::startTorqueLease(uint8_t servoId, uint32_t leaseMs,
                                          uint32_t nowMs) {
  if (stopped_ || !heartbeatFresh(nowMs) || !validServoId(servoId) ||
      !validTorqueLeaseMs(leaseMs) || leaseActive_ ||
      !torqueOffPendingFor(servoId) || holdAuthorizedFor(servoId, nowMs)) {
    return false;
  }
  leaseActive_ = true;
  leaseServoId_ = servoId;
  leaseStartedMs_ = nowMs;
  leaseDurationMs_ = leaseMs;
  return true;
}

bool SafetyStateMachine::torqueLeaseActive(uint32_t nowMs) const {
  return leaseActive_ && !stopped_ &&
         elapsed(nowMs, leaseStartedMs_) < leaseDurationMs_;
}

bool SafetyStateMachine::setHoldSet(const uint8_t* servoIds,
                                    uint8_t servoCount, uint32_t leaseMs,
                                    uint32_t nowMs) {
  if (stopped_ || !heartbeatFresh(nowMs) || servoCount > MAX_HOLD_SERVOS ||
      (servoCount > 0 && servoIds == nullptr) || leaseActive_ ||
      (servoCount > 0 && !validTorqueLeaseMs(leaseMs))) {
    return false;
  }
  for (uint8_t index = 0; index < servoCount; ++index) {
    if (!validServoId(servoIds[index]) ||
        !torqueOffPendingFor(servoIds[index])) {
      return false;
    }
    for (uint8_t prior = 0; prior < index; ++prior) {
      if (servoIds[prior] == servoIds[index]) {
        return false;
      }
    }
  }
  std::memset(holdSetIds_, 0, sizeof(holdSetIds_));
  if (servoCount > 0) {
    std::memcpy(holdSetIds_, servoIds, servoCount);
  }
  holdSetCount_ = servoCount;
  holdSetActive_ = servoCount > 0;
  holdSetStartedMs_ = nowMs;
  holdSetDurationMs_ = servoCount > 0 ? leaseMs : 0;
  return true;
}

bool SafetyStateMachine::holdSetActive(uint32_t nowMs) const {
  return holdSetActive_ && !stopped_ &&
         elapsed(nowMs, holdSetStartedMs_) < holdSetDurationMs_;
}

bool SafetyStateMachine::holdAuthorizedFor(uint8_t servoId,
                                           uint32_t nowMs) const {
  if (!holdSetActive(nowMs) || !torqueOffPendingFor(servoId)) {
    return false;
  }
  for (uint8_t index = 0; index < holdSetCount_; ++index) {
    if (holdSetIds_[index] == servoId) {
      return true;
    }
  }
  return false;
}

uint8_t SafetyStateMachine::holdSetCount() const { return holdSetCount_; }

uint8_t SafetyStateMachine::holdSetServoId(uint8_t index) const {
  return index < holdSetCount_ ? holdSetIds_[index] : 0;
}

uint32_t SafetyStateMachine::holdSetRemainingMs(uint32_t nowMs) const {
  if (!holdSetActive(nowMs)) {
    return 0;
  }
  return holdSetDurationMs_ - elapsed(nowMs, holdSetStartedMs_);
}

bool SafetyStateMachine::removeHoldServo(uint8_t servoId) {
  for (uint8_t index = 0; index < holdSetCount_; ++index) {
    if (holdSetIds_[index] != servoId) {
      continue;
    }
    for (uint8_t move = index; move + 1 < holdSetCount_; ++move) {
      holdSetIds_[move] = holdSetIds_[move + 1];
    }
    --holdSetCount_;
    holdSetIds_[holdSetCount_] = 0;
    if (holdSetCount_ == 0) {
      holdSetActive_ = false;
      holdSetDurationMs_ = 0;
    }
    return true;
  }
  return false;
}

bool SafetyStateMachine::torqueAuthorizedFor(uint8_t servoId,
                                             uint32_t nowMs) const {
  return (torqueLeaseActive(nowMs) && leaseServoId_ == servoId &&
          torqueOffPendingFor(servoId)) ||
         holdAuthorizedFor(servoId, nowMs);
}

uint8_t SafetyStateMachine::torqueLeaseId() const { return leaseServoId_; }

uint32_t SafetyStateMachine::torqueLeaseRemainingMs(uint32_t nowMs) const {
  if (!torqueLeaseActive(nowMs)) {
    return 0;
  }
  return leaseDurationMs_ - elapsed(nowMs, leaseStartedMs_);
}

bool SafetyStateMachine::torqueOffPending() const {
  return torqueOffCount_ > 0;
}

uint16_t SafetyStateMachine::torqueOffPendingCount() const {
  return torqueOffCount_;
}

bool SafetyStateMachine::torqueOffPendingFor(uint8_t servoId) const {
  for (uint16_t index = 0; index < torqueOffCount_; ++index) {
    if (torqueOffIds_[index] == servoId) {
      return true;
    }
  }
  return false;
}

uint8_t SafetyStateMachine::torqueOffServoId() const {
  return torqueOffCount_ == 0
             ? 0
             : torqueOffIds_[torqueOffRetryCursor_ % torqueOffCount_];
}

bool SafetyStateMachine::nextUnauthorizedTorqueOff(uint32_t nowMs,
                                                   uint8_t& servoId) const {
  for (uint16_t index = 0; index < torqueOffCount_; ++index) {
    if (!torqueAuthorizedFor(torqueOffIds_[index], nowMs)) {
      servoId = torqueOffIds_[index];
      return true;
    }
  }
  return false;
}

bool SafetyStateMachine::torqueOffRetryDue(uint32_t nowMs) const {
  return torqueOffPending() &&
         (!torqueOffAttemptSeen_ ||
          elapsed(nowMs, lastTorqueOffAttemptMs_) >= TORQUE_OFF_RETRY_MS);
}

void SafetyStateMachine::noteTorqueOffAttempt(uint8_t servoId,
                                              uint32_t nowMs) {
  if (!torqueOffPending()) {
    return;
  }
  torqueOffAttemptSeen_ = true;
  lastTorqueOffAttemptMs_ = nowMs;
  for (uint16_t index = 0; index < torqueOffCount_; ++index) {
    if (torqueOffIds_[index] == servoId) {
      torqueOffRetryCursor_ = (index + 1) % torqueOffCount_;
      return;
    }
  }
}

bool SafetyStateMachine::markTorqueOffConfirmed(uint8_t servoId) {
  uint16_t found = torqueOffCount_;
  for (uint16_t index = 0; index < torqueOffCount_; ++index) {
    if (torqueOffIds_[index] == servoId) {
      found = index;
      break;
    }
  }
  if (found == torqueOffCount_) {
    return false;
  }
  for (uint16_t index = found; index + 1 < torqueOffCount_; ++index) {
    torqueOffIds_[index] = torqueOffIds_[index + 1];
  }
  --torqueOffCount_;
  if (torqueOffCount_ == 0) {
    torqueOffRetryCursor_ = 0;
    torqueOffAttemptSeen_ = false;
  } else {
    torqueOffRetryCursor_ %= torqueOffCount_;
  }
  if (leaseServoId_ == servoId) {
    leaseActive_ = false;
    leaseDurationMs_ = 0;
  }
  (void)removeHoldServo(servoId);
  return true;
}

SafetyEvent SafetyStateMachine::tick(uint32_t nowMs) {
  if (!torqueOffPending()) {
    return SafetyEvent::NONE;
  }
  if (!heartbeatFresh(nowMs)) {
    // Authority lapses -- a silent host must never leave the arm energised --
    // but this no longer LATCHES. Measured on hardware: it fires during
    // torque-off settling because the host's 200 ms heartbeat shares one serial
    // lock with operations allowed to hold it for up to 1500 ms, and every
    // capture taken as it fired showed the bus online, no jam, no servo fault,
    // 12 V, packet ages under 5 ms. Nothing was wrong except that a heartbeat
    // queued behind a long read.
    //
    // Dropping authority is the part that protects the arm, and it is kept.
    // The latch only decided whether the operator had to go and clear it
    // afterwards -- and because the host renews its hold continuously,
    // `resetInspected` could never clear it, so it was a lockout rather than a
    // safety measure. Deliberate STOPs still latch; see `latchStop()`.
    revokeAuthority();
    return SafetyEvent::WATCHDOG_STOP;
  }
  bool leaseExpired = false;
  if (leaseActive_ && !torqueLeaseActive(nowMs)) {
    leaseActive_ = false;
    leaseExpired = true;
  }
  if (holdSetActive_ && !holdSetActive(nowMs)) {
    holdSetActive_ = false;
    leaseExpired = true;
  }
  if (leaseExpired) {
    return SafetyEvent::LEASE_EXPIRED;
  }
  if (stopped_) {
    return SafetyEvent::TORQUE_OFF_RETRY;
  }
  for (uint16_t index = 0; index < torqueOffCount_; ++index) {
    if (!torqueAuthorizedFor(torqueOffIds_[index], nowMs)) {
      return SafetyEvent::TORQUE_OFF_RETRY;
    }
  }
  return SafetyEvent::NONE;
}

void SafetyStateMachine::revokeAuthority() {
  leaseActive_ = false;
  holdSetActive_ = false;
}

void SafetyStateMachine::latchStop() {
  stopped_ = true;
  revokeAuthority();
}

bool SafetyStateMachine::stopped() const { return stopped_; }

bool SafetyStateMachine::resetInspected(uint32_t nowMs) {
  if (!stopped_ || !heartbeatFresh(nowMs)) {
    return false;
  }
  // Authority is DROPPED here rather than demanded to be already absent. It
  // used to refuse while a lease or hold set was live, which made the latch
  // unclearable in normal use: the host renews its hold every 800 ms, so there
  // was no moment at which the precondition held, and RESET_PRECONDITION came
  // back forever. Clearing the latch and revoking authority in the same breath
  // is strictly safer than a STOP nobody can lift -- the host has to ask for
  // its hold again afterwards, which it does on the next tick.
  //
  // A live heartbeat is still required: that is what proves someone is there
  // to have inspected the arm.
  revokeAuthority();
  stopped_ = false;
  return true;
}

}  // namespace armhat
