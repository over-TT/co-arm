#include "ArmHatController.h"

#include <cstdarg>
#include <cstdio>
#include <cstring>
#include <esp_system.h>

namespace armhat {
namespace {

constexpr uint8_t REGISTER_ID = 5;
constexpr uint8_t REGISTER_MINIMUM_POSITION = 9;
constexpr uint8_t REGISTER_MAXIMUM_POSITION = 11;
constexpr uint8_t REGISTER_PHASE = 0x12;
constexpr uint8_t REGISTER_RESOLUTION = 0x1E;
constexpr uint8_t REGISTER_OPERATING_MODE = 33;
constexpr uint8_t REGISTER_TORQUE_ENABLE = 40;
constexpr uint8_t REGISTER_ACCELERATION = 41;
constexpr uint8_t REGISTER_PRESENT_POSITION = 56;
constexpr uint8_t REGISTER_LOCK = 55;
constexpr uint8_t ST3215_OPERATING_MODE_MAX = 3;
constexpr uint8_t TELEMETRY_BYTES = 15;
constexpr uint8_t ST3215_PHASE_EXTENDED_POSITION = 0x10;
constexpr uint8_t ST3215_DEFAULT_RESOLUTION = 1;
// SCAN used to allow only 800 us while ordinary register reads allow 3000 us.
// A healthy but slightly slower Base could therefore work under STATUS and be
// deleted from the inventory by Scan. Match the normal read window and retry
// one silence; corrupt frames are never retried because they are collision
// evidence, not latency.
constexpr uint32_t SCAN_PING_TIMEOUT_US = 3000;
constexpr uint8_t SCAN_PING_ATTEMPTS = 2;
// Motion tolerance and timing now live in MotionPolicy, settable at runtime with
// CONFIG. A real servo under a gear train does not settle to a couple of ticks
// inside the execution window, and that number is a property of this arm, not of
// the controller — so it must be tunable without a reflash.

uint16_t littleEndian16(const uint8_t* bytes) {
  return static_cast<uint16_t>(bytes[0]) |
         (static_cast<uint16_t>(bytes[1]) << 8u);
}

uint16_t decode16(const uint8_t* bytes, bool bigEndian) {
  return bigEndian ? static_cast<uint16_t>((static_cast<uint16_t>(bytes[0]) << 8u) |
                                           static_cast<uint16_t>(bytes[1]))
                   : littleEndian16(bytes);
}

void encode16(uint8_t* bytes, uint16_t value, bool bigEndian) {
  bytes[0] = static_cast<uint8_t>(bigEndian ? (value >> 8u) : (value & 0xFFu));
  bytes[1] = static_cast<uint8_t>(bigEndian ? (value & 0xFFu) : (value >> 8u));
}

bool parseServoId(const char* text, uint8_t& id) {
  uint32_t value = 0;
  if (!parseUInt32(text, value) || !validServoId(value)) {
    return false;
  }
  id = static_cast<uint8_t>(value);
  return true;
}

uint32_t elapsedMs(uint32_t nowMs, uint32_t thenMs) {
  return nowMs - thenMs;
}

bool decodeExtendedPosition(uint16_t encoded, int32_t& position) {
  position = decodeSignedMagnitude16(encoded);
  return position >= -ST3215_MULTI_TURN_MAX &&
         position <= ST3215_MULTI_TURN_MAX;
}

bool nativeExtendedPositionContinuous(const ServoTelemetry& servo,
                                      int32_t position,
                                      uint32_t sampledAtMs) {
  if (!servo.odometerValid) {
    return true;
  }
  const int32_t previous =
      servo.revolutions * MULTI_TURN_ENCODER_TICKS +
      static_cast<int32_t>(servo.odometerLastRaw);
  const int32_t delta = position - previous;
  const int32_t magnitude = delta < 0 ? -delta : delta;
  const bool sudden =
      elapsedMs(sampledAtMs, servo.odometerSampledAtMs) <
      NATIVE_MULTI_TURN_REVOLUTION_MIN_MS;
  // A normal signed seam is +/-1 (4095 -> 4096 or -4095 -> -4096), not
  // +/-4096. Exact whole-turn loss is how a reset native frame collapses back
  // to the same wrapping encoder coordinate.
  const bool wholeTurnCollapse =
      magnitude != 0 && (magnitude % MULTI_TURN_ENCODER_TICKS) == 0;
  return !wholeTurnCollapse || (!sudden && !servo.nativeSampleMissed);
}

bool storeExtendedPosition(ServoTelemetry& servo, int32_t position,
                           uint32_t sampledAtMs,
                           bool enforceContinuity = true) {
  if (enforceContinuity &&
      !nativeExtendedPositionContinuous(servo, position, sampledAtMs)) {
    servo.odometerValid = false;
    forgetMultiTurnTruth(servo.multiTurnTruth);
    return false;
  }
  int32_t revolutions = position / MULTI_TURN_ENCODER_TICKS;
  int32_t raw = position - revolutions * MULTI_TURN_ENCODER_TICKS;
  if (raw < 0) {
    raw += MULTI_TURN_ENCODER_TICKS;
    --revolutions;
  }
  servo.revolutions = revolutions;
  servo.odometerLastRaw = static_cast<uint16_t>(raw);
  servo.odometerSampledAtMs = sampledAtMs;
  servo.nativeSampleMissed = false;
  if (servo.odometerValid) {
    seedMultiTurnTruth(servo.multiTurnTruth, position);
  }
  return true;
}

}  // namespace

const ServoDialect& dialectFor(ServoFamily family) {
  //                     bigEnd  posMax  lock  goal  gBytes  telem  accel  mode  current
  static const ServoDialect STS{false, 4095, 55, REGISTER_ACCELERATION, 7, 15,
                                true, true, true};
  // SCS drops the acceleration byte, so its goal block starts one register
  // later at 42 and is six bytes: position, time, speed -- each big-endian.
  static const ServoDialect SCS{true, 1023, 48, 42, 6, 11, false, false, false};
  return family == ServoFamily::SCS ? SCS : STS;
}

St3215Bus::St3215Bus(HardwareSerial& serial) : serial_(serial) {}

void St3215Bus::begin(uint32_t baud, int8_t rxPin, int8_t txPin) {
  serial_.begin(baud, SERIAL_8N1, rxPin, txPin);
  // A UART that has just come up reports the settling of its own pin as data.
  // Latching a jam from that used to poison the whole boot, so the founding
  // drain is explicitly not evidence of anything.
  discardInput();
  clearJam();
}

void St3215Bus::clearJam() {
  jammed_ = false;
  noise_ = BusNoise{};
}

void St3215Bus::setFamily(uint8_t id, ServoFamily family) {
  for (uint8_t index = 0; index < scsIdCount_; ++index) {
    if (scsIds_[index] != id) {
      continue;
    }
    if (family == ServoFamily::STS) {
      scsIds_[index] = scsIds_[--scsIdCount_];
    }
    return;
  }
  if (family == ServoFamily::SCS && scsIdCount_ < MAX_TRACKED_SERVOS) {
    scsIds_[scsIdCount_++] = id;
  }
}

void St3215Bus::setMultiTurn(uint8_t id, bool enabled) {
  for (uint8_t index = 0; index < multiTurnIdCount_; ++index) {
    if (multiTurnIds_[index] != id) {
      continue;
    }
    if (!enabled) {
      multiTurnIds_[index] = multiTurnIds_[--multiTurnIdCount_];
    }
    return;
  }
  if (enabled && multiTurnIdCount_ < MAX_TRACKED_SERVOS) {
    multiTurnIds_[multiTurnIdCount_++] = id;
  }
}

bool St3215Bus::isMultiTurn(uint8_t id) const {
  for (uint8_t index = 0; index < multiTurnIdCount_; ++index) {
    if (multiTurnIds_[index] == id) {
      return true;
    }
  }
  return false;
}

ServoFamily St3215Bus::familyOf(uint8_t id) const {
  for (uint8_t index = 0; index < scsIdCount_; ++index) {
    if (scsIds_[index] == id) {
      return ServoFamily::SCS;
    }
  }
  return ServoFamily::STS;
}

bool St3215Bus::discardInput() {
  // Bounded on both bytes and time. Unbounded, this loop was the hang that made
  // an id collision look like a dead controller: it runs inside every send, and
  // two servos answering the same id keep the line busy forever.
  constexpr std::size_t DISCARD_MAX_BYTES = 256;
  constexpr uint32_t DISCARD_TIMEOUT_US = 4000;
  const uint32_t startedUs = micros();
  BusNoise seen;
  bool sawNonZero = false;
  bool sawNonOnes = false;
  while (serial_.available() > 0) {
    const uint8_t value = static_cast<uint8_t>(serial_.read());
    if (seen.sampleCount < sizeof(seen.sample)) {
      seen.sample[seen.sampleCount++] = value;
    }
    sawNonZero = sawNonZero || value != 0x00;
    sawNonOnes = sawNonOnes || value != 0xFF;
    ++seen.bytes;
    if (seen.bytes >= DISCARD_MAX_BYTES ||
        static_cast<uint32_t>(micros() - startedUs) >= DISCARD_TIMEOUT_US) {
      // Keep the census. Which of these it is decides whether the fix is a
      // cable or a code change, and that was previously unknowable from here.
      seen.allZero = !sawNonZero;
      seen.allOnes = !sawNonOnes;
      noise_ = seen;
      jammed_ = true;
      return false;
    }
  }
  return true;
}

bool St3215Bus::readByteUntil(uint8_t& value, uint32_t startedUs,
                              uint32_t timeoutUs) {
  while (static_cast<uint32_t>(micros() - startedUs) < timeoutUs) {
    if (serial_.available() > 0) {
      value = static_cast<uint8_t>(serial_.read());
      return true;
    }
    delayMicroseconds(20);
  }
  return false;
}

bool St3215Bus::sendInstruction(uint8_t id, uint8_t instruction,
                                const uint8_t* parameters,
                                uint8_t parameterCount) {
  // Four STS goals need 2 header bytes plus 4 * (id + 7-byte goal) = 34
  // parameters, and the protocol adds six framing/checksum bytes. Keep headroom
  // explicit so the advertised four-member MOVE_SET cannot fail on packet size.
  uint8_t packet[48] = {};
  const std::size_t packetLength =
      buildScsInstruction(id, instruction, parameters, parameterCount, packet,
                          sizeof(packet));
  if (packetLength == 0) {
    return false;
  }
  if (!discardInput()) {
    // Talking over a bus that is already talking only produces more garbage.
    return false;
  }
  const std::size_t written = serial_.write(packet, packetLength);
  serial_.flush();
  return written == packetLength;
}

BusResult St3215Bus::receiveStatus(uint8_t expectedId, uint8_t* parameters,
                                   uint8_t parameterCapacity,
                                   uint8_t& parameterCount,
                                   uint8_t& servoError,
                                   uint32_t timeoutUs) {
  parameterCount = 0;
  servoError = 0;
  const uint32_t startedUs = micros();
  bool sawHeaderByte = false;
  uint8_t value = 0;
  while (true) {
    if (!readByteUntil(value, startedUs, timeoutUs)) {
      return BusResult::TIMEOUT;
    }
    if (value == 0xFF) {
      if (sawHeaderByte) {
        break;
      }
      sawHeaderByte = true;
    } else {
      sawHeaderByte = false;
    }
  }

  uint8_t responseId = 0;
  uint8_t responseLength = 0;
  if (!readByteUntil(responseId, startedUs, timeoutUs) ||
      !readByteUntil(responseLength, startedUs, timeoutUs)) {
    return BusResult::TIMEOUT;
  }
  if (responseId != expectedId || responseLength < 2 ||
      responseLength > 31) {
    return BusResult::CORRUPT;
  }

  uint8_t tail[31] = {};
  for (uint8_t index = 0; index < responseLength; ++index) {
    if (!readByteUntil(tail[index], startedUs, timeoutUs)) {
      return BusResult::TIMEOUT;
    }
  }

  uint8_t sum = static_cast<uint8_t>(responseId + responseLength);
  for (uint8_t index = 0; index + 1 < responseLength; ++index) {
    sum = static_cast<uint8_t>(sum + tail[index]);
  }
  if (tail[responseLength - 1] != static_cast<uint8_t>(~sum)) {
    return BusResult::CORRUPT;
  }

  servoError = tail[0];
  parameterCount = static_cast<uint8_t>(responseLength - 2);
  if (parameterCount > parameterCapacity ||
      (parameterCount > 0 && parameters == nullptr)) {
    parameterCount = 0;
    return BusResult::CORRUPT;
  }
  if (parameterCount > 0) {
    std::memcpy(parameters, tail + 1, parameterCount);
  }
  return servoError == 0 ? BusResult::OK : BusResult::SERVO_ERROR;
}

bool St3215Bus::broadcastTorqueOff() {
  return setTorque(ST3215_BROADCAST_ID, false);
}

bool St3215Bus::ping(uint8_t id, uint32_t timeoutUs, BusResult* outcome) {
  if (outcome != nullptr) {
    *outcome = BusResult::INVALID;
  }
  if (!validServoId(id) ||
      !sendInstruction(id, SCS_INSTRUCTION_PING, nullptr, 0)) {
    return false;
  }
  uint8_t parameterCount = 0;
  uint8_t servoError = 0;
  const BusResult result = receiveStatus(id, nullptr, 0, parameterCount,
                                         servoError, timeoutUs);
  if (outcome != nullptr) {
    *outcome = result;
  }
  return result == BusResult::OK || result == BusResult::SERVO_ERROR;
}

BusResult St3215Bus::readRegisters(uint8_t id, uint8_t address,
                                   uint8_t length, uint8_t* output,
                                   uint8_t& servoError,
                                   uint32_t timeoutUs) {
  if (!validServoId(id) || length == 0 || output == nullptr) {
    return BusResult::INVALID;
  }
  const uint8_t request[] = {address, length};
  if (!sendInstruction(id, SCS_INSTRUCTION_READ, request, sizeof(request))) {
    return BusResult::INVALID;
  }
  uint8_t received = 0;
  const BusResult result = receiveStatus(id, output, length, received,
                                         servoError, timeoutUs);
  if ((result == BusResult::OK || result == BusResult::SERVO_ERROR) &&
      received != length) {
    return BusResult::CORRUPT;
  }
  return result;
}

bool St3215Bus::writeRegister(uint8_t id, uint8_t address,
                              const uint8_t* values, uint8_t valueCount) {
  if ((id != ST3215_BROADCAST_ID && !validServoId(id)) || valueCount == 0 ||
      values == nullptr || valueCount > 24) {
    return false;
  }
  uint8_t parameters[25] = {};
  parameters[0] = address;
  std::memcpy(parameters + 1, values, valueCount);
  if (!sendInstruction(id, SCS_INSTRUCTION_WRITE, parameters,
                       static_cast<uint8_t>(valueCount + 1))) {
    return false;
  }
  if (id == ST3215_BROADCAST_ID) {
    // Broadcasts deliberately have no status packet. Give the devices time to
    // consume the write before the next bus operation.
    delayMicroseconds(500);
    return true;
  }

  // ST3215 response-level register 8 defaults to 1: every addressed
  // instruction returns a status packet. The vendor SDK's genWrite() waits for
  // and validates this ACK. Merely getting all bytes into the ESP32 UART used
  // to advance the multi-turn command ledger even when the servo never
  // accepted the step, producing reported motion with a stationary shaft.
  uint8_t parameterCount = 0;
  uint8_t servoError = 0;
  const BusResult result = receiveStatus(id, nullptr, 0, parameterCount,
                                         servoError, 3000);
  return result == BusResult::OK;
}

bool St3215Bus::setTorque(uint8_t id, bool enabled) {
  const uint8_t value = enabled ? 1 : 0;
  return writeRegister(id, REGISTER_TORQUE_ENABLE, &value, 1);
}

BusResult St3215Bus::readTorque(uint8_t id, bool& enabled,
                                uint8_t& servoError) {
  uint8_t value = 0;
  const BusResult result =
      readRegisters(id, REGISTER_TORQUE_ENABLE, 1, &value, servoError);
  if (result == BusResult::OK || result == BusResult::SERVO_ERROR) {
    enabled = value != 0;
  }
  return result;
}

BusResult St3215Bus::readTelemetry(uint8_t id,
                                   ServoTelemetry& telemetry) {
  // Both families lay 56.. out identically for the first eleven bytes, so only
  // the byte order and the tail differ: reading 15 from an SCS servo runs off
  // the end of its table, which is what made a mixed bus read as garbage.
  const ServoDialect& map = dialect(id);
  uint8_t bytes[TELEMETRY_BYTES] = {};
  uint8_t servoError = 0;
  const BusResult result = readRegisters(id, REGISTER_PRESENT_POSITION,
                                         map.telemetryBytes, bytes, servoError);
  if (result != BusResult::OK && result != BusResult::SERVO_ERROR) {
    return result;
  }

  telemetry.id = id;
  telemetry.online = true;
  telemetry.fresh = true;
  telemetry.sampled = true;
  telemetry.family = familyOf(id);
  telemetry.speed = decodeSignedMagnitude16(decode16(bytes + 2, map.bigEndian));
  telemetry.load = decodeServoLoad(decode16(bytes + 4, map.bigEndian));
  telemetry.voltage = bytes[6];
  telemetry.temperature = bytes[7];
  telemetry.error = bytes[9];
  telemetry.statusError = servoError;
  telemetry.moving = bytes[10];
  telemetry.currentRaw =
      map.hasCurrent ? decode16(bytes + 13, map.bigEndian) : 0;
  telemetry.sampledAtMs = millis();

  bool torqueEnabled = false;
  uint8_t torqueError = 0;
  const BusResult torqueResult = readTorque(id, torqueEnabled, torqueError);
  telemetry.statusError =
      static_cast<uint8_t>(telemetry.statusError | torqueError);
  telemetry.torque =
      torqueResult == BusResult::OK || torqueResult == BusResult::SERVO_ERROR
          ? (torqueEnabled ? TorqueState::ON : TorqueState::OFF)
          : TorqueState::UNKNOWN;
  uint8_t mode = 0;
  uint8_t modeError = 0;
  const BusResult modeResult = readOperatingMode(id, mode, modeError);
  telemetry.statusError =
      static_cast<uint8_t>(telemetry.statusError | modeError);
  telemetry.operatingModeKnown =
      modeResult == BusResult::OK || modeResult == BusResult::SERVO_ERROR;
  telemetry.operatingMode = mode;

  const uint16_t encodedPosition = decode16(bytes + 0, map.bigEndian);
  if (telemetry.family == ServoFamily::STS && !isMultiTurn(id) &&
      telemetry.operatingModeKnown && telemetry.operatingMode == 0 &&
      encodedPosition > ST3215_POSITION_MAX) {
    int32_t inferredPosition = 0;
    if (decodeExtendedPosition(encodedPosition, inferredPosition)) {
      // Native Mode 0 survives a HAT reboot, but this interpretation table is
      // RAM-only. A checksum-valid STS position outside the single-turn range
      // cannot be ordinary 0..4095 feedback, so restore only the decoder
      // classification. Odometer tracking and truth remain invalid until the
      // operator explicitly establishes zero.
      setMultiTurn(id, true);
    }
  }
  if (isMultiTurn(id)) {
    if (!telemetry.operatingModeKnown) {
      // The position bytes arrived, but without the mode subread they cannot be
      // interpreted as a signed extended coordinate. Retain the last trusted
      // anchor as stale and require a complete Mode-0 sample before reuse.
      telemetry.fresh = false;
      telemetry.nativeSampleMissed = true;
      telemetry.rawPosition = telemetry.odometerLastRaw;
    } else if (telemetry.operatingMode != 0) {
      // A confirmed non-position mode is real continuity loss, not silence.
      telemetry.odometerValid = false;
      telemetry.nativeSampleMissed = false;
      forgetMultiTurnTruth(telemetry.multiTurnTruth);
      telemetry.rawPosition = telemetry.odometerLastRaw;
    } else {
      int32_t position = 0;
      if (!decodeExtendedPosition(encodedPosition, position)) {
        telemetry.odometerValid = false;
        telemetry.nativeSampleMissed = false;
        forgetMultiTurnTruth(telemetry.multiTurnTruth);
      } else {
        storeExtendedPosition(telemetry, position, telemetry.sampledAtMs);
      }
      telemetry.rawPosition = telemetry.odometerLastRaw;
    }
  } else {
    telemetry.rawPosition = encodedPosition;
  }
  return result;
}

bool St3215Bus::writePosition(uint8_t id, int32_t position, uint16_t speed,
                              uint8_t acceleration) {
  const ServoDialect& map = dialect(id);
  const bool multiTurn = isMultiTurn(id);
  const int32_t low = multiTurn ? -ST3215_MULTI_TURN_MAX : 0;
  const int32_t high = multiTurn ? ST3215_MULTI_TURN_MAX
                                 : static_cast<int32_t>(map.positionMax);
  if (!validServoId(id) || position < low || position > high || speed == 0) {
    return false;
  }
  // Sign-magnitude, not two's complement: bit 15 carries the sign and the rest
  // carries the distance. A single-turn goal is never negative, so this is the
  // identity for every joint that is not multi-turn.
  const uint16_t encoded =
      position < 0 ? static_cast<uint16_t>(static_cast<uint16_t>(-position) |
                                           ST3215_SIGN_BIT)
                   : static_cast<uint16_t>(position);
  // One contiguous block on both families: [accel] position, time, speed. SCS
  // has no acceleration register, so its block starts at the position word.
  uint8_t values[7] = {};
  uint8_t count = 0;
  if (map.hasAcceleration) {
    values[count++] = acceleration;
  }
  encode16(values + count, encoded, map.bigEndian);
  count = static_cast<uint8_t>(count + 2);
  encode16(values + count, 0, map.bigEndian);
  count = static_cast<uint8_t>(count + 2);
  encode16(values + count, speed, map.bigEndian);
  count = static_cast<uint8_t>(count + 2);
  return writeRegister(id, map.goalRegister, values, count);
}

bool St3215Bus::syncWritePositions(ServoFamily family,
                                   const PositionTarget* targets,
                                   uint8_t targetCount) {
  if (targets == nullptr || targetCount == 0 ||
      targetCount > MAX_HOLD_SERVOS) {
    return false;
  }
  const ServoDialect& map = dialectFor(family);
  uint8_t parameters[2 + MAX_HOLD_SERVOS * 8] = {};
  parameters[0] = map.goalRegister;
  parameters[1] = map.goalBytes;
  uint8_t cursor = 2;
  for (uint8_t index = 0; index < targetCount; ++index) {
    const PositionTarget& target = targets[index];
    if (!validServoId(target.id) || familyOf(target.id) != family ||
        target.speed == 0) {
      return false;
    }
    const bool multiTurn = isMultiTurn(target.id);
    const int32_t low = multiTurn ? -ST3215_MULTI_TURN_MAX : 0;
    const int32_t high = multiTurn ? ST3215_MULTI_TURN_MAX
                                   : static_cast<int32_t>(map.positionMax);
    if (target.position < low || target.position > high) {
      return false;
    }
    const uint16_t encoded =
        target.position < 0
            ? static_cast<uint16_t>(static_cast<uint16_t>(-target.position) |
                                    ST3215_SIGN_BIT)
            : static_cast<uint16_t>(target.position);
    parameters[cursor++] = target.id;
    if (map.hasAcceleration) {
      parameters[cursor++] = target.acceleration;
    }
    encode16(parameters + cursor, encoded, map.bigEndian);
    cursor = static_cast<uint8_t>(cursor + 2);
    encode16(parameters + cursor, 0, map.bigEndian);
    cursor = static_cast<uint8_t>(cursor + 2);
    encode16(parameters + cursor, target.speed, map.bigEndian);
    cursor = static_cast<uint8_t>(cursor + 2);
  }
  if (!sendInstruction(ST3215_BROADCAST_ID, SCS_INSTRUCTION_SYNC_WRITE,
                       parameters, cursor)) {
    return false;
  }
  // SYNC_WRITE deliberately has no addressed response. Strict per-ID goal
  // readback in handleMoveSet() supplies the execution receipt.
  delayMicroseconds(500);
  return true;
}

bool St3215Bus::verifyPositionCommand(uint8_t id, int32_t position,
                                      uint16_t speed,
                                      uint8_t acceleration) {
  const ServoDialect& map = dialect(id);
  const uint16_t encoded =
      position < 0 ? static_cast<uint16_t>(static_cast<uint16_t>(-position) |
                                           ST3215_SIGN_BIT)
                   : static_cast<uint16_t>(position);
  for (uint8_t attempt = 0; attempt < POSITION_VERIFY_ATTEMPTS; ++attempt) {
    uint8_t values[7] = {};
    uint8_t servoError = 0;
    const BusResult result = readRegisters(
        id, map.goalRegister, map.goalBytes, values, servoError);
    if (result == BusResult::OK) {
      const uint8_t offset = map.hasAcceleration ? 1 : 0;
      const bool exact =
          (!map.hasAcceleration || values[0] == acceleration) &&
          decode16(values + offset, map.bigEndian) == encoded &&
          decode16(values + offset + 2, map.bigEndian) == 0 &&
          decode16(values + offset + 4, map.bigEndian) == speed;
      if (exact) {
        return true;
      }
    } else if (result != BusResult::TIMEOUT && result != BusResult::CORRUPT) {
      // An explicit servo error or invalid request is not a settling race.
      return false;
    }
    if (attempt + 1 < POSITION_VERIFY_ATTEMPTS) {
      delayMicroseconds(POSITION_VERIFY_RETRY_DELAY_US);
    }
  }
  return false;
}

bool St3215Bus::writeId(uint8_t oldId, uint8_t newId) {
  return validServoId(oldId) && validServoId(newId) &&
         writeRegister(oldId, REGISTER_ID, &newId, 1);
}

bool St3215Bus::writePositionMode(uint8_t id) {
  return writeOperatingMode(id, 0);
}

bool St3215Bus::writeOperatingMode(uint8_t id, uint8_t mode) {
  if (!validServoId(id)) {
    return false;
  }
  // SCS has no operating-mode register because the family has no other mode.
  // Writing 33 there would land in an undefined gap in its table.
  if (!dialect(id).hasOperatingMode) {
    return mode == 0;
  }
  return writeRegister(id, REGISTER_OPERATING_MODE, &mode, 1);
}

BusResult St3215Bus::readPositionLimits(uint8_t id, uint16_t& minimum,
                                        uint16_t& maximum,
                                        uint8_t& servoError) {
  const bool bigEndian = dialect(id).bigEndian;
  uint8_t values[4] = {};
  const BusResult result =
      readRegisters(id, REGISTER_MINIMUM_POSITION, sizeof(values), values,
                    servoError);
  if (result == BusResult::OK || result == BusResult::SERVO_ERROR) {
    minimum = decode16(values, bigEndian);
    maximum = decode16(values + 2, bigEndian);
  }
  return result;
}

bool St3215Bus::writeLock(uint8_t id, bool locked) {
  const uint8_t value = locked ? 1 : 0;
  return writeRegister(id, dialect(id).lockRegister, &value, 1);
}

BusResult St3215Bus::readLock(uint8_t id, bool& locked,
                              uint8_t& servoError) {
  uint8_t value = 0;
  const BusResult result =
      readRegisters(id, dialect(id).lockRegister, 1, &value, servoError);
  if (result == BusResult::OK) {
    if (value > 1) {
      return BusResult::CORRUPT;
    }
    locked = value == 1;
  }
  return result;
}

BusResult St3215Bus::readOperatingMode(uint8_t id, uint8_t& mode,
                                       uint8_t& servoError) {
  if (!dialect(id).hasOperatingMode) {
    mode = 0;
    servoError = 0;
    return BusResult::OK;
  }
  return readRegisters(id, REGISTER_OPERATING_MODE, 1, &mode, servoError);
}

ArmHatRuntime::ArmHatRuntime(Stream& host, HardwareSerial& servoSerial)
    : host_(host), bus_(servoSerial) {}

void ArmHatRuntime::begin(uint32_t servoBaud, int8_t servoRxPin,
                          int8_t servoTxPin) {
  bus_.begin(servoBaud, servoRxPin, servoTxPin);
  const uint32_t nowMs = millis();
  safety_.begin(nowMs);
  const uint64_t controllerMac = ESP.getEfuseMac();
  do {
    bootId_ = (static_cast<uint64_t>(esp_random()) << 32u) | esp_random();
  } while (bootId_ == 0);
  std::snprintf(controllerId_, sizeof(controllerId_), "armhat-%04lx%08lx",
                static_cast<unsigned long>(controllerMac >> 32u),
                static_cast<unsigned long>(controllerMac & 0xFFFFFFFFu));
  // Deliberately NOT done here. begin() runs inside setup(), before the host
  // link is serviced, so a bus that will not go quiet used to strand the
  // controller with no way to say so. The first loop() iteration does it, by
  // which point the operator can already reach the controller and press STOP.
  bootTorqueOffPending_ = true;
}

void ArmHatRuntime::poll() {
  std::size_t processed = 0;
  while (host_.available() > 0 &&
         processed < MAX_REQUEST_BYTES * 2) {
    ++processed;
    const int incoming = host_.read();
    if (incoming < 0) {
      break;
    }
    const char value = static_cast<char>(incoming);
    if (value == '\n') {
      processLine();
      requestLength_ = 0;
      requestOverflow_ = false;
      continue;
    }
    if (requestOverflow_) {
      continue;
    }
    if (requestLength_ + 1 >= sizeof(request_)) {
      requestOverflow_ = true;
      continue;
    }
    request_[requestLength_++] = value;
  }
}

void ArmHatRuntime::processLine() {
  // Enforce an expired lease or host watchdog before accepting the next host
  // command. HEARTBEAT cannot retroactively keep an already-expired lease alive.
  const bool hostWasFresh = safety_.heartbeatFresh(millis());
  tick();
  if (requestOverflow_) {
    sendError(0, "LINE_TOO_LONG");
    return;
  }
  request_[requestLength_] = '\0';
  ParsedCommand command{};
  const ParseResult result =
      parseCommandLine(request_, requestLength_, command);
  if (result != ParseResult::OK) {
    sendError(command.sequence, parseErrorCode(result));
    return;
  }
  if (hostWasFresh) {
    // Valid command traffic is itself proof that the supervised host is alive.
    // The Pi's explicit heartbeat shares one serial lock with STATUS/MOVE; not
    // counting those requests let a healthy, busy host lose authority while it
    // was actively talking to this controller. A stale host is never revived
    // here because tick() already enforced the lapse above.
    safety_.heartbeat(millis());
  }
  dispatch(command);
  if (hostWasFresh) {
    // Some bounded bus operations take longer than one watchdog window. The
    // firmware cannot run tick() while it is synchronously servicing them
    // anyway, so finish the same live-host transaction with a fresh timestamp
    // instead of revoking authority immediately after its response.
    safety_.heartbeat(millis());
  }
}

void ArmHatRuntime::dispatch(const ParsedCommand& command) {
  const bool takesMotionAuthority =
      command.operation == Operation::HOLD_SET ||
      command.operation == Operation::TORQUE_LEASE ||
      command.operation == Operation::PREPARE_NUDGE ||
      command.operation == Operation::EXECUTE_NUDGE ||
      command.operation == Operation::MOVE ||
      command.operation == Operation::MOVE_SET ||
      command.operation == Operation::FOLLOW_SET;
  if (takesMotionAuthority && safetyFault_ && !safety_.stopped()) {
    sendError(command.sequence, "RECOVERY_PENDING");
    return;
  }
  switch (command.operation) {
    case Operation::HELLO:
      handleHello(command.sequence, command);
      break;
    case Operation::HEARTBEAT:
      handleHeartbeat(command.sequence, command);
      break;
    case Operation::STATUS:
      handleStatus(command.sequence, command);
      break;
    case Operation::SCAN:
      handleScan(command.sequence, command);
      break;
    case Operation::FAMILY:
      handleFamily(command.sequence, command);
      break;
    case Operation::MULTITURN:
      handleMultiTurn(command.sequence, command);
      break;
    case Operation::ASSIGN_ID:
      handleAssignId(command.sequence, command);
      break;
    case Operation::SET_POSITION_MODE:
      handleSetPositionMode(command.sequence, command);
      break;
    case Operation::CAPTURE:
      handleCapture(command.sequence, command);
      break;
    case Operation::HOLD_SET:
      handleHoldSet(command.sequence, command);
      break;
    case Operation::TORQUE_LEASE:
      handleTorqueLease(command.sequence, command);
      break;
    case Operation::TORQUE_OFF:
      handleTorqueOff(command.sequence, command);
      break;
    case Operation::PREPARE_NUDGE:
      handlePrepareNudge(command.sequence, command);
      break;
    case Operation::EXECUTE_NUDGE:
      handleExecuteNudge(command.sequence, command);
      break;
    case Operation::STOP:
      handleStop(command.sequence, command);
      break;
    case Operation::RESET:
      handleReset(command.sequence, command);
      break;
    case Operation::ODO_ZERO:
      handleOdometerZero(command.sequence, command);
      break;
    case Operation::ODO_READ:
      handleOdometerRead(command.sequence, command);
      break;
    case Operation::REG_READ:
      handleRegisterRead(command.sequence, command);
      break;
    case Operation::REG_WRITE:
      handleRegisterWrite(command.sequence, command);
      break;
    case Operation::CONFIG:
      handleConfig(command.sequence, command);
      break;
    case Operation::MOVE:
      handleMove(command.sequence, command);
      break;
    case Operation::MOVE_SET:
      handleMoveSet(command.sequence, command);
      break;
    case Operation::FOLLOW_SET:
      handleFollowSet(command.sequence, command);
      break;
    case Operation::FOLLOW_READ:
      handleFollowRead(command.sequence, command);
      break;
  }
}

void ArmHatRuntime::sendPolicyJson() {
  append("{\"maxDeltaTicks\":%u,\"motionBudgetMs\":%u,"
         "\"positionToleranceTicks\":%u,\"executionTimeoutMs\":%u,"
         "\"maxSpeed\":%u,\"maxAccel\":%u,\"supervisionFaultTolerance\":%u}",
         static_cast<unsigned>(policy_.maxDeltaTicks),
         static_cast<unsigned>(policy_.motionBudgetMs),
         static_cast<unsigned>(policy_.positionToleranceTicks),
         static_cast<unsigned>(policy_.executionTimeoutMs),
         static_cast<unsigned>(policy_.maxSpeed),
         static_cast<unsigned>(policy_.maxAccel),
         static_cast<unsigned>(policy_.supervisionFaultTolerance));
}

void ArmHatRuntime::appendBusNoise() {
  const BusNoise& noise = bus_.noise();
  append(",\"busNoise\":{\"bytes\":%lu,\"allZero\":%s,\"allOnes\":%s,"
         "\"sample\":\"",
         static_cast<unsigned long>(noise.bytes),
         noise.allZero ? "true" : "false",
         noise.allOnes ? "true" : "false");
  for (uint8_t index = 0; index < noise.sampleCount; ++index) {
    append("%02x", static_cast<unsigned>(noise.sample[index]));
  }
  append("\"}");
}

bool ArmHatRuntime::multiTurnPositionOf(const ServoTelemetry& servo,
                                        int32_t& position) const {
  if (!bus_.isMultiTurn(servo.id)) {
    position = static_cast<int32_t>(servo.rawPosition);
    return true;
  }
  // Without an armed, unbroken odometer the wrap count is a guess, and a guess
  // here writes a goal a whole turn away from the truth. Refuse instead.
  if (!servo.odometerTracking || !servo.odometerValid ||
      !multiTurnTruthComplete(servo.multiTurnTruth)) {
    return false;
  }
  position = servo.revolutions * (static_cast<int32_t>(ST3215_POSITION_MAX) + 1) +
             static_cast<int32_t>(servo.odometerLastRaw);
  return position >= -ST3215_MULTI_TURN_MAX && position <= ST3215_MULTI_TURN_MAX;
}

void ArmHatRuntime::updateMultiTurnEstimate(ServoTelemetry& servo,
                                            int32_t remaining,
                                            uint32_t sampledAtMs) {
  const MultiTurnCountdownResult truthResult = observeMultiTurnCountdown(
      servo.multiTurnTruth, remaining, MULTI_TURN_ARRIVED_TICKS);
  if (truthResult == MultiTurnCountdownResult::RESYNC_REQUIRED) {
    servo.odometerValid = false;
  }

  const int32_t turn = static_cast<int32_t>(ST3215_POSITION_MAX) + 1;
  // The helper retains the last wire-supported reference on ambiguity; it
  // never substitutes an ACKed command endpoint for physical truth.
  const int32_t estimate = servo.multiTurnTruth.referencePosition;
  int32_t revolutions = estimate / turn;
  int32_t rawPart = estimate - revolutions * turn;
  if (rawPart < 0) {
    rawPart += turn;
    --revolutions;
  }
  servo.revolutions = revolutions;
  servo.odometerLastRaw = static_cast<uint16_t>(rawPart);
  servo.odometerSampledAtMs = sampledAtMs;
}

bool ArmHatRuntime::resyncMultiTurnFromEncoder(ServoTelemetry& servo) {
  if (!bus_.isMultiTurn(servo.id)) {
    return true;
  }
  if (servo.torque != TorqueState::OFF) {
    return false;
  }

  // Once parked in mode 0, normal samples track real encoder wraps. A valid
  // parked frame needs no reinterpretation; an invalid one has already lost
  // continuity and must never be "healed" by choosing the nearest turn later.
  if (servo.operatingModeKnown && servo.operatingMode == 0) {
    return true;
  }

  const bool referenceUsable =
      servo.odometerValid || multiTurnResyncRequired(servo.multiTurnTruth);

  // `odometerLastRaw` is the last trusted position before authority was removed.
  // At the configured speed and 10 ms sample cadence, the shaft cannot cross
  // half a motor turn before torque-off, so the nearest wrapped turn is
  // unambiguous. Leave the free joint in mode 0 afterwards: only there does
  // register 56 remain a real encoder while a person can back-drive the Base.
  const int32_t reference =
      servo.revolutions * MULTI_TURN_ENCODER_TICKS +
      static_cast<int32_t>(servo.odometerLastRaw);
  clearMultiTurnGoal(servo.id);
  servo.odometerValid = false;

  uint8_t mode = 0;
  uint8_t modeError = 0;
  if (!bus_.writeOperatingMode(servo.id, 0) ||
      bus_.readOperatingMode(servo.id, mode, modeError) != BusResult::OK ||
      mode != 0) {
    return false;
  }
  servo.operatingModeKnown = true;
  servo.operatingMode = 0;

  uint8_t bytes[2] = {};
  uint8_t positionError = 0;
  const BusResult positionResult = bus_.readRegisters(
      servo.id, REGISTER_PRESENT_POSITION, sizeof(bytes), bytes,
      positionError);
  const uint16_t trueRaw = littleEndian16(bytes);
  if (positionResult != BusResult::OK || trueRaw > ST3215_POSITION_MAX) {
    // Leaving the servo torque-off in mode 0 is safe, and a later encoder read
    // can recover. Never return to step mode without a verified counted frame.
    return false;
  }
  if (!referenceUsable) {
    // Mode 0 is proven and electrically safe, but the missing interval could
    // contain any number of motor turns. Preserve the current wrapped sample
    // only as a starting observation; explicit physical home is required.
    servo.odometerLastRaw = trueRaw;
    servo.odometerSampledAtMs = millis();
    forgetMultiTurnTruth(servo.multiTurnTruth);
    servo.odometerValid = false;
    busResponseSeen_ = true;
    return true;
  }
  int32_t actual = 0;
  if (!resyncMultiTurnTruthFromWrappedEncoder(
          servo.multiTurnTruth, reference, trueRaw, actual)) {
    servo.odometerValid = false;
    return false;
  }

  servo.operatingModeKnown = true;
  servo.operatingMode = 0;
  servo.revolutions = actual / MULTI_TURN_ENCODER_TICKS;
  int32_t rawPart = actual - servo.revolutions * MULTI_TURN_ENCODER_TICKS;
  if (rawPart < 0) {
    rawPart += MULTI_TURN_ENCODER_TICKS;
    --servo.revolutions;
  }
  servo.odometerLastRaw = static_cast<uint16_t>(rawPart);
  servo.odometerSampledAtMs = millis();
  servo.odometerValid = true;
  busResponseSeen_ = true;
  return true;
}

void ArmHatRuntime::handleMultiTurn(uint32_t sequence,
                                    const ParsedCommand& command) {
  uint8_t id = 0;
  if (command.argumentCount != 2 || !parseServoId(command.arguments[0], id)) {
    sendError(sequence, "BAD_ARGS");
    return;
  }
  bool enabled = false;
  if (std::strcmp(command.arguments[1], "ON") == 0) {
    enabled = true;
  } else if (std::strcmp(command.arguments[1], "OFF") != 0) {
    sendError(sequence, "BAD_ARGS");
    return;
  }
  // Extended absolute mode changes EEPROM-backed interpretation registers.
  // Prove torque off first, then perform the vendor sequence under an unlocked
  // EEPROM and verify every readback before advertising the capability.
  clearMultiTurnGoal(id);
  if (!resolvePendingTorqueOffFor(id)) {
    busDegraded_ = true;
    sendError(sequence, "TORQUE_UNCONFIRMED",
              "{\"phase\":\"multi_turn_torque_off\"}");
    return;
  }

  const auto failConfig = [&](const char* phase) {
    (void)bus_.writeLock(id, true);
    busDegraded_ = true;
    ServoTelemetry* failed = findServo(id);
    if (failed != nullptr) {
      failed->odometerValid = false;
      forgetMultiTurnTruth(failed->multiTurnTruth);
    }
    startResponse(sequence, false, "MULTI_TURN_CONFIG_FAILED");
    append("{\"phase\":\"%s\",\"torqueState\":\"off\"}", phase);
    sendResponse();
  };
  const auto readByte = [&](uint8_t address, uint8_t& value) {
    uint8_t servoError = 0;
    return bus_.readRegisters(id, address, 1, &value, servoError) ==
           BusResult::OK;
  };

  const uint16_t high = enabled ? 0 : ST3215_POSITION_MAX;
  const uint8_t wantedMode = 0;
  const auto finishConfig = [&](uint8_t phase, uint8_t resolution,
                                uint16_t minimum, uint16_t maximum) {
    ServoTelemetry* servo = findServo(id);
    if (servo != nullptr) {
      servo->operatingModeKnown = true;
      servo->operatingMode = wantedMode;
      // Register semantics are proven, but a servo-only power interruption can
      // reset its internal coordinate while the HAT keeps running. Require the
      // operator's explicit ODO_ZERO after every (re)configuration command.
      servo->odometerTracking = false;
      servo->odometerValid = false;
      forgetMultiTurnTruth(servo->multiTurnTruth);
    }
    bus_.setMultiTurn(id, enabled);
    removeHoldGoal(id);
    clearMultiTurnGoal(id);
    busResponseSeen_ = true;
    startResponse(sequence, true, nullptr);
    append("{\"servoId\":%u,\"multiTurn\":%s,\"angleMin\":%u,\"angleMax\":%u,"
           "\"operatingMode\":%u,\"phase\":%u,\"resolution\":%u}",
           static_cast<unsigned>(id), enabled ? "true" : "false",
           static_cast<unsigned>(minimum), static_cast<unsigned>(maximum),
           static_cast<unsigned>(wantedMode), static_cast<unsigned>(phase),
           static_cast<unsigned>(resolution));
    sendResponse();
  };

  // MULTITURN ON is routinely repeated after reconnect. Read the persisted
  // configuration while locked and take a zero-write fast path when it is
  // already exact; this avoids wearing EEPROM on every gateway restart.
  bool currentLocked = false;
  uint8_t preflightLockError = 0;
  uint8_t currentPhase = 0;
  uint8_t currentResolution = 0;
  uint16_t currentMin = 0;
  uint16_t currentMax = 0;
  uint8_t preflightLimitError = 0;
  uint8_t currentMode = 0;
  uint8_t preflightModeError = 0;
  const bool preflightRead =
      bus_.readLock(id, currentLocked, preflightLockError) == BusResult::OK &&
      readByte(REGISTER_PHASE, currentPhase) &&
      readByte(REGISTER_RESOLUTION, currentResolution) &&
      bus_.readPositionLimits(id, currentMin, currentMax,
                              preflightLimitError) == BusResult::OK &&
      bus_.readOperatingMode(id, currentMode, preflightModeError) ==
          BusResult::OK;
  if (!preflightRead) {
    failConfig("preflight_readback");
    return;
  }
  const bool phaseMatches =
      ((currentPhase & ST3215_PHASE_EXTENDED_POSITION) != 0) == enabled;
  const bool semanticConfigMatches =
      phaseMatches && currentResolution == ST3215_DEFAULT_RESOLUTION &&
      currentMin == 0 && currentMax == high && currentMode == wantedMode;
  if (semanticConfigMatches) {
    if (!currentLocked) {
      if (!bus_.writeLock(id, true)) {
        failConfig("relock_write");
        return;
      }
      bool relocked = false;
      uint8_t relockError = 0;
      if (bus_.readLock(id, relocked, relockError) != BusResult::OK ||
          !relocked) {
        failConfig("relock_readback");
        return;
      }
    }
    finishConfig(currentPhase, currentResolution, currentMin, currentMax);
    return;
  }

  if (!bus_.writeLock(id, false)) {
    failConfig("unlock_write");
    return;
  }
  bool unlocked = false;
  uint8_t lockError = 0;
  if (bus_.readLock(id, unlocked, lockError) != BusResult::OK || unlocked) {
    failConfig("unlock_readback");
    return;
  }

  uint8_t oldPhase = 0;
  if (!readByte(REGISTER_PHASE, oldPhase)) {
    failConfig("phase_read");
    return;
  }
  const uint8_t wantedPhase = enabled
                                  ? static_cast<uint8_t>(
                                        oldPhase | ST3215_PHASE_EXTENDED_POSITION)
                                  : static_cast<uint8_t>(
                                        oldPhase & ~ST3215_PHASE_EXTENDED_POSITION);
  if (!bus_.writeRegister(id, REGISTER_PHASE, &wantedPhase, 1)) {
    failConfig("phase_write");
    return;
  }
  uint8_t readPhase = 0;
  if (!readByte(REGISTER_PHASE, readPhase) || readPhase != wantedPhase) {
    failConfig("phase_readback");
    return;
  }

  // Resolution is EEPROM-backed too. The installed arm is commissioned at 1;
  // require that value without adding an unnecessary wear-producing write.
  uint8_t readResolution = 0;
  if (!readByte(REGISTER_RESOLUTION, readResolution) ||
      readResolution != ST3215_DEFAULT_RESOLUTION) {
    failConfig("resolution_readback");
    return;
  }

  const uint8_t maximum[] = {static_cast<uint8_t>(high & 0xFF),
                             static_cast<uint8_t>((high >> 8) & 0xFF)};
  const uint8_t minimum[] = {0, 0};
  if (!bus_.writeRegister(id, REGISTER_MAXIMUM_POSITION, maximum,
                          sizeof(maximum)) ||
      !bus_.writeRegister(id, REGISTER_MINIMUM_POSITION, minimum,
                          sizeof(minimum))) {
    failConfig("limits_write");
    return;
  }
  uint16_t readMin = 0;
  uint16_t readMax = 0;
  uint8_t servoError = 0;
  const BusResult result =
      bus_.readPositionLimits(id, readMin, readMax, servoError);
  if (result != BusResult::OK || readMin != 0 || readMax != high) {
    failConfig("limits_readback");
    return;
  }

  uint8_t readMode = 0;
  uint8_t modeError = 0;
  if (!bus_.writeOperatingMode(id, wantedMode) ||
      bus_.readOperatingMode(id, readMode, modeError) != BusResult::OK ||
      readMode != wantedMode) {
    failConfig("operating_mode_readback");
    return;
  }

  if (!bus_.writeLock(id, true)) {
    failConfig("relock_write");
    return;
  }
  bool locked = false;
  lockError = 0;
  if (bus_.readLock(id, locked, lockError) != BusResult::OK || !locked) {
    failConfig("relock_readback");
    return;
  }

  finishConfig(readPhase, readResolution, readMin, readMax);
}

void ArmHatRuntime::handleFamily(uint32_t sequence,
                                 const ParsedCommand& command) {
  uint8_t id = 0;
  if (command.argumentCount != 2 || !parseServoId(command.arguments[0], id)) {
    sendError(sequence, "BAD_ARGS");
    return;
  }
  ServoFamily family = ServoFamily::STS;
  if (std::strcmp(command.arguments[1], "SCS") == 0) {
    family = ServoFamily::SCS;
  } else if (std::strcmp(command.arguments[1], "STS") != 0) {
    sendError(sequence, "BAD_ARGS");
    return;
  }
  bus_.setFamily(id, family);
  // FAMILY is the Pi's declaration of configured inventory, replayed on every
  // connect. A fresh HAT used to keep servoCount_ at zero, so STATUS had no IDs
  // to refresh and Base stayed invisible until SCAN broadcast torque-off to the
  // whole bus. Register the declared ID without touching the servo bus; the
  // next normal STATUS proves whether it is actually present.
  ServoTelemetry* servo = rememberServo(id);
  if (servo == nullptr) {
    sendError(sequence, "CAPACITY");
    return;
  }
  servo->family = family;
  // Whatever was cached for this id was decoded under the other byte order,
  // so it is not merely stale, it is wrong. Force a fresh read.
  servo->fresh = false;
  servo->sampled = false;
  startResponse(sequence, true, nullptr);
  append("{\"servoId\":%u,\"family\":\"%s\"}", static_cast<unsigned>(id),
         family == ServoFamily::SCS ? "scs" : "sts");
  sendResponse();
}

void ArmHatRuntime::handleConfig(uint32_t sequence,
                                 const ParsedCommand& command) {
  if (command.argumentCount == 0) {
    startResponse(sequence, true, nullptr);
    sendPolicyJson();
    sendResponse();
    return;
  }
  uint32_t value = 0;
  if (command.argumentCount != 2 || !parseUInt32(command.arguments[1], value) ||
      value == 0) {
    sendError(sequence, "BAD_ARGS");
    return;
  }
  const char* key = command.arguments[0];
  uint16_t* target = nullptr;
  uint16_t ceiling = 0;
  if (std::strcmp(key, "MAX_DELTA") == 0) {
    target = &policy_.maxDeltaTicks;
    ceiling = CONFIG_MAX_DELTA_CEILING;
  } else if (std::strcmp(key, "MOTION_BUDGET_MS") == 0) {
    target = &policy_.motionBudgetMs;
    ceiling = CONFIG_MOTION_BUDGET_CEILING_MS;
  } else if (std::strcmp(key, "POS_TOLERANCE") == 0) {
    target = &policy_.positionToleranceTicks;
    ceiling = CONFIG_POSITION_TOLERANCE_CEILING;
  } else if (std::strcmp(key, "EXEC_TIMEOUT_MS") == 0) {
    target = &policy_.executionTimeoutMs;
    ceiling = CONFIG_EXECUTION_TIMEOUT_CEILING_MS;
  } else if (std::strcmp(key, "MAX_SPEED") == 0) {
    target = &policy_.maxSpeed;
    ceiling = CONFIG_SPEED_CEILING;
  } else if (std::strcmp(key, "MAX_ACCEL") == 0) {
    target = &policy_.maxAccel;
    ceiling = CONFIG_ACCEL_CEILING;
  } else if (std::strcmp(key, "SUPERVISION_TOLERANCE") == 0) {
    target = &policy_.supervisionFaultTolerance;
    ceiling = CONFIG_SUPERVISION_TOLERANCE_CEILING;
  } else {
    sendError(sequence, "UNKNOWN_KEY");
    return;
  }
  if (value > ceiling) {
    sendError(sequence, "OUT_OF_RANGE");
    return;
  }
  *target = static_cast<uint16_t>(value);
  startResponse(sequence, true, nullptr);
  sendPolicyJson();
  sendResponse();
}

void ArmHatRuntime::handleRegisterRead(uint32_t sequence,
                                       const ParsedCommand& command) {
  uint32_t id = 0;
  uint32_t address = 0;
  uint32_t length = 0;
  if (command.argumentCount != 3 || !parseUInt32(command.arguments[0], id) ||
      !parseUInt32(command.arguments[1], address) ||
      !parseUInt32(command.arguments[2], length) || !validServoId(id) ||
      address > 255 || length < 1 || length > REGISTER_ACCESS_MAX_LENGTH) {
    sendError(sequence, "OUT_OF_RANGE");
    return;
  }
  uint8_t bytes[REGISTER_ACCESS_MAX_LENGTH] = {};
  uint8_t servoError = 0;
  const BusResult result =
      bus_.readRegisters(static_cast<uint8_t>(id), static_cast<uint8_t>(address),
                         static_cast<uint8_t>(length), bytes, servoError);
  if (result != BusResult::OK && result != BusResult::SERVO_ERROR) {
    busDegraded_ = true;
    sendError(sequence, "BUS_ERROR");
    return;
  }
  busResponseSeen_ = true;
  startResponse(sequence, true, nullptr);
  append("{\"servoId\":%u,\"address\":%u,\"length\":%u,\"statusError\":%u,"
         "\"values\":[",
         static_cast<unsigned>(id), static_cast<unsigned>(address),
         static_cast<unsigned>(length), static_cast<unsigned>(servoError));
  for (uint32_t index = 0; index < length; ++index) {
    append("%s%u", index == 0 ? "" : ",", static_cast<unsigned>(bytes[index]));
  }
  append("]}");
  sendResponse();
}

void ArmHatRuntime::handleRegisterWrite(uint32_t sequence,
                                        const ParsedCommand& command) {
  (void)command;
  // Arbitrary writes cannot be made safe on an assembled arm. Besides ID,
  // lock and torque, the servo exposes operating mode, angle limits and the
  // goal block as ordinary registers. Letting REG_WRITE reach any of those
  // bypasses the heartbeat, authority and multi-turn truth state machines.
  // Dedicated operations own every supported mutation; REG_READ remains the
  // read-only bench diagnostic path.
  sendError(sequence, "REGISTER_WRITE_DISABLED");
}

void ArmHatRuntime::handleMove(uint32_t sequence,
                               const ParsedCommand& command) {
  uint32_t id = 0;
  int32_t goal = 0;
  uint32_t speed = 0;
  uint32_t acceleration = 0;
  // Signed: a multi-turn joint sits either side of its own zero, and clamping
  // the goal to one turn is exactly the cap multi-turn exists to lift.
  if (command.argumentCount != 4 || !parseUInt32(command.arguments[0], id) ||
      !parseInt32(command.arguments[1], goal) ||
      !parseUInt32(command.arguments[2], speed) ||
      !parseUInt32(command.arguments[3], acceleration) || !validServoId(id) ||
      speed < 1 || speed > policy_.maxSpeed ||
      acceleration < 1 || acceleration > policy_.maxAccel) {
    sendError(sequence, "OUT_OF_RANGE");
    return;
  }
  const bool multiTurn = bus_.isMultiTurn(static_cast<uint8_t>(id));
  const int32_t goalLow = multiTurn ? -ST3215_MULTI_TURN_MAX : 0;
  const int32_t goalHigh =
      multiTurn ? ST3215_MULTI_TURN_MAX
                : static_cast<int32_t>(ST3215_POSITION_MAX);
  if (goal < goalLow || goal > goalHigh) {
    sendError(sequence, "OUT_OF_RANGE");
    return;
  }
  const uint32_t nowMs = millis();
  if (safety_.stopped()) {
    sendError(sequence, "STOPPED");
    return;
  }
  if (!safety_.heartbeatFresh(nowMs)) {
    sendError(sequence, "HEARTBEAT_STALE");
    return;
  }
  // Motion still requires live torque authority, so a silent host de-energises
  // the joint exactly as before. Deliberately `torqueAuthorizedFor` and not
  // `torqueLeaseActive`: a hold-set is equally valid authority and is the only
  // one that can be renewed, because TORQUE_LEASE refuses while a lease is
  // already running. Requiring a lease made continuous driving impossible.
  if (!safety_.torqueAuthorizedFor(static_cast<uint8_t>(id), nowMs)) {
    sendError(sequence, "NO_TORQUE_LEASE");
    return;
  }
  if (multiTurn) {
    // Native extended-position mode consumes a signed absolute target. The
    // servo's own closed loop compares that target with its encoder feedback;
    // repeating the same MOVE is therefore idempotent rather than cumulative.
    ServoTelemetry* servo = findServo(static_cast<uint8_t>(id));
    if (servo == nullptr || !servo->online || !servo->fresh || !servo->sampled ||
        servo->error != 0 || servo->statusError != 0 ||
        !telemetryContractValid(*servo, nowMs) || !servo->odometerTracking ||
        !servo->odometerValid ||
        !servo->operatingModeKnown || servo->operatingMode != 0) {
      sendError(sequence, "ODOMETER_UNAVAILABLE");
      return;
    }
    if (!bus_.writePosition(static_cast<uint8_t>(id), goal,
                            static_cast<uint16_t>(speed),
                            static_cast<uint8_t>(acceleration)) ||
        !bus_.verifyPositionCommand(static_cast<uint8_t>(id), goal,
                                    static_cast<uint16_t>(speed),
                                    static_cast<uint8_t>(acceleration))) {
      busDegraded_ = true;
      sendError(sequence, "BUS_ERROR");
      return;
    }
  } else if (!bus_.writePosition(static_cast<uint8_t>(id), goal,
                                 static_cast<uint16_t>(speed),
                                 static_cast<uint8_t>(acceleration))) {
    busDegraded_ = true;
    sendError(sequence, "BUS_ERROR");
    return;
  }
  busResponseSeen_ = true;
  startResponse(sequence, true, nullptr);
  append("{\"servoId\":%u,\"goal\":%ld,\"speed\":%u,\"acceleration\":%u}",
         static_cast<unsigned>(id), static_cast<long>(goal),
         static_cast<unsigned>(speed), static_cast<unsigned>(acceleration));
  sendResponse();
}

bool ArmHatRuntime::stopMoveSetMembersAndConfirm(
    const MoveSetGoal* goals, uint8_t goalCount) {
  // MOVE_SET uses SYNC_WRITE, which has no deferred register state. On an
  // ambiguous family dispatch or readback, addressed torque-off is sufficient:
  // there is no later trigger capable of reviving a stale command.
  bool off = goals != nullptr && goalCount > 0;
  // Minimise coast during the addressed proof loop. The broadcast is only a
  // best-effort halt and never counted as proof; every member still gets its
  // own write plus torque-register readback below.
  const bool broadcastSent = bus_.broadcastTorqueOff();
  for (uint8_t index = 0; index < goalCount; ++index) {
    off = torqueOffAndConfirm(goals[index].servoId) && off;
  }
  (void)broadcastSent;
  // Addressed register readback is the proof. A short/failed best-effort
  // broadcast must not erase proof that every named member is actually off.
  return off;
}

void ArmHatRuntime::failMoveSetClosed(uint32_t sequence,
                                      const MoveSetGoal* goals,
                                      uint8_t goalCount, const char* phase,
                                      uint8_t failedIndex,
                                      uint8_t dispatchedFamilyCount) {
  safety_.revokeAuthority();
  clearHoldGoals();
  invalidateProposal();
  busDegraded_ = true;
  const bool off = stopMoveSetMembersAndConfirm(goals, goalCount);
  safetyFault_ = true;
  // An ambiguous grouped dispatch still aborts, de-energises every member,
  // invalidates the reviewed command and forces a fresh measured plan. It is
  // not an operator STOP, so fresh safe telemetry—not RESET—is the recovery.
  operatorInspectionRequired_ = false;
  startResponse(sequence, false, "MOVE_SET_FAILED");
  const bool dispatchAmbiguous =
      std::strcmp(phase, "dispatch") == 0 || dispatchedFamilyCount > 0;
  append("{\"phase\":\"%s\",\"failedIndex\":%u,\"stopped\":false,"
         "\"torqueState\":\"%s\",\"latentCommands\":false,"
         "\"motionMayHaveStarted\":%s,\"partialDispatchPossible\":%s,"
         "\"dispatchedFamilyCount\":%u}",
         phase, static_cast<unsigned>(failedIndex),
         off ? "off" : "unknown",
         dispatchAmbiguous ? "true" : "false",
         dispatchAmbiguous ? "true" : "false",
         static_cast<unsigned>(dispatchedFamilyCount));
  sendResponse();
}

void ArmHatRuntime::handleMoveSet(uint32_t sequence,
                                  const ParsedCommand& command) {
  if (command.argumentCount < 1 ||
      command.argumentCount > MAX_HOLD_SERVOS) {
    sendError(sequence, "BAD_ARGS");
    return;
  }

  MoveSetGoal goals[MAX_HOLD_SERVOS] = {};
  for (uint8_t index = 0; index < command.argumentCount; ++index) {
    // Each goal is one token so four joints stay within the unchanged global
    // MAX_ARGUMENTS. Parsing is exact: no empty fields, suffixes or overflow.
    const char* cursor = command.arguments[index];
    char fields[4][16] = {};
    for (uint8_t field = 0; field < 4; ++field) {
      uint8_t length = 0;
      while (*cursor != '\0' && *cursor != ',') {
        if (length + 1 >= sizeof(fields[field])) {
          sendError(sequence, "BAD_ARGS");
          return;
        }
        fields[field][length++] = *cursor++;
      }
      if (length == 0 || (field < 3 && *cursor != ',') ||
          (field == 3 && *cursor != '\0')) {
        sendError(sequence, "BAD_ARGS");
        return;
      }
      if (field < 3) {
        ++cursor;
      }
    }

    uint32_t id = 0;
    uint32_t speed = 0;
    uint32_t acceleration = 0;
    if (!parseUInt32(fields[0], id) || !parseInt32(fields[1], goals[index].goal) ||
        !parseUInt32(fields[2], speed) ||
        !parseUInt32(fields[3], acceleration) || !validServoId(id) ||
        speed < 1 || speed > policy_.maxSpeed || acceleration < 1 ||
        acceleration > policy_.maxAccel) {
      sendError(sequence, "OUT_OF_RANGE");
      return;
    }
    for (uint8_t prior = 0; prior < index; ++prior) {
      if (goals[prior].servoId == id) {
        sendError(sequence, "BAD_ARGS");
        return;
      }
    }
    goals[index].servoId = static_cast<uint8_t>(id);
    goals[index].speed = static_cast<uint16_t>(speed);
    goals[index].acceleration = static_cast<uint8_t>(acceleration);

    const bool multiTurn = bus_.isMultiTurn(goals[index].servoId);
    const int32_t goalLow = multiTurn ? -ST3215_MULTI_TURN_MAX : 0;
    const int32_t goalHigh =
        multiTurn ? ST3215_MULTI_TURN_MAX
                  : static_cast<int32_t>(bus_.dialect(goals[index].servoId)
                                             .positionMax);
    if (goals[index].goal < goalLow || goals[index].goal > goalHigh) {
      sendError(sequence, "OUT_OF_RANGE");
      return;
    }
  }

  const uint32_t nowMs = millis();
  if (safety_.stopped()) {
    sendError(sequence, "STOPPED");
    return;
  }
  if (!safety_.heartbeatFresh(nowMs)) {
    sendError(sequence, "HEARTBEAT_STALE");
    return;
  }

  // Full preflight precedes the first SYNC_WRITE. No malformed member, missing
  // authority, or unusable multi-turn frame can dispatch a partial group.
  for (uint8_t index = 0; index < command.argumentCount; ++index) {
    const MoveSetGoal& goal = goals[index];
    if (!safety_.torqueAuthorizedFor(goal.servoId, nowMs)) {
      sendError(sequence, "NO_TORQUE_LEASE");
      return;
    }
    ServoTelemetry* servo = findServo(goal.servoId);
    if (servo == nullptr || !servo->online || !servo->fresh ||
        !servo->operatingModeKnown || servo->operatingMode != 0 ||
        servo->error != 0 || servo->statusError != 0 ||
        !telemetryContractValid(*servo, nowMs)) {
      sendError(sequence, "SERVO_ERROR");
      return;
    }
    if (bus_.isMultiTurn(goal.servoId) &&
        (!servo->odometerTracking || !servo->odometerValid ||
         !multiTurnTruthComplete(servo->multiTurnTruth))) {
      sendError(sequence, "ODOMETER_UNAVAILABLE");
      return;
    }
  }

  St3215Bus::PositionTarget sts[MAX_HOLD_SERVOS] = {};
  St3215Bus::PositionTarget scs[MAX_HOLD_SERVOS] = {};
  uint8_t stsCount = 0;
  uint8_t scsCount = 0;
  for (uint8_t index = 0; index < command.argumentCount; ++index) {
    const MoveSetGoal& goal = goals[index];
    St3215Bus::PositionTarget target{goal.servoId, goal.goal, goal.speed,
                                     goal.acceleration};
    if (bus_.familyOf(goal.servoId) == ServoFamily::SCS) {
      scs[scsCount++] = target;
    } else {
      sts[stsCount++] = target;
    }
  }
  // One packet per dialect is necessary because STS and SCS use different
  // address/length/endianness. Same-family members start together; mixed-family
  // sets have one bounded sequential packet gap and are not cross-family atomic.
  uint8_t dispatchedFamilyCount = 0;
  if (stsCount > 0) {
    if (!bus_.syncWritePositions(ServoFamily::STS, sts, stsCount)) {
      failMoveSetClosed(sequence, goals, command.argumentCount, "dispatch",
                        0, dispatchedFamilyCount);
      return;
    }
    ++dispatchedFamilyCount;
  }
  if (scsCount > 0) {
    if (!bus_.syncWritePositions(ServoFamily::SCS, scs, scsCount)) {
      failMoveSetClosed(sequence, goals, command.argumentCount, "dispatch",
                        0, dispatchedFamilyCount);
      return;
    }
    ++dispatchedFamilyCount;
  }

  // SYNC_WRITE has no ACK. Strict addressed readback of every goal is the
  // execution receipt; ambiguity follows STOP + addressed torque-off.
  for (uint8_t index = 0; index < command.argumentCount; ++index) {
    if (!bus_.verifyPositionCommand(
            goals[index].servoId, goals[index].goal, goals[index].speed,
            goals[index].acceleration)) {
      failMoveSetClosed(sequence, goals, command.argumentCount, "verify", index,
                        dispatchedFamilyCount);
      return;
    }
  }
  busResponseSeen_ = true;
  busDegraded_ = false;
  startResponse(sequence, true, nullptr);
  append("{\"controllerId\":\"%s\",\"bootId\":\"boot-%08lx%08lx\","
         "\"firmwareVersion\":\"arm-hat-2.7.2\","
         "\"protocolVersion\":1,"
         "\"dispatch\":\"dialect_grouped_sync_write\","
         "\"crossFamilyAtomic\":false,\"count\":%u,\"moved\":[",
         controllerId_, static_cast<unsigned long>(bootId_ >> 32u),
         static_cast<unsigned long>(bootId_ & 0xFFFFFFFFu),
         static_cast<unsigned>(command.argumentCount));
  for (uint8_t index = 0; index < command.argumentCount; ++index) {
    append("%s{\"servoId\":%u,\"goal\":%ld,\"speed\":%u,"
           "\"acceleration\":%u}",
           index == 0 ? "" : ",",
           static_cast<unsigned>(goals[index].servoId),
           static_cast<long>(goals[index].goal),
           static_cast<unsigned>(goals[index].speed),
           static_cast<unsigned>(goals[index].acceleration));
  }
  append("]}");
  sendResponse();
}

void ArmHatRuntime::handleFollowSet(uint32_t sequence,
                                    const ParsedCommand& command) {
  // This low-latency path is intentionally narrower than MOVE_SET: the arm's
  // Shoulder and Elbow are both STS-family members, so exactly one grouped bus
  // packet is allowed and every success carries fresh feedback for both.
  constexpr uint8_t FOLLOW_MEMBER_COUNT = 2;
  if (command.argumentCount != FOLLOW_MEMBER_COUNT) {
    sendError(sequence, "BAD_ARGS");
    return;
  }

  MoveSetGoal goals[FOLLOW_MEMBER_COUNT] = {};
  for (uint8_t index = 0; index < FOLLOW_MEMBER_COUNT; ++index) {
    const char* cursor = command.arguments[index];
    char fields[4][16] = {};
    for (uint8_t field = 0; field < 4; ++field) {
      uint8_t length = 0;
      while (*cursor != '\0' && *cursor != ',') {
        if (length + 1 >= sizeof(fields[field])) {
          sendError(sequence, "BAD_ARGS");
          return;
        }
        fields[field][length++] = *cursor++;
      }
      if (length == 0 || (field < 3 && *cursor != ',') ||
          (field == 3 && *cursor != '\0')) {
        sendError(sequence, "BAD_ARGS");
        return;
      }
      if (field < 3) {
        ++cursor;
      }
    }

    uint32_t id = 0;
    uint32_t speed = 0;
    uint32_t acceleration = 0;
    if (!parseUInt32(fields[0], id) ||
        !parseInt32(fields[1], goals[index].goal) ||
        !parseUInt32(fields[2], speed) ||
        !parseUInt32(fields[3], acceleration) || !validServoId(id) ||
        speed < 1 || speed > policy_.maxSpeed || acceleration < 1 ||
        acceleration > policy_.maxAccel) {
      sendError(sequence, "OUT_OF_RANGE");
      return;
    }
    for (uint8_t prior = 0; prior < index; ++prior) {
      if (goals[prior].servoId == id) {
        sendError(sequence, "BAD_ARGS");
        return;
      }
    }
    goals[index].servoId = static_cast<uint8_t>(id);
    goals[index].speed = static_cast<uint16_t>(speed);
    goals[index].acceleration = static_cast<uint8_t>(acceleration);
    if (bus_.familyOf(goals[index].servoId) != ServoFamily::STS) {
      sendError(sequence, "UNSUPPORTED");
      return;
    }
    const bool multiTurn = bus_.isMultiTurn(goals[index].servoId);
    const int32_t goalLow = multiTurn ? -ST3215_MULTI_TURN_MAX : 0;
    const int32_t goalHigh =
        multiTurn ? ST3215_MULTI_TURN_MAX : ST3215_POSITION_MAX;
    if (goals[index].goal < goalLow || goals[index].goal > goalHigh) {
      sendError(sequence, "OUT_OF_RANGE");
      return;
    }
  }

  const uint32_t nowMs = millis();
  if (safety_.stopped()) {
    sendError(sequence, "STOPPED");
    return;
  }
  if (!safety_.heartbeatFresh(nowMs)) {
    sendError(sequence, "HEARTBEAT_STALE");
    return;
  }
  for (uint8_t index = 0; index < FOLLOW_MEMBER_COUNT; ++index) {
    const MoveSetGoal& goal = goals[index];
    if (!safety_.torqueAuthorizedFor(goal.servoId, nowMs)) {
      sendError(sequence, "NO_TORQUE_LEASE");
      return;
    }
    ServoTelemetry* servo = findServo(goal.servoId);
    if (servo == nullptr || !servo->online || !servo->fresh ||
        !servo->operatingModeKnown || servo->operatingMode != 0 ||
        servo->error != 0 || servo->statusError != 0 ||
        !telemetryContractValid(*servo, nowMs)) {
      sendError(sequence, "SERVO_ERROR");
      return;
    }
    if (bus_.isMultiTurn(goal.servoId) &&
        (!servo->odometerTracking || !servo->odometerValid ||
         !multiTurnTruthComplete(servo->multiTurnTruth))) {
      sendError(sequence, "ODOMETER_UNAVAILABLE");
      return;
    }
  }

  St3215Bus::PositionTarget targets[FOLLOW_MEMBER_COUNT] = {};
  for (uint8_t index = 0; index < FOLLOW_MEMBER_COUNT; ++index) {
    targets[index] = St3215Bus::PositionTarget{
        goals[index].servoId, goals[index].goal, goals[index].speed,
        goals[index].acceleration};
  }
  if (!bus_.syncWritePositions(ServoFamily::STS, targets,
                               FOLLOW_MEMBER_COUNT)) {
    failMoveSetClosed(sequence, goals, FOLLOW_MEMBER_COUNT, "dispatch", 0,
                      0);
    return;
  }
  constexpr uint8_t DISPATCHED_FAMILY_COUNT = 1;
  for (uint8_t index = 0; index < FOLLOW_MEMBER_COUNT; ++index) {
    if (!bus_.verifyPositionCommand(
            goals[index].servoId, goals[index].goal, goals[index].speed,
            goals[index].acceleration)) {
      failMoveSetClosed(sequence, goals, FOLLOW_MEMBER_COUNT, "verify", index,
                        DISPATCHED_FAMILY_COUNT);
      return;
    }
  }

  ServoTelemetry* feedback[FOLLOW_MEMBER_COUNT] = {};
  for (uint8_t index = 0; index < FOLLOW_MEMBER_COUNT; ++index) {
    if (!refreshFollowServo(goals[index].servoId)) {
      failMoveSetClosed(sequence, goals, FOLLOW_MEMBER_COUNT, "feedback",
                        index, DISPATCHED_FAMILY_COUNT);
      return;
    }
    feedback[index] = findServo(goals[index].servoId);
    const uint32_t sampledNowMs = millis();
    ServoTelemetry* servo = feedback[index];
    if (servo == nullptr || servo->id != goals[index].servoId ||
        !servo->online || !servo->fresh || !servo->sampled ||
        !servo->operatingModeKnown || servo->operatingMode != 0 ||
        servo->torque != TorqueState::ON || servo->error != 0 ||
        servo->statusError != 0 ||
        !telemetryContractValid(*servo, sampledNowMs)) {
      failMoveSetClosed(sequence, goals, FOLLOW_MEMBER_COUNT, "feedback",
                        index, DISPATCHED_FAMILY_COUNT);
      return;
    }
  }

  const uint32_t receiptNowMs = millis();
  busResponseSeen_ = true;
  busDegraded_ = false;
  startResponse(sequence, true, nullptr);
  append("{\"controllerId\":\"%s\",\"bootId\":\"boot-%08lx%08lx\","
         "\"firmwareVersion\":\"arm-hat-2.7.2\","
         "\"protocolVersion\":1,"
         "\"dispatch\":\"dialect_grouped_sync_write\","
         "\"crossFamilyAtomic\":false,\"count\":%u,\"moved\":[",
         controllerId_, static_cast<unsigned long>(bootId_ >> 32u),
         static_cast<unsigned long>(bootId_ & 0xFFFFFFFFu),
         static_cast<unsigned>(FOLLOW_MEMBER_COUNT));
  for (uint8_t index = 0; index < FOLLOW_MEMBER_COUNT; ++index) {
    append("%s{\"servoId\":%u,\"goal\":%ld,\"speed\":%u,"
           "\"acceleration\":%u}",
           index == 0 ? "" : ",",
           static_cast<unsigned>(goals[index].servoId),
           static_cast<long>(goals[index].goal),
           static_cast<unsigned>(goals[index].speed),
           static_cast<unsigned>(goals[index].acceleration));
  }
  append("],\"feedback\":[");
  for (uint8_t index = 0; index < FOLLOW_MEMBER_COUNT; ++index) {
    const ServoTelemetry& servo = *feedback[index];
    append("%s{\"servoId\":%u,\"rawPosition\":%u,\"moving\":%s,"
           "\"packetAgeMs\":%lu,\"voltageDeciVolts\":%u,"
           "\"temperatureC\":%u}",
           index == 0 ? "" : ",", static_cast<unsigned>(servo.id),
           static_cast<unsigned>(servo.rawPosition),
           servo.moving != 0 ? "true" : "false",
           static_cast<unsigned long>(
               elapsedMs(receiptNowMs, servo.sampledAtMs)),
           static_cast<unsigned>(servo.voltage),
           static_cast<unsigned>(servo.temperature));
  }
  append("]}");
  sendResponse();
}

void ArmHatRuntime::handleFollowRead(uint32_t sequence,
                                     const ParsedCommand& command) {
  constexpr uint8_t FOLLOW_MEMBER_COUNT = 2;
  if (command.argumentCount != FOLLOW_MEMBER_COUNT) {
    sendError(sequence, "BAD_ARGS");
    return;
  }

  uint8_t ids[FOLLOW_MEMBER_COUNT] = {};
  for (uint8_t index = 0; index < FOLLOW_MEMBER_COUNT; ++index) {
    uint32_t id = 0;
    if (!parseUInt32(command.arguments[index], id) || !validServoId(id)) {
      sendError(sequence, "OUT_OF_RANGE");
      return;
    }
    ids[index] = static_cast<uint8_t>(id);
    if ((index > 0 && ids[0] == ids[index])) {
      sendError(sequence, "BAD_ARGS");
      return;
    }
    if (bus_.familyOf(ids[index]) != ServoFamily::STS) {
      sendError(sequence, "UNSUPPORTED");
      return;
    }
  }

  const uint32_t nowMs = millis();
  if (safety_.stopped()) {
    sendError(sequence, "STOPPED");
    return;
  }
  if (!safety_.heartbeatFresh(nowMs)) {
    sendError(sequence, "HEARTBEAT_STALE");
    return;
  }
  for (uint8_t index = 0; index < FOLLOW_MEMBER_COUNT; ++index) {
    if (!safety_.torqueAuthorizedFor(ids[index], nowMs)) {
      sendError(sequence, "NO_TORQUE_LEASE");
      return;
    }
    if (findServo(ids[index]) == nullptr) {
      sendError(sequence, "SERVO_ERROR");
      return;
    }
  }

  ServoTelemetry* feedback[FOLLOW_MEMBER_COUNT] = {};
  for (uint8_t index = 0; index < FOLLOW_MEMBER_COUNT; ++index) {
    ServoTelemetry* servo = findServo(ids[index]);
    if (!refreshFollowServo(ids[index])) {
      busDegraded_ = true;
      servo->online = false;
      servo->fresh = false;
      servo->torque = TorqueState::UNKNOWN;
      startResponse(sequence, false, "FEEDBACK_UNAVAILABLE");
      append("{\"failedIndex\":%u}", static_cast<unsigned>(index));
      sendResponse();
      return;
    }
    const uint32_t sampledNowMs = millis();
    if (!safety_.heartbeatFresh(sampledNowMs) ||
        !safety_.torqueAuthorizedFor(ids[index], sampledNowMs)) {
      sendError(sequence, "NO_TORQUE_LEASE");
      return;
    }
    if (servo->id != ids[index] || !servo->online || !servo->fresh ||
        !servo->sampled || !servo->operatingModeKnown ||
        servo->operatingMode != 0 || servo->torque != TorqueState::ON ||
        servo->error != 0 || servo->statusError != 0 ||
        !telemetryContractValid(*servo, sampledNowMs)) {
      startResponse(sequence, false, "SERVO_ERROR");
      append("{\"failedIndex\":%u}", static_cast<unsigned>(index));
      sendResponse();
      return;
    }
    feedback[index] = servo;
  }

  const uint32_t receiptNowMs = millis();
  busDegraded_ = false;
  startResponse(sequence, true, nullptr);
  append("{\"controllerId\":\"%s\",\"bootId\":\"boot-%08lx%08lx\","
         "\"firmwareVersion\":\"arm-hat-2.7.2\","
         "\"protocolVersion\":1,\"count\":%u,\"feedback\":[",
         controllerId_, static_cast<unsigned long>(bootId_ >> 32u),
         static_cast<unsigned long>(bootId_ & 0xFFFFFFFFu),
         static_cast<unsigned>(FOLLOW_MEMBER_COUNT));
  for (uint8_t index = 0; index < FOLLOW_MEMBER_COUNT; ++index) {
    const ServoTelemetry& servo = *feedback[index];
    append("%s{\"servoId\":%u,\"rawPosition\":%u,\"moving\":%s,"
           "\"packetAgeMs\":%lu,\"voltageDeciVolts\":%u,"
           "\"temperatureC\":%u}",
           index == 0 ? "" : ",", static_cast<unsigned>(servo.id),
           static_cast<unsigned>(servo.rawPosition),
           servo.moving != 0 ? "true" : "false",
           static_cast<unsigned long>(
               elapsedMs(receiptNowMs, servo.sampledAtMs)),
           static_cast<unsigned>(servo.voltage),
           static_cast<unsigned>(servo.temperature));
  }
  append("]}");
  sendResponse();
}

void ArmHatRuntime::handleOdometerZero(uint32_t sequence,
                                       const ParsedCommand& command) {
  uint32_t id = 0;
  if (command.argumentCount != 1 || !parseUInt32(command.arguments[0], id) ||
      !validServoId(id)) {
    sendError(sequence, "OUT_OF_RANGE");
    return;
  }
  ServoTelemetry* servo = findServo(static_cast<uint8_t>(id));
  if (servo == nullptr) {
    sendError(sequence, "UNKNOWN_SERVO");
    return;
  }
  // Re-homing calls wherever the joint is standing zero. A held joint may be
  // part way through a move measured against the OLD zero, so doing it now
  // moves the ground under that move: the remaining distance flips sign and the
  // joint turns back, then forth, endlessly. Refuse and let the caller drop
  // torque first if it really means to re-home.
  if (safety_.torqueAuthorizedFor(static_cast<uint8_t>(id), millis())) {
    sendError(sequence, "JOINT_HELD");
    return;
  }
  const bool multiTurn = bus_.isMultiTurn(static_cast<uint8_t>(id));
  if (multiTurn &&
      (!servo->operatingModeKnown || servo->operatingMode != 0)) {
    sendError(sequence, "MODE_NOT_POSITION",
              "{\"phase\":\"odometer_seed\",\"operatingMode\":null}");
    return;
  }
  // The frame every outstanding goal was measured in is about to change.
  clearMultiTurnGoal(static_cast<uint8_t>(id));
  uint8_t bytes[2] = {};
  uint8_t servoError = 0;
  const BusResult result = bus_.readRegisters(
      servo->id, REGISTER_PRESENT_POSITION, sizeof(bytes), bytes, servoError);
  const uint16_t raw = littleEndian16(bytes);
  int32_t extended = static_cast<int32_t>(raw);
  const bool positionValid =
      multiTurn ? decodeExtendedPosition(raw, extended)
                : raw <= ST3215_POSITION_MAX;
  if ((result != BusResult::OK && result != BusResult::SERVO_ERROR) ||
      !positionValid) {
    busDegraded_ = true;
    servo->odometerValid = false;
    sendError(sequence, "BUS_ERROR");
    return;
  }
  busResponseSeen_ = true;
  servo->odometerTracking = true;
  servo->odometerValid = true;
  if (multiTurn) {
    storeExtendedPosition(*servo, extended, millis(), false);
  } else {
    servo->revolutions = 0;
    servo->odometerLastRaw = raw;
    servo->odometerSampledAtMs = millis();
    seedMultiTurnTruth(servo->multiTurnTruth, raw);
  }
  lastOdometerSampleMs_ = servo->odometerSampledAtMs;
  startResponse(sequence, true, nullptr);
  sendOdometerJson(*servo, servo->odometerSampledAtMs);
  sendResponse();
}

void ArmHatRuntime::handleOdometerRead(uint32_t sequence,
                                       const ParsedCommand& command) {
  uint32_t id = 0;
  if (command.argumentCount != 1 || !parseUInt32(command.arguments[0], id) ||
      !validServoId(id)) {
    sendError(sequence, "OUT_OF_RANGE");
    return;
  }
  ServoTelemetry* servo = findServo(static_cast<uint8_t>(id));
  if (servo == nullptr) {
    sendError(sequence, "UNKNOWN_SERVO");
    return;
  }
  startResponse(sequence, true, nullptr);
  sendOdometerJson(*servo, millis());
  sendResponse();
}

bool ArmHatRuntime::startResponse(uint32_t sequence, bool ok,
                                  const char* errorCode) {
  responseLength_ = 0;
  response_[0] = '\0';
  if (ok) {
    return append("A1 %lu OK ", static_cast<unsigned long>(sequence));
  }
  return append("A1 %lu ERR %s ", static_cast<unsigned long>(sequence),
                errorCode == nullptr ? "INTERNAL" : errorCode);
}

bool ArmHatRuntime::append(const char* format, ...) {
  if (responseLength_ >= sizeof(response_)) {
    return false;
  }
  va_list arguments;
  va_start(arguments, format);
  const int written = std::vsnprintf(response_ + responseLength_,
                                     sizeof(response_) - responseLength_,
                                     format, arguments);
  va_end(arguments);
  if (written < 0 ||
      static_cast<std::size_t>(written) >=
          sizeof(response_) - responseLength_) {
    responseLength_ = sizeof(response_);
    return false;
  }
  responseLength_ += static_cast<std::size_t>(written);
  return true;
}

void ArmHatRuntime::sendResponse() {
  if (!append("\n")) {
    static const char fallback[] =
        "A1 0 ERR INTERNAL {\"reason\":\"response_overflow\"}\n";
    host_.write(reinterpret_cast<const uint8_t*>(fallback),
                sizeof(fallback) - 1);
    return;
  }
  host_.write(reinterpret_cast<const uint8_t*>(response_), responseLength_);
}

void ArmHatRuntime::sendError(uint32_t sequence, const char* code,
                              const char* details) {
  startResponse(sequence, false, code);
  append("%s", details);
  sendResponse();
}

const char* ArmHatRuntime::torqueName(TorqueState state) {
  switch (state) {
    case TorqueState::OFF:
      return "off";
    case TorqueState::ON:
      return "on";
    default:
      return "unknown";
  }
}

const char* ArmHatRuntime::parseErrorCode(ParseResult result) {
  switch (result) {
    case ParseResult::BAD_VERSION:
      return "BAD_VERSION";
    case ParseResult::BAD_SEQUENCE:
      return "BAD_SEQUENCE";
    case ParseResult::UNKNOWN_OPERATION:
      return "UNKNOWN_OP";
    case ParseResult::TOO_MANY_ARGUMENTS:
      return "BAD_ARGS";
    default:
      return "BAD_FORMAT";
  }
}

bool ArmHatRuntime::requireNoArguments(const ParsedCommand& command) {
  return command.argumentCount == 0;
}

ServoTelemetry* ArmHatRuntime::findServo(uint8_t id) {
  for (uint8_t index = 0; index < servoCount_; ++index) {
    if (servos_[index].id == id) {
      return &servos_[index];
    }
  }
  return nullptr;
}

ServoTelemetry* ArmHatRuntime::rememberServo(uint8_t id) {
  ServoTelemetry* existing = findServo(id);
  if (existing != nullptr) {
    return existing;
  }
  if (servoCount_ >= MAX_TRACKED_SERVOS) {
    return nullptr;
  }
  ServoTelemetry& servo = servos_[servoCount_++];
  servo = ServoTelemetry{};
  servo.id = id;
  return &servo;
}

bool ArmHatRuntime::reconcileCompletedScan(uint8_t minimum, uint8_t maximum,
                                           const uint8_t* discovered,
                                           uint8_t discoveredCount) {
  ServoTelemetry previous[MAX_TRACKED_SERVOS] = {};
  uint8_t existingIds[MAX_TRACKED_SERVOS] = {};
  const uint8_t previousCount = servoCount_;
  for (uint8_t index = 0; index < previousCount; ++index) {
    previous[index] = servos_[index];
    existingIds[index] = servos_[index].id;
  }

  uint8_t reconciledIds[MAX_TRACKED_SERVOS] = {};
  std::size_t reconciledCount = 0;
  if (!reconcileCompletedScanIds(
          existingIds, previousCount, minimum, maximum, discovered,
          discoveredCount, reconciledIds, MAX_TRACKED_SERVOS,
          reconciledCount)) {
    return false;
  }

  servoCount_ = 0;
  for (std::size_t desired = 0; desired < reconciledCount; ++desired) {
    ServoTelemetry telemetry{};
    telemetry.id = reconciledIds[desired];
    for (uint8_t prior = 0; prior < previousCount; ++prior) {
      if (previous[prior].id == telemetry.id) {
        telemetry = previous[prior];
        break;
      }
    }
    servos_[servoCount_++] = telemetry;
  }
  inventoryScanned_ = true;
  return true;
}

bool ArmHatRuntime::refreshServo(uint8_t id) {
  ServoTelemetry* servo = rememberServo(id);
  if (servo == nullptr) {
    return false;
  }
  const BusResult result = bus_.readTelemetry(id, *servo);
  if (result == BusResult::OK || result == BusResult::SERVO_ERROR) {
    busResponseSeen_ = true;
    const bool multiTurnModeUnknown =
        bus_.isMultiTurn(id) && !servo->operatingModeKnown;
    if (bus_.isMultiTurn(id) && servo->operatingModeKnown &&
        servo->operatingMode != 0) {
      servo->odometerValid = false;
      forgetMultiTurnTruth(servo->multiTurnTruth);
    }
    if (servo->torque == TorqueState::OFF &&
        safety_.torqueOffPendingFor(id)) {
      (void)safety_.markTorqueOffConfirmed(id);
    }
    const uint32_t observedAtMs = millis();
    const bool authorizedOn =
        servo->torque == TorqueState::ON &&
        safety_.torqueAuthorizedFor(id, observedAtMs);
    if (servo->torque != TorqueState::OFF && !authorizedOn) {
      // ON is legal only inside the exact live lease for this ID. UNKNOWN is
      // never authority to continue. Queue before STOP so neither state loss
      // nor a failed addressed write/read-back can forget the servo.
      (void)safety_.queueTorqueOffObligation(id);
      safety_.revokeAuthority();
      safetyFault_ = true;
      (void)torqueOffAndConfirm(id);
      return false;
    }
    // Main telemetry plus torque still proves the servo is present, but an
    // absent operating-mode subread cannot authorize interpretation or motion.
    // Keep the last Base anchor and make this sample explicitly stale.
    return !multiTurnModeUnknown && servo->fresh;
  }
  busDegraded_ = true;
  servo->online = false;
  servo->fresh = false;
  servo->torque = TorqueState::UNKNOWN;
  if (bus_.isMultiTurn(id)) {
    // Silence is not proof that the native absolute frame changed. Retain the
    // last wire-supported anchor as stale; online/fresh/torque already block all
    // dispatch. The cached mode is not current evidence either: marking it
    // unknown prevents the position-only sampler from closing this gap. A later
    // complete Mode-0 sample must pass the continuity guard, including exact
    // whole-turn-collapse detection across this gap.
    servo->operatingModeKnown = false;
    servo->nativeSampleMissed = true;
  }
  return false;
}

bool ArmHatRuntime::refreshFollowServo(uint8_t id) {
  for (uint8_t attempt = 0; attempt < FOLLOW_FEEDBACK_ATTEMPTS; ++attempt) {
    if (refreshServo(id)) {
      return true;
    }
    if (attempt + 1 < FOLLOW_FEEDBACK_ATTEMPTS) {
      delayMicroseconds(FOLLOW_FEEDBACK_RETRY_DELAY_US);
    }
  }
  return false;
}

ArmHatRuntime::MultiTurnGoal* ArmHatRuntime::findMultiTurnGoal(uint8_t servoId) {
  for (auto& goal : multiTurnGoals_) {
    if (goal.active && goal.servoId == servoId) {
      return &goal;
    }
  }
  return nullptr;
}

void ArmHatRuntime::clearMultiTurnGoal(uint8_t servoId) {
  for (auto& goal : multiTurnGoals_) {
    if (goal.servoId == servoId) {
      goal = MultiTurnGoal{};
    }
  }
}

bool ArmHatRuntime::setMultiTurnGoal(uint8_t servoId, int32_t target,
                                     uint16_t speed, uint8_t acceleration) {
  clearMultiTurnGoal(servoId);
  for (auto& goal : multiTurnGoals_) {
    if (goal.active) {
      continue;
    }
    goal.servoId = servoId;
    goal.active = true;
    goal.target = target;
    goal.speed = speed;
    goal.acceleration = acceleration;
    // Zero, not now: the first step goes out immediately rather than waiting a
    // whole interval, so a short move feels like one command and not a stutter.
    goal.lastStepMs = 0;
    goal.lastRemaining = 0;
    goal.lastProgressMs = millis();
    return true;
  }
  return false;
}

void ArmHatRuntime::stepMultiTurnGoals() {
  const uint32_t nowMs = millis();
  for (auto& goal : multiTurnGoals_) {
    if (!goal.active) {
      continue;
    }
    ServoTelemetry* servo = findServo(goal.servoId);
    // Every one of these means the joint must stop being commanded rather than
    // be driven on a guess: no servo, no longer multi-turn, STOP latched,
    // authority lapsed, or an estimate that can no longer be trusted.
    const bool authorityLive =
        safety_.torqueAuthorizedFor(goal.servoId, nowMs);
    if (servo == nullptr || !bus_.isMultiTurn(goal.servoId) ||
        !servo->operatingModeKnown || servo->operatingMode != 3 ||
        safety_.stopped() || !authorityLive || !servo->odometerTracking ||
        !servo->odometerValid) {
      const bool unfinishedStep =
          servo != nullptr && multiTurnStepOutstanding(servo->multiTurnTruth);
      if (unfinishedStep) {
        // An unfinished relative step can be cancelled by torque-off. Its
        // remaining-count register may then read zero, which would turn the
        // requested target into a fake measured position. Refuse that frame
        // until the real mode-0 encoder is reconciled.
        invalidateMultiTurnTruth(servo->multiTurnTruth);
        servo->odometerValid = false;
      }
      goal.active = false;
      if (unfinishedStep) {
        (void)safety_.removeHoldServo(goal.servoId);
        (void)torqueOffAndConfirm(goal.servoId);
      }
      continue;
    }
    if (goal.lastStepMs != 0 &&
        elapsedMs(nowMs, goal.lastStepMs) < MULTI_TURN_STEP_INTERVAL_MS) {
      continue;
    }
    goal.lastStepMs = nowMs;
    if (multiTurnStepOutstanding(servo->multiTurnTruth)) {
      // Do not add a new relative command while the old one is still in
      // flight. The only live hardware experiment covered same-sign addition;
      // waiting for zero makes an opposite-side retarget deterministic too.
      const int32_t remaining = servo->multiTurnTruth.remaining;
      if (remaining != goal.lastRemaining) {
        goal.lastRemaining = remaining;
        goal.lastProgressMs = nowMs;
      } else if (elapsedMs(nowMs, goal.lastProgressMs) >=
                 MULTI_TURN_STALL_MS) {
        goal.active = false;
        invalidateMultiTurnTruth(servo->multiTurnTruth);
        servo->odometerValid = false;
        (void)safety_.removeHoldServo(goal.servoId);
        (void)torqueOffAndConfirm(goal.servoId);
      }
      continue;
    }

    // With no relative step outstanding, the write that closes the gap is
    // simply target - the last physically completed/resynchronised position.
    int32_t want = 0;
    if (!multiTurnDeltaToTarget(servo->multiTurnTruth, goal.target, want)) {
      goal.active = false;
      invalidateMultiTurnTruth(servo->multiTurnTruth);
      servo->odometerValid = false;
      (void)safety_.removeHoldServo(goal.servoId);
      (void)torqueOffAndConfirm(goal.servoId);
      continue;
    }
    if (want != 0) {
      const int32_t step =
          want > ST3215_MULTI_TURN_MAX
              ? ST3215_MULTI_TURN_MAX
              : (want < -ST3215_MULTI_TURN_MAX ? -ST3215_MULTI_TURN_MAX
                                               : want);
      if (!bus_.writePosition(goal.servoId, step, goal.speed,
                              goal.acceleration)) {
        // No ACK is not proof of no command: the packet may have reached the
        // servo and its status reply may be the part that was lost. A relative
        // step could therefore already be running even though we cannot add it
        // to the committed ledger. Kill authority and reconcile from the real
        // mode-0 encoder; leaving the hold live here permits untracked motion.
        busDegraded_ = true;
        goal.active = false;
        invalidateMultiTurnTruth(servo->multiTurnTruth);
        servo->odometerValid = false;
        (void)safety_.removeHoldServo(goal.servoId);
        (void)torqueOffAndConfirm(goal.servoId);
        continue;
      }
      if (!beginAckedMultiTurnStep(servo->multiTurnTruth, step)) {
        // The ACK may already have started motion. A ledger transition failure
        // is therefore ambiguous in exactly the same way as a lost ACK.
        goal.active = false;
        invalidateMultiTurnTruth(servo->multiTurnTruth);
        servo->odometerValid = false;
        (void)safety_.removeHoldServo(goal.servoId);
        (void)torqueOffAndConfirm(goal.servoId);
        continue;
      }
      goal.lastProgressMs = nowMs;

      // The addressed write ACK proves the servo accepted the packet, but the
      // servo can publish the new register-56 countdown a few update cycles
      // later. The old one-shot read after 1.2 ms regularly saw the previous
      // resting zero, declared RESYNC_REQUIRED, and dropped a healthy Base.
      // Poll for a bounded grace period. A large command that remains zero is
      // still never promoted to truth: the existing transition below forces a
      // torque-off mode-0 reconciliation at the deadline.
      const uint32_t verifyStartedAtMs = millis();
      uint8_t bytes[2] = {};
      uint8_t servoError = 0;
      BusResult verify = BusResult::TIMEOUT;
      int32_t remaining = 0;
      bool remainingValid = false;
      do {
        delayMicroseconds(MULTI_TURN_COUNTDOWN_POLL_US);
        verify = bus_.readRegisters(goal.servoId, REGISTER_PRESENT_POSITION,
                                    sizeof(bytes), bytes, servoError);
        if (verify == BusResult::OK) {
          remaining = decodeSignedMagnitude16(littleEndian16(bytes));
          remainingValid = remaining >= -ST3215_MULTI_TURN_MAX &&
                           remaining <= ST3215_MULTI_TURN_MAX;
          if (remainingValid &&
              (remaining != 0 ||
               (step <= MULTI_TURN_ARRIVED_TICKS &&
                step >= -MULTI_TURN_ARRIVED_TICKS))) {
            break;
          }
        }
      } while (elapsedMs(millis(), verifyStartedAtMs) <
               MULTI_TURN_COUNTDOWN_START_GRACE_MS);
      if (verify != BusResult::OK || !remainingValid) {
        busDegraded_ = true;
        goal.active = false;
        invalidateMultiTurnTruth(servo->multiTurnTruth);
        servo->odometerValid = false;
        (void)safety_.removeHoldServo(goal.servoId);
        (void)torqueOffAndConfirm(goal.servoId);
        continue;
      }
      updateMultiTurnEstimate(*servo, remaining, millis());
      if (multiTurnResyncRequired(servo->multiTurnTruth)) {
        goal.active = false;
        (void)safety_.removeHoldServo(goal.servoId);
        (void)torqueOffAndConfirm(goal.servoId);
        continue;
      }
      goal.lastRemaining = remaining;
      continue;
    }
    const int32_t remaining = servo->multiTurnTruth.remaining;
    if (multiTurnTruthComplete(servo->multiTurnTruth) &&
        remaining <= MULTI_TURN_ARRIVED_TICKS &&
        remaining >= -MULTI_TURN_ARRIVED_TICKS &&
        elapsedMs(nowMs, goal.lastProgressMs) >= 3 * MULTI_TURN_STEP_INTERVAL_MS) {
      goal.active = false;
      continue;
    }
    if (remaining != goal.lastRemaining) {
      goal.lastRemaining = remaining;
      goal.lastProgressMs = nowMs;
    }
  }
}

void ArmHatRuntime::sampleOdometers() {
  const uint32_t nowMs = millis();
  if (elapsedMs(nowMs, lastOdometerSampleMs_) < ODOMETER_SAMPLE_INTERVAL_MS) {
    return;
  }
  lastOdometerSampleMs_ = nowMs;
  for (uint8_t index = 0; index < servoCount_; ++index) {
    ServoTelemetry& servo = servos_[index];
    if (!servo.odometerTracking) {
      continue;
    }
    uint8_t bytes[2] = {};
    uint8_t servoError = 0;
    const BusResult result = bus_.readRegisters(
        servo.id, REGISTER_PRESENT_POSITION, sizeof(bytes), bytes, servoError);
    if (result != BusResult::OK && result != BusResult::SERVO_ERROR) {
      // A timeout/corrupt reply means only that this observation is stale. It
      // is not evidence that the Base crossed a wrap, and ordinary STATUS work
      // already makes gaps much longer than the old 25 ms cutoff. Preserve the
      // last wire-supported native position and try again next sample.
      if (bus_.isMultiTurn(servo.id) && servo.operatingModeKnown &&
          servo.operatingMode == 0) {
        servo.nativeSampleMissed = true;
        continue;
      }
      continue;
    }
    const uint16_t raw = littleEndian16(bytes);
    if (bus_.isMultiTurn(servo.id) && !servo.operatingModeKnown) {
      // A position-only background read cannot repair a missing mode proof.
      // Preserve the anchor as stale until refreshServo obtains Mode 0 and a
      // continuous signed-extended sample together.
      servo.nativeSampleMissed = true;
      continue;
    }
    if (bus_.isMultiTurn(servo.id) && servo.operatingMode != 0) {
      servo.odometerValid = false;
      forgetMultiTurnTruth(servo.multiTurnTruth);
      continue;
    }
    if (bus_.isMultiTurn(servo.id)) {
      int32_t position = 0;
      if (!decodeExtendedPosition(raw, position)) {
        servo.odometerValid = false;
        forgetMultiTurnTruth(servo.multiTurnTruth);
        continue;
      }
      storeExtendedPosition(servo, position, nowMs);
      continue;
    }
    if (raw > ST3215_POSITION_MAX) {
      servo.odometerValid = false;
      if (bus_.isMultiTurn(servo.id)) {
        forgetMultiTurnTruth(servo.multiTurnTruth);
      }
      continue;
    }
    if (servo.odometerValid) {
      const int32_t delta =
          static_cast<int32_t>(raw) - static_cast<int32_t>(servo.odometerLastRaw);
      if (delta > ODOMETER_WRAP_THRESHOLD_TICKS) {
        --servo.revolutions;
      } else if (delta < -ODOMETER_WRAP_THRESHOLD_TICKS) {
        ++servo.revolutions;
      }
      if (servo.revolutions > ODOMETER_MAX_REVOLUTIONS ||
          servo.revolutions < -ODOMETER_MAX_REVOLUTIONS) {
        // Far beyond any real joint: treat as a fault rather than wrap silently.
        servo.odometerValid = false;
      }
    }
    servo.odometerLastRaw = raw;
    servo.odometerSampledAtMs = nowMs;
    if (bus_.isMultiTurn(servo.id)) {
      if (servo.odometerValid) {
        // While parked torque-off in mode 0 the encoder, not the old mode-3
        // command ledger, is truth. A hand turn must therefore advance the
        // exact state that the next absolute target delta will consume.
        const int32_t counted =
            servo.revolutions * MULTI_TURN_ENCODER_TICKS +
            static_cast<int32_t>(servo.odometerLastRaw);
        seedMultiTurnTruth(servo.multiTurnTruth, counted);
      } else {
        // A successful endpoint read cannot reconstruct wraps hidden during a
        // broken free-mode interval. Keep the sample only as the next ODO_ZERO
        // observation and refuse to manufacture a counted frame from it.
        forgetMultiTurnTruth(servo.multiTurnTruth);
      }
    }
  }
}

bool ArmHatRuntime::confirmTorqueOff(uint8_t id) {
  // Off obligations deliberately outlive telemetry inventory. A retry for an
  // ID pruned by a completed scan must not silently add that stale ID back.
  ServoTelemetry* servo = findServo(id);
  if (!safety_.queueTorqueOffObligation(id)) {
    safety_.revokeAuthority();
    safetyFault_ = true;
    return false;
  }
  bool enabled = true;
  uint8_t servoError = 0;
  const BusResult result = bus_.readTorque(id, enabled, servoError);
  if (result != BusResult::OK && result != BusResult::SERVO_ERROR) {
    busDegraded_ = true;
    if (servo != nullptr) {
      servo->torque = TorqueState::UNKNOWN;
    }
    return false;
  }
  if (servo != nullptr) {
    servo->online = true;
    servo->torque = enabled ? TorqueState::ON : TorqueState::OFF;
  }
  busResponseSeen_ = true;
  if (!enabled) {
    (void)safety_.markTorqueOffConfirmed(id);
    if (servo != nullptr && bus_.isMultiTurn(id) &&
        !resyncMultiTurnFromEncoder(*servo)) {
      // Electrical safety is already proven. Keep that proof separate from
      // position truth: a failed mode-0 reconciliation disables multi-turn
      // motion but must not pretend torque is unknown or relatch STOP.
      busDegraded_ = true;
      servo->odometerValid = false;
    }
  }
  return !enabled;
}

bool ArmHatRuntime::confirmAllTrackedTorqueOff() {
  bool confirmed = true;
  for (uint8_t index = 0; index < servoCount_; ++index) {
    if (!confirmTorqueOff(servos_[index].id)) {
      confirmed = false;
    }
  }
  return confirmed;
}

bool ArmHatRuntime::torqueOffAndConfirm(uint8_t id) {
  if (!safety_.queueTorqueOffObligation(id)) {
    safety_.revokeAuthority();
    safetyFault_ = true;
    return false;
  }
  safety_.noteTorqueOffAttempt(id, millis());

  // Read-back is the electrical proof. Even if the write call reports a short
  // transfer, a subsequent addressed register-40 value of zero proves that the
  // servo is already off and safely resolves the retained obligation.
  (void)bus_.setTorque(id, false);
  return confirmTorqueOff(id);
}

bool ArmHatRuntime::resolvePendingTorqueOffFor(uint8_t id) {
  removeHoldGoal(id);
  (void)safety_.removeHoldServo(id);
  if (!safety_.queueTorqueOffObligation(id)) {
    return false;
  }
  return torqueOffAndConfirm(id) && !safety_.torqueOffPendingFor(id);
}

bool ArmHatRuntime::resolvePendingTorqueOff() {
  // This helper deliberately resolves the complete electrical-off set. Any
  // caller that uses it is abandoning every retained hold goal; targeted
  // release paths use torqueOffAndConfirm(id) instead.
  clearHoldGoals();
  if (!safety_.torqueOffPending()) {
    return true;
  }
  const uint16_t attempts = safety_.torqueOffPendingCount();
  bool allConfirmed = true;
  for (uint16_t attempt = 0;
       attempt < attempts && safety_.torqueOffPending(); ++attempt) {
    const uint8_t pendingId = safety_.torqueOffServoId();
    if (!torqueOffAndConfirm(pendingId)) {
      allConfirmed = false;
    }
  }
  if (allConfirmed && !safety_.torqueOffPending()) {
    return true;
  }
  // Never forget which servo might still be energized. Authority revocation
  // blocks motion while SafetyStateMachine retains the addressed retry
  // obligation.
  safety_.revokeAuthority();
  safetyFault_ = true;
  return false;
}

bool ArmHatRuntime::superviseActiveLease(uint32_t nowMs) {
  const bool singleLeaseActive = safety_.torqueLeaseActive(nowMs);
  const bool holdSetActive = safety_.holdSetActive(nowMs);
  if (!singleLeaseActive && !holdSetActive) {
    leaseSupervisionSeen_ = false;
    return true;
  }
  if (leaseSupervisionSeen_ &&
      elapsedMs(nowMs, lastLeaseSupervisionMs_) <
          LEASE_SUPERVISION_INTERVAL_MS) {
    return true;
  }

  leaseSupervisionSeen_ = true;
  lastLeaseSupervisionMs_ = nowMs;
  uint8_t supervisedIds[MAX_HOLD_SERVOS + 1] = {};
  uint8_t supervisedCount = 0;
  if (singleLeaseActive) {
    supervisedIds[supervisedCount++] = safety_.torqueLeaseId();
  }
  if (holdSetActive) {
    for (uint8_t index = 0; index < safety_.holdSetCount(); ++index) {
      const uint8_t id = safety_.holdSetServoId(index);
      bool duplicate = false;
      for (uint8_t prior = 0; prior < supervisedCount; ++prior) {
        duplicate = duplicate || supervisedIds[prior] == id;
      }
      if (!duplicate) {
        supervisedIds[supervisedCount++] = id;
      }
    }
  }

  uint8_t id = 0;
  bool healthy = supervisedCount > 0;
  for (uint8_t index = 0; index < supervisedCount; ++index) {
    id = supervisedIds[index];
    const bool readSucceeded = refreshServo(id);
    ServoTelemetry* servo = findServo(id);
    const bool servoHealthy =
        servo != nullptr &&
        validActiveLeaseTelemetry(
            readSucceeded, servo->torque == TorqueState::ON,
            servo->operatingModeKnown, servo->operatingMode, servo->error,
            servo->statusError,
            readSucceeded && servo->online && servo->fresh &&
                telemetryContractValid(*servo, millis()),
            0);
    if (!servoHealthy) {
      healthy = false;
      break;
    }
  }
  if (healthy) {
    supervisionFaults_ = 0;
    return true;
  }

  // One bad sample is not proof of a safety problem: a servo that is actually
  // moving can return a transient fault or status byte, and revoking on the
  // first of those latched STOP on every real move. Tolerate a bounded run of
  // consecutive bad samples, then treat it as sustained and revoke.
  ++supervisionFaults_;
  if (supervisionFaults_ <= policy_.supervisionFaultTolerance) {
    return true;
  }
  supervisionFaults_ = 0;

  // A timeout, unexpected torque state, wrong/unknown mode, servo fault, or
  // malformed sample immediately revokes motion authority. Queue the exact ID
  // before revoking authority: a failed addressed write/read-back must retain
  // the electrical-off obligation for the bounded retry loop.
  if (validServoId(id)) {
    (void)safety_.queueTorqueOffObligation(id);
  }
  safety_.revokeAuthority();
  clearHoldGoals();
  invalidateProposal();
  safetyFault_ = true;
  // Minimise coast first. Addressed write/readback follows to prove each
  // retained obligation rather than trusting this no-ACK broadcast.
  const bool broadcastSent = bus_.broadcastTorqueOff();
  const bool targetedConfirmed = resolvePendingTorqueOff();
  const bool trackedConfirmed =
      servoCount_ > 0 && confirmAllTrackedTorqueOff();
  (void)targetedConfirmed;
  (void)broadcastSent;
  (void)trackedConfirmed;
  // Preserve the automatic recovery gate after the emergency off/read-back.
  // A later complete STATUS proves every tracked servo healthy and off, then
  // clears this without requiring an operator RESET.
  safetyFault_ = true;
  return false;
}

void ArmHatRuntime::invalidateProposal() { proposal_ = Proposal{}; }

ArmHatRuntime::HoldGoal* ArmHatRuntime::findHoldGoal(uint8_t servoId) {
  for (uint8_t index = 0; index < holdGoalCount_; ++index) {
    if (holdGoals_[index].servoId == servoId) {
      return &holdGoals_[index];
    }
  }
  return nullptr;
}

const ArmHatRuntime::HoldGoal* ArmHatRuntime::findHoldGoal(
    uint8_t servoId) const {
  for (uint8_t index = 0; index < holdGoalCount_; ++index) {
    if (holdGoals_[index].servoId == servoId) {
      return &holdGoals_[index];
    }
  }
  return nullptr;
}

void ArmHatRuntime::removeHoldGoal(uint8_t servoId) {
  // Same reasoning as clearHoldGoals: this is table bookkeeping, and the
  // stepper's own authority check is what stops a joint that may not move.
  for (uint8_t index = 0; index < holdGoalCount_; ++index) {
    if (holdGoals_[index].servoId != servoId) {
      continue;
    }
    for (uint8_t move = index; move + 1 < holdGoalCount_; ++move) {
      holdGoals_[move] = holdGoals_[move + 1];
    }
    --holdGoalCount_;
    holdGoals_[holdGoalCount_] = HoldGoal{};
    return;
  }
}

void ArmHatRuntime::clearHoldGoals() {
  // Deliberately does NOT touch multiTurnGoals_. This is the hold TABLE being
  // rebuilt, which happens on every renewal -- clearing goals here killed a
  // multi-turn move 0.8 s after it started, every time. Loss of authority is
  // caught by the stepper itself, which rechecks stopped()/torqueAuthorizedFor
  // every 25 ms.
  for (uint8_t index = 0; index < MAX_HOLD_SERVOS; ++index) {
    holdGoals_[index] = HoldGoal{};
  }
  holdGoalCount_ = 0;
}

bool ArmHatRuntime::proposalFresh(uint32_t nowMs) const {
  return proposal_.active &&
         elapsedMs(nowMs, proposal_.createdAtMs) < PROPOSAL_TTL_MS;
}

const char* ArmHatRuntime::stateName(uint32_t nowMs) const {
  if (safety_.stopped()) {
    return "stopped_latched";
  }
  if (safetyFaultActive()) {
    return "faulted";
  }
  if (proposalFresh(nowMs)) {
    return "nudge_prepared";
  }
  if (safety_.torqueLeaseActive(nowMs)) {
    return "lease_active";
  }
  if (safety_.holdSetActive(nowMs)) {
    return "hold_active";
  }
  return "disarmed";
}

bool ArmHatRuntime::telemetryContractValid(const ServoTelemetry& servo,
                                           uint32_t nowMs) const {
  return servo.sampled && servo.rawPosition <= ST3215_POSITION_MAX &&
         servo.load >= -1000 && servo.load <= 1000 &&
         servo.temperature <= 150 &&
         elapsedMs(nowMs, servo.sampledAtMs) <= 86400000UL;
}

const char* ArmHatRuntime::aggregateTorqueState() const {
  if (servoCount_ == 0) {
    return "unknown";
  }
  bool unknown = false;
  for (uint8_t index = 0; index < servoCount_; ++index) {
    if (servos_[index].torque == TorqueState::ON) {
      return "on";
    }
    unknown = unknown || servos_[index].torque == TorqueState::UNKNOWN;
  }
  return unknown ? "unknown" : "off";
}

const char* ArmHatRuntime::aggregateServosState(uint32_t nowMs) const {
  if (servoCount_ == 0) {
    return "none_found";
  }
  for (uint8_t index = 0; index < servoCount_; ++index) {
    const ServoTelemetry& servo = servos_[index];
    if (!servo.online || !servo.fresh ||
        !telemetryContractValid(servo, nowMs)) {
      return "faulted";
    }
  }
  return "online";
}

const char* ArmHatRuntime::busStateName() const {
  // A controller STOP/recovery block is not evidence that the downstream
  // servo bus is bad. Keep those states separate so a latched red STOP cannot
  // manufacture the misleading "bus faulted" diagnosis seen by Terra.
  if (busDegraded_) {
    return "faulted";
  }
  return busResponseSeen_ ? "online" : "unknown";
}

const char* ArmHatRuntime::motionState(uint32_t) const {
  if (safety_.stopped()) {
    return "stopped";
  }
  if (safetyFaultActive()) {
    return "blocked";
  }
  for (uint8_t index = 0; index < servoCount_; ++index) {
    if (servos_[index].torque == TorqueState::ON &&
        servos_[index].moving != 0) {
      return "moving";
    }
  }
  return "ready";
}

bool ArmHatRuntime::safetyFaultActive() const {
  // The inspection latch is a safety fault even if a later diagnostic or
  // torque-off proof updates the ordinary transient fault bit. Only strict
  // RESET clears operatorInspectionRequired_, so reports cannot accidentally
  // advertise ready during same-boot recovery work.
  return safetyFault_ || safety_.stopped();
}

bool ArmHatRuntime::operatorClearRequired() const {
  // Firmware 2.5 watchdog expiry revokes authority without latching stopped_.
  // Consequently every stopped_ state represents either an explicit STOP or
  // a real controller safety fault and must survive a Pi process restart.
  return safety_.stopped();
}

const char* ArmHatRuntime::safetyStopReasonName() const {
  if (operatorInspectionRequired_) {
    return "MOVE_SET_FAILED";
  }
  if (explicitStopLatched_) {
    return "EXPLICIT_STOP";
  }
  return safety_.stopped() ? "SAFETY_FAULT" : nullptr;
}

void ArmHatRuntime::sendServoFields(const ServoTelemetry& servo,
                                    uint32_t nowMs) {
  const uint32_t ageMs = elapsedMs(nowMs, servo.sampledAtMs);
  append("\"id\":%u,\"rawPosition\":%u,\"speed\":%d,\"load\":%d,"
         "\"voltageVolts\":%u.%u,\"temperatureC\":%u,"
         "\"moving\":%s,\"currentRaw\":%u,\"torqueState\":\"%s\","
         "\"packetAgeMs\":%lu,\"errors\":[",
         static_cast<unsigned>(servo.id),
         static_cast<unsigned>(servo.rawPosition),
         static_cast<int>(servo.speed), static_cast<int>(servo.load),
         static_cast<unsigned>(servo.voltage / 10),
         static_cast<unsigned>(servo.voltage % 10),
         static_cast<unsigned>(servo.temperature),
         servo.moving != 0 ? "true" : "false",
         static_cast<unsigned>(servo.currentRaw), torqueName(servo.torque),
         static_cast<unsigned long>(ageMs));
  bool hasError = false;
  if (servo.error != 0) {
    append("\"servo_fault_0x%02X\"", static_cast<unsigned>(servo.error));
    hasError = true;
  }
  if (servo.statusError != 0) {
    append("%s\"status_error_0x%02X\"", hasError ? "," : "",
           static_cast<unsigned>(servo.statusError));
    hasError = true;
  }
  if (!servo.online) {
    append("%s\"bus_offline\"", hasError ? "," : "");
    hasError = true;
  } else if (!servo.fresh) {
    append("%s\"telemetry_stale\"", hasError ? "," : "");
    hasError = true;
  }
  if (servo.torque == TorqueState::UNKNOWN) {
    append("%s\"torque_unknown\"", hasError ? "," : "");
    hasError = true;
  }
  if (!servo.operatingModeKnown) {
    append("%s\"mode_unknown\"", hasError ? "," : "");
    hasError = true;
  } else if (servo.operatingMode != 0) {
    append("%s\"mode_not_position\"", hasError ? "," : "");
  }
  append("],\"statusError\":%u,\"online\":%s,\"fresh\":%s,"
         "\"operatingMode\":",
         static_cast<unsigned>(servo.statusError),
         servo.online ? "true" : "false",
         servo.fresh ? "true" : "false");
  if (servo.operatingModeKnown) {
    append("%u", static_cast<unsigned>(servo.operatingMode));
  } else {
    append("null");
  }
}

void ArmHatRuntime::sendServoJson(const ServoTelemetry& servo,
                                  uint32_t nowMs) {
  append("{");
  sendServoFields(servo, nowMs);
  append("}");
}

// Deliberately its own response rather than extra STATUS fields: the worst-case
// STATUS frame is size-asserted against the fixed response buffer.
void ArmHatRuntime::sendOdometerJson(const ServoTelemetry& servo,
                                     uint32_t nowMs) {
  const bool usable = servo.odometerTracking && servo.odometerValid;
  append("{\"servoId\":%u,\"tracking\":%s,\"valid\":%s,\"stepMode\":%s,\"revolutions\":%ld,"
         "\"rawPosition\":%u,\"multiTurnPosition\":%ld,\"sampleAgeMs\":%lu,"
         "\"stepOutstanding\":%s,\"countdownObserved\":%s,"
         "\"resyncNeeded\":%s,\"resyncCount\":%lu}",
         static_cast<unsigned>(servo.id),
         servo.odometerTracking ? "true" : "false",
         usable ? "true" : "false",
         "false",
         static_cast<long>(servo.revolutions),
         static_cast<unsigned>(servo.odometerLastRaw),
         static_cast<long>(servo.revolutions * 4096L +
                           static_cast<long>(servo.odometerLastRaw)),
         static_cast<unsigned long>(
             elapsedMs(nowMs, servo.odometerSampledAtMs)),
         "false", "false", "false",
         static_cast<unsigned long>(servo.multiTurnTruth.resyncCount));
}

void ArmHatRuntime::sendStatusJson(uint32_t nowMs) {
  append("{\"controllerId\":\"%s\",\"bootId\":\"boot-%08lx%08lx\","
         "\"firmwareVersion\":\"arm-hat-2.7.2\","
         "\"protocolVersion\":1,\"state\":\"%s\","
         "\"motionState\":\"%s\",\"torqueState\":\"%s\","
         "\"busState\":\"%s\",\"servosState\":\"%s\","
         "\"servoCount\":",
         controllerId_, static_cast<unsigned long>(bootId_ >> 32u),
         static_cast<unsigned long>(bootId_ & 0xFFFFFFFFu), stateName(nowMs),
         motionState(nowMs), aggregateTorqueState(), busStateName(),
         aggregateServosState(nowMs));
  if (!inventoryScanned_ && servoCount_ == 0) {
    append("null");
  } else {
    append("%u", static_cast<unsigned>(servoCount_));
  }
  append(",\"busBaud\":1000000,"
         "\"hardwareEstop\":\"not_detected\",\"stopped\":%s,"
         "\"watchdogFresh\":%s,\"hostAgeMs\":",
         safety_.stopped() ? "true" : "false",
         safety_.heartbeatFresh(nowMs) ? "true" : "false");
  if (safety_.heartbeatSeen()) {
    append("%lu", static_cast<unsigned long>(safety_.hostAgeMs(nowMs)));
  } else {
    append("null");
  }
  append(",\"safetyFault\":%s,\"bootTorqueOffSent\":%s,"
         "\"busJammed\":%s,\"operatorInspectionRequired\":%s,"
         "\"safetyStopReason\":",
         safetyFaultActive() ? "true" : "false",
         bootTorqueOffSent_ ? "true" : "false",
         bus_.jammed() ? "true" : "false",
         operatorClearRequired() ? "true" : "false");
  const char* stopReason = safetyStopReasonName();
  if (stopReason != nullptr) {
    append("\"%s\"", stopReason);
  } else {
    append("null");
  }
  appendBusNoise();
  append(",\"lease\":");
  if (safety_.torqueLeaseActive(nowMs)) {
    append("{\"servoId\":%u,\"remainingMs\":%lu}",
           static_cast<unsigned>(safety_.torqueLeaseId()),
           static_cast<unsigned long>(
               safety_.torqueLeaseRemainingMs(nowMs)));
  } else {
    append("null");
  }
  append(",\"holdSet\":");
  if (safety_.holdSetActive(nowMs)) {
    append("{\"servoIds\":[");
    for (uint8_t index = 0; index < safety_.holdSetCount(); ++index) {
      if (index > 0) {
        append(",");
      }
      append("%u", static_cast<unsigned>(safety_.holdSetServoId(index)));
    }
    append("],\"remainingMs\":%lu}",
           static_cast<unsigned long>(safety_.holdSetRemainingMs(nowMs)));
  } else {
    append("null");
  }
  append(",\"torqueOffPending\":%s,\"torqueOffPendingCount\":%u,"
         "\"torqueOffRetryServoId\":",
         safety_.torqueOffPending() ? "true" : "false",
         static_cast<unsigned>(safety_.torqueOffPendingCount()));
  if (safety_.torqueOffPending()) {
    append("%u", static_cast<unsigned>(safety_.torqueOffServoId()));
  } else {
    append("null");
  }
  append(",\"proposal\":");
  if (proposalFresh(nowMs)) {
    append("{\"proposalId\":\"%s\",\"servoId\":%u,"
           "\"targetRawPosition\":%u,"
           "\"remainingMs\":%lu}",
           proposal_.token, static_cast<unsigned>(proposal_.servoId),
           static_cast<unsigned>(proposal_.target),
           static_cast<unsigned long>(PROPOSAL_TTL_MS -
                                      elapsedMs(nowMs,
                                                proposal_.createdAtMs)));
  } else {
    append("null");
  }
  append(",\"servos\":[");
  bool wroteServo = false;
  for (uint8_t index = 0; index < servoCount_; ++index) {
    if (!telemetryContractValid(servos_[index], nowMs)) {
      continue;
    }
    if (wroteServo) {
      append(",");
    }
    sendServoJson(servos_[index], nowMs);
    wroteServo = true;
  }
  append("]}");
}

void ArmHatRuntime::handleHello(uint32_t sequence,
                                const ParsedCommand& command) {
  if (!requireNoArguments(command)) {
    sendError(sequence, "BAD_ARGS");
    return;
  }
  const uint32_t nowMs = millis();
  startResponse(sequence, true, nullptr);
  append("{\"controllerId\":\"%s\",\"bootId\":\"boot-%08lx%08lx\","
         "\"firmwareVersion\":\"arm-hat-2.7.2\","
         "\"protocolVersion\":1,\"state\":\"%s\","
         "\"motionState\":\"%s\",\"torqueState\":\"unknown\","
         "\"busState\":\"unknown\",\"bootTorqueOffSent\":%s,"
         "\"servosState\":\"unknown\",\"servoCount\":null,"
         "\"requestMaxBytes\":%u,"
         "\"responseMaxBytes\":%u,\"host\":{\"uart\":\"Serial0\","
         "\"baud\":115200},\"servoBus\":{\"uart\":\"Serial1\","
         "\"baud\":1000000,\"rx\":18,\"tx\":19},"
         "\"watchdogMs\":%lu,\"leaseMs\":{\"min\":%lu,"
         "\"max\":%lu},\"servoModel\":\"user_confirmation_required\","
         "\"registerProfile\":\"ST3215_candidate\","
         "\"safetyFault\":%s,\"operatorInspectionRequired\":%s,"
         "\"safetyStopReason\":",
         controllerId_, static_cast<unsigned long>(bootId_ >> 32u),
         static_cast<unsigned long>(bootId_ & 0xFFFFFFFFu), stateName(nowMs),
         motionState(nowMs), bootTorqueOffSent_ ? "true" : "false",
         static_cast<unsigned>(MAX_REQUEST_BYTES),
         static_cast<unsigned>(MAX_RESPONSE_BYTES),
         static_cast<unsigned long>(WATCHDOG_TIMEOUT_MS),
         static_cast<unsigned long>(TORQUE_LEASE_MIN_MS),
         static_cast<unsigned long>(TORQUE_LEASE_MAX_MS),
         safetyFaultActive() ? "true" : "false",
         operatorClearRequired() ? "true" : "false");
  const char* stopReason = safetyStopReasonName();
  if (stopReason != nullptr) {
    append("\"%s\"", stopReason);
  } else {
    append("null");
  }
  append(",\"capabilities\":[\"scan\",\"single_servo_id\","
         "\"set_position_mode\",\"capture\",\"hold_set\",\"torque_lease\",\"bounded_nudge\","
         "\"telemetry\",\"stop\",\"multi_turn_sense\",\"register_access\","
         "\"runtime_config\",\"direct_move\",\"servo_family\","
         "\"multi_turn_absolute_v1\",\"move_set_v1\","
         "\"follow_set_feedback_v1\",\"follow_feedback_v1\"]}");
  sendResponse();
}

void ArmHatRuntime::handleHeartbeat(uint32_t sequence,
                                    const ParsedCommand& command) {
  if (!requireNoArguments(command)) {
    sendError(sequence, "BAD_ARGS");
    return;
  }
  const uint32_t nowMs = millis();
  safety_.heartbeat(nowMs);
  startResponse(sequence, true, nullptr);
  append("{\"watchdogFresh\":true,\"hostAgeMs\":0,\"stopped\":%s,"
         "\"motionState\":\"%s\",\"torqueState\":\"%s\","
         "\"busState\":\"%s\",\"servosState\":\"%s\","
         "\"safetyFault\":%s,\"operatorInspectionRequired\":%s,"
         "\"safetyStopReason\":",
         safety_.stopped() ? "true" : "false", motionState(nowMs),
         aggregateTorqueState(), busStateName(), aggregateServosState(nowMs),
         safetyFaultActive() ? "true" : "false",
         operatorClearRequired() ? "true" : "false");
  const char* stopReason = safetyStopReasonName();
  if (stopReason != nullptr) {
    append("\"%s\"", stopReason);
  } else {
    append("null");
  }
  append(",\"servoCount\":");
  if (!inventoryScanned_ && servoCount_ == 0) {
    append("null}");
  } else {
    append("%u}", static_cast<unsigned>(servoCount_));
  }
  sendResponse();
}

void ArmHatRuntime::handleStatus(uint32_t sequence,
                                 const ParsedCommand& command) {
  if (!requireNoArguments(command)) {
    sendError(sequence, "BAD_ARGS");
    return;
  }
  if (servoCount_ > 0) {
    busDegraded_ = false;
  }
  bool allHealthyOff = servoCount_ > 0;
  for (uint8_t index = 0; index < servoCount_; ++index) {
    const bool refreshed = refreshServo(servos_[index].id);
    const ServoTelemetry& servo = servos_[index];
    allHealthyOff = allHealthyOff && refreshed && servo.online && servo.fresh &&
                    servo.torque == TorqueState::OFF &&
                    servo.operatingModeKnown && servo.operatingMode == 0 &&
                    servo.error == 0 && servo.statusError == 0 &&
                    telemetryContractValid(servo, millis());
  }
  if (!safety_.stopped() && allHealthyOff &&
      !safety_.torqueOffPending() && !busDegraded_) {
    // Automatic faults are recovery states, not latches. Fresh telemetry plus
    // addressed torque-off proof is sufficient; RESET is reserved for a
    // deliberate operator/hardware STOP.
    safetyFault_ = false;
    operatorInspectionRequired_ = false;
  }
  const uint32_t nowMs = millis();
  startResponse(sequence, true, nullptr);
  sendStatusJson(nowMs);
  sendResponse();
}

void ArmHatRuntime::handleScan(uint32_t sequence,
                               const ParsedCommand& command) {
  uint32_t minimum = 0;
  uint32_t maximum = 0;
  if (command.argumentCount != 2 ||
      !parseUInt32(command.arguments[0], minimum) ||
      !parseUInt32(command.arguments[1], maximum) ||
      !validScanRange(minimum, maximum)) {
    sendError(sequence, "OUT_OF_RANGE");
    return;
  }
  const bool recoveryWasPending = safetyFault_ && !safety_.stopped();
  const bool fullBusRange = minimum == 0 && maximum == 253;

  invalidateProposal();
  // A scan is the one operation whose entire job is to describe the bus as it
  // is now. Carrying a jam in from an earlier command would answer a question
  // nobody asked, so the verdict below is measured fresh every time.
  bus_.clearJam();
  // Proceeds even when torque-off will not confirm. A scan only PINGS -- it
  // commands no motion -- and refusing it made the one operation that can
  // diagnose a broken bus unavailable on a broken bus. The verdict is reported
  // instead, so the caller still learns torque is unproven.
  const bool torqueConfirmed = resolvePendingTorqueOff();
  const bool torqueOffSent = bus_.broadcastTorqueOff();
  if (!torqueOffSent) {
    safetyFault_ = true;
  }

  uint8_t discovered[MAX_TRACKED_SERVOS] = {};
  uint16_t discoveredCount = 0;
  // Garbage back from a ping means something answered but the frame did not
  // survive: two servos sharing that id, talking over each other. Silence is a
  // TIMEOUT and means nothing is there. Reporting the difference is the whole
  // point -- an id clash used to present as an unexplained dead controller.
  bool collisionSuspected = false;
  uint8_t collisionId = 0;
  for (uint16_t candidate = static_cast<uint16_t>(minimum);
       candidate <= static_cast<uint16_t>(maximum); ++candidate) {
    BusResult outcome = BusResult::TIMEOUT;
    bool answered = false;
    for (uint8_t attempt = 0; attempt < SCAN_PING_ATTEMPTS; ++attempt) {
      answered = bus_.ping(static_cast<uint8_t>(candidate),
                           SCAN_PING_TIMEOUT_US, &outcome);
      if (answered || outcome != BusResult::TIMEOUT) {
        break;
      }
    }
    if (answered) {
      busResponseSeen_ = true;
      if (!safety_.queueTorqueOffObligation(static_cast<uint8_t>(candidate))) {
        safety_.revokeAuthority();
        safetyFault_ = true;
      }
      if (discoveredCount < MAX_TRACKED_SERVOS) {
        discovered[discoveredCount] = static_cast<uint8_t>(candidate);
      }
      ++discoveredCount;
    } else if (outcome == BusResult::CORRUPT && !collisionSuspected) {
      collisionSuspected = true;
      collisionId = static_cast<uint8_t>(candidate);
    }
  }
  if (bus_.jammed() && !collisionSuspected) {
    collisionSuspected = true;
  }
  if (discoveredCount > MAX_TRACKED_SERVOS) {
    safety_.revokeAuthority();
    safetyFault_ = true;
    startResponse(sequence, false, "CAPACITY");
    append("{\"found\":%u,\"capacity\":%u,"
           "\"stopped\":false,\"torqueOffPendingCount\":%u}",
           static_cast<unsigned>(discoveredCount),
           static_cast<unsigned>(MAX_TRACKED_SERVOS),
           static_cast<unsigned>(safety_.torqueOffPendingCount()));
    sendResponse();
    return;
  }

  if (!reconcileCompletedScan(
          static_cast<uint8_t>(minimum), static_cast<uint8_t>(maximum),
          discovered, static_cast<uint8_t>(discoveredCount))) {
    const bool addressedOff = resolvePendingTorqueOff();
    if (!addressedOff || safety_.torqueOffPending()) {
      safety_.revokeAuthority();
      safetyFault_ = true;
    }
    startResponse(sequence, false, "CAPACITY");
    append("{\"found\":%u,\"capacity\":%u,"
           "\"preservedOutsideCompletedRange\":true,"
           "\"torqueOffPendingCount\":%u}",
           static_cast<unsigned>(discoveredCount),
           static_cast<unsigned>(MAX_TRACKED_SERVOS),
           static_cast<unsigned>(safety_.torqueOffPendingCount()));
    sendResponse();
    return;
  }

  const bool addressedOff = resolvePendingTorqueOff();
  if (discoveredCount > 0) {
    busDegraded_ = false;
  }
  for (uint16_t index = 0; index < discoveredCount; ++index) {
    refreshServo(discovered[index]);
  }
  bool allDiscoveredOff = true;
  for (uint16_t index = 0; index < discoveredCount; ++index) {
    ServoTelemetry* servo = findServo(discovered[index]);
    allDiscoveredOff = allDiscoveredOff && servo != nullptr &&
                       servo->torque == TorqueState::OFF;
  }
  // The census is already in hand by this point. Refusing here threw away the
  // answer the caller asked for because of a fault it could not act on without
  // that answer -- the same "gated behind the fault it diagnoses" trap as the
  // entry check. A partial census cannot prove that a pre-existing automatic
  // recovery fault is gone because it has not refreshed every possible servo.
  // Only a complete, collision-free census with addressed torque-off proof may
  // clear that fault; every scan can still introduce a new one.
  const bool scanFault =
      !addressedOff || safety_.torqueOffPending() || !allDiscoveredOff ||
      !torqueConfirmed || collisionSuspected || bus_.jammed();
  const bool completeScanProvesRecovery =
      fullBusRange && discoveredCount > 0 && !scanFault;
  safetyFault_ = scanFault ||
                 (recoveryWasPending && !completeScanProvesRecovery);
  startResponse(sequence, true, nullptr);
  append("{\"foundIds\":[");
  for (uint16_t index = 0; index < discoveredCount; ++index) {
    if (index > 0) {
      append(",");
    }
    append("%u", static_cast<unsigned>(discovered[index]));
  }
  append("],\"completeRange\":{\"minId\":%lu,\"maxId\":%lu},"
         "\"collisionSuspected\":%s,\"collisionId\":%u,\"busJammed\":%s,"
         "\"torqueUnconfirmed\":%s,\"torqueState\":\"%s\","
         "\"torqueOffBroadcastSent\":%s,\"torqueOffPendingCount\":%u,"
         "\"stopped\":%s",
         static_cast<unsigned long>(minimum),
         static_cast<unsigned long>(maximum),
         collisionSuspected ? "true" : "false",
         static_cast<unsigned>(collisionId),
         bus_.jammed() ? "true" : "false",
         safetyFaultActive() ? "true" : "false",
         safetyFaultActive() ? "unknown" : "off",
         torqueOffSent ? "true" : "false",
         static_cast<unsigned>(safety_.torqueOffPendingCount()),
         safety_.stopped() ? "true" : "false");
  appendBusNoise();
  append("}");
  sendResponse();
}

void ArmHatRuntime::handleAssignId(uint32_t sequence,
                                   const ParsedCommand& command) {
  uint8_t oldId = 0;
  uint8_t newId = 0;
  if (command.argumentCount != 3 ||
      !parseServoId(command.arguments[0], oldId) ||
      !parseServoId(command.arguments[1], newId) || oldId == newId ||
      std::strcmp(command.arguments[2], "SINGLE_SERVO") != 0) {
    sendError(sequence, "BAD_ARGS");
    return;
  }
  const uint32_t nowMs = millis();
  if (safety_.stopped()) {
    sendError(sequence, "STOPPED");
    return;
  }
  if (!safety_.heartbeatFresh(nowMs)) {
    sendError(sequence, "HEARTBEAT_STALE");
    return;
  }

  invalidateProposal();
  // Renaming writes one EEPROM byte and moves nothing, so an unconfirmed
  // torque-off is not a reason to refuse it. Refusing meant the only tool that
  // resolves an id collision was unusable during an id collision -- the
  // recovery path gated behind the fault it recovers from.
  (void)resolvePendingTorqueOff();
  (void)bus_.broadcastTorqueOff();

  uint16_t count = 0;
  bool oldSeen = false;
  bool newSeen = false;
  for (uint16_t candidate = 0; candidate <= 253; ++candidate) {
    if (bus_.ping(static_cast<uint8_t>(candidate), 800)) {
      busResponseSeen_ = true;
      if (!safety_.queueTorqueOffObligation(static_cast<uint8_t>(candidate))) {
        safety_.revokeAuthority();
        safetyFault_ = true;
      }
      ++count;
      oldSeen = oldSeen || candidate == oldId;
      newSeen = newSeen || candidate == newId;
    }
  }
  const bool scannedServosOff = resolvePendingTorqueOff();
  if (count != 1 || !oldSeen || newSeen) {
    if (!scannedServosOff || safety_.torqueOffPending()) {
      safety_.revokeAuthority();
      safetyFault_ = true;
    }
    startResponse(sequence, false, "NOT_SINGLE_SERVO");
    append("{\"found\":%u,\"oldSeen\":%s,\"newSeen\":%s,"
           "\"torqueOffPendingCount\":%u}",
           static_cast<unsigned>(count), oldSeen ? "true" : "false",
           newSeen ? "true" : "false",
           static_cast<unsigned>(safety_.torqueOffPendingCount()));
    sendResponse();
    return;
  }
  if (!scannedServosOff || safety_.torqueOffPending()) {
    safety_.revokeAuthority();
    safetyFault_ = true;
    sendError(sequence, "TORQUE_UNCONFIRMED",
              "{\"phase\":\"assign_id_scan_off\","
              "\"stopped\":false,\"torqueState\":\"unknown\"}");
    return;
  }

  bool unlocked = false;
  uint8_t servoError = 0;
  const bool unlockSent = bus_.writeLock(oldId, false);
  const BusResult unlockRead = bus_.readLock(oldId, unlocked, servoError);
  if (!unlockSent || unlockRead != BusResult::OK || unlocked) {
    bus_.writeLock(oldId, true);
    sendError(sequence, "ID_VERIFY_FAILED",
              "{\"phase\":\"unlock\"}");
    return;
  }

  const bool idWriteSent = bus_.writeId(oldId, newId);
  delay(4);
  // The dialect belongs to the servo, so it has to move with the rename. Left
  // behind, the re-lock below addresses the new id under the default STS map
  // and writes register 55 instead of the SCS lock at 48 -- the EEPROM stays
  // open, and the id it just wrote reverts on the next power cut.
  // Both ids keep the dialect until the outcome is known: if the rename did not
  // take, the servo is still answering on the old id and the recovery re-lock
  // below has to address it correctly too.
  const ServoFamily renamedFamily = bus_.familyOf(oldId);
  bus_.setFamily(newId, renamedFamily);
  const bool lockSent = bus_.writeLock(newId, true);
  delay(2);
  bool locked = false;
  const BusResult lockRead = bus_.readLock(newId, locked, servoError);
  const bool newResponds = bus_.ping(newId);
  const bool oldResponds = bus_.ping(oldId);
  const bool verified = idWriteSent && lockSent && newResponds &&
                        !oldResponds && locked && lockRead == BusResult::OK;
  if (!verified) {
    bus_.writeLock(oldId, true);
    bus_.writeLock(newId, true);
    // The rename did not stand, so the dialect stays where the servo is.
    bus_.setFamily(newId, ServoFamily::STS);
    safetyFault_ = true;
    startResponse(sequence, false, "ID_VERIFY_FAILED");
    append("{\"phase\":\"commit\",\"newResponds\":%s,"
           "\"oldResponds\":%s,\"locked\":%s}",
           newResponds ? "true" : "false",
           oldResponds ? "true" : "false", locked ? "true" : "false");
    sendResponse();
    return;
  }

  // Renamed and verified: nothing answers on the old id any more, so its
  // dialect entry would only be a trap for whatever is given that id next.
  bus_.setFamily(oldId, ServoFamily::STS);
  servoCount_ = 0;
  rememberServo(newId);
  inventoryScanned_ = true;
  ServoTelemetry* renamed = findServo(newId);
  if (renamed != nullptr) {
    renamed->family = renamedFamily;
  }
  const bool newIdOff = torqueOffAndConfirm(newId);
  const bool newIdFresh = newIdOff && refreshServo(newId);
  ServoTelemetry* reassigned = findServo(newId);
  if (!newIdFresh || reassigned == nullptr ||
      reassigned->torque != TorqueState::OFF ||
      safety_.torqueOffPending()) {
    safety_.revokeAuthority();
    safetyFault_ = true;
    startResponse(sequence, false, "ID_VERIFY_FAILED");
    append("{\"phase\":\"new_id_torque_off\","
           "\"newResponds\":%s,\"stopped\":false,"
           "\"torqueState\":\"%s\",\"torqueOffPendingCount\":%u}",
           newResponds ? "true" : "false",
           safety_.torqueOffPending() ? "unknown" : "off",
           static_cast<unsigned>(safety_.torqueOffPendingCount()));
    sendResponse();
    return;
  }
  busDegraded_ = false;
  startResponse(sequence, true, nullptr);
  append("{\"oldId\":%u,\"newId\":%u,\"verified\":true,"
         "\"locked\":true}",
         static_cast<unsigned>(oldId), static_cast<unsigned>(newId));
  sendResponse();
}

void ArmHatRuntime::handleSetPositionMode(
    uint32_t sequence, const ParsedCommand& command) {
  uint8_t id = 0;
  if (command.argumentCount != 3 ||
      !parseServoId(command.arguments[0], id) ||
      std::strcmp(command.arguments[1], "SINGLE_SERVO") != 0 ||
      std::strcmp(command.arguments[2], "ST3215") != 0) {
    sendError(sequence, "BAD_ARGS");
    return;
  }
  // Native extended-position configuration is owned by MULTITURN, including
  // its Phase/limit readbacks. Do not bypass that transaction here.
  if (bus_.isMultiTurn(id)) {
    sendError(sequence, "MULTI_TURN_ARMED");
    return;
  }
  const uint32_t nowMs = millis();
  if (safety_.stopped()) {
    sendError(sequence, "STOPPED");
    return;
  }
  if (!safety_.heartbeatFresh(nowMs)) {
    sendError(sequence, "HEARTBEAT_STALE");
    return;
  }

  invalidateProposal();
  if (!resolvePendingTorqueOff()) {
    const bool broadcastSent = bus_.broadcastTorqueOff();
    startResponse(sequence, false, "TORQUE_UNCONFIRMED");
    append("{\"phase\":\"set_mode_cancel_lease\","
           "\"stopped\":false,\"torqueState\":\"unknown\","
           "\"torqueOffBroadcastSent\":%s}",
           broadcastSent ? "true" : "false");
    sendResponse();
    return;
  }
  if (!bus_.broadcastTorqueOff()) {
    safetyFault_ = true;
    sendError(sequence, "BUS_CORRUPT",
              "{\"phase\":\"set_mode_broadcast_torque_off\"}");
    return;
  }

  uint16_t count = 0;
  bool targetSeen = false;
  for (uint16_t candidate = 0; candidate <= 253; ++candidate) {
    if (bus_.ping(static_cast<uint8_t>(candidate), 800)) {
      busResponseSeen_ = true;
      if (!safety_.queueTorqueOffObligation(static_cast<uint8_t>(candidate))) {
        safety_.revokeAuthority();
        safetyFault_ = true;
      }
      ++count;
      targetSeen = targetSeen || candidate == id;
    }
  }
  const bool scannedServosOff = resolvePendingTorqueOff();
  if (count != 1 || !targetSeen) {
    if (!scannedServosOff || safety_.torqueOffPending()) {
      safety_.revokeAuthority();
      safetyFault_ = true;
    }
    startResponse(sequence, false, "NOT_SINGLE_SERVO");
    append("{\"found\":%u,\"targetSeen\":%s,"
           "\"torqueOffPendingCount\":%u}",
           static_cast<unsigned>(count), targetSeen ? "true" : "false",
           static_cast<unsigned>(safety_.torqueOffPendingCount()));
    sendResponse();
    return;
  }
  if (!scannedServosOff || safety_.torqueOffPending()) {
    safety_.revokeAuthority();
    safetyFault_ = true;
    sendError(sequence, "TORQUE_UNCONFIRMED",
              "{\"phase\":\"set_mode_scan_off\","
              "\"stopped\":false,\"torqueState\":\"unknown\"}");
    return;
  }

  servoCount_ = 0;
  rememberServo(id);
  inventoryScanned_ = true;
  if (!torqueOffAndConfirm(id)) {
    safety_.revokeAuthority();
    safetyFault_ = true;
    sendError(sequence, "TORQUE_UNCONFIRMED",
              "{\"phase\":\"set_mode_target_off\","
              "\"stopped\":false,\"torqueState\":\"unknown\"}");
    return;
  }

  uint8_t previousMode = 0;
  uint8_t servoError = 0;
  const BusResult initialModeRead =
      bus_.readOperatingMode(id, previousMode, servoError);
  uint16_t previousMinimum = 0;
  uint16_t previousMaximum = 0;
  const BusResult initialLimitsRead = bus_.readPositionLimits(
      id, previousMinimum, previousMaximum, servoError);
  const bool initialLimitsValid =
      previousMinimum <= ST3215_POSITION_MAX &&
      previousMaximum <= ST3215_POSITION_MAX &&
      previousMinimum < previousMaximum;
  if (initialModeRead != BusResult::OK ||
      previousMode > ST3215_OPERATING_MODE_MAX ||
      initialLimitsRead != BusResult::OK || !initialLimitsValid) {
    bus_.writeLock(id, true);
    safety_.revokeAuthority();
    safetyFault_ = true;
    sendError(sequence, "MODE_VERIFY_FAILED",
              "{\"phase\":\"read_before\"}");
    return;
  }

  bool unlocked = false;
  if (previousMode != 0) {
    const bool unlockSent = bus_.writeLock(id, false);
    const BusResult unlockRead = bus_.readLock(id, unlocked, servoError);
    if (!unlockSent || unlockRead != BusResult::OK || unlocked) {
      bus_.writeLock(id, true);
      safety_.revokeAuthority();
      safetyFault_ = true;
      sendError(sequence, "MODE_VERIFY_FAILED",
                "{\"phase\":\"unlock\"}");
      return;
    }
    // Mode 1 is the ST3215 constant-speed mode. Restoring it to absolute
    // position mode must not silently rewrite its configured limits or offset.
    if (!bus_.writePositionMode(id)) {
      bus_.writeLock(id, true);
      safety_.revokeAuthority();
      safetyFault_ = true;
      sendError(sequence, "MODE_VERIFY_FAILED",
                "{\"phase\":\"write\"}");
      return;
    }
    delay(4);
  }

  const bool lockSent = bus_.writeLock(id, true);
  delay(2);
  bool locked = false;
  const BusResult lockRead = bus_.readLock(id, locked, servoError);
  uint8_t verifiedMode = 0xFF;
  const BusResult modeRead =
      bus_.readOperatingMode(id, verifiedMode, servoError);
  uint16_t verifiedMinimum = 0;
  uint16_t verifiedMaximum = 0;
  const BusResult limitsRead = bus_.readPositionLimits(
      id, verifiedMinimum, verifiedMaximum, servoError);
  const bool targetOff = torqueOffAndConfirm(id);
  const bool refreshed = targetOff && refreshServo(id);
  ServoTelemetry* verifiedServo = findServo(id);
  const bool verified =
      lockSent && locked &&
      lockRead == BusResult::OK && modeRead == BusResult::OK &&
      limitsRead == BusResult::OK && verifiedMode == 0 &&
      verifiedMinimum == previousMinimum &&
      verifiedMaximum == previousMaximum &&
      verifiedMinimum < verifiedMaximum && refreshed &&
      verifiedServo != nullptr && verifiedServo->statusError == 0 &&
      verifiedServo->operatingModeKnown &&
      verifiedServo->operatingMode == 0 &&
      verifiedServo->torque == TorqueState::OFF &&
      !safety_.torqueOffPending();
  if (!verified) {
    bus_.writeLock(id, true);
    const bool off = torqueOffAndConfirm(id);
    safety_.revokeAuthority();
    safetyFault_ = true;
    startResponse(sequence, false, "MODE_VERIFY_FAILED");
    append("{\"phase\":\"verify\",\"operatingMode\":%u,"
           "\"locked\":%s,\"torqueState\":\"%s\"}",
           static_cast<unsigned>(verifiedMode), locked ? "true" : "false",
           off ? "off" : "unknown");
    sendResponse();
    return;
  }

  busDegraded_ = false;
  startResponse(sequence, true, nullptr);
  append("{\"servoId\":%u,\"previousOperatingMode\":%u,"
         "\"operatingMode\":0,\"verified\":true,\"locked\":true,"
         "\"torqueState\":\"off\",\"minimumPosition\":%u,"
         "\"maximumPosition\":%u}",
         static_cast<unsigned>(id), static_cast<unsigned>(previousMode),
         static_cast<unsigned>(verifiedMinimum),
         static_cast<unsigned>(verifiedMaximum));
  sendResponse();
}

void ArmHatRuntime::handleCapture(uint32_t sequence,
                                  const ParsedCommand& command) {
  uint8_t id = 0;
  if (command.argumentCount != 1 ||
      !parseServoId(command.arguments[0], id)) {
    sendError(sequence, "BAD_ARGS");
    return;
  }
  const bool recoveryWasPending = safetyFault_ && !safety_.stopped();
  invalidateProposal();
  rememberServo(id);
  const bool targetOffConfirmed = resolvePendingTorqueOffFor(id);
  const bool targetStillPending = safety_.torqueOffPendingFor(id);
  if (!targetOffConfirmed || targetStillPending) {
    safety_.revokeAuthority();
    clearHoldGoals();
    safetyFault_ = true;
    startResponse(sequence, false, "TORQUE_UNCONFIRMED");
    append("{\"attempted\":true,\"stopped\":false,"
           "\"torqueState\":\"unknown\","
           "\"torqueOffPendingCount\":%u}",
           static_cast<unsigned>(safety_.torqueOffPendingCount()));
    sendResponse();
    return;
  }

  uint16_t positions[5] = {};
  for (uint8_t index = 0; index < 5; ++index) {
    const bool refreshed = refreshServo(id);
    ServoTelemetry* sample = findServo(id);
    if (!refreshed || sample == nullptr ||
        sample->torque != TorqueState::OFF) {
      const bool off = torqueOffAndConfirm(id);
      if (!off || safety_.torqueOffPendingFor(id)) {
        safety_.revokeAuthority();
        clearHoldGoals();
        safetyFault_ = true;
      }
      sendError(sequence, off ? "SERVO_ERROR" : "TORQUE_UNCONFIRMED",
                off ? "{\"phase\":\"capture_torque_changed\","
                      "\"torqueState\":\"off\"}"
                    : "{\"phase\":\"capture_torque_changed\","
                      "\"stopped\":false,\"torqueState\":\"unknown\"}");
      return;
    }
    positions[index] = sample->rawPosition;
    if (index + 1 < 5) {
      delay(4);
    }
  }
  uint16_t positionMedian = 0;
  uint16_t span = 0;
  if (!analyzePositionSamples4096(positions, 5, positionMedian, span)) {
    sendError(sequence, "SERVO_ERROR",
              "{\"phase\":\"capture_samples\"}");
    return;
  }
  if (span > 8) {
    startResponse(sequence, false, "UNSTABLE");
    append("{\"id\":%u,\"samples\":5,\"positionSpan\":%u}",
           static_cast<unsigned>(id), static_cast<unsigned>(span));
    sendResponse();
    return;
  }

  const uint32_t capturedAtMs = millis();
  ServoTelemetry* captured = findServo(id);
  if (captured == nullptr ||
      !telemetryContractValid(*captured, capturedAtMs)) {
    sendError(sequence, "SERVO_ERROR",
              "{\"phase\":\"capture_contract\"}");
    return;
  }
  captured->rawPosition = positionMedian;
  // CAPTURE proves only this servo. It must not erase a recovery fault that
  // still needs a complete STATUS or equivalent all-bus torque-off proof.
  safetyFault_ = recoveryWasPending;
  ++evidenceCounter_;
  // Reconcile the stable median against the last odometer sample so a capture
  // taken exactly on the 4095/0 boundary cannot be off by a whole revolution.
  const bool odometerUsable =
      captured->odometerTracking && captured->odometerValid;
  int32_t capturedRevolutions = captured->revolutions;
  if (odometerUsable) {
    const int32_t boundaryDelta = static_cast<int32_t>(positionMedian) -
                                  static_cast<int32_t>(captured->odometerLastRaw);
    if (boundaryDelta > ODOMETER_WRAP_THRESHOLD_TICKS) {
      --capturedRevolutions;
    } else if (boundaryDelta < -ODOMETER_WRAP_THRESHOLD_TICKS) {
      ++capturedRevolutions;
    }
  }
  startResponse(sequence, true, nullptr);
  append("{");
  sendServoFields(*captured, capturedAtMs);
  append(",\"sampleCount\":5,\"variationTicks\":%u,"
         "\"positionMedian\":%u,\"odometerValid\":%s,\"revolutions\":%ld,"
         "\"multiTurnPosition\":%ld,"
         "\"evidenceId\":\"obs_%08lx%08lx_%lu\"}",
         static_cast<unsigned>(span), static_cast<unsigned>(positionMedian),
         odometerUsable ? "true" : "false",
         static_cast<long>(odometerUsable ? capturedRevolutions : 0),
         static_cast<long>(capturedRevolutions * 4096L +
                           static_cast<long>(positionMedian)),
         static_cast<unsigned long>(bootId_ >> 32u),
         static_cast<unsigned long>(bootId_ & 0xFFFFFFFFu),
         static_cast<unsigned long>(evidenceCounter_));
  sendResponse();
}

void ArmHatRuntime::handleHoldSet(uint32_t sequence,
                                  const ParsedCommand& command) {
  if (command.argumentCount < 1 ||
      command.argumentCount > MAX_HOLD_SERVOS + 1) {
    sendError(sequence, "BAD_ARGS");
    return;
  }
  uint32_t leaseMs = 0;
  if (!parseUInt32(command.arguments[0], leaseMs) ||
      !validTorqueLeaseMs(leaseMs)) {
    sendError(sequence, "OUT_OF_RANGE");
    return;
  }
  const uint8_t desiredCount = command.argumentCount - 1;
  uint8_t desiredIds[MAX_HOLD_SERVOS] = {};
  for (uint8_t index = 0; index < desiredCount; ++index) {
    if (!parseServoId(command.arguments[index + 1], desiredIds[index])) {
      sendError(sequence, "BAD_ARGS");
      return;
    }
    for (uint8_t prior = 0; prior < index; ++prior) {
      if (desiredIds[prior] == desiredIds[index]) {
        sendError(sequence, "BAD_ARGS");
        return;
      }
    }
  }

  const uint32_t nowMs = millis();
  // HOLD_SET is itself the supervised host renewal. A latched explicit STOP or
  // pending automatic recovery is rejected before dispatch; while healthy,
  // one wire round-trip renews both host and hold leases.
  safety_.heartbeat(nowMs);
  if (safety_.stopped()) {
    sendError(sequence, "STOPPED");
    return;
  }
  if (safety_.torqueLeaseActive(nowMs)) {
    sendError(sequence, "LEASE_ACTIVE");
    return;
  }

  invalidateProposal();
  HoldGoal nextGoals[MAX_HOLD_SERVOS] = {};
  uint8_t addedIds[MAX_HOLD_SERVOS] = {};
  uint8_t addedCount = 0;

  // Read and program only newly added IDs. Existing holds retain their exact
  // goal and torque state; an unchanged HOLD_SET is therefore a pure renewal.
  for (uint8_t index = 0; index < desiredCount; ++index) {
    const uint8_t id = desiredIds[index];
    const HoldGoal* existing = findHoldGoal(id);
    if (existing != nullptr && safety_.holdAuthorizedFor(id, nowMs)) {
      nextGoals[index] = *existing;
      continue;
    }

    if (!refreshServo(id)) {
      sendError(sequence, "BUS_TIMEOUT",
                "{\"phase\":\"hold_capture\"}");
      return;
    }
    ServoTelemetry* servo = findServo(id);
    if (servo == nullptr || servo->torque != TorqueState::OFF) {
      sendError(sequence, "TORQUE_UNCONFIRMED",
                "{\"phase\":\"hold_capture\"}");
      return;
    }
    // Every positional joint, including a native extended-position Base, must
    // be in mode 0. Wheel/step modes cannot hold an absolute coordinate.
    const uint8_t expectedMode = 0;
    if (!servo->operatingModeKnown || servo->operatingMode != expectedMode) {
      sendError(sequence, "MODE_NOT_POSITION",
                "{\"phase\":\"hold_capture\",\"torqueState\":\"off\"}");
      return;
    }
    if (servo->error != 0 || servo->statusError != 0 ||
        !telemetryContractValid(*servo, millis())) {
      sendError(sequence, "SERVO_ERROR",
                "{\"phase\":\"hold_capture\",\"torqueState\":\"off\"}");
      return;
    }
    // In the frame the servo's own goal register uses. For a multi-turn joint
    // that is the wrap-counted position: writing the wrapping one names an
    // absolute position a whole turn away, so taking torque used to command the
    // joint to unwind a full revolution and latch STOP when it would not settle.
    int32_t holdPosition = 0;
    if (!multiTurnPositionOf(*servo, holdPosition)) {
      sendError(sequence, "SERVO_ERROR",
                "{\"phase\":\"hold_capture_odometer\",\"torqueState\":\"off\"}");
      return;
    }
    if (!bus_.writePosition(id, holdPosition, 1, 1) ||
        !bus_.verifyPositionCommand(id, holdPosition, 1, 1)) {
      sendError(sequence, "SERVO_ERROR",
                "{\"phase\":\"hold_goal_verify\",\"torqueState\":\"off\"}");
      return;
    }
    if (!safety_.queueTorqueOffObligation(id)) {
      safety_.revokeAuthority();
      clearHoldGoals();
      safetyFault_ = true;
      sendError(sequence, "CAPACITY",
                "{\"phase\":\"hold_obligation\",\"stopped\":false}");
      return;
    }
    nextGoals[index].servoId = id;
    nextGoals[index].holdPosition = holdPosition;
    addedIds[addedCount++] = id;
  }

  uint8_t enabledAddedCount = 0;
  for (uint8_t index = 0; index < addedCount; ++index) {
    const uint8_t id = addedIds[index];
    bool enabled = false;
    uint8_t servoError = 0;
    const bool sent = bus_.setTorque(id, true);
    const BusResult read = bus_.readTorque(id, enabled, servoError);
    if (!sent || read != BusResult::OK || !enabled) {
      bool additionsOff = true;
      for (uint8_t rollback = 0; rollback <= enabledAddedCount; ++rollback) {
        additionsOff = torqueOffAndConfirm(addedIds[rollback]) && additionsOff;
      }
      if (!additionsOff || safety_.torqueOffPendingCount() > holdGoalCount_) {
        safety_.revokeAuthority();
        clearHoldGoals();
        safetyFault_ = true;
      }
      sendError(sequence,
                additionsOff ? "SERVO_ERROR" : "TORQUE_UNCONFIRMED",
                additionsOff
                    ? "{\"phase\":\"hold_enable\"}"
                    : "{\"phase\":\"hold_enable\",\"stopped\":false}");
      return;
    }
    ++enabledAddedCount;
  }

  if (!safety_.setHoldSet(desiredIds, desiredCount, leaseMs, millis())) {
    bool additionsOff = true;
    for (uint8_t index = 0; index < addedCount; ++index) {
      additionsOff = torqueOffAndConfirm(addedIds[index]) && additionsOff;
    }
    if (!additionsOff) {
      safety_.revokeAuthority();
      clearHoldGoals();
      safetyFault_ = true;
    }
    sendError(sequence, "TORQUE_UNCONFIRMED",
              additionsOff
                  ? "{\"phase\":\"hold_authority\"}"
                  : "{\"phase\":\"hold_authority\",\"stopped\":false}");
    return;
  }

  HoldGoal previousGoals[MAX_HOLD_SERVOS] = {};
  const uint8_t previousCount = holdGoalCount_;
  for (uint8_t index = 0; index < previousCount; ++index) {
    previousGoals[index] = holdGoals_[index];
  }
  clearHoldGoals();
  for (uint8_t index = 0; index < desiredCount; ++index) {
    holdGoals_[holdGoalCount_++] = nextGoals[index];
  }

  bool removalsOff = true;
  for (uint8_t index = 0; index < previousCount; ++index) {
    bool stillDesired = false;
    for (uint8_t desired = 0; desired < desiredCount; ++desired) {
      stillDesired = stillDesired ||
                     desiredIds[desired] == previousGoals[index].servoId;
    }
    if (!stillDesired) {
      removalsOff = torqueOffAndConfirm(previousGoals[index].servoId) &&
                    removalsOff;
    }
  }

  bool desiredHealthy = true;
  for (uint8_t index = 0; index < desiredCount; ++index) {
    const uint8_t id = desiredIds[index];
    const bool refreshed = refreshServo(id);
    ServoTelemetry* servo = findServo(id);
    const uint8_t expectedMode = 0;
    desiredHealthy =
        desiredHealthy && refreshed && servo != nullptr &&
        servo->torque == TorqueState::ON && servo->operatingModeKnown &&
        servo->operatingMode == expectedMode && servo->error == 0 &&
        servo->statusError == 0 && telemetryContractValid(*servo, millis());
  }
  if (!removalsOff || !desiredHealthy) {
    safety_.revokeAuthority();
    clearHoldGoals();
    const bool targetedOff = resolvePendingTorqueOff();
    const bool broadcastSent = bus_.broadcastTorqueOff();
    const bool trackedOff = servoCount_ > 0 && confirmAllTrackedTorqueOff();
    (void)targetedOff;
    (void)broadcastSent;
    (void)trackedOff;
    safetyFault_ = true;
    sendError(sequence, "TORQUE_UNCONFIRMED",
              "{\"phase\":\"hold_verify\",\"stopped\":false}");
    return;
  }

  leaseSupervisionSeen_ = false;
  safetyFault_ = false;
  startResponse(sequence, true, nullptr);
  append("{\"servoIds\":[");
  for (uint8_t index = 0; index < desiredCount; ++index) {
    if (index > 0) {
      append(",");
    }
    append("%u", static_cast<unsigned>(desiredIds[index]));
  }
  append("],\"leaseMs\":%lu,\"holds\":[",
         static_cast<unsigned long>(leaseMs));
  for (uint8_t index = 0; index < holdGoalCount_; ++index) {
    if (index > 0) {
      append(",");
    }
    append("{\"servoId\":%u,\"holdRawPosition\":%ld}",
           static_cast<unsigned>(holdGoals_[index].servoId),
           static_cast<long>(holdGoals_[index].holdPosition));
  }
  append("],\"confirmed\":true}");
  sendResponse();
}

void ArmHatRuntime::handleTorqueLease(uint32_t sequence,
                                      const ParsedCommand& command) {
  uint8_t id = 0;
  uint32_t leaseMs = 0;
  if (command.argumentCount != 2 ||
      !parseServoId(command.arguments[0], id) ||
      !parseUInt32(command.arguments[1], leaseMs) ||
      !validTorqueLeaseMs(leaseMs)) {
    sendError(sequence, "OUT_OF_RANGE");
    return;
  }
  const uint32_t nowMs = millis();
  if (safety_.stopped()) {
    sendError(sequence, "STOPPED");
    return;
  }
  if (!safety_.heartbeatFresh(nowMs)) {
    sendError(sequence, "HEARTBEAT_STALE");
    return;
  }
  if (safety_.torqueLeaseActive(nowMs)) {
    sendError(sequence, "LEASE_ACTIVE");
    return;
  }
  if (!resolvePendingTorqueOff()) {
    sendError(sequence, "TORQUE_UNCONFIRMED",
              "{\"phase\":\"prior_lease_off\",\"stopped\":false,"
              "\"torqueState\":\"unknown\"}");
    return;
  }

  invalidateProposal();
  rememberServo(id);
  const bool broadcastSent = bus_.broadcastTorqueOff();
  const bool allKnownOff = confirmAllTrackedTorqueOff();
  const bool targetOffConfirmed = torqueOffAndConfirm(id);
  const bool targetFresh = targetOffConfirmed && refreshServo(id);
  if (!allKnownOff || !targetOffConfirmed || !targetFresh ||
      safety_.torqueOffPending()) {
    safety_.revokeAuthority();
    safetyFault_ = true;
    startResponse(sequence, false, "TORQUE_UNCONFIRMED");
    append("{\"phase\":\"preflight_off\",\"stopped\":false,"
           "\"torqueState\":\"unknown\","
           "\"torqueOffBroadcastSent\":%s,"
           "\"torqueOffPendingCount\":%u}",
           broadcastSent ? "true" : "false",
           static_cast<unsigned>(safety_.torqueOffPendingCount()));
    sendResponse();
    return;
  }
  ServoTelemetry* servo = findServo(id);
  const uint16_t holdPosition = servo->rawPosition;
  uint8_t operatingMode = 0;
  uint8_t modeError = 0;
  const BusResult modeResult =
      bus_.readOperatingMode(id, operatingMode, modeError);
  if (modeResult != BusResult::OK || operatingMode != 0 ||
      servo->error != 0 || servo->statusError != 0 ||
      !telemetryContractValid(*servo, millis())) {
    const bool off = torqueOffAndConfirm(id);
    if (!off || safety_.torqueOffPending()) {
      safety_.revokeAuthority();
      safetyFault_ = true;
    }
    const bool modeRejected = modeResult != BusResult::OK || operatingMode != 0;
    sendError(sequence,
              !off ? "TORQUE_UNCONFIRMED"
                   : modeRejected ? "MODE_NOT_POSITION" : "SERVO_ERROR",
              off ? "{\"phase\":\"position_mode_preflight\","
                    "\"torqueState\":\"off\"}"
                  : "{\"phase\":\"position_mode_preflight\","
                    "\"stopped\":false,\"torqueState\":\"unknown\"}");
    return;
  }
  if (!bus_.writePosition(id, holdPosition, 1, 1) ||
      !bus_.verifyPositionCommand(id, holdPosition, 1, 1)) {
    const bool off = torqueOffAndConfirm(id);
    if (!off || safety_.torqueOffPending()) {
      safety_.revokeAuthority();
      safetyFault_ = true;
    }
    sendError(sequence, off ? "SERVO_ERROR" : "TORQUE_UNCONFIRMED",
              off ? "{\"phase\":\"hold_goal_verify\","
                    "\"torqueState\":\"off\"}"
                  : "{\"phase\":\"hold_goal_verify\","
                    "\"stopped\":false,\"torqueState\":\"unknown\"}");
    return;
  }
  bool enabled = false;
  uint8_t servoError = 0;
  if (!safety_.armTorqueOffObligation(id)) {
    safety_.revokeAuthority();
    safetyFault_ = true;
    sendError(sequence, "TORQUE_UNCONFIRMED",
              "{\"phase\":\"lease_obligation\",\"stopped\":false,"
              "\"torqueState\":\"unknown\"}");
    return;
  }
  const bool torqueSent = bus_.setTorque(id, true);
  const BusResult torqueRead = bus_.readTorque(id, enabled, servoError);
  if (!torqueSent || torqueRead != BusResult::OK || !enabled ||
      !safety_.startTorqueLease(id, leaseMs, millis())) {
    const bool off = torqueOffAndConfirm(id);
    if (!off) {
      safety_.revokeAuthority();
    }
    safetyFault_ = !off;
    sendError(sequence, off ? "SERVO_ERROR" : "TORQUE_UNCONFIRMED",
              off ? "{\"phase\":\"lease_enable\","
                    "\"torqueState\":\"off\"}"
                  : "{\"phase\":\"lease_enable\",\"stopped\":false,"
                    "\"torqueState\":\"unknown\"}");
    return;
  }
  leaseSupervisionSeen_ = false;
  servo->torque = TorqueState::ON;
  safetyFault_ = false;
  startResponse(sequence, true, nullptr);
  append("{\"servoId\":%u,\"leaseMs\":%lu,"
         "\"holdRawPosition\":%u,\"torqueState\":\"on\","
         "\"confirmed\":true}",
         static_cast<unsigned>(id), static_cast<unsigned long>(leaseMs),
         static_cast<unsigned>(holdPosition));
  sendResponse();
}

void ArmHatRuntime::handleTorqueOff(uint32_t sequence,
                                    const ParsedCommand& command) {
  if (command.argumentCount != 1) {
    sendError(sequence, "BAD_ARGS");
    return;
  }
  const bool allRequested = std::strcmp(command.arguments[0], "ALL") == 0;
  uint8_t id = 0;
  if (!allRequested && !parseServoId(command.arguments[0], id)) {
    sendError(sequence, "BAD_ARGS");
    return;
  }
  const bool recoveryWasPending = safetyFault_ && !safety_.stopped();
  invalidateProposal();
  if (allRequested) {
    const bool priorLeaseResolved = resolvePendingTorqueOff();
    clearHoldGoals();
    (void)bus_.broadcastTorqueOff();
    const bool confirmed = servoCount_ > 0 && confirmAllTrackedTorqueOff();
    const bool safe = priorLeaseResolved && confirmed &&
                      !safety_.torqueOffPending();
    safetyFault_ = !safe;
    if (!safe) {
      safety_.revokeAuthority();
      startResponse(sequence, false, "TORQUE_UNCONFIRMED");
      append("{\"servoId\":null,\"all\":true,"
             "\"torqueState\":\"%s\",\"stopped\":%s,"
             "\"torqueOffPending\":%s}",
             safety_.torqueOffPending() ? "unknown" : "off",
             safety_.stopped() ? "true" : "false",
             safety_.torqueOffPending() ? "true" : "false");
      sendResponse();
      return;
    }
    startResponse(sequence, true, nullptr);
    safetyFault_ = false;
    append("{\"servoId\":null,\"all\":true,"
           "\"torqueState\":\"off\",\"confirmed\":true}");
    sendResponse();
    return;
  }

  ServoTelemetry* target = rememberServo(id);
  removeHoldGoal(id);
  (void)safety_.removeHoldServo(id);
  const bool confirmed = torqueOffAndConfirm(id);
  const bool fresh = confirmed && target != nullptr && refreshServo(id);
  const bool safe = confirmed && fresh && !safety_.torqueOffPendingFor(id);
  safetyFault_ = recoveryWasPending || operatorInspectionRequired_ || !safe;
  if (!safe) {
    safety_.revokeAuthority();
    clearHoldGoals();
    startResponse(sequence, false, "TORQUE_UNCONFIRMED");
    append("{\"servoId\":%u,\"all\":false,"
           "\"torqueState\":\"%s\",\"stopped\":%s,"
           "\"torqueOffPending\":%s}",
           static_cast<unsigned>(id),
           safety_.torqueOffPendingFor(id) ? "unknown" : "off",
           safety_.stopped() ? "true" : "false",
           safety_.torqueOffPendingFor(id) ? "true" : "false");
    sendResponse();
    return;
  }
  startResponse(sequence, true, nullptr);
  // A targeted off proves only one address and cannot clear a pre-existing
  // all-bus recovery fault.
  safetyFault_ = recoveryWasPending;
  append("{\"servoId\":%u,\"all\":false,"
         "\"torqueState\":\"off\",\"confirmed\":true}",
         static_cast<unsigned>(id));
  sendResponse();
}

void ArmHatRuntime::handlePrepareNudge(uint32_t sequence,
                                       const ParsedCommand& command) {
  uint8_t id = 0;
  int32_t delta = 0;
  uint32_t speed = 0;
  uint32_t acceleration = 0;
  if (command.argumentCount != 4 ||
      !parseServoId(command.arguments[0], id) ||
      !parseInt32(command.arguments[1], delta) ||
      !parseUInt32(command.arguments[2], speed) ||
      !parseUInt32(command.arguments[3], acceleration) ||
      !validNudge(delta, speed, acceleration, policy_.maxDeltaTicks,
                  policy_.maxSpeed, policy_.maxAccel)) {
    sendError(sequence, "OUT_OF_RANGE");
    return;
  }
  if (!validNudgeTiming(delta, speed, policy_.motionBudgetMs)) {
    startResponse(sequence, false, "NUDGE_TIMING_UNSAFE");
    append("{\"predictedMotionMs\":%lu,\"budgetMs\":%lu}",
           static_cast<unsigned long>(predictedNudgeMotionMs(delta, speed)),
           static_cast<unsigned long>(policy_.motionBudgetMs));
    sendResponse();
    return;
  }
  const uint32_t nowMs = millis();
  if (safety_.stopped()) {
    sendError(sequence, "STOPPED");
    return;
  }
  if (!safety_.heartbeatFresh(nowMs)) {
    sendError(sequence, "HEARTBEAT_STALE");
    return;
  }
  if (safety_.torqueLeaseActive(nowMs) &&
      safety_.torqueLeaseId() != id) {
    sendError(sequence, "LEASE_ACTIVE");
    return;
  }
  invalidateProposal();
  rememberServo(id);
  const bool targetOffConfirmed = resolvePendingTorqueOffFor(id);
  const bool targetFresh = targetOffConfirmed && refreshServo(id);
  if (!targetOffConfirmed || !targetFresh ||
      safety_.torqueOffPendingFor(id)) {
    safety_.revokeAuthority();
    clearHoldGoals();
    safetyFault_ = true;
    startResponse(sequence, false, "TORQUE_UNCONFIRMED");
    append("{\"phase\":\"prepare_torque_off\",\"stopped\":false,"
           "\"torqueState\":\"unknown\","
           "\"torqueOffPendingCount\":%u}",
           static_cast<unsigned>(safety_.torqueOffPendingCount()));
    sendResponse();
    return;
  }
  safetyFault_ = false;
  ServoTelemetry* servo = findServo(id);
  if (servo == nullptr || servo->torque != TorqueState::OFF) {
    sendError(sequence, "TORQUE_UNCONFIRMED");
    return;
  }
  if (!servo->operatingModeKnown || servo->operatingMode != 0) {
    sendError(sequence, "MODE_NOT_POSITION",
              "{\"torqueState\":\"off\"}");
    return;
  }
  if (servo->error != 0 || servo->statusError != 0 ||
      !telemetryContractValid(*servo, millis())) {
    sendError(sequence, "SERVO_ERROR",
              "{\"torqueState\":\"off\"}");
    return;
  }
  const int32_t target = static_cast<int32_t>(servo->rawPosition) + delta;
  if (target < 0 || target > ST3215_POSITION_MAX) {
    sendError(sequence, "TARGET_OUT_OF_RANGE");
    return;
  }

  proposal_ = Proposal{};
  proposal_.active = true;
  proposal_.servoId = id;
  proposal_.from = servo->rawPosition;
  proposal_.target = static_cast<uint16_t>(target);
  proposal_.delta = static_cast<int16_t>(delta);
  proposal_.speed = static_cast<uint16_t>(speed);
  proposal_.acceleration = static_cast<uint8_t>(acceleration);
  proposal_.createdAtMs = nowMs;
  ++proposalCounter_;
  std::snprintf(proposal_.token, sizeof(proposal_.token),
                "p%08lx%08lx_%lu",
                static_cast<unsigned long>(bootId_ >> 32u),
                static_cast<unsigned long>(bootId_ & 0xFFFFFFFFu),
                static_cast<unsigned long>(proposalCounter_));
  safetyFault_ = false;
  startResponse(sequence, true, nullptr);
  append("{\"proposalId\":\"%s\",\"servoId\":%u,"
         "\"startRawPosition\":%u,\"targetRawPosition\":%u,"
         "\"deltaTicks\":%d,\"speed\":%u,\"acceleration\":%u,"
         "\"expiresInMs\":%lu}",
         proposal_.token, static_cast<unsigned>(id),
         static_cast<unsigned>(proposal_.from),
         static_cast<unsigned>(proposal_.target),
         static_cast<int>(proposal_.delta),
         static_cast<unsigned>(proposal_.speed),
         static_cast<unsigned>(proposal_.acceleration),
         static_cast<unsigned long>(PROPOSAL_TTL_MS));
  sendResponse();
}

void ArmHatRuntime::handleExecuteNudge(uint32_t sequence,
                                       const ParsedCommand& command) {
  if (command.argumentCount != 1 ||
      !validProposalToken(command.arguments[0])) {
    sendError(sequence, "BAD_ARGS");
    return;
  }
  const uint32_t nowMs = millis();
  if (safety_.stopped()) {
    sendError(sequence, "STOPPED");
    return;
  }
  if (!safety_.heartbeatFresh(nowMs)) {
    sendError(sequence, "HEARTBEAT_STALE");
    return;
  }
  if (!proposal_.active) {
    sendError(sequence, "PROPOSAL_MISSING");
    return;
  }
  if (!proposalFresh(nowMs)) {
    invalidateProposal();
    sendError(sequence, "PROPOSAL_EXPIRED");
    return;
  }
  if (std::strcmp(command.arguments[0], proposal_.token) != 0) {
    sendError(sequence, "PROPOSAL_MISMATCH");
    return;
  }
  const Proposal accepted = proposal_;
  invalidateProposal();

  if (!refreshServo(accepted.servoId)) {
    (void)torqueOffAndConfirm(accepted.servoId);
    safety_.revokeAuthority();
    safetyFault_ = true;
    sendError(sequence, "BUS_TIMEOUT",
              "{\"stopped\":false,\"torqueState\":\"unknown\"}");
    return;
  }
  ServoTelemetry* preflight = findServo(accepted.servoId);
  const int32_t startDrift = static_cast<int32_t>(preflight->rawPosition) -
                             static_cast<int32_t>(accepted.from);
  if (preflight->torque != TorqueState::OFF ||
      !preflight->operatingModeKnown || preflight->operatingMode != 0 ||
      preflight->error != 0 || preflight->statusError != 0 ||
      !telemetryContractValid(*preflight, millis()) || startDrift < -2 ||
      startDrift > 2) {
    const bool forcedOff = torqueOffAndConfirm(accepted.servoId);
    safety_.revokeAuthority();
    safetyFault_ = true;
    if (!forcedOff) {
      sendError(sequence, "TORQUE_UNCONFIRMED",
                "{\"phase\":\"execute_preflight\","
                "\"stopped\":false,\"torqueState\":\"unknown\"}");
      return;
    }
    sendError(sequence,
              !preflight->operatingModeKnown || preflight->operatingMode != 0
                  ? "MODE_NOT_POSITION"
                  : "NUDGE_INCOMPLETE",
              "{\"phase\":\"execute_preflight\",\"stopped\":false,"
              "\"torqueState\":\"off\"}");
    return;
  }

  if (!bus_.writePosition(accepted.servoId, preflight->rawPosition, 1, 1) ||
      !bus_.verifyPositionCommand(accepted.servoId, preflight->rawPosition, 1,
                                  1)) {
    const bool forcedOff = torqueOffAndConfirm(accepted.servoId);
    safety_.revokeAuthority();
    safetyFault_ = true;
    sendError(sequence, forcedOff ? "SERVO_ERROR" : "TORQUE_UNCONFIRMED",
              forcedOff
                  ? "{\"phase\":\"execute_hold_goal\","
                    "\"stopped\":false,\"torqueState\":\"off\"}"
                  : "{\"phase\":\"execute_hold_goal\","
                    "\"stopped\":false,\"torqueState\":\"unknown\"}");
    return;
  }

  bool torqueEnabled = false;
  uint8_t torqueError = 0;
  if (!safety_.armTorqueOffObligation(accepted.servoId)) {
    safety_.revokeAuthority();
    safetyFault_ = true;
    sendError(sequence, "TORQUE_UNCONFIRMED",
              "{\"phase\":\"execute_obligation\",\"stopped\":false,"
              "\"torqueState\":\"unknown\"}");
    return;
  }
  const bool torqueEnableSent = bus_.setTorque(accepted.servoId, true);
  const BusResult torqueRead =
      bus_.readTorque(accepted.servoId, torqueEnabled, torqueError);
  const bool internalLeaseStarted =
      torqueEnableSent &&
      torqueRead == BusResult::OK && torqueEnabled && safety_.startTorqueLease(
                           accepted.servoId, INTERNAL_NUDGE_LEASE_MS, millis());
  if (!internalLeaseStarted) {
    const bool forcedOff = torqueOffAndConfirm(accepted.servoId);
    safety_.revokeAuthority();
    safetyFault_ = true;
    sendError(sequence, "TORQUE_UNCONFIRMED",
              forcedOff
                  ? "{\"phase\":\"execute_internal_lease\","
                    "\"stopped\":false,\"torqueState\":\"off\"}"
                  : "{\"phase\":\"execute_internal_lease\","
                    "\"stopped\":false,\"torqueState\":\"unknown\"}");
    return;
  }

  leaseSupervisionSeen_ = false;
  const bool written = bus_.writePosition(
      accepted.servoId, accepted.target, accepted.speed,
      accepted.acceleration);
  const bool verified =
      written && bus_.verifyPositionCommand(accepted.servoId, accepted.target,
                                            accepted.speed,
                                            accepted.acceleration);
  if (!verified) {
    safety_.revokeAuthority();
    const bool targetedOff = resolvePendingTorqueOff();
    const bool broadcastSent = bus_.broadcastTorqueOff();
    const bool allKnownOff = confirmAllTrackedTorqueOff();
    safetyFault_ = !targetedOff || !broadcastSent || !allKnownOff ||
                   safety_.torqueOffPending();
    sendError(sequence, "SERVO_ERROR",
              safety_.torqueOffPending()
                  ? "{\"phase\":\"nudge_goal_verify\","
                    "\"stopped\":false,\"torqueState\":\"unknown\"}"
                  : "{\"phase\":\"nudge_goal_verify\","
                    "\"stopped\":false,\"torqueState\":\"off\"}");
    return;
  }

  bool authorityMaintained = true;
  bool encoderReached = false;
  const uint32_t executionStartedMs = millis();
  while (elapsedMs(millis(), executionStartedMs) <
         policy_.executionTimeoutMs) {
    const uint32_t observedAtMs = millis();
    if (!safety_.heartbeatFresh(observedAtMs) ||
        !safety_.torqueLeaseActive(observedAtMs) ||
        (holdGoalCount_ > 0 && !safety_.holdSetActive(observedAtMs)) ||
        !superviseActiveLease(observedAtMs)) {
      authorityMaintained = false;
      break;
    }
    if (!refreshServo(accepted.servoId)) {
      authorityMaintained = false;
      break;
    }
    ServoTelemetry* observed = findServo(accepted.servoId);
    if (observed->torque != TorqueState::ON ||
        !observed->operatingModeKnown || observed->operatingMode != 0 ||
        observed->error != 0 || observed->statusError != 0 ||
        !telemetryContractValid(*observed, observedAtMs)) {
      authorityMaintained = false;
      break;
    }
    const int32_t observedDelta =
        static_cast<int32_t>(observed->rawPosition) -
        static_cast<int32_t>(accepted.from);
    if ((accepted.delta > 0 &&
         observedDelta < -static_cast<int32_t>(policy_.positionToleranceTicks)) ||
        (accepted.delta < 0 &&
         observedDelta > static_cast<int32_t>(policy_.positionToleranceTicks))) {
      // A 4095 <-> 0 reading is a boundary violation during motion, never an
      // adjacent modular encoder sample.
      authorityMaintained = false;
      break;
    }
    if (validNudgeCompletion(accepted.from, accepted.target,
                             observed->rawPosition, accepted.delta,
                             policy_.positionToleranceTicks)) {
      encoderReached = true;
      break;
    }
    delay(8);
  }

  const bool offConfirmed = torqueOffAndConfirm(accepted.servoId);
  const bool finalFresh = refreshServo(accepted.servoId);
  ServoTelemetry* finalServo = findServo(accepted.servoId);
  const bool finalEvidence =
      finalFresh && finalServo != nullptr &&
      telemetryContractValid(*finalServo, millis()) &&
      finalServo->torque == TorqueState::OFF &&
      finalServo->operatingModeKnown && finalServo->operatingMode == 0 &&
      finalServo->error == 0 && finalServo->statusError == 0 &&
      validNudgeCompletion(accepted.from, accepted.target,
                           finalServo->rawPosition, accepted.delta,
                           policy_.positionToleranceTicks);
  const int32_t finalPosition =
      finalServo == nullptr ? -1 : static_cast<int32_t>(finalServo->rawPosition);
  const int32_t measuredDelta =
      finalPosition < 0
          ? 0
          : finalPosition - static_cast<int32_t>(accepted.from);
  const int32_t positionError =
      finalPosition < 0
          ? 0
          : finalPosition - static_cast<int32_t>(accepted.target);

  // Losing torque authority or failing to confirm torque-off enters automatic
  // recovery and revokes authority. It does not invent an operator STOP latch.
  // Merely not landing on target is also non-latching once the servo is proven
  // de-energised.
  const bool safetyBreach = !authorityMaintained || !offConfirmed;
  if (safetyBreach || !encoderReached || !finalEvidence) {
    if (safetyBreach) {
      safety_.revokeAuthority();
    }
    const bool targetedOff = resolvePendingTorqueOff();
    const bool broadcastSent = bus_.broadcastTorqueOff();
    const bool allKnownOff = confirmAllTrackedTorqueOff();
    (void)targetedOff;
    (void)broadcastSent;
    (void)allKnownOff;
    safetyFault_ = safetyBreach;
    startResponse(sequence, false, "NUDGE_INCOMPLETE");
    append("{\"proposalId\":\"%s\",\"servoId\":%u,"
           "\"startRawPosition\":%u,\"targetRawPosition\":%u,"
           "\"rawPosition\":%ld,\"measuredDeltaTicks\":%ld,"
           "\"positionErrorTicks\":%ld,\"torqueState\":\"%s\","
           "\"completed\":false,\"stopped\":%s}",
           accepted.token, static_cast<unsigned>(accepted.servoId),
           static_cast<unsigned>(accepted.from),
           static_cast<unsigned>(accepted.target),
           static_cast<long>(finalPosition),
           static_cast<long>(measuredDelta),
           static_cast<long>(positionError),
           offConfirmed ? "off" : "unknown",
           safetyBreach ? "true" : "false");
    sendResponse();
    return;
  }

  safetyFault_ = false;
  ++evidenceCounter_;
  startResponse(sequence, true, nullptr);
  append("{\"proposalId\":\"%s\",\"servoId\":%u,"
         "\"startRawPosition\":%u,\"targetRawPosition\":%u,"
         "\"rawPosition\":%ld,\"measuredDeltaTicks\":%ld,"
         "\"positionErrorTicks\":%ld,\"torqueState\":\"off\","
         "\"completed\":true,"
         "\"evidenceId\":\"obs_%08lx%08lx_%lu\"}",
         accepted.token, static_cast<unsigned>(accepted.servoId),
         static_cast<unsigned>(accepted.from),
         static_cast<unsigned>(accepted.target),
         static_cast<long>(finalPosition), static_cast<long>(measuredDelta),
         static_cast<long>(positionError),
         static_cast<unsigned long>(bootId_ >> 32u),
         static_cast<unsigned long>(bootId_ & 0xFFFFFFFFu),
         static_cast<unsigned long>(evidenceCounter_));
  sendResponse();
}

void ArmHatRuntime::handleStop(uint32_t sequence,
                              const ParsedCommand& command) {
  if (!requireNoArguments(command)) {
    sendError(sequence, "BAD_ARGS");
    return;
  }
  const bool wasStopped = safety_.stopped();
  safety_.latchStop();
  if (!wasStopped && !operatorInspectionRequired_) {
    explicitStopLatched_ = true;
  }
  clearHoldGoals();
  invalidateProposal();
  // STOP must put the fast all-servo halt packet on the wire before spending
  // time on up to MAX_TRACKED_SERVOS addressed proof transactions.
  const bool sent = bus_.broadcastTorqueOff();
  const bool targetedConfirmed = resolvePendingTorqueOff();
  bool confirmed = servoCount_ > 0 && confirmAllTrackedTorqueOff();
  const bool safe = targetedConfirmed && confirmed &&
                    !safety_.torqueOffPending();
  safetyFault_ = operatorInspectionRequired_ || !safe;
  startResponse(sequence, true, nullptr);
  append("{\"stopped\":true,\"torqueState\":\"%s\","
         "\"confirmed\":%s,\"torqueOffBroadcastSent\":%s,"
         "\"torqueOffPending\":%s}",
         safe ? "off" : "unknown", safe ? "true" : "false",
         sent ? "true" : "false",
         safety_.torqueOffPending() ? "true" : "false");
  sendResponse();
}

void ArmHatRuntime::handleReset(uint32_t sequence,
                                const ParsedCommand& command) {
  if (command.argumentCount != 1 ||
      std::strcmp(command.arguments[0], "INSPECTED") != 0) {
    sendError(sequence, "BAD_ARGS");
    return;
  }
  const uint32_t nowMs = millis();
  if (!safety_.heartbeatFresh(nowMs)) {
    sendError(sequence, "HEARTBEAT_STALE");
    return;
  }
  const bool targetedConfirmed = resolvePendingTorqueOff();
  (void)bus_.broadcastTorqueOff();
  bool confirmed = servoCount_ > 0 && confirmAllTrackedTorqueOff();
  const bool torqueProven = targetedConfirmed && confirmed &&
                            !safety_.torqueOffPending();
  // INSPECTED is an operator assertion, not electrical proof. Never release
  // the STOP latch until every tracked/pending member is addressed and read
  // back off. A silent bus therefore leaves the inspection latch recoverable
  // by a later RESET after communication returns, without ever reporting OK.
  if (!torqueProven) {
    safetyFault_ = true;
    sendError(sequence, "RESET_PRECONDITION",
              "{\"inspectedToken\":true,\"torqueOffConfirmed\":false}");
    return;
  }
  if (!safety_.resetInspected(millis())) {
    safetyFault_ = true;
    sendError(sequence, "RESET_PRECONDITION",
              "{\"inspectedToken\":true,\"torqueOffConfirmed\":true}");
    return;
  }
  invalidateProposal();
  safetyFault_ = false;
  operatorInspectionRequired_ = false;
  explicitStopLatched_ = false;
  startResponse(sequence, true, nullptr);
  append("{\"stopped\":false,\"torqueState\":\"off\","
         "\"torqueOffConfirmed\":true,\"reset\":true}");
  sendResponse();
}

void ArmHatRuntime::tick() {
  const uint32_t nowMs = millis();
  if (bootTorqueOffPending_) {
    // First loop iteration, not setup(): poll() has already run, so the host
    // link is live and a bus that refuses to go quiet is now reportable rather
    // than terminal.
    bootTorqueOffPending_ = false;
    bootTorqueOffSent_ = bus_.broadcastTorqueOff();
    safety_.markBootTorqueOffAttempted();
    safetyFault_ = !bootTorqueOffSent_;
  }
  const SafetyEvent event = safety_.tick(nowMs);
  if (event == SafetyEvent::NONE) {
    (void)superviseActiveLease(nowMs);
    return;
  }
  leaseSupervisionSeen_ = false;
  invalidateProposal();
  if (event == SafetyEvent::LEASE_EXPIRED) {
    bool confirmed = true;
    const uint16_t attempts = safety_.torqueOffPendingCount();
    for (uint16_t attempt = 0; attempt < attempts; ++attempt) {
      uint8_t expiredId = 0;
      if (!safety_.nextUnauthorizedTorqueOff(nowMs, expiredId)) {
        break;
      }
      if (!torqueOffAndConfirm(expiredId)) {
        confirmed = false;
        break;
      }
    }
    if (!confirmed) {
      safety_.revokeAuthority();
      clearHoldGoals();
      (void)resolvePendingTorqueOff();
    } else if (!safety_.holdSetActive(nowMs)) {
      clearHoldGoals();
    }
    safetyFault_ = !confirmed;
    return;
  }
  if (event == SafetyEvent::WATCHDOG_STOP) {
    clearHoldGoals();
  }
  bool watchdogBroadcastSent = true;
  if (event == SafetyEvent::WATCHDOG_STOP) {
    // A dead host revokes every authority. Put the one-packet halt on the wire
    // before the slower addressed retry/proof work below.
    watchdogBroadcastSent = bus_.broadcastTorqueOff();
    safetyFault_ = !watchdogBroadcastSent;
  }
  if (!safety_.torqueOffRetryDue(nowMs)) {
    return;
  }
  uint8_t retryId = safety_.torqueOffServoId();
  (void)safety_.nextUnauthorizedTorqueOff(nowMs, retryId);
  const bool targetedConfirmed = torqueOffAndConfirm(retryId);
  if (!targetedConfirmed) {
    safety_.revokeAuthority();
  }
  if (event == SafetyEvent::WATCHDOG_STOP) {
    const bool confirmed = servoCount_ > 0 && confirmAllTrackedTorqueOff();
    safetyFault_ = !targetedConfirmed || !watchdogBroadcastSent || !confirmed ||
                   safety_.torqueOffPending();
    return;
  }
  safetyFault_ = !targetedConfirmed || safety_.torqueOffPending();
}

}  // namespace armhat
