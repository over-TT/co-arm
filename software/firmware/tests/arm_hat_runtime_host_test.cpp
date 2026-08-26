#include <cassert>
#include <array>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <initializer_list>
#include <string>
#include <vector>

#include <Arduino.h>
#include "ArmHatProtocol.h"

// The runtime deliberately keeps its bus, safety state, and telemetry ledger
// private. This host-only access shim seeds the same state that the public
// commissioning commands establish, then calls the public runtime loop entry
// points. Production is compiled as a separate, unmodified translation unit.
#define private public
#include "ArmHatController.h"
#undef private

namespace {

using armhat::ArmHatRuntime;
using armhat::MultiTurnTruthState;
using armhat::ServoTelemetry;
using armhat::TorqueState;

constexpr std::uint8_t kServoId = 1;
constexpr std::uint8_t kRegisterMinimum = 9;
constexpr std::uint8_t kRegisterMaximum = 11;
constexpr std::uint8_t kRegisterPhase = 0x12;
constexpr std::uint8_t kRegisterResolution = 0x1E;
constexpr std::uint8_t kRegisterOperatingMode = 33;
constexpr std::uint8_t kRegisterTorqueEnable = 40;
constexpr std::uint8_t kRegisterGoal = 41;
constexpr std::uint8_t kRegisterLock = 55;
constexpr std::uint8_t kRegisterPresentPosition = 56;

class NullHost final : public Stream {
 public:
  int available() override { return static_cast<int>(input_.size()); }
  int read() override {
    if (input_.empty()) {
      return -1;
    }
    const std::uint8_t value = input_.front();
    input_.pop_front();
    return value;
  }
  std::size_t write(const std::uint8_t* bytes, std::size_t size) override {
    output_.insert(output_.end(), bytes, bytes + size);
    return size;
  }

  void queue(const char* line) {
    while (*line != '\0') {
      input_.push_back(static_cast<std::uint8_t>(*line++));
    }
  }

  std::string outputText() const {
    return std::string(output_.begin(), output_.end());
  }

 private:
  std::deque<std::uint8_t> input_;
  std::vector<std::uint8_t> output_;
};

class ScriptedServoSerial final : public HardwareSerial {
 public:
  explicit ScriptedServoSerial(std::initializer_list<std::int32_t> countdown)
      : countdown_(countdown) {}

  int available() override {
    if (response_.empty() || armhat_test_clock::nowUs < responseReadyAtUs_) {
      return 0;
    }
    return static_cast<int>(response_.size());
  }

  int read() override {
    if (response_.empty() || armhat_test_clock::nowUs < responseReadyAtUs_) {
      return -1;
    }
    const std::uint8_t value = response_.front();
    response_.pop_front();
    return value;
  }

  std::size_t write(const std::uint8_t* packet, std::size_t size) override {
    assert(packet != nullptr);
    assert(size >= 6);
    assert(packet[0] == 0xFF && packet[1] == 0xFF);
    const std::uint8_t id = packet[2];
    const std::uint8_t instruction = packet[4];
    if (id == armhat::ST3215_BROADCAST_ID) {
      if (instruction == armhat::SCS_INSTRUCTION_WRITE && size >= 8 &&
          packet[5] == kRegisterTorqueEnable && packet[6] == 0) {
        packetOrder_.push_back('B');
      }
      return size;
    }
    if (instruction == armhat::SCS_INSTRUCTION_PING) {
      // A complete census still sends all 254 PINGs. This fixture models one
      // responder and silence everywhere else.
      if (id == kServoId && dropNextPingReply_) {
        dropNextPingReply_ = false;
        return size;
      }
      if (id == kServoId && !dropReplies_) {
        queueStatus(id, {});
        responseReadyAtUs_ = armhat_test_clock::nowUs + pingReplyDelayUs_;
      }
      return size;
    }
    assert(id == kServoId);
    if (dropReplies_) {
      return size;
    }

    if (instruction == armhat::SCS_INSTRUCTION_WRITE) {
      handleWrite(id, packet, size);
    } else if (instruction == armhat::SCS_INSTRUCTION_READ) {
      handleRead(id, packet, size);
    } else {
      assert(false && "unexpected servo instruction");
    }
    return size;
  }

  std::size_t countdownReads() const { return countdownReads_; }
  std::size_t torqueOffWrites() const { return torqueOffWrites_; }
  std::size_t modeZeroWrites() const { return modeZeroWrites_; }
  std::int32_t commandedStep() const { return commandedStep_; }
  std::size_t goalWrites() const { return goalWrites_; }
  std::size_t configurationWrites() const { return configurationWrites_; }
  std::size_t torqueOnWrites() const { return torqueOnWrites_; }
  void setModeZeroEncoder(std::uint16_t value) { modeZeroEncoder_ = value; }
  void setOperatingMode(std::uint8_t value) { operatingMode_ = value; }
  void setNativeAbsoluteConfigured() {
    phase_ = static_cast<std::uint8_t>(phase_ | 0x10u);
    resolution_ = 1;
    minimum_ = 0;
    maximum_ = 0;
    operatingMode_ = 0;
    locked_ = true;
  }
  void setTorqueEnabled(bool value) { torqueEnabled_ = value; }
  void setLocked(bool value) { locked_ = value; }
  void setDropReplies(bool value) { dropReplies_ = value; }
  void dropNextPingReply() { dropNextPingReply_ = true; }
  void setPingReplyDelayUs(std::uint32_t value) { pingReplyDelayUs_ = value; }
  void dropNextTelemetryRead() { dropNextTelemetryRead_ = true; }
  void dropNextOperatingModeRead() { dropNextOperatingModeRead_ = true; }
  std::uint8_t phase() const { return phase_; }
  std::uint8_t resolution() const { return resolution_; }
  std::uint16_t minimum() const { return minimum_; }
  std::uint16_t maximum() const { return maximum_; }
  std::uint8_t operatingMode() const { return operatingMode_; }
  bool locked() const { return locked_; }
  bool torqueEnabled() const { return torqueEnabled_; }
  const std::vector<char>& packetOrder() const { return packetOrder_; }

 private:
  static std::uint16_t encodeSignedMagnitude(std::int32_t value) {
    return value < 0
               ? static_cast<std::uint16_t>(-value) | armhat::ST3215_SIGN_BIT
               : static_cast<std::uint16_t>(value);
  }

  static std::int32_t decodeSignedMagnitude(std::uint16_t value) {
    const std::int32_t magnitude = value & ~armhat::ST3215_SIGN_BIT;
    return (value & armhat::ST3215_SIGN_BIT) != 0 ? -magnitude : magnitude;
  }

  void queueStatus(std::uint8_t id,
                   std::initializer_list<std::uint8_t> parameters) {
    assert(response_.empty());
    responseReadyAtUs_ = armhat_test_clock::nowUs;
    const std::uint8_t length =
        static_cast<std::uint8_t>(parameters.size() + 2u);
    response_.push_back(0xFF);
    response_.push_back(0xFF);
    response_.push_back(id);
    response_.push_back(length);
    response_.push_back(0);  // servo error
    std::uint8_t sum = static_cast<std::uint8_t>(id + length);
    for (const std::uint8_t value : parameters) {
      response_.push_back(value);
      sum = static_cast<std::uint8_t>(sum + value);
    }
    response_.push_back(static_cast<std::uint8_t>(~sum));
  }

  void handleWrite(std::uint8_t id, const std::uint8_t* packet,
                   std::size_t size) {
    assert(size >= 8);
    const std::uint8_t address = packet[5];
    if (address == kRegisterGoal) {
      assert(size >= 13);
      const std::uint16_t encoded = static_cast<std::uint16_t>(packet[7]) |
                                    (static_cast<std::uint16_t>(packet[8]) << 8u);
      commandedStep_ = decodeSignedMagnitude(encoded);
      commandedAcceleration_ = packet[6];
      commandedSpeed_ = static_cast<std::uint16_t>(packet[11]) |
                          (static_cast<std::uint16_t>(packet[12]) << 8u);
      ++goalWrites_;
    } else if (address == kRegisterTorqueEnable) {
      torqueEnabled_ = packet[6] != 0;
      if (!torqueEnabled_) {
        ++torqueOffWrites_;
        packetOrder_.push_back('A');
      } else {
        ++torqueOnWrites_;
      }
    } else if (address == kRegisterOperatingMode) {
      ++configurationWrites_;
      operatingMode_ = packet[6];
      if (operatingMode_ == 0) {
        ++modeZeroWrites_;
      }
    } else if (address == kRegisterLock) {
      ++configurationWrites_;
      locked_ = packet[6] != 0;
    } else if (address == kRegisterPhase) {
      ++configurationWrites_;
      phase_ = packet[6];
    } else if (address == kRegisterResolution) {
      ++configurationWrites_;
      resolution_ = packet[6];
    } else if (address == kRegisterMinimum) {
      ++configurationWrites_;
      minimum_ = static_cast<std::uint16_t>(packet[6]) |
                 (static_cast<std::uint16_t>(packet[7]) << 8u);
    } else if (address == kRegisterMaximum) {
      ++configurationWrites_;
      maximum_ = static_cast<std::uint16_t>(packet[6]) |
                 (static_cast<std::uint16_t>(packet[7]) << 8u);
    }
    queueStatus(id, {});
  }

  void handleRead(std::uint8_t id, const std::uint8_t* packet,
                  std::size_t size) {
    assert(size >= 8);
    const std::uint8_t address = packet[5];
    const std::uint8_t length = packet[6];
    if (address == kRegisterPresentPosition && length == 2) {
      std::uint16_t encoded = modeZeroEncoder_;
      if (operatingMode_ == 3) {
        ++countdownReads_;
        const std::int32_t remaining =
            countdownIndex_ < countdown_.size()
                ? countdown_[countdownIndex_++]
                : (countdown_.empty() ? 0 : countdown_.back());
        encoded = encodeSignedMagnitude(remaining);
      }
      queueStatus(id, {static_cast<std::uint8_t>(encoded & 0xFFu),
                       static_cast<std::uint8_t>(encoded >> 8u)});
      return;
    }
    if (address == kRegisterPresentPosition && length == 15) {
      if (dropNextTelemetryRead_) {
        dropNextTelemetryRead_ = false;
        return;
      }
      const std::uint16_t encoded = modeZeroEncoder_;
      queueStatus(id, {static_cast<std::uint8_t>(encoded & 0xFFu),
                       static_cast<std::uint8_t>(encoded >> 8u),
                       0, 0, 0, 0, 120, 25, 0, 0, 0, 0, 0, 0, 0});
      return;
    }
    if (address == kRegisterGoal && length == 7) {
      const std::uint16_t encoded = encodeSignedMagnitude(commandedStep_);
      queueStatus(id, {commandedAcceleration_,
                       static_cast<std::uint8_t>(encoded & 0xFFu),
                       static_cast<std::uint8_t>(encoded >> 8u),
                       0, 0,
                       static_cast<std::uint8_t>(commandedSpeed_ & 0xFFu),
                       static_cast<std::uint8_t>(commandedSpeed_ >> 8u)});
      return;
    }
    if (address == kRegisterTorqueEnable && length == 1) {
      queueStatus(id, {static_cast<std::uint8_t>(torqueEnabled_ ? 1 : 0)});
      return;
    }
    if (address == kRegisterOperatingMode && length == 1) {
      if (dropNextOperatingModeRead_) {
        dropNextOperatingModeRead_ = false;
        return;
      }
      queueStatus(id, {operatingMode_});
      return;
    }
    if (address == kRegisterLock && length == 1) {
      queueStatus(id, {static_cast<std::uint8_t>(locked_ ? 1 : 0)});
      return;
    }
    if (address == kRegisterPhase && length == 1) {
      queueStatus(id, {phase_});
      return;
    }
    if (address == kRegisterResolution && length == 1) {
      queueStatus(id, {resolution_});
      return;
    }
    if (address == kRegisterMinimum && length == 4) {
      queueStatus(id, {static_cast<std::uint8_t>(minimum_ & 0xFFu),
                       static_cast<std::uint8_t>(minimum_ >> 8u),
                       static_cast<std::uint8_t>(maximum_ & 0xFFu),
                       static_cast<std::uint8_t>(maximum_ >> 8u)});
      return;
    }
    assert(false && "unexpected register read");
  }

  std::deque<std::uint8_t> response_;
  std::vector<std::int32_t> countdown_;
  std::size_t countdownIndex_ = 0;
  std::size_t countdownReads_ = 0;
  std::size_t torqueOffWrites_ = 0;
  std::size_t modeZeroWrites_ = 0;
  std::size_t goalWrites_ = 0;
  std::size_t configurationWrites_ = 0;
  std::size_t torqueOnWrites_ = 0;
  std::int32_t commandedStep_ = 0;
  std::uint16_t commandedSpeed_ = 1;
  std::uint8_t commandedAcceleration_ = 1;
  std::uint16_t modeZeroEncoder_ = 0;
  std::uint8_t operatingMode_ = 3;
  bool torqueEnabled_ = true;
  std::vector<char> packetOrder_;
  bool dropReplies_ = false;
  bool dropNextPingReply_ = false;
  std::uint32_t pingReplyDelayUs_ = 0;
  std::uint64_t responseReadyAtUs_ = 0;
  bool dropNextTelemetryRead_ = false;
  bool dropNextOperatingModeRead_ = false;
  bool locked_ = true;
  std::uint8_t phase_ = 0x03;
  std::uint8_t resolution_ = 1;
  std::uint16_t minimum_ = 0;
  std::uint16_t maximum_ = armhat::ST3215_POSITION_MAX;
};

class MoveSetServoSerial final : public HardwareSerial {
 public:
  int available() override { return static_cast<int>(response_.size()); }

  int read() override {
    if (response_.empty()) {
      return -1;
    }
    const std::uint8_t value = response_.front();
    response_.pop_front();
    return value;
  }

  std::size_t write(const std::uint8_t* packet, std::size_t size) override {
    assert(packet != nullptr && size >= 6);
    assert(packet[0] == 0xFF && packet[1] == 0xFF);
    const std::uint8_t id = packet[2];
    const std::uint8_t instruction = packet[4];
    if (id == armhat::ST3215_BROADCAST_ID) {
      if (instruction == armhat::SCS_INSTRUCTION_SYNC_WRITE) {
        ++syncWriteCount_;
        lastSyncWriteAtUs_ = armhat_test_clock::nowUs;
        const std::uint8_t address = packet[5];
        const std::uint8_t valueCount = packet[6];
        std::size_t cursor = 7;
        while (cursor + valueCount < size - 1) {
          const std::uint8_t memberId = packet[cursor++];
          stageIds_.push_back(memberId);
          torqueEnabled_[memberId] = true;
          stagedAddress_[memberId] = address;
          stagedLength_[memberId] = valueCount;
          for (std::uint8_t index = 0; index < valueCount; ++index) {
            staged_[memberId][index] = packet[cursor++];
          }
          if (address == 41 && valueCount == 7) {
            presentPosition_[memberId] = static_cast<std::uint16_t>(
                staged_[memberId][1] |
                (static_cast<std::uint16_t>(staged_[memberId][2]) << 8u));
          }
        }
      } else if (instruction == armhat::SCS_INSTRUCTION_WRITE) {
        ++broadcastTorqueOffCount_;
        packetOrder_.push_back('B');
        if (shortBroadcastTorqueOff_) {
          return size - 1;
        }
        torqueEnabled_[2] = false;
        torqueEnabled_[4] = false;
      } else {
        assert(false && "unexpected broadcast instruction");
      }
      return size;
    }
    assert(id == 2 || id == 4);

    if (instruction == armhat::SCS_INSTRUCTION_WRITE) {
      assert(packet[5] == kRegisterTorqueEnable && size == 8);
      torqueEnabled_[id] = packet[6] != 0;
      if (!torqueEnabled_[id]) {
        torqueOffIds_.push_back(id);
        packetOrder_.push_back('A');
      }
      if (!(id == failedTorqueOffId_ && !torqueEnabled_[id])) {
        queueStatus(id, {});
      }
      return size;
    }
    if (instruction == armhat::SCS_INSTRUCTION_READ) {
      assert(size == 8);
      const std::uint8_t address = packet[5];
      const std::uint8_t length = packet[6];
      if (address == kRegisterTorqueEnable && length == 1) {
        if (id != failedTorqueOffId_) {
          queueStatus(id,
                      {static_cast<std::uint8_t>(torqueEnabled_[id] ? 1 : 0)});
        }
        return size;
      }
      if (address == 33 && length == 1) {
        queueStatus(id, {0});
        return size;
      }
      if (address == 56 && length == 15) {
        ++feedbackReadAttempts_[id];
        if (id == failedFeedbackId_ ||
            feedbackReadAttempts_[id] <= transientFeedbackFailures_[id]) {
          return size;
        }
        std::array<std::uint8_t, 15> telemetry{};
        // FOLLOW_SET is STS-only: acceleration precedes the little-endian
        // position in the staged address-41 block.
        telemetry[0] = static_cast<std::uint8_t>(presentPosition_[id] & 0xFFu);
        telemetry[1] = static_cast<std::uint8_t>(presentPosition_[id] >> 8u);
        telemetry[6] = 120;  // 12.0 V
        telemetry[7] = 28;
        telemetry[10] = 1;
        queueStatusBytes(id, telemetry.data(), telemetry.size());
        return size;
      }
      ++goalReadAttempts_[id];
      if (id == failedGoalReadId_ ||
          (id == transientGoalReadFailureId_ &&
           goalReadAttempts_[id] <= transientGoalReadFailureCount_) ||
          (id == quietGoalReadId_ &&
           armhat_test_clock::nowUs - lastSyncWriteAtUs_ <
               quietGoalReadAfterSyncUs_)) {
        return size;
      }
      assert(address == stagedAddress_[id]);
      assert(length == stagedLength_[id]);
      if (id == transientGoalMismatchId_ && goalReadAttempts_[id] == 1) {
        std::array<std::uint8_t, 7> mismatched = staged_[id];
        mismatched[0] = static_cast<std::uint8_t>(mismatched[0] + 1u);
        queueStatusBytes(id, mismatched.data(), stagedLength_[id]);
        return size;
      }
      queueStatusBytes(id, staged_[id].data(), stagedLength_[id]);
      return size;
    }
    assert(false && "unexpected move-set instruction");
    return size;
  }

  void setFailedTorqueOffId(std::uint8_t id) { failedTorqueOffId_ = id; }
  void setFailedGoalReadId(std::uint8_t id) { failedGoalReadId_ = id; }
  void setTransientGoalReadFailureId(std::uint8_t id) {
    transientGoalReadFailureId_ = id;
  }
  void setTransientGoalReadFailureCount(std::uint8_t id,
                                        std::size_t count) {
    transientGoalReadFailureId_ = id;
    transientGoalReadFailureCount_ = count;
  }
  void setTransientGoalMismatchId(std::uint8_t id) {
    transientGoalMismatchId_ = id;
  }
  void setGoalReadQuietAfterSync(std::uint8_t id, std::uint32_t quietUs) {
    quietGoalReadId_ = id;
    quietGoalReadAfterSyncUs_ = quietUs;
  }
  void setFailedFeedbackId(std::uint8_t id) { failedFeedbackId_ = id; }
  void setTransientFeedbackFailures(std::uint8_t id, std::size_t count) {
    transientFeedbackFailures_[id] = count;
  }
  void setPresentPosition(std::uint8_t id, std::uint16_t position) {
    presentPosition_[id] = position;
    torqueEnabled_[id] = true;
  }
  void setShortBroadcastTorqueOff(bool value) {
    shortBroadcastTorqueOff_ = value;
  }
  const std::vector<std::uint8_t>& stageIds() const { return stageIds_; }
  const std::vector<std::uint8_t>& torqueOffIds() const {
    return torqueOffIds_;
  }
  std::size_t syncWriteCount() const { return syncWriteCount_; }
  std::uint64_t lastSyncWriteAtUs() const { return lastSyncWriteAtUs_; }
  std::size_t goalReadAttempts(std::uint8_t id) const {
    return goalReadAttempts_[id];
  }
  std::size_t feedbackReadAttempts(std::uint8_t id) const {
    return feedbackReadAttempts_[id];
  }
  std::size_t broadcastTorqueOffCount() const {
    return broadcastTorqueOffCount_;
  }
  const std::vector<char>& packetOrder() const { return packetOrder_; }
  std::uint8_t stagedAddress(std::uint8_t id) const {
    return stagedAddress_[id];
  }
  std::uint8_t stagedLength(std::uint8_t id) const {
    return stagedLength_[id];
  }
  const std::array<std::uint8_t, 7>& staged(std::uint8_t id) const {
    return staged_[id];
  }

 private:
  void queueStatus(std::uint8_t id,
                   std::initializer_list<std::uint8_t> parameters) {
    std::array<std::uint8_t, 16> values{};
    std::uint8_t count = 0;
    for (const std::uint8_t value : parameters) {
      values[count++] = value;
    }
    queueStatusBytes(id, values.data(), count);
  }

  void queueStatusBytes(std::uint8_t id, const std::uint8_t* parameters,
                        std::uint8_t parameterCount) {
    assert(response_.empty());
    const std::uint8_t length = static_cast<std::uint8_t>(parameterCount + 2u);
    response_.push_back(0xFF);
    response_.push_back(0xFF);
    response_.push_back(id);
    response_.push_back(length);
    response_.push_back(0);
    std::uint8_t sum = static_cast<std::uint8_t>(id + length);
    for (std::uint8_t index = 0; index < parameterCount; ++index) {
      response_.push_back(parameters[index]);
      sum = static_cast<std::uint8_t>(sum + parameters[index]);
    }
    response_.push_back(static_cast<std::uint8_t>(~sum));
  }

  std::deque<std::uint8_t> response_;
  std::array<bool, 256> torqueEnabled_{};
  std::array<std::uint8_t, 256> stagedAddress_{};
  std::array<std::uint8_t, 256> stagedLength_{};
  std::array<std::array<std::uint8_t, 7>, 256> staged_{};
  std::array<std::uint16_t, 256> presentPosition_{};
  std::array<std::size_t, 256> goalReadAttempts_{};
  std::array<std::size_t, 256> feedbackReadAttempts_{};
  std::array<std::size_t, 256> transientFeedbackFailures_{};
  std::vector<std::uint8_t> stageIds_;
  std::vector<std::uint8_t> torqueOffIds_;
  std::vector<char> packetOrder_;
  std::uint8_t failedTorqueOffId_ = 0;
  std::uint8_t failedGoalReadId_ = 0;
  std::uint8_t transientGoalReadFailureId_ = 0;
  std::size_t transientGoalReadFailureCount_ = 1;
  std::uint8_t transientGoalMismatchId_ = 0;
  std::uint8_t quietGoalReadId_ = 0;
  std::uint32_t quietGoalReadAfterSyncUs_ = 0;
  std::uint8_t failedFeedbackId_ = 0;
  bool shortBroadcastTorqueOff_ = false;
  std::size_t syncWriteCount_ = 0;
  std::size_t broadcastTorqueOffCount_ = 0;
  std::uint64_t lastSyncWriteAtUs_ = 0;
};

struct RuntimeFixture {
  explicit RuntimeFixture(std::initializer_list<std::int32_t> countdown)
      : serial(countdown), runtime(host, serial) {
    armhat_test_clock::reset(1000u * 1000u);
    runtime.safety_.begin(millis());
    runtime.safety_.markBootTorqueOffAttempted();
    runtime.bootTorqueOffPending_ = false;
    // This fixture models an already commissioned runtime.
    runtime.safety_.heartbeat(millis());
    assert(runtime.safety_.queueTorqueOffObligation(kServoId));
    const std::uint8_t held[] = {kServoId};
    assert(runtime.safety_.setHoldSet(held, 1, 2000, millis()));

    runtime.servoCount_ = 1;
    servo = &runtime.servos_[0];
    *servo = ServoTelemetry{};
    servo->id = kServoId;
    servo->online = true;
    servo->fresh = true;
    servo->sampled = true;
    servo->operatingModeKnown = true;
    servo->operatingMode = 3;
    servo->torque = TorqueState::ON;
    servo->odometerTracking = true;
    servo->odometerValid = true;
    servo->odometerLastRaw = 0;
    servo->odometerSampledAtMs = millis();
    armhat::seedMultiTurnTruth(servo->multiTurnTruth, 0);
    runtime.bus_.setMultiTurn(kServoId, true);
    auto& goal = runtime.multiTurnGoals_[0];
    goal.servoId = kServoId;
    goal.active = true;
    goal.target = 6372;
    goal.speed = 1400;
    goal.acceleration = 12;
    goal.lastStepMs = 0;
    goal.lastRemaining = 0;
    goal.lastProgressMs = millis();
  }

  NullHost host;
  ScriptedServoSerial serial;
  ArmHatRuntime runtime;
  ServoTelemetry* servo = nullptr;
};

void configureMoveSetRuntime(ArmHatRuntime& runtime) {
  armhat_test_clock::reset(1000u * 1000u);
  runtime.bootTorqueOffPending_ = false;
  runtime.safety_.begin(millis());
  runtime.safety_.markBootTorqueOffAttempted();
  runtime.safety_.heartbeat(millis());
  runtime.policy_.maxSpeed = 2400;
  runtime.policy_.maxAccel = 80;
  const std::uint8_t held[] = {2, 4};
  assert(runtime.safety_.queueTorqueOffObligation(2));
  assert(runtime.safety_.queueTorqueOffObligation(4));
  assert(runtime.safety_.setHoldSet(held, 2, 2000, millis()));
  runtime.leaseSupervisionSeen_ = true;
  runtime.lastLeaseSupervisionMs_ = millis();
  runtime.servoCount_ = 2;
  for (std::uint8_t index = 0; index < 2; ++index) {
    ServoTelemetry& servo = runtime.servos_[index];
    servo = ServoTelemetry{};
    servo.id = held[index];
    servo.online = true;
    servo.fresh = true;
    servo.sampled = true;
    servo.rawPosition = 100;
    servo.operatingModeKnown = true;
    servo.operatingMode = 0;
    servo.torque = TorqueState::ON;
    servo.sampledAtMs = millis();
  }
  runtime.bus_.setFamily(4, armhat::ServoFamily::SCS);
  runtime.controllerId_[0] = 't';
  runtime.controllerId_[1] = 'e';
  runtime.controllerId_[2] = 's';
  runtime.controllerId_[3] = 't';
  runtime.bootId_ = 0x1234u;
}

void moveSetDispatchesMixedDialectSyncWrites() {
  NullHost host;
  MoveSetServoSerial serial;
  ArmHatRuntime runtime(host, serial);
  configureMoveSetRuntime(runtime);

  host.queue("A1 20 MOVE_SET 2,1000,200,10 4,512,150,15\n");
  runtime.poll();

  assert((serial.stageIds() == std::vector<std::uint8_t>{2, 4}));
  assert(serial.syncWriteCount() == 2);
  assert(serial.stagedAddress(2) == 41);
  assert(serial.stagedLength(2) == 7);
  assert(serial.staged(2)[0] == 10);    // STS acceleration first.
  assert(serial.staged(2)[1] == 0xE8);  // position 1000, little endian.
  assert(serial.staged(2)[2] == 0x03);
  assert(serial.staged(2)[5] == 200);   // speed 200, little endian.
  assert(serial.stagedAddress(4) == 42);
  assert(serial.stagedLength(4) == 6);
  assert(serial.staged(4)[0] == 0x02);  // position 512, big endian.
  assert(serial.staged(4)[1] == 0x00);
  assert(serial.staged(4)[4] == 0x00);  // speed 150, big endian.
  assert(serial.staged(4)[5] == 150);
  const std::string output = host.outputText();
  assert(output.find("A1 20 OK") != std::string::npos);
  assert(output.find("arm-hat-2.7.2") != std::string::npos);
  assert(output.find("\"bootId\":\"boot-") !=
         std::string::npos);
}

void followSetReturnsFreshFeedbackForTwoStsMembers() {
  NullHost host;
  MoveSetServoSerial serial;
  ArmHatRuntime runtime(host, serial);
  configureMoveSetRuntime(runtime);
  runtime.bus_.setFamily(4, armhat::ServoFamily::STS);

  host.queue("A1 120 FOLLOW_SET 2,1000,200,10 4,1500,250,15\n");
  runtime.poll();

  assert((serial.stageIds() == std::vector<std::uint8_t>{2, 4}));
  assert(serial.syncWriteCount() == 1);
  const std::string output = host.outputText();
  assert(output.find("A1 120 OK") != std::string::npos);
  assert(output.find("arm-hat-2.7.2") != std::string::npos);
  assert(output.find("\"feedback\":[{\"servoId\":2,\"rawPosition\":1000") !=
         std::string::npos);
  assert(output.find("\"voltageDeciVolts\":120,\"temperatureC\":28") !=
         std::string::npos);
  assert(output.find("{\"servoId\":4,\"rawPosition\":1500") !=
         std::string::npos);
}

void followSetRetriesOneTransientGoalMismatchThenRequiresExactReadback() {
  NullHost host;
  MoveSetServoSerial serial;
  serial.setTransientGoalMismatchId(2);
  ArmHatRuntime runtime(host, serial);
  configureMoveSetRuntime(runtime);
  runtime.bus_.setFamily(4, armhat::ServoFamily::STS);

  host.queue("A1 126 FOLLOW_SET 2,1000,200,10 4,1500,250,15\n");
  runtime.poll();

  assert(serial.syncWriteCount() == 1);
  assert(serial.goalReadAttempts(2) == 2);
  assert(serial.goalReadAttempts(4) == 1);
  assert(serial.broadcastTorqueOffCount() == 0);
  assert(serial.torqueOffIds().empty());
  assert(host.outputText().find("A1 126 OK") != std::string::npos);
}

void followSetAtMaxRetriesTwoSilentGoalReadsWithoutRedispatch() {
  NullHost host;
  MoveSetServoSerial serial;
  serial.setTransientGoalReadFailureCount(4, 2);
  ArmHatRuntime runtime(host, serial);
  configureMoveSetRuntime(runtime);
  runtime.bus_.setFamily(4, armhat::ServoFamily::STS);

  host.queue("A1 128 FOLLOW_SET 2,1000,2400,80 4,1500,2400,80\n");
  runtime.poll();

  assert(serial.syncWriteCount() == 1);
  assert(serial.goalReadAttempts(2) == 1);
  assert(serial.goalReadAttempts(4) == 3);
  assert(serial.staged(2)[0] == 80);
  assert(serial.staged(2)[5] == 0x60);
  assert(serial.staged(2)[6] == 0x09);
  assert(serial.staged(4)[0] == 80);
  assert(serial.staged(4)[5] == 0x60);
  assert(serial.staged(4)[6] == 0x09);
  assert(serial.broadcastTorqueOffCount() == 0);
  assert(serial.torqueOffIds().empty());
  assert(host.outputText().find("A1 128 OK") != std::string::npos);
}

void followSetAtMaxSurvivesTwentyMillisecondPostDispatchReplyGap() {
  NullHost host;
  MoveSetServoSerial serial;
  // Reproduce the measured max-load shape more faithfully than a fixed drop
  // count: the first addressed Shoulder proof is unavailable until 20 ms
  // after the single grouped write. The controller may retry readback, never
  // the motion command.
  serial.setGoalReadQuietAfterSync(2, 20000);
  ArmHatRuntime runtime(host, serial);
  configureMoveSetRuntime(runtime);
  runtime.bus_.setFamily(4, armhat::ServoFamily::STS);

  host.queue("A1 130 FOLLOW_SET 2,1000,2400,80 4,1500,2400,80\n");
  runtime.poll();

  assert(serial.syncWriteCount() == 1);
  assert(serial.goalReadAttempts(2) == 4);
  assert(serial.goalReadAttempts(4) == 1);
  assert(armhat_test_clock::nowUs - serial.lastSyncWriteAtUs() >= 20000);
  assert(serial.broadcastTorqueOffCount() == 0);
  assert(serial.torqueOffIds().empty());
  assert(host.outputText().find("A1 130 OK") != std::string::npos);
}

void followSetAtMaxRetriesTransientFeedbackWithoutRedispatch() {
  NullHost host;
  MoveSetServoSerial serial;
  serial.setTransientFeedbackFailures(4, 2);
  ArmHatRuntime runtime(host, serial);
  configureMoveSetRuntime(runtime);
  runtime.bus_.setFamily(4, armhat::ServoFamily::STS);

  host.queue("A1 129 FOLLOW_SET 2,1000,2400,80 4,1500,2400,80\n");
  runtime.poll();

  assert(serial.syncWriteCount() == 1);
  assert(serial.feedbackReadAttempts(2) == 1);
  assert(serial.feedbackReadAttempts(4) == 3);
  assert(serial.broadcastTorqueOffCount() == 0);
  assert(serial.torqueOffIds().empty());
  assert(host.outputText().find("A1 129 OK") != std::string::npos);
}

void followSetFeedbackAmbiguityStopsBothMembers() {
  NullHost host;
  MoveSetServoSerial serial;
  serial.setFailedFeedbackId(4);
  ArmHatRuntime runtime(host, serial);
  configureMoveSetRuntime(runtime);
  runtime.bus_.setFamily(4, armhat::ServoFamily::STS);

  host.queue("A1 121 FOLLOW_SET 2,1000,200,10 4,1500,250,15\n");
  runtime.poll();

  assert(serial.syncWriteCount() == 1);
  assert(serial.feedbackReadAttempts(4) ==
         armhat::FOLLOW_FEEDBACK_ATTEMPTS);
  assert(serial.broadcastTorqueOffCount() == 1);
  assert(serial.torqueOffIds().size() >= 2);
  const std::string output = host.outputText();
  assert(output.find("A1 121 ERR MOVE_SET_FAILED") != std::string::npos);
  assert(output.find("\"phase\":\"feedback\"") != std::string::npos);
  assert(output.find("\"failedIndex\":1") != std::string::npos);
  assert(output.find("\"dispatchedFamilyCount\":1") != std::string::npos);
}

void followSetRejectsNonStsMemberBeforeDispatch() {
  NullHost host;
  MoveSetServoSerial serial;
  ArmHatRuntime runtime(host, serial);
  configureMoveSetRuntime(runtime);  // id 4 is declared SCS.

  host.queue("A1 122 FOLLOW_SET 2,1000,200,10 4,512,250,15\n");
  runtime.poll();

  assert(serial.syncWriteCount() == 0);
  assert(host.outputText().find("A1 122 ERR UNSUPPORTED") !=
         std::string::npos);
}

void followReadReturnsFreshFeedbackWithoutAnyServoWrite() {
  NullHost host;
  MoveSetServoSerial serial;
  serial.setPresentPosition(2, 1111);
  serial.setPresentPosition(4, 2222);
  ArmHatRuntime runtime(host, serial);
  configureMoveSetRuntime(runtime);
  runtime.bus_.setFamily(4, armhat::ServoFamily::STS);

  host.queue("A1 123 FOLLOW_READ 2 4\n");
  runtime.poll();

  assert(serial.syncWriteCount() == 0);
  assert(serial.broadcastTorqueOffCount() == 0);
  assert(serial.torqueOffIds().empty());
  const std::string output = host.outputText();
  assert(output.find("A1 123 OK") != std::string::npos);
  assert(output.find("\"count\":2,\"feedback\":[{\"servoId\":2") !=
         std::string::npos);
  assert(output.find("\"rawPosition\":1111") != std::string::npos);
  assert(output.find("{\"servoId\":4,\"rawPosition\":2222") !=
         std::string::npos);
}

void followReadFailureDoesNotWriteGoalsOrTorque() {
  NullHost host;
  MoveSetServoSerial serial;
  serial.setPresentPosition(2, 1111);
  serial.setPresentPosition(4, 2222);
  serial.setFailedFeedbackId(4);
  ArmHatRuntime runtime(host, serial);
  configureMoveSetRuntime(runtime);
  runtime.bus_.setFamily(4, armhat::ServoFamily::STS);

  host.queue("A1 124 FOLLOW_READ 2 4\n");
  runtime.poll();

  assert(serial.syncWriteCount() == 0);
  assert(serial.broadcastTorqueOffCount() == 0);
  assert(serial.torqueOffIds().empty());
  const std::string output = host.outputText();
  assert(output.find("A1 124 ERR FEEDBACK_UNAVAILABLE") !=
         std::string::npos);
  assert(output.find("\"failedIndex\":1") != std::string::npos);
}

void followReadRetriesTransientMovingServoFeedback() {
  NullHost host;
  MoveSetServoSerial serial;
  serial.setPresentPosition(2, 1111);
  serial.setPresentPosition(4, 2222);
  serial.setTransientFeedbackFailures(4, 2);
  ArmHatRuntime runtime(host, serial);
  configureMoveSetRuntime(runtime);
  runtime.bus_.setFamily(4, armhat::ServoFamily::STS);

  host.queue("A1 130 FOLLOW_READ 2 4\n");
  runtime.poll();

  assert(serial.syncWriteCount() == 0);
  assert(serial.feedbackReadAttempts(2) == 1);
  assert(serial.feedbackReadAttempts(4) == 3);
  assert(serial.broadcastTorqueOffCount() == 0);
  assert(serial.torqueOffIds().empty());
  assert(host.outputText().find("A1 130 OK") != std::string::npos);
}

void followReadRejectsNonStsMemberBeforeBusTraffic() {
  NullHost host;
  MoveSetServoSerial serial;
  ArmHatRuntime runtime(host, serial);
  configureMoveSetRuntime(runtime);  // id 4 is declared SCS.

  host.queue("A1 125 FOLLOW_READ 2 4\n");
  runtime.poll();

  assert(serial.syncWriteCount() == 0);
  assert(serial.broadcastTorqueOffCount() == 0);
  assert(serial.torqueOffIds().empty());
  assert(host.outputText().find("A1 125 ERR UNSUPPORTED") !=
         std::string::npos);
}

void ambiguousMoveSetVerificationStopsAndLeavesNoLatentCommand() {
  NullHost host;
  MoveSetServoSerial serial;
  serial.setFailedGoalReadId(4);
  ArmHatRuntime runtime(host, serial);
  configureMoveSetRuntime(runtime);

  host.queue("A1 21 MOVE_SET 2,1000,200,10 4,512,150,15\n");
  runtime.poll();

  assert((serial.stageIds() == std::vector<std::uint8_t>{2, 4}));
  assert(serial.syncWriteCount() == 2);
  assert(serial.goalReadAttempts(4) == armhat::POSITION_VERIFY_ATTEMPTS);
  assert(!runtime.safety_.stopped());
  assert(serial.broadcastTorqueOffCount() == 1);
  assert(!serial.packetOrder().empty() && serial.packetOrder().front() == 'B');
  assert(serial.torqueOffIds().size() >= 2);
  const std::string output = host.outputText();
  assert(output.find("A1 21 ERR MOVE_SET_FAILED") != std::string::npos);
  assert(output.find("\"phase\":\"verify\"") != std::string::npos);
  assert(output.find("\"latentCommands\":false") != std::string::npos);
  assert(output.find("\"motionMayHaveStarted\":true") != std::string::npos);
  assert(output.find("\"partialDispatchPossible\":true") !=
         std::string::npos);
  assert(output.find("\"stopped\":false") != std::string::npos);
}

void moveSetRetriesOneTransientGoalReadTimeoutThenRequiresExactReadback() {
  NullHost host;
  MoveSetServoSerial serial;
  serial.setTransientGoalReadFailureId(4);
  ArmHatRuntime runtime(host, serial);
  configureMoveSetRuntime(runtime);

  host.queue("A1 127 MOVE_SET 2,1000,200,10 4,512,150,15\n");
  runtime.poll();

  assert(serial.syncWriteCount() == 2);
  assert(serial.goalReadAttempts(2) == 1);
  assert(serial.goalReadAttempts(4) == 2);
  assert(serial.broadcastTorqueOffCount() == 0);
  assert(serial.torqueOffIds().empty());
  assert(host.outputText().find("A1 127 OK") != std::string::npos);
}

void addressedMoveSetOffProofSurvivesFailedBroadcastWrite() {
  NullHost host;
  MoveSetServoSerial serial;
  serial.setFailedGoalReadId(4);
  serial.setShortBroadcastTorqueOff(true);
  ArmHatRuntime runtime(host, serial);
  configureMoveSetRuntime(runtime);

  host.queue("A1 23 MOVE_SET 2,1000,200,10 4,512,150,15\n");
  runtime.poll();

  assert(!runtime.safety_.stopped());
  assert(serial.broadcastTorqueOffCount() == 1);
  assert(serial.torqueOffIds().size() >= 2);
  assert(host.outputText().find("\"torqueState\":\"off\"") !=
         std::string::npos);
}

void moveSetInspectionLatchSurvivesStopStatusAndHelloOnTheSameBoot() {
  NullHost host;
  MoveSetServoSerial serial;
  serial.setFailedGoalReadId(4);
  ArmHatRuntime runtime(host, serial);
  configureMoveSetRuntime(runtime);

  host.queue("A1 28 MOVE_SET 2,1000,200,10 4,512,150,15\n");
  runtime.poll();
  serial.setFailedGoalReadId(0);
  host.queue("A1 29 STOP\n");
  runtime.poll();
  armhat_test_clock::advanceUs(
      (armhat::WATCHDOG_TIMEOUT_MS + 1u) * 1000u);
  runtime.tick();
  // The latch is independent of the telemetry inventory. Drop the test-only
  // fake inventory so public STATUS does not ask this packet-focused serial
  // double for the unrelated 15-byte telemetry block.
  runtime.servoCount_ = 0;
  runtime.inventoryScanned_ = false;
  host.queue("A1 30 STATUS\n");
  runtime.poll();
  host.queue("A1 31 HELLO\n");
  runtime.poll();

  const std::string output = host.outputText();
  const std::size_t statusAt = output.find("A1 30 OK");
  const std::size_t helloAt = output.find("A1 31 OK");
  assert(output.find("A1 29 OK") != std::string::npos);
  assert(statusAt != std::string::npos);
  assert(helloAt != std::string::npos);
  const std::string status = output.substr(statusAt, helloAt - statusAt);
  const std::string hello = output.substr(helloAt);
  assert(status.find("\"state\":\"stopped_latched\"") !=
         std::string::npos);
  assert(status.find("\"motionState\":\"stopped\"") !=
         std::string::npos);
  assert(status.find("\"safetyFault\":true") != std::string::npos);
  assert(status.find("\"operatorInspectionRequired\":true") !=
         std::string::npos);
  assert(status.find("\"safetyStopReason\":\"EXPLICIT_STOP\"") !=
         std::string::npos);
  assert(hello.find("\"state\":\"stopped_latched\"") !=
         std::string::npos);
  assert(hello.find("\"safetyFault\":true") != std::string::npos);
  assert(hello.find("\"operatorInspectionRequired\":true") !=
         std::string::npos);
  assert(hello.find("\"safetyStopReason\":\"EXPLICIT_STOP\"") !=
         std::string::npos);
}

void stopBroadcastPrecedesAddressedTorqueOffProof() {
  RuntimeFixture fixture({});

  fixture.host.queue("A1 24 STOP\n");
  fixture.runtime.poll();

  assert(fixture.runtime.safety_.stopped());
  assert(fixture.serial.packetOrder().size() >= 2);
  assert(fixture.serial.packetOrder()[0] == 'B');
  assert(fixture.serial.packetOrder()[1] == 'A');
}

void explicitStopSurvivesHelloAsOperatorClearRequired() {
  RuntimeFixture fixture({});

  fixture.host.queue("A1 32 STOP\n");
  fixture.runtime.poll();
  fixture.host.queue("A1 33 STATUS\n");
  fixture.runtime.poll();
  fixture.host.queue("A1 34 HEARTBEAT\n");
  fixture.runtime.poll();
  fixture.host.queue("A1 35 HELLO\n");
  fixture.runtime.poll();

  const std::string output = fixture.host.outputText();
  const std::size_t statusAt = output.find("A1 33 OK");
  const std::size_t heartbeatAt = output.find("A1 34 OK");
  const std::size_t helloAt = output.find("A1 35 OK");
  assert(statusAt != std::string::npos);
  assert(heartbeatAt != std::string::npos);
  assert(helloAt != std::string::npos);
  const std::string status = output.substr(statusAt, heartbeatAt - statusAt);
  const std::string heartbeat = output.substr(heartbeatAt, helloAt - heartbeatAt);
  const std::string hello = output.substr(helloAt);
  for (const std::string* report : {&status, &heartbeat, &hello}) {
    assert(report->find("\"safetyFault\":true") != std::string::npos);
    assert(report->find("\"operatorInspectionRequired\":true") !=
           std::string::npos);
    assert(report->find("\"safetyStopReason\":\"EXPLICIT_STOP\"") !=
           std::string::npos);
  }
  assert(hello.find("\"state\":\"stopped_latched\"") !=
         std::string::npos);
}

void resetRefusesToClearStopWithoutAddressedTorqueOffProof() {
  RuntimeFixture fixture({});
  fixture.runtime.safety_.latchStop();
  fixture.serial.setDropReplies(true);

  fixture.host.queue("A1 25 RESET INSPECTED\n");
  fixture.runtime.poll();

  const std::string output = fixture.host.outputText();
  assert(output.find("A1 25 ERR RESET_PRECONDITION") != std::string::npos);
  assert(output.find("\"inspectedToken\":true") != std::string::npos);
  assert(output.find("\"torqueOffConfirmed\":false") !=
         std::string::npos);
  assert(fixture.runtime.safety_.stopped());
  assert(fixture.runtime.safetyFault_);
}

void resetClearsStopAfterAddressedTorqueOffProofRecovers() {
  RuntimeFixture fixture({});
  fixture.runtime.safety_.latchStop();
  fixture.serial.setDropReplies(true);

  fixture.host.queue("A1 26 RESET INSPECTED\n");
  fixture.runtime.poll();
  assert(fixture.runtime.safety_.stopped());

  fixture.serial.setDropReplies(false);
  fixture.host.queue("A1 27 RESET INSPECTED\n");
  fixture.runtime.poll();

  const std::string output = fixture.host.outputText();
  assert(output.find("A1 27 OK") != std::string::npos);
  assert(output.find("\"stopped\":false") != std::string::npos);
  assert(output.find("\"torqueState\":\"off\"") != std::string::npos);
  assert(output.find("\"torqueOffConfirmed\":true") !=
         std::string::npos);
  assert(!fixture.runtime.safety_.stopped());
  assert(!fixture.runtime.safetyFault_);
}

void inspectionLatchSurvivesEverySafeMutatorReport() {
  const char* commands[] = {
      "CAPTURE 1",
      "SCAN 0 1",
      "TORQUE_OFF 1",
      "TORQUE_OFF ALL",
  };
  std::uint32_t sequence = 40;
  for (const char* operation : commands) {
    RuntimeFixture fixture({});
    fixture.runtime.safety_.latchStop();
    fixture.runtime.safetyFault_ = true;
    fixture.runtime.operatorInspectionRequired_ = true;

    const std::string command = "A1 " + std::to_string(sequence) + " " +
                                operation + "\n";
    fixture.host.queue(command.c_str());
    fixture.runtime.poll();
    const std::string statusCommand =
        "A1 " + std::to_string(sequence + 1) + " STATUS\n";
    fixture.host.queue(statusCommand.c_str());
    fixture.runtime.poll();
    const std::string heartbeatCommand =
        "A1 " + std::to_string(sequence + 2) + " HEARTBEAT\n";
    fixture.host.queue(heartbeatCommand.c_str());
    fixture.runtime.poll();
    const std::string helloCommand =
        "A1 " + std::to_string(sequence + 3) + " HELLO\n";
    fixture.host.queue(helloCommand.c_str());
    fixture.runtime.poll();

    const std::string output = fixture.host.outputText();
    const std::string ok = "A1 " + std::to_string(sequence) + " OK";
    const std::string statusOk =
        "A1 " + std::to_string(sequence + 1) + " OK";
    const std::string heartbeatOk =
        "A1 " + std::to_string(sequence + 2) + " OK";
    const std::string helloOk =
        "A1 " + std::to_string(sequence + 3) + " OK";
    const std::size_t statusAt = output.find(statusOk);
    const std::size_t heartbeatAt = output.find(heartbeatOk);
    const std::size_t helloAt = output.find(helloOk);
    assert(output.find(ok) != std::string::npos);
    assert(statusAt != std::string::npos);
    assert(heartbeatAt != std::string::npos);
    assert(helloAt != std::string::npos);
    const std::string status =
        output.substr(statusAt, heartbeatAt - statusAt);
    const std::string heartbeat =
        output.substr(heartbeatAt, helloAt - heartbeatAt);
    const std::string hello = output.substr(helloAt);
    for (const std::string* report : {&status, &heartbeat, &hello}) {
      assert(report->find("\"motionState\":\"stopped\"") !=
             std::string::npos);
      assert(report->find("\"safetyFault\":true") != std::string::npos);
      assert(report->find("\"operatorInspectionRequired\":true") !=
             std::string::npos);
      assert(report->find("\"safetyStopReason\":\"MOVE_SET_FAILED\"") !=
             std::string::npos);
    }
    assert(fixture.runtime.safety_.stopped());
    assert(fixture.runtime.operatorInspectionRequired_);
    sequence += 4;
  }
}

void automaticRecoveryFaultSurvivesIncompleteProofUntilFullStatus() {
  RuntimeFixture fixture({});
  fixture.serial.setOperatingMode(0);
  fixture.serial.setTorqueEnabled(false);
  fixture.servo->operatingMode = 0;
  fixture.servo->torque = TorqueState::OFF;
  fixture.runtime.safetyFault_ = true;

  fixture.host.queue("A1 60 CAPTURE 1\n");
  fixture.runtime.poll();
  assert(fixture.host.outputText().find("A1 60 OK") != std::string::npos);
  assert(fixture.runtime.safetyFault_);
  assert(!fixture.runtime.safety_.stopped());

  fixture.host.queue("A1 61 TORQUE_OFF 1\n");
  fixture.runtime.poll();
  assert(fixture.host.outputText().find("A1 61 OK") != std::string::npos);
  assert(fixture.runtime.safetyFault_);
  assert(!fixture.runtime.safety_.stopped());

  fixture.host.queue("A1 62 SCAN 1 1\n");
  fixture.runtime.poll();
  assert(fixture.host.outputText().find("A1 62 OK") != std::string::npos);
  assert(fixture.runtime.safetyFault_);
  assert(!fixture.runtime.safety_.stopped());

  const std::size_t torqueOnWrites = fixture.serial.torqueOnWrites();
  fixture.host.queue("A1 63 HOLD_SET 2000 1\n");
  fixture.runtime.poll();
  assert(fixture.host.outputText().find("A1 63 ERR RECOVERY_PENDING") !=
         std::string::npos);
  assert(fixture.serial.torqueOnWrites() == torqueOnWrites);

  // STATUS refreshes every tracked servo and proves that each one is healthy,
  // in position mode, and electrically off. That complete proof clears the
  // automatic recovery gate without inventing an operator RESET latch.
  fixture.host.queue("A1 64 STATUS\n");
  fixture.runtime.poll();
  assert(fixture.host.outputText().find("A1 64 OK") != std::string::npos);
  assert(!fixture.runtime.safetyFault_);
  assert(!fixture.runtime.safety_.stopped());

  fixture.host.queue("A1 65 HOLD_SET 2000\n");
  fixture.runtime.poll();
  assert(fixture.host.outputText().find("A1 65 OK") != std::string::npos);
}

void scanAcceptsAConfiguredServoAtNormalTelemetryReplyLatency() {
  RuntimeFixture fixture({});
  fixture.serial.setOperatingMode(0);
  fixture.serial.setTorqueEnabled(false);
  fixture.serial.setPingReplyDelayUs(1800);
  fixture.servo->operatingMode = 0;
  fixture.servo->torque = TorqueState::OFF;

  fixture.host.queue("A1 66 SCAN 1 1\n");
  fixture.runtime.poll();

  const std::string output = fixture.host.outputText();
  assert(output.find("A1 66 OK") != std::string::npos);
  assert(output.find("\"foundIds\":[1]") != std::string::npos);
  assert(fixture.runtime.servoCount_ == 1);
  assert(fixture.runtime.servos_[0].id == kServoId);
}

void scanRetriesOneSilentPingBeforeRemovingAConfiguredServo() {
  RuntimeFixture fixture({});
  fixture.serial.setOperatingMode(0);
  fixture.serial.setTorqueEnabled(false);
  fixture.serial.dropNextPingReply();
  fixture.servo->operatingMode = 0;
  fixture.servo->torque = TorqueState::OFF;

  fixture.host.queue("A1 67 SCAN 1 1\n");
  fixture.runtime.poll();

  const std::string output = fixture.host.outputText();
  assert(output.find("A1 67 OK") != std::string::npos);
  assert(output.find("\"foundIds\":[1]") != std::string::npos);
  assert(fixture.runtime.servoCount_ == 1);
  assert(fixture.runtime.servos_[0].id == kServoId);
}

void scanFamilyReplayRecoversNegativeNativeBaseWithoutInventingTruth() {
  RuntimeFixture fixture({});
  fixture.serial.setOperatingMode(0);
  fixture.serial.setTorqueEnabled(false);
  // The servo keeps native Mode 0 in EEPROM across a HAT reboot, while the
  // HAT's multi-turn ID table is RAM-only. 0x831e is signed-magnitude -798;
  // treating it as an ordinary unsigned single-turn position yields 33566 and
  // makes STATUS silently filter an otherwise healthy Base.
  fixture.serial.setModeZeroEncoder(0x831Eu);
  fixture.runtime.bus_.setMultiTurn(kServoId, false);
  fixture.servo->operatingMode = 0;
  fixture.servo->torque = TorqueState::OFF;
  fixture.servo->odometerTracking = false;
  fixture.servo->odometerValid = false;
  armhat::forgetMultiTurnTruth(fixture.servo->multiTurnTruth);

  fixture.host.queue("A1 68 FAMILY 1 STS\n");
  fixture.runtime.poll();
  fixture.host.queue("A1 69 SCAN 1 1\n");
  fixture.runtime.poll();
  fixture.host.queue("A1 70 STATUS\n");
  fixture.runtime.poll();
  // Also exercise the Pi's bounded recovery ordering: replay the family, then
  // sample once more. FAMILY must not undo the inferred decoder identity.
  fixture.host.queue("A1 71 FAMILY 1 STS\n");
  fixture.runtime.poll();
  fixture.host.queue("A1 72 STATUS\n");
  fixture.runtime.poll();
  fixture.host.queue("A1 73 ODO_READ 1\n");
  fixture.runtime.poll();

  const std::string output = fixture.host.outputText();
  assert(output.find("A1 69 OK") != std::string::npos);
  assert(output.find("\"foundIds\":[1]") != std::string::npos);
  const std::size_t firstStatusAt = output.find("A1 70 OK");
  const std::size_t familyReplayAt = output.find("A1 71 OK");
  assert(firstStatusAt != std::string::npos);
  assert(familyReplayAt != std::string::npos);
  const std::string firstStatus =
      output.substr(firstStatusAt, familyReplayAt - firstStatusAt);
  assert(firstStatus.find("\"servos\":[{\"id\":1,\"rawPosition\":3298") !=
         std::string::npos);
  const std::size_t statusAt = output.find("A1 72 OK");
  assert(statusAt != std::string::npos);
  const std::string status = output.substr(statusAt);
  assert(status.find("\"servos\":[{\"id\":1,\"rawPosition\":3298") !=
         std::string::npos);
  assert(fixture.runtime.bus_.isMultiTurn(kServoId));
  assert(fixture.servo->online);
  assert(fixture.servo->fresh);
  assert(fixture.servo->sampled);
  assert(fixture.servo->rawPosition == 3298);
  assert(!fixture.servo->odometerTracking);
  assert(!fixture.servo->odometerValid);
  assert(!armhat::multiTurnTruthComplete(fixture.servo->multiTurnTruth));
  const std::size_t odometerAt = output.find("A1 73 OK");
  assert(odometerAt != std::string::npos);
  const std::string odometer = output.substr(odometerAt);
  assert(odometer.find("\"servoId\":1,\"tracking\":false,\"valid\":false") !=
         std::string::npos);
}

void positiveNativeModeSampleAlsoRestoresDecoderClassification() {
  RuntimeFixture fixture({});
  fixture.serial.setOperatingMode(0);
  fixture.serial.setTorqueEnabled(false);
  fixture.serial.setModeZeroEncoder(5000);
  fixture.runtime.bus_.setMultiTurn(kServoId, false);
  fixture.servo->operatingMode = 0;
  fixture.servo->torque = TorqueState::OFF;
  fixture.servo->odometerTracking = false;
  fixture.servo->odometerValid = false;
  armhat::forgetMultiTurnTruth(fixture.servo->multiTurnTruth);

  fixture.host.queue("A1 74 FAMILY 1 STS\n");
  fixture.runtime.poll();
  fixture.host.queue("A1 75 STATUS\n");
  fixture.runtime.poll();

  const std::string output = fixture.host.outputText();
  const std::size_t statusAt = output.find("A1 75 OK");
  assert(statusAt != std::string::npos);
  const std::string status = output.substr(statusAt);
  assert(status.find("\"servos\":[{\"id\":1,\"rawPosition\":904") !=
         std::string::npos);
  assert(fixture.runtime.bus_.isMultiTurn(kServoId));
  assert(fixture.servo->online);
  assert(fixture.servo->fresh);
  assert(fixture.servo->sampled);
  assert(fixture.servo->rawPosition == 904);
  assert(!fixture.servo->odometerTracking);
  assert(!fixture.servo->odometerValid);
  assert(!armhat::multiTurnTruthComplete(fixture.servo->multiTurnTruth));
}

void supervisionFailureBroadcastPrecedesAddressedTorqueOffProof() {
  RuntimeFixture fixture({});
  fixture.runtime.policy_.supervisionFaultTolerance = 0;

  fixture.runtime.tick();

  // A sustained bad servo-bus sample must still revoke authority and put the
  // torque-off packets on the wire, but it is not an operator STOP. Once the
  // bus is readable again there must be no manual-reset latch left behind.
  assert(!fixture.runtime.safety_.stopped());
  assert(fixture.serial.packetOrder().size() >= 2);
  assert(fixture.serial.packetOrder()[0] == 'B');
  assert(fixture.serial.packetOrder()[1] == 'A');

  fixture.host.queue("A1 34 STATUS\n");
  fixture.runtime.poll();
  fixture.host.queue("A1 35 HEARTBEAT\n");
  fixture.runtime.poll();
  fixture.host.queue("A1 36 HELLO\n");
  fixture.runtime.poll();
  const std::string output = fixture.host.outputText();
  const std::size_t statusAt = output.find("A1 34 OK");
  const std::size_t heartbeatAt = output.find("A1 35 OK");
  const std::size_t helloAt = output.find("A1 36 OK");
  assert(statusAt != std::string::npos);
  assert(heartbeatAt != std::string::npos);
  assert(helloAt != std::string::npos);
  const std::string status = output.substr(statusAt, heartbeatAt - statusAt);
  const std::string heartbeat = output.substr(heartbeatAt, helloAt - heartbeatAt);
  const std::string hello = output.substr(helloAt);
  for (const std::string* report : {&status, &heartbeat, &hello}) {
    assert(report->find("\"safetyFault\":false") != std::string::npos);
    assert(report->find("\"operatorInspectionRequired\":false") !=
           std::string::npos);
    assert(report->find("\"safetyStopReason\":null") !=
           std::string::npos);
  }
}

void watchdogStopBroadcastPrecedesAddressedTorqueOffProof() {
  RuntimeFixture fixture({});
  armhat_test_clock::advanceUs(
      (armhat::WATCHDOG_TIMEOUT_MS + 1u) * 1000u);

  fixture.runtime.tick();

  assert(fixture.serial.packetOrder().size() >= 2);
  assert(fixture.serial.packetOrder()[0] == 'B');
  assert(fixture.serial.packetOrder()[1] == 'A');
}

void invalidMoveSetMemberRejectsBeforeAnyServoIsStaged() {
  NullHost host;
  MoveSetServoSerial serial;
  ArmHatRuntime runtime(host, serial);
  configureMoveSetRuntime(runtime);

  // ID 4 is SCS, whose declared single-turn maximum is 1023.
  host.queue("A1 22 MOVE_SET 2,1000,200,10 4,1024,150,15\n");
  runtime.poll();

  assert(serial.stageIds().empty());
  assert(serial.syncWriteCount() == 0);
  assert(!runtime.safety_.stopped());
  assert(host.outputText().find("A1 22 ERR OUT_OF_RANGE") !=
         std::string::npos);
}

void positiveExtendedFeedbackDrivesAbsoluteTargetIdempotently() {
  RuntimeFixture fixture({});
  fixture.serial.setOperatingMode(0);
  fixture.serial.setModeZeroEncoder(6372);
  fixture.servo->operatingMode = 0;
  fixture.servo->odometerValid = true;

  armhat_test_clock::advanceUs(armhat::ODOMETER_SAMPLE_INTERVAL_MS * 1000u);
  fixture.runtime.sampleOdometers();

  assert(fixture.servo->odometerValid);
  const std::int32_t measured =
      fixture.servo->revolutions * armhat::MULTI_TURN_ENCODER_TICKS +
      fixture.servo->odometerLastRaw;
  assert(measured == 6372);

  fixture.host.queue("A1 1 MOVE 1 7000 256 12\n");
  fixture.runtime.poll();
  fixture.host.queue("A1 2 MOVE 1 7000 256 12\n");
  fixture.runtime.poll();

  assert(fixture.serial.goalWrites() == 2);
  assert(fixture.serial.commandedStep() == 7000);
}

void negativeExtendedFeedbackDrivesNegativeAbsoluteTarget() {
  RuntimeFixture fixture({});
  fixture.serial.setOperatingMode(0);
  fixture.serial.setModeZeroEncoder(
      static_cast<std::uint16_t>(5000) | armhat::ST3215_SIGN_BIT);
  fixture.servo->operatingMode = 0;
  fixture.servo->odometerValid = true;

  armhat_test_clock::advanceUs(armhat::ODOMETER_SAMPLE_INTERVAL_MS * 1000u);
  fixture.runtime.sampleOdometers();

  assert(fixture.servo->odometerValid);
  const std::int32_t measured =
      fixture.servo->revolutions * armhat::MULTI_TURN_ENCODER_TICKS +
      fixture.servo->odometerLastRaw;
  assert(measured == -5000);

  fixture.host.queue("A1 3 MOVE 1 -7000 256 12\n");
  fixture.runtime.poll();
  assert(fixture.serial.goalWrites() == 1);
  assert(fixture.serial.commandedStep() == -7000);
}

void multiTurnOnConfiguresAndVerifiesNativeAbsoluteMode() {
  RuntimeFixture fixture({});
  fixture.serial.setOperatingMode(0);
  fixture.serial.setTorqueEnabled(false);
  fixture.servo->operatingMode = 0;
  fixture.servo->torque = TorqueState::OFF;
  fixture.servo->odometerValid = false;
  armhat::forgetMultiTurnTruth(fixture.servo->multiTurnTruth);
  fixture.runtime.bus_.setMultiTurn(kServoId, false);
  assert(fixture.runtime.safety_.setHoldSet(nullptr, 0, 2000, millis()));

  fixture.host.queue("A1 4 MULTITURN 1 ON\n");
  fixture.runtime.poll();

  assert(fixture.runtime.bus_.isMultiTurn(kServoId));
  assert((fixture.serial.phase() & 0x10u) != 0);
  assert((fixture.serial.phase() & 0x03u) == 0x03u);
  assert(fixture.serial.resolution() == 1);
  assert(fixture.serial.minimum() == 0);
  assert(fixture.serial.maximum() == 0);
  assert(fixture.serial.operatingMode() == 0);
  assert(fixture.serial.locked());
  assert(!fixture.servo->odometerValid);
}

void alreadyConfiguredNativeModeDoesNotRewriteEeprom() {
  RuntimeFixture fixture({});
  fixture.serial.setNativeAbsoluteConfigured();
  fixture.serial.setTorqueEnabled(false);
  fixture.servo->operatingMode = 0;
  fixture.servo->torque = TorqueState::OFF;
  fixture.servo->odometerValid = false;
  fixture.runtime.bus_.setMultiTurn(kServoId, false);
  assert(fixture.runtime.safety_.setHoldSet(nullptr, 0, 2000, millis()));

  fixture.host.queue("A1 8 MULTITURN 1 ON\n");
  fixture.runtime.poll();

  assert(fixture.runtime.bus_.isMultiTurn(kServoId));
  assert(fixture.serial.configurationWrites() == 0);
}

void alreadyConfiguredUnlockedNativeModeOnlyRelocks() {
  RuntimeFixture fixture({});
  fixture.serial.setNativeAbsoluteConfigured();
  fixture.serial.setLocked(false);
  fixture.serial.setTorqueEnabled(false);
  fixture.servo->operatingMode = 0;
  fixture.servo->torque = TorqueState::OFF;
  fixture.servo->odometerValid = false;
  fixture.runtime.bus_.setMultiTurn(kServoId, false);
  assert(fixture.runtime.safety_.setHoldSet(nullptr, 0, 2000, millis()));

  fixture.host.queue("A1 9 MULTITURN 1 ON\n");
  fixture.runtime.poll();

  assert(fixture.runtime.bus_.isMultiTurn(kServoId));
  assert(fixture.serial.locked());
  assert(fixture.serial.configurationWrites() == 1);
}

void explicitZeroEstablishesTheCurrentNativeCoordinateAfterRestart() {
  RuntimeFixture fixture({});
  fixture.serial.setOperatingMode(0);
  fixture.serial.setTorqueEnabled(false);
  fixture.serial.setModeZeroEncoder(
      static_cast<std::uint16_t>(5000) | armhat::ST3215_SIGN_BIT);
  fixture.servo->operatingMode = 0;
  fixture.servo->torque = TorqueState::OFF;
  fixture.servo->odometerTracking = false;
  fixture.servo->odometerValid = false;
  armhat::forgetMultiTurnTruth(fixture.servo->multiTurnTruth);
  assert(fixture.runtime.safety_.setHoldSet(nullptr, 0, 2000, millis()));

  fixture.host.queue("A1 5 ODO_ZERO 1\n");
  fixture.runtime.poll();

  assert(fixture.servo->odometerTracking);
  assert(fixture.servo->odometerValid);
  const std::int32_t measured =
      fixture.servo->revolutions * armhat::MULTI_TURN_ENCODER_TICKS +
      fixture.servo->odometerLastRaw;
  assert(measured == -5000);
  assert(armhat::multiTurnTruthComplete(fixture.servo->multiTurnTruth));
  assert(fixture.servo->multiTurnTruth.referencePosition == -5000);
}

void transientMainTelemetryMissPreservesAnchorButBlocksUntilRecovery() {
  RuntimeFixture fixture({});
  fixture.serial.setOperatingMode(0);
  fixture.serial.setModeZeroEncoder(6380);
  fixture.servo->operatingMode = 0;
  fixture.servo->odometerTracking = true;
  fixture.servo->odometerValid = true;
  fixture.servo->revolutions = 1;
  fixture.servo->odometerLastRaw = 2276;
  fixture.servo->odometerSampledAtMs = millis();
  armhat::seedMultiTurnTruth(fixture.servo->multiTurnTruth, 6372);
  fixture.runtime.leaseSupervisionSeen_ = true;
  fixture.runtime.lastLeaseSupervisionMs_ = millis();
  fixture.serial.dropNextTelemetryRead();

  fixture.host.queue("A1 6 STATUS\n");
  fixture.runtime.poll();

  assert(!fixture.servo->online);
  assert(!fixture.servo->fresh);
  assert(!fixture.servo->operatingModeKnown);
  assert(fixture.servo->odometerValid);
  assert(fixture.servo->nativeSampleMissed);
  assert(fixture.servo->multiTurnTruth.referencePosition == 6372);

  // A position-only background sample cannot repair a failed complete STATUS:
  // it has no fresh Mode-0 proof. Keep the old anchor stale until the next full
  // telemetry + mode sample arrives together.
  fixture.serial.setModeZeroEncoder(6392);
  armhat_test_clock::advanceUs(armhat::ODOMETER_SAMPLE_INTERVAL_MS * 1000u);
  fixture.runtime.sampleOdometers();
  assert(!fixture.servo->operatingModeKnown);
  assert(fixture.servo->nativeSampleMissed);
  assert(fixture.servo->multiTurnTruth.referencePosition == 6372);
  fixture.serial.setModeZeroEncoder(6380);

  const std::size_t writesBeforeMove = fixture.serial.goalWrites();
  fixture.host.queue("A1 7 MOVE 1 1000 256 12\n");
  fixture.runtime.poll();
  assert(fixture.serial.goalWrites() == writesBeforeMove);

  fixture.serial.setDropReplies(true);
  fixture.host.queue("A1 8 STATUS\n");
  fixture.runtime.poll();
  fixture.host.queue("A1 9 MOVE 1 1000 256 12\n");
  fixture.runtime.poll();
  assert(fixture.serial.goalWrites() == writesBeforeMove);

  fixture.serial.setDropReplies(false);
  fixture.host.queue("A1 10 STATUS\n");
  fixture.runtime.poll();
  assert(fixture.servo->online);
  assert(fixture.servo->fresh);
  assert(fixture.servo->operatingModeKnown);
  assert(fixture.servo->operatingMode == 0);
  assert(fixture.servo->odometerValid);
  assert(!fixture.servo->nativeSampleMissed);
  assert(fixture.servo->multiTurnTruth.referencePosition == 6380);
}

void transientModeReadMissPreservesAnchorButBlocksUntilRecovery() {
  RuntimeFixture fixture({});
  fixture.serial.setOperatingMode(0);
  fixture.serial.setModeZeroEncoder(6380);
  fixture.servo->operatingMode = 0;
  fixture.servo->odometerTracking = true;
  fixture.servo->odometerValid = true;
  fixture.servo->revolutions = 1;
  fixture.servo->odometerLastRaw = 2276;
  fixture.servo->odometerSampledAtMs = millis();
  armhat::seedMultiTurnTruth(fixture.servo->multiTurnTruth, 6372);
  fixture.runtime.leaseSupervisionSeen_ = true;
  fixture.runtime.lastLeaseSupervisionMs_ = millis();
  fixture.serial.dropNextOperatingModeRead();

  fixture.host.queue("A1 11 STATUS\n");
  fixture.runtime.poll();
  assert(fixture.servo->online);
  assert(!fixture.servo->fresh);
  assert(!fixture.servo->operatingModeKnown);
  assert(fixture.servo->odometerValid);
  assert(fixture.servo->nativeSampleMissed);
  assert(fixture.servo->multiTurnTruth.referencePosition == 6372);
  const std::size_t writesBeforeMove = fixture.serial.goalWrites();
  fixture.host.queue("A1 12 MOVE 1 1000 256 12\n");
  fixture.runtime.poll();
  assert(fixture.serial.goalWrites() == writesBeforeMove);

  fixture.host.queue("A1 13 STATUS\n");
  fixture.runtime.poll();
  assert(fixture.servo->online);
  assert(fixture.servo->fresh);
  assert(fixture.servo->operatingModeKnown);
  assert(fixture.servo->operatingMode == 0);
  assert(fixture.servo->odometerValid);
  assert(!fixture.servo->nativeSampleMissed);
  assert(fixture.servo->multiTurnTruth.referencePosition == 6380);
}

void confirmedWrongNativeModeStillHardInvalidatesTheAnchor() {
  RuntimeFixture fixture({});
  fixture.serial.setOperatingMode(1);
  fixture.serial.setModeZeroEncoder(2300);
  fixture.servo->operatingMode = 0;
  fixture.servo->odometerTracking = true;
  fixture.servo->odometerValid = true;
  fixture.servo->revolutions = 1;
  fixture.servo->odometerLastRaw = 2276;
  fixture.servo->odometerSampledAtMs = millis();
  armhat::seedMultiTurnTruth(fixture.servo->multiTurnTruth, 6372);

  fixture.host.queue("A1 14 STATUS\n");
  fixture.runtime.poll();
  assert(fixture.servo->operatingModeKnown);
  assert(fixture.servo->operatingMode == 1);
  assert(!fixture.servo->odometerValid);
  assert(!armhat::multiTurnTruthComplete(fixture.servo->multiTurnTruth));
}

void suddenWholeTurnFrameLossInvalidatesButSeamsRemainContinuous() {
  RuntimeFixture sampled({});
  sampled.serial.setOperatingMode(0);
  sampled.serial.setModeZeroEncoder(6372);
  sampled.servo->operatingMode = 0;
  sampled.servo->odometerTracking = true;
  sampled.servo->odometerValid = true;
  armhat::seedMultiTurnTruth(sampled.servo->multiTurnTruth, 6372);
  sampled.servo->revolutions = 1;
  sampled.servo->odometerLastRaw = 2276;
  sampled.servo->odometerSampledAtMs = millis();

  sampled.serial.setModeZeroEncoder(2276);
  armhat_test_clock::advanceUs(armhat::ODOMETER_SAMPLE_INTERVAL_MS * 1000u);
  sampled.runtime.sampleOdometers();
  assert(!sampled.servo->odometerValid);

  RuntimeFixture refreshed({});
  refreshed.serial.setOperatingMode(0);
  refreshed.serial.setModeZeroEncoder(6372);
  refreshed.servo->operatingMode = 0;
  refreshed.servo->odometerTracking = true;
  refreshed.servo->odometerValid = true;
  armhat::seedMultiTurnTruth(refreshed.servo->multiTurnTruth, 6372);
  refreshed.servo->revolutions = 1;
  refreshed.servo->odometerLastRaw = 2276;
  refreshed.servo->odometerSampledAtMs = millis();
  refreshed.serial.setModeZeroEncoder(2276);
  refreshed.host.queue("A1 10 STATUS\n");
  refreshed.runtime.poll();
  assert(!refreshed.servo->odometerValid);

  RuntimeFixture positiveSeam({});
  positiveSeam.serial.setOperatingMode(0);
  positiveSeam.servo->operatingMode = 0;
  positiveSeam.servo->odometerTracking = true;
  positiveSeam.servo->odometerValid = true;
  armhat::seedMultiTurnTruth(positiveSeam.servo->multiTurnTruth, 4095);
  positiveSeam.servo->revolutions = 0;
  positiveSeam.servo->odometerLastRaw = 4095;
  positiveSeam.servo->odometerSampledAtMs = millis();
  positiveSeam.serial.setModeZeroEncoder(4096);
  armhat_test_clock::advanceUs(armhat::ODOMETER_SAMPLE_INTERVAL_MS * 1000u);
  positiveSeam.runtime.sampleOdometers();
  assert(positiveSeam.servo->odometerValid);
  assert(positiveSeam.servo->multiTurnTruth.referencePosition == 4096);

  RuntimeFixture negativeSeam({});
  negativeSeam.serial.setOperatingMode(0);
  negativeSeam.servo->operatingMode = 0;
  negativeSeam.servo->odometerTracking = true;
  negativeSeam.servo->odometerValid = true;
  armhat::seedMultiTurnTruth(negativeSeam.servo->multiTurnTruth, -4095);
  negativeSeam.servo->revolutions = -1;
  negativeSeam.servo->odometerLastRaw = 1;
  negativeSeam.servo->odometerSampledAtMs = millis();
  negativeSeam.serial.setModeZeroEncoder(
      static_cast<std::uint16_t>(4096) | armhat::ST3215_SIGN_BIT);
  armhat_test_clock::advanceUs(armhat::ODOMETER_SAMPLE_INTERVAL_MS * 1000u);
  negativeSeam.runtime.sampleOdometers();
  assert(negativeSeam.servo->odometerValid);
  assert(negativeSeam.servo->multiTurnTruth.referencePosition == -4096);
}

void nativeHoldCapturesAbsoluteCoordinateAndRenewalIsPure() {
  RuntimeFixture fixture({});
  fixture.serial.setOperatingMode(0);
  fixture.serial.setTorqueEnabled(false);
  fixture.serial.setModeZeroEncoder(
      static_cast<std::uint16_t>(5000) | armhat::ST3215_SIGN_BIT);
  fixture.servo->operatingMode = 0;
  fixture.servo->torque = TorqueState::OFF;
  fixture.servo->odometerTracking = true;
  fixture.servo->odometerValid = true;
  fixture.servo->revolutions = -2;
  fixture.servo->odometerLastRaw = 3192;
  fixture.servo->odometerSampledAtMs = millis();
  armhat::seedMultiTurnTruth(fixture.servo->multiTurnTruth, -5000);
  assert(fixture.runtime.safety_.setHoldSet(nullptr, 0, 2000, millis()));

  fixture.host.queue("A1 11 HOLD_SET 2000 1\n");
  fixture.runtime.poll();
  assert(fixture.serial.torqueEnabled());
  assert(fixture.serial.goalWrites() == 1);
  assert(fixture.serial.commandedStep() == -5000);
  assert(fixture.runtime.safety_.holdAuthorizedFor(kServoId, millis()));
  const std::size_t configWrites = fixture.serial.configurationWrites();

  fixture.host.queue("A1 12 HOLD_SET 2000 1\n");
  fixture.runtime.poll();
  assert(fixture.serial.goalWrites() == 1);
  assert(fixture.serial.configurationWrites() == configWrites);
  assert(fixture.runtime.safety_.holdAuthorizedFor(kServoId, millis()));
}

void missedSampleExtendsWholeTurnCollapseDetectionAcrossTheGap() {
  RuntimeFixture fixture({});
  fixture.serial.setOperatingMode(0);
  fixture.servo->operatingMode = 0;
  fixture.servo->odometerTracking = true;
  fixture.servo->odometerValid = true;
  fixture.servo->revolutions = 1;
  fixture.servo->odometerLastRaw = 2276;
  fixture.servo->odometerSampledAtMs = millis();
  armhat::seedMultiTurnTruth(fixture.servo->multiTurnTruth, 6372);

  fixture.serial.setDropReplies(true);
  armhat_test_clock::advanceUs(300u * 1000u);
  fixture.runtime.sampleOdometers();
  assert(fixture.servo->odometerValid);  // one miss is stale, not lost truth

  fixture.serial.setDropReplies(false);
  fixture.serial.setModeZeroEncoder(2276);
  armhat_test_clock::advanceUs(armhat::ODOMETER_SAMPLE_INTERVAL_MS * 1000u);
  fixture.runtime.sampleOdometers();
  assert(!fixture.servo->odometerValid);

  RuntimeFixture ordinary({});
  ordinary.serial.setOperatingMode(0);
  ordinary.servo->operatingMode = 0;
  ordinary.servo->odometerTracking = true;
  ordinary.servo->odometerValid = true;
  ordinary.servo->revolutions = 1;
  ordinary.servo->odometerLastRaw = 2276;
  ordinary.servo->odometerSampledAtMs = millis();
  armhat::seedMultiTurnTruth(ordinary.servo->multiTurnTruth, 6372);
  ordinary.serial.setDropReplies(true);
  armhat_test_clock::advanceUs(300u * 1000u);
  ordinary.runtime.sampleOdometers();
  assert(ordinary.servo->odometerValid);
  assert(ordinary.servo->nativeSampleMissed);
  ordinary.serial.setDropReplies(false);
  ordinary.serial.setModeZeroEncoder(2300);
  armhat_test_clock::advanceUs(armhat::ODOMETER_SAMPLE_INTERVAL_MS * 1000u);
  ordinary.runtime.sampleOdometers();
  assert(ordinary.servo->odometerValid);
  assert(!ordinary.servo->nativeSampleMissed);
  assert(ordinary.servo->multiTurnTruth.referencePosition == 2300);
}

void delayedCountdownBecomesLiveWithoutResync() {
  RuntimeFixture fixture({0, 0, 1365, 0});
  const std::uint64_t startedUs = armhat_test_clock::nowUs;

  fixture.runtime.stepMultiTurnGoals();

  assert(fixture.serial.commandedStep() == 6372);
  assert(fixture.serial.countdownReads() == 3);
  assert(armhat_test_clock::nowUs - startedUs ==
         3u * armhat::MULTI_TURN_COUNTDOWN_POLL_US);
  assert(fixture.servo->odometerValid);
  assert(armhat::multiTurnStepOutstanding(fixture.servo->multiTurnTruth));
  assert(armhat::multiTurnCountdownObserved(fixture.servo->multiTurnTruth));
  assert(fixture.servo->multiTurnTruth.referencePosition == 5007);
  assert(fixture.servo->multiTurnTruth.resyncCount == 0);
  assert(fixture.serial.torqueOffWrites() == 0);
  assert(fixture.runtime.safety_.holdAuthorizedFor(kServoId, millis()));

  armhat_test_clock::advanceUs(armhat::ODOMETER_SAMPLE_INTERVAL_MS * 1000u);
  fixture.runtime.sampleOdometers();
  assert(fixture.serial.countdownReads() == 4);
  assert(armhat::multiTurnTruthComplete(fixture.servo->multiTurnTruth));
  assert(fixture.servo->multiTurnTruth.referencePosition == 6372);
  assert(fixture.servo->multiTurnTruth.resyncCount == 0);
  assert(fixture.serial.torqueOffWrites() == 0);
}

void zeroThroughDeadlineForcesModeZeroReconciliation() {
  RuntimeFixture fixture({0});
  fixture.serial.setModeZeroEncoder(0);
  const std::uint64_t startedUs = armhat_test_clock::nowUs;

  fixture.runtime.stepMultiTurnGoals();

  assert(armhat_test_clock::nowUs - startedUs >=
         armhat::MULTI_TURN_COUNTDOWN_START_GRACE_MS * 1000u);
  assert(armhat_test_clock::nowUs - startedUs <=
         armhat::MULTI_TURN_COUNTDOWN_START_GRACE_MS * 1000u +
             armhat::MULTI_TURN_COUNTDOWN_POLL_US);
  assert(fixture.serial.countdownReads() > 1);
  assert(fixture.serial.torqueOffWrites() == 1);
  assert(fixture.serial.modeZeroWrites() == 1);
  assert(fixture.servo->torque == TorqueState::OFF);
  assert(fixture.servo->operatingModeKnown);
  assert(fixture.servo->operatingMode == 0);
  assert(fixture.servo->odometerValid);
  assert(armhat::multiTurnTruthComplete(fixture.servo->multiTurnTruth));
  assert(fixture.servo->multiTurnTruth.referencePosition == 0);
  assert(fixture.servo->multiTurnTruth.resyncCount == 1);
  assert(!fixture.runtime.multiTurnGoals_[0].active);
  assert(!fixture.runtime.safety_.holdAuthorizedFor(kServoId, millis()));
}

}  // namespace

int main() {
  moveSetDispatchesMixedDialectSyncWrites();
  followSetReturnsFreshFeedbackForTwoStsMembers();
  followSetRetriesOneTransientGoalMismatchThenRequiresExactReadback();
  followSetAtMaxRetriesTwoSilentGoalReadsWithoutRedispatch();
  followSetAtMaxSurvivesTwentyMillisecondPostDispatchReplyGap();
  followSetAtMaxRetriesTransientFeedbackWithoutRedispatch();
  followSetFeedbackAmbiguityStopsBothMembers();
  followSetRejectsNonStsMemberBeforeDispatch();
  followReadReturnsFreshFeedbackWithoutAnyServoWrite();
  followReadFailureDoesNotWriteGoalsOrTorque();
  followReadRetriesTransientMovingServoFeedback();
  followReadRejectsNonStsMemberBeforeBusTraffic();
  ambiguousMoveSetVerificationStopsAndLeavesNoLatentCommand();
  moveSetRetriesOneTransientGoalReadTimeoutThenRequiresExactReadback();
  addressedMoveSetOffProofSurvivesFailedBroadcastWrite();
  moveSetInspectionLatchSurvivesStopStatusAndHelloOnTheSameBoot();
  stopBroadcastPrecedesAddressedTorqueOffProof();
  explicitStopSurvivesHelloAsOperatorClearRequired();
  resetRefusesToClearStopWithoutAddressedTorqueOffProof();
  resetClearsStopAfterAddressedTorqueOffProofRecovers();
  inspectionLatchSurvivesEverySafeMutatorReport();
  automaticRecoveryFaultSurvivesIncompleteProofUntilFullStatus();
  scanAcceptsAConfiguredServoAtNormalTelemetryReplyLatency();
  scanRetriesOneSilentPingBeforeRemovingAConfiguredServo();
  scanFamilyReplayRecoversNegativeNativeBaseWithoutInventingTruth();
  positiveNativeModeSampleAlsoRestoresDecoderClassification();
  supervisionFailureBroadcastPrecedesAddressedTorqueOffProof();
  watchdogStopBroadcastPrecedesAddressedTorqueOffProof();
  invalidMoveSetMemberRejectsBeforeAnyServoIsStaged();
  positiveExtendedFeedbackDrivesAbsoluteTargetIdempotently();
  negativeExtendedFeedbackDrivesNegativeAbsoluteTarget();
  multiTurnOnConfiguresAndVerifiesNativeAbsoluteMode();
  alreadyConfiguredNativeModeDoesNotRewriteEeprom();
  alreadyConfiguredUnlockedNativeModeOnlyRelocks();
  explicitZeroEstablishesTheCurrentNativeCoordinateAfterRestart();
  transientMainTelemetryMissPreservesAnchorButBlocksUntilRecovery();
  transientModeReadMissPreservesAnchorButBlocksUntilRecovery();
  confirmedWrongNativeModeStillHardInvalidatesTheAnchor();
  suddenWholeTurnFrameLossInvalidatesButSeamsRemainContinuous();
  nativeHoldCapturesAbsoluteCoordinateAndRenewalIsPure();
  missedSampleExtendsWholeTurnCollapseDetectionAcrossTheGap();
  return 0;
}
