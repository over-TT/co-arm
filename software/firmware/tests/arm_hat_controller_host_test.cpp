#include <cassert>
#include <cstdint>
#include <cstring>

#include "ArmHatProtocol.h"

using namespace armhat;

int main() {
  {
    const uint8_t body[] = {1, 2, 1};
    assert(scsChecksum(body, sizeof(body)) == 0xFB);

    uint8_t packet[16] = {};
    const uint8_t parameters[] = {56, 2};
    const size_t length = buildScsInstruction(
        1, SCS_INSTRUCTION_READ, parameters, sizeof(parameters), packet,
        sizeof(packet));
    const uint8_t expected[] = {0xFF, 0xFF, 0x01, 0x04, 0x02,
                                0x38, 0x02, 0xBE};
    assert(length == sizeof(expected));
    assert(std::memcmp(packet, expected, sizeof(expected)) == 0);
  }

  {
    // Exact ST3215 EEPROM recovery frames for servo ID 1. These prove the
    // fixed command can only unlock, select position mode 0, and re-lock.
    uint8_t packet[16] = {};
    const uint8_t unlock[] = {0x37, 0x00};
    const uint8_t expectedUnlock[] = {0xFF, 0xFF, 0x01, 0x04, 0x03,
                                      0x37, 0x00, 0xC0};
    std::size_t length = buildScsInstruction(
        1, SCS_INSTRUCTION_WRITE, unlock, sizeof(unlock), packet,
        sizeof(packet));
    assert(length == sizeof(expectedUnlock));
    assert(std::memcmp(packet, expectedUnlock, sizeof(expectedUnlock)) == 0);

    const uint8_t positionMode[] = {0x21, 0x00};
    const uint8_t expectedPositionMode[] = {0xFF, 0xFF, 0x01, 0x04, 0x03,
                                            0x21, 0x00, 0xD6};
    length = buildScsInstruction(1, SCS_INSTRUCTION_WRITE, positionMode,
                                 sizeof(positionMode), packet, sizeof(packet));
    assert(length == sizeof(expectedPositionMode));
    assert(std::memcmp(packet, expectedPositionMode,
                       sizeof(expectedPositionMode)) == 0);

    const uint8_t lock[] = {0x37, 0x01};
    const uint8_t expectedLock[] = {0xFF, 0xFF, 0x01, 0x04, 0x03,
                                    0x37, 0x01, 0xBF};
    length = buildScsInstruction(1, SCS_INSTRUCTION_WRITE, lock, sizeof(lock),
                                 packet, sizeof(packet));
    assert(length == sizeof(expectedLock));
    assert(std::memcmp(packet, expectedLock, sizeof(expectedLock)) == 0);

    const uint8_t readMode[] = {0x21, 0x01};
    const uint8_t expectedReadMode[] = {0xFF, 0xFF, 0x01, 0x04, 0x02,
                                        0x21, 0x01, 0xD6};
    length = buildScsInstruction(1, SCS_INSTRUCTION_READ, readMode,
                                 sizeof(readMode), packet, sizeof(packet));
    assert(length == sizeof(expectedReadMode));
    assert(std::memcmp(packet, expectedReadMode, sizeof(expectedReadMode)) ==
           0);
  }

  {
    char line[] = "A1 42 PREPARE_NUDGE 3 -64 200 10";
    ParsedCommand command{};
    assert(parseCommandLine(line, std::strlen(line), command) ==
           ParseResult::OK);
    assert(command.sequence == 42);
    assert(command.operation == Operation::PREPARE_NUDGE);
    assert(command.argumentCount == 4);
    int32_t delta = 0;
    assert(parseInt32(command.arguments[1], delta));
    assert(delta == -64);
  }

  {
    char line[] = "A1 43 SET_POSITION_MODE 3 SINGLE_SERVO ST3215";
    ParsedCommand command{};
    assert(parseCommandLine(line, std::strlen(line), command) ==
           ParseResult::OK);
    assert(command.operation == Operation::SET_POSITION_MODE);
    assert(command.argumentCount == 3);
    assert(std::strcmp(command.arguments[1], "SINGLE_SERVO") == 0);
    assert(std::strcmp(command.arguments[2], "ST3215") == 0);
  }

  {
    char line[] = "A1 44 HOLD_SET 1500 1 2 3";
    ParsedCommand command{};
    assert(parseCommandLine(line, std::strlen(line), command) ==
           ParseResult::OK);
    assert(command.operation == Operation::HOLD_SET);
    assert(command.argumentCount == 4);
    assert(std::strcmp(command.arguments[0], "1500") == 0);
    assert(std::strcmp(command.arguments[3], "3") == 0);
  }

  {
    // Four coordinated goals still consume only four A1 arguments: each goal
    // is one comma-delimited token parsed by MOVE_SET, so the protocol-wide
    // MAX_ARGUMENTS remains bounded for every existing operation.
    char line[] =
        "A1 45 MOVE_SET 1,-5000,2000,40 2,1024,2000,40 "
        "3,2048,2000,40 4,512,1000,30";
    ParsedCommand command{};
    assert(parseCommandLine(line, std::strlen(line), command) ==
           ParseResult::OK);
    assert(command.operation == Operation::MOVE_SET);
    assert(command.argumentCount == 4);
    assert(std::strcmp(command.arguments[0], "1,-5000,2000,40") == 0);
    assert(std::strcmp(command.arguments[3], "4,512,1000,30") == 0);
  }

  {
    char line[] =
        "A1 46 FOLLOW_SET 2,1024,2400,80 3,2048,2400,80";
    ParsedCommand command{};
    assert(parseCommandLine(line, std::strlen(line), command) ==
           ParseResult::OK);
    assert(command.operation == Operation::FOLLOW_SET);
    assert(command.argumentCount == 2);
    assert(std::strcmp(command.arguments[0], "2,1024,2400,80") == 0);
    assert(std::strcmp(command.arguments[1], "3,2048,2400,80") == 0);
  }

  {
    char line[] = "A1 47 FOLLOW_READ 2 3";
    ParsedCommand command{};
    assert(parseCommandLine(line, std::strlen(line), command) ==
           ParseResult::OK);
    assert(command.operation == Operation::FOLLOW_READ);
    assert(command.argumentCount == 2);
    assert(std::strcmp(command.arguments[0], "2") == 0);
    assert(std::strcmp(command.arguments[1], "3") == 0);
  }

  {
    std::uint8_t packet[16] = {};
    const std::uint8_t parameters[] = {41, 7, 1, 10, 0xE8, 0x03, 0, 0, 200, 0};
    const std::size_t length = buildScsInstruction(
        0xFE, SCS_INSTRUCTION_SYNC_WRITE, parameters, sizeof(parameters),
        packet, sizeof(packet));
    const std::uint8_t expected[] = {0xFF, 0xFF, 0xFE, 0x0C, 0x83,
                                     41, 7, 1, 10, 0xE8, 0x03, 0, 0, 200, 0,
                                     0x84};
    assert(length == sizeof(expected));
    assert(std::memcmp(packet, expected, sizeof(expected)) == 0);
  }

  {
    char wrongVersion[] = "A2 1 HELLO";
    ParsedCommand command{};
    assert(parseCommandLine(wrongVersion, std::strlen(wrongVersion), command) ==
           ParseResult::BAD_VERSION);
    char badSequence[] = "A1 -1 HELLO";
    assert(parseCommandLine(badSequence, std::strlen(badSequence), command) ==
           ParseResult::BAD_SEQUENCE);

    char unknownOperation[] = "A1 9 WRITE_REGISTER 1 2 3";
    assert(parseCommandLine(unknownOperation, std::strlen(unknownOperation),
                            command) == ParseResult::UNKNOWN_OPERATION);
    char tooManyArguments[] = "A1 10 HELLO 1 2 3 4 5 6 7";
    assert(parseCommandLine(tooManyArguments,
                            std::strlen(tooManyArguments), command) ==
           ParseResult::TOO_MANY_ARGUMENTS);
  }

  assert(validServoId(0));
  assert(validServoId(253));
  assert(!validServoId(254));
  assert(validScanRange(0, 253));
  assert(!validScanRange(20, 19));
  assert(validTorqueLeaseMs(100));
  assert(validTorqueLeaseMs(2000));
  assert(!validTorqueLeaseMs(2001));
  assert(validNudge(-64, 1, 1));
  assert(validNudge(64, 256, 20));
  assert(!validNudge(0, 100, 5));
  assert(!validNudge(65, 100, 5));
  assert(!validNudge(10, 257, 5));
  assert(!validNudge(10, 100, 21));
  assert(predictedNudgeMotionMs(23, 1) == 23092);
  assert(!validNudgeTiming(23, 1));
  assert(!validNudgeTiming(23, 57));
  assert(validNudgeTiming(23, 80));
  assert(validNudgeTiming(64, 256));
  assert(PROPOSAL_TTL_MS >= 10000 && PROPOSAL_TTL_MS <= 15000);
  assert(INTERNAL_NUDGE_LEASE_MS < WATCHDOG_TIMEOUT_MS);
  assert(LEASE_SUPERVISION_INTERVAL_MS > 0 &&
         LEASE_SUPERVISION_INTERVAL_MS <= 25);
  assert(MAX_TRACKED_SERVOS == 8);
  // Four: three arm joints plus the camera joint. The Pi's MAX_HELD must never
  // exceed this -- a HOLD_SET larger than the firmware accepts is refused
  // outright, which drops torque on the whole arm.
  assert(MAX_HOLD_SERVOS == 4);
  assert(MAX_ARGUMENTS == 6);
  assert(MAX_STATUS_FIXED_JSON_BYTES +
             MAX_TRACKED_SERVOS * MAX_SERVO_JSON_BYTES +
             MAX_RESPONSE_FRAME_OVERHEAD_BYTES <=
         MAX_RESPONSE_BYTES);

  {
    // An addressed-write ACK names only a candidate endpoint. A large step is
    // not completed truth until register 56 first exposes a live countdown and
    // that same owned countdown subsequently reaches zero.
    MultiTurnTruthState truth;
    seedMultiTurnTruth(truth, 0);
    assert(multiTurnTruthComplete(truth));
    assert(beginAckedMultiTurnStep(truth, 6372));
    assert(multiTurnStepOutstanding(truth));
    assert(!multiTurnCountdownObserved(truth));
    assert(!multiTurnTruthComplete(truth));
    assert(observeMultiTurnCountdown(truth, 1365, 32) ==
           MultiTurnCountdownResult::PENDING);
    assert(truth.remaining == 1365);
    assert(truth.referencePosition == 5007);
    assert(multiTurnCountdownObserved(truth));
    assert(!multiTurnTruthComplete(truth));
    assert(observeMultiTurnCountdown(truth, 0, 32) ==
           MultiTurnCountdownResult::COMPLETE);
    assert(truth.remaining == 0);
    assert(truth.referencePosition == 6372);
    assert(multiTurnTruthComplete(truth));
  }

  {
    // Zero before any live countdown is valid only for a tiny step that could
    // finish inside the first update window. It must never bless a large ACKed
    // command as physical position.
    MultiTurnTruthState truth;
    seedMultiTurnTruth(truth, 0);
    assert(beginAckedMultiTurnStep(truth, 6372));
    assert(observeMultiTurnCountdown(truth, 0, 32) ==
           MultiTurnCountdownResult::RESYNC_REQUIRED);
    assert(!multiTurnTruthComplete(truth));
    assert(multiTurnResyncRequired(truth));
    assert(truth.referencePosition == 0);
  }

  {
    // A missing ACK is ambiguous: the command packet may have arrived while
    // only its reply was lost. No subsequent mode-3 sample may self-heal it.
    MultiTurnTruthState truth;
    seedMultiTurnTruth(truth, 5000);
    invalidateMultiTurnTruth(truth);
    assert(multiTurnResyncRequired(truth));
    assert(observeMultiTurnCountdown(truth, 0, 32) ==
           MultiTurnCountdownResult::RESYNC_REQUIRED);
    assert(multiTurnResyncRequired(truth));
    int32_t delta = 0;
    assert(!multiTurnDeltaToTarget(truth, 0, delta));
  }

  {
    // Regression for the reported false zero: authority loss with 1,365 motor
    // ticks remaining retains the 5,007 wire-supported reference. It cannot
    // commit the requested 6,372 endpoint when torque clears the countdown.
    MultiTurnTruthState truth;
    seedMultiTurnTruth(truth, 0);
    assert(beginAckedMultiTurnStep(truth, 6372));
    assert(observeMultiTurnCountdown(truth, 1365, 32) ==
           MultiTurnCountdownResult::PENDING);
    invalidateMultiTurnTruth(truth);
    assert(truth.acceptedPosition == 6372);
    assert(truth.referencePosition == 5007);
    assert(multiTurnResyncRequired(truth));
    assert(observeMultiTurnCountdown(truth, 0, 32) ==
           MultiTurnCountdownResult::RESYNC_REQUIRED);
    assert(truth.referencePosition == 5007);
  }

  {
    // The mode-0 encoder, not the command ledger, restores truth. Wrapped 911
    // nearest reference 5,000 is 5,007; the next absolute target zero must
    // therefore request -5,007, never -6,372.
    MultiTurnTruthState truth;
    seedMultiTurnTruth(truth, 5000);
    invalidateMultiTurnTruth(truth);
    int32_t actual = 0;
    assert(resyncMultiTurnTruthFromWrappedEncoder(truth, 5000, 911, actual));
    assert(actual == 5007);
    assert(truth.referencePosition == 5007);
    assert(multiTurnTruthComplete(truth));
    assert(truth.resyncCount == 1);
    int32_t delta = 0;
    assert(multiTurnDeltaToTarget(truth, 0, delta));
    assert(delta == -5007);
    assert(delta != -6372);
  }

  {
    // A valid free mode-0 sample is also physical truth. If the Base is hand
    // turned seven counts while torque is off, the exact ledger consumed by
    // the next target must advance from 5,000 to 5,007.
    MultiTurnTruthState truth;
    seedMultiTurnTruth(truth, 5000);
    int32_t delta = 0;
    assert(multiTurnDeltaToTarget(truth, 0, delta));
    assert(delta == -5000);

    const int32_t revolutions = 1;
    const uint16_t freeModeRaw = 911;
    const int32_t freeModeCounted =
        revolutions * MULTI_TURN_ENCODER_TICKS + freeModeRaw;
    seedMultiTurnTruth(truth, freeModeCounted);
    assert(truth.referencePosition == 5007);
    assert(multiTurnDeltaToTarget(truth, 0, delta));
    assert(delta == -5007);

    // Losing free-mode continuity cannot be healed by the next endpoint read.
    forgetMultiTurnTruth(truth);
    assert(!multiTurnTruthComplete(truth));
    assert(!multiTurnDeltaToTarget(truth, 0, delta));
  }

  assert(validProposalToken("p1234_ab-CD"));
  assert(!validProposalToken("p1234.bad"));
  assert(!validProposalToken(""));
  assert(decodeSignedMagnitude16(0x000A) == 10);
  assert(decodeSignedMagnitude16(0x800A) == -10);
  assert(validNudgeCompletion(2000, 2032, 2030, 32, 2));
  assert(validNudgeCompletion(2000, 1968, 1970, -32, 2));
  assert(!validNudgeCompletion(2000, 2001, 2000, 1, 2));
  assert(!validNudgeCompletion(2000, 2032, 1999, 32, 2));
  assert(!validNudgeCompletion(2000, 2032, 2029, 32, 2));
  // Motion completion is intentionally non-modular: neither encoder boundary
  // may masquerade as an adjacent position during an energized nudge.
  assert(!validNudgeCompletion(4094, 4095, 0, 1, 2));
  assert(!validNudgeCompletion(1, 0, 4095, -1, 2));
  assert(validActiveLeaseTelemetry(true, true, true, 0, 0, 0, true));
  assert(!validActiveLeaseTelemetry(false, true, true, 0, 0, 0, true));
  assert(!validActiveLeaseTelemetry(true, false, true, 0, 0, 0, true));
  assert(!validActiveLeaseTelemetry(true, true, false, 0, 0, 0, true));
  assert(!validActiveLeaseTelemetry(true, true, true, 1, 0, 0, true));
  assert(!validActiveLeaseTelemetry(true, true, true, 0, 1, 0, true));
  assert(!validActiveLeaseTelemetry(true, true, true, 0, 0, 1, true));
  assert(!validActiveLeaseTelemetry(true, true, true, 0, 0, 0, false));
  {
    const uint16_t samples[] = {4094, 4095, 0, 1, 0};
    uint16_t median = 0;
    uint16_t variation = 0;
    assert(analyzePositionSamples4096(samples, 5, median, variation));
    assert(median == 0);
    assert(variation == 3);
  }

  {
    uint8_t packet[7] = {};
    const uint8_t parameters[] = {56, 2};
    assert(buildScsInstruction(1, SCS_INSTRUCTION_READ, parameters,
                               sizeof(parameters), packet,
                               sizeof(packet)) == 0);
  }

  {
    const uint8_t existing[] = {1, 2, 9};
    const uint8_t partialFound[] = {2, 4};
    uint8_t reconciled[MAX_TRACKED_SERVOS] = {};
    std::size_t reconciledCount = 0;
    assert(reconcileCompletedScanIds(existing, 3, 0, 5, partialFound, 2,
                                     reconciled, MAX_TRACKED_SERVOS,
                                     reconciledCount));
    assert(reconciledCount == 3);
    assert(reconciled[0] == 9);  // outside the completed partial range
    assert(reconciled[1] == 2);
    assert(reconciled[2] == 4);

    // A completed full rescan with no responses removes every stale ID.
    assert(reconcileCompletedScanIds(existing, 3, 0, 253, nullptr, 0,
                                     reconciled, MAX_TRACKED_SERVOS,
                                     reconciledCount));
    assert(reconciledCount == 0);

    // A range that does not touch existing IDs preserves all of them.
    assert(reconcileCompletedScanIds(existing, 3, 20, 30, nullptr, 0,
                                     reconciled, MAX_TRACKED_SERVOS,
                                     reconciledCount));
    assert(reconciledCount == 3);
  }

  {
    // A supervised hold set authorizes up to three independent obligations.
    // Renewing the same desired set changes only its deadline; targeted
    // removal revokes one ID while the other IDs remain authorized.
    SafetyStateMachine holds;
    holds.begin(1000);
    holds.markBootTorqueOffAttempted();
    holds.heartbeat(1000);
    assert(holds.queueTorqueOffObligation(1));
    assert(holds.queueTorqueOffObligation(2));
    assert(holds.queueTorqueOffObligation(3));
    const uint8_t initial[] = {1, 2, 3};
    assert(holds.setHoldSet(initial, 3, 1000, 1000));
    assert(holds.holdSetActive(1500));
    assert(holds.holdSetCount() == 3);
    assert(holds.holdAuthorizedFor(1, 1500));
    assert(holds.holdAuthorizedFor(2, 1500));
    assert(holds.holdAuthorizedFor(3, 1500));
    assert(holds.holdSetRemainingMs(1500) == 500);

    holds.heartbeat(1500);
    assert(holds.setHoldSet(initial, 3, 1000, 1500));
    assert(holds.holdSetRemainingMs(1500) == 1000);
    assert(holds.removeHoldServo(2));
    assert(!holds.holdAuthorizedFor(2, 1500));
    assert(holds.holdAuthorizedFor(1, 1500));
    assert(holds.holdAuthorizedFor(3, 1500));
    uint8_t unauthorized = 0;
    assert(holds.nextUnauthorizedTorqueOff(1500, unauthorized));
    assert(unauthorized == 2);
    assert(holds.tick(1500) == SafetyEvent::TORQUE_OFF_RETRY);
    assert(holds.markTorqueOffConfirmed(2));
    assert(holds.tick(1501) == SafetyEvent::NONE);

    // A bounded nudge lease may coexist with holds for other joints.
    assert(holds.queueTorqueOffObligation(4));
    assert(holds.startTorqueLease(4, 500, 1501));
    assert(holds.torqueAuthorizedFor(1, 1501));
    assert(holds.torqueAuthorizedFor(3, 1501));
    assert(holds.torqueAuthorizedFor(4, 1501));
    assert(!holds.startTorqueLease(2, 500, 1501));

    assert(holds.tick(2001) == SafetyEvent::LEASE_EXPIRED);
    assert(!holds.torqueAuthorizedFor(4, 2001));
    assert(holds.holdAuthorizedFor(1, 2001));
    assert(holds.nextUnauthorizedTorqueOff(2001, unauthorized));
    assert(unauthorized == 4);
    assert(holds.markTorqueOffConfirmed(4));
    holds.heartbeat(2250);
    assert(holds.tick(2499) == SafetyEvent::NONE);
    assert(holds.tick(2500) == SafetyEvent::LEASE_EXPIRED);
    assert(!holds.holdSetActive(2500));
    assert(!holds.holdAuthorizedFor(1, 2500));
    assert(holds.torqueOffPendingFor(1));
    assert(holds.torqueOffPendingFor(3));
  }

  {
    // Transport silence revokes the whole hold set even if its own lease was
    // longer than the independent host watchdog.
    SafetyStateMachine holds;
    holds.begin(3000);
    holds.markBootTorqueOffAttempted();
    holds.heartbeat(3000);
    assert(holds.queueTorqueOffObligation(5));
    assert(holds.queueTorqueOffObligation(6));
    const uint8_t ids[] = {5, 6};
    assert(holds.setHoldSet(ids, 2, 2000, 3000));
    assert(holds.tick(3749) == SafetyEvent::NONE);
    assert(holds.tick(3750) == SafetyEvent::WATCHDOG_STOP);
    // Authority is revoked and the torque-off obligations stand -- that is the
    // part that protects the arm. It deliberately does NOT latch: measured on
    // hardware, this fires during torque-off settling whenever a heartbeat
    // queues behind a long bus read, with the bus healthy and no servo fault,
    // and the host renews its hold too often for the latch ever to be cleared.
    assert(!holds.stopped());
    assert(!holds.holdAuthorizedFor(5, 3750));
    assert(holds.torqueOffPendingFor(5));
    assert(holds.torqueOffPendingFor(6));
  }

  {
    // A latch can always be lifted by an operator who is still talking to the
    // controller, even mid-hold. Refusing while a hold set was live made STOP
    // unclearable in normal use, because the host renews its hold continuously.
    SafetyStateMachine held;
    held.begin(3000);
    held.markBootTorqueOffAttempted();
    held.heartbeat(3000);
    assert(held.queueTorqueOffObligation(5));
    const uint8_t ids[] = {5};
    assert(held.setHoldSet(ids, 1, 2000, 3000));
    held.latchStop();
    assert(held.stopped());
    held.heartbeat(3100);
    assert(held.resetInspected(3100));
    assert(!held.stopped());
    assert(!held.holdAuthorizedFor(5, 3100));
  }

  {
    SafetyStateMachine state;
    state.begin(1000);
    assert(state.bootTorqueOffRequired());
    state.markBootTorqueOffAttempted();
    assert(!state.bootTorqueOffRequired());
    assert(!state.heartbeatFresh(1000));

    state.heartbeat(1000);
    assert(state.heartbeatFresh(1749));
    assert(!state.heartbeatFresh(1750));
    assert(state.armTorqueOffObligation(3));
    assert(state.startTorqueLease(3, 2000, 1000));
    assert(state.torqueAuthorizedFor(3, 1000));
    assert(!state.torqueAuthorizedFor(4, 1000));
    assert(state.torqueLeaseActive(1749));
    const SafetyEvent event = state.tick(1750);
    assert(event == SafetyEvent::WATCHDOG_STOP);
    // Revokes authority and keeps the electrical-off obligation, but does not
    // latch: a stale heartbeat is a starved serial link far more often than it
    // is a dead host, and the arm is already de-energised either way.
    assert(!state.stopped());
    assert(!state.torqueLeaseActive(1750));
    assert(!state.torqueAuthorizedFor(3, 1750));
    assert(state.torqueOffPending());
    assert(state.torqueOffServoId() == 3);
    assert(state.torqueOffRetryDue(1750));

    // A failed write/read-back attempt must not discard the electrical-off
    // obligation. It becomes due again after the bounded retry interval.
    state.noteTorqueOffAttempt(3, 1750);
    assert(state.torqueOffPending());
    assert(!state.torqueOffRetryDue(1750 + TORQUE_OFF_RETRY_MS - 1));
    assert(state.torqueOffRetryDue(1750 + TORQUE_OFF_RETRY_MS));
    assert(!state.markTorqueOffConfirmed(4));
    assert(state.torqueOffPending());

    state.heartbeat(1760);
    // Nothing to lift: the watchdog revoked authority without latching, so
    // there is no stop to clear and reset says so rather than inventing one.
    assert(!state.resetInspected(1760));
    assert(state.markTorqueOffConfirmed(3));
    assert(!state.stopped());

    // A deliberate stop still latches, and still lifts on inspection.
    state.latchStop();
    assert(state.stopped());
    assert(state.resetInspected(1760));
    assert(!state.stopped());
  }

  {
    SafetyStateMachine state;
    state.begin(1000);
    state.markBootTorqueOffAttempted();
    state.heartbeat(1000);
    assert(state.armTorqueOffObligation(7));
    assert(state.startTorqueLease(7, 100, 1000));
    state.heartbeat(1050);
    assert(state.torqueAuthorizedFor(7, 1099));
    assert(state.tick(1099) == SafetyEvent::NONE);
    assert(state.tick(1100) == SafetyEvent::LEASE_EXPIRED);
    assert(!state.stopped());
    assert(!state.torqueLeaseActive(1100));
    assert(!state.torqueAuthorizedFor(7, 1100));
    assert(state.torqueOffPending());
    assert(state.tick(1101) == SafetyEvent::TORQUE_OFF_RETRY);
    state.noteTorqueOffAttempt(7, 1101);
    // Simulate a failed hardware off write/read-back by intentionally not
    // confirming it. The same servo remains retained for another retry.
    assert(state.torqueOffServoId() == 7);
    assert(state.torqueOffPending());
    assert(state.torqueOffRetryDue(1101 + TORQUE_OFF_RETRY_MS));
    assert(state.markTorqueOffConfirmed(7));
    assert(!state.torqueOffPending());
    assert(state.tick(1200) == SafetyEvent::NONE);
  }

  {
    // Model the EXECUTE_NUDGE torque-enable window: the off obligation exists
    // before enable, survives STOP, and survives a failed off write/read-back.
    SafetyStateMachine nudge;
    nudge.begin(2000);
    nudge.markBootTorqueOffAttempted();
    nudge.heartbeat(2000);
    assert(nudge.armTorqueOffObligation(9));
    assert(nudge.startTorqueLease(9, INTERNAL_NUDGE_LEASE_MS, 2000));
    nudge.latchStop();
    assert(nudge.torqueOffPending());
    assert(nudge.torqueOffServoId() == 9);
    nudge.noteTorqueOffAttempt(9, 2001);  // failed hardware attempt: no confirm
    assert(nudge.torqueOffPending());
    // Reset lifts the latch on the operator's assertion. It deliberately does
    // NOT cancel the electrical-off obligation -- that is the part that keeps
    // the arm safe, and it is still pursued and retried until confirmed. The
    // two used to be coupled, which meant an unconfirmable torque-off left a
    // STOP that could never be lifted.
    assert(nudge.resetInspected(2002));
    assert(nudge.torqueOffPending());
    assert(nudge.torqueOffRetryDue(2001 + TORQUE_OFF_RETRY_MS));
    assert(nudge.markTorqueOffConfirmed(9));
    assert(!nudge.torqueOffPending());
  }

  {
    // Discovery/explicit-off obligations form a bounded round-robin set. A
    // failed servo cannot erase itself or starve the other addressed IDs.
    SafetyStateMachine discovered;
    discovered.begin(3000);
    discovered.heartbeat(3000);
    assert(discovered.queueTorqueOffObligation(1));
    assert(discovered.queueTorqueOffObligation(2));
    assert(discovered.queueTorqueOffObligation(3));
    assert(discovered.queueTorqueOffObligation(2));  // idempotent
    assert(discovered.torqueOffPendingCount() == 3);
    assert(discovered.torqueOffServoId() == 1);
    discovered.noteTorqueOffAttempt(1, 3000);  // failed scan off
    assert(discovered.torqueOffServoId() == 2);
    assert(discovered.markTorqueOffConfirmed(2));
    assert(discovered.torqueOffPendingFor(1));
    assert(discovered.torqueOffPendingFor(3));
    assert(discovered.torqueOffPendingCount() == 2);
    discovered.noteTorqueOffAttempt(discovered.torqueOffServoId(), 3025);
    assert(discovered.torqueOffPendingCount() == 2);  // failed again
  }

  {
    SafetyStateMachine state;
    const uint32_t nearWrap = 0xFFFFFFF0u;
    state.begin(nearWrap);
    state.markBootTorqueOffAttempted();
    state.heartbeat(nearWrap);
    assert(state.heartbeatFresh(nearWrap + 700u));
    assert(!state.heartbeatFresh(nearWrap + WATCHDOG_TIMEOUT_MS));
  }

  return 0;
}
