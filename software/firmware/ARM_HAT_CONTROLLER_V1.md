# ARM HAT Controller v1

Status: implemented by `firmware/esp32/arm_hat_controller`.

This is the deliberately small safety boundary between the Raspberry Pi and the
ESP32 on the serial-bus-servo HAT. It supports discovery, one-servo setup,
read-only calibration capture, short torque leases, bounded nudges, direct
held-joint moves, and additive boot-bound coordinated move sets. It does
**not** expose arbitrary register writes, trajectories, or a firmware-update
command.

Compilation is not hardware proof. Before the first live command, confirm the
servo label and HAT revision from the hardware. The checked-in register profile
is the ST3215 candidate profile; ST3215 versus a similar STS/ST3215-family model
is still a user-confirmed fact. The only EEPROM-changing operations in v1 are
guarded servo ID assignment and the fixed ST3215 position-mode restore.

## Physical transports

| Link | ESP32 UART | Settings | Pins |
| --- | --- | --- | --- |
| Raspberry Pi or USB host | `Serial` / UART0 | 115200, 8N1 | board UART0 routing |
| Servo bus | `Serial1` | 1,000,000, 8N1 | RX GPIO18, TX GPIO19 |

The HAT can route UART0 to the Raspberry Pi GPIO14/15 connection or expose it
through the ESP32 Type-C serial bridge. **Never drive UART0 from both at once.**
Type-C serial also does not replace the separately rated 12 V servo supply. Use
the HAT's intended power path and common reference; do not improvise power
back-feeding through signal or USB pins.

## Framing

The host sends one printable-ASCII line:

```text
A1 <seq> <OP> [integer-or-token arguments...]\n
```

The controller returns exactly one line with the same sequence:

```text
A1 <seq> OK <compact-json>\n
A1 <seq> ERR <STATIC_CODE> <compact-json>\n
```

- `seq` is unsigned decimal `0..4294967295`.
- Operations are uppercase and case-sensitive.
- Numeric operands are decimal. `deltaTicks`, `MOVE` goals, `MOVE_SET` goals,
  and `FOLLOW_SET` goals may be signed where the addressed servo is configured for native
  multi-turn absolute mode.
- Proposal tokens contain only `[A-Za-z0-9_-]` and are at most 31 characters.
- A request is at most 256 bytes including its final LF. A response is at most
  4096 bytes including its final LF.
- Tabs may separate request tokens. Other control bytes, non-ASCII bytes, empty
  required operands, and extra operands are rejected.
- The link is polling-only. The firmware never sends logs or unsolicited events
  on UART0. Debug text on this UART would violate the protocol.

The firmware tracks at most eight servos. Its portable compile-time bound reserves
768 bytes for fixed STATUS fields, 384 bytes per servo, and 32 bytes for the A1
frame (`3872 <= 4096`). The canonical worst-value/every-error eight-servo fixture
is checked by the host regression suite, so a valid STATUS does not degrade into
the sequence-0 overflow fallback.

If a line has no usable sequence, the error response uses sequence `0`.

## Safety state

At every boot the ESP32 initializes the 1 Mbps servo UART and sends broadcast
torque-off. A broadcast packet has no addressed status response and cannot
prove that an unknown servo received it. `HELLO` therefore reports
`bootTorqueOffSent` but honestly starts with
`torqueState:"unknown"` and motion blocked. Only addressed read-back of every
tracked/responding servo permits `torqueState:"off"`.

The host must send `HEARTBEAT` more often than every 750 ms; 250 ms is the
recommended interval while commissioning. A heartbeat age of exactly 750 ms is
stale. If it becomes stale while an energised servo still has a retained
torque-off obligation, the controller:

1. consumes any prepared nudge;
2. removes lease/hold motion authority without creating an operator STOP latch;
3. retains every affected servo ID as an electrical
   torque-off obligation; and
4. sends addressed and broadcast torque-off, then retries unconfirmed addressed
   register-40 read-backs round-robin every 25 ms.

After heartbeat and healthy telemetry recover, the Pi may issue a fresh reviewed
request. Automatic watchdog, supervision, bus, command, and grouped-dispatch
faults revoke authority, clear active goals, retain torque-off obligations, and
block new authority until a complete fresh all-tracked off proof succeeds. They
never create `stopped_latched` and never require `RESET INSPECTED`.

Explicit hold-test torque leases last 100 to 2000 ms and expire independently of
the heartbeat. While any lease is active, the ESP32 independently polls that
servo at least every 25 ms; this does not depend on the browser, Raspberry Pi, or
`STATUS` traffic. A telemetry timeout, torque not ON, unknown/non-position mode,
servo/status fault, or invalid telemetry contract immediately revokes authority
and starts addressed plus broadcast torque-off. No unverified load, current, or
undervoltage trip threshold is inferred. On normal lease expiry, that servo is
torque-disabled and read back. A read-only nudge proposal expires after 15
seconds and can be executed only once. `EXECUTE_NUDGE` creates its own 650 ms
internal lease; Prepare never energizes a servo.

The off-obligation set is separate from the eight-entry telemetry inventory and
is bounded by all 254 valid servo addresses. A discovered, captured, explicitly
targeted, ON, or UNKNOWN servo remains in that set until an addressed read of
register 40 returns zero. A failed write, timeout, STOP, scan reconciliation, or
heartbeat loss cannot erase it. `STATUS` exposes `torqueOffPending`,
`torqueOffPendingCount`, and the next round-robin `torqueOffRetryServoId`.

Explicit `STOP` is always allowed, even with a stale heartbeat. It alone latches the
stopped state and attempts torque-off. Its `OK` response may still say
`torqueState:"unknown"`; accepting the stop latch is not proof of actuator state.
Motion-capable commands remain blocked until a fresh heartbeat, physical
inspection, at least one responding tracked servo with confirmed torque-off, and
`RESET INSPECTED` all occur. A reset token is an operator assertion, not a sensor
substitute, and an empty inventory never satisfies the reset gate.

Every stopped latch is an explicit-clear boundary. `HELLO`, `STATUS`, and
`HEARTBEAT` repeat `operatorInspectionRequired:true` plus
`safetyStopReason:"EXPLICIT_STOP"` for a deliberate STOP. Only a successful
strict `RESET INSPECTED` clears it. Automatic recovery remains
`motionState:"blocked"`, reports no durable stop reason, and reopens only after
the complete fresh all-off proof. Legacy compatible firmware may report
automatic latch reasons; the Pi compatibility path may clear those only after
the same complete proof.

The reported controller state is one of:

- `disarmed`
- `lease_active`
- `nudge_prepared`
- `stopped_latched`
- `faulted`

## Commands

### `HELLO`

No arguments. Returns the exact gateway identity/summary fields plus fixed buffer
limits, UART mapping, watchdog/lease limits, the unconfirmed servo-model marker,
and the bounded capability list. Initial torque remains unknown until addressed
read-back. When any same-boot stopped latch is active, HELLO reports
`state:"stopped_latched"`, `motionState:"stopped"`, `safetyFault:true`,
`operatorInspectionRequired:true`, and the exact `safetyStopReason` above
instead of claiming a fresh disarmed state.

```text
A1 1 HELLO
```

The required payload fields are:

```json
{"controllerId":"armhat-example-controller","bootId":"boot-example-session","firmwareVersion":"arm-hat-1.0.0","protocolVersion":1,"state":"disarmed","motionState":"ready","torqueState":"unknown","busState":"unknown","bootTorqueOffSent":true,"servosState":"unknown","servoCount":null}
```

A clean controller may report `motionState:"ready"`; legacy controllers may
report `"blocked"`. HELLO proves only the controller hop. The host must remain
motion-blocked with bus and torque unknown until a later validated STATUS. A
non-latched automatic recovery reports the exact tuple `state:"faulted"`,
`motionState:"blocked"`, `safetyFault:true`,
`operatorInspectionRequired:false`, and `safetyStopReason:null`; the host may
connect so STATUS can prove automatic recovery, but must not turn that tuple
into an operator STOP latch.

`controllerId` uses the full 48-bit eFuse MAC. `bootId` is a nonzero 64-bit
nonce made from two ESP32 hardware-RNG draws, not a deterministic boot timer.
That format binds proposals/evidence to one boot; uniqueness across real power
cycles still requires hardware verification.

### `HEARTBEAT`

No arguments. Refreshes only the host watchdog. It does not enable torque, renew
a torque lease, execute a proposal, or clear a stop.

```text
A1 2 HEARTBEAT
```

### `STATUS`

No arguments. Polls all tracked IDs, then returns controller state, watchdog age,
lease/proposal state, safety flags, and the normalized servo array. IDs become
tracked after discovery or a command naming that ID. Summary fields include
`controllerId`, `bootId`, `firmwareVersion`, `protocolVersion`, `state`,
`motionState`, `torqueState`, `busState`, `servosState`, `servoCount`,
`busBaud:1000000`, `hardwareEstop:"not_detected"`, `torqueOffPending`,
`torqueOffPendingCount`, and `torqueOffRetryServoId`.

```text
A1 3 STATUS
```

### `SCAN <min> <max>`

Both IDs must be in `0..253`, with `min <= max`. Scanning first cancels any lease
or proposal and sends broadcast torque-off. Every responding ID is queued for
addressed off/read-back before it can be forgotten. At most eight discovered
servos can be represented; a larger result returns `CAPACITY`, retains all
unconfirmed off obligations, and never enables torque.

A completed range is authoritative for inventory: tracked IDs absent inside that
range are removed from telemetry, while tracked IDs outside a partial range are
preserved. Removing telemetry never removes an independent off obligation.

```text
A1 4 SCAN 0 20
```

Successful evidence has this shape:

```json
{"foundIds":[1,2,3],"completeRange":{"minId":0,"maxId":20},"collisionSuspected":false}
```

### `ASSIGN_ID <old> <new> SINGLE_SERVO`

This EEPROM-changing command requires a fresh heartbeat and a
non-stopped controller. It broadcasts torque-off, scans the complete `0..253`
range, and continues only when exactly one servo exists, it answers at `old`, and
no servo answers at `new`. The controller then unlocks EEPROM, writes register 5,
re-locks it, and verifies that `new` answers, `old` does not, and the lock reads
back enabled. Any uncertainty returns `ID_VERIFY_FAILED` and attempts to lock both
possible IDs.

Use it with one physically connected, unloaded servo only.

```text
A1 5 ASSIGN_ID 1 3 SINGLE_SERVO
```

### `SET_POSITION_MODE <id> SINGLE_SERVO ST3215`

This is a fixed recovery command, not generic mode or register access. It keeps
torque off, scans the complete `0..253` range, and continues only when the named
servo is the sole responder. It verifies the existing ST3215 position limits,
unlocks EEPROM, writes operating-mode register 33 to `0`, re-locks EEPROM, and
requires fresh read-back of mode `0`, unchanged limits, lock `1`, zero status
errors, and torque off. It is idempotent when the servo already reports mode
`0`. Any uncertain write or read-back returns `MODE_VERIFY_FAILED` and keeps
motion blocked.

```text
A1 6 SET_POSITION_MODE 3 SINGLE_SERVO ST3215
```

```json
{"servoId":3,"previousOperatingMode":1,"operatingMode":0,"verified":true,"locked":true,"torqueState":"off","minimumPosition":0,"maximumPosition":4095}
```

### `CAPTURE <id>`

Cancels motion authority, broadcasts torque-off, explicitly torque-disables the
named servo, and requires torque-off read-back. It then takes five feedback
samples. Samples are unwrapped with signed modular 4096-tick deltas, so a stable
`4095 -> 0` boundary crossing is not mistaken for a full-revolution jump. Capture
succeeds only when variation is at most eight ticks and returns the modular
median as `rawPosition` in a flat telemetry/evidence object.

This is the only place in v1 that uses modular encoder math. `PREPARE_NUDGE`
and `EXECUTE_NUDGE` use direct signed subtraction and reject a `4095 <-> 0`
observation as a boundary violation, because an energized position command must
never wrap.

This is the read-only primitive used to capture center/min/max calibration poses
after the operator moves an unpowered joint by hand.

```text
A1 6 CAPTURE 3
```

```json
{"id":3,"rawPosition":2048,"speed":0,"load":0,"voltageVolts":12.0,"temperatureC":31,"moving":false,"currentRaw":0,"torqueState":"off","packetAgeMs":0,"errors":[],"statusError":0,"online":true,"fresh":true,"operatingMode":0,"sampleCount":5,"variationTicks":2,"positionMedian":2048,"odometerValid":true,"revolutions":-2,"multiTurnPosition":-5992,"evidenceId":"obs_TEST_ONLY_EVIDENCE_1"}
```

`odometerValid` is `true` only when the odometer (below) is armed and has kept
wrap continuity for this servo. `multiTurnPosition` is then the wrap-counted
position, reconciled against the stable median so a capture taken exactly on the
`4095/0` boundary cannot be off by a whole revolution. When `odometerValid` is
`false`, `revolutions` is `0` and only `rawPosition` is meaningful.

### `ODO_ZERO <id>` and `ODO_READ <id>`

Multi-turn sensing, added in `arm-hat-1.2.0` and advertised as the
`multi_turn_sense` capability. The ST3215 encoder reports `0..4095` per motor
turn with no revolution count, so a geared joint (the Base ring gear is roughly
eight motor turns per joint turn) cannot be calibrated past half a motor turn
from raw readings alone.

`ODO_ZERO` arms wrap counting for one servo and calls its current position zero.
`ODO_READ` returns the current count. Both are read-only: neither energizes a
servo, and neither participates in the motion contract.

While a servo is armed, `loop()` polls its present position every
`ODOMETER_SAMPLE_INTERVAL_MS` (2 ms) and counts a wrap whenever consecutive
samples differ by more than 2048 ticks. The sampler runs from `loop()` and never
from `tick()`, because `tick()` also runs before every host command and must not
add bus traffic there.

`valid` drops to `false` — and stays false until the next `ODO_ZERO` — as soon as
continuity is broken: a failed or corrupt read, an out-of-range sample, or a
count beyond `ODOMETER_MAX_REVOLUTIONS` (64). A missed sample means an unknown
amount of travel happened unobserved, so the count is deliberately abandoned
rather than silently guessed.

```text
A1 7 ODO_ZERO 1
```

```json
{"servoId":1,"tracking":true,"valid":true,"revolutions":0,"rawPosition":2048,"multiTurnPosition":2048,"sampleAgeMs":0}
```

**The odometer operations are sensing only.** `PREPARE_NUDGE`/`EXECUTE_NUDGE`
remain single-turn and reject targets outside `0..4095`. Native Mode-0
multi-turn direct motion uses the separate `MOVE`/`MOVE_SET` contract below;
the nudge contract never silently widens into it.

### `MOVE <id> <goal> <speed> <accel>`

Requires a fresh heartbeat, a non-stopped controller, and current torque/hold
authority for the addressed servo. Single-turn goals use the declared family
range (`0..4095` for STS, `0..1023` for SCS). A servo explicitly configured for
`multi_turn_absolute_v1` accepts a signed absolute goal in
`-30719..30719`, but only while its Mode-0 odometer frame is valid. Speed and
acceleration remain bounded by the current runtime policy. The existing
single-servo command and response are retained unchanged for compatibility.
Compatibility does not permit guessing a dialect: a host with a locally
declared SCS ID must reject MOVE, hold/lease/nudge, capture, mode, and odometer
paths unless HELLO advertises `servo_family` and the declaration was accepted.
Legacy firmware without that capability remains usable only for STS targets;
it must never receive an SCS goal encoded through the default STS profile.

### `MOVE_SET <id,goal,speed,accel> [tuple...]`

Added by firmware `arm-hat-2.5.0` and advertised as `move_set_v1`. This
capability depends on `servo_family`; a host must reject a HELLO that advertises
`move_set_v1` without `servo_family`, because the family declaration selects
the goal address, length, and byte order. One to four comma-delimited tuple
tokens fit inside the unchanged six-argument A1 parser bound:

```text
A1 20 MOVE_SET 1,-5000,2000,40 2,1024,2000,40 3,2048,2000,40 4,512,1000,30
```

The controller parses and preflights the entire set before the first bus write:
IDs must be unique, every range/mode/odometer check must pass, the heartbeat
must be fresh, STOP must be clear, and every member must have current torque or
hold authority. It then sends one FEETECH `SYNC_WRITE` (`0x83`) packet for each
present register dialect: STS uses address 41/length 7/little endian, while SCS
uses address 42/length 6/big endian. Same-family members start from one packet.
A mixed-family request uses two sequential packets, so it is deliberately not
cross-family atomic; the packet gap is bounded by one UART send and 500 us
settle, without four Pi-to-ESP32 request/response round trips.

SYNC_WRITE has no ACK. Success is surfaced only after addressed goal
read-back verifies every tuple. The OK receipt repeats the complete controller
boot identity and exact moved list, so the Pi can bind this one command to the
same HELLO boot without a surrounding STATUS pair:

```json
{"controllerId":"armhat-example-controller","bootId":"boot-example-session","firmwareVersion":"arm-hat-2.7.0","protocolVersion":1,"dispatch":"dialect_grouped_sync_write","crossFamilyAtomic":false,"count":2,"moved":[{"servoId":2,"goal":1024,"speed":2000,"acceleration":40},{"servoId":4,"goal":512,"speed":1000,"acceleration":30}]}
```

Any ambiguous family dispatch or verification result aborts the request, clears
motion authority and active goals, and enters non-latching recovery. Firmware
immediately broadcasts torque-off to minimise coast, then torque-disables and
reads back every set member individually for proof.
There is no deferred register state and no later trigger that can revive a stale
command. The error receipt conservatively reports `motionMayHaveStarted` and
`partialDispatchPossible`; for example, STS may have started before an SCS
packet failure.

### `FOLLOW_SET <id,goal,speed,accel> <id,goal,speed,accel>`

Added by firmware `arm-hat-2.7.0` and advertised as
`follow_set_feedback_v1`. The capability depends on `servo_family`. This is a
narrow low-latency path for the two STS-family joints used by live Shoulder and
Elbow follow; it is not an unbounded or zero-delay transport.

```text
A1 21 FOLLOW_SET 2,1024,2400,50 3,2048,2400,50
```

The request must contain exactly two unique servo IDs and both IDs must resolve
to the STS dialect. Tuple syntax is identical to `MOVE_SET`. Goal, speed, and
acceleration are checked against the same current runtime policy and
single-/multi-turn position contract. The complete two-member authority,
heartbeat, mode, fault, freshness, and odometer preflight finishes before the
single STS `SYNC_WRITE` is sent.

After that one grouped write, the controller strictly reads back both goal
blocks. It then takes fresh telemetry, torque, and operating-mode samples from
both members. Success is returned only when both commanded blocks and both
fresh feedback samples are unambiguous. The receipt carries the standard
boot-bound grouped-dispatch fields plus compact feedback in request order:

Firmware `arm-hat-2.7.2` permits at most four exact goal-block read attempts,
5 ms apart, to cover the short reply-noise window observed during simultaneous
high-speed starts. Only the proof read is repeated: the preceding grouped
`SYNC_WRITE` is sent exactly once. A persistent timeout, corrupt reply, or
exact-value mismatch still enters the existing grouped fail-closed path.

```json
{"controllerId":"armhat-example-controller","bootId":"boot-example-session","firmwareVersion":"arm-hat-2.7.0","protocolVersion":1,"dispatch":"dialect_grouped_sync_write","crossFamilyAtomic":false,"count":2,"moved":[{"servoId":2,"goal":1024,"speed":2400,"acceleration":50},{"servoId":3,"goal":2048,"speed":2400,"acceleration":50}],"feedback":[{"servoId":2,"rawPosition":1018,"moving":true,"packetAgeMs":4,"voltageDeciVolts":120,"temperatureC":29},{"servoId":3,"rawPosition":2042,"moving":true,"packetAgeMs":1,"voltageDeciVolts":120,"temperatureC":30}]}
```

`feedback[].voltageDeciVolts` is the raw register-62 decivolt value, while
`packetAgeMs` is the wrap-safe age of the addressed telemetry block at receipt
construction. A dispatch, goal-verification, or feedback ambiguity returns
`MOVE_SET_FAILED` with phase `dispatch`, `verify`, or `feedback`. Every such
post-dispatch failure revokes authority, broadcasts torque-off, and performs
addressed torque-off proof for both members before reporting the result.

### `FOLLOW_READ <id> <id>`

Added by firmware `arm-hat-2.7.0` and advertised as `follow_feedback_v1`.
This is the read-only companion to `FOLLOW_SET`: it measures an unchanged
Shoulder/Elbow target while the servos continue travelling, without restarting
their acceleration ramps.

```text
A1 22 FOLLOW_READ 2 3
```

The two IDs must be unique, in range, and resolve to STS. A fresh heartbeat,
non-stopped controller, and current torque/hold authority for both IDs are
required. The controller performs addressed telemetry, torque, and
operating-mode reads for exactly those two members. The operation sends no
goal, torque, mode, or configuration write, and it never invokes the grouped
motion fail-close path. A failed sample returns `FEEDBACK_UNAVAILABLE` or
`SERVO_ERROR` with the zero-based `failedIndex`; ordinary lease/watchdog safety
continues independently.

Firmware `arm-hat-2.7.2` retains a separate compact-feedback window of at most
three attempts, 2 ms apart, for each sample. Unlike the four-read goal-block
verification window above, this never reissues a goal or torque command.
The Pi may also bridge up to six consecutive idle-only
`FEEDBACK_UNAVAILABLE` receipts while the session heartbeat and all other
continuity gates remain healthy; sustained loss still ends the session.

Success repeats the controller boot identity and returns the same compact
feedback schema as `FOLLOW_SET`, in request order:

```json
{"controllerId":"armhat-example-controller","bootId":"boot-example-session","firmwareVersion":"arm-hat-2.7.0","protocolVersion":1,"count":2,"feedback":[{"servoId":2,"rawPosition":1300,"moving":true,"packetAgeMs":4,"voltageDeciVolts":120,"temperatureC":29},{"servoId":3,"rawPosition":1900,"moving":true,"packetAgeMs":1,"voltageDeciVolts":120,"temperatureC":30}]}
```

Hosts must bind this receipt to the active HELLO boot, require exactly two rows
in request order, and reject stale, missing, duplicate, or out-of-range fields.
At the gateway this operation is intended for an active live-follow session at
no more than 20 Hz; it is not a general unsolicited telemetry stream.

### `TORQUE_LEASE <id> <leaseMs>`

Requires a fresh heartbeat and non-stopped state. `leaseMs` is `100..2000`.
Before enabling one servo, the controller broadcasts torque-off, confirms every
tracked servo is off, reads the requested servo position, writes a 1-tick/s,
acceleration-1 hold goal at that exact position, reads the goal registers back,
then enables and reads back torque. Failure at any step leaves or attempts to
leave torque off. Register 33 must read back operating mode `0` and the servo
fault byte must be zero. During the lease, the same servo is re-read every 25 ms
and any missing/invalid feedback or mode/fault change triggers non-latching
authority revocation and all-off recovery. This command exists only for the explicit short hold test; it is not part
of nudge review.

```text
A1 7 TORQUE_LEASE 3 1000
```

### `TORQUE_OFF <id|ALL>`

Always allowed. Cancels the current lease and proposal. A named ID uses an
individual write/read-back. `ALL` broadcasts torque-off and reads back all
tracked IDs. If no relevant read-back can be obtained, the result is
`TORQUE_UNCONFIRMED`, not a false success.

```text
A1 8 TORQUE_OFF ALL
```

### `PREPARE_NUDGE <id> <deltaTicks> <speed> <accel>`

Requires a fresh heartbeat and a non-stopped controller. Prepare is strictly
torque-off and read-only: it cancels any prior authority, broadcasts torque-off,
confirms every tracked servo off, explicitly confirms the named servo off, reads
fresh position/fault/mode feedback, and requires operating mode register 33 to be
known and equal to `0`. Bounds are intentionally small:

| Operand | Allowed |
| --- | --- |
| `deltaTicks` | `-64..-1` or `1..64` |
| `speed` | `1..256` raw units |
| `accel` | `1..20` raw units |
| resulting target | `0..4095` raw ticks |

The ST3215 position command stops at 0/4095; it does not safely wrap. Prepare
rejects any target outside `0..4095`, and a calibrated working range must never
cross that boundary. It also predicts travel using the documented speed scale
(`~1.004 ms per tick at raw speed 1`) and requires the predicted motion to fit a
350 ms budget. Predictable timeouts return `NUDGE_TIMING_UNSAFE` before a
proposal exists.

It does not energize or move the servo. It returns `proposalId`, `servoId`,
`startRawPosition`, `targetRawPosition`, `deltaTicks`, `speed`, `acceleration`,
and a 15,000 ms one-use review window.

```text
A1 9 PREPARE_NUDGE 3 -32 100 5
```

### `EXECUTE_NUDGE <proposalId>`

Requires the exact unexpired token and a fresh heartbeat. The proposal is
consumed before any energizing action. Execute rechecks torque-off, the prepared
start position within two direct (non-modular) ticks, fault byte zero, and
operating mode 0. It writes and reads back `goal = current`, enables and reads
back one internal 650 ms torque lease, writes and reads back the target, and polls
fresh encoder feedback for at most 450 ms. Any opposite-direction or 4095/0
boundary observation aborts the nudge. It then torque-disables that servo locally,
reads torque back off, and takes another fresh encoder sample.

`OK` is possible only when signed movement matches the proposed delta, the final
encoder is within two ticks of target, and torque reads back off. It returns the
final encoder as `rawPosition` so it shares the capture-evidence schema, plus a
boot-bound, monotonic `evidenceId`. A committed calibration profile must retain
that evidence ID; it is never optional on a successful nudge. The evidence is
self-contained:

```json
{"proposalId":"pTEST_ONLY_PROPOSAL_1","servoId":3,"startRawPosition":2048,"targetRawPosition":2071,"rawPosition":2070,"measuredDeltaTicks":22,"positionErrorTicks":-1,"torqueState":"off","completed":true,"evidenceId":"obs_TEST_ONLY_EVIDENCE_2"}
```

Any mode, encoder, bus, goal, watchdog, lease, fault, completion, or torque-off
mismatch returns `ERR`, attempts broadcast torque-off, and enters non-latching
all-off recovery.

```text
A1 10 EXECUTE_NUDGE p12ab34cd_1
```

### `STOP`

No arguments. Immediately latches stop, consumes the proposal, removes lease
authority without erasing its off obligation, sends addressed and broadcast
torque-off, and reports whether every tracked servo confirmed off. The command
itself returns `OK` because the stop latch is accepted even when electrical
read-back is unavailable; inspect `confirmed`, `torqueState`, and
`torqueOffPending`. If this command creates the latch, subsequent identity and
status receipts report `safetyStopReason:"EXPLICIT_STOP"`. If STOP arrives
during automatic recovery, the deliberate operator STOP becomes the durable
reason and still requires inspected reset.

```text
A1 11 STOP
```

### `RESET INSPECTED`

Requires a fresh heartbeat and a currently stopped controller. It sends another
broadcast torque-off, requires at least one responding tracked servo, confirms
all tracked and pending servos off with addressed read-back, and only then
clears the stop. The operator token is not electrical proof: if any off
obligation cannot be confirmed, firmware returns `RESET_PRECONDITION` with
`torqueOffConfirmed:false` and keeps STOP/fault latched. A successful receipt is
exactly disarmed evidence: `stopped:false`, `torqueState:"off"`,
`torqueOffConfirmed:true`, and `reset:true`.

The Pi keeps its inspection latch until that exact receipt and a same-boot
post-STATUS are both proven. If either reply is lost after the HAT actually
cleared its latch, a reconnect may find the same HAT boot already unlatched.
An explicit inspected retry then sends and verifies `STOP` first, followed by
the normal strict `RESET INSPECTED`; it never restores authority or sends a
motion/torque-enable command during this convergence path.

Calibration may update validated servo-family intent while stopped, but the
host sends no ordinary `FAMILY` mutation through its inspection gate. During
explicit clear it replays and exactly acknowledges every deferred family
declaration *before* `RESET INSPECTED`, while the HAT is still stopped. A
rejected or malformed family receipt therefore leaves both the HAT STOP and the
host motion gate closed; no MOVE or torque-enable command follows it.

```text
A1 12 RESET INSPECTED
```

## Servo telemetry

Every represented servo uses the same schema:

```json
{"id":3,"rawPosition":2048,"speed":0,"load":0,"voltageVolts":12.0,"temperatureC":31,"moving":false,"currentRaw":0,"torqueState":"off","packetAgeMs":0,"errors":[],"statusError":0,"online":true,"fresh":true,"operatingMode":0}
```

| Field | Meaning |
| --- | --- |
| `online` | the latest addressed transaction received a valid status packet |
| `fresh` | the latest telemetry refresh succeeded |
| `rawPosition` | registers 56-57, unsigned raw ticks |
| `speed` | registers 58-59, signed-magnitude raw speed estimate |
| `load` | registers 60-61, signed-magnitude raw load estimate |
| `voltageVolts` | register 62 converted with documented 0.1 V units |
| `temperatureC` | register 63 in documented degrees C |
| `moving` | register 66 converted to boolean |
| `currentRaw` | registers 69-70 unchanged; not advertised as milliamps |
| `torqueState` | register 40 read-back: `off`, `on`, or `unknown` |
| `packetAgeMs` | unsigned wrap-safe age of the telemetry block |
| `errors` | static fault/stale/mode/torque strings; empty is required for motion |
| `statusError` | OR of servo status-packet error bytes; motion requires zero |
| `operatingMode` | register 33 raw byte; motion requires known value `0` |

The controller never auto-writes operating mode or other model-specific EEPROM.
Register 33 changes only through the explicit fixed `SET_POSITION_MODE` command;
there is no caller-supplied mode value or raw-register route.
The raw current register remains unnormalized because no authoritative current
conversion has been established. A servo with no contract-valid sample is not
invented in the telemetry array. Previously sampled data may remain present with
`fresh:false` and an error marker; consumers must never treat it as live.

The underlying STS packet is implemented directly and bounded:

```text
FF FF ID LENGTH INSTRUCTION PARAMS... CHECKSUM
FF FF ID LENGTH ERROR PARAMS... CHECKSUM
```

`LENGTH = parameter_count + 2` and
`CHECKSUM = ~(ID + LENGTH + INSTRUCTION_OR_ERROR + PARAMS) & 0xff`.

## Static errors

| Code | Meaning |
| --- | --- |
| `LINE_TOO_LONG` | input exceeded the fixed request buffer; drained to LF |
| `BAD_FORMAT` | malformed or non-printable line |
| `BAD_VERSION` | first token was not `A1` |
| `BAD_SEQUENCE` | sequence was not an unsigned 32-bit decimal |
| `UNKNOWN_OP` | operation is not in v1 |
| `BAD_ARGS` | wrong count, token, or argument form |
| `OUT_OF_RANGE` | an otherwise numeric value exceeded an operation bound |
| `STOPPED` | stopped latch blocks the command |
| `HEARTBEAT_STALE` | motion/setup authority requires a fresh heartbeat |
| `BUS_TIMEOUT` | no complete servo response arrived in time |
| `BUS_CORRUPT` | the controller could not issue or validate a bus action |
| `SERVO_ERROR` | servo operation or goal read-back failed |
| `TORQUE_UNCONFIRMED` | torque state could not be read back as required |
| `LEASE_ACTIVE` | another torque lease is active |
| `MODE_NOT_POSITION` | register 33 was unknown or not position mode `0` |
| `PROPOSAL_MISSING` | no prepared nudge exists |
| `PROPOSAL_EXPIRED` | the 15-second review window elapsed |
| `PROPOSAL_MISMATCH` | token did not match the prepared nudge |
| `TARGET_OUT_OF_RANGE` | relative nudge would leave `0..4095` |
| `NUDGE_TIMING_UNSAFE` | speed/delta cannot fit the 350 ms planned budget |
| `NUDGE_INCOMPLETE` | encoder completion proof failed; authority was revoked and all-off recovery began |
| `MOVE_SET_FAILED` | grouped dispatch, goal read-back, or required follow feedback was ambiguous; authority was revoked, immediate broadcast-off was followed by addressed off proof, and partial-dispatch evidence is in the payload |
| `NOT_SINGLE_SERVO` | guarded ID scan did not find exactly the expected one |
| `ID_VERIFY_FAILED` | unlock/write/re-lock/new-ID verification was uncertain |
| `MODE_VERIFY_FAILED` | fixed position-mode restore could not be fully verified |
| `UNSTABLE` | five capture positions spanned more than eight ticks |
| `CAPACITY` | more than eight servos were discovered |
| `RESET_PRECONDITION` | inspection/heartbeat/torque-off reset gate failed |
| `INTERNAL` | bounded response construction failed |

## Deliberately absent from v1

- arbitrary register reads or writes;
- unsupervised arbitrary absolute position targets;
- trajectory queues or timed interpolation beyond one coordinated move set;
- continuous jogging or press-and-hold motion;
- arbitrary persistent trajectory queues;
- trajectories, IK, velocity control, or autonomous camera-driven motion;
- changing servo PID, limits, offsets, baud rate, or other model-specific EEPROM;
- flashing or self-update commands.

Those belong above this commissioning boundary and must not be added as aliases
or hidden debug commands.
