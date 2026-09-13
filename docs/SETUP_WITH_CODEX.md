# Set up and run co-arm with an agent

This guide takes the repo from a clean checkout to source checks, the dashboard,
Isaac Sim, or a real Raspberry Pi arm.

To open the dashboard first, use the shorter [quickstart](QUICKSTART.md).
Return here for validation, simulation, Pi setup, or the Codex connection.

All 12 current printed parts are included. The bearing details, screw/insert
list, print settings, wiring drawing, and power details are still being added.

## Pick your path

- **Source only:** clone, install, and run the checks in section 1. Section 2
  is an optional no-gateway dashboard smoke. No
  simulator, Raspberry Pi, controller, camera, or servo is contacted.
- **Dashboard only:** complete source setup, then run section 2's local
  dashboard with no configured backend. Controls remain unavailable.
- **Simulation:** complete source setup, then follow section 3 and configure
  MCP with `ARM_EXPECTED_BACKEND=sim`.
- **Existing configured arm:** validate source first, inventory the live build,
  then use the [existing-arm short path](#existing-arm-short-path) with
  `ARM_EXPECTED_BACKEND=real`.
- **New physical build:** use the 3MF parts and assembly map, then fill in the
  bearing, fastener, print, wiring, and power details that are still marked
  TODO.

### Existing-arm short path

For an already commissioned arm, start with section 1's source checks and
section 8's tunnel, read-only status, and dashboard connection. Check that the
installed source, hardware, calibration, and camera match the build you intend
to use. Section 5 helps diagnose a connection or device fault. Section 9 adds
the optional Codex connection.

Sections 4, 6, 7, and 10 cover physical inventory, firmware, deployment, and
commissioning. Revisit the relevant stage when that part of the installation
has changed or lacks evidence; reconnecting a working arm does not by itself
require reflashing it or repeating all calibration. A lost Base reference still
requires physical alignment with its zero mark and **Set zero here**.

## What you provide

- which path you want: source, dashboard, simulation, existing arm, or new
  build;
- the real hardware details that code cannot see, such as bearing size, wiring,
  zero marks, and whether the arm is assembled;
- the physical task you want the arm to do.

## What Codex does

- reads the repo and chooses the matching path;
- runs the source checks and tells you plainly what passed;
- for a configured arm, reads the current joints/camera, shows the move, runs
  it, and checks what happened;
- asks only when a missing physical fact changes the next step or the tool
  needs an exact confirmation.

## 0. Start the agent

Open the repository root in Codex or another coding agent and use:

```text
Help me run co-arm. Read AGENTS.md and docs/SETUP_WITH_CODEX.md, then check
whether I am using source only, Isaac Sim, or the real arm. Take the matching
path and keep updates short. If the real arm is already configured, read its
current state, show the move, run it, and check the joints or camera afterward.
Treat the physical task I ask for as the go-ahead for its normal steps. Stop
only if a real fault or missing physical fact changes the job.
```

Do not paste credentials, private keys, private endpoints, unique device IDs,
or unreviewed camera frames into prompts, issues, or committed files.

## 1. Clone and validate without hardware

Requirements:

- Git;
- Python 3.11 for the validated lock; the package declares 3.11 or newer, but
  another interpreter needs its own clean verification;
- Node.js 20.19.x or 22.12 or newer (Node 21 and Node 22.0-22.11 are not
  supported by the locked Vite version);
- pnpm matching `software/dashboard/package.json`;
- Windows PowerShell 5.1 or newer and OpenSSH `ssh`/`scp` for the retained
  deployment and recovery scripts;
- a Codex CLI build exposing `plugin` and `mcp` commands when installing the
  optional repo-local plugin/MCP integration;
- a C++17 compiler for complete firmware host-test proof; without one, the
  compile portions are reported as skipped;
- Isaac Sim and Arduino tooling only for their respective later stages.

On Windows PowerShell:

```powershell
git clone https://github.com/over-TT/co-arm.git
Set-Location co-arm

python -m venv .venv
.\.venv\Scripts\python.exe -m pip install --upgrade pip
.\.venv\Scripts\python.exe -m pip install --requirement ".\software\python\requirements.lock"
.\.venv\Scripts\python.exe -m pip install --no-deps --editable ".\software\python"

.\.venv\Scripts\python.exe tools\build_source_manifest.py --check
.\.venv\Scripts\python.exe tools\check_repo.py

Set-Location software\python
..\..\.venv\Scripts\python.exe -m pytest -q -p no:cacheprovider
..\..\.venv\Scripts\python.exe -m pytest -q -p no:cacheprovider ..\firmware\tests\test_arm_hat_controller_v1.py
Set-Location ..\dashboard
pnpm install --frozen-lockfile
pnpm test
pnpm build
Set-Location ..\..
```

The validation block can be translated to POSIX by using `.venv/bin/python`.
The deployment, recovery, ACL, and tunnel examples later in this guide are
Windows/PowerShell-first. On another OS, translate the whole command, including
paths, quoting, and required privileges. Commands shown as `sh` run on the Pi.

A passing checkout proves the tested source in that host environment. It does
not prove that a Pi service is installed, firmware is flashed, a controller or
servo bus is healthy, a joint arrived, or the camera saw anything.
The firmware host test also attempts its C++17 compile checks. If no compatible
C++ compiler is installed, pytest reports those checks as skipped; record that
toolchain boundary instead of treating the skipped compile as firmware proof.

## 2. Run the dashboard with no upstream gateway

The dashboard can be served locally before SIM or REAL is configured:

```powershell
.\.venv\Scripts\python.exe -m web_backend
```

Open `http://127.0.0.1:8765/`. With no gateway pair configured, physical and
simulated arm-control routes remain unavailable. The Control Center may still
list its fixed local setup/check actions, such as starting Isaac or running
source tests, when their required local tools are present. This proves the UI
shell and local setup surface, not an arm connection.

Keep that terminal running; **Ctrl+C** stops the dashboard. If the port is
already in use, add `--port 8766` and open `http://127.0.0.1:8766/` instead.

The dashboard contains arm controls only. The supported agent integration is
the repository instructions plus the external MCP plugin; no embedded Codex
binary, login, memory, or chat runtime is bundled.

## 3. Run the Isaac Sim arm

### 3.1 Resolve the Isaac launcher

The user must provide the installed Isaac Python launcher. Record its exact
Isaac Sim version/build, install source/path, GPU/driver environment, and
whether the launcher can import the installed modules. The source uses Isaac
Sim 6 experimental articulation, stage, and RTX camera APIs; this export does
not claim that every Isaac 6 build has the same surface. Treat the smoke
self-test below as the compatibility gate for the selected installation.

### 3.2 Create ignored local tokens

From `software/`:

```powershell
$env:PYTHONPATH = "python"
..\.venv\Scripts\python.exe operations\scripts\run_arm_sim.py --ensure-tokens
$simRuntime = (Resolve-Path ".\runtime\arm-sim").Path
$account = [System.Security.Principal.WindowsIdentity]::GetCurrent().Name
icacls $simRuntime /inheritance:r /grant:r "${account}:(OI)(CI)F"
Get-ChildItem -LiteralPath $simRuntime -Filter *.token -File | ForEach-Object {
  icacls $_.FullName /inheritance:r /grant:r "${account}:(R,W)"
}
```

This writes owner-local simulator credentials under ignored
`software/runtime/arm-sim/`. It does not start Isaac or contact the physical
arm. Never commit or print those files.

Resolve the installed launcher and editor experience, export both for the
Control Center, and run the deterministic headless scene/camera artifact test
before starting the persistent stack:

```powershell
$isaacPython = "<absolute-path-to-isaac-python>"
$isaacExperience = "<absolute-path-to-isaacsim.exp.full.kit>"
$env:ISAAC_PYTHON = $isaacPython
$env:ISAAC_EXPERIENCE = $isaacExperience
& $isaacPython .\python\arm_sim\isaac\smoke_scene.py --headless
```

Passing proves that the selected Isaac installation initialized this source and
wrote the allowlisted scene, RGB render, and report under
`software/runtime/arm-sim/`. It is not persistent-bridge or physical-arm proof;
the next stage verifies the persistent bridge separately.

### 3.3 Start the persistent simulator stack

```powershell
..\.venv\Scripts\python.exe operations\scripts\run_arm_sim.py `
  --isaac-python $isaacPython
```

Add `--gui` for an interactive window. The launcher owns only the local Isaac
bridge, simulator HTTP gateway, and ignored simulator runtime files. It never
connects to the Raspberry Pi.

Expected loopback endpoints:

| Port | Purpose |
| ---: | --- |
| 8788 | simulator gateway used by dashboard/MCP |
| 8790 | authenticated Isaac bridge |
| 8791 | authenticated launcher control/stop |

### 3.4 Attach the dashboard to SIM

From the repository root in another PowerShell:

```powershell
$env:ISAAC_PYTHON = "<absolute-path-to-isaac-python>"
$env:ISAAC_EXPERIENCE = "<absolute-path-to-isaacsim.exp.full.kit>"
.\.venv\Scripts\python.exe -m web_backend `
  --sim-url http://127.0.0.1:8788 `
  --sim-token-file .\software\runtime\arm-sim\gateway.token
```

Open `http://127.0.0.1:8765/#arm/overview` and verify, in order:

1. SIM inventory and exact source/process identity;
2. arm state from the persistent Isaac instance;
3. camera status and one smoke capture;
4. one motionless plan preview;
5. only then, an explicitly selected simulated apply.

A simulated JPEG or arrival is SIM evidence only.

### 3.5 Stop SIM

From `software/`:

```powershell
..\.venv\Scripts\python.exe operations\scripts\run_arm_sim.py --stop
```

The authenticated supervisor should close ports 8788, 8790, and 8791. Do not
substitute arbitrary process termination unless the exact local process has
been identified and the supervised path is broken.

## 4. Inventory a physical build

Before Pi deployment or servo work, resolve:

- exact Pi, HAT, camera, and all four servo labels/revisions;
- logical joint-to-servo ID mapping;
- supply voltage/current, polarity, regulator path, wire ratings,
  fuse/protection, common ground, and reachable physical servo-power cut;
- Base ratio and durable zero mark;
- joint hard stops, safe inward limits, cable limits, and gravity support;
- camera/bracket orientation and measured 60/180/220 mm geometry;
- whether the published parts and wiring actually match this mechanism.

Use [ASSEMBLY.md](ASSEMBLY.md), [ELECTRONICS.md](ELECTRONICS.md), and the BOM.
If a required fact is unresolved, stop at the last non-motion stage.

## 5. Read-only Raspberry Pi discovery

Start with servo power off and gravity-loaded links supported. Use an SSH
identity and pinned known-hosts file; do not disable host-key checking.

### 5.1 Blank-Pi prerequisite gate

The deployer does not create an OS user/group or install the Raspberry Pi OS
camera stack. On a clean Pi, first confirm all of the following without
connecting the servo rail:

- SSH and SCP work through a verified host key;
- the selected service user and primary group already exist;
- that user has the reviewed non-interactive `sudo` access required by the
  deployer;
- `systemd`, `curl`, `install`, Python 3.11+, `venv`, and `pip` are available;
- the `dialout`, `video`, and `render` groups exist and the service identity has
  the access needed by the installed UART and camera devices;
- Picamera2/libcamera and `rpicam-apps` come from Raspberry Pi OS packages, not
  the project virtual environment;
- the camera is detected before the gateway service is introduced.

Useful no-write checks on the Pi are:

```sh
id
getent group dialout video render
sudo -n true
command -v systemctl curl install python3
python3 --version
python3 -m venv --help >/dev/null
python3 -c 'import picamera2; print(picamera2.__file__)'
rpicam-hello --list-cameras
```

If a prerequisite is missing, the agent must show the exact OS package,
user/group, or sudoers change and its rollback before asking for authority. Do
not hide operating-system provisioning inside the arm deployment receipt.

### 5.2 Inspect the installed Pi state

On the Pi:

```sh
uname -a
cat /etc/os-release
python3 --version
rpicam-hello --list-cameras
systemctl status arm-gateway.service --no-pager
ss -ltn
```

Then diagnose layer by layer:

1. laptop-to-Pi SSH/tunnel transport;
2. Pi gateway process;
3. Pi-to-ESP32 Arm HAT UART;
4. controller identity and STOP state;
5. servo-bus scan and IDs;
6. fresh torque/motion/position telemetry;
7. Base continuity;
8. Camera Module 3 status and capture.

`/healthz` proves only the gateway/tunnel layer. Inspect the status helper before
using it:

```powershell
.\.venv\Scripts\python.exe software\operations\scripts\arm_status.py --help
```

Running the helper without `--scan` only reads current gateway state. `--scan`
actively checks the servo bus and may recover the controller, turn torque off,
and make the Base need its physical zero mark again. Use it when a bus scan is
actually wanted, and hold a raised arm because the links can go limp.

## 6. Build the ESP32 Arm HAT firmware

The repository does not vendor Arduino CLI or the ESP32 core. Install them from
their official source. The helper records whichever Arduino CLI executable the
caller supplies and strictly requires ESP32 core `3.3.11`; it does not claim a
tested universal Arduino CLI version.

Compile to a caller-selected directory outside the source tree:

```powershell
$arduinoCli = (Resolve-Path "<absolute-path-to-arduino-cli>").Path
$firmwareBuildRoot = "<absolute-temporary-build-directory-outside-co-arm>"
.\software\operations\scripts\compile-firmware.ps1 `
  -ArduinoCli $arduinoCli `
  -BuildRoot $firmwareBuildRoot
```

The helper builds the Arm HAT target and never uploads. Record the source
commit, toolchain/core versions, artifact size, and digest.

Flashing is a separate device write. Before upload, the agent must identify the
exact board and port, exclude competing UART hosts, preserve required device
regions, show the exact upload/readback command and rollback, and receive
deliberate operator approval. This export does not guess a universal port or
flash command.

## 7. Deploy the Raspberry Pi gateway

Inspect current help:

```powershell
Get-Help .\software\operations\scripts\deploy-arm-gateway.ps1 -Detailed
```

The reviewed dormant deployment shape is:

```powershell
$sshPort = <ssh-port>
$identityFile = (Resolve-Path "<absolute-private-key-path>").Path
$knownHostsFile = (Resolve-Path "<absolute-pinned-known-hosts-path>").Path
.\software\operations\scripts\deploy-arm-gateway.ps1 `
  -HostName <pi-host-or-address> `
  -UserName <pi-service-user> `
  -ServiceGroup <pi-service-group> `
  -IdentityFile $identityFile `
  -KnownHostsFile $knownHostsFile `
  -Port $sshPort
```

The default path installs the package and loopback service but keeps physical
UART dormant. It does not flash firmware, enable torque, or command motion.
It does stop and restart the gateway service; on an already active physical
installation that can let a hold/watchdog expire and a gravity-loaded arm sag.
Support the mechanism and establish the expected service/torque state before
authorizing deployment or restart.
`-PreservePhysicalUart` is only for an already-installed, byte-identical
reviewed drop-in; it is not first-time activation.

Fresh physical-UART activation is a separate, reversible commissioning gate for
an existing prepared Pi. Use one reviewed argument set for activation and its
dormant rollback:

```powershell
$armMode = @{
  HostName = "<pi-host-or-address>"
  Port = $sshPort
  UserName = "<pi-service-user>"
  ServiceGroup = "<pi-service-group>"
  IdentityFile = $identityFile
  KnownHostsFile = $knownHostsFile
  ExpectedControllerId = "<exact-controller-id>"
  ExpectedFirmwareVersion = "<exact-firmware-version>"
}

.\software\operations\scripts\set-arm-gateway-mode.ps1 `
  @armMode -Mode Activate

# Reversible dormant rollback.
.\software\operations\scripts\set-arm-gateway-mode.ps1 `
  @armMode -Mode Deactivate
```

The operation does not configure the UART overlay or serial console. Activation
refuses until those prerequisites and `/dev/serial0` are already correct. It
verifies the exact installed base unit and staged drop-in digest, seeds or
retains the fail-closed latch, restarts deliberately, then authenticates the
expected controller/firmware, required controller capabilities, and four fresh
torque-off stationary joints with STOP and inspection latched. Deactivation
removes only that reviewed drop-in,
retains the latch, restarts dormant, and proves that no physical controller port
or unexpected drop-in remains. Already-correct modes are verified without a
restart. Neither mode reboots, deploys, clears STOP, takes torque, moves, or
uses the camera.

After deployment, verify separately:

- installed source hashes;
- service status and restart count;
- loopback-only listener;
- token ownership/mode without printing its content;
- controller link, servo bus, STOP/torque/telemetry, and camera state.

### 7.1 Keep every Pi mutation recoverable

**Device acceptance is still a release blocker.** The archive preparation,
provenance, and Base-reference guards now have local regression coverage, but
no complete replacement-Pi recovery was performed for this candidate. Treat
physical restore as supervised validation, not a proven disaster-recovery kit.
The [release review](RELEASE_REVIEW.md) records the remaining proof.

The checkout is the reusable source of truth. Do not edit an ordinary gateway
module only on the Pi. Before a reachable persistent Pi change, take a protected
snapshot when useful prior state exists; after a successful deployment,
calibration, Base-reference, service/UART/boot, or network change, always take
and verify a new one. A failed required snapshot leaves the change incomplete.
The current backup command is:

```powershell
$piBackup = @{
  HostName = "<pi-host-or-address>"
  Port = $sshPort
  UserName = "<pi-service-user>"
  ServiceGroup = "<pi-service-group>"
  IdentityFile = $identityFile
  KnownHostsFile = $knownHostsFile
  ExpectedHostName = "<exact-pi-hostname>"
  ExpectedRootDevice = "<exact-root-device>"
  ExpectedBootDevice = "<exact-boot-device>"
  DirectLanAddressCidr = "<expected-address/cidr>"
  DirectLanProfilePath = "<exact-/etc/netplan/profile.yaml>"
}
.\software\operations\scripts\backup-arm-pi.ps1 @piBackup
```

The backup freezes the deployed files and refuses to complete unless all 13
reviewed gateway modules and `software/operations/requirements-pi.txt` match
the exact SHA-256 hashes from this checkout. It also records device-specific
calibration, safety/recovery gates, services, network/SSH/VNC state, boot and
optional USB-media evidence. Archives live under the ignored, owner-only
`software/runtime/robot-gateway/pi-backups/`; they contain secrets and machine
identity and must only rebuild that same Pi. Another person's Pi starts from
the reviewed source and creates its own credentials, commissioning evidence,
and protected snapshot history.

A fresh deployment does not need old restore metadata to create a backup.
When older calibration provenance exists, backup preserves its bytes and
records whether its hash is current, stale, malformed, or absent. It does not
rewrite that history as a new physical attestation.

Prepare restore inputs locally, using the SHA-256 retained when the same Pi's
protected archive was verified. Recomputing a checksum from an untrusted file
does not authenticate its origin. This phase does not contact the Pi:

```powershell
$piRestore = @{
  HostName = "<pi-host-or-address>"
  Port = $sshPort
  UserName = "<pi-service-user>"
  ServiceGroup = "<pi-service-group>"
  IdentityFile = $identityFile
  KnownHostsFile = $knownHostsFile
  TokenFile = "<protected-token-file-for-this-pi>"
  ExpectedHostName = "<exact-pi-hostname>"
  ExpectedRootDevice = "<exact-replacement-root-device>"
  ExpectedBootDevice = "<exact-replacement-boot-device>"
  ExpectedControllerId = "<verified-controller-id>"
  ExpectedFirmwareVersion = "<verified-firmware-version>"
  DirectLanAddressCidr = "<expected-address/cidr>"
  RecoveryArchivePath = "<protected-backup.tar.gz>"
  RecoveryArchiveSha256 = "<retained-lowercase-64-character-sha256>"
  PythonExecutable = "<python-3.11-or-newer-executable>"
}
.\software\operations\scripts\restore-arm-pi.ps1 -Phase Prepare @piRestore
```

The helper reads the frozen calibration without extracting an arbitrary
archive tree. It checks archive/member integrity and the recorded hostname and service user,
then generates archive-bound provenance and an independent Base-reference
marker under the protected ignored recovery directory. Normally omit both
`CalibrationFile` and `CalibrationProvenanceFile`; if supplied, they must match
the selected archive exactly. Archived stale provenance is historical data,
not a reason to fabricate a replacement attestation.

Later phases reuse the prepared manifest and refuse changed source, token,
calibration, or marker inputs. Bootstrap installs the Base marker before the
gateway resumes. Clear STOP does not remove it: place Base on its physical
zero mark and use the explicit **Set zero here** action. Verified homing,
durable calibration readback, and marker removal are all required to unlock
motion; malformed recovery state must be repaired first.

`Verify` checks the prepared, pre-commissioning recovery state. After re-zero,
the calibration and marker deliberately change: create and verify a new
protected backup instead of expecting the old recovery manifest to match.
Archive integrity and local tests still do not establish a physical round trip.

## 8. Connect the laptop to REAL

The deployer creates or preserves the Pi token at
`/home/<pi-user>/arm-gateway/.robot-gateway.token` with mode `0600` and never
prints it. Copy it before starting the blocking tunnel, over the same pinned
SSH trust path, to an explicit location outside the repository. Direct OpenSSH
treats `UserKnownHostsFile` as a space-separated list, so this manual path must
not contain whitespace; the deployer itself has a separate safe staging
workaround for paths with spaces.

```powershell
$sshPort = <ssh-port>
$identityFile = (Resolve-Path "<absolute-private-key-path>").Path
$knownHostsFile = (Resolve-Path "<absolute-pinned-known-hosts-path-with-no-spaces>").Path
if ($knownHostsFile -match '\s') {
  throw "Use an owner-controlled pinned known-hosts path with no whitespace for direct ssh/scp."
}
$localToken = "<absolute-owner-only-path-outside-the-repository>"
scp -P $sshPort `
  -i $identityFile `
  -o "UserKnownHostsFile=$knownHostsFile" `
  -o StrictHostKeyChecking=yes `
  <pi-user>@<pi-host>:/home/<pi-user>/arm-gateway/.robot-gateway.token `
  $localToken
$account = [System.Security.Principal.WindowsIdentity]::GetCurrent().Name
icacls $localToken /inheritance:r /grant:r "${account}:(R)"
```

On POSIX, use the equivalent pinned `scp` command followed by `chmod 600`.
Verify file ownership and permissions without printing the token.

Next, open a dedicated PowerShell for the blocking loopback tunnel and leave it
running. Define the trust variables again so the command is independently
copyable:

```powershell
$sshPort = <ssh-port>
$identityFile = (Resolve-Path "<absolute-private-key-path>").Path
$knownHostsFile = (Resolve-Path "<absolute-pinned-known-hosts-path-with-no-spaces>").Path
if ($knownHostsFile -match '\s') {
  throw "Use an owner-controlled pinned known-hosts path with no whitespace for direct ssh/scp."
}
ssh -N `
  -L 8787:127.0.0.1:8787 `
  -p $sshPort `
  -i $identityFile `
  -o "UserKnownHostsFile=$knownHostsFile" `
  -o StrictHostKeyChecking=yes `
  <pi-user>@<pi-host>
```

In a third PowerShell, define the token path again, run the authenticated
read-only status first, and only then start the dashboard:

```powershell
$localToken = "<absolute-owner-only-path-outside-the-repository>"
.\.venv\Scripts\python.exe .\software\operations\scripts\arm_status.py `
  --gateway-url http://127.0.0.1:8787 `
  --token-file $localToken

.\.venv\Scripts\python.exe -m web_backend `
  --real-url http://127.0.0.1:8787 `
  --real-token-file $localToken
```

Before motion, require REAL upstream identity, fresh controller/bus state,
STOP/torque state, calibration, Base continuity, and camera state. A tunnel or
health response alone is insufficient.

## 9. Configure the external MCP agent

The Python installation provides `python -m arm_mcp`. First install the
repo-local instruction plugin from the repository root:

```powershell
codex plugin marketplace add .\software\plugin
codex plugin add arm-alliance@co-arm
```

The plugin is `software/plugin/plugins/arm-alliance/` and contains no live MCP
configuration. Configure these values in machine-local MCP configuration,
never in Git:

- `ARM_GATEWAY_URL` — SIM `http://127.0.0.1:8788` or REAL
  `http://127.0.0.1:8787`;
- `ARM_EXPECTED_BACKEND` — exactly `sim` or `real`, selected independently of
  the port so a valid token connected to the wrong gateway is refused;
- `ARM_GATEWAY_TOKEN_FILE` — absolute owner-readable token path;
- `ARM_FRAME_DIR` — ignored local retained-frame directory.

Create an ignored frame directory and register the server with the exact
project interpreter. Select the SIM or REAL URL/token file established above:

```powershell
$projectPython = (Resolve-Path .\.venv\Scripts\python.exe).Path
$frameDir = Join-Path $env:LOCALAPPDATA "co-arm\frames"
New-Item -ItemType Directory -Force -Path $frameDir | Out-Null
codex mcp add co-arm `
  --env "ARM_GATEWAY_URL=<selected-loopback-gateway-url>" `
  --env "ARM_EXPECTED_BACKEND=<sim-or-real>" `
  --env "ARM_GATEWAY_TOKEN_FILE=<absolute-owner-readable-token-file>" `
  --env "ARM_FRAME_DIR=$frameDir" `
  -- $projectPython -m arm_mcp
codex mcp get co-arm
```

`codex mcp remove co-arm` is the machine-local rollback. Never commit an
absolute interpreter, endpoint, frame path, or credential path. Start a new
Codex task after plugin/MCP changes so the client reloads the skills and tools.

Check a new agent task in this order:

1. plugin/skill is visible;
2. MCP starts from the intended interpreter;
3. `arm_state` says whether it is connected to SIM or the real arm;
4. `arm_scene` is motionless;
5. preview shows the move before apply runs it.

`arm_release` turns servo torque off. Shoulder and Elbow go limp, so hold them
if they are raised; the tool asks for `confirmed_torque_release=true`.
`arm_set_base_zero` is different: line up the physical Base mark first, then
confirm it. Turning the floor guard off also asks for confirmation. Normal
look/preview/move work should not become a repeated permission loop.

The plugin's setup skill follows this runbook and `AGENTS.md`; its runtime
skill and live MCP tools apply only after the selected gateway is configured.

## 10. Commission and run

Follow [COMMISSIONING.md](COMMISSIONING.md). The condensed order is:

1. non-motion controller handshake;
2. one isolated servo identity at a time;
3. fresh readback and torque-off proof;
4. zero, direction, ratio, and inward-limit calibration;
5. Base physical mark and native-frame truth;
6. measured geometry and floor reference;
7. small single-joint reviewed motions;
8. slow coordinated plan/apply;
9. upright Camera Module 3 capture and autofocus evidence;
10. STOP, watchdog, lease, restart, and torque-off tests;
11. bounded Live Follow only after the ordinary path passes.

Record requested, resolved, and measured angles separately. Raw servo speed is
not degrees per second. A plan receipt is not arrival proof, and arrival is not
visual or object-outcome proof.

## 11. Completion definition

For SIM, you are done when the bridge and gateway run, state and camera work, a
simulated move works, and the dashboard or Codex can connect.

For an existing arm, you are done when the Pi, controller, bus, camera, and
joints work, calibration matches the mechanism, measured moves work, and the
dashboard or Codex can run the requested task.

For a new build, finish the bearing, fastener, print, wiring, power, and assembly
photo details before treating the guide as complete.

## 12. Public handoff

Before pushing:

- run manifest, repository, Python, dashboard, firmware, and script checks for
  the exact worktree/commit;
- review current files and Git history for private data;
- preserve the explicit noncommercial license split in `LICENSING.md`;
- review third-party notices;
- exclude generated builds, runtime state, raw media, backups, tokens, private
  reports, and toolchains;
- label every claim with its actual evidence tier.

Software is available under PolyForm Noncommercial 1.0.0; designs,
documentation, and media use CC BY-NC 4.0. See [Licensing](../LICENSING.md).
