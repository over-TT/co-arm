# Arm/Isaac process bridge

This package is the narrow process boundary between one persistent Isaac Sim
Python 3.12 process and the ordinary Python 3.11 robot gateway. It is a
foundation, not a calibrated digital twin.

## Boundary

The Isaac process constructs its simulation, implements `BridgeEngine`, then
runs `LoopbackBridgeServer.serve_forever()` on `127.0.0.1`. Engine creation and
every command stay serialized on that process thread. The gateway creates one
`BridgeClient`, an `IsaacArmController`, and an `IsaacCameraProvider`, then
injects the adapters through the existing `robot_gateway.runtime.create_app`
arguments. `robot_gateway/runtime.py` does not need to import Isaac.

On a clean checkout, `software/runtime/arm-sim/` is intentionally absent. The
same Isaac process first imports `arm_sim/assets/desk_camera_arm.urdf` with
fixed provisional settings and deterministically exports the smoke scene into
that ignored runtime directory before opening it. It still creates only one
`SimulationApp`; later launches reuse the generated scene.

Use a simulator-only bearer token, port, state directory, and gateway identity.
Do not reuse the Raspberry Pi token or port. A fresh `backendInstanceId` is
created for every bridge process. The client pins it and refuses a different
instance until `reconnect()` is called explicitly.

```python
client = BridgeClient(token=sim_token, port=8790)
controller = IsaacArmController(client, mappings=simulator_joint_mappings)
camera = IsaacCameraProvider(client)
camera_service = create_isaac_camera_service(camera)

app = create_app(
    token_file=sim_gateway_token_file,
    arm_controller=controller,
    camera_service=camera_service,
    arm_state_dir=dedicated_simulator_state_dir,
)
```

Use the service factory instead of passing `camera_provider` directly. It marks
the camera evidence as simulated and binds the evidence revision to the exact
bridge capture rather than to Raspberry Pi telemetry.

The adapter mappings and the dedicated simulator `JointStore` must use the
same raw zero, encoder resolution, ratio, direction, and limits. The provided
defaults are internally consistent contract-test values only.

## Protocol

The wire format is authenticated newline-delimited JSON. Requests and results
have exact schemas. Only these commands exist:

- `health`
- `reset` with an optional bounded integer seed
- `get_state`
- `set_joint_targets` with a non-empty subset of four logical degree targets
- `capture`, which returns one digest-bound JPEG in base64

There is no command for file access, USD paths, prim lookup, Python evaluation,
shell execution, depth, segmentation, contact truth, or privileged scene state.
Requests and responses are size-bounded, connections have explicit
timeouts, the server binds only to IPv4 loopback, and simulator exceptions are
sanitized.

`InMemoryBridgeEngine` exists only for fast IPC/adapter tests. A real Isaac
engine must step PhysX, prove its terminal joint state, and return rendered JPEG
bytes inside these same five operations. Current masses, drives, materials,
backlash, compliance, optics, and timing remain provisional until the physical
calibration jobs narrow them.

## Checks

These tests do not import or launch Isaac:

```powershell
python -m unittest arm_sim.tests.test_bridge_protocol -v
python -m unittest arm_sim.tests.test_bridge_ipc -v
python -m unittest robot_gateway.tests.test_isaac_bridge -v
```

## Persistent launchers

Start the one long-lived Isaac process with Isaac's Python. The endpoint is
fixed at `127.0.0.1:8790` and the token path is mandatory:

```powershell
& '<path-to-isaac-python>' 'python\arm_sim\isaac\bridge_server.py' `
  --token-file 'runtime\arm-sim\bridge.token'
```

Then run the normal-Python gateway on fixed port `8788` with a different HTTP
token and a dedicated simulator state directory:

```powershell
$env:PYTHONPATH = "python"
python -m arm_sim.bridge.sim_gateway `
  --bridge-token-file 'runtime\arm-sim\bridge.token' `
  --gateway-token-file 'runtime\arm-sim\gateway.token' `
  --state-dir 'runtime\arm-sim\state'
```

The gateway creates `arm-joints.json` only when it is absent, labels the values
as provisional simulator mappings, and refuses to start if an existing mapping
does not match its controller adapter. It never reads or writes the real arm's
state directory.
