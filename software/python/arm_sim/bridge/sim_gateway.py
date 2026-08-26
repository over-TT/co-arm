"""Run the regular-Python simulator gateway on fixed loopback port 8788.

Use ``python -m arm_sim.bridge.sim_gateway`` with three explicit, simulator-only
paths.  This process never imports Isaac; it connects to the persistent bridge
at ``127.0.0.1:8790`` and injects the existing ArmController/CameraProvider
adapters into ``robot_gateway.runtime.create_app``.
"""

from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import tempfile

import uvicorn
from fastapi import FastAPI

from arm_sim.bridge.client import BridgeClient
from robot_gateway.isaac_bridge import (
    IsaacArmController,
    IsaacCameraProvider,
    create_isaac_camera_service,
    default_servo_mappings,
)
from robot_gateway.runtime import create_app


GATEWAY_HOST = "127.0.0.1"
GATEWAY_PORT = 8788
BRIDGE_HOST = "127.0.0.1"
BRIDGE_PORT = 8790
SIM_STATE_FILENAME = "arm-joints.json"


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run the authenticated simulator-only Arm gateway."
    )
    parser.add_argument("--bridge-token-file", required=True, type=Path)
    parser.add_argument("--gateway-token-file", required=True, type=Path)
    parser.add_argument("--state-dir", required=True, type=Path)
    return parser.parse_args()


def _read_bridge_token(path: Path) -> str:
    resolved = path.resolve()
    if not resolved.is_file():
        raise ValueError("Simulator bridge token file is unavailable.")
    try:
        with resolved.open("r", encoding="utf-8") as handle:
            token = handle.read(258).strip()
    except (OSError, UnicodeError):
        raise ValueError("Simulator bridge token file is unreadable.") from None
    if (
        not 32 <= len(token) <= 256
        or any(character.isspace() or not character.isprintable() for character in token)
    ):
        raise ValueError("Simulator bridge token is invalid.")
    return token


def _sim_joint_document() -> dict[str, object]:
    document: dict[str, object] = {
        "_meta": {
            "schema": "arm-sim-joints.v1",
            "backendId": "sim",
            "simulated": True,
            "calibrationStatus": "provisional",
            "note": "Internally consistent simulator mapping, not physical calibration.",
        }
    }
    document.update({
        mapping.joint_id: {
            "servoId": mapping.servo_id,
            "rawZero": mapping.raw_zero,
            "rawMin": mapping.raw_min,
            "rawMax": mapping.raw_max,
            "ratio": mapping.ratio,
            "direction": mapping.direction,
            "speed": 2000 if mapping.family == "STS" else 800,
            "accel": 40,
        }
        for mapping in default_servo_mappings()
    })
    return document


def _ensure_simulator_state(state_dir: Path) -> Path:
    resolved = state_dir.resolve()
    resolved.mkdir(parents=True, exist_ok=True)
    path = resolved / SIM_STATE_FILENAME
    expected = _sim_joint_document()
    if path.exists():
        try:
            current = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, UnicodeError, json.JSONDecodeError):
            raise ValueError("Simulator arm state file is unreadable.") from None
        if current != expected:
            raise ValueError(
                "Simulator arm state does not match the bridge mapping; use a dedicated empty state directory."
            )
        return resolved
    encoded = (json.dumps(expected, indent=1, sort_keys=True) + "\n").encode("utf-8")
    descriptor, temporary = tempfile.mkstemp(prefix=".arm-sim-joints-", dir=resolved)
    try:
        with os.fdopen(descriptor, "wb") as handle:
            descriptor = -1
            handle.write(encoded)
            handle.flush()
            os.fsync(handle.fileno())
        os.replace(temporary, path)
    except OSError:
        if descriptor >= 0:
            os.close(descriptor)
        try:
            Path(temporary).unlink(missing_ok=True)
        except OSError:
            pass
        raise ValueError("Simulator arm state could not be initialized.") from None
    return resolved


def _install_simulator_health(app: FastAPI, client: BridgeClient) -> None:
    """Replace the generic process probe with the bridge process identity.

    The simulator gateway is the trust boundary visible to the laptop.  Its
    health reply must therefore expose the exact Isaac process instance pinned
    by ``BridgeClient`` rather than asking the laptop to invent an identity.
    """

    app.router.routes[:] = [
        route for route in app.router.routes if getattr(route, "path", None) != "/healthz"
    ]

    async def simulator_healthz() -> dict[str, object]:
        return {
            "status": "ok",
            "backendId": "sim",
            "backendInstanceId": client.backend_instance_id,
            "simulated": True,
            "version": "arm-sim-bridge-v1",
        }

    app.add_api_route(
        "/healthz",
        simulator_healthz,
        methods=["GET"],
        include_in_schema=False,
    )


def build_app(args: argparse.Namespace):
    bridge_token_path = args.bridge_token_file.resolve()
    gateway_token_path = args.gateway_token_file.resolve()
    if bridge_token_path == gateway_token_path:
        raise ValueError("Bridge and gateway must use separate token files.")
    bridge_token = _read_bridge_token(bridge_token_path)
    state_dir = _ensure_simulator_state(args.state_dir)
    client = BridgeClient(
        token=bridge_token,
        host=BRIDGE_HOST,
        port=BRIDGE_PORT,
        connect_timeout_seconds=2.0,
        request_timeout_seconds=30.0,
    )
    # Fail before opening the HTTP gateway if the selected process cannot prove
    # the strict SIM identity. The client pins backendInstanceId here.
    client.health()
    controller = IsaacArmController(client)
    camera = IsaacCameraProvider(client)
    camera_service = create_isaac_camera_service(camera)
    app = create_app(
        token_file=gateway_token_path,
        arm_controller=controller,
        camera_service=camera_service,
        arm_state_dir=state_dir,
    )
    _install_simulator_health(app, client)
    return app


def main() -> int:
    app = build_app(_parse_args())
    uvicorn.run(
        app,
        host=GATEWAY_HOST,
        port=GATEWAY_PORT,
        log_level="info",
        access_log=False,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
