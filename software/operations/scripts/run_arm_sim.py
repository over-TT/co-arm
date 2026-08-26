"""Start and supervise the local Isaac Arm bridge plus its HTTP gateway.

This launcher owns only simulator processes and files below ``runtime/arm-sim``.
It never connects to the Raspberry Pi gateway. Separate bearer tokens protect
the Isaac IPC bridge and the simulator HTTP gateway.
"""

from __future__ import annotations

import argparse
from dataclasses import dataclass
import json
import os
from pathlib import Path
import re
import secrets
import signal
import socket
import subprocess
import sys
import threading
import time
from typing import BinaryIO, Sequence
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen


PROJECT_ROOT = Path(__file__).resolve().parents[2]
PYTHON_ROOT = PROJECT_ROOT / "python"
RUNTIME_ROOT = PROJECT_ROOT / "runtime" / "arm-sim"
STATE_ROOT = RUNTIME_ROOT / "state"
SCENE_USD = RUNTIME_ROOT / "smoke_scene.usda"
BRIDGE_TOKEN = RUNTIME_ROOT / "bridge.token"
GATEWAY_TOKEN = RUNTIME_ROOT / "gateway.token"
BRIDGE_LOG_OUT = RUNTIME_ROOT / "isaac-bridge.out.log"
BRIDGE_LOG_ERR = RUNTIME_ROOT / "isaac-bridge.err.log"
GATEWAY_LOG_OUT = RUNTIME_ROOT / "sim-gateway.out.log"
GATEWAY_LOG_ERR = RUNTIME_ROOT / "sim-gateway.err.log"
_configured_isaac_python = os.environ.get("ISAAC_PYTHON", "").strip()
ISAAC_PYTHON = Path(_configured_isaac_python) if _configured_isaac_python else None
BRIDGE_SCRIPT = PYTHON_ROOT / "arm_sim" / "isaac" / "bridge_server.py"
BRIDGE_PORT = 8790
GATEWAY_PORT = 8788
CONTROL_PORT = 8791
EXISTING_STARTUP_TIMEOUT_SECONDS = 215.0
_SOURCE_DIGEST = re.compile(r"^sha256:[0-9a-f]{64}$")


def _project_path(path: Path) -> Path:
    resolved = path.resolve()
    if not resolved.is_relative_to(PROJECT_ROOT):
        raise RuntimeError(f"Simulator path escapes the project root: {resolved}")
    return resolved


def _ensure_tokens() -> tuple[Path, Path]:
    runtime = _project_path(RUNTIME_ROOT)
    runtime.mkdir(parents=True, exist_ok=True)

    # Import after pinning the project root so this file also works when run by
    # absolute path from the Control Center launcher.
    if str(PYTHON_ROOT) not in sys.path:
        sys.path.insert(0, str(PYTHON_ROOT))
    from robot_gateway.runtime import GatewayConfigurationError, generate_token_file

    for path in (BRIDGE_TOKEN, GATEWAY_TOKEN):
        target = _project_path(path)
        if not target.exists():
            try:
                generate_token_file(target)
            except GatewayConfigurationError:
                # A second local launcher may have won the create-only race.
                # Only accept that narrow case; every other generator failure
                # must remain visible.
                if not target.is_file():
                    raise
        if not target.is_file():
            raise RuntimeError(f"Simulator token is unavailable: {target}")

    bridge_value = BRIDGE_TOKEN.read_text(encoding="utf-8").strip()
    gateway_value = GATEWAY_TOKEN.read_text(encoding="utf-8").strip()
    if bridge_value == gateway_value:
        raise RuntimeError("Bridge and gateway tokens must be different.")
    return (
        BRIDGE_TOKEN.resolve(),
        GATEWAY_TOKEN.resolve(),
    )


def _listener_ready(port: int, timeout: float = 0.25) -> bool:
    try:
        with socket.create_connection(("127.0.0.1", port), timeout=timeout):
            return True
    except OSError:
        return False


def _receive_line(connection: socket.socket, maximum_bytes: int = 4096) -> bytes:
    chunks = bytearray()
    while len(chunks) <= maximum_bytes:
        chunk = connection.recv(min(1024, maximum_bytes + 1 - len(chunks)))
        if not chunk:
            break
        chunks.extend(chunk)
        if b"\n" in chunk:
            break
    if len(chunks) > maximum_bytes or b"\n" not in chunks:
        raise ValueError("invalid control request")
    return bytes(chunks.split(b"\n", 1)[0])


def _current_source_digest() -> str:
    if str(PYTHON_ROOT) not in sys.path:
        sys.path.insert(0, str(PYTHON_ROOT))
    from arm_sim.build_identity import simulator_source_digest

    return simulator_source_digest(PROJECT_ROOT)


def _control_loop(
    *,
    token: str,
    source_digest: str,
    stop_requested: threading.Event,
    ready: threading.Event,
) -> None:
    if _SOURCE_DIGEST.fullmatch(source_digest) is None:
        raise ValueError("invalid simulator source digest")
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
        listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listener.bind(("127.0.0.1", CONTROL_PORT))
        listener.listen(4)
        listener.settimeout(0.5)
        ready.set()
        while not stop_requested.is_set():
            try:
                connection, _address = listener.accept()
            except TimeoutError:
                continue
            with connection:
                connection.settimeout(2.0)
                try:
                    request = json.loads(_receive_line(connection))
                    supplied = request.get("token") if isinstance(request, dict) else None
                    command = request.get("command") if isinstance(request, dict) else None
                    authorized = isinstance(supplied, str) and secrets.compare_digest(
                        supplied, token
                    )
                    if not authorized or command not in {"ping", "stop"}:
                        raise ValueError("invalid control request")
                    if command == "ping":
                        response = {
                            "status": "ok",
                            "backendId": "sim",
                            "simulated": True,
                            "sourceDigest": source_digest,
                            "bridgePort": BRIDGE_PORT,
                            "gatewayPort": GATEWAY_PORT,
                            "controlPort": CONTROL_PORT,
                        }
                        connection.sendall(
                            json.dumps(
                                response,
                                sort_keys=True,
                                separators=(",", ":"),
                            ).encode("utf-8")
                            + b"\n"
                        )
                    else:
                        connection.sendall(b'{"status":"stopping"}\n')
                        stop_requested.set()
                except (OSError, UnicodeError, ValueError, json.JSONDecodeError):
                    try:
                        connection.sendall(b'{"status":"rejected"}\n')
                    except OSError:
                        pass


def _request_control(
    bridge_token: Path, command: str, *, response_timeout: float = 5.0
) -> dict[str, object]:
    token = bridge_token.read_text(encoding="utf-8").strip()
    payload = json.dumps({"token": token, "command": command}, separators=(",", ":"))
    try:
        with socket.create_connection(
            ("127.0.0.1", CONTROL_PORT), timeout=2.0
        ) as connection:
            connection.settimeout(response_timeout)
            connection.sendall(payload.encode("utf-8") + b"\n")
            response = json.loads(_receive_line(connection))
    except (OSError, UnicodeError, ValueError, json.JSONDecodeError) as error:
        raise RuntimeError(
            "The authenticated simulator supervisor did not answer."
        ) from error
    if not isinstance(response, dict):
        raise RuntimeError("The simulator supervisor returned an invalid response.")
    return response


def _validate_supervisor_identity(bridge_token: Path) -> None:
    response = _request_control(bridge_token, "ping")
    expected_digest = _current_source_digest()
    expected = {
        "status": "ok",
        "backendId": "sim",
        "simulated": True,
        "sourceDigest": expected_digest,
        "bridgePort": BRIDGE_PORT,
        "gatewayPort": GATEWAY_PORT,
        "controlPort": CONTROL_PORT,
    }
    if response != expected:
        received_digest = response.get("sourceDigest")
        if (
            isinstance(received_digest, str)
            and _SOURCE_DIGEST.fullmatch(received_digest) is not None
            and received_digest != expected_digest
        ):
            raise RuntimeError(
                "The listening simulator stack was launched from stale source. Stop it, then retry."
            )
        raise RuntimeError(
            "The listening simulator supervisor did not prove its exact identity."
        )


def _request_remote_stop(bridge_token: Path) -> bool:
    if not _listener_ready(CONTROL_PORT):
        if not _listener_ready(BRIDGE_PORT) and not _listener_ready(GATEWAY_PORT):
            return False
        raise RuntimeError(
            "Simulator endpoints are listening without the authenticated local supervisor."
        )
    try:
        response = _request_control(bridge_token, "stop")
    except RuntimeError as error:
        raise RuntimeError("The simulator supervisor did not accept the stop request.") from error
    if not isinstance(response, dict) or response.get("status") != "stopping":
        raise RuntimeError("The simulator supervisor rejected the stop request.")
    deadline = time.monotonic() + 30.0
    while time.monotonic() < deadline:
        if not any(
            _listener_ready(port)
            for port in (CONTROL_PORT, BRIDGE_PORT, GATEWAY_PORT)
        ):
            return True
        time.sleep(0.25)
    raise RuntimeError("The simulator supervisor did not close its endpoints in time.")


def _validate_stack_identity(bridge_token: Path, gateway_token: Path) -> None:
    # A bridge and gateway are insufficient proof on their own: only the
    # authenticated supervisor can bind them to this launch and to the exact
    # source digest currently present on disk.
    _validate_supervisor_identity(bridge_token)
    if str(PYTHON_ROOT) not in sys.path:
        sys.path.insert(0, str(PYTHON_ROOT))
    from arm_sim.bridge.client import BridgeClient

    token = bridge_token.read_text(encoding="utf-8").strip()
    bridge_health = BridgeClient(
        token=token,
        host="127.0.0.1",
        port=BRIDGE_PORT,
        connect_timeout_seconds=2.0,
        request_timeout_seconds=5.0,
    ).health()
    bridge_instance_id = bridge_health.backend_instance_id

    gateway_secret = gateway_token.read_text(encoding="utf-8").strip()
    health_request = Request(
        f"http://127.0.0.1:{GATEWAY_PORT}/healthz",
        headers={"Authorization": f"Bearer {gateway_secret}", "Accept": "application/json"},
        method="GET",
    )
    try:
        with urlopen(health_request, timeout=5.0) as response:
            gateway_health = json.loads(response.read(1_048_577))
    except (HTTPError, URLError, OSError, ValueError) as error:
        raise RuntimeError("The listening simulator gateway did not prove its identity.") from error
    if (
        not isinstance(gateway_health, dict)
        or gateway_health.get("status") != "ok"
        or gateway_health.get("backendId") != "sim"
        or gateway_health.get("simulated") is not True
        or gateway_health.get("backendInstanceId") != bridge_instance_id
    ):
        raise RuntimeError(
            "The simulator gateway is not pinned to the listening Isaac bridge instance."
        )

    request = Request(
        f"http://127.0.0.1:{GATEWAY_PORT}/api/camera/status",
        headers={"Authorization": f"Bearer {gateway_secret}", "Accept": "application/json"},
        method="GET",
    )
    try:
        with urlopen(request, timeout=5.0) as response:
            document = json.loads(response.read(1_048_577))
    except (HTTPError, URLError, OSError, ValueError) as error:
        raise RuntimeError("The listening simulator gateway did not prove its identity.") from error
    if str(PYTHON_ROOT) not in sys.path:
        sys.path.insert(0, str(PYTHON_ROOT))
    from arm_sim.camera_profile import load_camera_profile

    camera_profile = load_camera_profile()
    if (
        not isinstance(document, dict)
        or document.get("simulated") is not True
        or document.get("cameraId") != camera_profile.camera_id
        or document.get("sensorModel") != camera_profile.sensor_model
        or document.get("identityConfidence") != camera_profile.identity_confidence
    ):
        raise RuntimeError("The listening gateway is not the expected Isaac camera backend.")


def _wait_for_existing_stack(
    bridge_token: Path,
    gateway_token: Path,
    *,
    timeout: float = EXISTING_STARTUP_TIMEOUT_SECONDS,
) -> None:
    """Wait for an authenticated, current-source supervisor already starting.

    The supervisor opens its control endpoint before the comparatively slow
    Isaac bridge and then the HTTP gateway. A concurrent launcher may therefore
    observe zero or one data endpoint without that being stale. The
    authenticated control endpoint is the ownership proof that makes waiting
    safe; endpoint-only partial stacks still fail closed in ``_run``.
    """

    _validate_supervisor_identity(bridge_token)
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        bridge_ready = _listener_ready(BRIDGE_PORT)
        gateway_ready = _listener_ready(GATEWAY_PORT)
        if bridge_ready and gateway_ready:
            # Revalidates the supervisor/source digest and pins the gateway to
            # the exact bridge process before accepting the existing launch.
            _validate_stack_identity(bridge_token, gateway_token)
            return
        if not _listener_ready(CONTROL_PORT):
            raise RuntimeError(
                "The authenticated simulator supervisor exited during startup."
            )
        time.sleep(0.25)
    raise RuntimeError(
        "The authenticated simulator supervisor did not finish opening both "
        f"endpoints within {timeout:.0f} seconds."
    )


@dataclass
class _OwnedProcess:
    name: str
    process: subprocess.Popen[bytes]
    stdout: BinaryIO
    stderr: BinaryIO

    def close_logs(self) -> None:
        self.stdout.close()
        self.stderr.close()


def _hidden_flags() -> int:
    if sys.platform != "win32":
        return 0
    return int(getattr(subprocess, "CREATE_NO_WINDOW", 0)) | int(
        getattr(subprocess, "CREATE_NEW_PROCESS_GROUP", 0)
    )


def _start_process(
    name: str,
    arguments: Sequence[str],
    stdout_path: Path,
    stderr_path: Path,
) -> _OwnedProcess:
    stdout = _project_path(stdout_path).open("ab", buffering=0)
    stderr = _project_path(stderr_path).open("ab", buffering=0)
    try:
        process = subprocess.Popen(
            list(arguments),
            cwd=str(PROJECT_ROOT),
            shell=False,
            stdin=subprocess.DEVNULL,
            stdout=stdout,
            stderr=stderr,
            close_fds=True,
            creationflags=_hidden_flags(),
        )
    except BaseException:
        stdout.close()
        stderr.close()
        raise
    return _OwnedProcess(name=name, process=process, stdout=stdout, stderr=stderr)


def _wait_for_listener(owned: _OwnedProcess, port: int, timeout: float) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        exit_code = owned.process.poll()
        if exit_code is not None:
            raise RuntimeError(f"{owned.name} exited with code {exit_code}; see runtime/arm-sim logs.")
        if _listener_ready(port):
            return
        time.sleep(0.25)
    raise RuntimeError(f"{owned.name} did not open 127.0.0.1:{port} within {timeout:.0f} seconds.")


def _terminate_owned(owned: _OwnedProcess) -> None:
    process = owned.process
    if process.poll() is None:
        if sys.platform == "win32":
            # The Isaac entrypoint is a batch file.  Terminate only the exact
            # process tree this launcher created so its Python child is not
            # orphaned when the supervisor stops.
            try:
                subprocess.run(
                    ["taskkill.exe", "/PID", str(process.pid), "/T", "/F"],
                    check=False,
                    stdin=subprocess.DEVNULL,
                    stdout=subprocess.DEVNULL,
                    stderr=subprocess.DEVNULL,
                    timeout=10,
                    shell=False,
                )
            except (OSError, subprocess.TimeoutExpired):
                process.terminate()
        else:
            process.terminate()
        try:
            process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5)
    owned.close_logs()


def _bridge_command(
    bridge_token: Path,
    *,
    gui: bool,
    isaac_python: Path | None = None,
) -> tuple[str, ...]:
    launcher = isaac_python or ISAAC_PYTHON
    if launcher is None:
        raise RuntimeError(
            "Isaac Sim Python is not configured; pass --isaac-python or set ISAAC_PYTHON."
        )
    command = (
        str(launcher),
        str(BRIDGE_SCRIPT),
        "--token-file",
        str(bridge_token),
        "--scene",
        str(_project_path(SCENE_USD)),
    )
    if gui:
        return (*command, "--no-headless")
    return command


def _run(*, gui: bool = False, isaac_python: Path | None = None) -> int:
    bridge_token, gateway_token = _ensure_tokens()
    source_digest = _current_source_digest()
    _project_path(STATE_ROOT).mkdir(parents=True, exist_ok=True)
    launcher = isaac_python or ISAAC_PYTHON
    if launcher is None:
        raise RuntimeError(
            "Isaac Sim Python is not configured; pass --isaac-python or set ISAAC_PYTHON."
        )
    launcher = launcher.expanduser().resolve()
    if not launcher.is_file():
        raise RuntimeError(f"Isaac Sim Python is unavailable: {launcher}")
    if not BRIDGE_SCRIPT.is_file():
        raise RuntimeError(f"Isaac bridge entrypoint is unavailable: {BRIDGE_SCRIPT}")

    control_ready = _listener_ready(CONTROL_PORT)
    bridge_ready = _listener_ready(BRIDGE_PORT)
    gateway_ready = _listener_ready(GATEWAY_PORT)
    if control_ready:
        _wait_for_existing_stack(bridge_token, gateway_token)
        print("Arm simulator is already listening on 127.0.0.1:8790 and :8788.")
        return 0
    if bridge_ready and gateway_ready:
        _validate_stack_identity(bridge_token, gateway_token)
        print("Arm simulator is already listening on 127.0.0.1:8790 and :8788.")
        return 0
    if bridge_ready or gateway_ready:
        raise RuntimeError(
            "Only one Arm simulator endpoint is listening. Stop the stale local simulator process, then retry."
        )

    stop_requested = threading.Event()
    control_bound = threading.Event()
    control_error: list[BaseException] = []

    def request_stop(_signum: int, _frame: object) -> None:
        stop_requested.set()

    for signum in (signal.SIGINT, signal.SIGTERM):
        signal.signal(signum, request_stop)

    owned: list[_OwnedProcess] = []
    try:
        def run_control() -> None:
            try:
                _control_loop(
                    token=bridge_token.read_text(encoding="utf-8").strip(),
                    source_digest=source_digest,
                    stop_requested=stop_requested,
                    ready=control_bound,
                )
            except BaseException as error:
                control_error.append(error)
                control_bound.set()

        control_thread = threading.Thread(
            target=run_control,
            name="arm-sim-supervisor-control",
            daemon=True,
        )
        control_thread.start()
        if not control_bound.wait(2.0):
            raise RuntimeError(
                f"Simulator supervisor could not bind 127.0.0.1:{CONTROL_PORT}."
            )
        if control_error:
            # Another launcher may have won the bind race after our preflight.
            # Accept it only through the same authenticated/current-source
            # startup wait used above; never infer ownership from a port alone.
            if _listener_ready(CONTROL_PORT):
                _wait_for_existing_stack(bridge_token, gateway_token)
                print(
                    "Arm simulator is already listening on "
                    "127.0.0.1:8790 and :8788."
                )
                return 0
            raise RuntimeError(
                f"Simulator supervisor could not bind 127.0.0.1:{CONTROL_PORT}."
            ) from control_error[0]
        bridge = _start_process(
            "Isaac bridge",
            _bridge_command(
                bridge_token,
                gui=gui,
                isaac_python=launcher,
            ),
            BRIDGE_LOG_OUT,
            BRIDGE_LOG_ERR,
        )
        owned.append(bridge)
        _wait_for_listener(bridge, BRIDGE_PORT, timeout=180)

        gateway = _start_process(
            "Simulator HTTP gateway",
            (
                sys.executable,
                "-m",
                "arm_sim.bridge.sim_gateway",
                "--bridge-token-file",
                str(bridge_token),
                "--gateway-token-file",
                str(gateway_token),
                "--state-dir",
                str(STATE_ROOT.resolve()),
            ),
            GATEWAY_LOG_OUT,
            GATEWAY_LOG_ERR,
        )
        owned.append(gateway)
        _wait_for_listener(gateway, GATEWAY_PORT, timeout=30)
        _validate_stack_identity(bridge_token, gateway_token)
        print("Arm simulator ready: bridge 127.0.0.1:8790, gateway 127.0.0.1:8788.")

        while not stop_requested.wait(0.5):
            for child in owned:
                exit_code = child.process.poll()
                if exit_code is not None:
                    raise RuntimeError(
                        f"{child.name} exited with code {exit_code}; see runtime/arm-sim logs."
                    )
        return 0
    finally:
        stop_requested.set()
        for child in reversed(owned):
            _terminate_owned(child)


def _parse_args(argv: Sequence[str] | None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Run the local Arm Isaac simulator stack.")
    parser.add_argument(
        "--ensure-tokens",
        action="store_true",
        help="create the two simulator-only token files if missing, then exit",
    )
    parser.add_argument(
        "--stop",
        action="store_true",
        help="ask the authenticated local supervisor to stop, then exit",
    )
    parser.add_argument(
        "--gui",
        action="store_true",
        help="show the interactive Isaac Sim window for the controlled bridge",
    )
    parser.add_argument(
        "--isaac-python",
        type=Path,
        help=(
            "path to the Isaac Sim Python launcher; alternatively set the "
            "ISAAC_PYTHON environment variable"
        ),
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    arguments = _parse_args(argv)
    try:
        selected_modes = sum(
            bool(value)
            for value in (arguments.ensure_tokens, arguments.stop, arguments.gui)
        )
        if selected_modes > 1:
            raise ValueError("Choose only one of --ensure-tokens, --stop, or --gui.")
        if arguments.ensure_tokens:
            _ensure_tokens()
            print("Simulator token files are ready.")
            return 0
        if arguments.stop:
            bridge_token, _gateway_token = _ensure_tokens()
            if _request_remote_stop(bridge_token):
                print("Simulator stop requested.")
            else:
                print("Arm simulator is already stopped.")
            return 0
        return _run(gui=arguments.gui, isaac_python=arguments.isaac_python)
    except (OSError, RuntimeError, ValueError) as error:
        print(f"Arm simulator launcher failed: {error}", file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
