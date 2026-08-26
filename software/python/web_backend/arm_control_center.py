"""Read-only project overview and narrow launch actions for Arm Control Center.

The service intentionally has no robot-gateway client and no SIM/REAL mode
state.  It reads a fixed set of local, non-secret project artifacts and runs
only the six fixed actions declared below.  API callers cannot supply a path,
executable, argument, or working directory.
"""

from __future__ import annotations

import json
import hashlib
import os
import re
import subprocess
import sys
import threading
import time
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import Callable, Protocol

from .build_identity import build_identity


MANIFEST_SCHEMA = "arm-control-center.manifest.v1"
ACTION_RESULT_SCHEMA = "arm-control-center.action-result.v1"
SERVICE_VERSION = "arm-control-center.v2"
MAX_JSON_BYTES = 2 * 1024 * 1024
MANUAL_LAUNCH_CONFIGURATION_ID = "manual-unmanaged"
_OPAQUE_LAUNCH_CONFIGURATION_ID = re.compile(r"^[0-9a-f]{64}$")


def validate_launch_configuration_id(value: str) -> str:
    """Accept only the manual sentinel or an opaque metadata digest."""

    if value == MANUAL_LAUNCH_CONFIGURATION_ID:
        return value
    if isinstance(value, str) and _OPAQUE_LAUNCH_CONFIGURATION_ID.fullmatch(value):
        return value
    raise ValueError(
        "Launch configuration id must be an opaque lowercase SHA-256 or manual-unmanaged."
    )


def _loaded_source_sha256() -> str:
    """Pin readiness to the source bytes this process actually imported."""

    try:
        return hashlib.sha256(Path(__file__).resolve().read_bytes()).hexdigest()
    except OSError:
        return "unavailable"


LOADED_SOURCE_SHA256 = _loaded_source_sha256()
LOADED_BUILD_SHA256 = build_identity(Path(__file__).resolve().parents[2])

ACTION_START_SIMULATOR = "start_simulator"
ACTION_STOP_SIMULATOR = "stop_simulator"
ACTION_OPEN_ISAAC_SCENE = "open_isaac_scene"
ACTION_OPEN_LATEST_RENDER = "open_latest_render"
ACTION_OPEN_PROJECT_FOLDER = "open_project_folder"
ACTION_RUN_SIM_CHECKS = "run_sim_checks"
ACTION_IDS = (
    ACTION_START_SIMULATOR,
    ACTION_STOP_SIMULATOR,
    ACTION_OPEN_ISAAC_SCENE,
    ACTION_OPEN_LATEST_RENDER,
    ACTION_OPEN_PROJECT_FOLDER,
    ACTION_RUN_SIM_CHECKS,
)

_SOURCE_INDEX = Path("SOURCE_INDEX.json")
_MODEL_MANIFEST = Path("python/arm_sim/config/arm_model.json")
_SMOKE_REPORT = Path("runtime/arm-sim/smoke_report.json")
_ALLOWED_JSON_PATHS = frozenset(
    {_SOURCE_INDEX, _MODEL_MANIFEST, _SMOKE_REPORT}
)

_ISAAC_VIEWER = Path("python/arm_sim/isaac/view_scene.py")
_ISAAC_BRIDGE = Path("python/arm_sim/isaac/bridge_server.py")
_ISAAC_SCENE_FACTORY = Path("python/arm_sim/isaac/scene_factory.py")
_ARM_URDF = Path("python/arm_sim/assets/desk_camera_arm.urdf")
_ISAAC_SCENE = Path("runtime/arm-sim/smoke_scene.usda")
_SIM_LAUNCHER = Path("operations/scripts/run_arm_sim.py")
_SIM_SUPERVISOR_OUT = Path("runtime/arm-sim/supervisor.out.log")
_SIM_SUPERVISOR_ERR = Path("runtime/arm-sim/supervisor.err.log")
_LATEST_RENDER = Path("runtime/arm-sim/smoke_rgb.png")
_SIM_TEST_DIRECTORY = Path("python/arm_sim/tests")
_SIM_TEST_FILES = (
    Path("python/arm_sim/tests/test_arm_asset.py"),
    Path("python/arm_sim/tests/test_isaac_smoke_contract.py"),
)

_STATIC_LIMITATIONS = (
    "This page reports local source and simulator artifacts; it does not poll or command the Raspberry Pi gateway.",
    "Simulator results are not physical-arm, camera, contact, or outcome proof.",
    "The current simulation is a workflow asset, not a calibrated digital twin.",
    "Opening or checking local simulator files never changes the dashboard's SIM/REAL backend selection.",
)


class ArmControlCenterError(RuntimeError):
    """Base class for typed service failures suitable for API translation."""


class ArmControlCenterActionNotFound(ArmControlCenterError):
    def __init__(self, action_id: str) -> None:
        super().__init__(f"Unknown Arm Control Center action: {action_id}")
        self.action_id = action_id


class ArmControlCenterActionUnavailable(ArmControlCenterError):
    def __init__(self, action_id: str, reason: str) -> None:
        super().__init__(reason)
        self.action_id = action_id
        self.reason = reason


class ArmControlCenterActionLaunchError(ArmControlCenterError):
    def __init__(self, action_id: str, reason: str) -> None:
        super().__init__(reason)
        self.action_id = action_id
        self.reason = reason


class ProcessHandle(Protocol):
    pid: int

    def poll(self) -> int | None: ...


@dataclass(frozen=True)
class LaunchRequest:
    """A complete allowlisted child-process request.

    ``arguments`` is immutable and is never extended from an API payload.
    """

    arguments: tuple[str, ...]
    working_directory: Path
    hidden: bool = True
    stdout_path: Path | None = None
    stderr_path: Path | None = None


class ProcessLauncher(Protocol):
    def launch(self, request: LaunchRequest) -> ProcessHandle: ...

    def stop(self, process: ProcessHandle) -> None: ...


class SubprocessProcessLauncher:
    """Production launcher with no shell and no inherited console streams."""

    def launch(self, request: LaunchRequest) -> ProcessHandle:
        creation_flags = 0
        startup_info = None
        if request.hidden and os.name == "nt":
            creation_flags = getattr(subprocess, "CREATE_NO_WINDOW", 0)
            startup_info = subprocess.STARTUPINFO()
            startup_info.dwFlags |= subprocess.STARTF_USESHOWWINDOW
            startup_info.wShowWindow = subprocess.SW_HIDE
        stdout_handle = (
            request.stdout_path.open("ab", buffering=0)
            if request.stdout_path is not None
            else None
        )
        stderr_handle = (
            request.stderr_path.open("ab", buffering=0)
            if request.stderr_path is not None
            else None
        )
        try:
            return subprocess.Popen(
                list(request.arguments),
                cwd=str(request.working_directory),
                shell=False,
                stdin=subprocess.DEVNULL,
                stdout=stdout_handle or subprocess.DEVNULL,
                stderr=stderr_handle or subprocess.DEVNULL,
                close_fds=True,
                creationflags=creation_flags,
                startupinfo=startup_info,
            )
        finally:
            if stdout_handle is not None:
                stdout_handle.close()
            if stderr_handle is not None:
                stderr_handle.close()

    def stop(self, process: ProcessHandle) -> None:
        if process.poll() is not None:
            return
        if os.name == "nt":
            completed = subprocess.run(
                ["taskkill.exe", "/PID", str(process.pid), "/T", "/F"],
                check=False,
                stdin=subprocess.DEVNULL,
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
                timeout=15,
                shell=False,
            )
            deadline = time.monotonic() + 5.0
            while process.poll() is None and time.monotonic() < deadline:
                time.sleep(0.05)
            if process.poll() is None:
                raise OSError(
                    f"Simulator process tree {process.pid} did not terminate "
                    f"(taskkill exit {completed.returncode})."
                )
            return
        terminate = getattr(process, "terminate", None)
        wait = getattr(process, "wait", None)
        if callable(terminate):
            terminate()
        if callable(wait):
            try:
                wait(timeout=10)
            except subprocess.TimeoutExpired:
                kill = getattr(process, "kill", None)
                if callable(kill):
                    kill()
        deadline = time.monotonic() + 5.0
        while process.poll() is None and time.monotonic() < deadline:
            time.sleep(0.05)
        if process.poll() is None:
            raise OSError(f"Simulator process {process.pid} did not terminate.")


@dataclass(frozen=True)
class ControlCenterExecutables:
    python: Path
    isaac_python: Path
    explorer: Path
    isaac_experience: Path

    @classmethod
    def defaults(cls) -> "ControlCenterExecutables":
        return cls(
            python=Path(sys.executable).resolve(),
            isaac_python=Path(
                os.environ.get("ISAAC_PYTHON", r"C:\isaacsim\python.bat")
            ).resolve(),
            explorer=Path(r"C:\Windows\explorer.exe"),
            isaac_experience=Path(
                os.environ.get(
                    "ISAAC_EXPERIENCE",
                    r"C:\isaacsim\apps\isaacsim.exp.full.kit",
                )
            ).resolve(),
        )


@dataclass(frozen=True)
class _ActionDefinition:
    action_id: str
    label: str
    description: str
    kind: str
    request: LaunchRequest | None
    required_files: tuple[Path, ...]
    modes: tuple[str, ...] = ("sim", "real")
    required_directories: tuple[Path, ...] = ()


@dataclass
class _ActionRuntime:
    process: ProcessHandle | None = None
    status: str = "idle"
    pid: int | None = None
    started_at: str | None = None
    finished_at: str | None = None
    last_exit_code: int | None = None
    last_error: str | None = None


@dataclass(frozen=True)
class _JsonSource:
    status: str
    path: Path
    document: dict[str, object] | None = None
    updated_at: str | None = None
    error: str | None = None


def _utc_now() -> datetime:
    return datetime.now(timezone.utc)


def _iso_time(value: datetime) -> str:
    return value.astimezone(timezone.utc).isoformat().replace("+00:00", "Z")


def _mtime_iso(path: Path) -> str:
    return _iso_time(datetime.fromtimestamp(path.stat().st_mtime, timezone.utc))


def _string(value: object, default: str = "") -> str:
    return value if isinstance(value, str) else default


def _boolean(value: object) -> bool | None:
    return value if isinstance(value, bool) else None


def _number(value: object) -> int | float | None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        return None
    return value


def _mapping(value: object) -> dict[str, object]:
    return value if isinstance(value, dict) else {}


def _items(value: object) -> list[object]:
    return value if isinstance(value, list) else []


def _strings(value: object, *, limit: int = 128) -> list[str]:
    result: list[str] = []
    for item in _items(value)[:limit]:
        if isinstance(item, str) and len(item) <= 2048:
            result.append(item)
    return result


def _summary_item(value: object, fields: tuple[str, ...]) -> dict[str, object] | None:
    item = _mapping(value)
    if not item:
        return None
    result: dict[str, object] = {}
    for field in fields:
        candidate = item.get(field)
        if isinstance(candidate, (str, int, float, bool)) and not isinstance(candidate, str) or (
            isinstance(candidate, str) and len(candidate) <= 2048
        ):
            result[field] = candidate
    return result or None


class ArmControlCenterService:
    """Build the Control Center manifest and execute fixed local actions."""

    def __init__(
        self,
        *,
        project_root: Path,
        launch_configuration_id: str = MANUAL_LAUNCH_CONFIGURATION_ID,
        executables: ControlCenterExecutables | None = None,
        launcher: ProcessLauncher | None = None,
        clock: Callable[[], datetime] = _utc_now,
    ) -> None:
        self._project_root = project_root.resolve()
        self._launch_configuration_id = validate_launch_configuration_id(
            launch_configuration_id
        )
        self._executables = executables or ControlCenterExecutables.defaults()
        self._launcher = launcher or SubprocessProcessLauncher()
        self._clock = clock
        self._lock = threading.Lock()
        self._runtime = {action_id: _ActionRuntime() for action_id in ACTION_IDS}
        self._actions = self._build_actions()

    def manifest(self) -> dict[str, object]:
        """Return a bounded read-only view of the local Arm workspace."""

        source_index = self._read_allowed_json(_SOURCE_INDEX)
        model = self._read_allowed_json(_MODEL_MANIFEST)
        latest_result = self._read_allowed_json(_SMOKE_REPORT)

        required_sources = (source_index, model)
        source_ready = all(source.status == "ready" for source in required_sources)
        model_summary = self._model_summary(model)
        calibration_status = _string(
            _mapping(_mapping(model.document).get("realSimCalibration")).get("status"),
            "unknown",
        )
        simulation_status = "incomplete"
        if source_ready:
            simulation_status = (
                "calibrated" if calibration_status == "complete" else "provisional"
            )

        return {
            "schema": MANIFEST_SCHEMA,
            "service": {
                "version": SERVICE_VERSION,
                "sourceSha256": LOADED_SOURCE_SHA256,
                "buildSha256": LOADED_BUILD_SHA256,
                "launchConfigurationId": self._launch_configuration_id,
            },
            "generatedAt": _iso_time(self._clock()),
            "status": "ready" if source_ready else "incomplete",
            "project": {
                "name": "Arm Alliance",
                "root": ".",
                "sourceIndex": self._source_index_summary(source_index),
            },
            "simulation": {
                "status": simulation_status,
                "sourceReady": source_ready,
                "physicalCalibrationStatus": calibration_status,
                "model": model_summary,
                "latestResult": self._result_summary(latest_result),
            },
            "actions": self._action_documents(),
            "evidenceBoundary": {
                "physicalGatewayAccessed": False,
                "physicalArmMotion": False,
                "physicalProof": False,
                "limitations": list(_STATIC_LIMITATIONS),
            },
        }

    def run_action(self, action_id: str) -> dict[str, object]:
        """Start one fixed action or report its already-running process.

        There is deliberately no options argument: callers cannot alter the
        executable, script, target, arguments, working directory, or backend.
        """

        definition = self._actions.get(action_id)
        if definition is None:
            raise ArmControlCenterActionNotFound(action_id)

        with self._lock:
            if action_id == ACTION_STOP_SIMULATOR:
                start_runtime = self._refresh_runtime_locked(ACTION_START_SIMULATOR)
                if start_runtime.process is not None and start_runtime.status == "running":
                    return self._stop_simulator_locked()
            runtime = self._refresh_runtime_locked(action_id)
            if runtime.status == "running":
                return self._action_result(action_id, "already_running", runtime)

            available, reason = self._availability(definition)
            if not available:
                raise ArmControlCenterActionUnavailable(action_id, reason or "Action unavailable.")

            try:
                if definition.request is None:
                    raise ArmControlCenterActionUnavailable(
                        action_id, "This action has no launch request."
                    )
                for log_path in (
                    definition.request.stdout_path,
                    definition.request.stderr_path,
                ):
                    if log_path is not None:
                        log_path.parent.mkdir(parents=True, exist_ok=True)
                process = self._launcher.launch(definition.request)
            except OSError as error:
                message = str(error)[:512] or "The local process could not be started."
                runtime.process = None
                runtime.status = "failed"
                runtime.pid = None
                runtime.finished_at = _iso_time(self._clock())
                runtime.last_exit_code = None
                runtime.last_error = message
                raise ArmControlCenterActionLaunchError(action_id, message) from error

            runtime.process = process
            runtime.status = "running"
            runtime.pid = process.pid
            runtime.started_at = _iso_time(self._clock())
            runtime.finished_at = None
            runtime.last_exit_code = None
            runtime.last_error = None
            return self._action_result(action_id, "started", runtime)

    def close(self) -> None:
        """Stop only the simulator service process this instance launched."""

        with self._lock:
            self._stop_simulator_locked(record_stop_action=False)

    def _stop_simulator_locked(
        self, *, record_stop_action: bool = True
    ) -> dict[str, object]:
        start_runtime = self._refresh_runtime_locked(ACTION_START_SIMULATOR)
        stop_runtime = self._runtime[ACTION_STOP_SIMULATOR]
        now = _iso_time(self._clock())
        if start_runtime.process is None or start_runtime.status != "running":
            if record_stop_action:
                stop_runtime.status = "succeeded"
                stop_runtime.started_at = now
                stop_runtime.finished_at = now
                stop_runtime.last_exit_code = 0
                stop_runtime.last_error = None
            return self._action_result(
                ACTION_STOP_SIMULATOR, "already_stopped", stop_runtime
            )
        process = start_runtime.process
        try:
            self._launcher.stop(process)
        except (OSError, subprocess.SubprocessError) as error:
            message = str(error)[:512] or "The owned simulator process could not be stopped."
            if record_stop_action:
                stop_runtime.status = "failed"
                stop_runtime.started_at = now
                stop_runtime.finished_at = now
                stop_runtime.last_error = message
            raise ArmControlCenterActionLaunchError(
                ACTION_STOP_SIMULATOR, message
            ) from error
        exit_code = process.poll()
        if exit_code is None:
            message = "The simulator process did not confirm termination."
            if record_stop_action:
                stop_runtime.status = "failed"
                stop_runtime.started_at = now
                stop_runtime.finished_at = now
                stop_runtime.last_error = message
            raise ArmControlCenterActionLaunchError(ACTION_STOP_SIMULATOR, message)
        start_runtime.process = None
        start_runtime.status = "stopped"
        start_runtime.finished_at = now
        start_runtime.last_exit_code = exit_code
        if record_stop_action:
            stop_runtime.status = "succeeded"
            stop_runtime.started_at = now
            stop_runtime.finished_at = now
            stop_runtime.last_exit_code = 0
            stop_runtime.last_error = None
        return self._action_result(ACTION_STOP_SIMULATOR, "stopped", stop_runtime)

    def _build_actions(self) -> dict[str, _ActionDefinition]:
        root = self._project_root
        executable = self._executables
        simulator_launcher = self._project_path(_SIM_LAUNCHER)
        isaac_bridge = self._project_path(_ISAAC_BRIDGE)
        arm_urdf = self._project_path(_ARM_URDF)
        simulator_stdout = self._project_path(_SIM_SUPERVISOR_OUT)
        simulator_stderr = self._project_path(_SIM_SUPERVISOR_ERR)
        viewer = self._project_path(_ISAAC_VIEWER)
        scene = self._project_path(_ISAAC_SCENE)
        render = self._project_path(_LATEST_RENDER)
        python_root = self._project_path(Path("python"))
        test_directory = self._project_path(_SIM_TEST_DIRECTORY)
        test_files = tuple(self._project_path(path) for path in _SIM_TEST_FILES)
        actions = (
            _ActionDefinition(
                action_id=ACTION_START_SIMULATOR,
                label="Start simulator",
                description="Start the persistent Isaac bridge and isolated simulator gateway.",
                kind="service",
                request=LaunchRequest(
                    arguments=(
                        str(executable.python),
                        str(simulator_launcher),
                        "--isaac-python",
                        str(executable.isaac_python),
                    ),
                    working_directory=root,
                    stdout_path=simulator_stdout,
                    stderr_path=simulator_stderr,
                ),
                required_files=(
                    executable.python,
                    executable.isaac_python,
                    simulator_launcher,
                    isaac_bridge,
                    arm_urdf,
                ),
                modes=("sim",),
            ),
            _ActionDefinition(
                action_id=ACTION_STOP_SIMULATOR,
                label="Stop simulator",
                description="Stop the authenticated local simulator, including one retained across a dashboard restart.",
                kind="service",
                request=LaunchRequest(
                    arguments=(
                        str(executable.python),
                        str(simulator_launcher),
                        "--stop",
                    ),
                    working_directory=root,
                ),
                required_files=(executable.python, simulator_launcher),
                modes=("sim",),
            ),
            _ActionDefinition(
                action_id=ACTION_OPEN_ISAAC_SCENE,
                label="Open Isaac scene",
                description="Open the generated local desk-arm scene in Isaac Sim.",
                kind="viewer",
                request=LaunchRequest(
                    arguments=(
                        str(executable.isaac_python),
                        str(viewer),
                        "--scene",
                        str(scene),
                        "--experience",
                        str(executable.isaac_experience),
                    ),
                    working_directory=root,
                    hidden=False,
                ),
                required_files=(
                    executable.isaac_python,
                    executable.isaac_experience,
                    viewer,
                    self._project_path(_ISAAC_SCENE_FACTORY),
                    arm_urdf,
                ),
            ),
            _ActionDefinition(
                action_id=ACTION_OPEN_LATEST_RENDER,
                label="Open latest render",
                description="Open the allowlisted simulator smoke-test image.",
                kind="viewer",
                request=LaunchRequest(
                    arguments=(str(executable.explorer), str(render)),
                    working_directory=root,
                    hidden=False,
                ),
                required_files=(executable.explorer, render),
            ),
            _ActionDefinition(
                action_id=ACTION_OPEN_PROJECT_FOLDER,
                label="Open project folder",
                description="Open the Arm Alliance project root in Windows Explorer.",
                kind="folder",
                request=LaunchRequest(
                    arguments=(str(executable.explorer), str(root)),
                    working_directory=root,
                    hidden=False,
                ),
                required_files=(executable.explorer,),
                required_directories=(root,),
            ),
            _ActionDefinition(
                action_id=ACTION_RUN_SIM_CHECKS,
                label="Run simulation checks",
                description="Run the fixed Isaac-free arm_sim unittest suite.",
                kind="check",
                request=LaunchRequest(
                    arguments=(
                        str(executable.python),
                        "-m",
                        "unittest",
                        "discover",
                        "-s",
                        str(test_directory),
                        "-v",
                    ),
                    working_directory=python_root,
                ),
                required_files=(executable.python, *test_files),
                required_directories=(python_root, test_directory),
            ),
        )
        return {action.action_id: action for action in actions}

    def _project_path(self, relative_path: Path) -> Path:
        candidate = (self._project_root / relative_path).resolve()
        if not candidate.is_relative_to(self._project_root):
            raise ValueError(f"Control Center path escapes project root: {relative_path}")
        return candidate

    def _read_allowed_json(self, relative_path: Path) -> _JsonSource:
        if relative_path not in _ALLOWED_JSON_PATHS:
            raise ValueError(f"JSON source is not allowlisted: {relative_path}")
        path = self._project_path(relative_path)
        if not path.is_file():
            return _JsonSource(status="missing", path=path, error="File is missing.")
        try:
            size = path.stat().st_size
            if size > MAX_JSON_BYTES:
                return _JsonSource(
                    status="invalid",
                    path=path,
                    error=f"File exceeds the {MAX_JSON_BYTES}-byte read limit.",
                )
            raw = path.read_text(encoding="utf-8")
            document = json.loads(raw)
        except (OSError, UnicodeError, json.JSONDecodeError) as error:
            return _JsonSource(
                status="invalid",
                path=path,
                error=f"Could not read valid JSON: {str(error)[:256]}",
            )
        if not isinstance(document, dict):
            return _JsonSource(
                status="invalid",
                path=path,
                error="JSON root must be an object.",
            )
        return _JsonSource(
            status="ready",
            path=path,
            document=document,
            updated_at=_mtime_iso(path),
        )

    def _source_document(self, source: _JsonSource) -> dict[str, object]:
        result: dict[str, object] = {
            "status": source.status,
            "path": str(source.path),
        }
        if source.updated_at is not None:
            result["updatedAt"] = source.updated_at
        if source.error is not None:
            result["error"] = source.error
        return result

    def _source_index_summary(self, source: _JsonSource) -> dict[str, object]:
        result = self._source_document(source)
        document = _mapping(source.document)
        if not document:
            return result
        entries: list[dict[str, str]] = []

        def visit(value: object, prefix: tuple[str, ...]) -> None:
            if len(entries) >= 128:
                return
            if isinstance(value, str):
                entries.append({"id": ".".join(prefix), "path": value})
                return
            if isinstance(value, dict):
                for key, child in value.items():
                    if isinstance(key, str):
                        visit(child, (*prefix, key))

        for key, value in document.items():
            if key != "schema":
                visit(value, (key,))
        result.update(
            {
                "schema": _string(document.get("schema"), "unknown"),
                "canonicalEntry": _string(document.get("canonicalEntry")),
                "entryCount": len(entries),
                "entries": entries,
            }
        )
        return result

    def _model_summary(self, source: _JsonSource) -> dict[str, object]:
        result = self._source_document(source)
        document = _mapping(source.document)
        if not document:
            return result
        evidence = _mapping(document.get("evidenceBoundary"))
        kinematics = _mapping(document.get("kinematics"))
        calibration = _mapping(document.get("realSimCalibration"))
        joints: list[dict[str, object]] = []
        for value in _items(document.get("joints"))[:16]:
            summary = _summary_item(
                value,
                ("servoId", "name", "type", "limitStatus", "zeroMeaning"),
            )
            if summary is not None:
                joints.append(summary)
        result.update(
            {
                "schema": _string(document.get("schema"), "unknown"),
                "modelId": _string(document.get("modelId"), "unknown"),
                "description": _string(document.get("description")),
                "fixedBase": _boolean(document.get("fixedBase")),
                "dimensionsMm": _mapping(kinematics.get("dimensionsMm")),
                "joints": joints,
                "measured": _strings(evidence.get("measured")),
                "provisional": _strings(evidence.get("provisional")),
                "warning": _string(evidence.get("warning")),
                "calibrationStatus": _string(calibration.get("status"), "unknown"),
                "requiredBeforeDigitalTwinClaims": _strings(
                    calibration.get("requiredBeforeDigitalTwinClaims")
                ),
            }
        )
        return result

    def _result_summary(self, source: _JsonSource) -> dict[str, object]:
        result = self._source_document(source)
        document = _mapping(source.document)
        if not document:
            return result
        validation = _mapping(document.get("frameValidation"))
        passed = validation.get("passed") is True and document.get("status") == "ok"
        resolution = [
            number
            for value in _items(document.get("resolution"))[:2]
            if (number := _number(value)) is not None
        ]
        result.update(
            {
                "status": "passed" if passed else "failed",
                "sourceStatus": source.status,
                "reportStatus": _string(document.get("status"), "unknown"),
                "passed": passed,
                "physicalProof": document.get("physicalProof") is True,
                "seed": _number(document.get("seed")),
                "resolution": resolution,
                "frameValidation": {
                    key: validation[key]
                    for key in (
                        "passed",
                        "nonBlackFraction",
                        "nonBlackPixels",
                        "deskLikePixels",
                        "targetLikePixels",
                    )
                    if isinstance(validation.get(key), (bool, int, float))
                },
                "limitations": _strings(document.get("limitations")),
                "renderPath": str(self._project_path(_LATEST_RENDER)),
                "scenePath": str(self._project_path(_ISAAC_SCENE)),
            }
        )
        return result

    def _availability(self, definition: _ActionDefinition) -> tuple[bool, str | None]:
        missing_files = [str(path) for path in definition.required_files if not path.is_file()]
        if missing_files:
            return False, f"Required local file is missing: {missing_files[0]}"
        missing_directories = [
            str(path) for path in definition.required_directories if not path.is_dir()
        ]
        if missing_directories:
            return False, f"Required local directory is missing: {missing_directories[0]}"
        return True, None

    def _refresh_runtime_locked(self, action_id: str) -> _ActionRuntime:
        runtime = self._runtime[action_id]
        if runtime.process is None or runtime.status != "running":
            return runtime
        exit_code = runtime.process.poll()
        if exit_code is None:
            return runtime
        runtime.process = None
        runtime.status = "succeeded" if exit_code == 0 else "failed"
        runtime.finished_at = _iso_time(self._clock())
        runtime.last_exit_code = exit_code
        if exit_code != 0:
            request = self._actions[action_id].request
            if request is not None and request.stderr_path is not None:
                runtime.last_error = self._log_tail(request.stderr_path)
        return runtime

    @staticmethod
    def _log_tail(path: Path, maximum_bytes: int = 2048) -> str | None:
        try:
            with path.open("rb") as handle:
                handle.seek(0, os.SEEK_END)
                size = handle.tell()
                handle.seek(max(0, size - maximum_bytes))
                value = handle.read(maximum_bytes).decode("utf-8", errors="replace").strip()
        except OSError:
            return None
        return value[-maximum_bytes:] or None

    def _action_documents(self) -> list[dict[str, object]]:
        with self._lock:
            documents: list[dict[str, object]] = []
            for action_id in ACTION_IDS:
                definition = self._actions[action_id]
                runtime = self._refresh_runtime_locked(action_id)
                available, reason = self._availability(definition)
                document: dict[str, object] = {
                    "id": definition.action_id,
                    "label": definition.label,
                    "description": definition.description,
                    "kind": definition.kind,
                    "modes": list(definition.modes),
                    "available": available,
                    "runtime": self._runtime_document(runtime),
                }
                if reason is not None:
                    document["unavailableReason"] = reason
                documents.append(document)
            return documents

    def _runtime_document(self, runtime: _ActionRuntime) -> dict[str, object]:
        document: dict[str, object] = {"status": runtime.status}
        for key, value in (
            ("pid", runtime.pid),
            ("startedAt", runtime.started_at),
            ("finishedAt", runtime.finished_at),
            ("lastExitCode", runtime.last_exit_code),
            ("lastError", runtime.last_error),
        ):
            if value is not None:
                document[key] = value
        return document

    def _action_result(
        self, action_id: str, status: str, runtime: _ActionRuntime
    ) -> dict[str, object]:
        return {
            "schema": ACTION_RESULT_SCHEMA,
            "actionId": action_id,
            "status": status,
            "runtime": self._runtime_document(runtime),
            "physicalGatewayAccessed": False,
            "physicalArmMotion": False,
        }


def create_default_arm_control_center_service() -> ArmControlCenterService:
    """Create the production service rooted at this repository checkout."""

    return ArmControlCenterService(project_root=Path(__file__).resolve().parents[2])
