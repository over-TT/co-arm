from __future__ import annotations

import json
import os
import subprocess
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path

import pytest

from web_backend.arm_control_center import (
    ACTION_IDS,
    ACTION_OPEN_ISAAC_SCENE,
    ACTION_OPEN_LATEST_RENDER,
    ACTION_OPEN_PROJECT_FOLDER,
    ACTION_RUN_SIM_CHECKS,
    ACTION_START_SIMULATOR,
    ACTION_STOP_SIMULATOR,
    ArmControlCenterActionLaunchError,
    ArmControlCenterActionNotFound,
    ArmControlCenterActionUnavailable,
    ArmControlCenterService,
    ControlCenterExecutables,
    LaunchRequest,
    SubprocessProcessLauncher,
)


FIXED_NOW = datetime(2026, 8, 18, 12, 30, tzinfo=timezone.utc)


@dataclass
class FakeProcess:
    pid: int
    exit_code: int | None = None

    def poll(self) -> int | None:
        return self.exit_code


class FakeLauncher:
    def __init__(self) -> None:
        self.requests: list[LaunchRequest] = []
        self.processes: list[FakeProcess] = []
        self.stopped: list[FakeProcess] = []

    def launch(self, request: LaunchRequest) -> FakeProcess:
        process = FakeProcess(pid=4100 + len(self.processes))
        self.requests.append(request)
        self.processes.append(process)
        return process

    def stop(self, process: FakeProcess) -> None:
        self.stopped.append(process)
        process.exit_code = -15


class NonStoppingLauncher(FakeLauncher):
    def stop(self, process: FakeProcess) -> None:
        self.stopped.append(process)


def _write_json(root: Path, relative: str, document: object) -> Path:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(document), encoding="utf-8")
    return path


def _touch(root: Path, relative: str) -> Path:
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.touch()
    return path


def _workspace(tmp_path: Path) -> tuple[Path, ControlCenterExecutables]:
    root = tmp_path / "project"
    root.mkdir()
    _write_json(
        root,
        "SOURCE_INDEX.json",
        {
            "schema": "arm-alliance.source-index.v1",
            "canonicalEntry": "software/SOURCE_INDEX.json",
            "simulation": {
                "package": "arm_sim",
                "modelManifest": "arm_sim/config/arm_model.json",
            },
            "laptop": {"backend": "web_backend"},
        },
    )
    _write_json(
        root,
        "python/arm_sim/config/arm_model.json",
        {
            "schema": "arm-sim.model.v1",
            "modelId": "desk_camera_arm_v0",
            "description": "Test arm model.",
            "fixedBase": True,
            "evidenceBoundary": {
                "measured": ["arm lengths"],
                "provisional": ["contact parameters"],
                "warning": "Not a calibrated digital twin.",
            },
            "kinematics": {"dimensionsMm": {"upperArm": 180.0}},
            "joints": [
                {
                    "servoId": "joint_1",
                    "name": "base_yaw_joint",
                    "type": "revolute",
                    "limitStatus": "provisional",
                    "zeroMeaning": "forward",
                }
            ],
            "realSimCalibration": {
                "status": "not_started",
                "requiredBeforeDigitalTwinClaims": ["Measure camera intrinsics."],
            },
        },
    )
    _write_json(
        root,
        "runtime/arm-sim/smoke_report.json",
        {
            "status": "ok",
            "physicalProof": False,
            "seed": 7,
            "resolution": [480, 640],
            "frameValidation": {
                "passed": True,
                "nonBlackFraction": 0.35,
                "targetLikePixels": 1800,
            },
            "limitations": ["Contact is provisional."],
        },
    )
    _touch(root, "runtime/arm-sim/smoke_scene.usda")
    _touch(root, "runtime/arm-sim/smoke_rgb.png")
    _touch(root, "python/arm_sim/isaac/view_scene.py")
    _touch(root, "python/arm_sim/isaac/bridge_server.py")
    _touch(root, "python/arm_sim/isaac/scene_factory.py")
    _touch(root, "python/arm_sim/assets/desk_camera_arm.urdf")
    _touch(root, "operations/scripts/run_arm_sim.py")
    for test_file in (
        "test_arm_asset.py",
        "test_isaac_smoke_contract.py",
    ):
        _touch(root, f"python/arm_sim/tests/{test_file}")

    executables_root = tmp_path / "executables"
    python = _touch(executables_root, "python.exe")
    isaac_python = _touch(executables_root, "python.bat")
    explorer = _touch(executables_root, "explorer.exe")
    experience = _touch(executables_root, "isaacsim.exp.full.kit")
    return root, ControlCenterExecutables(
        python=python,
        isaac_python=isaac_python,
        explorer=explorer,
        isaac_experience=experience,
    )


def _service(
    root: Path,
    executables: ControlCenterExecutables,
    launcher: FakeLauncher | None = None,
    launch_configuration_id: str = "manual-unmanaged",
) -> ArmControlCenterService:
    return ArmControlCenterService(
        project_root=root,
        launch_configuration_id=launch_configuration_id,
        executables=executables,
        launcher=launcher or FakeLauncher(),
        clock=lambda: FIXED_NOW,
    )


def test_manifest_is_bounded_to_allowlisted_sources_and_explicit_evidence(
    tmp_path: Path,
) -> None:
    root, executables = _workspace(tmp_path)
    rogue = _write_json(
        root,
        "runtime/arm-sim/newer_secret_report.json",
        {"status": "ok", "frameValidation": {"passed": False}, "secret": "nope"},
    )
    rogue.touch()
    launcher = FakeLauncher()

    manifest = _service(root, executables, launcher).manifest()

    assert manifest["schema"] == "arm-control-center.manifest.v1"
    assert manifest["service"]["version"] == "arm-control-center.v2"
    assert len(manifest["service"]["sourceSha256"]) == 64
    assert len(manifest["service"]["buildSha256"]) == 64
    assert manifest["service"]["launchConfigurationId"] == "manual-unmanaged"
    assert manifest["generatedAt"] == "2026-08-18T12:30:00Z"
    assert manifest["status"] == "ready"
    assert manifest["project"]["sourceIndex"]["entryCount"] == 4
    assert manifest["simulation"]["status"] == "provisional"
    assert manifest["simulation"]["model"]["measured"] == ["arm lengths"]
    assert manifest["simulation"]["model"]["provisional"] == [
        "contact parameters"
    ]
    assert manifest["simulation"]["latestResult"]["status"] == "passed"
    assert manifest["simulation"]["latestResult"]["seed"] == 7
    assert "secret" not in json.dumps(manifest)
    assert manifest["evidenceBoundary"]["physicalGatewayAccessed"] is False
    assert manifest["evidenceBoundary"]["physicalArmMotion"] is False
    assert manifest["evidenceBoundary"]["physicalProof"] is False
    assert len(manifest["evidenceBoundary"]["limitations"]) == 4
    assert [action["id"] for action in manifest["actions"]] == list(ACTION_IDS)
    assert all(action["available"] for action in manifest["actions"])
    assert launcher.requests == []


def test_manifest_exposes_only_the_opaque_launch_configuration_id(
    tmp_path: Path,
) -> None:
    root, executables = _workspace(tmp_path)
    opaque_id = "a" * 64

    manifest = _service(
        root,
        executables,
        launch_configuration_id=opaque_id,
    ).manifest()

    assert manifest["service"]["launchConfigurationId"] == opaque_id
    assert "token" not in json.dumps(manifest).lower()

    with pytest.raises(ValueError, match="Launch configuration id"):
        _service(
            root,
            executables,
            launch_configuration_id="not-an-opaque-id",
        )


def test_manifest_reports_missing_and_invalid_sources_without_crashing(
    tmp_path: Path,
) -> None:
    root, executables = _workspace(tmp_path)
    (root / "python/arm_sim/config/arm_model.json").write_text(
        "not-json", encoding="utf-8"
    )
    (root / "runtime/arm-sim/smoke_report.json").unlink()

    manifest = _service(root, executables).manifest()

    assert manifest["status"] == "incomplete"
    assert manifest["simulation"]["status"] == "incomplete"
    assert manifest["simulation"]["model"]["status"] == "invalid"
    assert "valid JSON" in manifest["simulation"]["model"]["error"]
    assert manifest["simulation"]["latestResult"]["status"] == "missing"


def test_actions_use_exact_allowlisted_arguments_and_no_user_options(tmp_path: Path) -> None:
    root, executables = _workspace(tmp_path)
    launcher = FakeLauncher()
    service = _service(root, executables, launcher)

    checks = service.run_action(ACTION_RUN_SIM_CHECKS)
    request = launcher.requests[-1]

    assert checks["status"] == "started"
    assert checks["physicalGatewayAccessed"] is False
    assert checks["physicalArmMotion"] is False
    assert request.arguments == (
        str(executables.python),
        "-m",
        "unittest",
        "discover",
        "-s",
        str((root / "python/arm_sim/tests").resolve()),
        "-v",
    )
    assert request.working_directory == (root / "python").resolve()
    assert request.hidden is True

    service.run_action(ACTION_START_SIMULATOR)
    assert launcher.requests[-1].arguments == (
        str(executables.python),
        str((root / "operations/scripts/run_arm_sim.py").resolve()),
        "--isaac-python",
        str(executables.isaac_python),
    )
    assert launcher.requests[-1].stdout_path == (
        root / "runtime/arm-sim/supervisor.out.log"
    ).resolve()
    assert launcher.requests[-1].stderr_path == (
        root / "runtime/arm-sim/supervisor.err.log"
    ).resolve()

    service.run_action(ACTION_OPEN_ISAAC_SCENE)
    assert launcher.requests[-1].arguments == (
        str(executables.isaac_python),
        str((root / "python/arm_sim/isaac/view_scene.py").resolve()),
        "--scene",
        str((root / "runtime/arm-sim/smoke_scene.usda").resolve()),
        "--experience",
        str(executables.isaac_experience),
    )

    service.run_action(ACTION_OPEN_LATEST_RENDER)
    assert launcher.requests[-1].arguments == (
        str(executables.explorer),
        str((root / "runtime/arm-sim/smoke_rgb.png").resolve()),
    )

    service.run_action(ACTION_OPEN_PROJECT_FOLDER)
    assert launcher.requests[-1].arguments == (
        str(executables.explorer),
        str(root.resolve()),
    )


def test_duplicate_action_is_not_launched_twice_and_completion_is_retained(
    tmp_path: Path,
) -> None:
    root, executables = _workspace(tmp_path)
    launcher = FakeLauncher()
    service = _service(root, executables, launcher)

    first = service.run_action(ACTION_OPEN_ISAAC_SCENE)
    duplicate = service.run_action(ACTION_OPEN_ISAAC_SCENE)

    assert first["status"] == "started"
    assert duplicate["status"] == "already_running"
    assert duplicate["runtime"]["pid"] == first["runtime"]["pid"]
    assert len(launcher.requests) == 1

    launcher.processes[0].exit_code = 0
    action = next(
        item
        for item in service.manifest()["actions"]
        if item["id"] == ACTION_OPEN_ISAAC_SCENE
    )
    assert action["runtime"]["status"] == "succeeded"
    assert action["runtime"]["lastExitCode"] == 0
    assert action["runtime"]["finishedAt"] == "2026-08-18T12:30:00Z"

    restarted = service.run_action(ACTION_OPEN_ISAAC_SCENE)
    assert restarted["status"] == "started"
    assert len(launcher.requests) == 2


def test_unknown_and_unavailable_actions_are_typed_failures(tmp_path: Path) -> None:
    root, executables = _workspace(tmp_path)
    service = _service(root, executables)

    with pytest.raises(ArmControlCenterActionNotFound):
        service.run_action("open_any_path")

    (root / "runtime/arm-sim/smoke_scene.usda").unlink()
    assert service.run_action(ACTION_OPEN_ISAAC_SCENE)["status"] == "started"

    (root / "runtime/arm-sim/smoke_rgb.png").unlink()
    with pytest.raises(ArmControlCenterActionUnavailable) as error:
        service.run_action(ACTION_OPEN_LATEST_RENDER)
    assert error.value.action_id == ACTION_OPEN_LATEST_RENDER
    assert "smoke_rgb.png" in error.value.reason


def test_user_facing_viewers_are_visible_but_service_helpers_stay_hidden(
    tmp_path: Path,
) -> None:
    root, executables = _workspace(tmp_path)
    launcher = FakeLauncher()
    service = _service(root, executables, launcher)

    for action_id in (
        ACTION_OPEN_ISAAC_SCENE,
        ACTION_OPEN_LATEST_RENDER,
        ACTION_OPEN_PROJECT_FOLDER,
    ):
        service.run_action(action_id)
        assert launcher.requests[-1].hidden is False

    service.run_action(ACTION_START_SIMULATOR)
    assert launcher.requests[-1].hidden is True
    service.run_action(ACTION_RUN_SIM_CHECKS)
    assert launcher.requests[-1].hidden is True


def test_simulator_stop_is_owned_idempotent_and_close_cleans_up(tmp_path: Path) -> None:
    root, executables = _workspace(tmp_path)
    launcher = FakeLauncher()
    service = _service(root, executables, launcher)

    started = service.run_action(ACTION_START_SIMULATOR)
    stopped = service.run_action(ACTION_STOP_SIMULATOR)
    stopped_again = service.run_action(ACTION_STOP_SIMULATOR)

    assert started["status"] == "started"
    assert stopped["status"] == "stopped"
    assert stopped_again["status"] == "started"
    assert launcher.stopped == [launcher.processes[0]]
    assert launcher.requests[-1].arguments == (
        str(executables.python),
        str((root / "operations/scripts/run_arm_sim.py").resolve()),
        "--stop",
    )
    launcher.processes[1].exit_code = 0

    service.run_action(ACTION_START_SIMULATOR)
    service.run_action(ACTION_OPEN_ISAAC_SCENE)
    service.close()

    assert launcher.processes[2] in launcher.stopped
    assert launcher.processes[3] not in launcher.stopped


def test_subprocess_launcher_never_uses_a_shell_or_inherited_streams(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    captured: dict[str, object] = {}
    expected = FakeProcess(pid=9001)

    def fake_popen(arguments: list[str], **kwargs: object) -> FakeProcess:
        captured["arguments"] = arguments
        captured.update(kwargs)
        return expected

    monkeypatch.setattr(subprocess, "Popen", fake_popen)
    request = LaunchRequest(
        arguments=(str(tmp_path / "fixed.exe"), "--fixed"),
        working_directory=tmp_path,
        hidden=True,
    )

    process = SubprocessProcessLauncher().launch(request)

    assert process is expected
    assert captured["arguments"] == list(request.arguments)
    assert captured["cwd"] == str(tmp_path)
    assert captured["shell"] is False
    assert captured["stdin"] is subprocess.DEVNULL
    assert captured["stdout"] is subprocess.DEVNULL
    assert captured["stderr"] is subprocess.DEVNULL
    assert captured["close_fds"] is True
    if os.name == "nt":
        assert int(captured["creationflags"]) & subprocess.CREATE_NO_WINDOW


def test_failed_stop_keeps_owned_process_visible(tmp_path: Path) -> None:
    root, executables = _workspace(tmp_path)
    launcher = NonStoppingLauncher()
    service = _service(root, executables, launcher)
    service.run_action(ACTION_START_SIMULATOR)

    with pytest.raises(ArmControlCenterActionLaunchError, match="confirm termination"):
        service.run_action(ACTION_STOP_SIMULATOR)

    start_action = next(
        action
        for action in service.manifest()["actions"]
        if action["id"] == ACTION_START_SIMULATOR
    )
    assert start_action["runtime"]["status"] == "running"
    assert start_action["runtime"]["pid"] == launcher.processes[0].pid
