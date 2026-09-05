from __future__ import annotations

import json
import time
from pathlib import Path

import pytest
from fastapi import HTTPException

from robot_gateway.arm_controller import ControllerCommandError
from robot_gateway.simple_arm_api import ArmService, CalibrateRequest, JointStore
from robot_gateway.tests.test_simple_arm_api import _connected


@pytest.fixture
def recovered_service(tmp_path: Path):
    """Use a surviving HAT frame, the case a Pi-only restore must distrust."""
    controller = _connected()
    store = JointStore(tmp_path)
    store.get("joint_1").rawZero = 2048
    assert store.save(required=True)
    service = ArmService(controller, store)
    service.close()
    service._service.join(timeout=1.0)
    service._refresh_multi_turn()
    base = next(row for row in service.state()["joints"] if row["id"] == "joint_1")
    assert base["positionTrusted"] is True
    marker = tmp_path / "base-reference-required.json"
    marker.write_text(json.dumps({
        "schema": "arm-base-reference-required.v1",
        "baseReferenceRequired": True,
        "reason": "RECOVERY_ARCHIVE_BASE_FRAME_UNTRUSTED",
        "sourceArchiveSha256": "a" * 64,
        "calibrationSha256": "b" * 64,
    }), encoding="utf-8")
    controller.commands.clear()
    return service, store, controller, marker


def test_recovery_hides_surviving_base_frame_until_explicit_zero(recovered_service):
    service, store, _, marker = recovered_service
    state = service.state()
    base = next(row for row in state["joints"] if row["id"] == "joint_1")
    assert state["baseReferenceRequired"] is True
    assert base["positionTrusted"] is False
    assert base["rawPosition"] is None
    assert base["degrees"] is None
    assert base["multiTurnTruth"] is None
    # Keep the saved coordinate only as recovery input, not as measured truth.
    assert store.get("joint_1").rawZero == 2048
    assert marker.is_file()


@pytest.mark.parametrize("lane", ["hold", "plan", "live", "commissioning"])
def test_recovery_blocks_each_authority_lane(recovered_service, lane):
    service, _, controller, _ = recovered_service
    with pytest.raises(ControllerCommandError, match="BASE_REFERENCE_REQUIRED"):
        if lane == "hold":
            service.torque([2, 3])
        elif lane == "plan":
            service._require_plan_health(controller.transport_state())
        elif lane == "live":
            service._live_follow_motion_health(controller.transport_state())
        else:
            with service.commissioning_mutation():
                pytest.fail("recovery allowed physical commissioning")
    assert not controller.commands


@pytest.mark.parametrize("failure", ["save", "readback", "unlink"])
def test_recovery_zero_io_failure_retains_gate(recovered_service, monkeypatch, failure):
    service, store, controller, marker = recovered_service
    if failure == "save":
        def fail_save(*, required=False):
            assert required is True
            raise OSError("disk write refused")
        monkeypatch.setattr(store, "save", fail_save)
    elif failure == "readback":
        monkeypatch.setattr(store, "persisted_base_zero_matches", lambda _: False)
    else:
        original_unlink = Path.unlink
        def fail_marker_unlink(path, *args, **kwargs):
            if path == marker:
                raise OSError("marker removal refused")
            return original_unlink(path, *args, **kwargs)
        monkeypatch.setattr(Path, "unlink", fail_marker_unlink)
    with pytest.raises((OSError, ControllerCommandError)):
        service.calibrate("joint_1", CalibrateRequest(
            here=["rawZero"], confirmedPhysicalBaseZero=True,
        ))
    assert any(row["operation"] == "HOME_MULTI_TURN" for row in controller.commands)
    assert marker.is_file()
    assert service.state()["baseReferenceRequired"] is True
    assert service.state()["joints"][0]["positionTrusted"] is False
    with pytest.raises(ControllerCommandError, match="BASE_REFERENCE_REQUIRED"):
        service.torque([2])


def test_recovery_gate_leaves_release_and_stop_available(recovered_service):
    service, _, controller, marker = recovered_service
    service.torque([])
    service.stop()
    assert marker.is_file()
    assert service.state()["baseReferenceRequired"] is True
    assert not any(
        row["operation"] == "HOLD_SET" and row.get("servoIds")
        for row in controller.commands
    )



def test_recovery_base_reference_gate_survives_clear_stop_and_only_homing_clears(
    tmp_path: Path,
) -> None:
    from robot_gateway.arm_controller import ControllerCommandError
    from robot_gateway.simple_arm_api import ArmService, CalibrateRequest, JointStore

    state_dir = tmp_path / "recovery-base-gate"
    state_dir.mkdir()
    marker = state_dir / "base-reference-required.json"
    marker.write_text(
        json.dumps(
            {
                "schema": "arm-base-reference-required.v1",
                "baseReferenceRequired": True,
                "reason": "RECOVERY_ARCHIVE_BASE_FRAME_UNTRUSTED",
                "sourceArchiveSha256": "a" * 64,
                "calibrationSha256": "b" * 64,
            },
            sort_keys=True,
            separators=(",", ":"),
        )
        + "\n",
        encoding="utf-8",
    )
    controller = _connected()
    service = ArmService(controller, JointStore(state_dir))
    service.close()
    service._service.join(timeout=1.0)
    controller.commands.clear()

    assert service.state()["baseReferenceRequired"] is True
    with pytest.raises(ControllerCommandError, match="BASE_REFERENCE_REQUIRED"):
        service.torque([2])
    assert not any(
        command["operation"] in {"HOLD_SET", "MOVE", "MOVE_SET", "TORQUE_LEASE"}
        for command in controller.commands
    )

    # The normal inspected STOP reset owns only arm-clear-required.json. It must
    # not make an archive-derived Base frame motion-authoritative.
    service.clear_stop_for_commissioning()
    assert marker.is_file()
    with pytest.raises(ControllerCommandError, match="BASE_REFERENCE_REQUIRED"):
        service.torque([2])

    with pytest.raises(HTTPException) as missing_confirmation:
        service.calibrate("joint_1", CalibrateRequest(here=["rawZero"]))
    assert missing_confirmation.value.status_code == 409
    assert "current-turn confirmation" in str(missing_confirmation.value.detail)
    assert marker.is_file()
    assert not any(command["operation"] == "HOME_MULTI_TURN" for command in controller.commands)

    original_home = controller.home_multi_turn
    controller.home_multi_turn = lambda _servo_id: (_ for _ in ()).throw(  # type: ignore[method-assign]
        RuntimeError("homing failed")
    )
    with pytest.raises(RuntimeError, match="homing failed"):
        service.calibrate(
            "joint_1",
            CalibrateRequest(
                here=["rawZero"],
                confirmedPhysicalBaseZero=True,
            ),
        )
    assert marker.is_file()

    controller.home_multi_turn = original_home  # type: ignore[method-assign]
    homed = service.calibrate(
        "joint_1",
        CalibrateRequest(
            here=["rawZero"],
            confirmedPhysicalBaseZero=True,
        ),
    )
    assert homed["baseReferenceRequired"] is False
    assert marker.exists() is False
    assert (state_dir / "arm-joints.json").is_file()
    service.torque([2])
    assert any(
        command["operation"] == "HOLD_SET" and command.get("servoIds") == [2]
        for command in controller.commands
    )



def test_malformed_recovery_base_reference_marker_fails_closed(tmp_path: Path) -> None:
    from robot_gateway.arm_controller import ControllerCommandError
    from robot_gateway.simple_arm_api import ArmService, JointStore

    marker = tmp_path / "base-reference-required.json"
    marker.write_text('{"baseReferenceRequired":false}\n', encoding="utf-8")
    service = ArmService(_connected(), JointStore(tmp_path))
    service.close()
    service._service.join(timeout=1.0)

    state = service.state()
    assert state["baseReferenceRequired"] is True
    assert state["baseReferenceReason"] == "BASE_REFERENCE_MARKER_INVALID"
    with pytest.raises(ControllerCommandError, match="BASE_REFERENCE_REQUIRED"):
        service.torque([2])



@pytest.mark.parametrize(
    "override",
    [
        {"rawZero": 123},
        {"zeroFromLimits": True},
        {"clear": ["rawZero"]},
    ],
)
def test_recovery_base_home_rejects_same_request_zero_overrides(
    tmp_path: Path,
    override: dict[str, object],
) -> None:
    from robot_gateway.simple_arm_api import ArmService, CalibrateRequest, JointStore

    marker = tmp_path / "base-reference-required.json"
    marker.write_text(
        json.dumps(
            {
                "schema": "arm-base-reference-required.v1",
                "baseReferenceRequired": True,
                "reason": "RECOVERY_ARCHIVE_BASE_FRAME_UNTRUSTED",
                "sourceArchiveSha256": "c" * 64,
                "calibrationSha256": "d" * 64,
            },
            sort_keys=True,
            separators=(",", ":"),
        )
        + "\n",
        encoding="utf-8",
    )
    controller = _connected()
    service = ArmService(controller, JointStore(tmp_path))
    service.close()
    service._service.join(timeout=1.0)
    controller.commands.clear()

    with pytest.raises(HTTPException) as refusal:
        service.calibrate(
            "joint_1",
            CalibrateRequest(
                here=["rawZero"],
                confirmedPhysicalBaseZero=True,
                **override,
            ),
        )

    assert refusal.value.status_code == 409
    assert "unambiguous Set zero here" in str(refusal.value.detail)
    assert marker.is_file()
    assert not any(command["operation"] == "HOME_MULTI_TURN" for command in controller.commands)



def test_recovery_base_marker_blocks_and_forgets_background_goal_chase(
    tmp_path: Path,
) -> None:
    from robot_gateway.simple_arm_api import ArmService, JointStore

    state_dir = tmp_path / "recovery-chase-gate"
    state_dir.mkdir()
    marker = state_dir / "base-reference-required.json"
    marker.write_text(
        json.dumps(
            {
                "schema": "arm-base-reference-required.v1",
                "baseReferenceRequired": True,
                "reason": "RECOVERY_ARCHIVE_BASE_FRAME_UNTRUSTED",
                "sourceArchiveSha256": "e" * 64,
                "calibrationSha256": "f" * 64,
            },
            sort_keys=True,
            separators=(",", ":"),
        )
        + "\n",
        encoding="utf-8",
    )
    controller = _connected()
    service = ArmService(controller, JointStore(state_dir))
    service.close()
    service._service.join(timeout=1.0)
    with service._lock:
        service._held = [2]
        service._goals["joint_2"] = (2300, 2, time.monotonic() + 5)
    controller.commands.clear()

    service._chase_goals()

    with service._lock:
        assert service._goals == {}
    assert not any(
        command["operation"] in {"MOVE", "MOVE_MULTI_TURN", "MOVE_SET"}
        for command in controller.commands
    )
