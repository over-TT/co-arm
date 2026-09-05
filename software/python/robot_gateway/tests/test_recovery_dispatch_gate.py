from __future__ import annotations

import json
from pathlib import Path

import pytest

from robot_gateway.arm_controller import ControllerCommandError
from robot_gateway.simple_arm_api import ArmService, JointStore
from robot_gateway.tests.test_simple_arm_api import _connected


@pytest.fixture
def dispatch_service(tmp_path: Path):
    controller = _connected()
    store = JointStore(tmp_path)
    for name in ("joint_2", "joint_3"):
        store.get(name).rawZero = 2048
    service = ArmService(controller, store)
    service.close()
    service._service.join(timeout=1.0)
    controller.commands.clear()
    return service, store, controller, tmp_path / "base-reference-required.json"


def _require_recovery(marker: Path) -> None:
    marker.write_text(json.dumps({
        "schema": "arm-base-reference-required.v1",
        "baseReferenceRequired": True,
        "reason": "RECOVERY_ARCHIVE_BASE_FRAME_UNTRUSTED",
        "sourceArchiveSha256": "a" * 64,
        "calibrationSha256": "b" * 64,
    }), encoding="utf-8")


@pytest.mark.parametrize("grouped", [False, True], ids=["legacy", "grouped"])
def test_gate_appearing_during_hold_blocks_final_dispatch(dispatch_service, monkeypatch, grouped):
    service, store, controller, marker = dispatch_service
    if not grouped:
        monkeypatch.setattr(controller, "move_set", None)
    original_hold = service._ensure_held

    def hold_then_require_recovery(servo_ids):
        original_hold(servo_ids)
        _require_recovery(marker)

    monkeypatch.setattr(service, "_ensure_held", hold_then_require_recovery)
    with pytest.raises(ControllerCommandError, match="BASE_REFERENCE_REQUIRED"):
        service._drive_prepared_goals_locked([
            ("joint_2", store.get("joint_2"), 2300),
            ("joint_3", store.get("joint_3"), 2200),
        ])

    assert any(row["operation"] == "HOLD_SET" for row in controller.commands)
    assert not any(row["operation"] in {"MOVE", "MOVE_SET", "MOVE_MULTI_TURN"} for row in controller.commands)
    assert store.base_reference_required()
    assert not service._goals


def test_gate_appearing_between_legacy_moves_blocks_remaining_dispatch(dispatch_service, monkeypatch):
    service, store, controller, marker = dispatch_service
    monkeypatch.setattr(controller, "move_set", None)
    original_move = controller.move

    def move_then_require_recovery(*args, **kwargs):
        result = original_move(*args, **kwargs)
        _require_recovery(marker)
        return result

    monkeypatch.setattr(controller, "move", move_then_require_recovery)
    with pytest.raises(ControllerCommandError, match="BASE_REFERENCE_REQUIRED"):
        service._drive_prepared_goals_locked([
            ("joint_2", store.get("joint_2"), 2300),
            ("joint_3", store.get("joint_3"), 2200),
        ])

    moves = [row for row in controller.commands if row["operation"] == "MOVE"]
    assert len(moves) == 1
    assert store.base_reference_required()
    assert not service._goals
