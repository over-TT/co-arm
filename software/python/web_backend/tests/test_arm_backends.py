from __future__ import annotations

from pathlib import Path

import pytest

from web_backend.arm_backends import ArmBackendDescriptor, ArmBackendRegistry
from web_backend.robot_gateway_client import (
    CameraFrameResponse,
    RobotGatewayError,
    SimpleArmPlanExecuteRequest,
    SimpleArmPlanPreviewRequest,
)


class FakeGateway:
    def __init__(self, name: str) -> None:
        self.name = name
        self.closed = False
        self.executions = 0

    def close(self) -> None:
        self.closed = True

    def arm_state(self) -> dict[str, object]:
        return {"state": self.name}

    def arm_plan_preview(self, _request: object) -> dict[str, object]:
        return {
            "planId": "armplan_shared1234",
            "planDigest": "sha256:" + "a" * 64,
            "expiresInMs": 30_000,
        }

    def arm_plan_execute(self, _request: object) -> dict[str, object]:
        self.executions += 1
        return {"executed": True}

    def camera_frame(self) -> CameraFrameResponse:
        return CameraFrameResponse(
            content=b"\xff\xd8\xff\xd9",
            media_type="image/jpeg",
            headers={"X-Frame-Id": "frame_12345678"},
        )


class IdentityGateway(FakeGateway):
    def __init__(self, name: str, instance_id: str) -> None:
        super().__init__(name)
        self.instance_id = instance_id
        self.cached: dict[str, object] | None = None

    def backend_identity(self, *, refresh: bool = False) -> dict[str, object] | None:
        if not refresh:
            return dict(self.cached) if self.cached else None
        self.cached = {
            "backendId": "sim",
            "backendInstanceId": self.instance_id,
            "simulated": True,
        }
        return dict(self.cached)


class ResponseIdentityGateway(FakeGateway):
    def __init__(self, name: str, instance_id: str) -> None:
        super().__init__(name)
        self.instance_id = instance_id
        self.identity_refreshes = 0

    def arm_state(self) -> dict[str, object]:
        return {
            "state": self.name,
            "backendId": "sim",
            "backendInstanceId": self.instance_id,
            "simulated": True,
        }

    def backend_identity(self, *, refresh: bool = False) -> dict[str, object] | None:
        self.identity_refreshes += 1
        return {
            "backendId": "sim",
            "backendInstanceId": self.instance_id,
            "simulated": True,
        }


class MismatchedOperationIdentityGateway(FakeGateway):
    def backend_identity(self, *, refresh: bool = False) -> dict[str, object]:
        return {
            "backendId": "real",
            "backendInstanceId": "gateway:process-before",
            "simulated": False,
        }

    def arm_plan_preview(self, _request: object) -> dict[str, object]:
        return {
            **super().arm_plan_preview(_request),
            "backendId": "real",
            "backendInstanceId": "gateway:process-after",
            "simulated": False,
        }


def descriptor(backend_id: str, gateway: FakeGateway) -> ArmBackendDescriptor:
    return ArmBackendDescriptor.create(  # type: ignore[arg-type]
        backend_id, backend_id.upper(), gateway, instance_id=f"{backend_id}-instance"  # type: ignore[arg-type]
    )


def preview_request() -> SimpleArmPlanPreviewRequest:
    return SimpleArmPlanPreviewRequest.model_validate({"targets": {"joint_1": 0.0}})


def execute_request() -> SimpleArmPlanExecuteRequest:
    return SimpleArmPlanExecuteRequest.model_validate(
        {
            "planId": "armplan_shared1234",
            "planDigest": "sha256:" + "a" * 64,
        }
    )


def test_backend_listing_and_request_default_are_explicit() -> None:
    sim = FakeGateway("sim")
    real = FakeGateway("real")
    registry = ArmBackendRegistry(
        [descriptor("real", real), descriptor("sim", sim)],
        default_backend_id="sim",
    )

    assert registry.normalize_backend_id(None) == "sim"
    assert registry.normalize_backend_id(" REAL ") == "real"
    listing = registry.public_backends()
    assert listing["defaultBackendId"] == "sim"
    assert [row["backendId"] for row in listing["backends"]] == ["sim", "real"]

    with pytest.raises(RobotGatewayError, match="not configured") as raised:
        registry.normalize_backend_id("other")
    assert raised.value.status_code == 422


def test_results_carry_backend_identity() -> None:
    sim = FakeGateway("sim")
    registry = ArmBackendRegistry(
        [descriptor("sim", sim)], default_backend_id="sim"
    )

    state = registry.bound_client("sim").arm_state()

    assert state == {
        "state": "sim",
        "backendId": "sim",
        "backendInstanceId": "sim-instance",
        "simulated": True,
    }


def test_plan_cannot_cross_backends_and_is_one_use() -> None:
    sim = FakeGateway("sim")
    real = FakeGateway("real")
    registry = ArmBackendRegistry(
        [descriptor("real", real), descriptor("sim", sim)],
        default_backend_id="sim",
    )
    registry.bound_client("sim").arm_plan_preview(preview_request())

    with pytest.raises(RobotGatewayError, match="another backend") as crossed:
        registry.bound_client("real").arm_plan_execute(execute_request())
    assert crossed.value.status_code == 409
    assert real.executions == 0

    result = registry.bound_client("sim").arm_plan_execute(execute_request())
    assert result["executed"] is True
    assert result["backendId"] == "sim"
    assert sim.executions == 1

    with pytest.raises(RobotGatewayError, match="no longer current"):
        registry.bound_client("sim").arm_plan_execute(execute_request())
    assert sim.executions == 1


def test_registry_closes_shared_clients_once() -> None:
    gateway = FakeGateway("both")
    registry = ArmBackendRegistry(
        [descriptor("sim", gateway), descriptor("real", gateway)],
        default_backend_id="sim",
    )

    registry.close()

    assert gateway.closed is True


def test_upstream_instance_restart_invalidates_a_reviewed_sim_plan() -> None:
    gateway = IdentityGateway("sim", "sim_bridge_instance_one")
    registry = ArmBackendRegistry(
        [
            ArmBackendDescriptor.create(  # type: ignore[arg-type]
                "sim",
                "SIM",
                gateway,  # type: ignore[arg-type]
                require_upstream_identity=True,
            )
        ],
        default_backend_id="sim",
    )
    client = registry.bound_client("sim")

    preview = client.arm_plan_preview(preview_request())
    gateway.instance_id = "sim_bridge_instance_two"

    assert preview["backendInstanceId"] == "sim_bridge_instance_one"
    with pytest.raises(RobotGatewayError, match="another backend") as restarted:
        client.arm_plan_execute(execute_request())
    assert restarted.value.status_code == 409
    assert gateway.executions == 0
    assert registry.public_backends()["backends"][0]["backendInstanceId"] == (
        "sim_bridge_instance_two"
    )


def test_real_plan_preview_rejects_an_in_flight_gateway_identity_change() -> None:
    gateway = MismatchedOperationIdentityGateway("real")
    registry = ArmBackendRegistry(
        [
            ArmBackendDescriptor.create(  # type: ignore[arg-type]
                "real", "REAL", gateway  # type: ignore[arg-type]
            )
        ],
        default_backend_id="real",
    )

    with pytest.raises(RobotGatewayError, match="changed while") as changed:
        registry.bound_client("real").arm_plan_preview(preview_request())

    assert changed.value.status_code == 409


def test_evidence_refreshes_upstream_identity_after_restart() -> None:
    gateway = IdentityGateway("sim", "sim_bridge_instance_one")
    registry = ArmBackendRegistry(
        [
            ArmBackendDescriptor.create(  # type: ignore[arg-type]
                "sim",
                "SIM",
                gateway,  # type: ignore[arg-type]
                require_upstream_identity=True,
            )
        ],
        default_backend_id="sim",
    )
    client = registry.bound_client("sim")

    first = client.arm_state()
    gateway.instance_id = "sim_bridge_instance_two"
    second = client.arm_state()

    assert first["backendInstanceId"] == "sim_bridge_instance_one"
    assert second["backendInstanceId"] == "sim_bridge_instance_two"


def test_unprobed_inventory_identity_is_null_not_invented() -> None:
    gateway = IdentityGateway("sim", "sim_bridge_instance_one")
    registry = ArmBackendRegistry(
        [
            ArmBackendDescriptor.create(  # type: ignore[arg-type]
                "sim",
                "SIM",
                gateway,  # type: ignore[arg-type]
                require_upstream_identity=True,
            )
        ],
        default_backend_id="sim",
    )

    assert registry.public_backends()["backends"][0]["backendInstanceId"] is None


def test_response_bound_identity_avoids_a_second_probe() -> None:
    gateway = ResponseIdentityGateway("sim", "sim_bridge_instance_one")
    registry = ArmBackendRegistry(
        [
            ArmBackendDescriptor.create(  # type: ignore[arg-type]
                "sim",
                "SIM",
                gateway,  # type: ignore[arg-type]
                require_upstream_identity=True,
            )
        ],
        default_backend_id="sim",
    )

    state = registry.bound_client("sim").arm_state()

    assert state["backendInstanceId"] == "sim_bridge_instance_one"
    assert gateway.identity_refreshes == 0


def test_direct_real_camera_frame_omits_unknown_instance_header() -> None:
    gateway = FakeGateway("real")
    registry = ArmBackendRegistry(
        [
            ArmBackendDescriptor.create(  # type: ignore[arg-type]
                "real", "REAL", gateway  # type: ignore[arg-type]
            )
        ],
        default_backend_id="real",
    )

    frame = registry.bound_client("real").camera_frame()

    assert frame.headers["X-Arm-Backend-Id"] == "real"
    assert frame.headers["X-Simulated"] == "false"
    assert "X-Arm-Backend-Instance" not in frame.headers
