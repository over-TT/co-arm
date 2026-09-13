from __future__ import annotations

from pathlib import Path
import hashlib

import httpx
import pytest

from web_backend.arm_backends import ArmBackendDescriptor, ArmBackendRegistry
from web_backend.robot_gateway_client import (
    RobotGatewayClient,
    RobotGatewayError,
    SimpleArmPlanExecuteRequest,
    SimpleArmPlanPreviewRequest,
    SimpleArmSequenceExecuteCaptureRequest,
    SimpleArmSequencePreviewRequest,
)


def _token_file(tmp_path: Path) -> Path:
    path = tmp_path / "gateway.token"
    path.write_text("test-token-" + "x" * 40, encoding="utf-8")
    return path


def test_real_gateway_header_keeps_state_preview_and_execute_on_one_process_identity(
    tmp_path: Path,
) -> None:
    gateway_instance = "gateway:0123456789abcdef0123456789abcdef"
    controller_boot = 0

    def upstream(request: httpx.Request) -> httpx.Response:
        nonlocal controller_boot
        controller_boot += 1
        if request.url.path == "/api/robot/arm/state":
            document: dict[str, object] = {
                "controller": {"bootId": f"boot_{controller_boot}"},
                "connection": "online",
            }
        elif request.url.path == "/api/robot/arm/plans/preview":
            document = {
                "planId": "armplan_stable1234",
                "planDigest": "sha256:" + "a" * 64,
                "expiresInMs": 30_000,
                "controller": {"bootId": f"boot_{controller_boot}"},
            }
        elif request.url.path == "/api/robot/arm/plans/execute":
            document = {
                "executed": True,
                "controller": {"bootId": f"boot_{controller_boot}"},
            }
        else:
            raise AssertionError(request.url.path)
        return httpx.Response(
            200,
            json=document,
            headers={"X-Robot-Gateway-Instance": gateway_instance},
        )

    client = RobotGatewayClient(
        "http://127.0.0.1:8787",
        _token_file(tmp_path),
        transport=httpx.MockTransport(upstream),
    )
    preview_request = SimpleArmPlanPreviewRequest.model_validate(
        {"targets": {"joint_4": 10.0}}
    )
    execute_request = SimpleArmPlanExecuteRequest.model_validate(
        {
            "planId": "armplan_stable1234",
            "planDigest": "sha256:" + "a" * 64,
        }
    )

    state = client.arm_state()
    preview = client.arm_plan_preview(preview_request)
    executed = client.arm_plan_execute(execute_request)

    assert {
        state["backendInstanceId"],
        preview["backendInstanceId"],
        executed["backendInstanceId"],
    } == {gateway_instance}
    for response in (state, preview, executed):
        assert response["backendId"] == "real"
        assert response["simulated"] is False
    assert client.observe_backend_identity(state) == {
        "backendId": "real",
        "backendInstanceId": gateway_instance,
        "simulated": False,
    }
    assert controller_boot == 3
    client.close()


@pytest.mark.parametrize("later_header", [None, "", "controller:unexpected"])
def test_real_gateway_invalid_header_after_negotiation_fails_closed(
    tmp_path: Path,
    later_header: str | None,
) -> None:
    calls = 0
    gateway_instance = "gateway:0123456789abcdef0123456789abcdef"

    def upstream(_: httpx.Request) -> httpx.Response:
        nonlocal calls
        calls += 1
        headers = (
            {"X-Robot-Gateway-Instance": gateway_instance}
            if calls == 1
            else {} if later_header is None
            else {"X-Robot-Gateway-Instance": later_header}
        )
        return httpx.Response(
            200,
            json={
                "connection": "online",
                "controller": {"bootId": f"controller_boot_{calls}"},
            },
            headers=headers,
        )

    client = RobotGatewayClient(
        "http://127.0.0.1:8787",
        _token_file(tmp_path),
        transport=httpx.MockTransport(upstream),
    )

    first = client.arm_state()
    assert first["backendInstanceId"] == gateway_instance
    with pytest.raises(RobotGatewayError, match="process provenance") as omitted:
        client.arm_state()
    assert omitted.value.status_code == 502
    client.close()


def test_sim_response_uses_bridge_identity_even_with_gateway_process_header(
    tmp_path: Path,
) -> None:
    bridge_identity = {
        "backendId": "sim",
        "backendInstanceId": "isaac:bridge-instance",
        "simulated": True,
    }
    client = RobotGatewayClient(
        "http://127.0.0.1:8788",
        _token_file(tmp_path),
        expected_simulated=True,
        transport=httpx.MockTransport(
            lambda _: httpx.Response(
                200,
                json=bridge_identity,
                headers={"X-Robot-Gateway-Instance": "gateway:unrelated-process"},
            )
        ),
    )
    try:
        assert client.backend_identity(refresh=True) == bridge_identity
        assert client.arm_state() == bridge_identity
    finally:
        client.close()


@pytest.mark.parametrize("later_header", [None, "invalid", "gateway:second-process"])
def test_camera_frame_carries_and_enforces_negotiated_process_identity(
    tmp_path: Path,
    later_header: str | None,
) -> None:
    content = b"\xff\xd8identity-test\xff\xd9"
    digest = "sha256:" + hashlib.sha256(content).hexdigest()
    calls = 0

    def upstream(request: httpx.Request) -> httpx.Response:
        nonlocal calls
        calls += 1
        if calls == 1:
            return httpx.Response(
                200,
                json={"connection": "online"},
                headers={"X-Robot-Gateway-Instance": "gateway:first-process"},
            )
        assert request.url.path == "/api/camera/frames/frame_test"
        headers = {
            "Content-Type": "image/jpeg",
            "X-Frame-Id": "frame_test",
            "X-Content-SHA256": digest,
            "ETag": f'"{digest}"',
            "X-Simulated": "false",
            "X-Read-Only": "true",
            "X-Camera-Source": "picamera2",
        }
        if later_header is not None:
            headers["X-Robot-Gateway-Instance"] = later_header
        return httpx.Response(200, content=content, headers=headers)

    client = RobotGatewayClient(
        "http://127.0.0.1:8787",
        _token_file(tmp_path),
        transport=httpx.MockTransport(upstream),
    )
    try:
        client.arm_state()
        if later_header == "gateway:second-process":
            frame = client.camera_frame("frame_test")
            assert frame.content == content
            assert frame.headers["X-Arm-Backend-Instance"] == later_header
            assert client.backend_identity()["backendInstanceId"] == later_header
        else:
            with pytest.raises(RobotGatewayError, match="process provenance"):
                client.camera_frame("frame_test")
    finally:
        client.close()


@pytest.mark.parametrize(
    ("kind", "operation"),
    [("plan", "preview"), ("plan", "execute"), ("sequence", "preview"),
     ("sequence", "execute"), ("sequence", "execute-and-capture")],
)
def test_reviewed_operation_rejects_in_flight_gateway_restart(
    tmp_path: Path,
    kind: str,
    operation: str,
) -> None:
    instance = "gateway:first-process"
    requests: list[str] = []
    artifact = (
        {"planId": "armplan_reviewed123", "planDigest": "sha256:" + "a" * 64}
        if kind == "plan"
        else {"sequenceId": "armseq_reviewed123", "sequenceDigest": "sha256:" + "a" * 64}
    )
    target_path = f"/api/robot/arm/{kind}s/{operation}"

    def upstream(request: httpx.Request) -> httpx.Response:
        nonlocal instance
        requests.append(request.url.path)
        if request.url.path == target_path:
            instance = "gateway:second-process"
        return httpx.Response(
            200,
            json={**artifact, "expiresInMs": 30_000},
            headers={"X-Robot-Gateway-Instance": instance},
        )

    client = RobotGatewayClient(
        "http://127.0.0.1:8787",
        _token_file(tmp_path),
        transport=httpx.MockTransport(upstream),
    )
    registry = ArmBackendRegistry(
        [ArmBackendDescriptor.create("real", "REAL", client)],
    )
    bound = registry.bound_client("real")
    preview = (
        SimpleArmPlanPreviewRequest(targets={"joint_4": 10.0})
        if kind == "plan"
        else SimpleArmSequencePreviewRequest(
            waypoints=[{"targets": {"joint_4": 10.0}}, {"targets": {"joint_4": 0.0}}]
        )
    )
    execute = (
        SimpleArmPlanExecuteRequest(**artifact)
        if kind == "plan"
        else SimpleArmSequenceExecuteCaptureRequest(**artifact)
    )
    try:
        if operation != "preview":
            getattr(bound, f"arm_{kind}_preview")(preview)
        method = getattr(bound, f"arm_{kind}_{operation.replace('-', '_')}")
        with pytest.raises(RobotGatewayError, match="changed while") as changed:
            method(preview if operation == "preview" else execute)
        assert changed.value.status_code == 409
        assert requests.count(target_path) == 1
        if operation != "preview":
            # The old review cannot be replayed against the new process.
            with pytest.raises(RobotGatewayError, match="another backend"):
                method(execute)
            assert requests.count(target_path) == 1
    finally:
        registry.close()


def test_legacy_real_gateway_without_process_header_keeps_controller_boot_fallback(
    tmp_path: Path,
) -> None:
    def upstream(request: httpx.Request) -> httpx.Response:
        if request.url.path == "/healthz":
            return httpx.Response(200, json={"status": "ok"})
        assert request.url.path == "/api/robot/arm/state"
        return httpx.Response(
            200,
            json={"controller": {"bootId": "legacy_boot_1234"}},
        )

    client = RobotGatewayClient(
        "http://127.0.0.1:8787",
        _token_file(tmp_path),
        transport=httpx.MockTransport(upstream),
    )

    assert client.backend_identity(refresh=True) == {
        "backendId": "real",
        "backendInstanceId": "controller:legacy_boot_1234",
        "simulated": False,
    }
    client.close()


def test_real_gateway_restart_is_propagated_on_the_next_independent_state(
    tmp_path: Path,
) -> None:
    instances = iter(
        (
            "gateway:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
            "gateway:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
        )
    )

    def upstream(_: httpx.Request) -> httpx.Response:
        return httpx.Response(
            200,
            json={"connection": "online"},
            headers={"X-Robot-Gateway-Instance": next(instances)},
        )

    client = RobotGatewayClient(
        "http://127.0.0.1:8787",
        _token_file(tmp_path),
        transport=httpx.MockTransport(upstream),
    )

    first = client.arm_state()
    second = client.arm_state()

    assert first["backendInstanceId"] == (
        "gateway:aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa"
    )
    assert second["backendInstanceId"] == (
        "gateway:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb"
    )
    assert client.backend_identity() == {
        "backendId": "real",
        "backendInstanceId": "gateway:bbbbbbbbbbbbbbbbbbbbbbbbbbbbbbbb",
        "simulated": False,
    }
    client.close()


@pytest.mark.parametrize(
    ("header_instance", "body_instance"),
    [
        ("not-a-gateway-instance", None),
        (
            "gateway:0123456789abcdef0123456789abcdef",
            "gateway:fedcba9876543210fedcba9876543210",
        ),
    ],
)
def test_real_gateway_response_rejects_malformed_or_mismatched_process_provenance(
    tmp_path: Path,
    header_instance: str,
    body_instance: str | None,
) -> None:
    document: dict[str, object] = {"connection": "online"}
    if body_instance is not None:
        document.update(
            {
                "backendId": "real",
                "backendInstanceId": body_instance,
                "simulated": False,
            }
        )
    client = RobotGatewayClient(
        "http://127.0.0.1:8787",
        _token_file(tmp_path),
        transport=httpx.MockTransport(
            lambda _: httpx.Response(
                200,
                json=document,
                headers={"X-Robot-Gateway-Instance": header_instance},
            )
        ),
    )

    with pytest.raises(RobotGatewayError, match="process provenance") as caught:
        client.arm_state()
    assert caught.value.status_code == 502
    client.close()


@pytest.mark.parametrize(
    ("field", "value"),
    [
        ("backendId", "sim"),
        ("backendInstanceId", None),
        ("simulated", True),
    ],
)
def test_real_gateway_header_validates_each_present_body_provenance_field(
    tmp_path: Path,
    field: str,
    value: object,
) -> None:
    client = RobotGatewayClient(
        "http://127.0.0.1:8787",
        _token_file(tmp_path),
        transport=httpx.MockTransport(
            lambda _: httpx.Response(
                200,
                json={"connection": "online", field: value},
                headers={
                    "X-Robot-Gateway-Instance": (
                        "gateway:0123456789abcdef0123456789abcdef"
                    )
                },
            )
        ),
    )

    with pytest.raises(RobotGatewayError, match="mismatched process provenance"):
        client.arm_state()
    client.close()
