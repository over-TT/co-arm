from __future__ import annotations

import io
import json
from pathlib import Path
from typing import Any

import httpx
import pytest

from arm_mcp import server


def _module3_wide_profile(*, simulated: bool = False) -> dict[str, object]:
    profile: dict[str, object] = {
        "id": "module3-wide",
        "productName": "Raspberry Pi Camera Module 3 Wide",
        "sensorModel": "imx708",
        "lensVariant": "wide",
        "nativeDimensions": {"width": 4608, "height": 2592},
        "nominalFocalLengthMm": 2.75,
        "nominalFieldOfViewDegrees": {"horizontal": 102.0, "vertical": 67.0},
        "captureProfiles": {
            "survey": {"width": 2304, "height": 1296},
            "detail": {"width": 4608, "height": 2592},
        },
    }
    if simulated:
        profile.update(
            {
                "simulatedPinholeFitAxis": "horizontal",
                "simulatedPinholeFieldOfViewDegrees": {
                    "horizontal": 102.0,
                    "vertical": 69.56998,
                },
            }
        )
    return profile


def _projection_state(camera_degrees: float = 55.0) -> dict[str, object]:
    return {
        "connection": "online",
        "bus": "online",
        "stopped": False,
        "collisionSuspected": False,
        "floorGuard": {
            "enabled": True,
            "floorMm": 40.0,
            "geometry": {
                "baseHeightMm": 60.0,
                "upperArmMm": 180.0,
                "distalMm": 220.0,
            },
        },
        "joints": [
            {"id": "joint_1", "degrees": 0.0},
            {"id": "joint_2", "degrees": 0.0},
            {"id": "joint_3", "degrees": 0.0},
            {"id": "joint_4", "degrees": camera_degrees},
        ],
    }


def _health(backend: str) -> dict[str, object]:
    return {
        "status": "ok",
        "backendId": backend,
        "backendInstanceId": f"gateway:{backend}:test-instance",
        "simulated": backend == "sim",
        "version": "test",
    }


def _state() -> dict[str, object]:
    return {
        "connection": "online",
        "bus": "online",
        "stopped": False,
        "collisionSuspected": False,
        "floorGuard": {"enabled": True, "floorMm": 40},
        "joints": [],
    }


def _configured_gateway(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    *,
    expected: str,
    actual: str,
    requests: list[httpx.Request],
) -> server.Gateway:
    token_file = tmp_path / "gateway.token"
    token_file.write_text("test-secret\n", encoding="utf-8")
    monkeypatch.setenv("ARM_GATEWAY_URL", "http://127.0.0.1:8787")
    monkeypatch.setenv("ARM_EXPECTED_BACKEND", expected)
    monkeypatch.setenv("ARM_GATEWAY_TOKEN_FILE", str(token_file))

    digest = "sha256:" + "a" * 64

    def respond(request: httpx.Request) -> httpx.Response:
        requests.append(request)
        if request.url.path == "/healthz":
            return httpx.Response(200, json=_health(actual))
        if request.url.path == "/api/robot/arm/state":
            return httpx.Response(200, json=_state())
        if request.url.path == "/api/robot/arm/plans/preview":
            return httpx.Response(
                200,
                json={
                    "planId": "armplan_abcdefgh",
                    "planDigest": digest,
                    "resolvedPose": {"joint_1": 0.0},
                    "warnings": [],
                },
            )
        if request.url.path == "/api/robot/arm/plans/execute":
            return httpx.Response(
                200,
                json={
                    "planId": "armplan_abcdefgh",
                    "executed": True,
                    "accepted": True,
                },
            )
        raise AssertionError(f"unexpected request: {request.method} {request.url.path}")

    return server.Gateway(transport=httpx.MockTransport(respond))


def test_operational_call_authenticates_health_first_and_retains_identity(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    requests: list[httpx.Request] = []
    gateway = _configured_gateway(
        tmp_path,
        monkeypatch,
        expected="sim",
        actual="sim",
        requests=requests,
    )
    monkeypatch.setattr(server, "GATEWAY", gateway)

    result = server.call_tool("arm_state", {})

    assert [request.url.path for request in requests] == [
        "/healthz",
        "/api/robot/arm/state",
    ]
    assert all(request.headers["authorization"] == "Bearer test-secret" for request in requests)
    identity = {
        "backendId": "sim",
        "backendInstanceId": "gateway:sim:test-instance",
        "simulated": True,
    }
    assert result["isError"] is False
    assert result["structuredContent"]["backend"] == identity
    assert result["structuredContent"]["data"]["backend"] == identity
    assert result["_meta"]["co-arm/backend"] == identity
    assert json.loads(result["content"][0]["text"])["backend"] == identity


@pytest.mark.parametrize(
    ("expected", "actual"),
    [("sim", "real"), ("real", "sim")],
)
def test_backend_mismatch_refuses_before_operational_request(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    expected: str,
    actual: str,
) -> None:
    requests: list[httpx.Request] = []
    gateway = _configured_gateway(
        tmp_path,
        monkeypatch,
        expected=expected,
        actual=actual,
        requests=requests,
    )
    monkeypatch.setattr(server, "GATEWAY", gateway)

    result = server.call_tool("arm_state", {})

    assert [request.url.path for request in requests] == ["/healthz"]
    assert result["isError"] is True
    assert result["structuredContent"]["error"] == {
        "code": "backend_mismatch",
        "message": (
            f"The authenticated gateway proved backend {actual!r}, but "
            f"ARM_EXPECTED_BACKEND is {expected!r}. No operational request was sent."
        ),
        "retryable": False,
    }
    assert result["structuredContent"]["backend"]["backendId"] == actual
    assert result["content"][0]["annotations"] == {
        "audience": ["assistant"],
        "priority": 1.0,
    }


def test_plan_and_apply_outputs_keep_exact_backend_identity(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    requests: list[httpx.Request] = []
    gateway = _configured_gateway(
        tmp_path,
        monkeypatch,
        expected="real",
        actual="real",
        requests=requests,
    )
    monkeypatch.setattr(server, "GATEWAY", gateway)
    digest = "sha256:" + "a" * 64

    planned = server.call_tool("arm_plan", {"base": 0})
    applied = server.call_tool(
        "arm_apply_plan",
        {"plan_id": "armplan_abcdefgh", "plan_digest": digest},
    )

    expected_paths = [
        "/healthz",
        "/api/robot/arm/plans/preview",
        "/healthz",
        "/api/robot/arm/plans/execute",
    ]
    assert [request.url.path for request in requests] == expected_paths
    for result in (planned, applied):
        assert result["isError"] is False
        assert result["structuredContent"]["backend"]["backendId"] == "real"
        assert result["structuredContent"]["data"]["backend"]["backendId"] == "real"
        assert json.loads(result["content"][0]["text"])["backend"]["backendId"] == "real"


def test_response_less_get_is_retried_once(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    token_file = tmp_path / "gateway.token"
    token_file.write_text("test-secret\n", encoding="utf-8")
    monkeypatch.setenv("ARM_GATEWAY_URL", "http://127.0.0.1:8787")
    monkeypatch.setenv("ARM_EXPECTED_BACKEND", "real")
    monkeypatch.setenv("ARM_GATEWAY_TOKEN_FILE", str(token_file))
    requests: list[str] = []

    def respond(request: httpx.Request) -> httpx.Response:
        requests.append(request.url.path)
        if request.url.path == "/healthz" and requests.count("/healthz") == 1:
            raise httpx.RemoteProtocolError("closed before response", request=request)
        if request.url.path == "/healthz":
            return httpx.Response(200, json=_health("real"))
        if request.url.path == "/api/robot/arm/state":
            return httpx.Response(200, json=_state())
        raise AssertionError(request.url.path)

    gateway = server.Gateway(transport=httpx.MockTransport(respond))

    assert gateway.json("GET", "/api/robot/arm/state")["connection"] == "online"
    assert requests == ["/healthz", "/healthz", "/api/robot/arm/state"]


@pytest.mark.parametrize(
    "path",
    [
        "/api/robot/arm/plans/execute",
        "/api/robot/arm/plans/execute-and-capture",
        "/api/robot/arm/sequences/execute",
        "/api/robot/arm/sequences/execute-and-capture",
        "/api/camera/captures",
    ],
)
def test_response_less_write_is_never_replayed_and_reports_unknown_outcome(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    path: str,
) -> None:
    token_file = tmp_path / "gateway.token"
    token_file.write_text("test-secret\n", encoding="utf-8")
    monkeypatch.setenv("ARM_GATEWAY_URL", "http://127.0.0.1:8787")
    monkeypatch.setenv("ARM_EXPECTED_BACKEND", "real")
    monkeypatch.setenv("ARM_GATEWAY_TOKEN_FILE", str(token_file))
    requests: list[str] = []

    def respond(request: httpx.Request) -> httpx.Response:
        requests.append(request.url.path)
        if request.url.path == "/healthz":
            return httpx.Response(200, json=_health("real"))
        if request.url.path == path:
            raise httpx.RemoteProtocolError("closed before response", request=request)
        raise AssertionError(request.url.path)

    gateway = server.Gateway(transport=httpx.MockTransport(respond))

    with pytest.raises(server.ArmToolError) as caught:
        gateway.request("POST", path, {})

    assert caught.value.code == "gateway_response_lost"
    assert caught.value.retryable is False
    assert caught.value.backend is not None
    assert caught.value.backend["backendId"] == "real"
    assert "outcome is unknown" in str(caught.value)
    assert "not replayed" in str(caught.value)
    assert requests == ["/healthz", path]


def test_stdio_protocol_initialize_list_and_call_return_typed_tools(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    requests: list[httpx.Request] = []
    gateway = _configured_gateway(
        tmp_path,
        monkeypatch,
        expected="sim",
        actual="sim",
        requests=requests,
    )
    monkeypatch.setattr(server, "GATEWAY", gateway)
    messages = [
        {
            "jsonrpc": "2.0",
            "id": 1,
            "method": "initialize",
            "params": {"protocolVersion": server.PROTOCOL_VERSION},
        },
        {"jsonrpc": "2.0", "method": "notifications/initialized"},
        {"jsonrpc": "2.0", "id": 2, "method": "tools/list", "params": {}},
        {
            "jsonrpc": "2.0",
            "id": 3,
            "method": "tools/call",
            "params": {"name": "arm_state", "arguments": {}},
        },
    ]
    stdin = io.StringIO("".join(json.dumps(message) + "\n" for message in messages))
    stdout = io.StringIO()

    server.serve(stdin=stdin, stdout=stdout)

    replies = [json.loads(line) for line in stdout.getvalue().splitlines()]
    assert [reply["id"] for reply in replies] == [1, 2, 3]
    assert replies[0]["result"]["protocolVersion"] == server.PROTOCOL_VERSION
    tools = {tool["name"]: tool for tool in replies[1]["result"]["tools"]}
    assert tools["arm_state"]["outputSchema"] == server._TOOL_OUTPUT_SCHEMA
    assert tools["arm_state"]["annotations"] == {
        "readOnlyHint": True,
        "destructiveHint": False,
        "idempotentHint": True,
        "openWorldHint": False,
    }
    called = replies[2]["result"]
    assert called["isError"] is False
    assert called["structuredContent"]["ok"] is True
    assert called["structuredContent"]["tool"] == "arm_state"
    assert called["structuredContent"]["backend"]["backendId"] == "sim"


class _RecordingGateway:
    def __init__(self) -> None:
        self.calls: list[tuple[str, str, dict[str, Any] | None]] = []
        self._identity = {
            "backendId": "real",
            "backendInstanceId": "gateway:real:test-instance",
            "simulated": False,
        }

    def backend_identity(self) -> dict[str, Any]:
        return dict(self._identity)

    def json(
        self,
        method: str,
        path: str,
        payload: dict[str, Any] | None = None,
        **_kwargs: Any,
    ) -> dict[str, Any]:
        self.calls.append((method, path, payload))
        if path == "/api/robot/arm/floor-guard":
            return {**_state(), "floorGuard": {"enabled": payload == {"enabled": True}}}
        if path == "/api/robot/arm/torque":
            return _state()
        raise AssertionError(f"unexpected call: {path}")


def test_high_risk_tools_require_literal_confirmation_before_gateway_call(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    gateway = _RecordingGateway()
    monkeypatch.setattr(server, "GATEWAY", gateway)

    release_refused = server.call_tool("arm_release", {})
    floor_refused = server.call_tool("arm_floor_guard", {"enabled": False})

    assert release_refused["structuredContent"]["error"]["code"] == "confirmation_required"
    assert floor_refused["structuredContent"]["error"]["code"] == "confirmation_required"
    assert gateway.calls == []

    released = server.call_tool("arm_release", {"confirmed_torque_release": True})
    disabled = server.call_tool(
        "arm_floor_guard",
        {"enabled": False, "confirmed_floor_guard_disable": True},
    )
    enabled = server.call_tool("arm_floor_guard", {"enabled": True})

    assert released["isError"] is False
    assert disabled["isError"] is False
    assert enabled["isError"] is False
    assert gateway.calls == [
        ("POST", "/api/robot/arm/torque", {"hold": []}),
        ("POST", "/api/robot/arm/floor-guard", {"enabled": False}),
        ("POST", "/api/robot/arm/floor-guard", {"enabled": True}),
    ]
    tools = {tool["name"]: tool for tool in server.TOOLS}
    assert tools["arm_release"]["inputSchema"]["required"] == [
        "confirmed_torque_release"
    ]
    assert "confirmed_floor_guard_disable" in tools["arm_floor_guard"]["inputSchema"][
        "properties"
    ]


def test_non_object_tool_arguments_return_structured_invalid_arguments() -> None:
    response = server.handle(
        {
            "jsonrpc": "2.0",
            "id": 7,
            "method": "tools/call",
            "params": {"name": "arm_state", "arguments": []},
        }
    )

    assert response is not None
    result = response["result"]
    assert result["isError"] is True
    assert result["structuredContent"]["error"]["code"] == "invalid_arguments"


def test_sim_projection_uses_effective_renderer_fov_and_bumps_contract() -> None:
    profile = server._capture_camera_profile(_module3_wide_profile(simulated=True))
    state = _projection_state()

    projection = server._camera_desk_projection(
        {"simulated": True},
        profile,
        {"before": state, "after": state},
    )

    assert server.SERVER_VERSION == "1.8.0"
    assert projection is not None
    assert projection["status"] == "available_sim_scene_model"
    assert projection["fovDeg"] == {
        "horizontal": 102.0,
        "vertical": 69.56998,
    }
    assert projection["fovProvenance"] == {
        "source": "effective_simulated_pinhole_projection",
        "fitAxis": "horizontal",
        "referenceNominalDeg": {"horizontal": 102.0, "vertical": 67.0},
    }


def test_authored_sim_projection_refuses_nominal_fov_as_renderer_truth() -> None:
    profile = server._capture_camera_profile(_module3_wide_profile())
    state = _projection_state()

    projection = server._camera_desk_projection(
        {"simulated": True},
        profile,
        {"before": state, "after": state},
    )

    assert projection is not None
    assert projection["status"] == "unavailable_sim_projection_profile"
    assert "effective pinhole field of view" in projection["reason"]
