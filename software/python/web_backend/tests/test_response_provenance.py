from __future__ import annotations

from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from threading import Event

import httpx
import pytest

from web_backend import robot_gateway_client as gateway_module
from web_backend.arm_backends import ArmBackendDescriptor, ArmBackendRegistry
from web_backend.robot_gateway_client import RobotGatewayClient


def _token_file(tmp_path: Path) -> Path:
    path = tmp_path / "gateway.token"
    path.write_text("test-token-" + "x" * 40, encoding="utf-8")
    return path


def _normalization_payload(path: str, *, unsupported: bool = False) -> dict[str, object]:
    autofocus = {
        "capability": "unsupported" if unsupported else "supported",
        "mode": "fixed" if unsupported else "continuous",
        "state": "fixed" if unsupported else "focused",
        "range": "unavailable" if unsupported else "full",
    }
    if path == "/api/camera/status":
        return {
            "simulated": False, "readOnly": True, "state": "started",
            "available": True, "latestFrameId": None, "historySize": 0,
            "historyLimit": 8, "retainedBytes": 0, "byteLimit": 1024,
            "cameraId": "test-camera", "sensorModel": "imx708",
            "identityConfidence": "configured_candidate", "autofocus": autofocus,
        }
    if path == "/api/camera/autofocus":
        return {
            "simulated": False, "physicalArmMotion": False, "attempted": True,
            "result": "focused", "autofocus": autofocus,
        }
    if path in {"/api/camera/captures", "/api/camera/observations/latest"}:
        return {
            "simulated": False, "readOnly": True, "source": "picamera2",
            "frameId": "frame_old", "cameraId": "test-camera", "sensorModel": "imx708",
            "identityConfidence": "configured_candidate", "capturedAt": "2026-09-04T12:00:00Z",
            "ageSeconds": 0, "dimensions": {"width": 640, "height": 480},
            "captureProfile": "detail", "byteCount": 1234,
            "sha256": "a" * 64, "contentSha256": "sha256:" + "a" * 64,
            "stateRevision": 1,
        }
    if path == "/api/robot/physical/arm/calibration/profile":
        return {
            "mode": "physical", "motionState": "disarmed", "committed": False,
            "profileRevision": 0, "profile": None, "profileHash": None,
        }
    if path == "/api/robot/physical/arm/status":
        return {
            "mode": "physical", "motionState": "disarmed", "torqueState": "off",
            "connections": {
                "pi": {"state": "online", "detail": "connected"},
                "controller": {"state": "offline", "detail": "not connected"},
                "bus": {"state": "unknown", "detail": "not scanned", "baud": 1000000},
                "servos": {"state": "unknown", "detail": "not scanned",
                           "expectedIds": [1, 2, 3, 4], "respondingIds": []},
            },
            "servos": [], "blockers": [], "nextAction": "Inspect controller",
            "stop": {"state": "not_latched", "detail": "clear"},
            "hardwareEstop": "not_detected",
        }
    raise AssertionError(path)


@pytest.mark.parametrize(
    ("method_name", "normalizer_name", "unsupported"),
    [
        ("camera_status", "_camera_status", False),
        ("capture_camera", "_camera_observation", False),
        ("latest_camera_observation", "_camera_observation", False),
        ("autofocus_camera", "_camera_autofocus_attempt", False),
        ("autofocus_camera", "_camera_status", True),
        ("physical_status", "_physical_status", False),
        ("physical_profile", "_physical_profile_response", False),
    ],
)
def test_normalized_response_keeps_its_own_instance_during_concurrent_restart(
    tmp_path: Path,
    monkeypatch: pytest.MonkeyPatch,
    method_name: str,
    normalizer_name: str,
    unsupported: bool,
) -> None:
    entered = Event()
    resume = Event()
    original = getattr(gateway_module, normalizer_name)
    calls: list[str] = []

    def delayed_normalizer(document, *args, **kwargs):
        entered.set()
        assert resume.wait(timeout=5)
        return original(document, *args, **kwargs)

    def upstream(request: httpx.Request) -> httpx.Response:
        calls.append(request.url.path)
        if request.url.path == "/api/robot/arm/state":
            document = {"connection": "online"}
            instance = "gateway:new-process"
        else:
            document = _normalization_payload(request.url.path, unsupported=unsupported)
            instance = "gateway:old-process"
        return httpx.Response(
            200, json=document, headers={"X-Robot-Gateway-Instance": instance}
        )

    monkeypatch.setattr(gateway_module, normalizer_name, delayed_normalizer)
    client = RobotGatewayClient(
        "http://127.0.0.1:8787", _token_file(tmp_path),
        transport=httpx.MockTransport(upstream),
    )
    registry = ArmBackendRegistry([ArmBackendDescriptor.create("real", "REAL", client)])
    bound = registry.bound_client("real")
    try:
        with ThreadPoolExecutor(max_workers=1) as pool:
            future = pool.submit(getattr(bound, method_name))
            try:
                assert entered.wait(timeout=5)
                assert bound.arm_state()["backendInstanceId"] == "gateway:new-process"
            finally:
                resume.set()
            response = future.result(timeout=5)
        assert response["backendId"] == "real"
        assert response["backendInstanceId"] == "gateway:old-process"
        assert response["simulated"] is False
        assert "/healthz" not in calls
        if unsupported:
            assert response["attempted"] is False
            assert "/api/camera/autofocus" not in calls
    finally:
        registry.close()
