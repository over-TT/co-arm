from __future__ import annotations

import os
from pathlib import Path
import stat

from fastapi.testclient import TestClient
import pytest

from robot_gateway import runtime


TOKEN = "correct-token-" + ("a" * 40)


@pytest.fixture
def token_file(tmp_path: Path) -> Path:
    path = tmp_path / "gateway.token"
    path.write_text(f"  {TOKEN}\n", encoding="utf-8")
    return path


@pytest.fixture
def client(token_file: Path) -> TestClient:
    with TestClient(runtime.create_app(token_file=token_file)) as test_client:
        yield test_client


def test_health_is_unauthenticated_and_contains_no_secret(client: TestClient) -> None:
    response = client.get("/healthz")

    assert response.status_code == 200
    document = response.json()
    assert document["status"] == "ok"
    assert document["backendId"] == "real"
    assert document["simulated"] is False
    assert document["version"] == "robot-gateway-v1"
    assert document["backendInstanceId"].startswith("gateway:")
    assert TOKEN not in response.text
    assert all(TOKEN not in value for value in response.headers.values())


def test_every_response_carries_the_same_gateway_process_identity(
    client: TestClient,
) -> None:
    health = client.get("/healthz")
    instance = health.json()["backendInstanceId"]
    assert health.headers[runtime.GATEWAY_INSTANCE_HEADER] == instance
    for path, headers, status_code in (
        ("/api/robot/arm/state", {"Authorization": f"Bearer {TOKEN}"}, 200),
        ("/api/robot/physical/arm/status", {"Authorization": f"Bearer {TOKEN}"}, 200),
        ("/api/robot/arm/state", {}, 401),
        ("/missing-route", {}, 404),
    ):
        response = client.get(path, headers=headers)
        assert response.status_code == status_code
        assert response.headers[runtime.GATEWAY_INSTANCE_HEADER] == instance


def test_new_gateway_application_gets_a_distinct_process_identity(
    token_file: Path,
) -> None:
    with TestClient(runtime.create_app(token_file=token_file)) as first:
        first_instance = first.get("/healthz").json()["backendInstanceId"]
    with TestClient(runtime.create_app(token_file=token_file)) as second:
        second_instance = second.get("/healthz").json()["backendInstanceId"]
    assert first_instance != second_instance


@pytest.mark.parametrize(
    "headers",
    [
        {},
        {"Authorization": "Bearer wrong-token-that-is-long-enough-000000"},
    ],
)
def test_robot_api_rejects_missing_or_wrong_token(
    client: TestClient, headers: dict[str, str]
) -> None:
    response = client.get("/api/robot/arm/state", headers=headers)

    assert response.status_code == 401
    assert response.json() == {"detail": "Unauthorized"}
    assert response.headers["www-authenticate"] == "Bearer"
    assert TOKEN not in response.text


def test_core_arm_routes_accept_the_right_token(client: TestClient) -> None:
    headers = {"Authorization": f"Bearer {TOKEN}"}

    simple = client.get("/api/robot/arm/state", headers=headers)
    physical = client.get("/api/robot/physical/arm/status", headers=headers)

    assert simple.status_code == 200
    assert [joint["id"] for joint in simple.json()["joints"]] == [
        "joint_1",
        "joint_2",
        "joint_3",
        "joint_4",
    ]
    assert physical.status_code == 200
    assert physical.json()["mode"] == "physical"
    assert "controller" in physical.json()["connections"]


@pytest.mark.parametrize(
    "legacy_route",
    [
        "/api/robot/status",
        "/api/robot/pose",
        "/api/arm/calibration",
    ],
)
def test_legacy_generic_simulator_routes_are_absent(
    client: TestClient, legacy_route: str
) -> None:
    response = client.get(
        legacy_route,
        headers={"Authorization": f"Bearer {TOKEN}"},
    )

    assert response.status_code == 404


def test_camera_routes_are_absent_unless_explicitly_configured(
    client: TestClient,
) -> None:
    response = client.get(
        "/api/camera/status",
        headers={"Authorization": f"Bearer {TOKEN}"},
    )

    assert response.status_code == 404


def test_load_token_strips_surrounding_whitespace(token_file: Path) -> None:
    assert runtime.load_token(token_file) == TOKEN


@pytest.mark.parametrize(
    "contents",
    [
        "x" * 31,
        "x" * 257,
        ("x" * 32) + " " + ("y" * 8),
        ("x" * 32) + "\x00",
        ("x" * 32) + "\n" + "y",
    ],
)
def test_load_token_rejects_invalid_content(tmp_path: Path, contents: str) -> None:
    path = tmp_path / "invalid.token"
    path.write_text(contents, encoding="utf-8")

    with pytest.raises(runtime.GatewayConfigurationError) as error:
        runtime.load_token(path)

    assert contents not in str(error.value)


def test_load_token_read_error_is_sanitized(tmp_path: Path) -> None:
    missing = tmp_path / "private-secret-name.token"

    with pytest.raises(runtime.GatewayConfigurationError) as error:
        runtime.load_token(missing)

    assert str(missing) not in str(error.value)


def test_generate_token_is_exclusive_and_owner_only_where_supported(
    tmp_path: Path,
) -> None:
    path = tmp_path / "new.token"

    assert runtime.generate_token_file(path) == path
    original = path.read_bytes()
    generated = runtime.load_token(path)

    assert len(generated) == 64
    assert all(character in "0123456789abcdef" for character in generated)
    if os.name == "posix":
        assert stat.S_IMODE(path.stat().st_mode) == 0o600

    with pytest.raises(runtime.GatewayConfigurationError, match="overwrite"):
        runtime.generate_token_file(path)
    assert path.read_bytes() == original


@pytest.mark.parametrize("host", ["127.0.0.1", "localhost", "::1"])
def test_validate_bind_host_accepts_only_named_loopbacks(host: str) -> None:
    assert runtime.validate_bind_host(host) == host


@pytest.mark.parametrize(
    "host",
    ["0.0.0.0", "192.168.1.20", "example.test", "LOCALHOST", "127.0.0.2", ""],
)
def test_validate_bind_host_rejects_non_allowlisted_values(host: str) -> None:
    with pytest.raises(runtime.GatewayConfigurationError, match="SSH tunnel"):
        runtime.validate_bind_host(host)


def test_cli_runs_the_core_gateway_without_a_simulator_flag(
    monkeypatch: pytest.MonkeyPatch, token_file: Path
) -> None:
    app_arguments: list[dict[str, object]] = []
    served_apps: list[object] = []

    def fake_create_app(**kwargs: object) -> object:
        app_arguments.append(kwargs)
        return object()

    def fake_run(app: object, **_: object) -> None:
        served_apps.append(app)

    monkeypatch.setattr(runtime, "create_app", fake_create_app)
    monkeypatch.setattr(runtime.uvicorn, "run", fake_run)

    assert runtime.main(["serve", "--token-file", str(token_file)]) == 0
    assert len(app_arguments) == 1
    assert "simulator" not in app_arguments[0]
    assert len(served_apps) == 1


def test_cli_rejects_the_removed_simulator_flag(token_file: Path) -> None:
    with pytest.raises(SystemExit) as exit_info:
        runtime.main(
            ["serve", "--simulator", "--token-file", str(token_file)]
        )

    assert exit_info.value.code == 2


def test_cli_rejects_lan_bind(
    monkeypatch: pytest.MonkeyPatch, token_file: Path
) -> None:
    called = False

    def unexpected_run(*args: object, **kwargs: object) -> None:
        nonlocal called
        called = True

    monkeypatch.setattr(runtime.uvicorn, "run", unexpected_run)

    with pytest.raises(SystemExit) as exit_info:
        runtime.main(
            [
                "serve",
                "--token-file",
                str(token_file),
                "--host",
                "0.0.0.0",
            ]
        )

    assert exit_info.value.code == 2
    assert called is False


def test_cli_instantiates_camera_only_for_explicit_picamera2_selection(
    monkeypatch: pytest.MonkeyPatch, token_file: Path
) -> None:
    providers: list[object] = []
    app_arguments: list[dict[str, object]] = []
    served_apps: list[object] = []

    class FakeCameraProvider:
        pass

    def provider_factory() -> FakeCameraProvider:
        provider = FakeCameraProvider()
        providers.append(provider)
        return provider

    def fake_create_app(**kwargs: object) -> object:
        app_arguments.append(kwargs)
        return object()

    def fake_run(app: object, **_: object) -> None:
        served_apps.append(app)

    monkeypatch.setattr(runtime, "PiCameraProvider", provider_factory)
    monkeypatch.setattr(runtime, "create_app", fake_create_app)
    monkeypatch.setattr(runtime.uvicorn, "run", fake_run)

    assert runtime.main(
        ["serve", "--token-file", str(token_file)]
    ) == 0
    assert providers == []
    assert app_arguments[-1]["camera_provider"] is None

    assert runtime.main(
        [
            "serve",
            "--token-file",
            str(token_file),
            "--camera",
            "picamera2",
        ]
    ) == 0
    assert len(providers) == 1
    assert app_arguments[-1]["camera_provider"] is providers[0]
    assert len(served_apps) == 2


def test_cli_can_enable_the_serial_arm_controller(
    monkeypatch: pytest.MonkeyPatch, token_file: Path, tmp_path: Path
) -> None:
    controllers: list[object] = []
    app_arguments: list[dict[str, object]] = []

    class FakeSerialController:
        def __init__(self, port: str) -> None:
            self.port = port
            controllers.append(self)

    def fake_create_app(**kwargs: object) -> object:
        app_arguments.append(kwargs)
        return object()

    monkeypatch.setattr(runtime, "SerialArmController", FakeSerialController)
    monkeypatch.setattr(runtime, "create_app", fake_create_app)
    monkeypatch.setattr(runtime.uvicorn, "run", lambda *_args, **_kwargs: None)
    state_dir = tmp_path / "arm-state"

    result = runtime.main(
        [
            "serve",
            "--token-file",
            str(token_file),
            "--arm-controller-port",
            "/dev/serial0",
            "--arm-state-dir",
            str(state_dir),
        ]
    )

    assert result == 0
    assert len(controllers) == 1
    assert controllers[0].port == "/dev/serial0"
    assert "simulator" not in app_arguments[0]
    assert app_arguments[0]["arm_controller"] is controllers[0]
    assert app_arguments[0]["arm_state_dir"] == state_dir
