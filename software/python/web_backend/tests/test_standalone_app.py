from pathlib import Path

from fastapi.testclient import TestClient

from web_backend.app import create_app
from web_backend.arm_control_center import ArmControlCenterService
from web_backend import runtime


def build_client(tmp_path: Path) -> TestClient:
    app = create_app(
        project_root=tmp_path,
        arm_control_center=ArmControlCenterService(project_root=tmp_path),
    )
    return TestClient(app)


def test_clean_checkout_is_readable_without_a_gateway(tmp_path: Path) -> None:
    with build_client(tmp_path) as client:
        session = client.get("/api/session")
        assert session.status_code == 200
        token = session.json()["actionToken"]
        assert isinstance(token, str) and len(token) >= 32
        assert session.headers["cache-control"] == "no-store"
        assert session.headers["pragma"] == "no-cache"

        backends = client.get("/api/arm/backends")
        assert backends.status_code == 200
        assert backends.json()["backends"] == []

def test_mutation_token_and_missing_gateway_fail_closed(tmp_path: Path) -> None:
    with build_client(tmp_path) as client:
        assert client.get("/api/arm/simple/state").status_code == 403
        token = client.get("/api/session").json()["actionToken"]
        response = client.get(
            "/api/arm/simple/state",
            headers={"X-Co-Arm-Token": token},
        )
        assert response.status_code == 503
        assert "No ARM gateway" in response.json()["detail"]


def test_foreign_host_cannot_read_the_action_token(tmp_path: Path) -> None:
    with build_client(tmp_path) as client:
        response = client.get("/api/session", headers={"Host": "attacker.example"})
        assert response.status_code == 400


def test_dashboard_cli_refuses_to_claim_missing_frontend_assets(
    tmp_path: Path,
    monkeypatch,
    capsys,
) -> None:
    def unexpected_server_start(*_args, **_kwargs) -> None:
        raise AssertionError("uvicorn must not start without a built dashboard")

    monkeypatch.setattr(runtime.uvicorn, "run", unexpected_server_start)

    result = runtime.main(
        ["--static-directory", str(tmp_path / "missing-dist"), "--no-access-log"]
    )

    assert result == 2
    error = capsys.readouterr().err
    assert "not bundled in the Python wheel" in error
    assert "--static-directory" in error
