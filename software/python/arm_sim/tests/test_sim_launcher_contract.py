from __future__ import annotations

import importlib.util
from pathlib import Path
import socket
import sys
import threading

import pytest


PROJECT_ROOT = Path(__file__).resolve().parents[3]
PYTHON_ROOT = PROJECT_ROOT / "python"
LAUNCHER = PROJECT_ROOT / "operations" / "scripts" / "run_arm_sim.py"


def _load_launcher():
    name = "arm_sim_test_run_arm_sim"
    specification = importlib.util.spec_from_file_location(name, LAUNCHER)
    assert specification is not None and specification.loader is not None
    module = importlib.util.module_from_spec(specification)
    sys.modules[name] = module
    specification.loader.exec_module(module)
    return module


def _redirect_runtime(module, root: Path) -> None:
    module.PROJECT_ROOT = root
    module.PYTHON_ROOT = PYTHON_ROOT
    module.RUNTIME_ROOT = root / "runtime" / "arm-sim"
    module.STATE_ROOT = module.RUNTIME_ROOT / "state"
    module.SCENE_USD = module.RUNTIME_ROOT / "smoke_scene.usda"
    module.BRIDGE_TOKEN = module.RUNTIME_ROOT / "bridge.token"
    module.GATEWAY_TOKEN = module.RUNTIME_ROOT / "gateway.token"
    module.BRIDGE_LOG_OUT = module.RUNTIME_ROOT / "isaac-bridge.out.log"
    module.BRIDGE_LOG_ERR = module.RUNTIME_ROOT / "isaac-bridge.err.log"
    module.GATEWAY_LOG_OUT = module.RUNTIME_ROOT / "sim-gateway.out.log"
    module.GATEWAY_LOG_ERR = module.RUNTIME_ROOT / "sim-gateway.err.log"
    module.BRIDGE_SCRIPT = PYTHON_ROOT / "arm_sim" / "isaac" / "bridge_server.py"


def test_launcher_creates_distinct_simulator_only_tokens_without_overwrite(
    tmp_path: Path,
) -> None:
    module = _load_launcher()
    _redirect_runtime(module, tmp_path)

    bridge, gateway = module._ensure_tokens()
    before = (bridge.read_bytes(), gateway.read_bytes())
    bridge_again, gateway_again = module._ensure_tokens()

    assert len({bridge, gateway}) == 2
    assert len(set(before)) == 2
    assert all(32 <= len(value.strip()) <= 256 for value in before)
    assert bridge_again.read_bytes() == before[0]
    assert gateway_again.read_bytes() == before[1]


def test_launcher_refuses_paths_outside_its_project_root(tmp_path: Path) -> None:
    module = _load_launcher()
    root = tmp_path / "project"
    root.mkdir()
    _redirect_runtime(module, root)

    assert module._project_path(root / "runtime") == (root / "runtime").resolve()
    with pytest.raises(RuntimeError, match="escapes"):
        module._project_path(tmp_path / "outside")


def test_launcher_contract_pins_loopback_ports_and_real_entrypoints() -> None:
    module = _load_launcher()
    source = LAUNCHER.read_text(encoding="utf-8")

    assert module.BRIDGE_PORT == 8790
    assert module.GATEWAY_PORT == 8788
    assert module.CONTROL_PORT == 8791
    assert module.SCENE_USD == PROJECT_ROOT / "runtime" / "arm-sim" / "smoke_scene.usda"
    assert module.ISAAC_PYTHON is None
    assert module.BRIDGE_SCRIPT == PROJECT_ROOT / "python" / "arm_sim" / "isaac" / "bridge_server.py"
    assert 'os.environ.get("ISAAC_PYTHON", "").strip()' in source
    assert source.count('"--token-file"') == 1
    assert 'f"http://127.0.0.1:{GATEWAY_PORT}/healthz"' in source
    assert 'gateway_health.get("backendInstanceId") != bridge_instance_id' in source
    assert "from arm_sim.camera_profile import load_camera_profile" in source
    assert 'document.get("sensorModel") != camera_profile.sensor_model' in source
    assert "isaac-renderer-provisional" not in source


def test_launcher_gui_mode_keeps_the_controlled_bridge_and_disables_headless() -> None:
    module = _load_launcher()
    bridge_token = Path("bridge.token")
    isaac_python = Path("isaac-python.bat")

    headless = module._bridge_command(
        bridge_token,
        gui=False,
        isaac_python=isaac_python,
    )
    visible = module._bridge_command(
        bridge_token,
        gui=True,
        isaac_python=isaac_python,
    )

    assert headless[0] == str(isaac_python)
    assert headless[headless.index("--token-file") + 1] == str(bridge_token)
    assert Path(headless[headless.index("--scene") + 1]) == module.SCENE_USD.resolve()
    assert "--no-headless" not in headless
    assert visible[:-1] == headless
    assert visible[-1] == "--no-headless"
    assert module._parse_args([]).gui is False
    assert module._parse_args(["--gui"]).gui is True


def test_authenticated_supervisor_stop_channel_is_bounded_to_loopback(
    tmp_path: Path,
) -> None:
    module = _load_launcher()
    _redirect_runtime(module, tmp_path)
    bridge_token, _gateway_token = module._ensure_tokens()
    token = bridge_token.read_text(encoding="utf-8").strip()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        module.CONTROL_PORT = probe.getsockname()[1]
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        module.BRIDGE_PORT = probe.getsockname()[1]
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        module.GATEWAY_PORT = probe.getsockname()[1]

    stopped = threading.Event()
    ready = threading.Event()
    source_digest = f"sha256:{'a' * 64}"
    server = threading.Thread(
        target=module._control_loop,
        kwargs={
            "token": token,
            "source_digest": source_digest,
            "stop_requested": stopped,
            "ready": ready,
        },
        daemon=True,
    )
    server.start()

    assert ready.wait(2)
    assert module._request_control(bridge_token, "ping") == {
        "status": "ok",
        "backendId": "sim",
        "simulated": True,
        "sourceDigest": source_digest,
        "bridgePort": module.BRIDGE_PORT,
        "gatewayPort": module.GATEWAY_PORT,
        "controlPort": module.CONTROL_PORT,
    }
    assert module._request_remote_stop(bridge_token) is True
    assert stopped.wait(2)
    server.join(timeout=2)


def test_existing_stack_requires_authenticated_supervisor_and_current_source(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    module = _load_launcher()
    _redirect_runtime(module, tmp_path)
    bridge_token, _gateway_token = module._ensure_tokens()
    token = bridge_token.read_text(encoding="utf-8").strip()
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as probe:
        probe.bind(("127.0.0.1", 0))
        module.CONTROL_PORT = probe.getsockname()[1]
    with pytest.raises(RuntimeError, match="authenticated simulator supervisor"):
        module._validate_supervisor_identity(bridge_token)

    stale_digest = f"sha256:{'a' * 64}"
    current_digest = f"sha256:{'b' * 64}"
    monkeypatch.setattr(module, "_current_source_digest", lambda: current_digest)
    stopped = threading.Event()
    ready = threading.Event()
    server = threading.Thread(
        target=module._control_loop,
        kwargs={
            "token": token,
            "source_digest": stale_digest,
            "stop_requested": stopped,
            "ready": ready,
        },
        daemon=True,
    )
    server.start()
    assert ready.wait(2)
    try:
        with pytest.raises(RuntimeError, match="stale source"):
            module._validate_supervisor_identity(bridge_token)
    finally:
        module._request_control(bridge_token, "stop")
        assert stopped.wait(2)
        server.join(timeout=2)


def test_authenticated_existing_startup_waits_for_both_data_endpoints(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    module = _load_launcher()
    _redirect_runtime(module, tmp_path)
    bridge_token, gateway_token = module._ensure_tokens()
    startup_round = {"value": 0}
    identity_calls: list[str] = []

    def listener_ready(port: int, timeout: float = 0.25) -> bool:
        del timeout
        if port == module.CONTROL_PORT:
            return True
        if port == module.BRIDGE_PORT:
            return True
        if port == module.GATEWAY_PORT:
            return startup_round["value"] >= 1
        raise AssertionError(f"unexpected port {port}")

    monkeypatch.setattr(module, "_listener_ready", listener_ready)
    monkeypatch.setattr(
        module,
        "_validate_supervisor_identity",
        lambda token: identity_calls.append(f"supervisor:{token.name}"),
    )
    monkeypatch.setattr(
        module,
        "_validate_stack_identity",
        lambda bridge, gateway: identity_calls.append(
            f"stack:{bridge.name}:{gateway.name}"
        ),
    )
    monkeypatch.setattr(
        module.time,
        "sleep",
        lambda _seconds: startup_round.__setitem__("value", 1),
    )

    module._wait_for_existing_stack(bridge_token, gateway_token, timeout=1.0)

    assert identity_calls == [
        "supervisor:bridge.token",
        "stack:bridge.token:gateway.token",
    ]


def test_run_joins_authenticated_partial_start_instead_of_calling_it_stale(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    module = _load_launcher()
    _redirect_runtime(module, tmp_path)
    module.ISAAC_PYTHON = tmp_path / "isaac-python.bat"
    module.BRIDGE_SCRIPT = tmp_path / "bridge_server.py"
    module.ISAAC_PYTHON.write_text("@echo off\n", encoding="utf-8")
    module.BRIDGE_SCRIPT.write_text("# bridge\n", encoding="utf-8")
    bridge_token, gateway_token = module._ensure_tokens()
    waited: list[tuple[Path, Path]] = []

    monkeypatch.setattr(module, "_current_source_digest", lambda: f"sha256:{'a' * 64}")
    monkeypatch.setattr(
        module,
        "_listener_ready",
        lambda port, timeout=0.25: port in {module.CONTROL_PORT, module.BRIDGE_PORT},
    )
    monkeypatch.setattr(
        module,
        "_wait_for_existing_stack",
        lambda bridge, gateway: waited.append((bridge, gateway)),
    )
    monkeypatch.setattr(
        module,
        "_start_process",
        lambda *_args, **_kwargs: pytest.fail("must not start a duplicate process"),
    )

    assert module._run() == 0
    assert waited == [(bridge_token, gateway_token)]


def test_run_bind_race_loser_joins_authenticated_winner(
    tmp_path: Path, monkeypatch: pytest.MonkeyPatch
) -> None:
    module = _load_launcher()
    _redirect_runtime(module, tmp_path)
    module.ISAAC_PYTHON = tmp_path / "isaac-python.bat"
    module.BRIDGE_SCRIPT = tmp_path / "bridge_server.py"
    module.ISAAC_PYTHON.write_text("@echo off\n", encoding="utf-8")
    module.BRIDGE_SCRIPT.write_text("# bridge\n", encoding="utf-8")
    bridge_token, gateway_token = module._ensure_tokens()
    waited: list[tuple[Path, Path]] = []
    control_checks = {"value": 0}

    def listener_ready(port: int, timeout: float = 0.25) -> bool:
        del timeout
        if port == module.CONTROL_PORT:
            control_checks["value"] += 1
            return control_checks["value"] > 1
        return False

    class ImmediateThread:
        def __init__(self, *, target, name: str, daemon: bool) -> None:
            del name, daemon
            self._target = target

        def start(self) -> None:
            self._target()

    def lose_control_bind(**_kwargs) -> None:
        raise OSError("address already in use")

    monkeypatch.setattr(module, "_current_source_digest", lambda: f"sha256:{'a' * 64}")
    monkeypatch.setattr(module, "_listener_ready", listener_ready)
    monkeypatch.setattr(module, "_control_loop", lose_control_bind)
    monkeypatch.setattr(module.threading, "Thread", ImmediateThread)
    monkeypatch.setattr(module.signal, "signal", lambda *_args: None)
    monkeypatch.setattr(
        module,
        "_wait_for_existing_stack",
        lambda bridge, gateway: waited.append((bridge, gateway)),
    )
    monkeypatch.setattr(
        module,
        "_start_process",
        lambda *_args, **_kwargs: pytest.fail("must not start a duplicate process"),
    )

    assert module._run() == 0
    assert waited == [(bridge_token, gateway_token)]
