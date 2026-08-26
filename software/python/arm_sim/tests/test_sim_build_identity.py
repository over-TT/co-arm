from __future__ import annotations

from pathlib import Path

from arm_sim.build_identity import simulator_source_digest, simulator_source_files


def _write(path: Path, text: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(text, encoding="utf-8")


def test_simulator_source_digest_covers_runtime_sources_and_authored_inputs(
    tmp_path: Path,
) -> None:
    root = tmp_path / "project"
    _write(root / "python" / "arm_sim" / "bridge" / "server.py", "BRIDGE = 1\n")
    _write(root / "python" / "arm_sim" / "isaac" / "scene.py", "SCENE = 1\n")
    _write(root / "python" / "arm_sim" / "assets" / "arm.urdf", "<robot/>\n")
    _write(root / "python" / "arm_sim" / "config" / "arm.json", '{"version":1}\n')
    _write(
        root / "python" / "arm_sim" / "config" / "camera_profile.json",
        '{"profileId":"imx708-wide"}\n',
    )
    _write(root / "python" / "arm_sim" / "generated" / "scene.usda", "generated-v1\n")
    _write(root / "python" / "arm_sim" / "tests" / "test_only.py", "TEST_ONLY = 1\n")
    _write(root / "python" / "robot_gateway" / "isaac_bridge.py", "ADAPTER = 1\n")
    _write(root / "python" / "robot_gateway" / "runtime.py", "RUNTIME = 1\n")
    _write(root / "python" / "robot_gateway" / "tests" / "test_only.py", "TEST_ONLY = 1\n")
    _write(root / "operations" / "scripts" / "run_arm_sim.py", "LAUNCHER = 1\n")
    _write(root / "operations" / "requirements-pi.txt", "fastapi==1\n")
    _write(root / "requirements-web.txt", "uvicorn==1\n")
    _write(root / "toolchain.lock.json", "{}\n")

    files = {
        path.relative_to(root).as_posix() for path in simulator_source_files(root)
    }
    assert "python/arm_sim/bridge/server.py" in files
    assert "python/arm_sim/isaac/scene.py" in files
    assert "python/arm_sim/assets/arm.urdf" in files
    assert "python/arm_sim/config/arm.json" in files
    assert "python/arm_sim/config/camera_profile.json" in files
    assert "python/robot_gateway/isaac_bridge.py" in files
    assert "python/robot_gateway/runtime.py" in files
    assert "operations/scripts/run_arm_sim.py" in files
    assert "operations/requirements-pi.txt" in files
    assert "python/arm_sim/generated/scene.usda" not in files
    assert "python/arm_sim/tests/test_only.py" not in files
    assert "python/robot_gateway/tests/test_only.py" not in files
    assert "requirements-web.txt" not in files
    assert "toolchain.lock.json" not in files

    original = simulator_source_digest(root)
    assert original.startswith("sha256:") and len(original) == 71

    _write(root / "python" / "arm_sim" / "generated" / "scene.usda", "generated-v2\n")
    _write(root / "python" / "arm_sim" / "tests" / "test_only.py", "TEST_ONLY = 2\n")
    assert simulator_source_digest(root) == original

    _write(root / "python" / "robot_gateway" / "runtime.py", "RUNTIME = 2\n")
    assert simulator_source_digest(root) != original


def test_simulator_source_digest_is_path_sensitive_and_deterministic(
    tmp_path: Path,
) -> None:
    root = tmp_path / "project"
    _write(root / "python" / "arm_sim" / "bridge" / "one.py", "same\n")
    _write(root / "python" / "robot_gateway" / "isaac_bridge.py", "adapter\n")
    _write(root / "python" / "robot_gateway" / "runtime.py", "runtime\n")
    _write(root / "operations" / "scripts" / "run_arm_sim.py", "launcher\n")
    _write(root / "operations" / "requirements-pi.txt", "pi\n")
    _write(root / "requirements-web.txt", "web\n")
    _write(root / "toolchain.lock.json", "{}\n")

    first = simulator_source_digest(root)
    second = simulator_source_digest(root)
    assert first == second

    (root / "python" / "arm_sim" / "bridge" / "one.py").rename(
        root / "python" / "arm_sim" / "bridge" / "two.py"
    )
    assert simulator_source_digest(root) != first
