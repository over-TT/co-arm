from __future__ import annotations

import os
import subprocess
import sys
from pathlib import Path

import httpx
import pytest

from arm_mcp.config import (
    GatewayConfigurationError,
    validated_expected_backend,
    validated_loopback_gateway_url,
)
from arm_mcp.server import ArmToolError, Gateway


@pytest.mark.parametrize(
    ("raw", "canonical"),
    [
        ("http://localhost:8787", "http://localhost:8787"),
        ("HTTP://LOCALHOST:8787/", "http://localhost:8787"),
        ("http://127.0.0.1:8788/", "http://127.0.0.1:8788"),
        ("http://[::1]:8789", "http://[::1]:8789"),
    ],
)
def test_validated_loopback_gateway_url_canonicalizes_allowed_origins(
    raw: str,
    canonical: str,
) -> None:
    assert validated_loopback_gateway_url(raw) == canonical


@pytest.mark.parametrize(
    "raw",
    [
        "https://127.0.0.1:8787",
        "http://127.0.0.1",
        "http://127.0.0.1:0",
        "http://127.0.0.1:65536",
        "http://127.0.0.2:8787",
        "http://127.1:8787",
        "http://2130706433:8787",
        "http://localhost.example:8787",
        "http://[::2]:8787",
        "http://user@localhost:8787",
        "http://localhost:8787/api",
        "http://localhost:8787?",
        "http://localhost:8787?target=remote",
        "http://localhost:8787#",
        "http://localhost:8787#remote",
        "http://localhost:8787\n",
        "localhost:8787",
        "",
    ],
)
def test_validated_loopback_gateway_url_rejects_unsafe_values(raw: str) -> None:
    with pytest.raises(GatewayConfigurationError):
        validated_loopback_gateway_url(raw)


@pytest.mark.parametrize(
    ("raw", "expected"),
    [("sim", "sim"), (" real ", "real"), ("SIM", "sim")],
)
def test_validated_expected_backend_accepts_only_explicit_backend_kind(
    raw: str,
    expected: str,
) -> None:
    assert validated_expected_backend(raw) == expected


@pytest.mark.parametrize("raw", [None, "", "physical", "simulation", "sim/real"])
def test_validated_expected_backend_rejects_missing_or_implicit_values(
    raw: object,
) -> None:
    with pytest.raises(GatewayConfigurationError, match="explicitly to sim or real"):
        validated_expected_backend(raw)  # type: ignore[arg-type]


def test_mcp_requires_expected_backend_before_loading_token(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    token_reads = 0

    def unexpected_token_read() -> str:
        nonlocal token_reads
        token_reads += 1
        raise AssertionError("token lookup must not run")

    monkeypatch.setenv("ARM_GATEWAY_URL", "http://127.0.0.1:8787")
    monkeypatch.delenv("ARM_EXPECTED_BACKEND", raising=False)
    monkeypatch.setattr("arm_mcp.server._load_token", unexpected_token_read)

    with pytest.raises(ArmToolError, match="ARM_EXPECTED_BACKEND") as caught:
        Gateway().client()
    assert caught.value.code == "configuration_error"
    assert token_reads == 0


def test_mcp_validates_gateway_before_loading_token(monkeypatch: pytest.MonkeyPatch) -> None:
    token_reads = 0

    def unexpected_token_read() -> str:
        nonlocal token_reads
        token_reads += 1
        raise AssertionError("token lookup must not run")

    monkeypatch.setenv("ARM_GATEWAY_URL", "http://example.com:8787")
    monkeypatch.setattr("arm_mcp.server._load_token", unexpected_token_read)

    with pytest.raises(ArmToolError, match="host must be exactly"):
        Gateway().client()
    assert token_reads == 0


def test_mcp_connection_error_does_not_assume_backend(
    monkeypatch: pytest.MonkeyPatch,
) -> None:
    gateway = Gateway()

    def fail(*args, **kwargs):  # noqa: ANN002, ANN003, ANN202
        raise httpx.ConnectError("refused")

    monkeypatch.setattr(gateway, "_send_once", fail)
    with pytest.raises(ArmToolError) as caught:
        gateway.request("GET", "/healthz")

    message = str(caught.value)
    assert "local loopback port" in message
    assert "SSH" not in message
    assert "Pi" not in message
    assert "tunnel" not in message


def test_arm_status_rejects_remote_url_before_token_lookup(tmp_path: Path) -> None:
    python_root = Path(__file__).resolve().parents[2]
    script = python_root.parent / "operations" / "scripts" / "arm_status.py"
    missing_token = tmp_path / "does-not-exist.token"
    environment = os.environ.copy()
    environment["PYTHONPATH"] = os.pathsep.join(
        item
        for item in (str(python_root), environment.get("PYTHONPATH", ""))
        if item
    )

    result = subprocess.run(
        [
            sys.executable,
            str(script),
            "--gateway-url",
            "http://example.com:8787",
            "--token-file",
            str(missing_token),
        ],
        cwd=tmp_path,
        env=environment,
        capture_output=True,
        text=True,
        timeout=10,
        check=False,
    )

    assert result.returncode != 0
    assert "host must be exactly" in result.stderr
    assert "No gateway token" not in result.stderr
