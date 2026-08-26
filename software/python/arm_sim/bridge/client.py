"""Bounded Python 3.11 client for the Isaac-hosted bridge process."""

from __future__ import annotations

from dataclasses import dataclass
import json
import secrets
import socket
from threading import RLock
from typing import Mapping

from .protocol import (
    MAX_RESPONSE_BYTES,
    PROTOCOL_NAME,
    ProtocolViolation,
    validate_request,
    validate_response,
)
from .server import LOOPBACK_HOST


class BridgeClientError(RuntimeError):
    """Base class for sanitized bridge client failures."""


class BridgeUnavailableError(BridgeClientError):
    """No trusted simulator response was available."""


class BridgeTimeoutError(BridgeUnavailableError):
    """The simulator did not complete within the configured timeout."""


class BridgeProtocolError(BridgeClientError):
    """The server response did not satisfy the strict protocol."""


class BridgeRemoteError(BridgeClientError):
    """The simulator rejected or failed a well-formed request."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(f"{code}: {message}")
        self.code = code
        self.message = message


@dataclass(frozen=True, slots=True)
class BridgeReply:
    backend_id: str
    backend_instance_id: str
    simulated: bool
    result: dict[str, object]

    @property
    def backend(self) -> dict[str, object]:
        return {
            "backendId": self.backend_id,
            "backendInstanceId": self.backend_instance_id,
            "simulated": self.simulated,
        }



class BridgeClient:
    """One-request-per-connection client with process-identity pinning."""

    def __init__(
        self,
        *,
        token: str,
        host: str = LOOPBACK_HOST,
        port: int,
        connect_timeout_seconds: float = 1.0,
        request_timeout_seconds: float = 15.0,
    ) -> None:
        if host != LOOPBACK_HOST:
            raise ValueError("Arm simulator bridge client may connect only to 127.0.0.1")
        if isinstance(port, bool) or not isinstance(port, int) or not 1 <= port <= 65535:
            raise ValueError("port must be between 1 and 65535")
        if not isinstance(token, str):
            raise ValueError("token must be a string")
        # Reuse the request validator for the exact token policy.
        validate_request(
            {
                "protocol": PROTOCOL_NAME,
                "requestId": "validate_token",
                "token": token,
                "command": "health",
                "params": {},
            }
        )
        for value, label, maximum in (
            (connect_timeout_seconds, "connect timeout", 30.0),
            (request_timeout_seconds, "request timeout", 300.0),
        ):
            if (
                isinstance(value, bool)
                or not isinstance(value, (int, float))
                or not 0.01 <= float(value) <= maximum
            ):
                raise ValueError(f"{label} is outside its allowed range")
        self._token = token
        self._host = host
        self._port = port
        self._connect_timeout = float(connect_timeout_seconds)
        self._request_timeout = float(request_timeout_seconds)
        self._lock = RLock()
        self._backend_instance_id: str | None = None

    @property
    def backend_instance_id(self) -> str | None:
        with self._lock:
            return self._backend_instance_id

    def reconnect(self) -> None:
        """Explicitly allow a new simulator-process identity on the next call."""

        with self._lock:
            self._backend_instance_id = None

    def call(
        self, command: str, params: Mapping[str, object] | None = None
    ) -> BridgeReply:
        request_id = f"req_{secrets.token_urlsafe(14)}"
        request = {
            "protocol": PROTOCOL_NAME,
            "requestId": request_id,
            "token": self._token,
            "command": command,
            "params": dict(params or {}),
        }
        validated = validate_request(request)
        encoded = json.dumps(
            request,
            sort_keys=True,
            separators=(",", ":"),
            allow_nan=False,
        ).encode("utf-8") + b"\n"

        with self._lock:
            try:
                with socket.create_connection(
                    (self._host, self._port), timeout=self._connect_timeout
                ) as connection:
                    connection.settimeout(self._request_timeout)
                    connection.sendall(encoded)
                    with connection.makefile("rb") as reader:
                        raw = reader.readline(MAX_RESPONSE_BYTES + 1)
            except socket.timeout:
                raise BridgeTimeoutError("Simulator bridge timed out.") from None
            except OSError:
                raise BridgeUnavailableError("Simulator bridge is unavailable.") from None
            if not raw or len(raw) > MAX_RESPONSE_BYTES or not raw.endswith(b"\n"):
                raise BridgeProtocolError("Simulator bridge response is incomplete.")
            try:
                decoded = json.loads(raw.decode("utf-8"))
                backend, result, error = validate_response(
                    decoded,
                    expected_request_id=request_id,
                    command=validated.command,
                )
            except (UnicodeDecodeError, json.JSONDecodeError, ProtocolViolation):
                raise BridgeProtocolError("Simulator bridge response is invalid.") from None

            instance_id = backend["backendInstanceId"]
            assert isinstance(instance_id, str)
            if self._backend_instance_id is None:
                self._backend_instance_id = instance_id
            elif self._backend_instance_id != instance_id:
                raise BridgeProtocolError(
                    "Simulator process identity changed; reconnect explicitly."
                )
            if error is not None:
                raise BridgeRemoteError(error["code"], error["message"])
            assert result is not None
            return BridgeReply(
                backend_id="sim",
                backend_instance_id=instance_id,
                simulated=True,
                result=result,
            )

    def health(self) -> BridgeReply:
        return self.call("health")

    def reset(self, *, seed: int | None = None) -> BridgeReply:
        return self.call("reset", {} if seed is None else {"seed": seed})

    def get_state(self) -> BridgeReply:
        return self.call("get_state")

    def set_joint_targets(
        self, *, targets: Mapping[str, float], duration_ms: int = 0
    ) -> BridgeReply:
        return self.call(
            "set_joint_targets",
            {"targets": dict(targets), "durationMs": duration_ms},
        )

    def capture(self, *, profile: str = "detail") -> BridgeReply:
        return self.call("capture", {"profile": profile})


__all__ = [
    "BridgeClient",
    "BridgeClientError",
    "BridgeProtocolError",
    "BridgeRemoteError",
    "BridgeReply",
    "BridgeTimeoutError",
    "BridgeUnavailableError",
]
