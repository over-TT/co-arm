"""Persistent, authenticated loopback server hosted by the Isaac process."""

from __future__ import annotations

import hashlib
import hmac
import json
import logging
import secrets
import socket
import socketserver
from threading import RLock
from typing import Callable, Mapping

from .engine import BridgeEngine
from .protocol import (
    MAX_REQUEST_BYTES,
    PROTOCOL_NAME,
    ProtocolViolation,
    backend_identity,
    validate_request,
    validate_result,
)


LOOPBACK_HOST = "127.0.0.1"
DEFAULT_PORT = 8790
DEFAULT_READ_TIMEOUT_SECONDS = 5.0
_MAX_VALIDATION_REASON_CHARS = 160


_LOGGER = logging.getLogger(__name__)


class _ResultValidationFailure(Exception):
    def __init__(
        self, *, protocol: str, command: str, violation: ProtocolViolation
    ) -> None:
        reason = " ".join(str(violation).split()) or "protocol violation"
        self.protocol = protocol
        self.command = command
        self.reason = reason[:_MAX_VALIDATION_REASON_CHARS]
        super().__init__(self.reason)


def _valid_token(token: object) -> bool:
    return (
        isinstance(token, str)
        and 32 <= len(token) <= 256
        and all(character.isprintable() and not character.isspace() for character in token)
    )


class LoopbackBridgeServer(socketserver.TCPServer):
    """Single-engine JSON-lines server.

    ``TCPServer`` is deliberately single-threaded.  Isaac/PhysX operations stay
    serialized on the same thread that calls ``serve_forever``; an engine can
    step the simulation until arrival before returning from a command.
    """

    allow_reuse_address = True

    def __init__(
        self,
        *,
        engine: BridgeEngine,
        token: str,
        host: str = LOOPBACK_HOST,
        port: int = DEFAULT_PORT,
        request_timeout_seconds: float = DEFAULT_READ_TIMEOUT_SECONDS,
        backend_instance_id: str | None = None,
        service_action: Callable[[], None] | None = None,
    ) -> None:
        if host != LOOPBACK_HOST:
            raise ValueError("Arm simulator bridge may bind only to 127.0.0.1")
        if isinstance(port, bool) or not isinstance(port, int) or not 0 <= port <= 65535:
            raise ValueError("port must be between 0 and 65535")
        if not _valid_token(token):
            raise ValueError("token must contain 32 to 256 printable non-space characters")
        if (
            isinstance(request_timeout_seconds, bool)
            or not isinstance(request_timeout_seconds, (int, float))
            or not 0.05 <= float(request_timeout_seconds) <= 60.0
        ):
            raise ValueError("request timeout must be between 0.05 and 60 seconds")
        if not isinstance(engine, BridgeEngine):
            raise TypeError("engine does not implement the bridge contract")
        if service_action is not None and not callable(service_action):
            raise TypeError("service_action must be callable")

        self.engine = engine
        self._expected_token_digest = hashlib.sha256(token.encode("utf-8")).digest()
        self.request_timeout_seconds = float(request_timeout_seconds)
        self.backend_instance_id = backend_instance_id or (
            f"sim_{secrets.token_urlsafe(18)}"
        )
        self.backend = backend_identity(self.backend_instance_id)
        self._engine_lock = RLock()
        self._service_action = service_action
        super().__init__((host, port), _BridgeRequestHandler, bind_and_activate=True)

    @property
    def address(self) -> tuple[str, int]:
        host, port = self.server_address
        return str(host), int(port)

    def token_matches(self, candidate: object) -> bool:
        if not isinstance(candidate, str):
            return False
        candidate_digest = hashlib.sha256(candidate.encode("utf-8")).digest()
        return hmac.compare_digest(candidate_digest, self._expected_token_digest)

    def service_actions(self) -> None:
        """Run an optional same-thread action between serialized requests."""

        if self._service_action is not None:
            self._service_action()

    def dispatch(self, command: str, params: Mapping[str, object]) -> dict[str, object]:
        with self._engine_lock:
            if command == "health":
                result = self.engine.health()
            elif command == "reset":
                seed = params.get("seed")
                result = self.engine.reset(
                    seed=seed if isinstance(seed, int) and not isinstance(seed, bool) else None
                )
            elif command == "get_state":
                result = self.engine.get_state()
            elif command == "set_joint_targets":
                targets = params["targets"]
                duration_ms = params["durationMs"]
                if not isinstance(targets, Mapping) or not isinstance(duration_ms, int):
                    raise ProtocolViolation("validated command parameters changed")
                result = self.engine.set_joint_targets(
                    targets={key: float(value) for key, value in targets.items()},
                    duration_ms=duration_ms,
                )
            elif command == "capture":
                profile = params["profile"]
                if not isinstance(profile, str):
                    raise ProtocolViolation("validated capture profile changed")
                result = self.engine.capture(profile=profile)
            else:
                raise ProtocolViolation("unsupported command")
        try:
            return validate_result(command, result)
        except ProtocolViolation as violation:
            raise _ResultValidationFailure(
                protocol=PROTOCOL_NAME,
                command=command,
                violation=violation,
            ) from None


class _BridgeRequestHandler(socketserver.StreamRequestHandler):
    server: LoopbackBridgeServer

    def handle(self) -> None:
        self.connection.settimeout(self.server.request_timeout_seconds)
        request_id = "invalid_request"
        try:
            raw = self.rfile.readline(MAX_REQUEST_BYTES + 1)
            if not raw or len(raw) > MAX_REQUEST_BYTES or not raw.endswith(b"\n"):
                self._error(request_id, "INVALID_REQUEST", "Invalid bridge request.")
                return
            try:
                decoded = json.loads(raw.decode("utf-8"))
            except (UnicodeDecodeError, json.JSONDecodeError):
                self._error(request_id, "INVALID_REQUEST", "Invalid bridge request.")
                return
            if isinstance(decoded, dict):
                candidate_id = decoded.get("requestId")
                if isinstance(candidate_id, str) and 8 <= len(candidate_id) <= 128:
                    request_id = candidate_id
            candidate_token = decoded.get("token") if isinstance(decoded, dict) else None
            response_protocol = PROTOCOL_NAME
            if not self.server.token_matches(candidate_token):
                self._error(
                    request_id,
                    "UNAUTHORIZED",
                    "Unauthorized.",
                    protocol=response_protocol,
                )
                return
            try:
                request = validate_request(decoded)
            except ProtocolViolation:
                self._error(
                    request_id,
                    "INVALID_REQUEST",
                    "Invalid bridge request.",
                    protocol=response_protocol,
                )
                return
            request_id = request.request_id
            try:
                result = self.server.dispatch(request.command, request.params)
            except _ResultValidationFailure as failure:
                _LOGGER.error(
                    "Simulator result validation failed protocol=%s command=%s reason=%s",
                    failure.protocol,
                    failure.command,
                    failure.reason,
                )
                self._error(
                    request_id,
                    "ENGINE_ERROR",
                    "Simulator returned an invalid response.",
                    protocol=response_protocol,
                )
                return
            except ProtocolViolation:
                self._error(
                    request_id,
                    "ENGINE_ERROR",
                    "Simulator returned an invalid response.",
                    protocol=response_protocol,
                )
                return
            except Exception:
                # Keep the IPC response sanitized while retaining a local
                # traceback for simulator-process diagnosis.
                _LOGGER.exception("Simulator bridge engine command failed")
                self._error(
                    request_id,
                    "ENGINE_ERROR",
                    "Simulator command failed.",
                    protocol=response_protocol,
                )
                return
            self._write(
                {
                    "protocol": response_protocol,
                    "requestId": request_id,
                    "ok": True,
                    "backend": self.server.backend,
                    "result": result,
                }
            )
        except (OSError, socket.timeout):
            # The peer is gone or never completed a bounded request.  No retry
            # or simulator mutation is attempted beneath an ambiguous caller.
            return

    def _error(
        self,
        request_id: str,
        code: str,
        message: str,
        *,
        protocol: str = PROTOCOL_NAME,
    ) -> None:
        self._write(
            {
                "protocol": protocol,
                "requestId": request_id,
                "ok": False,
                "backend": self.server.backend,
                "error": {"code": code, "message": message},
            }
        )

    def _write(self, response: Mapping[str, object]) -> None:
        try:
            encoded = json.dumps(
                response,
                sort_keys=True,
                separators=(",", ":"),
                allow_nan=False,
            ).encode("utf-8") + b"\n"
            self.wfile.write(encoded)
            self.wfile.flush()
        except (OSError, TypeError, ValueError):
            return


__all__ = [
    "DEFAULT_PORT",
    "LOOPBACK_HOST",
    "LoopbackBridgeServer",
]
