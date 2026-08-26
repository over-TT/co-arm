"""Security-sensitive configuration shared by gateway bearer-token clients."""

from __future__ import annotations

from urllib.parse import urlsplit


class GatewayConfigurationError(ValueError):
    """The configured gateway origin is not an allowed local HTTP endpoint."""


_ALLOWED_HOSTS = frozenset({"localhost", "127.0.0.1", "::1"})
_ALLOWED_BACKENDS = frozenset({"sim", "real"})


def validated_expected_backend(value: str | None) -> str:
    """Return the explicitly selected gateway kind or reject the configuration.

    A loopback port is not an identity boundary: either the simulator or the
    physical gateway can listen on any permitted local port. Requiring this
    independent selection lets the MCP client authenticate ``/healthz`` and
    refuse a valid credential connected to the wrong kind of arm.
    """

    if not isinstance(value, str):
        raise GatewayConfigurationError(
            "ARM_EXPECTED_BACKEND must be set explicitly to sim or real."
        )
    candidate = value.strip().lower()
    if candidate not in _ALLOWED_BACKENDS:
        raise GatewayConfigurationError(
            "ARM_EXPECTED_BACKEND must be set explicitly to sim or real."
        )
    return candidate


def validated_loopback_gateway_url(value: str) -> str:
    """Return a canonical local gateway origin or reject the configuration.

    Gateway bearer tokens are machine-local credentials.  Requiring one exact
    HTTP loopback origin, with no URL suffix or user information, prevents a
    typo or hostile environment variable from sending that token elsewhere.
    """

    if not isinstance(value, str):
        raise GatewayConfigurationError("The gateway URL must be a string.")
    candidate = value
    if not candidate:
        raise GatewayConfigurationError("The gateway URL is empty.")
    if any(
        character.isspace() or ord(character) < 32 or ord(character) == 127
        for character in candidate
    ):
        raise GatewayConfigurationError(
            "The gateway URL must not contain whitespace or control characters."
        )
    try:
        parsed = urlsplit(candidate)
        hostname = parsed.hostname
        port = parsed.port
        username = parsed.username
        password = parsed.password
    except ValueError as error:
        raise GatewayConfigurationError("The gateway URL is malformed.") from error

    if parsed.scheme != "http":
        raise GatewayConfigurationError("The gateway URL must use http://.")
    if not parsed.netloc or hostname is None:
        raise GatewayConfigurationError("The gateway URL must include a loopback host.")
    if username is not None or password is not None or "@" in parsed.netloc:
        raise GatewayConfigurationError("The gateway URL must not contain user information.")
    if hostname not in _ALLOWED_HOSTS:
        raise GatewayConfigurationError(
            "The gateway host must be exactly localhost, 127.0.0.1, or ::1."
        )
    if port is None or not 1 <= port <= 65_535:
        raise GatewayConfigurationError(
            "The gateway URL must include an explicit TCP port from 1 to 65535."
        )
    if parsed.path not in {"", "/"} or "?" in candidate or "#" in candidate:
        raise GatewayConfigurationError(
            "The gateway URL must not include a non-root path, query, or fragment."
        )

    canonical_host = f"[{hostname}]" if hostname == "::1" else hostname
    return f"http://{canonical_host}:{port}"
