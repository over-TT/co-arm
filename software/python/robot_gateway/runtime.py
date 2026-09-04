"""Secure, loopback-only runtime for the Raspberry Pi robot gateway.

This module deliberately exposes only a health check and the bounded robot API.
It does not provide shell execution or general filesystem access.
"""

from __future__ import annotations

import argparse
from contextlib import asynccontextmanager
import hashlib
import hmac
import os
from pathlib import Path
import secrets
import stat
from typing import AsyncIterator, Awaitable, Callable, Sequence

from fastapi import Depends, FastAPI, HTTPException, Request, Response, status
from fastapi.security import HTTPAuthorizationCredentials, HTTPBearer
import uvicorn

from robot_gateway.arm_controller import ArmController, UnavailableArmController
from robot_gateway.camera_api import CameraEvidenceService, CameraProvider, create_camera_router
from robot_gateway.camera_profiles import CAMERA_PROFILE_CHOICES, MODULE3_WIDE_PROFILE_ID
from robot_gateway.physical_arm_api import (
    PhysicalArmProfileStore,
    PhysicalProfileStoreError,
    create_physical_arm_router,
)
from robot_gateway.pi_camera import PiCameraProvider
from robot_gateway.serial_arm_controller import SerialArmController
from robot_gateway.simple_arm_api import ArmService, JointStore, create_simple_arm_router


DEFAULT_HOST = "127.0.0.1"
DEFAULT_PORT = 8787
ALLOWED_BIND_HOSTS = frozenset({"127.0.0.1", "localhost", "::1"})
MIN_TOKEN_LENGTH = 32
MAX_TOKEN_LENGTH = 256
_TOKEN_FILE_MODE = stat.S_IRUSR | stat.S_IWUSR
GATEWAY_INSTANCE_HEADER = "X-Robot-Gateway-Instance"


class GatewayConfigurationError(ValueError):
    """A safe-to-display gateway configuration error."""


def _token_is_valid(token: str) -> bool:
    return (
        MIN_TOKEN_LENGTH <= len(token) <= MAX_TOKEN_LENGTH
        and all(character.isprintable() and not character.isspace() for character in token)
    )


def load_token(token_file: str | os.PathLike[str]) -> str:
    """Read and validate a bearer token without leaking file contents in errors."""

    try:
        token = Path(token_file).read_text(encoding="utf-8").strip()
    except (OSError, UnicodeError):
        raise GatewayConfigurationError("Unable to read the token file.") from None

    if not _token_is_valid(token):
        raise GatewayConfigurationError(
            "The token file must contain 32 to 256 printable, non-whitespace characters."
        )
    return token


def generate_token_file(output: str | os.PathLike[str]) -> Path:
    """Atomically create a new owner-only token file and never overwrite a path."""

    output_path = Path(output)
    token = secrets.token_hex(32)  # 32 random bytes = 256 bits of entropy.
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL
    if hasattr(os, "O_CLOEXEC"):
        flags |= os.O_CLOEXEC

    try:
        descriptor = os.open(output_path, flags, _TOKEN_FILE_MODE)
    except FileExistsError:
        raise GatewayConfigurationError("Refusing to overwrite an existing token file.") from None
    except OSError:
        raise GatewayConfigurationError("Unable to create the token file.") from None

    try:
        if os.name == "posix" and hasattr(os, "fchmod"):
            os.fchmod(descriptor, _TOKEN_FILE_MODE)
        with os.fdopen(descriptor, "w", encoding="utf-8", newline="\n") as handle:
            descriptor = -1
            handle.write(token)
            handle.write("\n")
            handle.flush()
            os.fsync(handle.fileno())
    except OSError:
        if descriptor >= 0:
            os.close(descriptor)
        try:
            output_path.unlink(missing_ok=True)
        except OSError:
            pass
        raise GatewayConfigurationError("Unable to write the token file securely.") from None

    return output_path


def validate_bind_host(host: str) -> str:
    """Accept only literal loopback bind targets."""

    if host not in ALLOWED_BIND_HOSTS:
        raise GatewayConfigurationError(
            "The gateway may bind only to 127.0.0.1, localhost, or ::1. "
            "Use an SSH tunnel for remote access."
        )
    return host


def _bearer_dependency(expected_token: str):
    bearer = HTTPBearer(auto_error=False)
    expected_digest = hashlib.sha256(expected_token.encode("utf-8")).digest()

    async def require_bearer_token(
        credentials: HTTPAuthorizationCredentials | None = Depends(bearer),
    ) -> None:
        # Hash first so compare_digest always receives equal-length byte strings.
        candidate = credentials.credentials if credentials is not None else ""
        candidate_digest = hashlib.sha256(candidate.encode("utf-8")).digest()
        token_matches = hmac.compare_digest(candidate_digest, expected_digest)
        bearer_scheme = credentials is not None and credentials.scheme.casefold() == "bearer"
        if not (bearer_scheme and token_matches):
            raise HTTPException(
                status_code=status.HTTP_401_UNAUTHORIZED,
                detail="Unauthorized",
                headers={"WWW-Authenticate": "Bearer"},
            )

    return require_bearer_token


def create_app(
    *,
    token_file: str | os.PathLike[str],
    camera_provider: CameraProvider | None = None,
    camera_service: CameraEvidenceService | None = None,
    arm_controller: ArmController | None = None,
    arm_state_dir: str | os.PathLike[str] | None = None,
) -> FastAPI:
    """Create the gateway app using an explicitly supplied token file."""

    expected_token = load_token(token_file)
    gateway_instance_id = f"gateway:{secrets.token_hex(16)}"
    configured_controller = arm_controller or UnavailableArmController(configured=False)
    physical_profile_store = PhysicalArmProfileStore(arm_state_dir)
    require_bearer_token = _bearer_dependency(expected_token)
    if camera_provider is not None and camera_service is not None:
        raise GatewayConfigurationError(
            "Supply either a camera provider or a camera service, not both."
        )
    simple_arm_service = ArmService(
        configured_controller,
        JointStore(arm_state_dir),
    )
    configured_camera = camera_service
    if camera_provider is not None:
        camera_simulated = getattr(camera_provider, "simulated", False)
        camera_source = getattr(camera_provider, "source", "picamera2")
        camera_identity_confidence = getattr(
            camera_provider, "identity_confidence", "configured_candidate"
        )
        try:
            configured_camera = CameraEvidenceService(
                camera_provider,
                status_callback=simple_arm_service.evidence_status,
                simulated=camera_simulated,
                source=camera_source,
                identity_confidence=camera_identity_confidence,
            )
        except Exception:
            simple_arm_service.close()
            raise
    @asynccontextmanager
    async def lifespan(_: FastAPI) -> AsyncIterator[None]:
        try:
            configured_controller.start()
        except Exception:
            # A missing or faulted physical controller must not hide camera and
            # gateway diagnostics. Its routes remain present and fail-closed.
            pass
        if configured_camera is not None:
            configured_camera.start()
        try:
            yield
        finally:
            simple_arm_service.close()
            if configured_camera is not None:
                configured_camera.close()
            configured_controller.close()

    app = FastAPI(
        title="Robot Gateway",
        docs_url=None,
        redoc_url=None,
        openapi_url=None,
        lifespan=lifespan,
    )

    @app.middleware("http")
    async def attach_gateway_instance(
        request: Request, call_next: Callable[[Request], Awaitable[Response]]
    ) -> Response:
        # Process provenance stays stable when the controller reconnects. A new
        # gateway process receives a new value, invalidating reviewed plans.
        response = await call_next(request)
        response.headers[GATEWAY_INSTANCE_HEADER] = gateway_instance_id
        return response

    @app.get("/healthz", include_in_schema=False)
    async def healthz() -> dict[str, object]:
        return {
            "status": "ok",
            "backendId": "real",
            "backendInstanceId": gateway_instance_id,
            "simulated": False,
            "version": "robot-gateway-v1",
        }

    app.include_router(
        create_physical_arm_router(
            configured_controller,
            profile_store=physical_profile_store,
            operation_lock=simple_arm_service.operation_lock,
            admission_lock=simple_arm_service.operation_admission_lock,
            mutation_context=simple_arm_service.commissioning_mutation,
            clear_mutation_context=simple_arm_service.commissioning_clear_mutation,
            stop_callback=simple_arm_service.stop_for_commissioning,
            reset_callback=simple_arm_service.clear_stop_for_commissioning,
        ),
        dependencies=[Depends(require_bearer_token)],
    )
    app.include_router(
        create_simple_arm_router(
            configured_controller,
            arm_service=simple_arm_service,
            camera_service=configured_camera,
        ),
        dependencies=[Depends(require_bearer_token)],
    )
    if configured_camera is not None:
        app.include_router(
            create_camera_router(configured_camera),
            dependencies=[Depends(require_bearer_token)],
        )
    return app


# A descriptive alias for callers that prefer to name the deployment boundary.
create_gateway_app = create_app


def _port_number(value: str) -> int:
    try:
        port = int(value)
    except ValueError:
        raise argparse.ArgumentTypeError("port must be an integer") from None
    if not 1 <= port <= 65535:
        raise argparse.ArgumentTypeError("port must be between 1 and 65535")
    return port


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m robot_gateway",
        description="Run the loopback-only Raspberry Pi robot gateway.",
    )
    subcommands = parser.add_subparsers(dest="command", required=True)

    generate = subcommands.add_parser(
        "generate-token",
        help="create a new bearer-token file without overwriting",
    )
    generate.add_argument("--output", required=True, type=Path)

    serve = subcommands.add_parser("serve", help="serve the robot gateway")
    serve.add_argument("--token-file", required=True, type=Path)
    serve.add_argument("--host", default=DEFAULT_HOST)
    serve.add_argument("--port", default=DEFAULT_PORT, type=_port_number)
    serve.add_argument(
        "--camera",
        choices=("none", "picamera2"),
        default="none",
        help="enable an explicitly selected read-only camera provider",
    )
    serve.add_argument(
        "--camera-profile",
        choices=CAMERA_PROFILE_CHOICES,
        default=MODULE3_WIDE_PROFILE_ID,
        help=(
            "select the connected camera profile; module3-wide is the deployed "
            "default, ov5647 is legacy compatibility, and auto uses driver identity"
        ),
    )
    serve.add_argument(
        "--arm-controller-port",
        help="enable the bounded A1 controller protocol on this serial device",
    )
    serve.add_argument(
        "--arm-state-dir",
        type=Path,
        help="private directory for the atomic physical calibration profile",
    )
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    parser = build_parser()
    arguments = parser.parse_args(argv)

    try:
        if arguments.command == "generate-token":
            generate_token_file(arguments.output)
            print("Token file created.")
            return 0

        if arguments.command == "serve":
            host = validate_bind_host(arguments.host)
            camera_provider = None
            if arguments.camera == "picamera2":
                # Preserve the zero-argument provider construction path for
                # deployments and test doubles that rely on the provider's
                # explicit Module 3 Wide default. Non-default selections are
                # always forwarded verbatim.
                camera_provider = (
                    PiCameraProvider()
                    if arguments.camera_profile == MODULE3_WIDE_PROFILE_ID
                    else PiCameraProvider(camera_profile=arguments.camera_profile)
                )
            app = create_app(
                token_file=arguments.token_file,
                camera_provider=camera_provider,
                arm_controller=(
                    SerialArmController(arguments.arm_controller_port)
                    if arguments.arm_controller_port
                    else UnavailableArmController(configured=False)
                ),
                arm_state_dir=arguments.arm_state_dir,
            )
            uvicorn.run(app, host=host, port=arguments.port)
            return 0
    except (
        GatewayConfigurationError,
        PhysicalProfileStoreError,
    ) as exc:
        parser.error(str(exc))

    parser.error("unknown command")
    return 2  # pragma: no cover - argparse.error always raises SystemExit.


__all__ = [
    "ALLOWED_BIND_HOSTS",
    "DEFAULT_HOST",
    "DEFAULT_PORT",
    "GatewayConfigurationError",
    "GATEWAY_INSTANCE_HEADER",
    "build_parser",
    "create_app",
    "create_gateway_app",
    "generate_token_file",
    "load_token",
    "main",
    "validate_bind_host",
]
