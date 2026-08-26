"""Command-line runtime for the standalone co-arm dashboard."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys
from typing import Sequence

from fastapi import FastAPI
import uvicorn

from .app import create_app
from .arm_backends import ArmBackendDescriptor, ArmBackendRegistry
from .arm_control_center import ArmControlCenterService
from .robot_gateway_client import RobotGatewayClient, RobotGatewayConfigurationError


LOCAL_HOST = "127.0.0.1"
LOCAL_PORT = 8765
SOFTWARE_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_STATIC_DIRECTORY = SOFTWARE_ROOT / "dashboard" / "dist"


class RuntimeConfigurationError(RuntimeError):
    """Raised when an explicit standalone runtime pair is incomplete."""


def _validate_pair(url: str | None, token_file: Path | None, label: str) -> None:
    if (url is None) != (token_file is None):
        raise RuntimeConfigurationError(
            f"{label} URL and token file must be supplied together."
        )


def _validated_dashboard_directory(path: Path) -> Path:
    """Require an explicitly built dashboard instead of serving a false shell.

    The Python wheel contains the backend and simulator resources, but the Vite
    build is a separate artifact. A source checkout gets the conventional
    ``software/dashboard/dist`` default; an installed wheel must be pointed at
    a real build with ``--static-directory``.
    """

    resolved = path.expanduser().resolve()
    if not (resolved / "index.html").is_file():
        raise RuntimeConfigurationError(
            "Standalone dashboard assets are not bundled in the Python wheel. "
            "Build software/dashboard and pass its dist directory with "
            "--static-directory."
        )
    return resolved


def create_runtime_app(
    *,
    software_root: Path = SOFTWARE_ROOT,
    static_directory: Path | None = DEFAULT_STATIC_DIRECTORY,
    real_url: str | None = None,
    real_token_file: Path | None = None,
    sim_url: str | None = None,
    sim_token_file: Path | None = None,
) -> FastAPI:
    _validate_pair(real_url, real_token_file, "REAL gateway")
    _validate_pair(sim_url, sim_token_file, "SIM gateway")

    root = software_root.resolve()
    descriptors: list[ArmBackendDescriptor] = []
    if real_url is not None and real_token_file is not None:
        descriptors.append(
            ArmBackendDescriptor.create(
                "real",
                "Real arm",
                RobotGatewayClient(real_url, real_token_file),
            )
        )
    if sim_url is not None and sim_token_file is not None:
        descriptors.append(
            ArmBackendDescriptor.create(
                "sim",
                "Isaac simulator",
                RobotGatewayClient(
                    sim_url,
                    sim_token_file,
                    expected_simulated=True,
                    camera_source="isaac_rgb",
                    camera_identity_confidence="simulated_model",
                ),
                require_upstream_identity=True,
            )
        )

    registry = (
        ArmBackendRegistry(
            descriptors,
            default_backend_id="sim" if sim_url is not None else "real",
        )
        if descriptors
        else None
    )
    return create_app(
        project_root=root,
        static_directory=static_directory.resolve() if static_directory else None,
        arm_backend_registry=registry,
        arm_control_center=ArmControlCenterService(project_root=root),
    )


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        prog="python -m web_backend",
        description="Run the ARM-only dashboard on local loopback.",
    )
    parser.add_argument("--real-url", help="Loopback origin of the Raspberry Pi gateway.")
    parser.add_argument(
        "--real-token-file",
        type=Path,
        help="Local bearer-token file for the Raspberry Pi gateway.",
    )
    parser.add_argument("--sim-url", help="Loopback origin of the simulator gateway.")
    parser.add_argument(
        "--sim-token-file",
        type=Path,
        help="Local bearer-token file for the simulator gateway.",
    )
    parser.add_argument(
        "--static-directory",
        type=Path,
        default=DEFAULT_STATIC_DIRECTORY,
        help="Built standalone dashboard directory.",
    )
    parser.add_argument("--port", type=int, default=LOCAL_PORT)
    parser.add_argument("--no-access-log", action="store_true")
    return parser


def main(argv: Sequence[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    if not 1024 <= arguments.port <= 65_535:
        print("Dashboard port must be between 1024 and 65535.", file=sys.stderr)
        return 2
    try:
        static_directory = _validated_dashboard_directory(arguments.static_directory)
        app = create_runtime_app(
            static_directory=static_directory,
            real_url=arguments.real_url,
            real_token_file=arguments.real_token_file,
            sim_url=arguments.sim_url,
            sim_token_file=arguments.sim_token_file,
        )
    except (OSError, ValueError, RobotGatewayConfigurationError, RuntimeConfigurationError) as error:
        print(f"co-arm dashboard cannot start: {error}", file=sys.stderr)
        return 2

    print(f"co-arm dashboard: http://{LOCAL_HOST}:{arguments.port}/")
    uvicorn.run(
        app,
        host=LOCAL_HOST,
        port=arguments.port,
        log_level="info",
        access_log=not arguments.no_access_log,
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
