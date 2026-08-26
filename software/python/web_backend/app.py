"""Standalone Co-Arm dashboard service.

This module serves only the browser-facing ARM and camera contracts needed by
the standalone dashboard.
"""

from __future__ import annotations

from contextlib import asynccontextmanager
from pathlib import Path
import secrets
from typing import Literal
from urllib.parse import urlsplit

from fastapi import APIRouter, Depends, FastAPI, HTTPException, Query, Request, Response
from fastapi.responses import FileResponse, JSONResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel, ConfigDict, Field
from starlette.middleware.trustedhost import TrustedHostMiddleware

from .arm_backends import ARM_BACKEND_HEADER, ArmBackendRegistry, BoundRobotGatewayClient
from .arm_control_center import (
    ArmControlCenterActionLaunchError,
    ArmControlCenterActionNotFound,
    ArmControlCenterActionUnavailable,
    ArmControlCenterService,
)
from .robot_gateway_client import (
    AssignServoIdRequest,
    ExecuteNudgeRequest,
    HoldSetRequest,
    PhysicalCalibrationProfileRequest,
    PhysicalResetRequest,
    PhysicalScanRequest,
    PrepareNudgeRequest,
    RobotGatewayClient,
    RobotGatewayError,
    ServoCaptureRequest,
    ServoMoveRequest,
    ServoOdometerRequest,
    ServoRegisterReadRequest,
    SetServoPositionModeRequest,
    SimpleArmLiveFollowEndRequest,
    SimpleArmLiveFollowFrameRequest,
    SimpleArmLiveFollowHeartbeatRequest,
    SimpleArmLiveFollowStartCancelRequest,
    SimpleArmLiveFollowStartRequest,
    SimpleArmPlanExecuteRequest,
    SimpleArmPlanPreviewRequest,
    SimpleArmSequenceExecuteCaptureRequest,
    SimpleArmSequenceExecuteRequest,
    SimpleArmSequencePreviewRequest,
    TorqueLeaseRequest,
    TorqueOffRequest,
)


SimpleJointId = Literal["joint_1", "joint_2", "joint_3", "joint_4"]


class SimpleCalibrateRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")

    rawZero: int | None = Field(default=None, ge=0, le=4095)
    rawMin: int | None = Field(default=None, ge=-262_144, le=262_144)
    rawMax: int | None = Field(default=None, ge=-262_144, le=262_144)
    minDegrees: float | None = Field(default=None, ge=-3600, le=3600)
    maxDegrees: float | None = Field(default=None, ge=-3600, le=3600)
    ratio: float | None = Field(default=None, gt=0.01, le=64)
    direction: Literal[-1, 1] | None = None
    speed: int | None = Field(default=None, ge=1, le=4095)
    accel: int | None = Field(default=None, ge=1, le=255)
    servoId: int | None = Field(default=None, ge=0, le=253)
    zeroFromLimits: bool | None = None
    clear: list[Literal["rawZero", "rawMin", "rawMax"]] | None = None
    here: list[Literal["rawZero", "rawMin", "rawMax"]] | None = None


class SimpleTargetRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    degrees: float = Field(ge=-3600, le=3600)


class SimpleMultiTargetRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    joint_1: float | None = Field(default=None, ge=-3600, le=3600)
    joint_2: float | None = Field(default=None, ge=-3600, le=3600)
    joint_3: float | None = Field(default=None, ge=-3600, le=3600)
    joint_4: float | None = Field(default=None, ge=-3600, le=3600)


class SimpleTorqueRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    hold: list[int] = Field(default_factory=list, max_length=4)


class SimpleFloorGuardRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    enabled: bool


class SimpleAssignIdRequest(BaseModel):
    model_config = ConfigDict(extra="forbid")
    oldId: int = Field(ge=0, le=253)
    newId: int = Field(ge=0, le=253)


def _gateway_failure(error: RobotGatewayError) -> HTTPException:
    return HTTPException(status_code=error.status_code, detail=str(error))


def create_app(
    *,
    project_root: Path,
    static_directory: Path | None = None,
    robot_gateway_client: RobotGatewayClient | None = None,
    arm_backend_registry: ArmBackendRegistry | None = None,
    arm_control_center: ArmControlCenterService | None = None,
) -> FastAPI:
    """Create the loopback-only ARM dashboard API.

    Gateway clients and local stores are constructed by ``web_backend.runtime``
    so tests can inject in-memory implementations without touching hardware.
    """

    if robot_gateway_client is not None and arm_backend_registry is not None:
        raise ValueError("Configure either one gateway client or a backend registry, not both.")

    @asynccontextmanager
    async def lifespan(_: FastAPI):
        try:
            yield
        finally:
            if arm_backend_registry is not None:
                arm_backend_registry.close()
            elif robot_gateway_client is not None:
                robot_gateway_client.close()
            if arm_control_center is not None:
                arm_control_center.close()

    app = FastAPI(
        title="co-arm local dashboard service",
        version="1.0.0",
        lifespan=lifespan,
    )
    app.add_middleware(
        TrustedHostMiddleware,
        allowed_hosts=["127.0.0.1", "localhost", "[::1]", "testserver"],
    )

    if arm_backend_registry is not None:

        @app.middleware("http")
        async def bind_arm_backend(request: Request, call_next):
            try:
                token = arm_backend_registry.activate_request(
                    request.headers.get(ARM_BACKEND_HEADER)
                )
            except RobotGatewayError as error:
                return JSONResponse(
                    status_code=error.status_code,
                    content={"detail": str(error)},
                )
            try:
                response = await call_next(request)
            finally:
                arm_backend_registry.reset_request(token)
            response.headers["X-Arm-Backend-Id"] = (
                request.headers.get(ARM_BACKEND_HEADER)
                or arm_backend_registry.default_backend_id
            ).strip().lower()
            return response

    action_token = secrets.token_urlsafe(32)

    def require_action_session(request: Request) -> None:
        origin = request.headers.get("origin")
        if origin is not None:
            try:
                supplied_origin = urlsplit(origin)
                local_origin = urlsplit(str(request.base_url))
                same_origin = (
                    supplied_origin.scheme == local_origin.scheme
                    and supplied_origin.hostname == local_origin.hostname
                    and supplied_origin.port == local_origin.port
                )
            except (TypeError, ValueError):
                same_origin = False
            if not same_origin:
                raise HTTPException(
                    status_code=403,
                    detail="This browser origin cannot use the local action session.",
                )
        supplied = request.headers.get("x-co-arm-token", "")
        if not supplied or not secrets.compare_digest(supplied, action_token):
            raise HTTPException(
                status_code=403,
                detail="This hardware action session is not current.",
            )

    def require_gateway() -> RobotGatewayClient | BoundRobotGatewayClient:
        if arm_backend_registry is not None:
            return arm_backend_registry.current_client()
        if robot_gateway_client is None:
            raise HTTPException(
                status_code=503,
                detail="No ARM gateway is configured. Start with a REAL or SIM gateway pair.",
            )
        return robot_gateway_client

    @app.get("/api/session")
    def browser_session(response: Response) -> dict[str, str]:
        response.headers["Cache-Control"] = "no-store"
        response.headers["Pragma"] = "no-cache"
        return {"actionToken": action_token}

    @app.get("/api/arm/backends")
    def arm_backends() -> dict[str, object]:
        if arm_backend_registry is not None:
            return arm_backend_registry.public_backends()
        if robot_gateway_client is not None:
            return {
                "backends": [
                    {
                        "backendId": "real",
                        "backendInstanceId": "standalone-real",
                        "displayName": "Real arm",
                        "simulated": False,
                        "configured": True,
                    }
                ],
                "defaultBackendId": "real",
                "selectionScope": "request",
            }
        return {"backends": [], "defaultBackendId": "sim", "selectionScope": "request"}

    # Fixed Control Center source actions ----------------------------------------
    @app.get("/api/arm/control-center/manifest")
    def control_center_manifest() -> dict[str, object]:
        if arm_control_center is None:
            raise HTTPException(status_code=503, detail="Arm Control Center is not configured.")
        return arm_control_center.manifest()

    @app.post(
        "/api/arm/control-center/actions/{action_id}",
        dependencies=[Depends(require_action_session)],
    )
    def control_center_action(action_id: str) -> dict[str, object]:
        if arm_control_center is None:
            raise HTTPException(status_code=503, detail="Arm Control Center is not configured.")
        try:
            return arm_control_center.run_action(action_id)
        except ArmControlCenterActionNotFound as error:
            raise HTTPException(status_code=404, detail=str(error)) from error
        except ArmControlCenterActionUnavailable as error:
            raise HTTPException(status_code=409, detail=str(error)) from error
        except ArmControlCenterActionLaunchError as error:
            raise HTTPException(
                status_code=503,
                detail="The fixed Control Center action could not be started.",
            ) from error

    # Camera and gateway-backed ARM routes ---------------------------------------
    @app.get("/api/camera/status")
    def camera_status() -> dict[str, object]:
        try:
            return require_gateway().camera_status()
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @app.post("/api/camera/autofocus", dependencies=[Depends(require_action_session)])
    def camera_autofocus() -> dict[str, object]:
        try:
            return require_gateway().autofocus_camera()
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @app.post("/api/camera/captures", dependencies=[Depends(require_action_session)])
    def camera_capture(
        profile: Literal["survey", "detail"] = Query("detail"),
    ) -> dict[str, object]:
        try:
            return require_gateway().capture_camera(profile)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @app.get(
        "/api/camera/observations/latest",
        dependencies=[Depends(require_action_session)],
    )
    def camera_latest() -> dict[str, object]:
        try:
            return require_gateway().latest_camera_observation()
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @app.get(
        "/api/camera/frames/{frame_id}",
        response_class=Response,
        dependencies=[Depends(require_action_session)],
    )
    def camera_frame(frame_id: str) -> Response:
        try:
            frame = require_gateway().camera_frame(frame_id)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error
        return Response(content=frame.content, media_type=frame.media_type, headers=frame.headers)

    @app.get(
        "/api/camera/transfers/{transfer_token}",
        response_class=Response,
        dependencies=[Depends(require_action_session)],
    )
    def camera_transfer(transfer_token: str) -> Response:
        try:
            frame = require_gateway().camera_transfer(transfer_token)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error
        return Response(content=frame.content, media_type=frame.media_type, headers=frame.headers)

    arm_router = APIRouter(
        prefix="/api/arm",
        dependencies=[Depends(require_action_session)],
    )

    @arm_router.get("/simple/state")
    def simple_state() -> dict[str, object]:
        try:
            return require_gateway().arm_state()
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/simple/scan")
    def simple_scan() -> dict[str, object]:
        try:
            return require_gateway().arm_scan()
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/simple/joints/{joint}/calibrate")
    def simple_calibrate(
        joint: SimpleJointId, body: SimpleCalibrateRequest
    ) -> dict[str, object]:
        try:
            return require_gateway().arm_calibrate(joint, body.model_dump(exclude_none=True))
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/simple/joints/{joint}/target")
    def simple_target(joint: SimpleJointId, body: SimpleTargetRequest) -> dict[str, object]:
        try:
            return require_gateway().arm_target(joint, body.degrees)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/simple/target")
    def simple_targets(body: SimpleMultiTargetRequest) -> dict[str, object]:
        try:
            return require_gateway().arm_targets(body.model_dump(exclude_none=True))
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/simple/live-follow/start")
    def live_follow_start(body: SimpleArmLiveFollowStartRequest) -> dict[str, object]:
        try:
            return require_gateway().arm_live_follow_start(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/simple/live-follow/start/cancel")
    def live_follow_start_cancel(
        body: SimpleArmLiveFollowStartCancelRequest,
    ) -> dict[str, object]:
        try:
            return require_gateway().arm_live_follow_start_cancel(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/simple/live-follow/frame")
    def live_follow_frame(body: SimpleArmLiveFollowFrameRequest) -> dict[str, object]:
        try:
            return require_gateway().arm_live_follow_frame(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/simple/live-follow/heartbeat")
    def live_follow_heartbeat(
        body: SimpleArmLiveFollowHeartbeatRequest,
    ) -> dict[str, object]:
        try:
            return require_gateway().arm_live_follow_heartbeat(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/simple/live-follow/end")
    def live_follow_end(body: SimpleArmLiveFollowEndRequest) -> dict[str, object]:
        try:
            return require_gateway().arm_live_follow_end(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/simple/plans/preview")
    def plan_preview(body: SimpleArmPlanPreviewRequest) -> dict[str, object]:
        try:
            return require_gateway().arm_plan_preview(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/simple/plans/execute")
    def plan_execute(body: SimpleArmPlanExecuteRequest) -> dict[str, object]:
        try:
            return require_gateway().arm_plan_execute(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/simple/sequences/preview")
    def sequence_preview(body: SimpleArmSequencePreviewRequest) -> dict[str, object]:
        try:
            return require_gateway().arm_sequence_preview(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/simple/sequences/execute")
    def sequence_execute(body: SimpleArmSequenceExecuteRequest) -> dict[str, object]:
        try:
            return require_gateway().arm_sequence_execute(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/simple/sequences/execute-and-capture")
    def sequence_execute_capture(
        body: SimpleArmSequenceExecuteCaptureRequest,
    ) -> dict[str, object]:
        try:
            return require_gateway().arm_sequence_execute_and_capture(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/simple/torque")
    def simple_torque(body: SimpleTorqueRequest) -> dict[str, object]:
        try:
            return require_gateway().arm_torque(body.hold)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/simple/floor-guard")
    def simple_floor_guard(body: SimpleFloorGuardRequest) -> dict[str, object]:
        try:
            return require_gateway().arm_floor_guard(body.enabled)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/simple/servo-id")
    def simple_servo_id(body: SimpleAssignIdRequest) -> dict[str, object]:
        try:
            return require_gateway().arm_assign_id(body.oldId, body.newId)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/simple/stop")
    def simple_stop() -> dict[str, object]:
        try:
            return require_gateway().arm_stop()
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/simple/clear-stop")
    def simple_clear_stop() -> dict[str, object]:
        try:
            return require_gateway().arm_clear_stop()
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.get("/physical/status")
    def physical_status() -> dict[str, object]:
        try:
            return require_gateway().physical_status()
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/physical/controller/reconnect")
    def physical_reconnect() -> dict[str, object]:
        try:
            return require_gateway().physical_reconnect()
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/physical/bus/scan")
    def physical_scan(body: PhysicalScanRequest) -> dict[str, object]:
        try:
            return require_gateway().physical_scan(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/physical/servos/assign-id")
    def physical_assign_id(body: AssignServoIdRequest) -> dict[str, object]:
        try:
            return require_gateway().physical_assign_id(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/physical/servos/set-position-mode")
    def physical_position_mode(body: SetServoPositionModeRequest) -> dict[str, object]:
        try:
            return require_gateway().physical_set_position_mode(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/physical/servos/capture")
    def physical_capture(body: ServoCaptureRequest) -> dict[str, object]:
        try:
            return require_gateway().physical_capture(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/physical/servos/move")
    def physical_move(body: ServoMoveRequest) -> dict[str, object]:
        try:
            return require_gateway().physical_move(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/physical/servos/registers/read")
    def physical_registers(body: ServoRegisterReadRequest) -> dict[str, object]:
        try:
            return require_gateway().physical_read_registers(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/physical/servos/odometer/zero")
    def physical_odometer_zero(body: ServoOdometerRequest) -> dict[str, object]:
        try:
            return require_gateway().physical_odometer_zero(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/physical/servos/odometer/read")
    def physical_odometer_read(body: ServoOdometerRequest) -> dict[str, object]:
        try:
            return require_gateway().physical_odometer_read(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/physical/servos/torque-lease")
    def physical_torque_lease(body: TorqueLeaseRequest) -> dict[str, object]:
        try:
            return require_gateway().physical_torque_lease(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/physical/servos/torque-off")
    def physical_torque_off(body: TorqueOffRequest) -> dict[str, object]:
        try:
            return require_gateway().physical_torque_off(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/physical/servos/hold-set")
    def physical_hold_set(body: HoldSetRequest) -> dict[str, object]:
        try:
            return require_gateway().physical_hold_set(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/physical/tests/prepare-nudge")
    def physical_prepare_nudge(body: PrepareNudgeRequest) -> dict[str, object]:
        try:
            return require_gateway().physical_prepare_nudge(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/physical/tests/execute-nudge")
    def physical_execute_nudge(body: ExecuteNudgeRequest) -> dict[str, object]:
        try:
            return require_gateway().physical_execute_nudge(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/physical/stop")
    def physical_stop() -> dict[str, object]:
        try:
            return require_gateway().physical_stop()
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/physical/reset")
    def physical_reset(body: PhysicalResetRequest) -> dict[str, object]:
        try:
            return require_gateway().physical_reset(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.get("/physical/calibration/profile")
    def physical_profile() -> dict[str, object]:
        try:
            return require_gateway().physical_profile()
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    @arm_router.post("/physical/calibration/profile")
    def physical_profile_commit(
        body: PhysicalCalibrationProfileRequest,
    ) -> dict[str, object]:
        try:
            return require_gateway().commit_physical_profile(body)
        except RobotGatewayError as error:
            raise _gateway_failure(error) from error

    app.include_router(arm_router)

    if static_directory is not None:
        index_path = static_directory / "index.html"
        assets_path = static_directory / "assets"
        if assets_path.is_dir():
            app.mount("/assets", StaticFiles(directory=assets_path), name="assets")

        @app.get("/", include_in_schema=False)
        def dashboard_index() -> FileResponse:
            if not index_path.is_file():
                raise HTTPException(status_code=503, detail="Build the dashboard first.")
            return FileResponse(index_path)

        @app.get("/{path:path}", include_in_schema=False)
        def dashboard_fallback(path: str) -> FileResponse:
            if path.startswith("api/") or not index_path.is_file():
                raise HTTPException(status_code=404, detail="Not found.")
            return FileResponse(index_path)

    return app
