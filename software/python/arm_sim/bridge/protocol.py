"""Schema validation for the loopback Arm/Isaac JSON-lines protocol.

This is intentionally standard-library-only so the exact same code runs in
Isaac Sim's Python 3.12 runtime and in the gateway's Python 3.11 runtime.
Requests cannot contain paths, Python expressions, USD prim names, or arbitrary
method names. Only the five bounded public commands below enter the agent path.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
import base64
import binascii
import hashlib
import math
import re
from typing import Mapping


PROTOCOL_NAME = "arm-sim-bridge.v1"
BACKEND_ID = "sim"
COMMANDS = frozenset(
    {"health", "reset", "get_state", "set_joint_targets", "capture"}
)
CAPTURE_PROFILES = frozenset({"survey", "detail"})
DEFAULT_CAPTURE_PROFILE = "detail"
JOINT_IDS = ("joint_1", "joint_2", "joint_3", "joint_4")
JOINT_LIMITS_DEGREES: dict[str, tuple[float, float]] = {
    "joint_1": (-180.0, 180.0),
    "joint_2": (-90.0, 90.0),
    "joint_3": (-90.0, 90.0),
    "joint_4": (-90.0, 90.0),
}
MAX_REQUEST_BYTES = 64 * 1024
MAX_RESPONSE_BYTES = 24 * 1024 * 1024
MAX_CAPTURE_BYTES = 16 * 1024 * 1024
MIN_TOKEN_LENGTH = 32
MAX_TOKEN_LENGTH = 256
MAX_DURATION_MS = 120_000
MAX_SEED = 2_147_483_647

_SAFE_REQUEST_ID = re.compile(r"^[A-Za-z0-9_-]{8,128}$")
_SAFE_INSTANCE_ID = re.compile(r"^sim_[A-Za-z0-9_-]{12,96}$")
_SAFE_FRAME_ID = re.compile(r"^[A-Za-z0-9_-]{8,128}$")
_SAFE_ENGINE_NAME = re.compile(r"^[a-z][a-z0-9_-]{1,63}$")
_SAFE_METADATA_ID = re.compile(r"^[a-z0-9][a-z0-9._-]{1,127}$")
_SHA256 = re.compile(r"^sha256:[0-9a-f]{64}$")
_ERROR_CODES = frozenset(
    {
        "INVALID_REQUEST",
        "UNAUTHORIZED",
        "ENGINE_ERROR",
        "UNSUPPORTED_COMMAND",
    }
)


class ProtocolViolation(ValueError):
    """A message is not a canonical bridge protocol value."""


@dataclass(frozen=True, slots=True)
class ValidatedRequest:
    request_id: str
    token: str
    command: str
    params: dict[str, object]


def _object(value: object, label: str) -> Mapping[str, object]:
    if not isinstance(value, Mapping):
        raise ProtocolViolation(f"{label} must be an object")
    if not all(isinstance(key, str) for key in value):
        raise ProtocolViolation(f"{label} keys must be strings")
    return value


def _exact_keys(
    value: Mapping[str, object],
    *,
    required: set[str],
    optional: set[str] | None = None,
    label: str,
) -> None:
    optional = optional or set()
    keys = set(value)
    missing = required - keys
    extra = keys - required - optional
    if missing or extra:
        raise ProtocolViolation(f"{label} has unexpected fields")


def _finite_number(value: object, label: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ProtocolViolation(f"{label} must be a finite number")
    converted = float(value)
    if not math.isfinite(converted):
        raise ProtocolViolation(f"{label} must be a finite number")
    return converted


def _bounded_float(
    value: object, minimum: float, maximum: float, label: str
) -> float:
    converted = _finite_number(value, label)
    if not minimum <= converted <= maximum:
        raise ProtocolViolation(f"{label} is outside its allowed range")
    return converted


def _bounded_int(value: object, minimum: int, maximum: int, label: str) -> int:
    if isinstance(value, bool) or not isinstance(value, int):
        raise ProtocolViolation(f"{label} must be an integer")
    if not minimum <= value <= maximum:
        raise ProtocolViolation(f"{label} is outside its allowed range")
    return value


def _timestamp(value: object, label: str) -> str:
    if not isinstance(value, str) or not 1 <= len(value) <= 64:
        raise ProtocolViolation(f"{label} must be a bounded timestamp")
    try:
        parsed = datetime.fromisoformat(value.replace("Z", "+00:00"))
    except ValueError:
        raise ProtocolViolation(f"{label} must be an ISO-8601 timestamp") from None
    if parsed.tzinfo is None or parsed.utcoffset() is None:
        raise ProtocolViolation(f"{label} must include a timezone")
    return value


def _metadata_id(value: object, label: str) -> str:
    if not isinstance(value, str) or _SAFE_METADATA_ID.fullmatch(value) is None:
        raise ProtocolViolation(f"{label} must be a bounded identifier")
    return value


def _metadata_text(value: object, label: str, *, optional: bool = False) -> str | None:
    if optional and value is None:
        return None
    if (
        not isinstance(value, str)
        or not value
        or len(value) > 1_024
        or not value.isprintable()
    ):
        raise ProtocolViolation(f"{label} must be bounded text")
    return value


def _validate_camera_profile_metadata(value: object) -> dict[str, object]:
    """Bound the public simulator camera profile carried by health replies."""

    profile = _object(value, "cameraProfile")
    _exact_keys(
        profile,
        required={
            "profileId",
            "cameraId",
            "sensorModel",
            "referenceSensorModel",
            "identityConfidence",
            "calibrationStatus",
            "renderProfiles",
            "projection",
            "focusModel",
        },
        label="cameraProfile",
    )
    render_profiles = _object(profile["renderProfiles"], "cameraProfile.renderProfiles")
    _exact_keys(
        render_profiles,
        required={"survey", "detail"},
        label="cameraProfile.renderProfiles",
    )
    normalized_render: dict[str, object] = {}
    for name in ("survey", "detail"):
        render = _object(render_profiles[name], f"cameraProfile.renderProfiles.{name}")
        _exact_keys(
            render,
            required={"widthPx", "heightPx"},
            optional={"limitation"},
            label=f"cameraProfile.renderProfiles.{name}",
        )
        normalized: dict[str, object] = {
            "widthPx": _bounded_int(render["widthPx"], 1, 16_384, "widthPx"),
            "heightPx": _bounded_int(render["heightPx"], 1, 16_384, "heightPx"),
        }
        if "limitation" in render:
            normalized["limitation"] = _metadata_text(
                render["limitation"], "render limitation", optional=True
            )
        normalized_render[name] = normalized

    projection = _object(profile["projection"], "cameraProfile.projection")
    _exact_keys(
        projection,
        required={
            "model",
            "nominalFitAxis",
            "horizontalFovDeg",
            "verticalFovDeg",
            "status",
            "limitation",
        },
        label="cameraProfile.projection",
    )
    focus = _object(profile["focusModel"], "cameraProfile.focusModel")
    _exact_keys(
        focus,
        required={"physicalTargetCapability", "reportedCapability", "status"},
        label="cameraProfile.focusModel",
    )
    return {
        "profileId": _metadata_id(profile["profileId"], "cameraProfile.profileId"),
        "cameraId": _metadata_id(profile["cameraId"], "cameraProfile.cameraId"),
        "sensorModel": _metadata_id(
            profile["sensorModel"], "cameraProfile.sensorModel"
        ),
        "referenceSensorModel": _metadata_id(
            profile["referenceSensorModel"], "cameraProfile.referenceSensorModel"
        ),
        "identityConfidence": _metadata_id(
            profile["identityConfidence"], "cameraProfile.identityConfidence"
        ),
        "calibrationStatus": _metadata_id(
            profile["calibrationStatus"], "cameraProfile.calibrationStatus"
        ),
        "renderProfiles": normalized_render,
        "projection": {
            "model": _metadata_id(projection["model"], "projection.model"),
            "nominalFitAxis": _metadata_id(
                projection["nominalFitAxis"], "projection.nominalFitAxis"
            ),
            "horizontalFovDeg": _bounded_float(
                projection["horizontalFovDeg"], 0.01, 179.0, "horizontalFovDeg"
            ),
            "verticalFovDeg": _bounded_float(
                projection["verticalFovDeg"], 0.01, 179.0, "verticalFovDeg"
            ),
            "status": _metadata_id(projection["status"], "projection.status"),
            "limitation": _metadata_text(
                projection["limitation"], "projection.limitation"
            ),
        },
        "focusModel": {
            "physicalTargetCapability": _metadata_id(
                focus["physicalTargetCapability"], "focusModel.physicalTargetCapability"
            ),
            "reportedCapability": _metadata_id(
                focus["reportedCapability"], "focusModel.reportedCapability"
            ),
            "status": _metadata_id(focus["status"], "focusModel.status"),
        },
    }


def _joint_positions(
    value: object, *, label: str, require_all: bool
) -> dict[str, float]:
    supplied = _object(value, label)
    keys = set(supplied)
    allowed = set(JOINT_IDS)
    if not keys or not keys <= allowed or (require_all and keys != allowed):
        raise ProtocolViolation(f"{label} contains invalid joint ids")
    normalized: dict[str, float] = {}
    for joint_id in JOINT_IDS:
        if joint_id not in supplied:
            continue
        degrees = _finite_number(supplied[joint_id], f"{label}.{joint_id}")
        minimum, maximum = JOINT_LIMITS_DEGREES[joint_id]
        if not minimum <= degrees <= maximum:
            raise ProtocolViolation(f"{label}.{joint_id} is outside simulator limits")
        normalized[joint_id] = degrees
    return normalized


def _validate_params(command: str, value: object) -> dict[str, object]:
    params = _object(value, "params")
    if command in {"health", "get_state"}:
        _exact_keys(params, required=set(), label="params")
        return {}
    if command == "capture":
        _exact_keys(params, required=set(), optional={"profile"}, label="params")
        profile = params.get("profile", DEFAULT_CAPTURE_PROFILE)
        if not isinstance(profile, str) or profile not in CAPTURE_PROFILES:
            raise ProtocolViolation("capture profile must be survey or detail")
        return {"profile": profile}
    if command == "reset":
        _exact_keys(params, required=set(), optional={"seed"}, label="params")
        normalized: dict[str, object] = {}
        if "seed" in params:
            normalized["seed"] = _bounded_int(params["seed"], 0, MAX_SEED, "seed")
        return normalized
    if command == "set_joint_targets":
        _exact_keys(
            params,
            required={"targets", "durationMs"},
            label="params",
        )
        return {
            "targets": _joint_positions(
                params["targets"], label="targets", require_all=False
            ),
            "durationMs": _bounded_int(
                params["durationMs"], 0, MAX_DURATION_MS, "durationMs"
            ),
        }
    raise ProtocolViolation("unsupported command")


def validate_request(value: object) -> ValidatedRequest:
    request = _object(value, "request")
    _exact_keys(
        request,
        required={"protocol", "requestId", "token", "command", "params"},
        label="request",
    )
    if request["protocol"] != PROTOCOL_NAME:
        raise ProtocolViolation("unsupported protocol")
    request_id = request["requestId"]
    if not isinstance(request_id, str) or _SAFE_REQUEST_ID.fullmatch(request_id) is None:
        raise ProtocolViolation("requestId is invalid")
    token = request["token"]
    if (
        not isinstance(token, str)
        or not MIN_TOKEN_LENGTH <= len(token) <= MAX_TOKEN_LENGTH
        or any(character.isspace() or not character.isprintable() for character in token)
    ):
        raise ProtocolViolation("token is invalid")
    command = request["command"]
    if not isinstance(command, str) or command not in COMMANDS:
        raise ProtocolViolation("unsupported command")
    return ValidatedRequest(
        request_id=request_id,
        token=token,
        command=command,
        params=_validate_params(command, request["params"]),
    )


def validate_backend(value: object) -> dict[str, object]:
    backend = _object(value, "backend")
    _exact_keys(
        backend,
        required={"backendId", "backendInstanceId", "simulated"},
        label="backend",
    )
    instance_id = backend["backendInstanceId"]
    if (
        backend["backendId"] != BACKEND_ID
        or backend["simulated"] is not True
        or not isinstance(instance_id, str)
        or _SAFE_INSTANCE_ID.fullmatch(instance_id) is None
    ):
        raise ProtocolViolation("backend identity is invalid")
    return dict(backend)


def _validate_state(value: object) -> dict[str, object]:
    state = _object(value, "state result")
    _exact_keys(
        state,
        required={
            "stateRevision",
            "jointPositionsDegrees",
            "moving",
            "stopped",
            "capturedAt",
        },
        label="state result",
    )
    if not isinstance(state["moving"], bool) or not isinstance(state["stopped"], bool):
        raise ProtocolViolation("state flags must be booleans")
    return {
        "stateRevision": _bounded_int(
            state["stateRevision"], 0, 9_007_199_254_740_991, "stateRevision"
        ),
        "jointPositionsDegrees": _joint_positions(
            state["jointPositionsDegrees"],
            label="jointPositionsDegrees",
            require_all=True,
        ),
        "moving": state["moving"],
        "stopped": state["stopped"],
        "capturedAt": _timestamp(state["capturedAt"], "capturedAt"),
    }


def validate_result(command: str, value: object) -> dict[str, object]:
    if command == "health":
        result = _object(value, "health result")
        _exact_keys(
            result,
            required={"status", "engine", "calibrationStatus"},
            optional={"cameraProfile"},
            label="health result",
        )
        engine = result["engine"]
        if (
            result["status"] != "ok"
            or not isinstance(engine, str)
            or _SAFE_ENGINE_NAME.fullmatch(engine) is None
            or result["calibrationStatus"]
            not in {"provisional", "calibrated", "unknown"}
        ):
            raise ProtocolViolation("health result is invalid")
        normalized = {
            "status": "ok",
            "engine": engine,
            "calibrationStatus": result["calibrationStatus"],
        }
        if "cameraProfile" in result:
            normalized["cameraProfile"] = _validate_camera_profile_metadata(
                result["cameraProfile"]
            )
        return normalized
    if command in {"reset", "get_state", "set_joint_targets"}:
        return _validate_state(value)
    if command == "capture":
        result = _object(value, "capture result")
        _exact_keys(
            result,
            required={
                "frameId",
                "mimeType",
                "width",
                "height",
                "capturedAt",
                "stateRevision",
                "jointPositionsDegrees",
                "sha256",
                "dataBase64",
            },
            label="capture result",
        )
        frame_id = result["frameId"]
        digest = result["sha256"]
        encoded = result["dataBase64"]
        if not isinstance(frame_id, str) or _SAFE_FRAME_ID.fullmatch(frame_id) is None:
            raise ProtocolViolation("frameId is invalid")
        if result["mimeType"] != "image/jpeg":
            raise ProtocolViolation("capture must be a JPEG")
        if not isinstance(digest, str) or _SHA256.fullmatch(digest) is None:
            raise ProtocolViolation("capture sha256 is invalid")
        if not isinstance(encoded, str) or len(encoded) > ((MAX_CAPTURE_BYTES + 2) // 3) * 4:
            raise ProtocolViolation("capture payload is too large")
        try:
            data = base64.b64decode(encoded, validate=True)
        except (binascii.Error, ValueError):
            raise ProtocolViolation("capture payload is not canonical base64") from None
        if not data or len(data) > MAX_CAPTURE_BYTES:
            raise ProtocolViolation("capture payload size is invalid")
        actual = f"sha256:{hashlib.sha256(data).hexdigest()}"
        if actual != digest:
            raise ProtocolViolation("capture digest does not match its bytes")
        return {
            "frameId": frame_id,
            "mimeType": "image/jpeg",
            "width": _bounded_int(result["width"], 1, 8192, "width"),
            "height": _bounded_int(result["height"], 1, 8192, "height"),
            "capturedAt": _timestamp(result["capturedAt"], "capturedAt"),
            "stateRevision": _bounded_int(
                result["stateRevision"], 0, 9_007_199_254_740_991, "stateRevision"
            ),
            "jointPositionsDegrees": _joint_positions(
                result["jointPositionsDegrees"],
                label="jointPositionsDegrees",
                require_all=True,
            ),
            "sha256": digest,
            "dataBase64": encoded,
        }
    raise ProtocolViolation("unsupported command")


def validate_response(
    value: object, *, expected_request_id: str, command: str
) -> tuple[dict[str, object], dict[str, object] | None, dict[str, str] | None]:
    response = _object(value, "response")
    required = {"protocol", "requestId", "ok", "backend"}
    if response.get("ok") is True:
        _exact_keys(response, required=required | {"result"}, label="response")
    elif response.get("ok") is False:
        _exact_keys(response, required=required | {"error"}, label="response")
    else:
        raise ProtocolViolation("response ok must be a boolean")
    if response["protocol"] != PROTOCOL_NAME or response["requestId"] != expected_request_id:
        raise ProtocolViolation("response identity does not match request")
    backend = validate_backend(response["backend"])
    if response["ok"] is True:
        return backend, validate_result(command, response["result"]), None
    error = _object(response["error"], "error")
    _exact_keys(error, required={"code", "message"}, label="error")
    code, message = error["code"], error["message"]
    if (
        not isinstance(code, str)
        or code not in _ERROR_CODES
        or not isinstance(message, str)
        or not 1 <= len(message) <= 160
    ):
        raise ProtocolViolation("error response is invalid")
    return backend, None, {"code": code, "message": message}


def backend_identity(instance_id: str) -> dict[str, object]:
    return validate_backend(
        {
            "backendId": BACKEND_ID,
            "backendInstanceId": instance_id,
            "simulated": True,
        }
    )


def decoded_capture_bytes(result: Mapping[str, object]) -> bytes:
    """Decode a capture that has already passed ``validate_result``."""

    encoded = result.get("dataBase64")
    if not isinstance(encoded, str):
        raise ProtocolViolation("capture result is missing dataBase64")
    return base64.b64decode(encoded, validate=True)


def capture_result(
    *,
    frame_id: str,
    data: bytes,
    width: int,
    height: int,
    captured_at: str,
    state_revision: int,
    joint_positions_degrees: Mapping[str, float],
) -> dict[str, object]:
    """Build and self-validate the only binary-bearing result type."""

    candidate = {
        "frameId": frame_id,
        "mimeType": "image/jpeg",
        "width": width,
        "height": height,
        "capturedAt": captured_at,
        "stateRevision": state_revision,
        "jointPositionsDegrees": dict(joint_positions_degrees),
        "sha256": f"sha256:{hashlib.sha256(data).hexdigest()}",
        "dataBase64": base64.b64encode(data).decode("ascii"),
    }
    return validate_result("capture", candidate)


__all__ = [
    "BACKEND_ID",
    "CAPTURE_PROFILES",
    "COMMANDS",
    "DEFAULT_CAPTURE_PROFILE",
    "JOINT_IDS",
    "JOINT_LIMITS_DEGREES",
    "MAX_CAPTURE_BYTES",
    "MAX_DURATION_MS",
    "MAX_REQUEST_BYTES",
    "MAX_RESPONSE_BYTES",
    "PROTOCOL_NAME",
    "ProtocolViolation",
    "ValidatedRequest",
    "backend_identity",
    "capture_result",
    "decoded_capture_bytes",
    "validate_request",
    "validate_response",
    "validate_result",
]
