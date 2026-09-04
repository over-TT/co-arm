"""Request-scoped SIM/REAL routing for the local Arm Control Center.

The browser chooses a backend on every request.  There is deliberately no
mutable process-wide "active arm": two tabs may inspect different backends
without racing one another, and a reviewed plan is consumed only by the
backend that prepared it.
"""

from __future__ import annotations

from dataclasses import dataclass
from contextvars import ContextVar, Token
from threading import RLock
import time
from typing import Callable, Literal

from .robot_gateway_client import (
    CameraFrameResponse,
    RobotGatewayClient,
    RobotGatewayError,
    SimpleArmPlanExecuteRequest,
    SimpleArmPlanPreviewRequest,
    SimpleArmSequenceExecuteRequest,
    SimpleArmSequencePreviewRequest,
)


ArmBackendId = Literal["sim", "real"]
ARM_BACKEND_HEADER = "x-arm-backend-id"
_PLAN_KINDS = frozenset({"plan", "sequence"})
_REQUEST_BACKEND: ContextVar[ArmBackendId | None] = ContextVar(
    "arm_request_backend", default=None
)


@dataclass(frozen=True, slots=True)
class ArmBackendDescriptor:
    backend_id: ArmBackendId
    display_name: str
    simulated: bool
    client: RobotGatewayClient
    instance_id: str | None
    require_upstream_identity: bool

    @classmethod
    def create(
        cls,
        backend_id: ArmBackendId,
        display_name: str,
        client: RobotGatewayClient,
        *,
        instance_id: str | None = None,
        require_upstream_identity: bool = False,
    ) -> "ArmBackendDescriptor":
        return cls(
            backend_id=backend_id,
            display_name=display_name,
            simulated=backend_id == "sim",
            client=client,
            instance_id=instance_id,
            require_upstream_identity=require_upstream_identity,
        )


@dataclass(frozen=True, slots=True)
class _PreparedBinding:
    backend_id: ArmBackendId
    instance_id: str
    expires_monotonic: float


class ArmBackendRegistry:
    """Own configured gateway clients and backend-bound reviewed artifacts."""

    def __init__(
        self,
        descriptors: list[ArmBackendDescriptor],
        *,
        default_backend_id: ArmBackendId = "real",
        monotonic_clock: Callable[[], float] = time.monotonic,
        binding_ttl_seconds: float = 120.0,
    ) -> None:
        if not descriptors:
            raise ValueError("At least one Arm backend must be configured.")
        by_id = {descriptor.backend_id: descriptor for descriptor in descriptors}
        if len(by_id) != len(descriptors):
            raise ValueError("Arm backend identifiers must be unique.")
        if default_backend_id not in by_id:
            raise ValueError("The default Arm backend must be configured.")
        if not 1.0 <= binding_ttl_seconds <= 600.0:
            raise ValueError("The reviewed-artifact binding TTL is invalid.")
        self._descriptors = by_id
        self.default_backend_id = default_backend_id
        self._clock = monotonic_clock
        self._binding_ttl_seconds = binding_ttl_seconds
        self._bindings: dict[
            tuple[str, str, str], set[_PreparedBinding]
        ] = {}
        self._lock = RLock()

    def normalize_backend_id(self, value: str | None) -> ArmBackendId:
        candidate = value.strip().lower() if isinstance(value, str) else ""
        if not candidate:
            return self.default_backend_id
        if candidate not in self._descriptors:
            raise RobotGatewayError(
                "That Arm backend is not configured.", status_code=422
            )
        return candidate  # type: ignore[return-value]

    def public_backends(self) -> dict[str, object]:
        rows = [
            {
                "backendId": descriptor.backend_id,
                "backendInstanceId": self._instance_id(descriptor),
                "displayName": descriptor.display_name,
                "simulated": descriptor.simulated,
                "configured": True,
            }
            for descriptor in sorted(
                self._descriptors.values(),
                key=lambda item: (not item.simulated, item.backend_id),
            )
        ]
        return {
            "backends": rows,
            "defaultBackendId": self.default_backend_id,
            "selectionScope": "request",
        }

    def _instance_id(
        self, descriptor: ArmBackendDescriptor, *, refresh: bool = False
    ) -> str | None:
        provider = getattr(descriptor.client, "backend_identity", None)
        if callable(provider):
            try:
                identity = provider(refresh=refresh)
            except RobotGatewayError:
                if descriptor.require_upstream_identity:
                    raise
                identity = None
            if isinstance(identity, dict):
                backend_id = identity.get("backendId")
                instance_id = identity.get("backendInstanceId")
                simulated = identity.get("simulated")
                if (
                    backend_id == descriptor.backend_id
                    and isinstance(instance_id, str)
                    and instance_id
                    and simulated is descriptor.simulated
                ):
                    return instance_id
                if descriptor.require_upstream_identity:
                    raise RobotGatewayError(
                        "The Arm backend returned a mismatched process identity.",
                        status_code=502,
                    )
        if descriptor.require_upstream_identity and refresh:
            raise RobotGatewayError(
                "The Arm backend did not provide a process identity.",
                status_code=502,
            )
        return descriptor.instance_id

    def _instance_id_from_response(
        self, descriptor: ArmBackendDescriptor, value: object
    ) -> str | None:
        observer = getattr(descriptor.client, "observe_backend_identity", None)
        if callable(observer):
            identity = observer(value)
            if isinstance(identity, dict):
                instance_id = identity.get("backendInstanceId")
                if isinstance(instance_id, str) and instance_id:
                    return instance_id
        if isinstance(value, dict):
            backend_id = value.get("backendId")
            instance_id = value.get("backendInstanceId")
            simulated = value.get("simulated")
            if (
                backend_id == descriptor.backend_id
                and isinstance(instance_id, str)
                and instance_id
                and simulated is descriptor.simulated
            ):
                return instance_id
        return None

    def bound_client(self, backend_id: ArmBackendId) -> "BoundRobotGatewayClient":
        try:
            descriptor = self._descriptors[backend_id]
        except KeyError:
            raise RobotGatewayError(
                "That Arm backend is not configured.", status_code=422
            ) from None
        return BoundRobotGatewayClient(self, descriptor)

    def activate_request(self, value: str | None) -> Token[ArmBackendId | None]:
        return _REQUEST_BACKEND.set(self.normalize_backend_id(value))

    @staticmethod
    def reset_request(token: Token[ArmBackendId | None]) -> None:
        _REQUEST_BACKEND.reset(token)

    def current_client(self) -> "BoundRobotGatewayClient":
        backend_id = _REQUEST_BACKEND.get() or self.default_backend_id
        return self.bound_client(backend_id)

    def current_backend_id(self) -> ArmBackendId:
        return _REQUEST_BACKEND.get() or self.default_backend_id

    def close(self) -> None:
        seen: set[int] = set()
        for descriptor in self._descriptors.values():
            identity = id(descriptor.client)
            if identity in seen:
                continue
            seen.add(identity)
            descriptor.client.close()

    def _purge_expired(self) -> None:
        now = self._clock()
        for key, bindings in list(self._bindings.items()):
            current = {
                binding for binding in bindings if binding.expires_monotonic > now
            }
            if current:
                self._bindings[key] = current
            else:
                self._bindings.pop(key, None)

    def bind_reviewed_artifact(
        self,
        *,
        kind: str,
        identifier: str,
        digest: str,
        descriptor: ArmBackendDescriptor,
        instance_id: str,
        expires_in_ms: object = None,
    ) -> None:
        if kind not in _PLAN_KINDS or not identifier or not digest:
            raise RobotGatewayError(
                "The Arm backend returned an incomplete reviewed artifact.",
                status_code=502,
            )
        ttl = self._binding_ttl_seconds
        if (
            isinstance(expires_in_ms, (int, float))
            and not isinstance(expires_in_ms, bool)
            and 0 < float(expires_in_ms) <= 600_000
        ):
            ttl = min(ttl, float(expires_in_ms) / 1000.0)
        binding = _PreparedBinding(
            descriptor.backend_id,
            instance_id,
            self._clock() + ttl,
        )
        with self._lock:
            self._purge_expired()
            self._bindings.setdefault((kind, identifier, digest), set()).add(binding)

    def require_reviewed_artifact(
        self,
        *,
        kind: str,
        identifier: str,
        digest: str,
        descriptor: ArmBackendDescriptor,
        instance_id: str,
    ) -> None:
        key = (kind, identifier, digest)
        with self._lock:
            self._purge_expired()
            bindings = self._bindings.get(key, set())
            expected = _PreparedBinding(
                descriptor.backend_id,
                instance_id,
                0.0,
            )
            matches = any(
                binding.backend_id == expected.backend_id
                and binding.instance_id == expected.instance_id
                for binding in bindings
            )
        if not matches:
            raise RobotGatewayError(
                "That reviewed Arm plan belongs to another backend or is no longer current. Preview it again.",
                status_code=409,
            )

    def consume_reviewed_artifact(
        self,
        *,
        kind: str,
        identifier: str,
        digest: str,
        descriptor: ArmBackendDescriptor,
        instance_id: str,
    ) -> None:
        key = (kind, identifier, digest)
        with self._lock:
            bindings = self._bindings.get(key)
            if bindings is None:
                return
            remaining = {
                binding
                for binding in bindings
                if not (
                    binding.backend_id == descriptor.backend_id
                    and binding.instance_id == instance_id
                )
            }
            if remaining:
                self._bindings[key] = remaining
            else:
                self._bindings.pop(key, None)


class BoundRobotGatewayClient:
    """Decorate one gateway client with immutable request provenance."""

    def __init__(
        self,
        registry: ArmBackendRegistry,
        descriptor: ArmBackendDescriptor,
    ) -> None:
        self._registry = registry
        self._descriptor = descriptor
        self._client = descriptor.client

    def _instance_id(self, *, refresh: bool = False) -> str | None:
        return self._registry._instance_id(self._descriptor, refresh=refresh)

    def _require_operation_response_instance(
        self, value: object, expected_instance_id: str
    ) -> str:
        observed_instance_id = self._registry._instance_id_from_response(
            self._descriptor, value
        )
        if (
            observed_instance_id is not None
            and observed_instance_id != expected_instance_id
        ):
            raise RobotGatewayError(
                "The Arm backend process changed while the request was in flight. Preview it again.",
                status_code=409,
            )
        return observed_instance_id or expected_instance_id

    def _decorate(self, value: object, *, instance_id: str | None = None) -> object:
        resolved_instance = instance_id if instance_id is not None else self._instance_id()
        if isinstance(value, dict):
            return {
                **value,
                "backendId": self._descriptor.backend_id,
                "backendInstanceId": resolved_instance,
                "simulated": self._descriptor.simulated,
            }
        if isinstance(value, CameraFrameResponse):
            headers = {
                **value.headers,
                "X-Arm-Backend-Id": self._descriptor.backend_id,
                "X-Simulated": str(self._descriptor.simulated).lower(),
            }
            if resolved_instance is not None:
                headers["X-Arm-Backend-Instance"] = resolved_instance
            return CameraFrameResponse(
                content=value.content,
                media_type=value.media_type,
                headers=headers,
            )
        return value

    def arm_plan_preview(
        self, request: SimpleArmPlanPreviewRequest
    ) -> dict[str, object]:
        instance_id = self._instance_id(refresh=True)
        if instance_id is None:
            raise RobotGatewayError(
                "The Arm backend did not provide a process identity.", status_code=502
            )
        result = self._client.arm_plan_preview(request)
        instance_id = self._require_operation_response_instance(
            result, instance_id
        )
        identifier = result.get("planId")
        digest = result.get("planDigest", result.get("digest"))
        if not isinstance(identifier, str) or not isinstance(digest, str):
            raise RobotGatewayError(
                "The Arm backend returned an incomplete reviewed plan.",
                status_code=502,
            )
        self._registry.bind_reviewed_artifact(
            kind="plan",
            identifier=identifier,
            digest=digest,
            descriptor=self._descriptor,
            instance_id=instance_id,
            expires_in_ms=result.get("expiresInMs"),
        )
        return self._decorate(result, instance_id=instance_id)  # type: ignore[return-value]

    def arm_plan_execute(
        self, request: SimpleArmPlanExecuteRequest
    ) -> dict[str, object]:
        instance_id = self._instance_id(refresh=True)
        if instance_id is None:
            raise RobotGatewayError(
                "The Arm backend did not provide a process identity.", status_code=502
            )
        self._registry.require_reviewed_artifact(
            kind="plan",
            identifier=request.planId,
            digest=request.planDigest,
            descriptor=self._descriptor,
            instance_id=instance_id,
        )
        result = self._client.arm_plan_execute(request)
        instance_id = self._require_operation_response_instance(
            result, instance_id
        )
        self._registry.consume_reviewed_artifact(
            kind="plan",
            identifier=request.planId,
            digest=request.planDigest,
            descriptor=self._descriptor,
            instance_id=instance_id,
        )
        return self._decorate(result, instance_id=instance_id)  # type: ignore[return-value]

    def arm_sequence_preview(
        self, request: SimpleArmSequencePreviewRequest
    ) -> dict[str, object]:
        instance_id = self._instance_id(refresh=True)
        if instance_id is None:
            raise RobotGatewayError(
                "The Arm backend did not provide a process identity.", status_code=502
            )
        result = self._client.arm_sequence_preview(request)
        instance_id = self._require_operation_response_instance(
            result, instance_id
        )
        identifier = result.get("sequenceId")
        digest = result.get("sequenceDigest")
        if not isinstance(identifier, str) or not isinstance(digest, str):
            raise RobotGatewayError(
                "The Arm backend returned an incomplete reviewed sequence.",
                status_code=502,
            )
        self._registry.bind_reviewed_artifact(
            kind="sequence",
            identifier=identifier,
            digest=digest,
            descriptor=self._descriptor,
            instance_id=instance_id,
            expires_in_ms=result.get("expiresInMs"),
        )
        return self._decorate(result, instance_id=instance_id)  # type: ignore[return-value]

    def _execute_sequence(
        self,
        request: SimpleArmSequenceExecuteRequest,
        *,
        capture: bool,
    ) -> dict[str, object]:
        instance_id = self._instance_id(refresh=True)
        if instance_id is None:
            raise RobotGatewayError(
                "The Arm backend did not provide a process identity.", status_code=502
            )
        self._registry.require_reviewed_artifact(
            kind="sequence",
            identifier=request.sequenceId,
            digest=request.sequenceDigest,
            descriptor=self._descriptor,
            instance_id=instance_id,
        )
        method = (
            self._client.arm_sequence_execute_and_capture
            if capture
            else self._client.arm_sequence_execute
        )
        result = method(request)
        instance_id = self._require_operation_response_instance(
            result, instance_id
        )
        self._registry.consume_reviewed_artifact(
            kind="sequence",
            identifier=request.sequenceId,
            digest=request.sequenceDigest,
            descriptor=self._descriptor,
            instance_id=instance_id,
        )
        return self._decorate(result, instance_id=instance_id)  # type: ignore[return-value]

    def arm_sequence_execute(
        self, request: SimpleArmSequenceExecuteRequest
    ) -> dict[str, object]:
        return self._execute_sequence(request, capture=False)

    def arm_sequence_execute_and_capture(
        self, request: SimpleArmSequenceExecuteRequest
    ) -> dict[str, object]:
        return self._execute_sequence(request, capture=True)

    def close(self) -> None:
        # The registry owns the underlying client lifetime.
        return None

    def __getattr__(self, name: str):
        attribute = getattr(self._client, name)
        if not callable(attribute):
            return attribute

        def call(*args: object, **kwargs: object):
            result = attribute(*args, **kwargs)
            # Prefer the identity bound to the evidence itself. SIM state and
            # future Pi responses carry it directly, avoiding an extra health
            # and state round trip on every dashboard poll. A SIM response that
            # lacks provenance must still prove the bridge process explicitly.
            instance_id = self._registry._instance_id_from_response(
                self._descriptor, result
            )
            if instance_id is None and self._descriptor.require_upstream_identity:
                instance_id = self._instance_id(refresh=True)
            elif instance_id is None:
                instance_id = self._instance_id()
            return self._decorate(
                result, instance_id=instance_id
            )

        return call


__all__ = [
    "ARM_BACKEND_HEADER",
    "ArmBackendDescriptor",
    "ArmBackendId",
    "ArmBackendRegistry",
    "BoundRobotGatewayClient",
]
