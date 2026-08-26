"""Strict local IPC boundary between Isaac Sim and the robot gateway.

The package deliberately imports no Isaac modules.  An Isaac 3.12 process
provides a :class:`BridgeEngine` implementation to ``LoopbackBridgeServer``;
the ordinary Python 3.11 gateway uses ``BridgeClient`` and the adapters in
``robot_gateway.isaac_bridge``.
"""

from .client import (
    BridgeClient,
    BridgeClientError,
    BridgeProtocolError,
    BridgeRemoteError,
    BridgeReply,
    BridgeTimeoutError,
    BridgeUnavailableError,
)
from .engine import BridgeEngine, InMemoryBridgeEngine
from .protocol import BACKEND_ID, COMMANDS, JOINT_IDS, PROTOCOL_NAME
from .server import LoopbackBridgeServer

__all__ = [
    "BACKEND_ID",
    "COMMANDS",
    "JOINT_IDS",
    "PROTOCOL_NAME",
    "BridgeClient",
    "BridgeClientError",
    "BridgeEngine",
    "BridgeProtocolError",
    "BridgeRemoteError",
    "BridgeReply",
    "BridgeTimeoutError",
    "BridgeUnavailableError",
    "InMemoryBridgeEngine",
    "LoopbackBridgeServer",
]
