from __future__ import annotations

from threading import Event, Thread, get_ident
import time
import unittest

from arm_sim.bridge.client import (
    BridgeClient,
    BridgeProtocolError,
    BridgeRemoteError,
    BridgeTimeoutError,
)
from arm_sim.bridge.engine import InMemoryBridgeEngine
from arm_sim.bridge.server import LoopbackBridgeServer


TOKEN = "s" * 32


class _SlowHealthEngine(InMemoryBridgeEngine):
    def health(self) -> dict[str, object]:
        time.sleep(0.15)
        return super().health()


class _CaptureProfileRecordingEngine(InMemoryBridgeEngine):
    def __init__(self) -> None:
        super().__init__()
        self.capture_profiles: list[str] = []

    def capture(self, profile: str = "detail") -> dict[str, object]:
        self.capture_profiles.append(profile)
        return super().capture(profile=profile)


class _ThreadRecordingEngine(InMemoryBridgeEngine):
    def __init__(self) -> None:
        super().__init__()
        self.health_thread_id: int | None = None

    def health(self) -> dict[str, object]:
        self.health_thread_id = get_ident()
        return super().health()


class BridgeIpcTests(unittest.TestCase):
    def start_server(self, engine=None) -> tuple[LoopbackBridgeServer, Thread]:
        server = LoopbackBridgeServer(
            engine=engine or InMemoryBridgeEngine(), token=TOKEN, port=0
        )
        thread = Thread(target=server.serve_forever, daemon=True)
        thread.start()

        def cleanup() -> None:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)

        self.addCleanup(cleanup)
        return server, thread

    def test_round_trip_pins_backend_identity_and_preserves_state(self) -> None:
        server, _ = self.start_server()
        client = BridgeClient(token=TOKEN, port=server.address[1])

        health = client.health()
        self.assertEqual(health.backend_id, "sim")
        self.assertTrue(health.simulated)
        self.assertTrue(health.backend_instance_id.startswith("sim_"))
        self.assertEqual(health.result["calibrationStatus"], "provisional")

        moved = client.set_joint_targets(
            targets={"joint_1": 12.5, "joint_4": -8.0}, duration_ms=0
        )
        self.assertEqual(moved.backend_instance_id, health.backend_instance_id)
        self.assertEqual(moved.result["jointPositionsDegrees"]["joint_1"], 12.5)
        self.assertEqual(client.get_state().result["stateRevision"], 1)

        captured = client.capture()
        self.assertEqual(captured.result["mimeType"], "image/jpeg")
        self.assertEqual(captured.result["stateRevision"], 1)

        reset = client.reset(seed=7)
        self.assertEqual(reset.result["stateRevision"], 2)
        self.assertEqual(
            reset.result["jointPositionsDegrees"],
            {"joint_1": 0.0, "joint_2": 0.0, "joint_3": 0.0, "joint_4": 0.0},
        )

    def test_capture_profile_reaches_engine_and_default_is_detail(self) -> None:
        engine = _CaptureProfileRecordingEngine()
        server, _ = self.start_server(engine)
        client = BridgeClient(token=TOKEN, port=server.address[1])

        client.capture(profile="survey")
        client.capture()

        self.assertEqual(engine.capture_profiles, ["survey", "detail"])

    def test_bad_token_is_rejected_without_engine_access(self) -> None:
        server, _ = self.start_server()
        client = BridgeClient(token="x" * 32, port=server.address[1])
        with self.assertRaises(BridgeRemoteError) as raised:
            client.health()
        self.assertEqual(raised.exception.code, "UNAUTHORIZED")

    def test_service_action_runs_on_the_serialized_engine_thread(self) -> None:
        engine = _ThreadRecordingEngine()
        action_called = Event()
        action_thread_ids: list[int] = []

        def service_action() -> None:
            action_thread_ids.append(get_ident())
            action_called.set()

        server = LoopbackBridgeServer(
            engine=engine,
            token=TOKEN,
            port=0,
            service_action=service_action,
        )
        thread = Thread(
            target=server.serve_forever,
            kwargs={"poll_interval": 0.01},
            daemon=True,
        )
        thread.start()
        try:
            BridgeClient(token=TOKEN, port=server.address[1]).health()
            self.assertTrue(action_called.wait(timeout=1.0))
            self.assertEqual(engine.health_thread_id, thread.ident)
            self.assertTrue(action_thread_ids)
            self.assertTrue(
                all(thread_id == engine.health_thread_id for thread_id in action_thread_ids)
            )
        finally:
            server.shutdown()
            server.server_close()
            thread.join(timeout=2.0)

    def test_service_action_must_be_callable(self) -> None:
        with self.assertRaisesRegex(TypeError, "service_action must be callable"):
            LoopbackBridgeServer(
                engine=InMemoryBridgeEngine(),
                token=TOKEN,
                port=0,
                service_action=object(),  # type: ignore[arg-type]
            )

    def test_client_timeout_is_bounded(self) -> None:
        server, _ = self.start_server(_SlowHealthEngine())
        client = BridgeClient(
            token=TOKEN,
            port=server.address[1],
            request_timeout_seconds=0.03,
        )
        with self.assertRaises(BridgeTimeoutError):
            client.health()

    def test_process_restart_requires_explicit_reconnect(self) -> None:
        first = LoopbackBridgeServer(
            engine=InMemoryBridgeEngine(), token=TOKEN, port=0
        )
        port = first.address[1]
        first_thread = Thread(target=first.serve_forever, daemon=True)
        first_thread.start()
        client = BridgeClient(token=TOKEN, port=port)
        first_instance = client.health().backend_instance_id
        first.shutdown()
        first.server_close()
        first_thread.join(timeout=2.0)

        second = LoopbackBridgeServer(
            engine=InMemoryBridgeEngine(), token=TOKEN, port=port
        )
        second_thread = Thread(target=second.serve_forever, daemon=True)
        second_thread.start()
        try:
            self.assertNotEqual(second.backend_instance_id, first_instance)
            with self.assertRaises(BridgeProtocolError):
                client.health()
            client.reconnect()
            self.assertEqual(
                client.health().backend_instance_id,
                second.backend_instance_id,
            )
        finally:
            second.shutdown()
            second.server_close()
            second_thread.join(timeout=2.0)


if __name__ == "__main__":
    unittest.main()
