"""One command that answers "is the arm actually there?".

The panel shows an empty list for several different reasons -- controller not
answering, bus unpowered, STOP latched, servos at ids nothing is looking for --
and they need different fixes. This prints which one it is.

    python scripts/arm_status.py
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import urllib.error
import urllib.request
from pathlib import Path

from arm_mcp.config import GatewayConfigurationError, validated_loopback_gateway_url

SOFTWARE_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_GATEWAY = os.environ.get("ARM_GATEWAY_URL", "http://127.0.0.1:8787")
TOKEN_FILE = SOFTWARE_ROOT / "runtime" / "robot-gateway" / "arm-pi.token"


class _NoRedirectHandler(urllib.request.HTTPRedirectHandler):
    """Keep the bearer token on the one validated loopback origin."""

    def redirect_request(self, request, fp, code, msg, headers, newurl):  # noqa: ANN001, ANN201
        return None


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--token-file", type=Path, default=TOKEN_FILE)
    parser.add_argument("--gateway-url", default=DEFAULT_GATEWAY)
    parser.add_argument(
        "--scan",
        action="store_true",
        help=(
            "run a commissioning bus scan before reading state; this is not a "
            "read-only ping: it clears proposals, may request torque-off, and "
            "can clear Base truth until recalibrated"
        ),
    )
    arguments = parser.parse_args()
    try:
        gateway = validated_loopback_gateway_url(arguments.gateway_url)
    except GatewayConfigurationError as error:
        raise SystemExit(f"Invalid --gateway-url: {error}") from error

    if not arguments.token_file.exists():
        raise SystemExit(f"No gateway token at {arguments.token_file}.")
    token = arguments.token_file.read_text(encoding="utf-8").strip()
    headers = {"Authorization": f"Bearer {token}", "Content-Type": "application/json"}
    opener = urllib.request.build_opener(
        urllib.request.ProxyHandler({}),
        _NoRedirectHandler(),
    )

    def call(path: str, post: bool = False) -> dict:
        request = urllib.request.Request(
            f"{gateway}{path}", headers=headers, method="POST" if post else "GET",
            data=b"{}" if post else None,
        )
        with opener.open(request, timeout=10) as response:
            return json.loads(response.read())

    if arguments.scan:
        try:
            call("/api/robot/arm/scan", post=True)
            print("scan: ok")
        except urllib.error.HTTPError as error:
            body = error.read().decode()[:200]
            print(f"scan: HTTP {error.code} {body}")
            # 503 is the controller being absent; 502 is it answering badly. Both
            # mean the ESP32, not the servos, so say so before listing joints.
            print("  -> 503 = controller not connected (HAT unpowered, or UART busy)")
            print("  -> 502 = controller link unhealthy (wrong baud, or a second master on UART0)")

    try:
        state = call("/api/robot/arm/state")
    except urllib.error.HTTPError as error:
        raise SystemExit(f"state: HTTP {error.code} {error.read().decode()[:300]}")
    except urllib.error.URLError as error:
        raise SystemExit(f"Cannot reach the configured gateway at {gateway}. ({error})")

    print(
        f"controller={state.get('connection')}  bus={state.get('bus')}  "
        f"stopped={state.get('stopped')}  held={state.get('held')}  refreshMs={state.get('refreshMs')}"
    )
    if state.get("stopped"):
        print("  !! STOP is latched -- every command is refused. Press Clear STOP in the panel.")
    online = 0
    for joint in state.get("joints", []):
        online += bool(joint["online"])
        reach = (
            f"{joint['reachMin']:.1f}..{joint['reachMax']:.1f} deg"
            if joint.get("reachMin") is not None else "uncalibrated"
        )
        print(
            f"  {joint['name']:9} servo {joint['servoId']}  "
            f"{'ONLINE ' if joint['online'] else 'offline'}  "
            f"raw={joint['rawPosition']}  torque={joint['torque']}  {reach}"
        )
    if online == 0:
        print("\nNo servo answered. In order of likelihood:")
        print("  1. servo rail off -- it powers the servos AND the HAT's own regulator")
        print("  2. the HAT is not seated on the Pi header, or its UART cable is out")
        print("  3. a second master on UART0 (laptop USB still plugged into the HAT)")
        print("  4. the servos hold ids nothing is looking for -- rerun with --scan")
    return 0


if __name__ == "__main__":
    sys.exit(main())
