# Security and safety reporting

## Security findings

Do not open a public issue containing credentials, private network details,
device identifiers, raw logs, unreviewed camera frames, or a reproducible path
to unsafe physical motion. Contact the repository owner privately through the
security-reporting channel configured on the GitHub repository. If no private
channel is configured yet, withhold sensitive details until one exists.

Include the affected revision, software component, minimal reproduction,
expected fail-closed behavior, and whether hardware was connected. Use a
simulator or disposable fixture whenever possible.

## Immediate physical concern

If the arm behaves unexpectedly:

1. Use the reachable physical servo-power cut.
2. Support elevated links before torque is removed.
3. Keep people and loose objects out of the sweep.
4. Do not retry the same command until telemetry, configuration continuity,
   and the physical pose are understood.
5. Preserve only sanitized evidence.

A software STOP is useful but is not a certified emergency stop. The camera is
not a safety sensor, and the planner is not general 3D collision avoidance.
