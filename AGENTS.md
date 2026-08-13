# Agent instructions for co-arm

Read `README.md`, `docs/STATUS.md`, `docs/CONTROL_AND_SAFETY.md`, and
`docs/AI_CONTEXT.md` before changing arm behavior or documentation.

For setup, Raspberry Pi connection, assembly, or commissioning work, read
`docs/SETUP_WITH_CODEX.md` first. Inventory the actual checkout before giving
install commands. The current public tree is documentation-first; when the
software folders contain only placeholder READMEs, continue with documentation,
parts intake, or read-only Pi discovery and say that executable setup is
waiting for the source release. Never invent package, service, flash, or motion
commands.

During a guided setup:

- keep one checklist with confirmed, unknown, blocked, and not-yet-applicable
  states;
- ask for a critical physical fact only when its stage is reached;
- begin a connected-Pi session with read-only OS, Python, and camera discovery;
- explain where every command runs, whether it writes or moves anything, what
  success proves, and what the stop condition is;
- require explicit operator approval for wiring changes, flashing, deployment,
  torque changes, STOP reset, guard changes, and motion;
- keep private hosts, usernames, addresses, key paths, device identifiers, and
  captured desk content out of committed files.

Keep these invariants:

- Never infer current pose from documentation or a named viewpoint.
- Keep source/build, live-telemetry, operator, and image proof separate.
- Never claim arrival from command acceptance alone.
- The installed OV5647 is fixed focus; do not describe standoff selection as
  optical autofocus.
- Base continuity loss requires physical alignment and an explicit re-home;
  never reconstruct an output turn from a wrapped encoder alone.
- Preserve the floor guard and the Pi/ESP32 safety split.
- Physical motion, torque release, firmware flashing, deployment, and safety
  bypasses require deliberate operator authority and live preconditions.
- Never publish personal paths, credentials context, exact device IDs, network
  topology, raw camera frames, transcripts, account state, or personal memory.
- New CAD/media must pass the repository provenance and privacy workflow.

Run `python tools/check_repo.py` before proposing a commit.
