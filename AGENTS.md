# Agent instructions for co-arm

Read `README.md`, `docs/STATUS.md`, `docs/CONTROL_AND_SAFETY.md`, and
`docs/AI_CONTEXT.md` before changing arm behavior or documentation.

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
