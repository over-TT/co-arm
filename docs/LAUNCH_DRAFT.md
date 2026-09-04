# Launch draft

Draft only. Use after the release blockers in [Release review](RELEASE_REVIEW.md)
are resolved, the owner approves public terms/media, and the exact source is
available at the public repository link. Do not describe this as open source
until the chosen licenses are actually present.

## Release title

co-arm v0.1 — a four-axis desk camera arm

## Release summary

I built co-arm to give an agent a camera it can physically move around my desk.
It uses a Raspberry Pi, an ESP32 servo controller, printed parts, and a small
dashboard. Codex connects through typed tools to inspect state, preview a move,
and check the returned joint readings and camera image.

This release contains the focused software stack and 12 printable parts:
dashboard, Pi gateway, controller firmware, agent tools, simulator source, tests,
and setup/build notes. Start with the dashboard quickstart; it works without
connecting any hardware.

It is an experimental reference build. The camera is not a calibrated depth
sensor, the arm has no gripper, and software tests do not establish mechanical
reliability. Keep any unfinished assembly or recovery items explicitly listed
in the released limitations.

## X post

> I built co-arm: a 3D-printed desk camera arm that Codex can move through typed
> tools. It previews a route, checks joint readings, and takes a real image.
>
> Source, 12 printable parts, and build notes: REPO_LINK
>
> Still a prototype. Here's a real run.

Replace `REPO_LINK` with the verified public URL. Attach an approved real clip;
do not use a simulator recording without labeling it.

## Clip outline

- Show the physical arm and the plain-language inspection request.
- Briefly show the preview, then the arm moving in the same continuous shot.
- Show the returned image and what it actually reveals.
- End on the repository name/link.

Keep one attempt understandable. If it needed a retry, show or disclose it.
Avoid speed, accuracy, autonomy, or reliability claims that were not measured.
