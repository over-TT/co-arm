---
name: arm-alliance
description: Use a configured four-joint Raspberry Pi camera arm to look around, preview and run moves, read joint state, or do a requested lightweight desk task.
---

# Arm Alliance

Use this skill after the local MCP server is configured with
`ARM_GATEWAY_URL`, `ARM_EXPECTED_BACKEND=sim|real`,
`ARM_GATEWAY_TOKEN_FILE`, and `ARM_FRAME_DIR`.

## Working style

Keep it practical. Once the user asks for an arm task, do not ask for the same
permission again at every step.

The normal loop is:

1. read current state and scene;
2. take a current picture when the task needs one;
3. preview the move or route;
4. run it;
5. check the joints or camera for the result.

Stop only for a real blocker: STOP, an offline controller or servo bus, stale
required joints, lost Base position, a refused path, or something actually in
the planned sweep.

## Pick the task

- **Look or identify:** use `arm_look`, then move the viewpoint only when a
  better view is needed. Judge the returned picture, not the name of a pose.
- **Move to a view:** use `arm_plan` and `arm_apply_and_look` when one pose is
  enough.
- **Follow a known route:** use one `arm_plan_sequence` and one
  `arm_apply_sequence` for two to eight waypoints.
- **Move a lightweight object:** get one current picture, make sure the contact
  part of the route actually crosses the object in the useful direction, run
  one route with enough follow-through, then take one result picture.

Do not add extra overview poses, crops, or identification passes when the target
and route are already clear.

## Physical facts

- Torque off makes Shoulder and Elbow go limp. Hold them if they are raised.
  `arm_release` asks for `confirmed_torque_release=true` because this changes
  the physical hold state.
- If the Base loses its place, line up the physical zero mark before using
  `arm_set_base_zero`; the tool asks for `confirmed_physical_zero=true`.
- Turning off the floor check asks for
  `confirmed_floor_guard_disable=true`. Do not use that as a workaround for a
  bad route.
- A move reply is not the same as measured arrival, and measured arrival is not
  the same as seeing the requested result. Check the one the task needs.

For camera work, Base changes the direction around the desk,
Shoulder/Elbow change height and distance, and Camera changes aim. The reference
build uses Camera Module 3 Wide with autofocus. Judge focus from the returned
image, not only from a successful autofocus command.

On REAL, the camera's nominal 102 x 67 degree FOV is a framing hint, not a
pixel-to-desk calibration. Use `deskProjection` numerically only when its
`status` starts with `available_`; `unavailable_uncalibrated` forbids converting
pixels to desk millimetres or Base bearing. SIM may expose its separate
effective square-pixel pinhole projection (102 x 69.57 degrees, fitted on the
horizontal axis); never treat that authored SIM model as physical-camera proof.
