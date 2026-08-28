# Vision and active perception

The end camera is an evidence sensor for observation and framing. It is not a
collision sensor, safety-rated device, or substitute for live joint telemetry.

## Reference camera behavior

The current reference build uses Raspberry Pi Camera Module 3 Wide. Its IMX708
sensor exposes a 4608 x 2592 native array and powered autofocus. The bounded
capture profiles use 2304 x 1296 for routine `survey` evidence and 4608 x 2592
for `detail`. Survey prefers continuous normal/full-range focus; detail may
request macro focus. The camera is physically mounted upside down, and the Pi
applies one 180-degree transform so newly delivered images are upright.

The gateway checks the driver-reported sensor and array against the explicitly
selected Wide profile; IMX708 alone cannot prove the lens variant. A focus
control request does not prove the subject became sharp. Use the same capture's
AF state/lens metadata together with visible fine detail in the subject region.
The older OV5647 path remains a fixed-focus compatibility profile, not the
current reference camera.

The simulator is deliberately separate from that physical-lens contract. Its
2304 x 1296 square-pixel pinhole model fits the nominal 102 degree horizontal
axis and therefore has a 69.57 degree effective vertical axis, not the physical
lens's nominal 67 degrees. SIM projection metadata exposes that effective FOV
and its fit axis. On REAL, the nominal 102 x 67 degree values are framing hints,
not calibrated pixel-to-desk geometry.

## Camera ray convention

- Camera servo `0 deg` points along the forearm.
- Positive Camera rotates the view downward.
- Negative Camera rotates the view upward.
- In the side-view model:

```text
forearm_absolute = (shoulder_servo + 90) + (elbow_servo - 90)
camera_ray       = forearm_absolute - camera_servo
```

Base centers the view horizontally around the desk. Shoulder and Elbow choose
height and standoff. Camera tilt changes vertical framing without changing the
kinematic endpoint.

## Three kinds of visual evidence

| Interface | Evidence | Important limitation |
| --- | --- | --- |
| `arm_scene` | Calculated side-view joints, tip, floor, and optical ray from measured angles | Not real pixels; does not show obstacles or framing |
| `arm_look` survey/detail | A newly captured upright JPEG plus near-capture pose and metadata | Proves only what is visible in that frame |
| `arm_detail` | A bounded crop from a retained full-resolution capture | No movement and no new photograph |

Use `deskProjection` numerically only when its `status` starts with
`available_`. In particular, `unavailable_uncalibrated` on REAL means the
camera intrinsics, lens distortion, mount extrinsics, or desk registration are
not sufficient to convert normalized pixels into desk millimetres or Base
bearing.

Capture retention is bounded. Treat the source token as ephemeral runtime data,
not a durable public identifier.

## Default active-perception loop

### 1. Establish live geometry

Read `arm_state`, then `arm_scene`. Do not use a remembered viewpoint as the
current pose. Confirm controller/bus health, trusted measured angles for every
needed joint, STOP clear, floor guard enabled, and a clear planned sweep.

### 2. Make a deliberate wide survey

Prepare an elevated, retracted, downward-looking whole-desk view with **Base
exactly `0 deg`**. Derive Shoulder, Elbow, and Camera from the current measured
scene and the assembled arm's verified calibration. Review the solid measured
arm, dashed proposal, exact angles, warnings, and swept clearance; then apply
the one-use plan.

Settle and verify measured arrival. Capture `arm_look(view='survey')`.

Arrival at the intended pose is necessary but not sufficient. The actual
pixels must visibly cover the useful desk area. If they do not, calculate one
bounded coordinated correction from the live scene and image error, plan/apply
it, and capture another survey before localizing.

### 3. Localize without another photograph

Use `arm_detail` on the successful survey's retained full-resolution source.
This is a digital crop: it makes no movement and takes no second picture. Use it
to select a candidate region and estimate how Base, standoff, height, and camera
tilt should change.

### 4. Make a centered physical close pass

Center primarily with Base. Use Shoulder/Elbow for height and distance, and
Camera for vertical aim. Preview the coordinated sweep, apply it, settle, read
measured state, and take a new capture. Use another retained-source crop before
requesting a full-resolution complete frame.

### 5. Rank useful detail

Judge focus first from the subject region:

- fine edges;
- readable silkscreen or labels;
- connector shapes;
- chip markings;
- distinctive layout.

`FocusFoM`, when present, is a relative same-frame libcamera metric. Compare it
only for the same subject at similar framing. A whole-frame score can rise
because the background is sharp while the target is not. Visible subject detail
outranks the score. For the current autofocus camera, require a settled AF
state/lens position from the same frame; for the legacy fixed-focus profile,
change standoff when a closer sample is visibly softer.

### 6. Repeat while evidence improves

There is no arbitrary fixed count of moves, captures, minutes, or iterations.
Continue bounded, reviewed viewpoints while they materially improve evidence.
Stop when the request is answered, useful progress is exhausted, a concrete
blocker appears, or the operator says stop.

## Identification standard

A wide survey can support a location or family-level candidate. It is not
enough for an exact model. Exact identification requires discriminating
close-frame evidence such as readable text, characteristic ports, chip labels,
or a unique board layout. If those cues are absent or soft, report uncertainty
and the next view that would resolve it.

## Named viewpoints

A named viewpoint is a calibration-scoped **destination**, never live state.
Store it only after:

1. measured arrival was verified;
2. pixels proved the intended coverage;
3. the camera/bracket and joint calibration revision were recorded;
4. the operator asked to retain it or confirmed it as useful.

Invalidate/reverify it after camera mount, bracket, link geometry, zero, limit,
or calibration changes. Device-specific viewpoint angles and proof-frame
identifiers belong in local calibration records, not the public generic guide.

## Camera and capture failure boundaries

- A stale wall-clock timestamp does not by itself prove a stale camera feed;
  compare two deliberate captures or use monotonic capture age where available.
- A successful shutter request followed by an image-load failure is different
  from a camera capture failure; report the failed stage.
- If the active compatibility profile reports fixed focus, do not call
  standoff search "autofocus."
- If AF controls are merely configured, do not claim the lens focused.
- If a survey token has expired from bounded retention, recapture instead of
  guessing its pixels.
- A correct `arm_scene` ray with an unhelpful photo means the geometry estimate
  was not sufficient for the real scene; use the pixels to correct the next
  bounded view.

## Privacy and GitHub media

Desk captures may contain faces, screens, labels, addresses, account data, or
other private material. Frames are runtime evidence and are never automatically
safe to publish. Before adding a photo or video to `media/`:

- review the entire frame and audio track;
- crop or redact unrelated personal information;
- remove embedded location/device metadata where appropriate;
- use descriptive filenames rather than runtime frame identifiers;
- say whether the media shows a real run, a staged demonstration, or a
  simulation.

See [`CONTROL_AND_SAFETY.md`](CONTROL_AND_SAFETY.md) for motion preconditions
and [`TROUBLESHOOTING.md`](TROUBLESHOOTING.md) for camera-specific diagnosis.
