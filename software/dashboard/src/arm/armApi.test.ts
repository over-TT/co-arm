import { describe, expect, it, vi } from "vitest";
import { ArmApiError, createCameraApi, createPhysicalArmApi, createSimpleArmApi } from "./armApi";
import type { PhysicalCalibrationProfilePayload } from "./armTypes";

function response(body: unknown) {
  return new Response(JSON.stringify(body), {
    status: 200,
    headers: { "Content-Type": "application/json" },
  });
}

function liveReceipt(overrides: Record<string, unknown> = {}) {
  return {
    schema: "arm-live-follow-v2",
    sessionId: "live_abc123",
    state: "active",
    reason: null,
    settings: {
      joint_2: { speed: 400, accel: 10, maxDeltaDegrees: 30 },
      joint_3: { speed: 400, accel: 10, maxDeltaDegrees: 30 },
    },
    envelope: {
      maxDeltaDegrees: { joint_2: 30, joint_3: 30 },
      floorMm: 40,
      lowestPointMm: 80,
      joint_2: { minDegrees: 0, maxDegrees: 0 },
      joint_3: { minDegrees: 0, maxDegrees: 0 },
    },
    commanded: { joint_2: 0, joint_3: 0 },
    measured: {
      joint_2: { degrees: 0, rawPosition: 2048, velocityDegreesPerSecond: 0, moving: false, packetAgeMs: 8 },
      joint_3: { degrees: 0, rawPosition: 2048, velocityDegreesPerSecond: 0, moving: false, packetAgeMs: 8 },
    },
    stats: {
      receivedFrames: 0,
      dispatchedFrames: 0,
      coalescedFrames: 0,
      clampedFrames: 0,
      lastAcceptedSequence: 0,
      lastDispatchedSequence: 0,
      lastDispatchLatencyMs: null,
    },
    timing: {
      inputLeaseMs: 400,
      maxDurationMs: 30_000,
      minDispatchIntervalMs: 50,
      startedAt: "2026-08-21T00:00:00Z",
      expiresAt: "2026-08-21T00:00:30Z",
    },
    ...overrides,
  };
}

describe("arm API contract", () => {
  it("reads camera status without attaching action credentials", async () => {
    const request = vi.fn<typeof fetch>(async () => response({
      available: true,
      state: "available",
      latestFrameId: null,
    }));

    await createCameraApi(request, "s".repeat(40)).status();

    expect(request).toHaveBeenCalledWith(
      "/api/camera/status",
      expect.objectContaining({ headers: { Accept: "application/json" } }),
    );
  });

  it("sends exact detail and survey profiles with the local action credential", async () => {
    const actionToken = "c".repeat(40);
    const request = vi.fn<typeof fetch>(async () => response({
      frameId: "frame_abc123",
      frameUrl: "/api/camera/frames/frame_abc123",
    }));

    const api = createCameraApi(request, actionToken);
    await api.capture();
    await api.capture("survey");

    expect(request).toHaveBeenNthCalledWith(
      1,
      "/api/camera/captures?profile=detail",
      expect.objectContaining({
        method: "POST",
        headers: expect.objectContaining({ "X-Co-Arm-Token": actionToken }),
      }),
    );
    expect(request).toHaveBeenNthCalledWith(
      2,
      "/api/camera/captures?profile=survey",
      expect.objectContaining({
        method: "POST",
        headers: expect.objectContaining({ "X-Co-Arm-Token": actionToken }),
      }),
    );
  });

  it("posts the exact manual autofocus request with the action credential and no body", async () => {
    const actionToken = "a".repeat(40);
    const request = vi.fn<typeof fetch>(async () => response({
      simulated: false,
      physicalArmMotion: false,
      attempted: true,
      result: "focused",
      autofocus: {
        capability: "supported",
        mode: "continuous",
        state: "focused",
        range: "macro",
      },
    }));

    await createCameraApi(request, actionToken).autofocus();

    expect(request).toHaveBeenCalledTimes(1);
    expect(request).toHaveBeenCalledWith("/api/camera/autofocus", {
      method: "POST",
      headers: {
        Accept: "application/json",
        "X-Co-Arm-Token": actionToken,
      },
    });
    expect(request.mock.calls[0]?.[1]).not.toHaveProperty("body");
  });

  it("rejects autofocus without a local action token before fetching", async () => {
    const request = vi.fn<typeof fetch>();

    await expect(createCameraApi(request, null).autofocus()).rejects.toThrow(
      "A local action session is required to try camera autofocus.",
    );

    expect(request).not.toHaveBeenCalled();
  });

  it("reads the latest observation through the authenticated fixed local route", async () => {
    const actionToken = "l".repeat(40);
    const request = vi.fn<typeof fetch>(async () => response({
      frameId: "frame_latest",
      frameUrl: "/api/camera/frames/frame_latest",
    }));

    await createCameraApi(request, actionToken).latest();

    expect(request).toHaveBeenCalledWith(
      "/api/camera/observations/latest",
      expect.objectContaining({
        headers: expect.objectContaining({ "X-Co-Arm-Token": actionToken }),
      }),
    );
  });

  it("loads JPEG evidence through an authenticated validated frame route", async () => {
    const actionToken = "f".repeat(40);
    const request = vi.fn<typeof fetch>(async () => new Response(new Blob(["jpeg"], { type: "image/jpeg" }), {
      status: 200,
      headers: { "Content-Type": "image/jpeg", "Cache-Control": "no-store" },
    }));

    const frame = await createCameraApi(request, actionToken).frame("camera_frame_1");

    expect(frame.type).toBe("image/jpeg");
    expect(request).toHaveBeenCalledWith(
      "/api/camera/frames/camera_frame_1",
      expect.objectContaining({
        headers: expect.objectContaining({
          Accept: "image/jpeg",
          "X-Co-Arm-Token": actionToken,
        }),
      }),
    );
  });

  it("previews without motion and executes only the displayed one-time plan", async () => {
    const actionToken = "p".repeat(40);
    const request = vi.fn<typeof fetch>(async (input) => {
      const url = String(input);
      if (url.endsWith("/plans/preview")) return response({
        planId: "plan_abc",
        planDigest: `sha256:${"b".repeat(64)}`,
        resolvedPose: { joint_2: 14 },
        measuredPose: { joint_2: 0 },
        warnings: [],
        lowestClearanceMm: 18,
      });
      return response({ executed: true, planId: "plan_abc", resolvedPose: { joint_2: 14 }, moved: ["joint_2"] });
    });
    const api = createSimpleArmApi(request, actionToken);

    const plan = await api.previewPlan({ joint_2: 14 });
    await api.executePlan(plan);

    expect(request.mock.calls[0]?.[0]).toBe("/api/arm/simple/plans/preview");
    expect(JSON.parse(String(request.mock.calls[0]?.[1]?.body))).toEqual({ targets: { joint_2: 14 } });
    expect(request.mock.calls[1]?.[0]).toBe("/api/arm/simple/plans/execute");
    expect(JSON.parse(String(request.mock.calls[1]?.[1]?.body))).toEqual({
      planId: "plan_abc",
      planDigest: `sha256:${"b".repeat(64)}`,
    });
  });

  it("uses the action token and exact bounded live-follow request contract", async () => {
    const actionToken = "l".repeat(40);
    const startAttemptId = "frontend_attempt_0001";
    const request = vi.fn<typeof fetch>(async (input) => {
      const url = String(input);
      if (url.endsWith("/start/cancel")) return response({
        schema: "arm-live-follow-v2",
        startAttemptId,
        cancelled: true,
        inputLeaseMs: 400,
      });
      return response(liveReceipt({
        ...(url.endsWith("/start") ? { startAttemptId } : {}),
        ...(url.endsWith("/end") ? { state: "ended" } : {}),
      }));
    });
    const api = createSimpleArmApi(request, actionToken);

    await api.startLiveFollow({
      joint_2: { speed: 400, accel: 10, maxDeltaDegrees: 30 },
      joint_3: { speed: 1400, accel: 45, maxDeltaDegrees: 45 },
    }, { startAttemptId, startTimeoutMs: 1_500 });
    await api.cancelLiveFollowStart(startAttemptId);
    await api.sendLiveFollowFrame("live_abc123", 1, { joint_2: 12.5, joint_3: -18 });
    await api.heartbeatLiveFollow("live_abc123");
    await api.endLiveFollow("live_abc123", true);

    expect(request.mock.calls.map(([url]) => url)).toEqual([
      "/api/arm/simple/live-follow/start",
      "/api/arm/simple/live-follow/start/cancel",
      "/api/arm/simple/live-follow/frame",
      "/api/arm/simple/live-follow/heartbeat",
      "/api/arm/simple/live-follow/end",
    ]);
    expect(JSON.parse(String(request.mock.calls[0]?.[1]?.body))).toEqual({
      joint_2: { speed: 400, accel: 10, maxDeltaDegrees: 30 },
      joint_3: { speed: 1400, accel: 45, maxDeltaDegrees: 45 },
      startAttemptId,
      startTimeoutMs: 1_500,
    });
    expect(JSON.parse(String(request.mock.calls[1]?.[1]?.body))).toEqual({ startAttemptId });
    expect(JSON.parse(String(request.mock.calls[2]?.[1]?.body))).toEqual({
      sessionId: "live_abc123",
      sequence: 1,
      joint_2: 12.5,
      joint_3: -18,
    });
    expect(JSON.parse(String(request.mock.calls[3]?.[1]?.body))).toEqual({ sessionId: "live_abc123" });
    expect(JSON.parse(String(request.mock.calls[4]?.[1]?.body))).toEqual({ sessionId: "live_abc123", flushPending: true });
    for (const [, init] of request.mock.calls) {
      expect(init?.headers).toEqual(expect.objectContaining({ "X-Co-Arm-Token": actionToken }));
    }
  });

  it("rejects an otherwise valid start receipt bound to another attempt", async () => {
    const request = vi.fn<typeof fetch>(async () => response(liveReceipt({ startAttemptId: "another_attempt_0002" })));
    const api = createSimpleArmApi(request, "l".repeat(40));

    await expect(api.startLiveFollow({
      joint_2: { speed: 400, accel: 10, maxDeltaDegrees: 30 },
      joint_3: { speed: 400, accel: 10, maxDeltaDegrees: 30 },
    }, { startAttemptId: "frontend_attempt_0001", startTimeoutMs: 1_500 })).rejects.toMatchObject({
      status: 502,
      message: "The arm returned an invalid live-follow receipt.",
    });
  });

  it.each(["frame", "heartbeat", "end"] as const)(
    "rejects a %s receipt bound to a different live-follow session",
    async (route) => {
      const request = vi.fn<typeof fetch>(async () => response(liveReceipt({
        sessionId: "live_other_session",
        ...(route === "end" ? { state: "ended" } : {}),
      })));
      const api = createSimpleArmApi(request, "l".repeat(40));
      const operation = route === "frame"
        ? api.sendLiveFollowFrame("live_abc123", 1, { joint_2: 12.5, joint_3: -18 })
        : route === "heartbeat"
          ? api.heartbeatLiveFollow("live_abc123")
          : api.endLiveFollow("live_abc123", false);

      await expect(operation).rejects.toMatchObject({
        status: 502,
        message: "The arm returned an invalid live-follow receipt.",
      });
    },
  );

  it("aborts a stalled heartbeat well before the 400 ms Pi input lease", async () => {
    vi.useFakeTimers();
    try {
      let abortedAt: number | null = null;
      const request = vi.fn<typeof fetch>(async (_input, init) => await new Promise<Response>((resolve, reject) => {
        const slowReply = globalThis.setTimeout(() => resolve(response(liveReceipt())), 350);
        init?.signal?.addEventListener("abort", () => {
          abortedAt = performance.now();
          globalThis.clearTimeout(slowReply);
          reject(new DOMException("The operation was aborted.", "AbortError"));
        }, { once: true });
      }));
      const outcome = createSimpleArmApi(request, "l".repeat(40))
        .heartbeatLiveFollow("live_abc123")
        .then((value) => ({ value, error: null }), (error: unknown) => ({ value: null, error }));

      await vi.advanceTimersByTimeAsync(400);
      const result = await outcome;

      expect(result.value).toBeNull();
      expect(result.error).toBeInstanceOf(ArmApiError);
      expect(result.error).toMatchObject({ status: 408 });
      expect((result.error as Error).message).toMatch(/heartbeat timed out/i);
      expect(abortedAt).not.toBeNull();
      expect(abortedAt!).toBeLessThan(400);
      expect(request.mock.calls[0]?.[1]?.signal?.aborted).toBe(true);
    } finally {
      vi.useRealTimers();
    }
  });

  it("rejects out-of-range or fractional per-joint settings before any request", async () => {
    const request = vi.fn<typeof fetch>();
    const api = createSimpleArmApi(request, "l".repeat(40));
    const valid = {
      joint_2: { speed: 400, accel: 10, maxDeltaDegrees: 30 },
      joint_3: { speed: 400, accel: 10, maxDeltaDegrees: 30 },
    };
    const invalid = [
      { ...valid, joint_2: { ...valid.joint_2, speed: 0 } },
      { ...valid, joint_2: { ...valid.joint_2, speed: 2400.5 } },
      { ...valid, joint_3: { ...valid.joint_3, accel: 51 } },
      { ...valid, joint_3: { ...valid.joint_3, maxDeltaDegrees: 91 } },
    ];

    for (const settings of invalid) {
      await expect(api.startLiveFollow(settings, {
        startAttemptId: "frontend_attempt_invalid",
        startTimeoutMs: 1_500,
      })).rejects.toThrow(/integer speed 1 to 2400/);
    }
    expect(request).not.toHaveBeenCalled();
  });

  it("rejects incompatible v1 live-follow receipts", async () => {
    const startAttemptId = "frontend_attempt_0003";
    const request = vi.fn<typeof fetch>(async () => response(liveReceipt({ schema: "arm-live-follow-v1", startAttemptId })));
    await expect(createSimpleArmApi(request, "l".repeat(40)).startLiveFollow({
      joint_2: { speed: 400, accel: 10, maxDeltaDegrees: 30 },
      joint_3: { speed: 400, accel: 10, maxDeltaDegrees: 30 },
    }, { startAttemptId, startTimeoutMs: 1_500 })).rejects.toThrow("invalid live-follow receipt");
  });

  it("rejects malformed live-follow receipts instead of treating dispatch as measured truth", async () => {
    const startAttemptId = "frontend_attempt_0004";
    const request = vi.fn<typeof fetch>(async () => response(liveReceipt({
      startAttemptId,
      measured: { joint_2: { degrees: 0 }, joint_3: { degrees: 0 } },
    })));

    await expect(createSimpleArmApi(request, "l".repeat(40)).startLiveFollow({
      joint_2: { speed: 400, accel: 10, maxDeltaDegrees: 30 },
      joint_3: { speed: 400, accel: 10, maxDeltaDegrees: 30 },
    }, { startAttemptId, startTimeoutMs: 1_500 }))
      .rejects.toThrow("invalid live-follow receipt");
  });

  it("previews a bounded waypoint sequence once and executes only its exact digest", async () => {
    const actionToken = "q".repeat(40);
    const request = vi.fn<typeof fetch>(async (input) => {
      const url = String(input);
      if (url.endsWith("/sequences/preview")) return response({
        sequenceId: "armseq_frontend1",
        sequenceDigest: `sha256:${"c".repeat(64)}`,
        expiresAt: "2099-01-01T00:00:00Z",
        expiresInMs: 30_000,
        waypointCount: 2,
        measuredPose: { joint_1: 0, joint_2: 0, joint_3: 0, joint_4: 0 },
        waypoints: [
          { index: 0, startPose: { joint_1: 0 }, resolvedPose: { joint_1: 20 }, warnings: [], lowestClearanceMm: 30 },
          { index: 1, startPose: { joint_1: 20 }, resolvedPose: { joint_1: -10 }, warnings: [], lowestClearanceMm: 18 },
        ],
        warnings: [],
        lowestClearanceMm: 18,
      });
      return response({
        executed: true,
        sequenceId: "armseq_frontend1",
        operationId: "armop_frontend1",
        outcome: "completed",
        waypointCount: 2,
        completedWaypointCount: 2,
        failedWaypointIndex: null,
        waypointResults: [
          { index: 0, outcome: "arrived", dispatched: true, arrival: { proved: true }, moved: ["joint_1"] },
          { index: 1, outcome: "arrived", dispatched: true, arrival: { proved: true }, moved: ["joint_1"] },
        ],
        resolvedPose: { joint_1: -10 },
      });
    });
    const api = createSimpleArmApi(request, actionToken);
    const waypoints = [
      { targets: { joint_1: 20, joint_4: 30 } },
      { targets: { joint_1: -10, joint_4: 15 } },
    ];

    const plan = await api.previewSequence(waypoints);
    const result = await api.executeSequence(plan, 9_000);

    expect(result.completedWaypointCount).toBe(2);
    expect(request.mock.calls[0]?.[0]).toBe("/api/arm/simple/sequences/preview");
    expect(JSON.parse(String(request.mock.calls[0]?.[1]?.body))).toEqual({ waypoints });
    expect(request.mock.calls[1]?.[0]).toBe("/api/arm/simple/sequences/execute");
    expect(JSON.parse(String(request.mock.calls[1]?.[1]?.body))).toEqual({
      sequenceId: "armseq_frontend1",
      sequenceDigest: `sha256:${"c".repeat(64)}`,
      arrivalTimeoutMs: 9_000,
    });
  });

  it("rejects an incomplete sequence preview instead of enabling Apply all", async () => {
    const request = vi.fn<typeof fetch>(async () => response({
      sequenceId: "armseq_frontend1",
      waypoints: [],
    }));

    await expect(createSimpleArmApi(request, "q".repeat(40)).previewSequence([
      { targets: { joint_1: 10 } },
      { targets: { joint_1: -10 } },
    ])).rejects.toThrow("incomplete sequence");
  });

  it("rejects a mismatched or malformed terminal sequence receipt", async () => {
    const request = vi.fn<typeof fetch>(async () => response({
      executed: true,
      sequenceId: "armseq_someone_else",
      operationId: "armop_frontend1",
      outcome: "completed",
      waypointCount: 2,
      completedWaypointCount: 2,
      failedWaypointIndex: null,
      waypointResults: [{ index: 0, outcome: "arrived" }, { index: 1 }],
    }));

    await expect(createSimpleArmApi(request, "q".repeat(40)).executeSequence({
      sequenceId: "armseq_frontend1",
      sequenceDigest: `sha256:${"c".repeat(64)}`,
      waypointCount: 2,
    })).rejects.toThrow("invalid sequence execution receipt");
  });

  it("rejects a terminal sequence receipt that contradicts the reviewed path", async () => {
    const request = vi.fn<typeof fetch>(async () => response({
      executed: false,
      sequenceId: "armseq_frontend1",
      operationId: "armop_frontend1",
      outcome: "completed",
      waypointCount: 2,
      completedWaypointCount: 2,
      failedWaypointIndex: null,
      waypointResults: [],
    }));

    await expect(createSimpleArmApi(request, "q".repeat(40)).executeSequence({
      sequenceId: "armseq_frontend1",
      sequenceDigest: `sha256:${"c".repeat(64)}`,
      waypointCount: 2,
    })).rejects.toThrow("invalid sequence execution receipt");
  });

  it("turns null or undispatched arrival rows into a bounded receipt error", async () => {
    for (const waypointResults of [
      [null, null],
      [
        { index: 0, outcome: "arrived", dispatched: false, arrival: { proved: true } },
        { index: 1, outcome: "arrived", dispatched: false, arrival: { proved: true } },
      ],
    ]) {
      const request = vi.fn<typeof fetch>(async () => response({
        executed: true,
        sequenceId: "armseq_frontend1",
        operationId: "armop_frontend1",
        outcome: "completed",
        waypointCount: 2,
        completedWaypointCount: 2,
        failedWaypointIndex: null,
        waypointResults,
      }));

      await expect(createSimpleArmApi(request, "q".repeat(40)).executeSequence({
        sequenceId: "armseq_frontend1",
        sequenceDigest: `sha256:${"c".repeat(64)}`,
        waypointCount: 2,
      })).rejects.toThrow("invalid sequence execution receipt");
    }
  });

  it("sends one exact desired servo hold set through the fixed physical route", async () => {
    const actionToken = "h".repeat(40);
    const request = vi.fn<typeof fetch>(async () => response({
      servoIds: [2, 3],
      leaseMs: 1500,
      confirmed: true,
    }));

    await createPhysicalArmApi(request, actionToken).setTorqueHoldSet(
      [2, 3],
      1500,
      true,
      "ST3215",
    );

    expect(request).toHaveBeenCalledWith(
      "/api/arm/physical/servos/hold-set",
      expect.objectContaining({
        method: "POST",
        headers: expect.objectContaining({ "X-Co-Arm-Token": actionToken }),
        body: JSON.stringify({
          servoIds: [2, 3],
          leaseMs: 1500,
          acknowledgedPhysicalPowerCut: true,
          confirmedServoModel: "ST3215",
        }),
      }),
    );
  });

  it("preserves motor turns per joint turn in the physical profile payload", async () => {
    const request = vi.fn<typeof fetch>(async () => response({ committed: true }));
    const profile: PhysicalCalibrationProfilePayload = {
      expectedControllerBootId: "boot_abc",
      confirmedServoModel: "ST3215",
      joints: {
        joint_1: {
          logicalId: "joint_1",
          name: "base_yaw",
          servoId: 1,
          rawZero: 2048,
          direction: 1,
          motorTurnsPerJointTurn: 4,
        multiTurn: false,
        declaredLimits: false,
          negativeLimitTicks: -500,
          positiveLimitTicks: 500,
          limitMarginTicks: 40,
          maxVelocityDegreesPerSecond: 20,
          maxAccelerationDegreesPerSecond2: 40,
          evidenceIds: ["obs_base_1"],
        },
        joint_2: {
          logicalId: "joint_2",
          name: "shoulder_pitch",
          servoId: 2,
          rawZero: 2048,
          direction: 1,
          motorTurnsPerJointTurn: 1,
        multiTurn: false,
        declaredLimits: false,
          negativeLimitTicks: -500,
          positiveLimitTicks: 500,
          limitMarginTicks: 40,
          maxVelocityDegreesPerSecond: 20,
          maxAccelerationDegreesPerSecond2: 40,
          evidenceIds: ["obs_shoulder_2"],
        },
        joint_3: {
          logicalId: "joint_3",
          name: "elbow_pitch",
          servoId: 3,
          rawZero: 2048,
          direction: 1,
          motorTurnsPerJointTurn: 1,
        multiTurn: false,
        declaredLimits: false,
          negativeLimitTicks: -500,
          positiveLimitTicks: 500,
          limitMarginTicks: 40,
          maxVelocityDegreesPerSecond: 20,
          maxAccelerationDegreesPerSecond2: 40,
          evidenceIds: ["obs_elbow_3"],
        },
      },
      geometry: {
        baseHeightMm: 82,
        upperArmMm: 165,
        forearmMm: 155,
        toolOffsetMm: 35,
      },
      acknowledgedRemainDisarmed: true,
    };

    await createPhysicalArmApi(request, "p".repeat(40)).commitCalibrationProfile(profile);

    const sent = JSON.parse(String(request.mock.calls[0]?.[1]?.body));
    expect(sent.joints.joint_1.motorTurnsPerJointTurn).toBe(4);
    expect(sent).toEqual(profile);
  });
});
