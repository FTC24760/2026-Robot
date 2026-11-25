# BIOBUZZ AUTO + shared-logic design spec (draft 1)

Status: design only, no code changed. Written for the team's programmers.
Base branch: `codex/competition-cleanup` (ae36ccc), which is the latest code; all new work goes on top of it. Sources: that branch; BIOBUZZ manual TU10 (Figures 9-2, 10-2, 11-1, G304).
Figure positions are measured from pixels, about ±2 in.

## 1. Goals

1. One AUTO OpMode that works for either alliance, either start spot, and a partner that is passive, FIRST, or SECOND.
2. Auto and TeleOp share the same subsystem and controller code (no duplicated aiming or shooting logic).
3. Always bank LEAVE and PARK. Shots are worth more, but they never put LEAVE/PARK at risk.

Non-goals (for now): FLOWER collection (no mechanism), NECTAR handling (robot cannot intake it), Limelight blob pipeline for POLLEN.

## 2. Field and frame

- Frame = repo frame: origin at the lower-left corner, inches, +X toward blue, +Y away from the audience, heading 0 = +X, CCW positive. Bottom edge = audience wall (confirmed by the team). Red alliance area is on the left.
- All routines are written for RED. Blue = red rotated half a turn about the field centre:
  `x' = 144 - x`, `y' = 144 - y`, `heading' = heading + 180`.
- Verified against the manual figures (red frame):

| Item | Position |
|---|---|
| Red GARDEN | bottom-left corner on the audience wall, x about 0-23 |
| Red LOADING ZONE | red wall, x about 0-11, y about 96-119 |
| FLOWERS | (0,48), (48,144), (95.5,0), (144,95.5); each about 6 in. wide |
| Red HIVE cells | x about 59; y about 61 (RIGHT, near audience) and y about 88 (LEFT) |
| Red CELL up at start | RIGHT (y about 61) |

- Starting rules (G304): fully inside own columns (red x <= 72), touching the perimeter wall, not touching a FLOWER scoring volume, not in the LOADING ZONE, touching exactly 4 pre-load POLLEN, motionless after INIT.
- Both robots start at 0:00 and cannot talk to each other. "FIRST/SECOND" is agreed before the match.
- Both robots shoot into the same alliance HIVE. A partner's shots can tip it, so the up-CELL can change at any moment.

## 3. Start presets (red frame, robot 17 in. wide x 18 in. long)

`StartPreset` enum, defined once in red; blue derived with the half-turn above.

| Preset | Pose (x, y, deg) | Footprint | Notes |
|---|---|---|---|
| A | (59, 8.5, 180) | x 50-68, y 0-17 | audience wall; clear of GARDEN and LOADING ZONE; current start |
| B | (61.5, 135.5, 180) | x 52.5-70.5, y 127-144 | far wall; FLOWER at x 45-51 needs left edge >= 51; column C needs right edge <= 72, so centre x must be 60-63 |

Blue: A -> (85, 135.5, 0); B -> (82.5, 8.5, 0).
B has only about 1.5 in. of slack each side. Use a jig or tape mark. Existing `SHOOT_LEFT_UP_X = 59` overlaps the FLOWER by about 1 in.; change it to 61.5.

```java
public enum StartPreset {
    A(59.0, 8.5, 180.0),
    B(61.5, 135.5, 180.0);
    public final double x, y, headingDeg;
    StartPreset(double x, double y, double headingDeg) {...}
    public Pose pose(Alliance a) {
        return a == Alliance.RED
            ? new Pose(x, y, Math.toRadians(headingDeg))
            : new Pose(144 - x, 144 - y, Math.toRadians(headingDeg + 180));
    }
}
```

## 4. Init menu (gamepad 1, in `init_loop`) — implemented

Alliance stays the OpMode (`Auto: Red` / `Auto: Blue`): the Limelight and `preselectTeleOp` are fixed at INIT, so a menu choice would be too late for both. Partner mode and FIRST/SECOND are merged into one "slot". Existing bindings (cycles, up-CELL) are kept so the drive team does not relearn them.

| Input | Control | Default |
|---|---|---|
| Start preset | D-pad left/right (A / B) | A |
| Slot | A cycles SOLO / FIRST / SECOND | SOLO |
| Start delay | bumpers, 0.5 s steps, 0-15 s; used only for SECOND | 5 s |
| Cycles | D-pad up/down, 0-3 (existing) | `DEFAULT_CYCLES` (0) |
| Up-CELL override | X = LEFT, B = RIGHT, Y = automatic (existing) | automatic |

Telemetry shows the selection, the exact coordinates the robot should be placed at for this alliance, and warnings: SECOND with zero delay; a delay so long that PARK may be skipped. Selections live in `AutoSelection` statics, so they persist between matches until the app restarts or code is deployed. Not done: Limelight wrong-alliance and pose-vs-preset warnings (`USE_AUTO_AIM` is currently false on the base branch).

## 5. Plan builder

`Plan` = ordered `AutoStep`s built from the menu (reuse the existing `Steps` framework). Robot holds 4 POLLEN (capacity 4), so there is no collecting before the first burst.

Timeline (30 s):

```
0:00  [wait delay]            (SECOND only; may move into the shooting zone, no intake)
      spin up; drive into shooting zone; shoot 4 (pre-loads)
      repeat up to `cycles` while time allows:
          intake on; drive to GARDEN (red: audience wall, x ~9); take 4 POLLEN
          drive into shooting zone; shoot 4
reserve window (last ~4 s): drive to LEAVE+PARK, regardless of state
```

- PARK pose: partly in the LOADING ZONE (red: x 0-11, y 96-119), current `(12, 108, 180)`. LEAVE = no longer touching the perimeter wall, so driving toward PARK satisfies it.
- Skip rule: a cycle starts only if `timeLeft >= cycleEstimate + parkWindow`. Cycle estimate is measured, not guessed; start with the current `CYCLE_MIN_TIME_LEFT_S = 9`.
- PASSIVE partner, or a partner that only parks (SOLO): shoot the pre-loads at once, then keep cycling for as long as time allows (`DEFAULT_CYCLES` = 3 is a maximum; each cycle needs `CYCLE_MIN_TIME_LEFT_S`). After each burst the robot waits `TIP_SETTLE_S` (camera only) so the up CELL is known; the next trip goes back to the same spot if the HIVE did not tip, or to the other CELL's spot if it did. No idle wait before parking (`SOLO_PARK_DELAY_S` = 0); the reserve window banks PARK. Without a camera the tip is not seen, so every cycle shoots at the CELL believed up (`BALLS_TO_TIP` = 0: no guess).
- ACTIVE, FIRST: delay 0; leave the shooting zone early so the partner can use it.
- ACTIVE, SECOND: delay = menu value (initial guess 5 s; to be measured against the partner's burst).
- Implemented (`END_DELAY_ON_TIP`): the SECOND wait ends as soon as the up CELL changes (the camera saw the HIVE tip). The fixed delay stays as the fallback. After the wait, if the start pose is not in the shooting zone for the CELL up now (`DRIVE_TO_ZONE_FIRST`; e.g. the HIVE tipped, or start B with the audience CELL up), the robot drives to the shot spot before the pre-loads.
- Shooting position (implemented, `auto/ShotSpots`): no separate polygon. The zone is the TeleOp one (`ShotZoneGeometry`). SOLO/FIRST shoot from the fixed spot (`SHOOT_*`); SECOND picks the allowed candidate farthest from it (the fixed spot turned about the CELL in `SPOT_ARC_STEP_DEG` steps, same distance so same table row; must be in the zone, on our side with margin, inside the field, `SPOT_KEEP_OUT_IN` from FLOWERS). Hand-computed with the current constants: SECOND shoots at about (25, 21) into the RIGHT CELL and (30, 126) into the LEFT CELL.
- FIRST leaves early (`FIRST_PARK_DELAY_S`, default 0): parks right after its last shot instead of waiting `PARK_DELAY_S`, so the partner has the shooting area.

## 6. Subsystems and controllers (shared by Auto and TeleOp)

| Class | Hardware / role |
|---|---|
| `Drive` | 4 mecanum motors, Pinpoint, Pedro `Follower` (keep the tuned Foresight config). Field-centric drive, `driveTo()`, `hold()`. |
| `Flywheel` | Encoder-equipped flywheel. Velocity control with voltage compensation, recovery boost, `isReadyToFeed()` (move existing `Shooter` logic here, unchanged at first). |
| `Aim` | Turret (2 servos, open-loop), hood servo, slew limits, "settled" timer. |
| `Feed` | Intake, transfer, blocker servo, pulsing. The blocker servo is slow with no feedback: opening is modelled by `BLOCKER_OPEN_SETTLE_S` (the `OPENING_GATE` stage), closing by `BLOCKER_CLOSE_SETTLE_S` (intake/transfer held while closing). |
| `Vision` | Limelight AprilTag pipeline, HIVE fix, up-CELL detection. (`HiveTracker` stays.) |

Controllers (no hardware of their own):
- `AimController`: pose + vision fix -> turret bearing, hood, RPM, via `ShotTable` (implemented: measured distance -> RPM/hood rows in `ShotTableConstants`, linear interpolation, filled with the `Test: Shot Calibration` OpMode; replaces the physics `ShotSolver`). Used in TeleOp and Auto; Auto falls back to the odometry distance to the up CELL when the camera has no target, and both fall back to the fixed `Configurables` values when there is no distance at all. The operator's RPM buttons are a trim on top.
- `ShootController`: fire gate = in zone (hysteresis) AND valid reachable shot AND aim settled for N ms AND flywheel ready AND robot speed below limit AND up-CELL still the one we aimed at. Calls the existing shoot sequence. TeleOp (button / auto-fire toggle) and Auto (`shoot()` step) both go through it.
  - Implemented so far: one `ShootSequence` for TeleOp and Auto (`AutoShootSequence` merged in; Auto passes a feed limit). A ball is fed only when the wheel is ready AND `Robot.aimReady()`: turret settled on target for `AIM_SETTLE_S` while auto-aiming, robot below `MAX_SPEED_IN_PER_S` / `MAX_TURN_DEG_PER_S` (`ShootGateConstants`). Feeding pauses and resumes by itself. The up-CELL check stays in Auto's `ShootStep`.
  - TeleOp shooting zone (decided with the team): no auto-fire. The zone is the area the shot table is calibrated for, i.e. target distance within the measured rows + margin, with hysteresis, AND the shot within `ZONE_MAX_OFF_AXIS_DEG` of the up CELL's opening axis (the CELL opens along field y: AUDIENCE cell toward the audience wall, FAR cell toward the far wall, tilted 30 deg up; manual figs 9-9/9-10; a shot perpendicular to it does not go in). After a shot that fed balls the zone is disarmed (no pre-spin, not ready) until the intake has run `REARM_INTAKE_S` or the robot leaves the zone; there is no ball sensor, so intake time stands in for "reloaded". In the zone the wheel pre-spins to the table speed; when in zone + aim ready + wheel at speed, the robot is "ready to fire": rumble on becoming ready (with cooldown), gamepad LED green/red, telemetry headline. The driver fires with the normal shoot button (still allowed outside the zone as an override). The stop button cancels pre-spin until the robot leaves the zone. Settings in `ShootGateConstants`.

Split so far: `sequences/ShootZone` (zone, ready-to-fire, reload arming, rumble/LED; no hardware) is out of `Robot.java`; `aim/AimController` (camera + pose -> turret bearing, wheel speed, hood; aim gate; target distance and approach angle; RPM trim) is out too. `Robot` keeps thin delegating methods so Auto and the test OpModes are unchanged.

`Robot.java` is split along these lines; `Steps`, `FullAuto`, `ShotSolver`, `HiveTracker`, and the Pedro config are reused. The base branch has two near-identical shoot sequences (`ShootSequence` for teleop, `AutoShootSequence` for auto, which adds a feed timeout); `ShootController` should replace both with one.

## 7. Safeguards

1. Reserve window forces LEAVE+PARK even if a step is stuck (global timer, not per step). Implemented: `AutoRoutine.withReserve()` + `RESERVE_TIME_LEFT_S` (6 s, placeholder). If that many seconds are left and the route has not started its own park drive (`ctx.parkStarted`), the running step is ended, shooter/intake/shoot sequence stop, and the robot drives to PARK (telemetry shows `RESERVE > ...`).
2. Every drive has a timeout (existing `DRIVE_TIMEOUT_S`) and a skip.
3. Check the up-CELL immediately before the first fed ball and re-check during feeding. A tip mid-burst stops feeding and retargets (existing `ShootStep` behaviour, extended to the pre-feed check).
4. If the Limelight is lost, aim from odometry to the remembered CELL (existing behaviour).
5. Paths stay on our half of the field (G402: do not disrupt the opposing alliance's AUTO).
6. Menu warnings for any selection that looks inconsistent.

## 8. Constants to tune on the field (all `@Configurable`)

Shooting-zone polygon; delay default; `CYCLE_*` order and count; park window length; `BALLS_TO_TIP` (unknown; the manual only says "enough"); `SHOOT_FEED_S`; aim-settle time; speed limit for firing; B preset x within 60-63.

- Own-side check (G402: red = columns A-C, x <= 72): `AutoContext.pose()` records any pose with red-frame x past `OWN_SIDE_MAX_X_IN - OWN_SIDE_MARGIN_IN`; `checkRouteConstants()` runs the fixed route poses through it at init, and the init screen shows `!! Route goes past our side`. All current poses pass (max x 61.5).

- Shoot on the move (`ShootGateConstants.SHOOT_ON_MOVE`, off by default, experimental): turret bearing, shot distance and the zone's distance test use a virtual target = CELL - robot velocity x `FLIGHT_TIME_S` (shift capped at `MAX_LEAD_IN`), and the fire gate uses the `MOVING_MAX_*` limits. Constant velocity only. Tune after the stationary calibration: flight time first, then raise the speed limits.

- Shared zone test: `utils/ShotZoneGeometry` (distance ring + approach angle, with hysteresis) is the single definition used by TeleOp (`ShootZone`) and by the Auto init screen, which warns `!! Outside the shooting zone` for the start pose (pre-loads into the CELL up at the start) and the two fixed shot spots. There is no separate Auto polygon; Auto's extra constraints (own side, field elements, partner) are applied on top.

## 9. Assumptions and open items

- Robot capacity 4 balls, no NECTAR intake (stated by the team).
- Limelight is chassis-mounted (the code supports both mounts; confirm).
- Transfer motor and blocker servo exist (as the current code assumes; confirm).
- `ShotSolver` and the shooter calibration have not been reviewed.
- Manual details not read in full: G403-G427 and section 11.4.1 (alliance priority).
- Figure 11-1 examples show legal starts anywhere along our half's walls; only A and B are planned.
- Unverified: first-burst timing for the partner; the SECOND delay of 5 s is a placeholder.

## 10. Implementation order

1. Extract subsystems from `Robot.java` with no behaviour change; confirm TeleOp is unchanged.
2. `StartPreset` + init menu + alliance-neutral OpMode; verify poses on a real field for A and B on both alliances.
3. Plan builder with delay, cycles, reserve window; test with partner PASSIVE.
4. `ShootController` fire gate; use it from both Auto and TeleOp.
5. Optional: early end of the delay from up-CELL detection; zone-based shooting position.

Test points: each preset on each alliance (pose on the field matches the telemetry), delay 0 and 5, with and without a Limelight fix, forced stuck-step to confirm LEAVE+PARK still happens.

## Camera pan servo (optional, off by default)

`CameraPanConstants.ENABLED` adds a servo (270 degrees, or geared) that turns the Limelight
about a vertical axis so it keeps facing the middle of our HIVE while the robot drives. The
target is the HIVE centre from `AutoConstants` CELL points (not field (0,0)), flipped for
blue, aimed from the odometry pose with a velocity lead (`CameraPan`, called from
`AimController.update`). Vision applies the pan angle (commanded angle `SERVO_LAG_S` before
frame capture) and swings the lens about `AXIS_*` when converting tags to the robot frame, so
`HiveTracker` and the up-CELL check are unchanged. Effect: the camera can read which CELL is
up while the robot collects or drives, so a tip is seen without returning to the shot spot.
Limits: the servo has no feedback (angle is the command), travel is 270 degrees so a HIVE
behind the robot can be out of reach (`OUT OF TRAVEL` in telemetry), and the pan servo name,
centre position, direction and axis position must be calibrated before enabling.
