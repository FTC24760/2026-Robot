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
- PASSIVE partner, or a partner that only parks (SOLO): shoot the pre-loads at once, then keep cycling for as long as time allows (`DEFAULT_CYCLES` = 3 is a maximum; each cycle needs `CYCLE_MIN_TIME_LEFT_S`). After each burst the robot waits at least `TIP_SETTLE_MIN_S`, then until the camera has read one of our CELLS (capped at `TIP_SETTLE_S`), so the up CELL is known; on arriving at a shot spot it waits up to `ARRIVE_CHECK_S` for a camera read taken since the trip began and, if the up CELL changed, drives on to the other CELL's spot before spinning up; the next trip goes back to the same spot if the HIVE did not tip, or to the other CELL's spot if it did. No idle wait before parking (`PARK_DELAY_S` = 0); the reserve window banks PARK. Without a camera the tip is not seen, so every cycle shoots at the CELL believed up (`BALLS_TO_TIP` = 0: no guess).
- ACTIVE, FIRST: delay 0; shoot the pre-loads, then follow the agreed no-tip plan (below).
- ACTIVE, SECOND: delay = menu value (initial guess 5 s; to be measured against the partner's burst).
- Implemented (`END_DELAY_ON_TIP`): the SECOND wait ends as soon as the up CELL changes (the camera saw the HIVE tip). The fixed delay stays as the fallback. After the wait, if the start pose is not in the shooting zone for the CELL up now (`DRIVE_TO_ZONE_FIRST`; e.g. the HIVE tipped, or start B with the audience CELL up), the robot drives to the shot spot before the pre-loads.
- Shooting position (implemented, `auto/ShotSpots`): no separate polygon. The zone is the TeleOp one (`ShotZoneGeometry`). SOLO/FIRST shoot from the fixed spot (`SHOOT_*`); SECOND picks the allowed candidate farthest from it (the fixed spot turned about the CELL in `SPOT_ARC_STEP_DEG` steps, same distance so same table row; must be in the zone, on our side with margin, inside the field, `SPOT_KEEP_OUT_IN` from FLOWERS). Hand-computed with the current constants: SECOND shoots at about (25, 21) into the RIGHT CELL and (30, 126) into the LEFT CELL.
- FIRST leaves early (`FIRST_PARK_DELAY_S`, default 0): parks right after its last shot instead of waiting `PARK_DELAY_S` (0 for SOLO and SECOND), so the partner has the shooting area.

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

## AUTO state machine (draft, up to three tips)

State: `up` (CellSide; starts RIGHT, the camera sets it), `tips` (observed tips since the start, `AutoContext.tips`), `cycles` done (max `DEFAULT_CYCLES` = 3), `timeLeft`. A tip is only ever *observed* (the camera sees `up` change, or a burst is interrupted); each observation does `tips++` and `up = other`. `BALLS_TO_TIP` = 0, so no tip is guessed. With no camera read, `up` and `tips` do not change.

| State | Does | Leaves to |
|---|---|---|
| WAIT (SECOND only) | stay still | GOTO on a tip (`tips`=1), delay end, or (STAY plan) `SECOND_GO_ANYWAY_LEFT_S` |
| GOTO | skip the drive if the pose is already in the zone for `up`, else drive to `ShotSpots.spot(up, slot)` | ARRIVE |
| ARRIVE | wait <= `ARRIVE_CHECK_S` for a camera read since the trip began | GOTO if `up` changed (`tips`++), else SHOOT |
| SHOOT | spin up, burst; a tip mid-burst (`tips`++) finishes the burst once more at the other CELL | SETTLE |
| SETTLE | wait >= `TIP_SETTLE_MIN_S`, then for a camera read, <= `TIP_SETTLE_S` | DECIDE |
| DECIDE | first rule that applies wins (below) | COLLECT or PARK |
| COLLECT | drive to the cycle's source with the intake on | GOTO |
| PARK | LEAVE/PARK pose | end |

`timeLeft` <= `RESERVE_TIME_LEFT_S` preempts any state and goes to PARK.

DECIDE: (1) `timeLeft` < `CYCLE_MIN_TIME_LEFT_S` or `cycles` = 3 -> PARK; (2) FIRST with LEAVE -> PARK; (3) FIRST with STAY -> PARK once `tips` >= 1, else COLLECT; (4) otherwise COLLECT. `up` never changes the decision, only where GOTO goes.

Ideal plan (every 4-ball burst scores and tips the HIVE; only POLLEN, NECTAR is ignored): the auto has 20 POLLEN, 4 pre-loaded on each robot and one 4-ball cycle at each of the GARDEN and the two FLOWERS.

| Slot | Collects (`*_SOURCES`) | Bursts |
|---|---|---|
| SOLO | GARDEN, back FLOWER | 3: pre-load, GARDEN at the other CELL, back FLOWER at the first CELL again |
| FIRST (start A) | GARDEN, wall FLOWER | 3: pre-load, GARDEN, wall FLOWER |
| SECOND (start B) | back FLOWER | 2: pre-load, back FLOWER |

Pair handshake (FIRST or SECOND): each robot shoots only into the CELL nearest its start (A: audience CELL, B: far CELL; `AutoSelection.ownSide`, `AutoContext.shotSide`) from the straight-on spot, and only while that CELL is up. Each burst tips the HIVE, which frees the partner. Turns alternate F S F S F, so the up CELL before each burst is RIGHT, LEFT, RIGHT, LEFT, RIGHT, and neither robot drives to the other side. A robot whose CELL is down waits (`awaitShotCell`, at most `OWN_CELL_WAIT_S`, then `ownCellGivenUp`: shoot at whichever CELL is up, with the old SECOND offset spot). A tip during our own burst ends the burst. FIRST no longer parks after its pre-loads; `noTipPlan` now only controls whether SECOND waits for a tip or for its delay. There is no ball sensor, so an emptied source is not detected; the cycle still costs its time.

Not-so-ideal, FIRST (the same rule, shown on a miss): the pre-loads do not tip, so FIRST's CELL is still up and it shoots the GARDEN balls at it with no wait. If that burst tips, the partner shoots its pre-loads, collects the back FLOWER and shoots again. If that tips, FIRST's CELL is up again and FIRST shoots the wall-FLOWER balls. If it does not, FIRST stops waiting after `OWN_CELL_WAIT_S`, or at `GIVE_UP_AT_TIME_LEFT_S` left (the latest time to drive over, shoot and still park), drives to the other CELL's offset spot (not the partner's straight-on spot), shoots, then parks (reserve window).

Time budget (30 s): the full ideal plan probably does not fit. With rough placeholder times (burst 4.5 s, GARDEN cycle 4 s, wall FLOWER 8.5 s, back FLOWER 7 s, cross-field trip 4 s, PARK trip 4 s; assumed ~35 in/s, not measured), FIRST is pre-load 4.5 + GARDEN 4 + burst 4.5 = 13 s, then the wall-FLOWER cycle (8.5 + 4.5 + park 4 + margin 1.5 = 18.5 s) does not fit in the 17 s left, so FIRST parks after two bursts. SOLO: pre-load, GARDEN, cross, burst = about 17 s, and the back FLOWER does not fit. The cross trip is charged only when the next shot is at a different spot than the one we are at (`AutoContext.spotSide()` vs `shotSide()`), so a burst that did not tip does not pay for it. Rule: a cycle starts only if its cost + burst + park trip + margin fits in `timeLeft` (`FullAuto.cycleFits`; `*_CYCLE_S`, `BURST_S`, `PARK_TRIP_S`, `CYCLE_MARGIN_S`, `CROSS_TRIP_S`). PARK is always banked: the reserve window (`RESERVE_TIME_LEFT_S`) abandons whatever is running and drives to PARK. Every step change is logged as `AutoStep` with the time left; replace the placeholders with those times.

By tips (side after each observed tip; the table assumes every tip is seen):

| tips | up | SOLO | FIRST, LEAVE | FIRST, STAY | SECOND |
|---|---|---|---|---|---|
| 0 | RIGHT | shoot pre-loads, cycle | pre-loads, park | pre-loads, cycle | wait; if no tip by the delay, go to RIGHT's offset spot on the partner's side |
| 1 | LEFT | next trip to LEFT's spot, cycle | park | park (tip seen) | partner tipped: shoot LEFT at once from our spot, cycle |
| 2 | RIGHT | next trip to RIGHT's spot, cycle | park | park | cycle, RIGHT's spot |
| 3 | LEFT | next trip to LEFT's spot, cycle | park | park | cycle, LEFT's spot |

Fixed: `tippedSinceStart()` used to compare the current side with the starting side, so after two tips it read false again and FIRST/STAY would resume cycling. It is now `tips > 0` (and the SECOND wait uses `tips > tipsAtStart`). The table stops at 3 tips; the counter itself does not.

Known gaps (fixed camera): a tip while the robot is collecting or driving is only seen at ARRIVE or SETTLE, and a tip while blind is never seen, so a burst can go at the down CELL. The mid-burst check is the only recovery.

## Camera limitation (fixed, decided)

There is one Limelight 3A, mounted on the chassis, with no pan servo (a pan servo was built and then removed). The camera sees the HIVE only while the chassis faces it, so:
- tips are read at the shot spot (after a burst, on arrival), not while collecting or driving;
- the odometry-based plan is the fallback whenever the camera has no view (timeouts in `Steps.awaitCamera` end at once without a sighting);
- redirecting mid-drive on a tip seen en route is not possible and is out of scope.

## TeleOp ready-to-fire and the up/down state

Ready = in the shot zone (distance ring and approach angle) AND loaded (intake ran `REARM_INTAKE_S` since the last shot, or the robot left and re-entered the zone) AND the up/down state is confirmed. The driver presses shoot; there is no auto-fire.

The turret and hood always aim at the believed-up CELL from odometry (`HiveTracker` remembers the field position). The camera is fixed, so it only reads the HIVE while the chassis faces it; what odometry cannot know is whether the HIVE has tipped. `ShootGateConstants.UPSIDE_CHECK` therefore requires the last camera read of our CELLS (up or down) to be at most `UPSIDE_MAX_AGE_S` old and to be later than the last shot that fed balls. A normal cycle passes or drives under the HIVE before reaching the zone, so the read is fresh. A down-read of one CELL switches the aim to the other (known field point from `AutoConstants`). If no recent read: amber LED, "UP CELL UNCONFIRMED", no rumble; the shoot button still works and D-pad right (driver or operator) flips the aimed CELL, which counts as a read until the camera disagrees. In TeleOp only (`Robot.enableTeleOpMemory`) the remembered CELL position never expires (`TELEOP_ODOMETRY_FALLBACK_MAX_S` = 0), so the turret keeps aiming from odometry and only the up/down state ages; a down-read also switches the aim to the other CELL. AUTO keeps its own odometry fallback (`ODOMETRY_FALLBACK_MAX_S` 20 s, `ctx.upSide`). TeleOp starts aimed at the CELL AUTO believed was up (`PoseHandoff`), unconfirmed until the first camera read. The age of the read is logged (`ShootZone` tag) each time the ready signal fires, to set `UPSIDE_MAX_AGE_S`.

### SECOND (partner shoots first), after the delay

Same loop as SOLO. The wait ends at the delay or when the camera sees the up CELL change. If the partner tipped the HIVE, the up CELL is now the one on our side: shoot it at once from our spot (pre-loads), then collect and shoot as the camera reports. If it did not tip, drive to the up CELL's offset spot on the partner's side (`ShotSpots`) and shoot; if our shots tip it, the next trip goes to the other CELL's spot on our side; if not, keep collecting and shooting at the same CELL. No idle wait before parking (`PARK_DELAY_S` = 0): cycles stop at `CYCLE_MIN_TIME_LEFT_S` and the reserve window banks PARK.

### FIRST, and the no-tip plan (agreed with the partner)

If FIRST's pre-loads do not tip the HIVE, the two robots must already agree what happens, since they cannot talk. One init-menu flag (left stick press, `AutoSelection.noTipPlan`, default `DEFAULT_NO_TIP_PLAN` = LEAVE) is set the same on both robots:

- **LEAVE:** FIRST shoots the pre-loads and parks (no cycles, no tip-settle wait). SECOND waits its delay (ending early on a tip), then drives to the up CELL's offset spot and shoots.
- **STAY:** FIRST cycles until the camera sees the HIVE tip (`tippedSinceStart`), then parks, leaving the area to SECOND. SECOND ignores the delay and waits for the tip, giving up and going anyway at `SECOND_GO_ANYWAY_LEFT_S` (15 s) left.

If the pre-loads do tip it, FIRST parks in both plans (mid-burst tips still finish the burst at the other CELL). SOLO ignores the flag. Not covered: the partner picking the other plan; the menu only shows it, it cannot check.

### Camera-confirmed tip checks (SOLO, also used by FIRST/SECOND)

`Steps.awaitCamera` waits for a camera read of one of our CELLS (`ctx.lastCellReadS`, set in `AutoOpMode.trackUpSide`) taken after a given time, capped, and ends at once without a camera. A CELL read facing down also sets the other one up (the two are joined). Used twice: after a burst (min `TIP_SETTLE_MIN_S` 0.4 s so the HIVE has time to tip, cap `TIP_SETTLE_S` 1.5 s) and on arrival at a shot spot (`ARRIVE_CHECK_S` 0.5 s; if `upSide` differs from the side the trip was planned for, `driveToShot` drives to the other spot before spin-up). Not possible with the fixed camera: redirecting mid-drive when a tip is seen en route; the extra leg is paid only after arriving.
