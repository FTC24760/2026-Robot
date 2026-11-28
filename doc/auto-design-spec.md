# BIOBUZZ AUTO + shared-logic design spec (draft 1)

Status: implemented on branch `auto-redesign` (PR #10); see the summary below for current behavior. Written for the team's programmers.
Base branch: `codex/competition-cleanup` (ae36ccc), which is the latest code; all new work goes on top of it. Sources: that branch; BIOBUZZ manual TU10 (Figures 9-2, 10-2, 11-1, G304).
Figure positions are measured from pixels, about ±2 in.

## AUTO behavior summary (current, 2026-11-27)

This is what the code on `auto-redesign` does. The sections below hold the reasoning and history; where they disagree, this section is current. All numbers are placeholders until measured on the robot.

### Every shot, all slots

- **Getting into the zone:** if the robot is already in the TeleOp shooting zone (distance ring + off-axis angle, `ShotZoneGeometry`), it does not drive. Otherwise it drives to the nearest point `ZONE_INSET_IN` (3 in) inside the zone edge (`ZoneTargets`). Those points are on our side of the middle line, at least `WALL_TURN_CLEAR_IN` from every wall, clear of FLOWERS, and in a pair at least `PARTNER_CLEAR_IN` (24 in) from the partner's start. The robot faces the CELL there so the fixed camera sees the HIVE.
- **Firing:** the same "ready to fire" test as TeleOp (`ShootZone`): in the zone, aim and hood settled, wheel at speed, and a recent camera read of which CELL is up. Auto fires as soon as it passes, or after `READY_TIMEOUT_S` (1.5 s) anyway. The wheel pre-spins in the zone.
- **After a burst:** wait up to `TIP_SETTLE_S` (1.5 s) for the camera to show whether the HIVE tipped. With no read, go by our own ball count: the first tip needs 4 balls, each later tip 8.
- **Tip mid-burst:** feeding stops at once and the burst is finished at the other CELL.
- **Time budget:** a cycle starts only if collecting, the shot, the drive to park and a margin all fit in the time left.
- **End of auto:** with `RESERVE_TIME_LEFT_S` (6 s) left, whatever is running is dropped and the robot parks.
- **Turning near walls:** the robot never turns with its centre closer than `WALL_TURN_CLEAR_IN` (13.5 in) to a wall. It moves straight off the wall first, or finishes the turn before moving in.

### SOLO (start A)

1. Shoots the pre-loads from the start, in Zone A.
2. Before each later shot, collects at the nearest unused source, then shoots whichever CELL is up.
3. No tip: GARDEN, shoot in Zone A, wall FLOWER, shoot in Zone A, park.
4. Pre-loads tip: GARDEN, shoot in Zone B, back FLOWER, shoot in Zone B (that tips it), then the wall FLOWER and a shot in Zone A if it fits, otherwise park.

### FIRST (stronger robot, start A)

1. Shoots the pre-loads from the start in Zone A, with no delay.
2. GARDEN, then shoots whichever CELL is up: Zone A if the pre-loads missed, Zone B if they tipped.
3. Wall FLOWER, then shoots whichever CELL is up.
4. Parks. Never goes to the back FLOWER.

### SECOND (weaker robot, start B)

1. Waits its delay (5 s by default, bumpers). The wait ends early if the camera sees FIRST tip the HIVE.
2. If the camera has not read the HIVE recently at that point, it assumes FIRST tipped it.
3. Only ever shoots into its own CELL, Zone B, so it never goes near Zone A where FIRST shoots.
4. Shoots the pre-loads from the start, then collects at the back FLOWER, shoots in Zone B again, and parks.
5. Whenever its CELL is facing down at fire time, it holds fire in Zone B and waits for a tip. If none comes, the end-of-auto park takes over and it keeps its balls for TeleOp.

### EQUAL (both robots strong, goal three tips)

Each robot shoots only its own CELL (start A: Zone A, start B: Zone B), collects only its side's sources, and holds fire while its CELL is down. The start is picked by hand. The 20 balls then make exactly three tips:

| Tip | Needs | From |
|---|---|---|
| 1 (RIGHT CELL, Zone A) | 4 | A robot's pre-loads |
| 2 (LEFT CELL, Zone B) | 8 | B robot's pre-loads + back FLOWER |
| 3 (RIGHT CELL, Zone A) | 8 | A robot's GARDEN + wall FLOWER |

- **From A:** pre-loads in Zone A; GARDEN, then wait in Zone A for tip 2 and shoot; wall FLOWER, shoot in Zone A; park.
- **From B:** like SECOND: delay (ends early on a seen tip), pre-loads in Zone B, back FLOWER, shoot in Zone B, park.

Timing risk (placeholder speeds): the A robot waits about 10 s for tip 2 with the GARDEN load on board, so its wall FLOWER cycle starts at roughly 21 s and does not fit before the reserve window; tip 3 likely needs measured, faster cycles.

### Partner

The partner's auto is unknown; we only run our code on our robot. The only partner rule is static: stay `PARTNER_CLEAR_IN` from the partner's start. Agree routes with the partner team before each match.

## TeleOp behavior summary (current, 2026-11-27)

What `TeleOp: Red` / `TeleOp: Blue` (`MatchTeleOp`, `Robot.update`) do on `auto-redesign`. Numbers are placeholders until measured.

### Start

- **Pose:** carries on from where AUTO left the robot (`PoseHandoff`) if AUTO ran for the same alliance within `PoseHandoff.MAX_AGE_S` (60 s); otherwise starts from `TeleOpConstants`, and telemetry says to check it.
- **Up CELL:** aims at the CELL AUTO believed was up. It counts as unconfirmed until the camera reads the HIVE.

### Aiming

- The turret auto-aims at the up CELL: at the camera target when it sees one, otherwise from odometry at the remembered CELL position. In TeleOp that memory never expires (`TELEOP_ODOMETRY_FALLBACK_MAX_S` = 0); only the up/down state ages.
- Wheel speed and hood come from the shot table for the target distance, plus the RPM trim.
- A camera read of one CELL facing down switches the aim to the other CELL.
- The operator's right stick moves the turret by hand while it is pushed; A (either pad) switches auto-aim off and on.

### Shooting

- **No auto-fire.** The driver or operator holds the right trigger to shoot; releasing it stops the burst.
- **Fire gate (every ball):** wheel at speed, turret settled on the target (`AIM_SETTLE_S`), hood settled (`HOOD_SETTLE_S`), robot slower than `MAX_SPEED_IN_PER_S` and turning slower than `MAX_TURN_DEG_PER_S`.
- Feeding pulses by default (operator X toggles it).
- D-pad left (driver) or a reverse request cancels the shot.

### Shooting zone and the ready signal

- **Zone:** target distance inside the shot table's calibrated range (+ margin) and the shot within `ZONE_MAX_OFF_AXIS_DEG` of the up CELL's opening axis, with hysteresis. It is the same zone AUTO uses.
- **Pre-spin:** in the zone the wheel spins up to the table speed so a shot goes at once. D-pad left stops pre-spin until the robot leaves the zone.
- **Ready to fire:** in the zone, aim and hood settled, wheel at speed, loaded, and a recent camera read of which CELL is up (`UPSIDE_MAX_AGE_S`, and after the last shot).
  - Ready: gamepad LED green and a short rumble on both pads (`READY_RUMBLE_MS`, with a cooldown).
  - Everything ready except the up/down read: LED amber, "UP CELL UNCONFIRMED", no rumble. D-pad right (either pad) flips the aimed CELL, which counts as a read until the camera disagrees.
  - Not ready: LED red.
- **Loaded:** there is no ball sensor. After a shot that fed balls the zone is disarmed (no pre-spin, not ready) until the intake has run `REARM_INTAKE_S` or the robot leaves the zone.
- The shoot trigger works whether or not the robot is ready, inside the zone or out.

### Intake

Priority, highest first: reverse (left bumper, either pad) > pulse feeding while shooting > feeding > hold while a shot spins up > intake (driver left trigger, operator right bumper) > off. Y (either pad) raises and lowers the FLOWER arm.

### Controls

| Control | Driver (gamepad 1) | Operator (gamepad 2) |
|---|---|---|
| Drive (field-centric) | left stick move, right stick X turn | - |
| Shoot (hold) | right trigger | right trigger |
| Intake (hold) | left trigger | right bumper |
| Reverse intake (hold) | left bumper | left bumper |
| FLOWER arm toggle | Y | Y |
| Auto-aim on/off | A | A |
| Flip aimed CELL | D-pad right | D-pad right |
| RPM trim +/- 50 (max +/- 500) | D-pad up / down | D-pad up / down |
| Stop shooter and pre-spin | D-pad left | - |
| Pulse feeding on/off | - | X |
| Manual turret | - | right stick X |
| Reset pose | Create / Back | - |

## 1. Goals

1. One AUTO OpMode that works for either alliance, either start spot, and a partner that is passive, FIRST, or SECOND.
2. Auto and TeleOp share the same subsystem and controller code (no duplicated aiming or shooting logic).
3. Always bank LEAVE and PARK. Shots are worth more, but they never put LEAVE/PARK at risk.

Non-goals (for now): scoring into FLOWERS (the robot collects POLLEN from FLOWERS, but has no mechanism to put NECTAR or POLLEN into a FLOWER at end game), NECTAR handling (robot cannot intake it), Limelight blob pipeline for POLLEN.

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
| A | (59, 9, 90) | x 50.5-67.5, y 0-18 | audience wall, back to the wall, facing the AUDIENCE (RIGHT) CELL 52 in away; clear of GARDEN and LOADING ZONE |
| B | (61.5, 135, 270) | x 53-70, y 126-144 | far wall, back to the wall, facing the FAR (LEFT) CELL 47 in away; FLOWER at x 45-51 and column C (<= 72) allow centre x 59.5-63.5 |

Blue: A -> (85, 135, 270); B -> (82.5, 9, 90).
Both starts face the HIVE (decided 2026-11-27) so the fixed camera watches it from the start: FIRST sees whether its pre-loads tipped it, SECOND sees the partner's burst during its delay. Both are inside the shooting zone for the CELL they face.

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
| Slot | A cycles SOLO / FIRST / SECOND / EQUAL; also sets the start (SOLO and FIRST: A, SECOND: B, EQUAL: unchanged) | SOLO |
| Start delay | bumpers, 0.5 s steps, 0-15 s; used only for SECOND | 5 s |
| Cycles | D-pad up/down, 0-3 (existing) | `DEFAULT_CYCLES` (3) |
| Up-CELL override | X = LEFT, B = RIGHT, Y = automatic (existing) | automatic |

Telemetry shows the selection, the exact coordinates the robot should be placed at for this alliance, and warnings: SECOND with zero delay; a delay so long that PARK may be skipped; a start that does not match the slot; a start outside the zone; a CELL with no usable zone point. Selections live in `AutoSelection` statics, so they persist between matches until the app restarts or code is deployed. Not done: Limelight wrong-alliance and pose-vs-preset warnings (`USE_AUTO_AIM` is currently false on the base branch).

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
- PASSIVE partner, or a partner that only parks (SOLO): shoot the pre-loads at once, then keep cycling for as long as time allows (`DEFAULT_CYCLES` = 3 is a maximum; each cycle needs `CYCLE_MIN_TIME_LEFT_S`). After each burst the robot waits at least `TIP_SETTLE_MIN_S`, then until the camera has read one of our CELLS (capped at `TIP_SETTLE_S`), so the up CELL is known; on arriving at a shot spot it waits up to `ARRIVE_CHECK_S` for a camera read taken since the trip began and, if the up CELL changed, drives on to the other CELL's spot before spinning up; the next trip goes back to the same spot if the HIVE did not tip, or to the other CELL's spot if it did. No idle wait before parking (`PARK_DELAY_S` = 0); the reserve window banks PARK. Without a camera read the tip is not seen; the ball count is then believed (4 for the first tip, 8 after; see below).
- ACTIVE, FIRST: delay 0; shoot the pre-loads, then follow the agreed no-tip plan (below).
- ACTIVE, SECOND: delay = menu value (initial guess 5 s; to be measured against the partner's burst).
- Implemented (`END_DELAY_ON_TIP`): the SECOND wait ends as soon as the up CELL changes (the camera saw the HIVE tip). The fixed delay stays as the fallback. After the wait, if the start pose is not in the shooting zone for the CELL up now (`DRIVE_TO_ZONE_FIRST`; e.g. the HIVE tipped, or start B with the audience CELL up), the robot drives to the shot spot before the pre-loads.
- Shooting position (implemented, `auto/ZoneTargets`, decided 2026-11-27): no fixed spot. The zone is the TeleOp one (`ShotZoneGeometry`, ring + angle; numbers still to be measured). Already in the zone: no drive. Otherwise the robot drives to the nearest point `ZONE_INSET_IN` (3 in) inside the zone edge and faces the up CELL there (camera on the HIVE). The only extra auto limits: not past the middle line (G402, `OWN_SIDE_*`), `WALL_TURN_CLEAR_IN` from every wall so it can turn, `SPOT_KEEP_OUT_IN` from FLOWERS, and in a pair, for the partner's CELL, `PARTNER_CLEAR_IN` from the partner's start. With the placeholder constants (one table row at 52.5 in, 60 deg): from the GARDEN to the RIGHT CELL about (21.5, 21.5), from the wall FLOWER (17.5, 33.5), from the back FLOWER to the LEFT CELL (31.5, 129.5).
- Firing (decided 2026-11-27): the same "ready to fire" decision as TeleOp (`ShootZone`: in zone + aim/hood settled + wheel at speed + recent up/down read + loaded), run every auto loop with the driver feedback off (`Robot.updateAutoZone`). The wheel pre-spins in the zone as in TeleOp. The difference: auto fires by itself the first loop it is ready (`Steps.toZoneAndReady`), or after `READY_TIMEOUT_S` anyway. If the camera sees the up CELL change before firing, it goes to the other CELL's zone (`MAX_REDIRECTS`).
- Turning near walls: the robot (17 x 18 in) sweeps its corners out to 12.4 in when it turns, so it turns only with its centre `WALL_TURN_CLEAR_IN` (13.5 in) from every wall. A drive that starts against a wall with a turn moves straight off it first; one that ends against a wall finishes the turn and moves straight in (`Steps.legs`). The GARDEN pass lines up against the wall at `GARDEN_LEAD_IN_X` first, then drives along it with no turn.
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

Shooting-zone polygon; delay default; `CYCLE_*` order and count; park window length; `BALLS_FIRST_TIP` (4) and `BALLS_NEXT_TIP` (8) (confirm on the field); `SHOOT_FEED_S`; aim-settle time; speed limit for firing; B preset x within 60-63.

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

State: `up` (CellSide; starts RIGHT, the camera sets it), `tips` (observed tips since the start, `AutoContext.tips`), `cycles` done (max `DEFAULT_CYCLES` = 3), `timeLeft`. A tip is observed (the camera sees `up` change, or a burst is interrupted) or, when the camera has not read the HIVE since a burst, predicted from our ball count (4 for the first tip, 8 after); each does `tips++` and `up = other`.

| State | Does | Leaves to |
|---|---|---|
| WAIT (SECOND only) | stay still | GOTO on a seen tip (`tips`=1) or delay end |
| GOTO | skip the drive if already in the zone for `up`, else drive to `ZoneTargets.target(up)` facing the CELL | READY |
| READY | wait for the TeleOp ready-to-fire test, <= `READY_TIMEOUT_S` | GOTO if `up` changed (`tips`++), else SHOOT |
| SHOOT | burst (auto-fire); a tip mid-burst (`tips`++) finishes the burst once more at the other CELL | SETTLE |
| SETTLE | wait >= `TIP_SETTLE_MIN_S`, then for a camera read, <= `TIP_SETTLE_S` | DECIDE |
| DECIDE | first rule that applies wins (below) | COLLECT or PARK |
| COLLECT | drive to the cycle's source with the intake on | GOTO |
| PARK | LEAVE/PARK pose | end |

`timeLeft` <= `RESERVE_TIME_LEFT_S` preempts any state and goes to PARK.

DECIDE: (1) no source left that fits (trips + `BURST_S` + park trip + `CYCLE_MARGIN_S` <= `timeLeft`) or `cycles` = 3 -> PARK; (2) otherwise COLLECT. Every slot, FIRST included, cycles until then. `up` never changes the decision, only where GOTO goes.

Tips and the plan: a tip needs balls in the up CELL, and the first needs only 4 POLLEN (each CELL starts with 3 NECTAR in it); every later tip needs 8, which is two bursts (`BALLS_FIRST_TIP`, `BALLS_NEXT_TIP`). The 20 POLLEN (4 pre-loaded on each robot, one 4-ball cycle at each of the GARDEN and the two FLOWERS; NECTAR ignored) therefore make at most three tips: 4 + 8 + 8. Either robot's balls count, and whichever CELL is up gets them.

| Slot | Collects (`*_SOURCES`) | Ideal bursts |
|---|---|---|
| SOLO | GARDEN, back FLOWER | 3: pre-load at the first CELL (tips), GARDEN at the other CELL, back FLOWER at the same CELL (second 4 of 8, tips) |
| FIRST (start A) | GARDEN, wall FLOWER | pre-load at its CELL (tips), GARDEN at the other CELL (joins SECOND's burst there: 8, tips), wall FLOWER back at its CELL |
| SECOND (start B) | back FLOWER | pre-load at its CELL (4 of 8), back FLOWER at the CELL that is up then |

Each robot goes to whichever CELL is up (`AutoContext.shotSide` = `upSide`), to the nearest usable point of that CELL's zone; for the partner's CELL it keeps `PARTNER_CLEAR_IN` from the partner's start. A trip is re-checked before firing and redirected if the HIVE tipped on the way. Nobody waits for "their" CELL, because one is always up. Tip belief: the camera decides; our own balls are counted (`AutoContext.ballsToTip`, `recordShot`), and when the camera has not read the HIVE since the burst the count is believed (`settleTipBelief`, `PREDICT_TIP_WHEN_BLIND`). Alone, a burst that leaves the count under the threshold skips the camera wait (`mayHaveTipped`). With a partner its balls are not counted, so the camera wait stays. SECOND only shoots its own CELL and holds fire while it is down. FIRST never parks early. There is no ball sensor, so an emptied source is not detected; the cycle still costs its time.

SECOND (the partner agreed to shoot first): it waits the agreed delay, or a seen tip, and then decides at once (`assumePartnerTipped`). If the camera read the HIVE within `BLIND_READ_MAX_AGE_S` it trusts that read (our CELL still down means the partner has not tipped it, so SECOND shoots the CELL that is up); with no recent read it takes it that the partner tipped the HIVE and shoots its own CELL. If the partner never fires, the blind case wastes one burst.

Time budget (30 s) and the choice of source: a cycle starts only if its two trips, one burst (`BURST_S`), the trip from the next shot spot to PARK and `CYCLE_MARGIN_S` fit in `timeLeft`. Trips are worked out from the poses (`TripTimes`): straight-line distance, x `CROSS_DETOUR` when the drive crosses between the audience and far halves, at `AVG_SPEED_IN_PER_S`, plus `TURN_S`; a FLOWER adds `FLOWER_ARM_S` + `FLOWER_COLLECT_S`, the GARDEN is collected while driving. Sources (decided 2026-11-27): SOLO keeps shooting the CELL that is up until it tips, then the other; each shot after the pre-loads is preceded by a collection at the unused source nearest to the robot that still fits (`SOLO_NEAREST_SOURCE`). From start A that gives: no tip -> GARDEN, Zone A, wall FLOWER, Zone A, park; pre-loads tip -> GARDEN, Zone B, back FLOWER, Zone B (tips), then the wall FLOWER if it fits. In a pair the split is fixed: FIRST GARDEN, wall FLOWER; SECOND back FLOWER. A source that does not fit is skipped; none fits -> park. PARK is always banked: the reserve window (`RESERVE_TIME_LEFT_S`) abandons whatever is running and drives to PARK. Every step change is logged as `AutoStep` with the time left; use those times to replace the placeholder speed and costs. With the placeholders (about 35 in/s, not measured), SOLO from start A does: burst, GARDEN, burst at the far spot, then parks (no third burst fits); after a miss on the first burst it stays at the first spot, takes the GARDEN, and bursts again.

By tips (side after each observed tip; the table assumes every tip is seen):

| tips | up | SOLO (A) | FIRST (A) | SECOND (B) |
|---|---|---|---|---|
| 0 | RIGHT | shoot Zone A, cycle | shoot Zone A, cycle | wait in Zone B for the tip (delay first) |
| 1 | LEFT | next shot in Zone B, cycle | next shot in Zone B, cycle | shoot Zone B, back FLOWER, Zone B |
| 2 | RIGHT | next shot in Zone A, cycle | next shot in Zone A, cycle | hold fire in Zone B, wait for the next tip; reserve window parks |
| 3 | LEFT | next shot in Zone B, cycle | next shot in Zone B, cycle | shoot Zone B |

`tippedSinceStart()` is `tips > 0` (the SECOND wait uses `tips > tipsAtStart`), so two tips are not read as none.

Known gaps (fixed camera): a tip while the robot is collecting or driving is only seen at ARRIVE or SETTLE, and a tip while blind is never seen, so a burst can go at the down CELL. The mid-burst check is the only recovery.

## Camera limitation (fixed, decided)

There is one Limelight 3A, mounted on the chassis, with no pan servo (a pan servo was built and then removed). The camera sees the HIVE only while the chassis faces it, so:
- tips are read while the robot is at the start or in the zone (it faces the up CELL there: before firing, during the burst, after it), not while collecting or driving;
- the odometry-based plan is the fallback whenever the camera has no view (timeouts in `Steps.awaitCamera` end at once without a sighting);
- redirecting mid-drive on a tip seen en route is not possible and is out of scope.

## TeleOp ready-to-fire and the up/down state

Ready = in the shot zone (distance ring and approach angle) AND loaded (intake ran `REARM_INTAKE_S` since the last shot, or the robot left and re-entered the zone) AND the up/down state is confirmed. The driver presses shoot; there is no auto-fire.

The turret and hood always aim at the believed-up CELL from odometry (`HiveTracker` remembers the field position). The camera is fixed, so it only reads the HIVE while the chassis faces it; what odometry cannot know is whether the HIVE has tipped. `ShootGateConstants.UPSIDE_CHECK` therefore requires the last camera read of our CELLS (up or down) to be at most `UPSIDE_MAX_AGE_S` old and to be later than the last shot that fed balls. A normal cycle passes or drives under the HIVE before reaching the zone, so the read is fresh. A down-read of one CELL switches the aim to the other (known field point from `AutoConstants`). If no recent read: amber LED, "UP CELL UNCONFIRMED", no rumble; the shoot button still works and D-pad right (driver or operator) flips the aimed CELL, which counts as a read until the camera disagrees. In TeleOp only (`Robot.enableTeleOpMemory`) the remembered CELL position never expires (`TELEOP_ODOMETRY_FALLBACK_MAX_S` = 0), so the turret keeps aiming from odometry and only the up/down state ages; a down-read also switches the aim to the other CELL. AUTO keeps its own odometry fallback (`ODOMETRY_FALLBACK_MAX_S` 20 s, `ctx.upSide`). TeleOp starts aimed at the CELL AUTO believed was up (`PoseHandoff`), unconfirmed until the first camera read. The age of the read is logged (`ShootZone` tag) each time the ready signal fires, to set `UPSIDE_MAX_AGE_S`.

### SECOND (partner shoots first), after the delay

Superseded 2026-11-27: SECOND only shoots Zone B and holds fire while its CELL is down (see the summary at the top and the next section).

### Slots and the partner (decided 2026-11-27)

The stronger robot takes FIRST and always starts at A; the weaker takes SECOND and starts at B; SOLO starts at A. Picking the slot on the init menu sets the start (`AutoSelection.presetForSlot`); a hand-changed start shows a warning.

- FIRST: pre-loads in Zone A; GARDEN, then shoot whichever CELL is up (Zone A if the pre-loads missed, Zone B if they tipped); wall FLOWER, then shoot whichever is up; park. Never the back FLOWER.
- SECOND: only ever shoots its own CELL, Zone B (`AutoContext.shotSide`), so it never drives into Zone A where FIRST shoots. Delay, then pre-loads in Zone B; back FLOWER, then Zone B; park. Whenever its CELL is facing down at fire time (FIRST missed, or the HIVE has tipped back), it holds fire in Zone B and waits for the tip (`waitsForCell`); if the tip never comes, the reserve window parks it with the balls kept for TeleOp. Without a recent camera read after the delay it assumes the partner tipped (`assumePartnerTipped`). The partner's auto is unknown (we only run our code on our robot), so the only partner rule is static: no zone point within `PARTNER_CLEAR_IN` (24 in) of the partner's start, whichever CELL we shoot, because most autos shoot their pre-loads from their start or close by. Not covered: a partner that drives into the zone we use; agree routes with the partner team before the match.

The no-tip plan flag (LEAVE/STAY) was removed 2026-11-27: with SECOND holding fire until its own CELL is up, both plans behaved the same.

### Camera-confirmed tip checks (SOLO, also used by FIRST/SECOND)

`Steps.awaitCamera` waits for a camera read of one of our CELLS (`ctx.lastCellReadS`, set in `AutoOpMode.trackUpSide`) taken after a given time, capped, and ends at once without a camera. A CELL read facing down also sets the other one up (the two are joined). Used twice: after a burst (min `TIP_SETTLE_MIN_S` 0.4 s so the HIVE has time to tip, cap `TIP_SETTLE_S` 1.5 s) and, in effect, before firing: `toZoneAndReady` waits for a fresh up/down read as part of the ready test and goes to the other CELL's zone if `upSide` changed. Not possible with the fixed camera: redirecting mid-drive when a tip is seen en route; the extra leg is paid only after arriving.
