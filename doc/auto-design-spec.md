# BIOBUZZ AUTO and TeleOp spec

Current behavior of the code on branch `auto-redesign` (PR #10), for the team's programmers. Sources: the code; BIOBUZZ manual TU10 (Figures 9-2, 9-9, 9-10, 10-2, 11-1, G304, G402). Field positions are read off the manual figures, about ±2 in.

## Goals

1. One AUTO OpMode per alliance that works from either start, alone (SOLO) or with a partner (PAIR).
2. Auto and TeleOp share the same aiming, shooting-zone and fire-gate code.
3. Always bank LEAVE and PARK. Shots are worth more, but they never put LEAVE/PARK at risk.

Non-goals (for now): scoring into FLOWERS (the robot collects POLLEN from FLOWERS, but has no mechanism to put NECTAR or POLLEN into a FLOWER at end game), NECTAR handling (robot cannot intake it), Limelight blob pipeline for POLLEN.

## AUTO

What the code on `auto-redesign` does. All numbers are placeholders until measured on the robot.

### Every shot, all slots

- **Getting into the zone:** if the robot is already in the TeleOp shooting zone (distance ring + off-axis angle, `ShotZoneGeometry`), it does not drive. Otherwise it drives to the nearest point `ZONE_INSET_IN` (3 in) inside the zone edge (`ZoneTargets`). Those points are on our side of the middle line, at least `WALL_TURN_CLEAR_IN` from every wall, clear of FLOWERS, and in a pair at least `PARTNER_CLEAR_IN` (24 in) from the partner's start. The robot faces the CELL there so the fixed camera sees the HIVE.
- **Firing:** the same "ready to fire" test as TeleOp (`ShootZone`): in the zone, aim and hood settled, wheel at speed, and a recent camera read of which CELL is up. Auto fires as soon as it passes, or after `READY_TIMEOUT_S` (1.5 s) anyway. The wheel pre-spins in the zone.
- **After a burst:** each HIVE tips at a slightly different speed, so there is no fixed wait. SOLO, when our balls should have tipped it (3 for the first tip, then 8), keeps watching until the camera sees the tip, at most `TIP_WATCH_S` (1.5 s), then plans the next trip. PAIR does not wait. With no camera read since the burst, the robot goes by its ball count. Before every shot a fresh camera read is required anyway (the ready test).
- **Tip mid-burst:** feeding stops at once. SOLO finishes the burst at the other CELL; PAIR holds the rest until its own CELL is up again.
- **Time budget:** a cycle starts only if collecting, the shot, the drive to park and a margin all fit in the time left.
- **End of auto:** with `RESERVE_TIME_LEFT_S` (6 s) left, whatever is running is dropped and the robot parks.
- **Turning near walls:** the robot never turns with its centre closer than `WALL_TURN_CLEAR_IN` (13.5 in) to a wall. It moves straight off the wall first, or finishes the turn before moving in.

### SOLO (start A)

1. Shoots the pre-loads from the start, in Zone A.
2. Before each later shot, collects at the nearest unused source, then shoots whichever CELL is up.
3. No tip: GARDEN, shoot in Zone A, wall FLOWER, shoot in Zone A, park.
4. Pre-loads tip: GARDEN, shoot in Zone B, back FLOWER, shoot in Zone B (that tips it), then the wall FLOWER and a shot in Zone A if it fits, otherwise park.

### PAIR (partner shoots too): each robot takes care of its own zone

The simplest safe option: we do not know the partner's auto, so we never drive into its zone. We shoot only our own CELL (start A: Zone A, start B: Zone B), collect only our side's sources, and hold fire while our CELL is facing down. The start is picked by hand: the stronger robot of the pair takes start A (it shoots first), the weaker takes start B. Every match starts with the RIGHT CELL up (manual 10.3.1), which is Zone A's: start A shoots at once, start B waits for the tip.

- **From A:** pre-loads in Zone A; GARDEN, shoot in Zone A; wall FLOWER, shoot in Zone A; park. Each shot waits in Zone A while our CELL is down.
- **From B:** no fixed delay: hold fire at the start until our own camera sees our CELL up (the partner's pre-loads tipped the HIVE); then pre-loads in Zone B; back FLOWER, shoot in Zone B; park.
- **If our CELL never comes up:** the reserve window parks us with the balls on board for TeleOp.
- As a backstop, no shooting point is within `PARTNER_CLEAR_IN` of the other preset.

Cost: with a weak partner, tip 2 depends on the partner putting 8 balls into Zone B, so from A we usually get one tip and hold the GARDEN load. Two strong robots can make three tips (A: 3 + later 8, B: 8), if the time allows.

## TeleOp

What `TeleOp: Red` / `TeleOp: Blue` (`MatchTeleOp`, `Robot.update`) do. Numbers are placeholders until measured.

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

## Field and frame

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
- Both robots start at 0:00 and cannot talk to each other; the slot (SOLO or PAIR) and our start are agreed with the partner team before the match.
- Both robots shoot into the same alliance HIVE. A partner's shots can tip it, so the up-CELL can change at any moment.

## Start presets (red frame, robot 17 in. wide x 18 in. long)

`StartPreset`, poses in `AutoConstants` (red); blue is the half-turn above.

| Preset | Pose (x, y, deg) | Footprint | Notes |
|---|---|---|---|
| A | (59, 9, 90) | x 50.5-67.5, y 0-18 | audience wall, back to the wall, facing the AUDIENCE (RIGHT) CELL 52 in away; clear of GARDEN and LOADING ZONE |
| B | (61.5, 135, 270) | x 53-70, y 126-144 | far wall, back to the wall, facing the FAR (LEFT) CELL 47 in away; FLOWER at x 45-51 and column C (<= 72) allow centre x 59.5-63.5 |

Blue: A -> (85, 135, 270); B -> (82.5, 9, 90). Both starts face the HIVE so the fixed camera watches it from the start, and both are inside the shooting zone for the CELL they face (B only just: 47 in against a 46.5 in edge with the current shot table).

## Init menu (gamepad 1, in `init_loop`)

The alliance is the OpMode (`Auto: Red` / `Auto: Blue`), so the right TeleOp is preselected.

| Input | Control | Default |
|---|---|---|
| Start preset | D-pad left/right (A / B) | A |
| Slot | A cycles SOLO / PAIR; SOLO also sets start A | SOLO |
| Cycles | D-pad up/down, 0-3 | `DEFAULT_CYCLES` (3) |

There is no up-CELL override: which CELL is up at the start comes only from the camera (during init), else RIGHT as the manual sets up every match. Telemetry shows the selection, the exact coordinates to place the robot at for this alliance, what PAIR will do from this start, and warnings: a start that does not match the slot; a start outside the zone; a CELL with no usable zone point; poses past our side of the field (G402). Selections are statics in `AutoSelection`, so they persist between matches until the app restarts or code is deployed.

## Robot and camera

- Holds 4 balls; no NECTAR intake.
- One Limelight 3A, fixed on the chassis facing forward (`CAMERA_YAW_DEG` = 0, to confirm), no pan servo. It sees the HIVE only while the chassis faces it: at the start and in the zone (before firing, during the burst, after it), not while collecting or driving. Redirecting mid-drive on a tip seen en route is not possible.

## Open items (measure on the robot)

- Shot table rows and the zone angle limit (`ZONE_MAX_OFF_AXIS_DEG`); they decide where the zone points are.
- Average drive speed (`AVG_SPEED_IN_PER_S`), `BURST_S`, `TURN_S`, FLOWER times: from the `AutoStep` log lines of a timed practice run.
- `READY_TIMEOUT_S`, `UPSIDE_MAX_AGE_S`: from the `ShootZone` log lines. `TIP_WATCH_S`: above the slowest tip seen.
- Hood servo full-travel time (`HOOD_SETTLE_S`, 0.3 s placeholder). Blocker: measured about 0.25 s, set to 0.3 s.
- `WALL_TURN_CLEAR_IN` with the real bumpers; camera view of the CELL tags from both starts (Hive Vision test OpMode).
