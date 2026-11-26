# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

An FTC (FIRST Tech Challenge) robot for the 2026–2027 BIOBUZZ season. The code is an Android app built on FTC SDK 12.0. All team code lives in `TeamCode/`. `FtcRobotController/` is the stock SDK: don't edit it except to read its samples in `external/samples/`.

## Commands

Run these from the repo root (on Windows use `gradlew.bat` or `./gradlew` in Git Bash):

- Build: `./gradlew :TeamCode:assembleDebug`
- Install to a connected Control Hub: `./gradlew :TeamCode:installDebug` (Android Studio's Run button does the same thing)
- The Sloth plugin (`dev.frozenmilk.sinister.sloth.load`) is applied, so TeamCode can be hot-reloaded without reinstalling the app. Run its task from Android Studio's Gradle panel. The task name hasn't been checked here.
- The command line needs `JAVA_HOME` set (for example to Android Studio's bundled JBR). Without it, `gradlew` fails right away.

There are no unit tests. A change is only checked for real when it runs on the robot. "Tests" here means the hardware test OpModes in `opmodes/test/`. A clean `assembleDebug` is the most you can check from this machine.

## Architecture

Package root: `TeamCode/src/main/java/org/firstinspires/ftc/teamcode/`.

- **`Robot`** owns every subsystem plus `HubManager`, the Pedro `Follower`, `Controls`, `ShotSolver` and `ShotController`. OpModes only forward lifecycle calls: `init → new Robot(this)`, `start → robot.start()`, `loop → robot.update()`, `stop → robot.stop()`. `Robot.update()` must call `hubManager.clearBulkCache()` first, because hubs use MANUAL bulk caching, so sensor reads are stale until the cache is cleared each loop. It then sets the loop timing (`robot.nowNs`, `robot.loopSeconds`) and the static `Robot.voltageScale`, which other classes read instead of timing things or reading the battery themselves. The order of the `run*()` calls is listed in the `Robot` Javadoc.
- **`subsystems/`**: each mechanism implements `Subsystem` (`stop()`, `addTelemetry()`). A subsystem owns its hardware and never reads a gamepad. The usual pattern is: set public fields for what it should do (e.g. `intake.intakeOn`, `shooter.state`), then call its `update()` once per loop to write the hardware. `Robot` loops over the subsystem list for telemetry and stop, so a new subsystem must be added to that list.
- **Shooting**: `utils/ShotController` aims the turret, sets the flywheel speed and runs the shot sequence `IDLE → BLOCKER_OPEN → SHOOTING` while `shootPressed` is held (dropping to `IDLE` on release). `Robot.runIntake()` opens the blocker and runs the transfer from that state. `utils/ShotSolver` supplies the auto-aim targets; it's a placeholder for now. `automaticShot` switches between solver aim and manual turret + `MANUAL_RPM`.
- **Units**: `Turret` works in radians to the left of robot forward; degree constants (servo range, deadbands) are converted inside the class. `TurretConstants.MANUAL_RATE` is rad/s.
- **Style**: plain public fields instead of getters/setters that only read or write a field. Keep a method only when it does more (clamping, writing hardware, derived values).
- **`controls/`**: one `Controls` class reads both gamepads (driver = gamepad 1, operator = gamepad 2) and turns them into intents; thresholds are in `ControlsConstants`. The button map is in its Javadoc. `controls.update()` must run each loop before the `*Pressed` edge fields are read. Use `utils/EdgeDetector` for press/toggle detection.
- **`constants/`**: all tunable numbers and hardware-config device names, as `public static` fields on `@Configurable` classes. These can be edited live from Panels (bylazar) on the robot, so don't hard-code tunables or device names in subsystems. Values marked `CALIBRATE` haven't been measured yet.
- **`opmodes/teleop/`**: `RobotTeleOp` (lifecycle base) → `MatchTeleOp` → `TeleOpRed` / `TeleOpBlue`. The alliance only picks which HIVE to aim at.
- **`opmodes/test/`**: standalone single-mechanism OpModes (group `"Test"`) that build one subsystem directly instead of a full `Robot`.
- **`pedro/`**: Pedro Pathing setup. `Constants.create(hardwareMap)` builds the `Follower` used by `Drivetrain` for field-centric drive and odometry. `Tuning` and `procedures/` are Pedro's tuner OpModes.
- **Vision**: a Limelight 3A (`subsystems/Vision`, `utils/HiveTracker`, `HiveCell`/`HiveSide`/`CellSide`) finds the HIVE target. The turret auto-aims from that bearing, plus odometry-velocity lead and angular-velocity feedforward (`Turret.setFeedforward`).
- **Shooter logging**: `utils/AsyncRpmLogger` writes shooter RPM CSVs off the loop thread to `shooter_diagnostics/` on the robot. Pulled copies are kept in `shooter_logs/` at the repo root.

## Current state

The code is partway through a rewrite. TeleOp currently drives, intakes, aims the turret (auto or manual), runs the shot sequence and toggles the flower arm (Y). Not wired up yet: the `ShotSolver` math (it aims straight ahead at `MANUAL_RPM`), `Shooter.isAtTargetRpm()` (a placeholder that always returns `true`), `Hood`, `Vision`, `Butterfly`, and the reset-pose buttons. Update the comments when you change the behaviour they describe.
