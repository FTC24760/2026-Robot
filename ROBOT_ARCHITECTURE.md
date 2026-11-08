# Robot code organization

## Team-owned code

- `opmodes/teleop/ManualPidfTeleOp.java`: software PIDF shooter drive, original checked-in tune.
- `opmodes/teleop/HubVelocityTeleOp.java`: separate REV hub velocity PIDF experiment; uses its own PIDF values and legacy A-button behavior.
- `opmodes/teleop/ShooterBaselineDiagnosticTeleOp.java`: nonblocking CSV logging test mode, tuned separately from the production manual PIDF mode.
- `opmodes/teleop/CompetitionTeleOp.java`: existing competition-mode placeholder; currently does not implement driving or shooting.
- `opmodes/testing/`: motor/servo and field-centric drive tests.
- `subsystems/FieldCentricMecanumDrive.java`: shared field-relative/robot-relative drive and IMU setup.
- `subsystems/IntakeTurretMechanisms.java`: transfer/intake, turret and blocker; supports both PIDF and legacy control behaviors.
- `subsystems/ShooterVelocityController.java`: software PIDF extracted verbatim in terms of its control algorithm.
- `diagnostics/ShooterCsvLogger.java`: reusable nonblocking CSV writer used by the baseline shooter diagnostic.
- `subsystems/ButterflyController.java`: existing butterfly controller.
- `constants/RobotHardwareNames.java`: motor/servo/IMU configuration names and shooter encoder resolution.
- `constants/ManualShooterTuning.java`: the *checked-in original* ManualPidf tune and mechanism setpoints.

`subsystems/Shooter.java` and `utils/SpeedController.java` are a separate older shooter implementation using the original root `ShooterConstants`. They are not silently substituted for the manually tuned software controller; consolidation requires comparing their different units and coefficients on hardware.

`pedro/` and `pedroPathing/` contain trajectory/pathing configuration and tuning tools. Their package names are intentionally unchanged to avoid breaking SDK and generated references.

## Hardware and software cautions

- The FTC Control Hub hardware configuration names **must** match the string values in `RobotHardwareNames` (and the butterfly-specific names in `ButterflyController`).
- The diagnostics baseline previously used **1410 RPM** and **kP 0.0015**; the main checked-in TeleOp used **1470 RPM** and **kP 0.0051**. This refactor intentionally retains both distinct settings.
- The 28-tick encoder belongs to the bare goBILDA motor shaft. A 1:1 motor-to-flywheel belt makes the nominal motor RPM equal to flywheel RPM, but the encoder does not directly measure actual flywheel or counterroller RPM.
- No additional shot-recovery boost or feed gating has been added. Continuous transfer power remains 0.8 in the checked-in manual PIDF mode.
- The Pedro `Constants.create()` currently returns `null` in this repository. The Pedro TeleOps should **not** be treated as competition-ready until that follower initialization is implemented and tested.
- The `CompetitionTeleOp` class is currently a stub.
- The older `ServoTests` implementation used lowercase butterfly hardware names, unlike `ButterflyController`. The testing code retains those exact names until the robot's configuration is confirmed.

## Before merging

1. Run the FTC Android Gradle build (`./gradlew :TeamCode:assembleDebug`) with the team SDK available.
2. Inspect the Driver Station list: old joke-named OpModes are now listed as `Manual Software PIDF TeleOp`, `Hub Velocity TeleOp`, and `Shooter Baseline Diagnostic`.
3. Verify the shooter motor direction and existing right-trigger/left-trigger controls.
4. Verify A-button differences: manual software PIDF toggles the blocker and B resets yaw; hub-velocity mode uses A for the momentary blocker **and** yaw reset.
5. Test both drive modes and each butterfly/turret servo with wheels raised before driving on the field.
6. Confirm diagnostics write and stop without blocking before relying on CSVs.

This is a structure-first refactor, not a claim of validated shooter performance. Do not deploy to a live match without compile and on-robot checks.
