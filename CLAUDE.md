# WHCompCode2026 — Team 219 "Team Impact" FRC Robot Program

## Project Overview

This is the FRC robot program for **Team 219 — Team Impact**, competing in the **FIRST Robotics Competition 2026** season. The game is **REBUILT**, which involves fuel/hub scoring mechanics.

## Technology Stack

- **WPILib 2026** — FRC Java robot framework
- **GradleRIO 2026.2.1** — build system
- **CTRE Phoenix 6** — swerve drivetrain motor controllers
- **PhotonVision 2026.3.1** — computer vision / AprilTag pose estimation
- **JDK**: `JAVA_HOME=/home/gary/wpilib/2026/jdk`

## Build

```
JAVA_HOME=/home/gary/wpilib/2026/jdk bash gradlew build
```

## Robot Architecture

- **Swerve drivetrain** via CTRE Phoenix 6 (`CommandSwerveDrivetrain`, `TunerConstants3`)
- **Vision** via PhotonVision on an Orange Pi (`VisionSubsystem`) — front and side cameras
- **Hub-facing auto-align** command (`FaceHubCommand`) using `FieldCentricFacingAngle`

## Key Source Files

| File | Purpose |
|------|---------|
| `src/main/java/frc/robot/RobotContainer.java` | Subsystem instantiation and button bindings |
| `src/main/java/frc/robot/generated/TunerConstants3.java` | Swerve constants and motor IDs |
| `src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java` | Drivetrain subsystem |
| `src/main/java/frc/robot/subsystems/VisionSubsystem.java` | PhotonVision integration |
| `src/main/java/frc/robot/commands/FaceHubCommand.java` | Hub-facing command |
