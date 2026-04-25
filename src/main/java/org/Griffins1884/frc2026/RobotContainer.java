package org.Griffins1884.frc2026;

import static org.Griffins1884.frc2026.Config.Controllers.getDriverController;
import static org.Griffins1884.frc2026.Config.Subsystems.AUTONOMOUS_ENABLED;
import static org.Griffins1884.frc2026.Config.Subsystems.DRIVETRAIN_ENABLED;
import static org.Griffins1884.frc2026.GlobalConstants.MODE;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.BACK_LEFT;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.BACK_RIGHT;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.FRONT_LEFT;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.FRONT_RIGHT;
import static org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants.GYRO_TYPE;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.PPLibTelemetry;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import java.io.IOException;
import java.util.Optional;
import org.Griffins1884.frc2026.GlobalConstants.RobotMode;
import org.Griffins1884.frc2026.OI.DriverMap;
import org.Griffins1884.frc2026.commands.AutoCommands;
import org.Griffins1884.frc2026.commands.DriveCommands;
import org.Griffins1884.frc2026.simulation.drive.Season2026DriveSimulation;
import org.Griffins1884.frc2026.simulation.drive.Season2026SwerveCorner;
import org.Griffins1884.frc2026.simulation.maple.MapleArenaSetup;
import org.Griffins1884.frc2026.simulation.maple.Rebuilt2026FieldModel;
import org.Griffins1884.frc2026.simulation.visualization.RobotStateVisualizer;
import org.Griffins1884.frc2026.subsystems.objectivetracker.OperatorBoardIOServer;
import org.Griffins1884.frc2026.subsystems.objectivetracker.OperatorBoardTracker;
import org.Griffins1884.frc2026.subsystems.swerve.GyroIO;
import org.Griffins1884.frc2026.subsystems.swerve.GyroIONavX;
import org.Griffins1884.frc2026.subsystems.swerve.GyroIOPigeon2;
import org.Griffins1884.frc2026.subsystems.swerve.GyroIOSim;
import org.Griffins1884.frc2026.subsystems.swerve.ModuleIO;
import org.Griffins1884.frc2026.subsystems.swerve.ModuleIOFullKraken;
import org.Griffins1884.frc2026.subsystems.swerve.ModuleIOSim;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveConstants;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  private final SwerveSubsystem drive;
  private Season2026DriveSimulation driveSimulation;
  private final OperatorBoardTracker operatorBoard;
  private final DriverMap driver = getDriverController();
  private final LoggedDashboardChooser<Command> characterizationChooser;
  private final Command characterizationIdleCommand;
  private final RobotStateVisualizer robotStateVisualizer;
  private boolean autoAllianceZeroed = false;

  public RobotContainer() {
    characterizationChooser = new LoggedDashboardChooser<>("Characterization/Diagnostics");
    characterizationIdleCommand = Commands.none();
    characterizationChooser.addDefaultOption("None", characterizationIdleCommand);

    drive = DRIVETRAIN_ENABLED ? buildDrive() : null;

    if (Config.Subsystems.WEBUI_ENABLED) {
      operatorBoard = new OperatorBoardTracker(new OperatorBoardIOServer(), drive);
    } else {
      operatorBoard = null;
    }

    robotStateVisualizer =
        new RobotStateVisualizer(
            drive,
            MODE == RobotMode.SIM && driveSimulation != null ? driveSimulation::getPose3d : null);

    registerCharacterizationOptions();
    configureDriverButtonBindings();
    configurePathPlannerAutonomous();
    configurePathPlannerTelemetry();

    SmartDashboard.putBoolean("drive/test", DriveCommands.getTest().get());
  }

  private SwerveSubsystem buildDrive() {
    return switch (MODE) {
      case REAL ->
          new SwerveSubsystem(
              switch (GYRO_TYPE) {
                case PIGEON -> new GyroIOPigeon2();
                case NAVX -> new GyroIONavX();
                case ADIS -> new GyroIO() {};
              },
              new ModuleIOFullKraken(FRONT_LEFT),
              new ModuleIOFullKraken(FRONT_RIGHT),
              new ModuleIOFullKraken(BACK_LEFT),
              new ModuleIOFullKraken(BACK_RIGHT));
      case SIM -> buildSimDrive();
      case REPLAY ->
          new SwerveSubsystem(
              new GyroIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {},
              new ModuleIO() {});
    };
  }

  private SwerveSubsystem buildSimDrive() {
    MapleArenaSetup.ensure2026RebuiltArena();
    driveSimulation =
        Season2026DriveSimulation.mapleTerrainAware(
            new SwerveDriveSimulation(
                SwerveConstants.MAPLE_SIM_CONFIG, new Pose2d(3, 3, new Rotation2d())),
            Rebuilt2026FieldModel.contactModel(),
            Rebuilt2026FieldModel.CHASSIS_FOOTPRINT,
            Rebuilt2026FieldModel.CHASSIS_MASS_PROPERTIES,
            Timer::getFPGATimestamp);
    SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.mapleSimulation());

    return new SwerveSubsystem(
        new GyroIOSim(driveSimulation),
        new ModuleIOSim(
            driveSimulation, Season2026SwerveCorner.FRONT_LEFT, driveSimulation.module(0)),
        new ModuleIOSim(
            driveSimulation, Season2026SwerveCorner.FRONT_RIGHT, driveSimulation.module(1)),
        new ModuleIOSim(
            driveSimulation, Season2026SwerveCorner.REAR_LEFT, driveSimulation.module(2)),
        new ModuleIOSim(
            driveSimulation, Season2026SwerveCorner.REAR_RIGHT, driveSimulation.module(3)));
  }

  private void registerCharacterizationOptions() {
    if (!DRIVETRAIN_ENABLED || drive == null) {
      return;
    }
    characterizationChooser.addOption(
        "Drive | SysId (Full Routine)", drive.sysIdRoutine().ignoringDisable(true));
    characterizationChooser.addOption(
        "Drive | Wheel Radius Characterization",
        DriveCommands.wheelRadiusCharacterization(drive).ignoringDisable(true));
    characterizationChooser.addOption(
        "Drive | Wheel Radius Characterization + Save",
        DriveCommands.wheelRadiusCharacterization(drive, true).ignoringDisable(true));
    characterizationChooser.addOption(
        "Drive | Clear Saved Wheel Radius",
        DriveCommands.clearSavedWheelRadius().ignoringDisable(true));
    characterizationChooser.addOption(
        "Drive | Capture Module Zero Offsets",
        DriveCommands.captureModuleZeroOffsets(drive).ignoringDisable(true));
    characterizationChooser.addOption(
        "Drive | Capture FL Zero Offset",
        DriveCommands.captureModuleZeroOffset(drive, 0, "FL").ignoringDisable(true));
    characterizationChooser.addOption(
        "Drive | Capture FR Zero Offset",
        DriveCommands.captureModuleZeroOffset(drive, 1, "FR").ignoringDisable(true));
    characterizationChooser.addOption(
        "Drive | Capture BL Zero Offset",
        DriveCommands.captureModuleZeroOffset(drive, 2, "BL").ignoringDisable(true));
    characterizationChooser.addOption(
        "Drive | Capture BR Zero Offset",
        DriveCommands.captureModuleZeroOffset(drive, 3, "BR").ignoringDisable(true));
    characterizationChooser.addOption(
        "Drive | Clear Module Zero Offsets",
        DriveCommands.clearModuleZeroOffsets(drive).ignoringDisable(true));
    characterizationChooser.addOption(
        "Drive | Clear FL Zero Offset",
        DriveCommands.clearModuleZeroOffset(drive, 0, "FL").ignoringDisable(true));
    characterizationChooser.addOption(
        "Drive | Clear FR Zero Offset",
        DriveCommands.clearModuleZeroOffset(drive, 1, "FR").ignoringDisable(true));
    characterizationChooser.addOption(
        "Drive | Clear BL Zero Offset",
        DriveCommands.clearModuleZeroOffset(drive, 2, "BL").ignoringDisable(true));
    characterizationChooser.addOption(
        "Drive | Clear BR Zero Offset",
        DriveCommands.clearModuleZeroOffset(drive, 3, "BR").ignoringDisable(true));
    characterizationChooser.addOption(
        "Drive | Feedforward Characterization",
        DriveCommands.feedforwardCharacterization(drive).ignoringDisable(true));
    characterizationChooser.addOption(
        "Drive | SysId (Quasistatic Forward)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward).ignoringDisable(true));
    characterizationChooser.addOption(
        "Drive | SysId (Quasistatic Reverse)",
        drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse).ignoringDisable(true));
    characterizationChooser.addOption(
        "Drive | SysId (Dynamic Forward)",
        drive.sysIdDynamic(SysIdRoutine.Direction.kForward).ignoringDisable(true));
    characterizationChooser.addOption(
        "Drive | SysId (Dynamic Reverse)",
        drive.sysIdDynamic(SysIdRoutine.Direction.kReverse).ignoringDisable(true));
    characterizationChooser.addOption(
        "Turn | SysId (Full Routine)", drive.sysIdTurnRoutine().ignoringDisable(true));
    characterizationChooser.addOption(
        "Turn | SysId (Quasistatic Forward)",
        drive.sysIdTurnQuasistatic(SysIdRoutine.Direction.kForward).ignoringDisable(true));
    characterizationChooser.addOption(
        "Turn | SysId (Quasistatic Reverse)",
        drive.sysIdTurnQuasistatic(SysIdRoutine.Direction.kReverse).ignoringDisable(true));
    characterizationChooser.addOption(
        "Turn | SysId (Dynamic Forward)",
        drive.sysIdTurnDynamic(SysIdRoutine.Direction.kForward).ignoringDisable(true));
    characterizationChooser.addOption(
        "Turn | SysId (Dynamic Reverse)",
        drive.sysIdTurnDynamic(SysIdRoutine.Direction.kReverse).ignoringDisable(true));
  }

  private void configureDriverButtonBindings() {
    if (!DRIVETRAIN_ENABLED || drive == null) {
      return;
    }

    drive.setDefaultCommand(
        DriveCommands.joystickDriveCommand(
            drive, driver.getYAxis(), driver.getXAxis(), driver.getRotAxis()));

    driver
        .alignWithBall()
        .whileTrue(
            DriveCommands.joystickDriveRobotRelativeFlippedCommand(
                drive, driver.getYAxis(), driver.getXAxis(), driver.getRotAxis()));

    driver
        .resetOdometry()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      Optional<Alliance> alliance = DriverStation.getAlliance();
                      if (alliance.isEmpty()) {
                        Logger.recordOutput("Odometry/AllianceZero/Failed", true);
                        Logger.recordOutput("Odometry/AllianceZero/Reason", "ALLIANCE_UNKNOWN");
                        return;
                      }
                      drive.zeroGyroAndOdometryToAllianceWall(alliance.get());
                    },
                    drive)
                .ignoringDisable(true));
  }

  private void configurePathPlannerAutonomous() {
    AutoCommands.registerAutoCommands(drive);
    if (drive == null) {
      return;
    }
    try {
      RobotConfig robotConfig = RobotConfig.fromGUISettings();
      AutoBuilder.configure(
          drive::getPose,
          pose -> resetRobotToPose(pose, MODE == RobotMode.SIM),
          drive::getRobotRelativeSpeeds,
          drive::runVelocity,
          new PPHolonomicDriveController(new PIDConstants(5.0), new PIDConstants(5.0)),
          robotConfig,
          () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
          drive);
    } catch (IOException | ParseException ex) {
      DriverStation.reportError(
          "Failed to configure PathPlanner autonomous: " + ex.getMessage(), ex.getStackTrace());
    }
  }

  private void configurePathPlannerTelemetry() {
    if (drive == null) {
      return;
    }

    PathPlannerLogging.setLogCurrentPoseCallback(
        pose -> Logger.recordOutput("PathPlanner/CurrentPose", pose));
    PathPlannerLogging.setLogTargetPoseCallback(
        pose -> Logger.recordOutput("PathPlanner/TargetPose", pose));
    PathPlannerLogging.setLogActivePathCallback(
        poses -> Logger.recordOutput("PathPlanner/ActivePath", poses.toArray(Pose2d[]::new)));
  }

  public Command getAutonomousCommand() {
    if (!AUTONOMOUS_ENABLED) {
      return null;
    }
    return operatorBoard != null ? operatorBoard.getAutonomousCommand() : null;
  }

  public void applyQueuedAutonomousStartPose() {
    if (!AUTONOMOUS_ENABLED || drive == null) {
      return;
    }
    getQueuedAutonomousStartPose()
        .ifPresent(
            pose -> {
              resetRobotToPose(pose, MODE == RobotMode.SIM);
              Logger.recordOutput("Autonomous/QueuedStartPoseApplied", pose);
            });
  }

  public Command getCharacterizationCommand() {
    Command selected = characterizationChooser.get();
    return selected == characterizationIdleCommand ? null : selected;
  }

  public Command getDriveSysIdCommand() {
    if (!DRIVETRAIN_ENABLED || drive == null) {
      return Commands.none();
    }
    return drive.sysIdRoutine().ignoringDisable(true);
  }

  public void resetSimulationField() {
    if (MODE != RobotMode.SIM || driveSimulation == null || drive == null) {
      return;
    }
    resetRobotToPose(new Pose2d(3, 3, new Rotation2d()), true);
  }

  public void tryAutoZeroOdometryToAllianceWall() {
    if (autoAllianceZeroed || drive == null || !DriverStation.isDisabled()) {
      return;
    }
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return;
    }
    drive.zeroGyroAndOdometryToAllianceWall(alliance.get());
    autoAllianceZeroed = true;
    Logger.recordOutput("Odometry/AutoAllianceZeroed", true);
  }

  public void displaySimFieldToAdvantageScope() {
    if (MODE == RobotMode.SIM) {
      robotStateVisualizer.periodic();
    }
  }

  public void setTeleopState() {}

  public void periodic() {
    if (drive != null && MODE != RobotMode.REPLAY) {
      PPLibTelemetry.setCurrentPose(drive.getPose());
    }
    if (MODE != RobotMode.SIM) {
      robotStateVisualizer.periodic();
    }
  }

  public void applySimulationValidationInputs() {}

  public void captureSimulationValidationStep() {}

  private Optional<Pose2d> getQueuedAutonomousStartPose() {
    return operatorBoard != null ? operatorBoard.getQueuedStartPose() : Optional.empty();
  }

  private void resetRobotToPose(Pose2d pose, boolean resetSimulationField) {
    if (drive == null || pose == null) {
      return;
    }

    if (resetSimulationField && MODE == RobotMode.SIM && driveSimulation != null) {
      driveSimulation.resetState(pose, new ChassisSpeeds());
      SimulatedArena.getInstance().resetFieldForAuto();
      robotStateVisualizer.reset();
    }

    drive.resetOdometry(pose, true);
  }
}
