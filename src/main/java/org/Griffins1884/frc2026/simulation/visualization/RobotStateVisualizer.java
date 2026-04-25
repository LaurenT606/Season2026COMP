package org.Griffins1884.frc2026.simulation.visualization;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.List;
import java.util.function.Supplier;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.littletonrobotics.junction.Logger;

/** Publishes only the robot pose for AdvantageScope. */
public final class RobotStateVisualizer {
  private static final ShotTelemetrySnapshot EMPTY_SHOT_TELEMETRY =
      new ShotTelemetrySnapshot(0, 0, null, null, null, List.of());

  private final SwerveSubsystem drive;
  private final Supplier<Pose3d> simPose3dSupplier;
  private RenderTelemetry lastRenderTelemetry = new RenderTelemetry(null, null, null, false);

  public RobotStateVisualizer(SwerveSubsystem drive, Supplier<Pose3d> simPose3dSupplier) {
    this.drive = drive;
    this.simPose3dSupplier = simPose3dSupplier;
  }

  public void periodic() {
    Pose2d odometryPose = drive != null ? drive.getPose() : null;
    if (!isValidPose(odometryPose)) {
      clearOutputs();
      return;
    }

    Pose3d odometryPose3d = new Pose3d(odometryPose);
    Pose3d simPose3d = simPose3dSupplier != null ? simPose3dSupplier.get() : null;
    boolean usingSimPose = isValidPose3d(simPose3d);
    Pose3d publishedPose = usingSimPose ? simPose3d : odometryPose3d;

    Logger.recordOutput("FieldSimulation/RobotPosition", publishedPose);
    Logger.recordOutput("FieldSimulation/RobotPose3d", publishedPose);
    Logger.recordOutput(
        "FieldSimulation/RobotPose3dAuthoritative", usingSimPose ? simPose3d : new Pose3d());
    Logger.recordOutput("FieldSimulation/RobotPose3dTerrainAdjusted", odometryPose3d);
    Logger.recordOutput("FieldSimulation/RobotPose3dUsesAuthoritativeSource", usingSimPose);

    lastRenderTelemetry =
        new RenderTelemetry(
            publishedPose, usingSimPose ? simPose3d : null, odometryPose3d, usingSimPose);
  }

  public void reset() {
    clearOutputs();
  }

  public RenderTelemetry getRenderTelemetry() {
    return lastRenderTelemetry;
  }

  public ShotTelemetrySnapshot getShotTelemetrySnapshot() {
    return EMPTY_SHOT_TELEMETRY;
  }

  private void clearOutputs() {
    Logger.recordOutput("FieldSimulation/RobotPosition", new Pose3d());
    Logger.recordOutput("FieldSimulation/RobotPose3d", new Pose3d());
    Logger.recordOutput("FieldSimulation/RobotPose3dAuthoritative", new Pose3d());
    Logger.recordOutput("FieldSimulation/RobotPose3dTerrainAdjusted", new Pose3d());
    Logger.recordOutput("FieldSimulation/RobotPose3dUsesAuthoritativeSource", false);
    lastRenderTelemetry = new RenderTelemetry(null, null, null, false);
  }

  private static boolean isValidPose(Pose2d pose) {
    return pose != null
        && Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getRotation().getRadians());
  }

  private static boolean isValidPose3d(Pose3d pose) {
    return pose != null
        && Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getZ())
        && Double.isFinite(pose.getRotation().getX())
        && Double.isFinite(pose.getRotation().getY())
        && Double.isFinite(pose.getRotation().getZ());
  }

  public record RenderTelemetry(
      Pose3d publishedRobotPose3d,
      Pose3d authoritativeRobotPose3d,
      Pose3d terrainAdjustedRobotPose3d,
      boolean usingAuthoritativeRobotPose) {}

  public record ShotTelemetrySnapshot(
      int releaseCount,
      int completedCount,
      Object activeShot,
      Object lastReleasedShot,
      Object lastCompletedShot,
      List<?> visibleObservedArcSamples) {}
}
