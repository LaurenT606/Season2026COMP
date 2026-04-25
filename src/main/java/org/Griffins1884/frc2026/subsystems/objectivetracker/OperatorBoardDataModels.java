package org.Griffins1884.frc2026.subsystems.objectivetracker;

import java.time.Instant;
import java.util.List;
import org.Griffins1884.frc2026.BuildConstants;

public final class OperatorBoardDataModels {
  public static final String SCHEMA_VERSION = "2026.04";

  private OperatorBoardDataModels() {}

  public static AssetMetadata metadata(String type, String id, String displayName) {
    return new AssetMetadata(
        id,
        type,
        displayName,
        Instant.now().toString(),
        System.currentTimeMillis(),
        BuildConstants.VERSION,
        BuildConstants.GIT_SHA,
        BuildConstants.GIT_BRANCH);
  }

  public static SubsystemDescriptionsDocument emptyDefaultSubsystemDescriptions() {
    return new SubsystemDescriptionsDocument(
        SCHEMA_VERSION,
        metadata("subsystemDescriptions", "subsystem-descriptions", "Subsystem Descriptions"),
        List.of(
            new SubsystemDescription(
                "drive",
                "Swerve Drive",
                "Primary drivetrain for teleop, autonomous alignment, and odometry.",
                List.of("Four swerve drive modules", "Gyro"),
                List.of(
                    "Wheel positions", "Wheel velocities", "Robot pose", "Field-relative velocity"),
                List.of(
                    "Keep odometry valid before autonomous start.",
                    "Use alliance-wall zeroing when the field heading needs to be reset."),
                List.of("Teleop drive", "Alliance zero", "Drive SysId", "PathPlanner autos"),
                List.of(
                    "Drivers can place the robot accurately in teleop.",
                    "Autos start from a known heading and pose."),
                List.of(
                    new StateDescription(
                        "FIELD_RELATIVE",
                        "Normal match driving.",
                        "The robot follows translation and rotation commands relative to the field.",
                        "Gyro and odometry are reporting sane values.",
                        "Used during most teleop and autonomous path execution.",
                        "Bad odometry or gyro loss reduces confidence in field-relative motion."),
                    new StateDescription(
                        "CHARACTERIZATION",
                        "Drivebase diagnostic mode.",
                        "The drivetrain runs scripted SysId or wheel-radius routines.",
                        "Only active when operators intentionally start a characterization command.",
                        "Ends when the selected routine finishes or is cancelled.",
                        "Do not run during match play or while other commands are active."))),
            new SubsystemDescription(
                "autonomous",
                "Autonomous Queue",
                "Loads PathPlanner and Choreo autos from deploy and exposes a drive-only queue preview.",
                List.of("Named drive commands", "Auto queue builder"),
                List.of("Selected auto metadata", "Preview pose", "Queue validation results"),
                List.of(
                    "Queued poses must stay inside safe autonomous bounds.",
                    "Named commands must exist before an auto is considered runnable."),
                List.of("Select deploy auto", "Preview queue", "Quick run validation"),
                List.of(
                    "Operators see which auto is staged before enable.",
                    "Queue validation reports why an auto cannot run."),
                List.of(
                    new StateDescription(
                        "READY",
                        "A deploy auto is selected and staged.",
                        "Queue preview and step list are available to the dashboard.",
                        "At least one runnable step is present.",
                        "Entered after selecting a valid deploy auto.",
                        "Missing paths, missing named commands, or invalid poses block readiness."),
                    new StateDescription(
                        "RUNNING",
                        "The queue is actively executing commands.",
                        "Drive alignment or named commands are running in sequence.",
                        "The queue has started and still has active work.",
                        "Ends when the queue completes, is interrupted, or errors.",
                        "Drive loss or command build failures stop progress."))),
            new SubsystemDescription(
                "logging",
                "Logging And Runtime",
                "AdvantageKit logging controls, runtime profile selection, and diagnostic export.",
                List.of("Runtime profile manager", "Log rollover utility"),
                List.of(
                    "Runtime profile JSON", "Log rollover counters", "Diagnostic bundle manifests"),
                List.of(
                    "Profile edits must deserialize successfully before apply.",
                    "Log cleanup should never delete active match logs blindly."),
                List.of(
                    "Apply runtime profile",
                    "Reset runtime profile",
                    "Roll logs",
                    "Export diagnostics"),
                List.of(
                    "Operators can switch logging profiles without redeploying.",
                    "Diagnostic bundles capture the current dashboard evidence."),
                List.of(
                    new StateDescription(
                        "PROFILE_READY",
                        "Runtime profile can be applied safely.",
                        "The staged JSON is valid and the active profile is known.",
                        "Profile status is READY, SPEC_RECEIVED, APPLIED, or RESET_TO_DEFAULTS.",
                        "Transitions on apply, reset, or new staged JSON.",
                        "Invalid JSON leaves the profile in an error state."),
                    new StateDescription(
                        "LOG_MANAGEMENT",
                        "Manual log maintenance actions.",
                        "Log rollover or cleanup runs on explicit operator request.",
                        "The log utility reports completion or failure.",
                        "Used only when operators request rollover or cleanup.",
                        "Filesystem issues or unavailable log storage prevent completion.")))));
  }

  public record AssetMetadata(
      String id,
      String type,
      String displayName,
      String updatedAtIso,
      long updatedAtEpochMs,
      String buildVersion,
      String gitSha,
      String gitBranch) {}

  public record SubsystemDescriptionsDocument(
      String schemaVersion, AssetMetadata metadata, List<SubsystemDescription> subsystems) {}

  public record SubsystemDescription(
      String key,
      String name,
      String purpose,
      List<String> actuators,
      List<String> sensors,
      List<String> safetyConditions,
      List<String> commands,
      List<String> operatorOutcomes,
      List<StateDescription> states) {}

  public record StateDescription(
      String name,
      String purpose,
      String expectedMotion,
      String readyCondition,
      String transitionConditions,
      String faultBlockers) {}

  public record StorageInventoryDocument(
      String schemaVersion,
      AssetMetadata metadata,
      List<StoredAsset> assets,
      SyncWorkflow syncWorkflow) {}

  public record StoredAsset(
      String key,
      String category,
      String format,
      String authority,
      String conflictPolicy,
      String localPath,
      String robotPath,
      String deployPath,
      boolean editableFromDashboard,
      boolean includedInDeployPreserve,
      String description) {}

  public record SyncWorkflow(
      String preDeploySummary,
      String postDeploySummary,
      List<String> conflictRules,
      List<String> versionTags) {}

  public record DiagnosticBundleManifest(
      String schemaVersion,
      AssetMetadata metadata,
      String bundleId,
      String bundleDirectory,
      String overallStatus,
      List<String> files) {}

  public record DiagnosticBundleSummary(
      String schemaVersion,
      AssetMetadata metadata,
      String overallStatus,
      String summary,
      String mode,
      String actionStatus,
      String autoSummary,
      String mechanismSummary,
      String runtimeProfileStatus) {}

  public record DiagnosticObservedData(
      String systemCheck,
      String autoCheck,
      String ntDiagnostics,
      String mechanismStatus,
      String actionTrace,
      String runtimeProfile,
      String selectedAuto,
      String autoQueue,
      String subsystemDescriptions) {}
}
