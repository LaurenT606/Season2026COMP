package org.Griffins1884.frc2026.subsystems.objectivetracker;

import com.fasterxml.jackson.core.JsonProcessingException;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.sun.net.httpserver.Headers;
import com.sun.net.httpserver.HttpExchange;
import com.sun.net.httpserver.HttpHandler;
import com.sun.net.httpserver.HttpServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.io.OutputStream;
import java.net.InetSocketAddress;
import java.net.URLDecoder;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.List;
import java.util.Locale;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import org.Griffins1884.frc2026.Config;
import org.Griffins1884.frc2026.runtime.RuntimeModeManager;
import org.Griffins1884.frc2026.runtime.RuntimeProfileCodec;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;
import org.Griffins1884.frc2026.util.LogRollover;

public class OperatorBoardTracker extends SubsystemBase implements AutoCloseable {
  private static final double TELEMETRY_PUBLISH_PERIOD_SEC = 0.05;
  private static final double DISABLED_TELEMETRY_PUBLISH_PERIOD_SEC = 0.10;
  private static final ObjectMapper JSON = new ObjectMapper();

  private final OperatorBoardIO io;
  private final OperatorBoardIO.OperatorBoardIOInputs inputs =
      new OperatorBoardIO.OperatorBoardIOInputs();
  private final SwerveSubsystem drive;
  private final RebuiltAutoQueue autoQueue;
  private final DeployAutoLibrary deployAutoLibrary;
  private final OperatorBoardPersistence persistence;
  private final OperatorBoardDiagnosticBundleWriter diagnosticBundleWriter;
  private final OperatorBoardWebServer webServer;

  private String lastRequestedState = "DRIVE_ONLY";
  private String lastCurrentState = "DRIVE_ONLY";
  private boolean lastRequestAccepted = true;
  private String lastRequestReason = "";
  private String lastRuntimeProfileSpecJson =
      RuntimeProfileCodec.toJson(RuntimeModeManager.getActiveProfile());
  private String lastRuntimeProfileStatus = "READY";
  private String lastSubsystemDescriptionsJson =
      writeJson(OperatorBoardDataModels.emptyDefaultSubsystemDescriptions());
  private double lastTelemetryPublishTimestampSec = Double.NEGATIVE_INFINITY;
  private double lastQueueActionTimestampSec = Double.NEGATIVE_INFINITY;
  private ActionTraceState lastActionTraceState =
      new ActionTraceState(
          Timer.getFPGATimestamp(),
          "SYSTEM",
          "OPERATOR_BOARD_INIT",
          "pass",
          "Drive-only operator board initialized.",
          lastRequestedState,
          lastCurrentState);
  private OperatorBoardDiagnosticBundleWriter.OperatorBoardDiagnosticSnapshot
      lastDiagnosticSnapshot = null;

  public OperatorBoardTracker(OperatorBoardIO io, SwerveSubsystem drive) {
    this.io = Objects.requireNonNull(io, "io");
    this.drive = drive;
    this.persistence = new OperatorBoardPersistence();
    this.persistence.initialize();
    this.diagnosticBundleWriter = new OperatorBoardDiagnosticBundleWriter(persistence);
    this.deployAutoLibrary =
        new DeployAutoLibrary(
            Filesystem.getDeployDirectory().toPath().resolve("pathplanner").resolve("autos"));
    this.autoQueue = new RebuiltAutoQueue(new RebuiltSpotLibrary(), drive, deployAutoLibrary);
    this.webServer = maybeStartWebServer();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);

    handleUnsupportedRequests();
    handleRuntimeProfileInputs();
    handleLogRequests();

    autoQueue.handleInputs(inputs);
    autoQueue.periodic();
    autoQueue.logState();
    syncQueueActionTrace();

    if (shouldPublishTelemetry()) {
      publishTelemetry();
    }
  }

  public Command getAutonomousCommand() {
    return autoQueue.createAutonomousCommand();
  }

  public Optional<Pose2d> getQueuedStartPose() {
    return autoQueue.getQueuedStartPose();
  }

  @Override
  public void close() {
    if (webServer != null) {
      webServer.close();
    }
  }

  private void handleUnsupportedRequests() {
    if (inputs.autoStateEnableRequested) {
      rejectDashboardAction("AUTO_STATE_ENABLE", "Auto state enable removed for drive-only robot.");
    }
    if (inputs.requestedState.length > 0) {
      String requested = inputs.requestedState[inputs.requestedState.length - 1];
      if (requested != null && !requested.isBlank()) {
        lastRequestedState = requested.trim();
        rejectDashboardAction("STATE_REQUEST", "State requests removed for drive-only robot.");
      }
    }
    if (inputs.playSwerveMusicRequested || inputs.stopSwerveMusicRequested) {
      rejectDashboardAction("SWERVE_MUSIC", "Swerve music removed for champs cleanup.");
    }
    if (Double.isFinite(inputs.swerveMusicVolume)) {
      rejectDashboardAction("SWERVE_MUSIC_VOLUME", "Swerve music removed for champs cleanup.");
    }
    if (inputs.requestIntakeDeployRezero
        || inputs.cancelIntakeDeployRezero
        || inputs.requestManualIntakeDeployZeroSeek
        || inputs.cancelManualIntakeDeployZeroSeek) {
      rejectDashboardAction(
          "MECHANISM_SERVICE", "Mechanism service actions removed for drive-only robot.");
    }
  }

  private void handleRuntimeProfileInputs() {
    if (inputs.runtimeProfileSpec.length > 0) {
      lastRuntimeProfileSpecJson = inputs.runtimeProfileSpec[inputs.runtimeProfileSpec.length - 1];
      lastRuntimeProfileStatus = "SPEC_RECEIVED";
      recordActionTrace("RUNTIME", "RUNTIME_PROFILE_SPEC", true, "Runtime profile spec staged.");
    }
    if (inputs.resetRuntimeProfile) {
      RuntimeModeManager.resetToDefaults();
      lastRuntimeProfileSpecJson =
          RuntimeProfileCodec.toJson(RuntimeModeManager.getActiveProfile());
      lastRuntimeProfileStatus = "RESET_TO_DEFAULTS";
      lastRequestAccepted = true;
      lastRequestReason = "";
      recordActionTrace(
          "RUNTIME", "RUNTIME_PROFILE_RESET", true, "Runtime profile reset to defaults.");
    }
    if (inputs.applyRuntimeProfile) {
      try {
        RuntimeModeManager.setActiveProfile(
            RuntimeProfileCodec.fromJson(lastRuntimeProfileSpecJson));
        lastRuntimeProfileSpecJson =
            RuntimeProfileCodec.toJson(RuntimeModeManager.getActiveProfile());
        lastRuntimeProfileStatus = "APPLIED";
        lastRequestAccepted = true;
        lastRequestReason = "";
        recordActionTrace("RUNTIME", "RUNTIME_PROFILE_APPLY", true, "Runtime profile applied.");
      } catch (Exception ex) {
        lastRuntimeProfileStatus = "INVALID_SPEC: " + ex.getMessage();
        lastRequestAccepted = false;
        lastRequestReason = "Runtime profile invalid";
        recordActionTrace("RUNTIME", "RUNTIME_PROFILE_APPLY", false, lastRequestReason);
      }
    }
  }

  private void handleLogRequests() {
    if (inputs.rollLogsRequested) {
      boolean rolled = LogRollover.roll();
      lastRequestAccepted = rolled;
      lastRequestReason = rolled ? "" : "Log rollover unavailable";
      recordActionTrace(
          "UTILITY", "ROLL_LOGS", rolled, rolled ? "Log rollover completed." : lastRequestReason);
    }
    if (inputs.cleanLogsRequested) {
      boolean cleaned = LogRollover.cleanLogsFolder();
      lastRequestAccepted = cleaned;
      lastRequestReason = cleaned ? "" : "Log cleanup unavailable";
      recordActionTrace(
          "UTILITY", "CLEAN_LOGS", cleaned, cleaned ? "Log cleanup completed." : lastRequestReason);
    }
  }

  private void rejectDashboardAction(String action, String reason) {
    lastRequestAccepted = false;
    lastRequestReason = reason;
    recordActionTrace("DASHBOARD", action, false, reason);
  }

  private boolean shouldPublishTelemetry() {
    double now = Timer.getFPGATimestamp();
    double publishPeriod =
        DriverStation.isDisabled()
            ? DISABLED_TELEMETRY_PUBLISH_PERIOD_SEC
            : TELEMETRY_PUBLISH_PERIOD_SEC;
    if (now - lastTelemetryPublishTimestampSec < publishPeriod) {
      return false;
    }
    lastTelemetryPublishTimestampSec = now;
    return true;
  }

  private void publishTelemetry() {
    lastCurrentState = "DRIVE_ONLY";
    String autoQueueStateJson = autoQueue.getQueueStateJson();
    String selectedAutoStateJson = autoQueue.getSelectedAutoStateJson();
    Optional<Pose2d> queuePreviewPose = autoQueue.getPreviewPose();
    double[] previewPoseArray =
        queuePreviewPose.map(OperatorBoardTracker::toPoseArray).orElseGet(() -> new double[] {});
    double[] robotPoseArray = computeRobotPose();
    String runtimeProfileStateJson =
        RuntimeProfileCodec.toJson(RuntimeModeManager.getActiveProfile());
    String systemCheckStateJson = buildSystemCheckStateJson();
    String autoCheckStateJson = buildAutoCheckStateJson(queuePreviewPose);
    String autoQuickRunStateJson = autoQueue.getQuickRunStateJson();
    String mechanismStatusStateJson = buildMechanismStatusStateJson();
    String actionTraceStateJson = writeJson(lastActionTraceState);
    String ntDiagnosticsStateJson = buildNtDiagnosticsStateJson(autoQueueStateJson);

    io.setRequestedState(lastRequestedState);
    io.setCurrentState(lastCurrentState);
    io.setRequestAccepted(lastRequestAccepted);
    io.setRequestReason(lastRequestReason);
    io.setTargetType(queuePreviewPose.isPresent() ? "AUTO_PREVIEW" : "NONE");
    io.setTargetPose(previewPoseArray);
    io.setTargetPoseValid(queuePreviewPose.isPresent());
    io.setRobotPose(robotPoseArray);
    io.setAutoQueueState(autoQueueStateJson);
    io.setAutoQueuePreviewPose(previewPoseArray);
    io.setAutoQueuePreviewPoseValid(queuePreviewPose.isPresent());
    io.setSelectedAutoState(selectedAutoStateJson);
    io.setRuntimeProfileState(runtimeProfileStateJson);
    io.setRuntimeProfileStatus(lastRuntimeProfileStatus);
    io.setSystemCheckState(systemCheckStateJson);
    io.setAutoCheckState(autoCheckStateJson);
    io.setAutoQuickRunState(autoQuickRunStateJson);
    io.setMechanismStatusState(mechanismStatusStateJson);
    io.setActionTraceState(actionTraceStateJson);
    io.setNtDiagnosticsState(ntDiagnosticsStateJson);

    io.setHasBall(false);
    io.setDsMode(getDsMode());
    io.setBatteryVoltage(RobotController.getBatteryVoltage());
    io.setBrownout(RobotController.isBrownedOut());
    io.setAlliance(getAlliance());
    io.setMatchTime(DriverStation.getMatchTime());
    io.setHubTimeframe("UNAVAILABLE");
    io.setHubStatusValid(false);
    io.setRedHubStatus("UNAVAILABLE");
    io.setBlueHubStatus("UNAVAILABLE");
    io.setOurHubStatus("UNAVAILABLE");
    io.setOurHubActive(false);
    io.setAutoWinnerAlliance("UNKNOWN");
    io.setGameDataRaw("");
    io.setHubRecommendation("Drivebase-only robot");
    io.setTurretAtSetpoint(false);
    io.setTurretMode("REMOVED");
    io.setVisionStatus("REMOVED");
    io.setVisionPoseVisible(false);
    io.setShootEnabled(false);
    io.setIntakeRollersHeld(false);
    io.setIntakeDeployed(false);
    io.setTeleopOverrideActive(false);
    io.setDriverControllerControlActive(false);
    io.setShootReadyLatched(false);
    io.setIntakeDeployRezeroInProgress(false);
    io.setManualIntakeDeployZeroSeekInProgress(false);
    io.setSysIdDrivePhase(drive != null ? drive.getDriveSysIdPhase() : "UNAVAILABLE");
    io.setSysIdDriveActive(drive != null && drive.isDriveSysIdActive());
    io.setSysIdDriveLastCompleted(drive != null ? drive.getDriveSysIdLastCompleted() : Double.NaN);
    io.setSysIdDriveLastCompletedPhase(
        drive != null ? drive.getDriveSysIdLastCompletedPhase() : "UNAVAILABLE");
    io.setSysIdTurnPhase(drive != null ? drive.getTurnSysIdPhase() : "UNAVAILABLE");
    io.setSysIdTurnActive(drive != null && drive.isTurnSysIdActive());
    io.setSysIdTurnLastCompleted(drive != null ? drive.getTurnSysIdLastCompleted() : Double.NaN);
    io.setSysIdTurnLastCompletedPhase(
        drive != null ? drive.getTurnSysIdLastCompletedPhase() : "UNAVAILABLE");
    io.setLogRollStatus(LogRollover.getStatus());
    io.setLogRollLastTimestamp(LogRollover.getLastRollTimestampSec());
    io.setLogRollCount(LogRollover.getRollCount());
    io.setLogCleanStatus(LogRollover.getCleanStatus());
    io.setLogCleanLastTimestamp(LogRollover.getLastCleanTimestampSec());
    io.setLogCleanCount(LogRollover.getCleanCount());
    io.setLogCleanDeletedEntries(LogRollover.getLastCleanDeletedEntries());

    lastDiagnosticSnapshot =
        new OperatorBoardDiagnosticBundleWriter.OperatorBoardDiagnosticSnapshot(
            extractSummaryStatus(systemCheckStateJson),
            extractSummaryText(systemCheckStateJson),
            extractSummaryText(autoCheckStateJson),
            extractSummaryText(mechanismStatusStateJson),
            lastRuntimeProfileStatus,
            lastActionTraceState.status(),
            getDsMode(),
            systemCheckStateJson,
            autoCheckStateJson,
            ntDiagnosticsStateJson,
            mechanismStatusStateJson,
            actionTraceStateJson,
            runtimeProfileStateJson,
            selectedAutoStateJson,
            autoQueueStateJson,
            lastSubsystemDescriptionsJson);
  }

  private String buildSystemCheckStateJson() {
    double speedMetersPerSecond = computeDriveSpeedMetersPerSecond();
    List<CheckItem> items =
        List.of(
            new CheckItem(
                "Drive availability",
                drive != null ? "pass" : "fail",
                drive != null ? "Swerve subsystem available." : "Swerve subsystem unavailable."),
            new CheckItem(
                "Robot pose",
                hasValidRobotPose() ? "pass" : "warn",
                hasValidRobotPose() ? "Odometry pose is valid." : "Odometry pose unavailable."),
            new CheckItem(
                "Battery",
                batteryStatus(),
                String.format(Locale.ROOT, "%.2fV", RobotController.getBatteryVoltage())),
            new CheckItem(
                "Drive speed",
                "pass",
                String.format(Locale.ROOT, "%.2f m/s", speedMetersPerSecond)),
            new CheckItem(
                "Auto selection",
                autoQueue.getQueueLength() > 0 ? "pass" : "warn",
                autoQueue.getQueueLength() > 0
                    ? autoQueue.getQueueLength() + " queued step(s) staged."
                    : "No auto selected."));
    return writeJson(buildCheckReport("system", items));
  }

  private String buildAutoCheckStateJson(Optional<Pose2d> queuePreviewPose) {
    List<CheckItem> items =
        List.of(
            new CheckItem(
                "Drive availability",
                drive != null ? "pass" : "fail",
                drive != null ? "Drive subsystem available." : "Drive subsystem unavailable."),
            new CheckItem(
                "Queued start pose",
                queuePreviewPose.isPresent() ? "pass" : "fail",
                queuePreviewPose
                    .map(OperatorBoardTracker::formatPose)
                    .orElse("Preview pose unavailable.")),
            new CheckItem(
                "Queue length",
                autoQueue.getQueueLength() > 0 ? "pass" : "fail",
                autoQueue.getQueueLength() + " queued step(s)."),
            new CheckItem(
                "Driver Station mode",
                DriverStation.isAutonomous() || DriverStation.isDisabled() ? "pass" : "warn",
                "Current mode: " + getDsMode()));
    return writeJson(buildCheckReport("auto", items));
  }

  private String buildMechanismStatusStateJson() {
    String controlMode =
        DriverStation.isAutonomous() ? "AUTO" : DriverStation.isTeleop() ? "TELEOP" : "IDLE";
    MechanismItem driveItem =
        new MechanismItem(
            "Swerve Drive",
            drive != null ? "NOMINAL" : "OFFLINE",
            drive != null,
            true,
            controlMode,
            String.format(Locale.ROOT, "%.2f m/s", computeDriveSpeedMetersPerSecond()));
    return writeJson(
        new MechanismStatusState(
            Timer.getFPGATimestamp(),
            drive != null ? "Drive-only robot online." : "Drive subsystem unavailable.",
            List.of(driveItem)));
  }

  private String buildNtDiagnosticsStateJson(String autoQueueStateJson) {
    var canStatus = RobotController.getCANStatus();
    return writeJson(
        new NtDiagnosticsState(
            NetworkTableInstance.getDefault().getConnections().length,
            null,
            (double) autoQueueStateJson.getBytes(StandardCharsets.UTF_8).length,
            DriverStation.isDisabled()
                ? DISABLED_TELEMETRY_PUBLISH_PERIOD_SEC
                : TELEMETRY_PUBLISH_PERIOD_SEC,
            canStatus.percentBusUtilization,
            canStatus.txFullCount,
            canStatus.receiveErrorCount,
            canStatus.transmitErrorCount,
            0.0));
  }

  private CheckReport buildCheckReport(String mode, List<CheckItem> items) {
    long failCount = items.stream().filter(item -> "fail".equals(item.status())).count();
    long warnCount = items.stream().filter(item -> "warn".equals(item.status())).count();
    String status = failCount > 0 ? "fail" : warnCount > 0 ? "warn" : "pass";
    String summary =
        status.equals("fail")
            ? failCount + " fail, " + warnCount + " warn"
            : status.equals("warn")
                ? warnCount + " warning" + (warnCount == 1 ? "" : "s")
                : "READY";
    return new CheckReport(Timer.getFPGATimestamp(), mode, status, summary, items);
  }

  private boolean hasValidRobotPose() {
    if (drive == null) {
      return false;
    }
    Pose2d pose = drive.getPose();
    return pose != null
        && Double.isFinite(pose.getX())
        && Double.isFinite(pose.getY())
        && Double.isFinite(pose.getRotation().getRadians());
  }

  private double[] computeRobotPose() {
    if (!hasValidRobotPose()) {
      return new double[] {};
    }
    return toPoseArray(drive.getPose());
  }

  private double computeDriveSpeedMetersPerSecond() {
    if (drive == null) {
      return Double.NaN;
    }
    ChassisSpeeds speeds = drive.getRobotRelativeSpeeds();
    return Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond);
  }

  private String batteryStatus() {
    double battery = RobotController.getBatteryVoltage();
    if (!Double.isFinite(battery)) {
      return "warn";
    }
    if (battery < 11.2) {
      return "fail";
    }
    if (battery < 12.0) {
      return "warn";
    }
    return "pass";
  }

  private void syncQueueActionTrace() {
    RebuiltAutoQueue.QueueActionTrace queueActionTrace = autoQueue.getLastActionTrace();
    if (queueActionTrace == null
        || queueActionTrace.timestampSec() <= lastQueueActionTimestampSec) {
      return;
    }
    lastQueueActionTimestampSec = queueActionTrace.timestampSec();
    lastActionTraceState =
        new ActionTraceState(
            queueActionTrace.timestampSec(),
            "QUEUE",
            queueActionTrace.action(),
            queueActionTrace.accepted() ? "pass" : "fail",
            queueActionTrace.detail(),
            lastRequestedState,
            lastCurrentState);
  }

  private void recordActionTrace(String source, String action, boolean accepted, String detail) {
    lastActionTraceState =
        new ActionTraceState(
            Timer.getFPGATimestamp(),
            source,
            action,
            accepted ? "pass" : "fail",
            detail == null ? "" : detail,
            lastRequestedState,
            lastCurrentState);
  }

  private OperatorBoardWebServer maybeStartWebServer() {
    if (!Config.WebUIConfig.ENABLED) {
      return null;
    }
    try {
      OperatorBoardWebServer server =
          new OperatorBoardWebServer(
              Filesystem.getDeployDirectory().toPath().resolve("operatorboard"),
              Config.WebUIConfig.BIND_ADDRESS,
              Config.WebUIConfig.PORT);
      server.start();
      return server;
    } catch (IOException ex) {
      DriverStation.reportError("Failed to start operator board web server", ex.getStackTrace());
      return null;
    }
  }

  private String extractSummaryStatus(String json) {
    return extractJsonField(json, "status", "unknown");
  }

  private String extractSummaryText(String json) {
    return extractJsonField(json, "summary", "UNKNOWN");
  }

  private static String extractJsonField(String json, String field, String fallback) {
    if (json == null || json.isBlank()) {
      return fallback;
    }
    try {
      var node = JSON.readTree(json);
      return node != null && node.hasNonNull(field) ? node.get(field).asText(fallback) : fallback;
    } catch (IOException ex) {
      return fallback;
    }
  }

  private static String getDsMode() {
    if (DriverStation.isDisabled()) {
      return "DISABLED";
    }
    if (DriverStation.isAutonomous()) {
      return "AUTO";
    }
    if (DriverStation.isTeleop()) {
      return "TELEOP";
    }
    if (DriverStation.isTest()) {
      return "TEST";
    }
    return "UNKNOWN";
  }

  private static String getAlliance() {
    Optional<DriverStation.Alliance> alliance = DriverStation.getAlliance();
    if (alliance.isEmpty()) {
      return "UNKNOWN";
    }
    return alliance.get() == DriverStation.Alliance.Red ? "RED" : "BLUE";
  }

  private static double[] toPoseArray(Pose2d pose) {
    return new double[] {pose.getX(), pose.getY(), pose.getRotation().getRadians()};
  }

  private static String formatPose(Pose2d pose) {
    return String.format(
        Locale.ROOT,
        "x=%.2f, y=%.2f, h=%.1f deg",
        pose.getX(),
        pose.getY(),
        pose.getRotation().getDegrees());
  }

  private static String writeJson(Object value) {
    try {
      return JSON.writeValueAsString(value);
    } catch (JsonProcessingException ex) {
      return "{}";
    }
  }

  private final class OperatorBoardWebServer implements AutoCloseable {
    private final Path contentRoot;
    private final HttpServer server;
    private final ExecutorService executor = Executors.newCachedThreadPool();

    private OperatorBoardWebServer(Path contentRoot, String bindAddress, int port)
        throws IOException {
      this.contentRoot = contentRoot.toAbsolutePath().normalize();
      this.server = HttpServer.create(new InetSocketAddress(bindAddress, port), 0);
      this.server.setExecutor(executor);
      this.server.createContext(
          "/planner-autos/index.json", exchange -> handlePlannerManifest(exchange));
      this.server.createContext("/planner-autos/", exchange -> handlePlannerPreview(exchange));
      this.server.createContext(
          "/api/subsystem-descriptions", exchange -> handleSubsystemDescriptions(exchange));
      this.server.createContext(
          "/api/storage/inventory", exchange -> handleStorageInventory(exchange));
      this.server.createContext(
          "/api/diagnostics/latest", exchange -> handleLatestDiagnostic(exchange));
      this.server.createContext(
          "/api/diagnostics/export", exchange -> handleExportDiagnostic(exchange));
      this.server.createContext(
          "/music-upload", exchange -> sendText(exchange, 410, "Music upload removed."));
      this.server.createContext("/", new StaticFileHandler());
    }

    private void start() {
      server.start();
    }

    @Override
    public void close() {
      server.stop(0);
      executor.shutdownNow();
    }

    private void handlePlannerManifest(HttpExchange exchange) throws IOException {
      if (!"GET".equalsIgnoreCase(exchange.getRequestMethod())) {
        sendText(exchange, 405, "Method not allowed");
        return;
      }
      sendJson(exchange, 200, deployAutoLibrary.buildManifestJson());
    }

    private void handlePlannerPreview(HttpExchange exchange) throws IOException {
      if (!"GET".equalsIgnoreCase(exchange.getRequestMethod())) {
        sendText(exchange, 405, "Method not allowed");
        return;
      }
      String rawPath = exchange.getRequestURI().getPath();
      String relativePath =
          URLDecoder.decode(rawPath.substring("/planner-autos/".length()), StandardCharsets.UTF_8);
      if (relativePath.isBlank() || relativePath.contains("..")) {
        sendText(exchange, 400, "Invalid auto path");
        return;
      }
      Optional<String> previewJson = deployAutoLibrary.loadAutoPreviewJson(relativePath);
      if (previewJson.isEmpty()) {
        sendText(exchange, 404, "Auto preview unavailable");
        return;
      }
      sendJson(exchange, 200, previewJson.get());
    }

    private void handleSubsystemDescriptions(HttpExchange exchange) throws IOException {
      if (!"GET".equalsIgnoreCase(exchange.getRequestMethod())) {
        sendText(exchange, 405, "Method not allowed");
        return;
      }
      sendJson(exchange, 200, lastSubsystemDescriptionsJson);
    }

    private void handleStorageInventory(HttpExchange exchange) throws IOException {
      if (!"GET".equalsIgnoreCase(exchange.getRequestMethod())) {
        sendText(exchange, 405, "Method not allowed");
        return;
      }
      sendJson(exchange, 200, writeJson(persistence.buildStorageInventory()));
    }

    private void handleLatestDiagnostic(HttpExchange exchange) throws IOException {
      if (!"GET".equalsIgnoreCase(exchange.getRequestMethod())) {
        sendText(exchange, 405, "Method not allowed");
        return;
      }
      Optional<OperatorBoardDataModels.DiagnosticBundleManifest> manifest =
          persistence.readLatestDiagnosticManifest();
      if (manifest.isEmpty()) {
        sendText(exchange, 404, "No diagnostic bundle");
        return;
      }
      sendJson(exchange, 200, writeJson(manifest.get()));
    }

    private void handleExportDiagnostic(HttpExchange exchange) throws IOException {
      if (!"POST".equalsIgnoreCase(exchange.getRequestMethod())) {
        sendText(exchange, 405, "Method not allowed");
        return;
      }
      if (lastDiagnosticSnapshot == null) {
        sendText(exchange, 503, "Diagnostic snapshot not ready");
        return;
      }
      OperatorBoardDataModels.DiagnosticBundleManifest manifest =
          diagnosticBundleWriter.forceWrite(lastDiagnosticSnapshot);
      sendJson(exchange, 200, writeJson(manifest));
    }

    private final class StaticFileHandler implements HttpHandler {
      @Override
      public void handle(HttpExchange exchange) throws IOException {
        if (!"GET".equalsIgnoreCase(exchange.getRequestMethod())) {
          sendText(exchange, 405, "Method not allowed");
          return;
        }
        String requestPath = exchange.getRequestURI().getPath();
        String relativePath = requestPath.equals("/") ? "index.html" : requestPath.substring(1);
        Path target = contentRoot.resolve(relativePath).normalize();
        if (!target.startsWith(contentRoot) || !Files.isRegularFile(target)) {
          sendText(exchange, 404, "Not found");
          return;
        }
        byte[] body = Files.readAllBytes(target);
        Headers headers = exchange.getResponseHeaders();
        headers.set("Content-Type", contentType(target));
        headers.set("Cache-Control", "no-store");
        exchange.sendResponseHeaders(200, body.length);
        try (OutputStream os = exchange.getResponseBody()) {
          os.write(body);
        }
      }
    }

    private String contentType(Path file) throws IOException {
      String detected = Files.probeContentType(file);
      if (detected != null) {
        return detected;
      }
      String name = file.getFileName().toString().toLowerCase(Locale.ROOT);
      if (name.endsWith(".js")) {
        return "application/javascript";
      }
      if (name.endsWith(".css")) {
        return "text/css";
      }
      if (name.endsWith(".json")) {
        return "application/json";
      }
      if (name.endsWith(".png")) {
        return "image/png";
      }
      if (name.endsWith(".html")) {
        return "text/html; charset=utf-8";
      }
      return "application/octet-stream";
    }
  }

  private static void sendJson(HttpExchange exchange, int statusCode, String body)
      throws IOException {
    byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
    exchange.getResponseHeaders().set("Content-Type", "application/json; charset=utf-8");
    exchange.getResponseHeaders().set("Cache-Control", "no-store");
    exchange.sendResponseHeaders(statusCode, bytes.length);
    try (OutputStream os = exchange.getResponseBody()) {
      os.write(bytes);
    }
  }

  private static void sendText(HttpExchange exchange, int statusCode, String body)
      throws IOException {
    byte[] bytes = body.getBytes(StandardCharsets.UTF_8);
    exchange.getResponseHeaders().set("Content-Type", "text/plain; charset=utf-8");
    exchange.getResponseHeaders().set("Cache-Control", "no-store");
    exchange.sendResponseHeaders(statusCode, bytes.length);
    try (OutputStream os = exchange.getResponseBody()) {
      os.write(bytes);
    }
  }

  private record CheckItem(String label, String status, String detail) {}

  private record CheckReport(
      double timestampSec, String mode, String status, String summary, List<CheckItem> items) {}

  private record MechanismItem(
      String label,
      String health,
      boolean connected,
      boolean atGoal,
      String controlMode,
      String detail) {}

  private record MechanismStatusState(
      double timestampSec, String summary, List<MechanismItem> items) {}

  private record NtDiagnosticsState(
      Integer connectionCount,
      Integer topicCount,
      Double queueStateBytes,
      Double publishPeriodSec,
      Double canBusUtilization,
      Integer canTxFullCount,
      Integer canReceiveErrorCount,
      Integer canTransmitErrorCount,
      Double sampledPublishBytesPerSec) {}

  private record ActionTraceState(
      double timestampSec,
      String source,
      String action,
      String status,
      String detail,
      String requestedState,
      String currentState) {}
}
