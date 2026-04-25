package org.Griffins1884.frc2026;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import org.Griffins1884.frc2026.util.LoggedTunableNumber;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants.
 */
public final class GlobalConstants {
  public static final RobotMode MODE = RobotMode.REAL;
  public static final RobotType ROBOT = RobotType.ECLAIR;
  public static final LoggingMode LOGGING_MODE = LoggingMode.DEBUG;
  public static final double ODOMETRY_FREQUENCY = 250.0;

  public static final double FIELD_WIDTH_METERS = Units.feetToMeters(26 + (5.0 / 12));
  public static final double FIELD_LENGTH_METERS = Units.feetToMeters(57 + (6.875 / 12));

  public static boolean TUNING_MODE = false;

  private GlobalConstants() {}

  public enum RobotMode {
    /** Competition robot runtime mode. */
    REAL,
    SIM,
    REPLAY,
  }

  public enum RobotType {
    ECLAIR,
    SIMBOT
  }

  public enum LoggingMode {
    DEBUG,
    COMP
  }

  public static boolean isDebugMode() {
    return LOGGING_MODE == LoggingMode.DEBUG;
  }

  public static boolean isCompMode() {
    return LOGGING_MODE == LoggingMode.COMP;
  }

  /**
   * Checks whether the correct robot is selected when deploying (the main method is only ever
   * called on deploy).
   */
  public static void main(String... args) {
    if (ROBOT == RobotType.SIMBOT) {
      new Alert("SIM robot loaded in REAL mode, gains likely breaking!", AlertType.kWarning)
          .set(true);
    }
  }

  /**
   * Minimal compatibility shim for surviving drivebase field helpers. Delete this once remaining
   * callers stop using the nested class.
   */
  public static final class FieldConstants {
    public static final double fieldWidth = FIELD_WIDTH_METERS;
    public static final double fieldLength = FIELD_LENGTH_METERS;

    public static final class Hub {
      private static final double CENTER_X_METERS = Units.inchesToMeters(181.56);
      private static final double CENTER_Z_METERS = Units.inchesToMeters(72.0);

      public static final Translation3d topCenterPoint =
          new Translation3d(CENTER_X_METERS, fieldWidth / 2.0, CENTER_Z_METERS);
      public static final Translation3d oppTopCenterPoint =
          new Translation3d(fieldLength - CENTER_X_METERS, fieldWidth / 2.0, CENTER_Z_METERS);

      private Hub() {}
    }

    private FieldConstants() {}
  }

  /** PID + FF gains, using LoggedTunableNumber for live tuning. */
  public record Gains(
      LoggedTunableNumber kP,
      LoggedTunableNumber kI,
      LoggedTunableNumber kD,
      LoggedTunableNumber kS,
      LoggedTunableNumber kV,
      LoggedTunableNumber kA,
      LoggedTunableNumber kG) {
    public Gains(String prefix, double kP, double kI, double kD) {
      this(prefix, kP, kI, kD, 0, 0, 0, 0);
    }

    public Gains(String prefix, double kP, double kI, double kD, double kS, double kV, double kA) {
      this(prefix, kP, kI, kD, kS, kV, kA, 0);
    }

    public Gains(
        String prefix,
        double kP,
        double kI,
        double kD,
        double kS,
        double kV,
        double kA,
        double kG) {
      this(
          new LoggedTunableNumber(prefix + "/kP", kP),
          new LoggedTunableNumber(prefix + "/kI", kI),
          new LoggedTunableNumber(prefix + "/kD", kD),
          new LoggedTunableNumber(prefix + "/kS", kS),
          new LoggedTunableNumber(prefix + "/kV", kV),
          new LoggedTunableNumber(prefix + "/kA", kA),
          new LoggedTunableNumber(prefix + "/kG", kG));
    }
  }
}
