package org.Griffins1884.frc2026.commands;

import com.pathplanner.lib.auto.NamedCommands;
import org.Griffins1884.frc2026.subsystems.swerve.SwerveSubsystem;

public final class AutoCommands {
  private AutoCommands() {}

  public static void registerAutoCommands(SwerveSubsystem drive) {
    NamedCommands.registerCommand("AlignToHP", DriveCommands.alignToHPd(drive));
    NamedCommands.registerCommand("AlignToDepot", DriveCommands.alignToDepot(drive));
  }
}
