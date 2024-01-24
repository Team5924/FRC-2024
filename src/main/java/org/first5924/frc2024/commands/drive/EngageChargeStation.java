// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.drive;

import org.first5924.frc2024.constants.AutoConstants;
import org.first5924.frc2024.subsystems.drive.Drive;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class EngageChargeStation extends Command {
  private final Drive drive;
  private double startSettleTimestamp;
  private boolean isWaitingForSettle = false;

  /** Creates a new AutoEngageChargeStation. */
  public EngageChargeStation(Drive drive) {
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (isWaitingForSettle) {
      if (Timer.getFPGATimestamp() >= startSettleTimestamp + 0.5) {
        isWaitingForSettle = false;
      }
    } else {
      if (drive.getPitch().getDegrees() < -AutoConstants.kMaxSettlePitchDegrees) {
        drive.drive(0.2, 0, 0, false);
      } else if (drive.getPitch().getDegrees() > AutoConstants.kMaxSettlePitchDegrees) {
        drive.drive(-0.2, 0, 0, false);
      } else {
        drive.stop();
        isWaitingForSettle = true;
        startSettleTimestamp = Timer.getFPGATimestamp();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
