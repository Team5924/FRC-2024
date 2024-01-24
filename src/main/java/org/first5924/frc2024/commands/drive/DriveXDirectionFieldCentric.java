// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.drive;

import org.first5924.frc2024.subsystems.drive.Drive;

import edu.wpi.first.wpilibj2.command.Command;

public class DriveXDirectionFieldCentric extends Command {
  private final Drive drive;
  private final double speed;

  /** Creates a new DriveDirection. */
  public DriveXDirectionFieldCentric(Drive drive, double speed) {
    this.drive = drive;
    this.speed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    drive.drive(speed, 0, 0, true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
