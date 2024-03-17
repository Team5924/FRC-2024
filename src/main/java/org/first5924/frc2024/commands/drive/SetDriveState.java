// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.drive;

import org.first5924.frc2024.constants.DriveConstants.DriveState;
import org.first5924.frc2024.subsystems.drive.Drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class SetDriveState extends InstantCommand {
  private Drive drive;
  private DriveState state;

  /** Creates a new SetDriveState. */
  public SetDriveState(Drive drive, DriveState state) {
    this.state = state;
    this.drive = drive;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setState(state);
  }
}
