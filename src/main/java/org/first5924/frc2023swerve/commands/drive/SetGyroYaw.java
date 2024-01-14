// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023swerve.commands.drive;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import org.first5924.frc2023swerve.subsystems.drive.Drive;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetGyroYaw extends InstantCommand {
  private final Drive drive;
  private final double yaw;

  public SetGyroYaw(Drive drive, double yaw) {
    this.drive = drive;
    this.yaw = yaw;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drive.setGyroYaw(yaw);
  }
}
