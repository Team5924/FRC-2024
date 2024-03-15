// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.vision;

import org.first5924.frc2024.subsystems.drive.Drive;
import org.first5924.frc2024.subsystems.vision.Vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class RunVisionPoseEstimation extends Command {
  private final Drive drive;
  private final Vision vision;

  /** Creates a new RunVisionPoseEstimation. */
  public RunVisionPoseEstimation(Drive drive, Vision vision) {
    this.drive = drive;
    this.vision = vision;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(vision);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Pose2d estimatedPose = vision.getBotPose2dBlue();
    if (estimatedPose.getX() != 0 && estimatedPose.getY() != 0 && vision.getNumberFiducialsSpotted() >= 1) {
      drive.addVisionMeasurement(estimatedPose, Timer.getFPGATimestamp() - vision.getLatencySeconds());
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
