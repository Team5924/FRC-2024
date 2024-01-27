// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.vision;

import org.first5924.dev.LimelightHelpers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  public Vision() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public final Pose2d robotPose() {
    return LimelightHelpers.getBotPose2d(getName());
  }
}
