// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.vision;

import org.first5924.frc2024.constants.VisionConstants;
import org.first5924.lib.LimelightHelpers;

import edu.wpi.first.math.geometry.Pose2d;

/** Add your docs here. */
public class VisionIOReal implements VisionIO {
  public VisionIOReal() {
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    Pose2d botPose = LimelightHelpers.getBotPose2d_wpiBlue(VisionConstants.aprilTagLimelightName);
    inputs.botPoseRotationRadians = botPose.getRotation().getRadians();
    inputs.botPoseX = botPose.getX();
    inputs.botPoseY = botPose.getY();
  }
}
