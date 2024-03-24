// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.vision;

import org.first5924.frc2024.constants.VisionConstants;
import org.first5924.lib.LimelightHelpers;

/** Add your docs here. */
public class VisionIOReal implements VisionIO {
  public VisionIOReal() {
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    LimelightHelpers.PoseEstimate botPose = LimelightHelpers.getBotPoseEstimate_wpiBlue(VisionConstants.kAprilTagLimelightName);
    inputs.botPoseRotationRadians = botPose.pose.getRotation().getRadians();
    inputs.botPoseX = botPose.pose.getX();
    inputs.botPoseY = botPose.pose.getY();
    inputs.numberFiducialsSpotted = botPose.tagCount;
    double lowestTagAmbiguity = 1;
    for (LimelightHelpers.RawFiducial rawFiducial : botPose.rawFiducials) {
      double tagAmbiguity = rawFiducial.ambiguity;
      if (tagAmbiguity < lowestTagAmbiguity) {
        lowestTagAmbiguity = tagAmbiguity;
      }
    }
    inputs.lowestTagAmbiguity = lowestTagAmbiguity;
    inputs.aprilTagPipelineLatencySeconds = LimelightHelpers.getLatency_Pipeline(VisionConstants.kAprilTagLimelightName) / 1000;
    inputs.aprilTagCaptureLatencySeconds = LimelightHelpers.getLatency_Capture(VisionConstants.kAprilTagLimelightName) / 1000;
  }
}
