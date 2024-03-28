// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.vision;

import org.first5924.frc2024.constants.VisionConstants;
import org.first5924.lib.LimelightHelpers;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
  /** Creates a new Vision. */
  private final VisionIO io;
  private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

  private final BooleanSubscriber allianceSubscriber = NetworkTableInstance.getDefault().getTable("FMSInfo").getBooleanTopic("IsRedAlliance").subscribe(true);
  private boolean previousAllianceSubscriberValue = true;

  public Vision(VisionIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Vision", inputs);

    boolean isRedAlliance = allianceSubscriber.get();
    if (isRedAlliance != previousAllianceSubscriberValue) {
      previousAllianceSubscriberValue = isRedAlliance;
      LimelightHelpers.setPipelineIndex(
        VisionConstants.kAprilTagLimelightName,
        isRedAlliance ? 0 : 1
      );
    }
  }

  public Pose2d getBotPose2dBlue() {
    return new Pose2d(inputs.botPoseX, inputs.botPoseY, new Rotation2d(inputs.botPoseRotationRadians));
  }

  public double getLatencySeconds() {
    return inputs.aprilTagCaptureLatencySeconds + inputs.aprilTagPipelineLatencySeconds;
  }

  public int getNumberFiducialsSpotted() {
    return inputs.numberFiducialsSpotted;
  }

  public double getLowestTagAmbiguity() {
    return inputs.lowestTagAmbiguity;
  }
}
