// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface VisionIO {
  @AutoLog
  public static class VisionIOInputs {
    public double botPoseRotationRadians = 0.0;
    public double botPoseX = 0.0;
    public double botPoseY = 0.0;
    public double aprilTagPipelineLatencySeconds = 0.0;
    public double aprilTagCaptureLatencySeconds = 0.0;
    public int numberFiducialsSpotted = 0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(VisionIOInputs inputs) {
  }
}
