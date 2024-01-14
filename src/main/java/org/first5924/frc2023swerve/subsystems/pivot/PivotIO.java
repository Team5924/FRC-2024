// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023swerve.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public double pivotPositionDegrees = 0.0;
    public double pivotVelocityDegreesPerSecond = 0.0;
    public double supplyCurrent = 0.0;
  }

  /** Updates the set of loggable inputs */
  public default void updateInputs(PivotIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void setEncoderPosition(double position) {}

  public default void setBrakeMode(boolean enable) {}
}
