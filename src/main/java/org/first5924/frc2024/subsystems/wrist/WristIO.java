// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.wrist;


import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public double motorTempCelsius = 0.0;
    public double motorCurrentAmps = 0.0;
    public double wristAngleDegrees = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(WristIOInputs inputs) {
  }

  public default void setVoltage(double volts) {
  }

  public default void setAngle(double degrees){
  }
}