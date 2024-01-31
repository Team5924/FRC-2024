// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double supplyCurrent = 0.0;
    public double rollerMotorTempCelsius = 0.0;
    public double rollerMotorCurrentAmps = 0.0;
  }

  /** Updates the set of loggable inputs */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollerVoltage(double voltage) {}
}