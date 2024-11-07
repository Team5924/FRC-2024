// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double rollerMotorTempCelsius = 0.0;
    public double rollerMotorCurrentAmps = 0.0;
    public double rollerMotorAppliedVolts = 0.0;
    public double pivotAngleDegrees = 0.0;
    public double pivotMotorTempCelsius = 0.0;
    public double pivotMotorCurrentAmps = 0.0;
    public double pivotMotorAppliedVolts = 0.0;
    public double laserCanMeasurementMillimeters = 0;
  }

  /** Updates the set of loggable inputs */
  public default void updateInputs(IntakeIOInputs inputs) {}

  public default void setRollerPercent(double percent) {}

  public default void setPivotPosition(double degrees) {}

  public default void setPivotVoltage(double volts) {}
}