// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double upperMotorTempCelsius = 0.0;
    public double lowerMotorTempCelsius = 0.0;
    public double upperMotorCurrentAmps = 0.0;
    public double lowerMotorCurrentAmps = 0.0;
    public double upperMotorAppliedVolts = 0.0;
    public double lowerMotorAppliedVolts = 0.0;
    public double upperMotorVelocityRotationsPerSecond = 0.0;
    public double lowerMotorVelocityRotationsPerSecond = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {
  }

  public default void setPercent(double percent) {
  }
}
