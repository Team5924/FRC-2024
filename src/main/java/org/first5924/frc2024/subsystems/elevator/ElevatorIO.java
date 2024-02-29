// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public double leftMotorTempCelsius = 0.0;
    public double leftMotorCurrentAmps = 0.0;
    public double rightMotorTempCelsius = 0.0;
    public double rightMotorCurrentAmps = 0.0;
    public double drumPosition = 0.0;
    public double elevatorHeightMeters = 0.0;
    // LaserCAN reading accounting for the height it has at the elevator's lowest position
    public double laserCanMillimetersAboveAtLowest = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ElevatorIOInputs inputs) {
  }

  public default void setElevatorHeight(double meters) {
  }

  public default void setVoltage(double volts) {
  }

  public default void setEncoder(double drumRotations) {
  }
}