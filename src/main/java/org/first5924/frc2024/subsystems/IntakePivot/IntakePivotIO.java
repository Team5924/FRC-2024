// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.intakePivot;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.first5924.frc2024.commands.intakePivot.SetPercent;
import org.first5924.frc2024.subsystems.intakePivot.IntakePivotIO;
import org.littletonrobotics.junction.AutoLog;

public interface IntakePivotIO {
  /** Creates a new IntakePivotIO. */
  @AutoLog
  public static class IntakePivotIOInputs{
    public double pivotMotorEncoderPosition = 0.0;
    public double pivotMotorCurrentAmps = 0.0;
    public double pivotMotorTempCelsius = 0.0;
    public double pivotPositionAngle = 0.0;
    
  }
    //make pivot position
    public default void updateInputs(IntakePivotIOInputs inputs) {}

    public default void pivotMotorEncoderPosition(double voltage) {}

    public default void pivotMotorCurrentAmps(double amps) {}

    //public default void pivotMotorTempCelsius(double temp) {}

    public default void setPercent(double percent){}

    public default void setVoltage(double volts) {}

    public default void setBrakeMode(boolean enable) {}

    public default void getEncoderPosition(double pivotPosition) {}

    public default void setPivotPosition(double position) {}

    public default void getEncoderVelocity(double pivotVelocity) {}






  
}
