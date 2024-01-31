// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.first5924.frc2024.subsystems.intakePivot.IntakePivotIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new PivotSubsystem. */
  private final IntakeIO io;

  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
  }

  public void setRollerVoltage(double percentSpeed) {
    io.setRollerVoltage(percentSpeed);
  }

  public double getOutputCurrent() {
    return inputs.supplyCurrent;
  }

//public static void setRollerVoltage(double voltage) {
	// TODO Auto-generated method stub
	//throw new UnsupportedOperationException("Unimplemented method 'setRollerVoltage'");
//}
}