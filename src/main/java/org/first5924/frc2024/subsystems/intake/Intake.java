// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.first5924.frc2024.constants.IntakeConstants.IntakeState;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  private IntakeState state = IntakeState.RETRACT;
  private IntakeState stateBeforeEject = IntakeState.RETRACT;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    SmartDashboard.putString("Intake State Before", getStateBeforeEject().toString());
    SmartDashboard.putString("Intake State", getState().toString());
  }

  public void setState(IntakeState state) {
    this.state = state;
    if (state != IntakeState.EJECT) {
      stateBeforeEject = state;
    }
  }

  public IntakeState getState() {
    return state;
  }

  public IntakeState getStateBeforeEject() {
    return stateBeforeEject;
  }

  public void setRollerPercent(double percent) {
    io.setRollerPercent(percent);
  }

  public void setPivotPosition(double degrees) {
    io.setPivotPosition(degrees);
  }

  public void setPivotVoltage(double volts) {
    io.setPivotVoltage(volts);
  }

  public boolean isReadyToEject() {
    return Math.abs(inputs.pivotAngleDegrees - IntakeState.EJECT.getPivotAngle()) < 5;
  }
}