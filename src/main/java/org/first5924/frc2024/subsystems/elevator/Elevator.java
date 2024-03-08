// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.elevator;

import org.first5924.frc2024.constants.ElevatorConstants;
import org.first5924.frc2024.constants.WristAndElevatorState;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new elevator. */
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private WristAndElevatorState wristAndElevatorState = WristAndElevatorState.INTAKE;

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    SmartDashboard.putString("Wrist and Elevator State", getWristAndElevatorState().toString());

    // TODO: test if this works!
    double elevatorHeightPercent = (inputs.elevatorHeightMeters - ((double)ElevatorConstants.kLaserCanReadingAtLowestMillimeters) / 1000) / ElevatorConstants.kForwardSoftLimitThreshold;
    String elevatorHeightPercentString = String.valueOf(elevatorHeightPercent);
    SmartDashboard.putString("Elevator Height %", elevatorHeightPercentString);
  }

  public WristAndElevatorState getWristAndElevatorState() {
    return wristAndElevatorState;
  }

  public void setWristAndElevatorState(WristAndElevatorState wristAndElevatorState) {
    this.wristAndElevatorState = wristAndElevatorState;
  }

  public double getDrumRotationsFromLaserCan() {
    double drumRotationGuess = (double) (inputs.laserCanMillimetersAboveAtLowest) / 1000 / ElevatorConstants.kSpoolCircumferenceMeters;
    Logger.recordOutput("Elevator/DrumRotationGuess", drumRotationGuess);
    return drumRotationGuess;
  }

  public double getHeightMeters() {
    return inputs.elevatorHeightMeters;
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public void setHeight(double height){
    io.setElevatorHeight(height);
  }
}
