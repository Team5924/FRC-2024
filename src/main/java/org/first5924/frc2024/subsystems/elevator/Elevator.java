// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.elevator;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.util.PIDConstants;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new elevator. */
  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();
  private final PIDController mPID = new PIDController(10, 0, 0);
  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
    // This method will be called once per scheduler run
  }

  public double getMotorTempCelsius() {
    return inputs.motorTempCelsius;
  } 


  public double getMotorCurrentAmps() {
    return inputs.motorCurrentAmps;
  } 


  public double getMotorCurrentVelocity() {
    return inputs.motorCurrentVelocity;
  } 


  public double getMotorPosition() {
    return inputs.motorPosition;
  } 


  public double getLastAxleEncoderPosition() {
    return inputs.lastAxleEncoderPosition;
  } 

  public double getHeight() {
    return inputs.height;
  }

  public void setPercent(double percent) {
    io.setPercent(percent);
  }

  public void setHeight(double height) {
    io.setVoltage(mPID.calculate(inputs.height, height));
  }

}
