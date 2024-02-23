// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.feeder;

import org.first5924.frc2024.constants.FeederConstants;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
    // This method will be called once per scheduler run
  }
  
  public double getMotorTempCelsius() {
    return inputs.motorTempCelsius;
  }

    public double getMotorCurrentAmps() {
    return inputs.motorCurrentAmps;
  }

    public double motorCurrentVelocity() {
    return inputs.motorCurrentVelocity;
  }

  //public Boolean isNoteIn() {
  //    return inputs.distanceToNextObject < FeederConstants.distanceToNoteFromLaser;
   // }

  public void setPercent(double percent) {
    io.setPercent(percent);
  }
}