// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.feeder;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);
    isNoteFullyIn();
  }

  public double getLaserCanMeasurementMillimeters() {
    return inputs.laserCanMeasurementMillimeters;
  }

  public void setPercent(double percent) {
    io.setPercent(percent);
  }

  public boolean isNoteFullyIn() {
    SmartDashboard.putBoolean("Is Note Fully In", inputs.laserCanMeasurementMillimeters < 40);
    return inputs.laserCanMeasurementMillimeters < 40;
  }
}