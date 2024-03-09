// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.feeder;

import org.first5924.frc2024.constants.FeederConstants;
import org.first5924.frc2024.constants.FeederConstants.FeederState;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Feeder extends SubsystemBase {
  private final FeederIO io;
  private final FeederIOInputsAutoLogged inputs = new FeederIOInputsAutoLogged();

  private FeederState state = FeederState.MANUAL;

  public Feeder(FeederIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Feeder", inputs);

    SmartDashboard.putBoolean("Note Fully In?", isNoteFullyIn());
  }

  public FeederState getState() {
    return state;
  }

  public void setState(FeederState state) {
    this.state = state;
  }

  public double getLaserCanMeasurementMillimeters() {
    return inputs.laserCanMeasurementMillimeters;
  }

  public void setPercent(double percent) {
    io.setPercent(percent);
  }

  public boolean isNoteFullyIn() {
    return inputs.laserCanMeasurementMillimeters <= FeederConstants.kDistanceWhenNoteIn;
  }
}