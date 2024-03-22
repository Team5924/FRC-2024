// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.shooter;

import org.first5924.frc2024.constants.ShooterConstants.ShooterState;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private ShooterState state = ShooterState.OFF;

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
  }

  public ShooterState getState() {
    return state;
  }

  public void setState(ShooterState state) {
    this.state = state;
  }

  public void setPercent(double percent) {
    io.setPercent(percent);
  }

  public boolean isUpToSpeed() {
    return inputs.upperMotorVelocityRotationsPerSecond > 83;
  }
}
