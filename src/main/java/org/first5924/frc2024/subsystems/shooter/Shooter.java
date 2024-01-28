// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.shooter;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  public Shooter(ShooterIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);
    // This method will be called once per scheduler run
  }
  
  public double getUpperMotorTempCelsius() {
    return inputs.upperMotorTempCelsius;
  }

    public double getLowerMotorTempCelsius() {
    return inputs.upperMotorTempCelsius;
  }

    public double getUpperMotorCurrentAmps() {
    return inputs.upperMotorTempCelsius;
  }

    public double getLowerMotorCurrentAmps() {
    return inputs.upperMotorTempCelsius;
  }

    public double getUpperMotorCurrentVelocity() {
    return inputs.upperMotorTempCelsius;
  }

    public double getLowerMotorCurrentVelocity() {
    return inputs.upperMotorTempCelsius;
  }

  public void setPercent(double percent) {
    io.setPercent(percent);
  }


}
