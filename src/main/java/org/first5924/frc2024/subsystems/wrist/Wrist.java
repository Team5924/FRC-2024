// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.wrist;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public Wrist(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    SmartDashboard.putNumber("Wrist angle", getAngleDegrees());
  }

  public double getAngleDegrees() {
    return inputs.wristAngleDegrees;
  }

  public void setAngle(double degrees) {
    io.setAngle(degrees);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }
}
