// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.wrist;

import org.first5924.frc2024.constants.WristAndElevatorState;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private final InterpolatingDoubleTreeMap lowAimInterpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();

  public Wrist(WristIO io) {
    this.io = io;

    lowAimInterpolatingDoubleTreeMap.put(1.13, 54.0);
    lowAimInterpolatingDoubleTreeMap.put(2.0, 40.0);
    lowAimInterpolatingDoubleTreeMap.put(3.0, 30.0);
    lowAimInterpolatingDoubleTreeMap.put(4.05, 23.2);
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

  public double calculateWristAngle(WristAndElevatorState wristAndElevatorState, double distance) {
    if (wristAndElevatorState == WristAndElevatorState.AIM_LOW) {
      return lowAimInterpolatingDoubleTreeMap.get(distance);
    }
    return 0;
  }
}
