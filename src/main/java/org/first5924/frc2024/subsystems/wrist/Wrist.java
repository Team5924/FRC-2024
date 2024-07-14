// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.wrist;

import org.first5924.frc2024.constants.WristAndElevatorState;
import org.first5924.frc2024.constants.WristConstants;
import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  private final InterpolatingDoubleTreeMap lowAimInterpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap minWristAngleFromElevatorInterpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();
  private final InterpolatingDoubleTreeMap maxWristAngleClimbFromElevatorInterpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();

  private double targetAngle;

  private GenericEntry shuffleboardTargetAngle;

  public Wrist(WristIO io) {
    this.io = io;

    lowAimInterpolatingDoubleTreeMap.put(1.06, 54.0);
    lowAimInterpolatingDoubleTreeMap.put(1.2615, 50.5);
    lowAimInterpolatingDoubleTreeMap.put(1.503, 45.5);
    lowAimInterpolatingDoubleTreeMap.put(1.75, 41.0);
    lowAimInterpolatingDoubleTreeMap.put(2.04, 38.25);
    lowAimInterpolatingDoubleTreeMap.put(2.26, 34.85);
    lowAimInterpolatingDoubleTreeMap.put(2.495, 33.0);
    lowAimInterpolatingDoubleTreeMap.put(2.74, 29.36);
    lowAimInterpolatingDoubleTreeMap.put(3.02, 28.0);
    lowAimInterpolatingDoubleTreeMap.put(3.23, 26.0);
    lowAimInterpolatingDoubleTreeMap.put(3.56, 25.6);
    lowAimInterpolatingDoubleTreeMap.put(3.735, 25.0);
    // Guess, untested
    lowAimInterpolatingDoubleTreeMap.put(3.91, 25.6);
    lowAimInterpolatingDoubleTreeMap.put(4.09, 24.2);
    lowAimInterpolatingDoubleTreeMap.put(4.245, 21.62);
    lowAimInterpolatingDoubleTreeMap.put(4.5, 20.5);
    lowAimInterpolatingDoubleTreeMap.put(4.8, 19.8);
    lowAimInterpolatingDoubleTreeMap.put(5.0, 19.4);
    lowAimInterpolatingDoubleTreeMap.put(5.5, 18.2);

    minWristAngleFromElevatorInterpolatingDoubleTreeMap.put(0.0, 3.5);
    minWristAngleFromElevatorInterpolatingDoubleTreeMap.put(0.0965, -7.4);
    minWristAngleFromElevatorInterpolatingDoubleTreeMap.put(0.1596, -51.81);
    minWristAngleFromElevatorInterpolatingDoubleTreeMap.put(0.212, -74.004);

    maxWristAngleClimbFromElevatorInterpolatingDoubleTreeMap.put(-90.0, 0.59);
    maxWristAngleClimbFromElevatorInterpolatingDoubleTreeMap.put(3.23, 0.0);
    maxWristAngleClimbFromElevatorInterpolatingDoubleTreeMap.put(0.0, 0.0);

    shuffleboardTargetAngle = Shuffleboard.getTab("Manual PID")
      .add("Target Wrist Angle", 40)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    SmartDashboard.putNumber("Wrist Angle", getAngleDegrees());
  }

  public double getAngleDegrees() {
    return inputs.wristAngleDegrees;
  }

  public void setAngle(double degrees, double currentHeight) {
    targetAngle = MathUtil.clamp(degrees, getMinAngle(currentHeight), WristConstants.kMaxAngle);
    io.setAngle(targetAngle);
  }

  public double getMinAngle(double currentHeight) {
    return minWristAngleFromElevatorInterpolatingDoubleTreeMap.get(currentHeight);
  }

  public double getMaxAngleClimb(double currentHeight) {
    return maxWristAngleClimbFromElevatorInterpolatingDoubleTreeMap.get(currentHeight);
  }

  public double getShuffleboardAngle() {
    return shuffleboardTargetAngle.getDouble(inputs.wristAngleDegrees);
  }

  public void setVoltage(double volts) {
    io.setVoltage(volts);
  }

  public double getShootAngle(WristAndElevatorState wristAndElevatorState, double distance) {
    if (wristAndElevatorState == WristAndElevatorState.AIM_LOW) {
      return lowAimInterpolatingDoubleTreeMap.get(distance);
    }
    return 30;
  }

  public boolean isAtSetpoint() {
    return Math.abs(getAngleDegrees() - targetAngle) < 1;
  }
}
