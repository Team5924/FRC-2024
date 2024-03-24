// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.shooter;

import org.first5924.frc2024.constants.ShooterConstants.ShooterState;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private ShooterState state = ShooterState.OFF;

  private final InterpolatingDoubleTreeMap launchInterpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();

  private GenericEntry shuffleboardTargetPercent;

  public Shooter(ShooterIO io) {
    this.io = io;

    shuffleboardTargetPercent = Shuffleboard.getTab("Manual PID")
      .add("Target Shooter Percent", 1)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();

    launchInterpolatingDoubleTreeMap.put(39.0, 0.7);
    launchInterpolatingDoubleTreeMap.put(37.3, 0.65);
    launchInterpolatingDoubleTreeMap.put(34.2, 0.6);
    launchInterpolatingDoubleTreeMap.put(28.2, 0.55);
    launchInterpolatingDoubleTreeMap.put(25.1, 0.5);
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

  public double getShuffleboardPercent() {
    return shuffleboardTargetPercent.getDouble(1);
  }

  public double getLaunchPercent(double distance) {
    double distanceFeet = distance / 12;
    return launchInterpolatingDoubleTreeMap.get(distanceFeet);
  }

  public boolean isUpToSpeed() {
    return inputs.upperMotorVelocityRotationsPerSecond > 83;
  }
}
