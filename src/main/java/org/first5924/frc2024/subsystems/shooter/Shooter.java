// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.shooter;

import org.first5924.frc2024.constants.ShooterConstants.ShooterState;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  private ShooterState state = ShooterState.OFF;

  private GenericEntry shuffleboardTargetPercent;

  private Timer timer = new Timer();
  private boolean isCurrentlyAboveThreshold = false;

  public Shooter(ShooterIO io) {
    this.io = io;

    shuffleboardTargetPercent = Shuffleboard.getTab("Manual PID")
      .add("Target Shooter Percent", 1)
      .withWidget(BuiltInWidgets.kTextView)
      .getEntry();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Is Shooter Up to Speed?", isUpToSpeed());

    if (isCurrentlyAboveThreshold) {
      if (inputs.upperMotorVelocityRotationsPerSecond < 85) {
        isCurrentlyAboveThreshold = false;
      }
    } else {
      if (inputs.upperMotorVelocityRotationsPerSecond > 85) {
        if (timer.get() == 0) {
          timer.start();
        } else if (timer.get() > 0.2) {
          isCurrentlyAboveThreshold = true;
          timer.stop();
          timer.reset();
        }
      } else if (timer.get() > 0) {
        timer.stop();
        timer.reset();
      }
    }
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

  public boolean isUpToSpeed() {
    return inputs.upperMotorVelocityRotationsPerSecond > 83;
  }
}
