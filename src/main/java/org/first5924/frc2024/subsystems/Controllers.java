// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Controllers extends SubsystemBase {
  private final CommandXboxController driverController;
  private final CommandXboxController operatorController;

  private double timeToStopRumble = 0;

  /** Creates a new DriverController. */
  public Controllers(int driverPort, int operatorPort) {
    driverController = new CommandXboxController(driverPort);
    operatorController = new CommandXboxController(operatorPort);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Timer.getFPGATimestamp() < timeToStopRumble) {
      driverController.getHID().setRumble(RumbleType.kBothRumble, 1);
      operatorController.getHID().setRumble(RumbleType.kBothRumble, 1);
    } else {
      driverController.getHID().setRumble(RumbleType.kBothRumble, 0);
      operatorController.getHID().setRumble(RumbleType.kBothRumble, 0);
    }
  }

  public CommandXboxController getDriverController() {
    return driverController;
  }

  public CommandXboxController getOperatorController() {
    return operatorController;
  }

  public void rumbleForTime(double seconds) {
    double timeToStopRumbleForLastRequest = Timer.getFPGATimestamp() + seconds;
    if (timeToStopRumbleForLastRequest > timeToStopRumble) {
      timeToStopRumble = timeToStopRumbleForLastRequest;
    }
  }
}
