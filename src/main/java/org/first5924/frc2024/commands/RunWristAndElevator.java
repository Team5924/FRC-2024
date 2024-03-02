// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands;

import java.util.function.DoubleSupplier;

import org.first5924.frc2024.constants.ElevatorConstants;
import org.first5924.frc2024.constants.WristConstants;
import org.first5924.frc2024.subsystems.elevator.Elevator;
import org.first5924.frc2024.subsystems.wrist.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class RunWristAndElevator extends Command {
  private final Wrist wrist;
  private final Elevator elevator;
  private final DoubleSupplier rightJoystickY;

  /** Creates a new RunWristAndElevator. */
  public RunWristAndElevator(Wrist wrist, Elevator elevator, DoubleSupplier rightJoystickY) {
    this.wrist = wrist;
    this.elevator = elevator;
    this.rightJoystickY = rightJoystickY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(elevator.getWristAndElevatorState()) {
      case INTAKE:
        wrist.setAngle(WristConstants.kIntakeAngle);
        elevator.setHeight(ElevatorConstants.kIntakeHeight);
      case AMP:
        wrist.setAngle(WristConstants.kAmpAngle);
        elevator.setHeight(ElevatorConstants.kAmpHeight);
      case AIM_LOW:
        elevator.setHeight(ElevatorConstants.kAimLowHeight);
      case AIM_HIGH:
        elevator.setHeight(ElevatorConstants.kAimHighHeight);
      case CLIMB:
        wrist.setAngle(WristConstants.kClimbAngle);
        elevator.setVoltage(-rightJoystickY.getAsDouble() * ElevatorConstants.kPeakForwardVoltage);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setVoltage(0);
    elevator.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
