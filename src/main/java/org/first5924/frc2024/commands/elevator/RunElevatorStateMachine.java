// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.elevator;

import java.util.function.DoubleSupplier;

import org.first5924.frc2024.constants.ElevatorConstants;
import org.first5924.frc2024.constants.WristAndElevatorState;
import org.first5924.frc2024.subsystems.elevator.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;

public class RunElevatorStateMachine extends Command {
  private final Elevator elevator;
  private final DoubleSupplier rightJoystickY;

  /** Creates a new RunWristAndElevator. */
  public RunElevatorStateMachine(Elevator elevator, DoubleSupplier rightJoystickY) {
    this.elevator = elevator;
    this.rightJoystickY = rightJoystickY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(elevator.getWristAndElevatorState()) {
      case INTAKE:
        elevator.setHeight(ElevatorConstants.kIntakeHeight);
        break;
      case AMP:
        elevator.setHeight(ElevatorConstants.kAmpHeight);
        break;
      case AIM_LOW:
        elevator.setHeight(ElevatorConstants.kAimLowHeight);
        break;
      case AIM_HIGH:
        elevator.setHeight(ElevatorConstants.kAimHighHeight);
        break;
      case CLIMB:
        elevator.setVoltage(MathUtil.applyDeadband(-rightJoystickY.getAsDouble(), 0.1) * ElevatorConstants.kPeakForwardVoltage);
        break;
      case CLIMB_MAX_HEIGHT: {
        elevator.setHeight(ElevatorConstants.kMaxHeight);
        if (Math.abs(rightJoystickY.getAsDouble()) > 0.1) {
          elevator.setWristAndElevatorState(WristAndElevatorState.CLIMB);
        }
        break;
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
