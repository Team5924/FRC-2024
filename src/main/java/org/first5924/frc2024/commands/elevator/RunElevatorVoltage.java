// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.elevator;

import java.util.function.DoubleSupplier;

import org.first5924.frc2024.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class RunElevatorVoltage extends Command {
  Elevator elevator;
  DoubleSupplier operatorJoystickLeftY;
  /** Creates a new RunElevatorVoltage. */
  public RunElevatorVoltage(Elevator elevator, DoubleSupplier operatorJoystickLeftY) {
    this.elevator = elevator;
    this.operatorJoystickLeftY = operatorJoystickLeftY;
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setVoltage(operatorJoystickLeftY.getAsDouble());
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
