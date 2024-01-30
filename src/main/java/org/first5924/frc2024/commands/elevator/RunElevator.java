// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.elevator;

import java.util.function.DoubleSupplier;

import org.first5924.frc2024.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class SetPercent extends Command {
  /** Creates a new SetPercent. (still need joystick y setup) */
  private final Elevator elevator;
  private final DoubleSupplier mJoystickY;
  public SetPercent(Elevator elevator, DoubleSupplier joystickY) {
    this.elevator = elevator;
    mJoystickY = joystickY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    elevator.setPercent(mJoystickY.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
