// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.elevator;

import org.first5924.frc2024.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.Command;

public class SetHeight extends Command {
  /** Creates a new SetHeight. */
  private final Elevator elevator;
  private final double mPosition;

  public SetHeight(Elevator elevator, double position) {
    this.elevator = elevator;
    mPosition = position;
    addRequirements(elevator);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // elevator.setHeight(mPosition);
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
