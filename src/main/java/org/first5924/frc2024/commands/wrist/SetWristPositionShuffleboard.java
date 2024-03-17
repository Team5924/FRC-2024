// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.wrist;

import org.first5924.frc2024.subsystems.elevator.Elevator;
import org.first5924.frc2024.subsystems.wrist.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class SetWristPositionShuffleboard extends Command {
  private final Wrist wrist;
  private final Elevator elevator;

  /** Creates a new PIDTest. */
  public SetWristPositionShuffleboard(Wrist wrist, Elevator elevator) {
    this.wrist = wrist;
    this.elevator = elevator;
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setAngle(wrist.getShuffleboardAngle(), elevator.getHeightMeters());
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
