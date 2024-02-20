// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.intakePivot;

import javax.swing.text.Position;

import org.first5924.frc2024.constants.IntakePivotConstants;
import org.first5924.frc2024.subsystems.IntakePivot.IntakePivot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;


public class SetPivotPosition extends Command {
  /** Creates a new SetPivotPosition. */

  private final IntakePivot intakePivot;

  public SetPivotPosition(IntakePivot intakePivot) {
    this.intakePivot = intakePivot;
    addRequirements(intakePivot);


    // Use addRequirements
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakePivot.setPivotPosition(IntakePivotConstants.PivotPosition.FLOOR.getPivotAngle());
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
