// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;

import org.first5924.frc2024.subsystems.intake.Intake;


public class RunIntakeStateMachine extends Command {
  private final Intake intake;

  /** Creates a new Spin. */
  public RunIntakeStateMachine(Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setPivotPosition(intake.getState().getPivotAngle());
    intake.setRollerPercent(intake.getState().getRollerPercent());
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
