// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.wrist;

import org.first5924.frc2024.subsystems.wrist.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class SetWristAngle extends Command {
  /** Creates a new SetWristAngle. */
  private final Wrist wrist;
  private final double Angle;

  public SetWristAngle(Wrist wrist, double angle) {
    this.wrist = wrist;
    Angle = angle;
    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setAngle(Angle);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
