// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.feeder;

import org.first5924.frc2024.subsystems.feeder.Feeder;

import edu.wpi.first.wpilibj2.command.Command;

public class FeedShooter extends Command {
  /** Creates a new slowFeeder. */
  private final Feeder feeder;

  public FeedShooter(Feeder feeder) {
    this.feeder = feeder;
    addRequirements(feeder);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //feeder.setPercent(MathUtil.applyDeadband(mJoystickY.getAsDouble(), 0.2));
    feeder.setPercent(0.7);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.setPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
