// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;

import org.first5924.frc2024.constants.WristAndElevatorState;
import org.first5924.frc2024.subsystems.elevator.Elevator;
import org.first5924.frc2024.subsystems.shooter.Shooter;

public class RunShooterStateMachine extends Command {
  /** Creates a new ShooterOn. */
  private final Shooter shooter;
  private final Elevator elevator;

  public RunShooterStateMachine(Shooter shooter, Elevator elevator) {
    this.shooter = shooter;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (shooter.getState()) {
      case ON:
        if (elevator.getWristAndElevatorState() == WristAndElevatorState.AMP) {
          shooter.setPercent(0.3);
        } else {
          shooter.setPercent(1);
        }
        break;
      case OFF:
        shooter.setPercent(0);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
