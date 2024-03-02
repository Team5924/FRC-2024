// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.wrist;

import org.first5924.frc2024.constants.WristConstants;
import org.first5924.frc2024.subsystems.elevator.Elevator;
import org.first5924.frc2024.subsystems.wrist.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class RunWrist extends Command {
  private final Wrist wrist;
  private final Elevator elevator;

  /** Creates a new RunWristAndElevator. */
  public RunWrist(Wrist wrist, Elevator elevator) {
    this.wrist = wrist;
    this.elevator = elevator;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(elevator.getWristAndElevatorState()) {
      case INTAKE:
        wrist.setAngle(WristConstants.kIntakeAngle);
        break;
      case AMP:
        wrist.setAngle(WristConstants.kAmpAngle);
        break;
      case AIM_LOW:
        break;
      case AIM_HIGH:
        break;
      case CLIMB:
        wrist.setAngle(WristConstants.kClimbAngle);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setVoltage(0);
    elevator.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
