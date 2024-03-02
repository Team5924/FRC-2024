// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands;

import org.first5924.frc2024.constants.WristAndElevatorState;
import org.first5924.frc2024.subsystems.elevator.Elevator;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetWristAndElevatorState extends InstantCommand {
  private final Elevator elevator;
  private final WristAndElevatorState wristAndElevatorState;

  public SetWristAndElevatorState(Elevator elevator, WristAndElevatorState wristAndElevatorState) {
    this.elevator = elevator;
    this.wristAndElevatorState = wristAndElevatorState;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setWristAndElevatorState(wristAndElevatorState);
  }
}
