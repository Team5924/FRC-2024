// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.intake;

import org.first5924.frc2024.constants.WristAndElevatorState;
import org.first5924.frc2024.constants.IntakeConstants.IntakeState;
import org.first5924.frc2024.subsystems.elevator.Elevator;
import org.first5924.frc2024.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetIntakeState extends InstantCommand {
  private final Intake intake;
  private final Elevator elevator;
  private final IntakeState intakeState;

  public SetIntakeState(Intake intake, Elevator elevator, IntakeState intakeState) {
    this.intake = intake;
    this.elevator = elevator;
    this.intakeState = intakeState;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setState(intakeState);
    if (intakeState == IntakeState.RETRACT) {
      elevator.setWristAndElevatorState(WristAndElevatorState.AIM_LOW);
    } else if (intakeState == IntakeState.FLOOR) {
      elevator.setWristAndElevatorState(WristAndElevatorState.INTAKE);
    }
  }
}
