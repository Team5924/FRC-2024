// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands;

import org.first5924.frc2024.constants.WristAndElevatorState;
import org.first5924.frc2024.constants.FeederConstants.FeederState;
import org.first5924.frc2024.constants.IntakeConstants.IntakeState;
import org.first5924.frc2024.constants.ShooterConstants.ShooterState;
import org.first5924.frc2024.subsystems.elevator.Elevator;
import org.first5924.frc2024.subsystems.feeder.Feeder;
import org.first5924.frc2024.subsystems.intake.Intake;
import org.first5924.frc2024.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

public class TeleopReset extends InstantCommand {
  private final Elevator elevator;
  private final Feeder feeder;
  private final Intake intake;
  private final Shooter shooter;

  /** Creates a new TeleopReset. */
  public TeleopReset(Elevator elevator, Feeder feeder, Intake intake, Shooter shooter) {
    this.elevator = elevator;
    this.feeder = feeder;
    this.intake = intake;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setWristAndElevatorState(WristAndElevatorState.AIM_LOW);
    feeder.setState(FeederState.MANUAL);
    intake.setState(IntakeState.RETRACT);
    shooter.setState(ShooterState.OFF);
  }
}
