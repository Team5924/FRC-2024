// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.shooter;

import org.first5924.frc2024.constants.ShooterConstants.ShooterState;
import org.first5924.frc2024.subsystems.shooter.Shooter;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetShooterState extends InstantCommand {
  private final Shooter shooter;
  private final ShooterState state;

  public SetShooterState(Shooter shooter, ShooterState state) {
    this.shooter = shooter;
    this.state = state;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setState(state);
  }
}
