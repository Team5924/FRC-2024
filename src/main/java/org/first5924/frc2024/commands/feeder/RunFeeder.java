// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.feeder;

import org.first5924.frc2024.constants.IntakeConstants.IntakeState;
import org.first5924.frc2024.subsystems.feeder.Feeder;
import org.first5924.frc2024.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.Command;

public class RunFeeder extends Command {
  private final Feeder feeder;
  private final Intake intake;

  /** Creates a new FeederShoot. */
  public RunFeeder(Feeder feeder, Intake intake) {
    this.feeder = feeder;
    this.intake = intake;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (intake.getIntakeState() == IntakeState.FLOOR && !feeder.isNoteFullyIn()) {
      feeder.setPercent(0.5);
    } else {
      feeder.setPercent(0);
    }
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
