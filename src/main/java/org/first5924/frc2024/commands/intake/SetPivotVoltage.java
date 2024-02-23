// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;

import org.first5924.frc2024.subsystems.intake.Intake;


public class SetPivotVoltage extends Command {
  private final Intake intake;
  private final double volts;

  /** Creates a new Spin. */
  public SetPivotVoltage(Intake intake, double volts) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.volts = volts;
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intake.setPivotVoltage(volts);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.setPivotVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
