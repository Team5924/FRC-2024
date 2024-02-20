// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.intakePivot;

import java.util.function.DoubleSupplier;

import org.first5924.frc2024.subsystems.IntakePivot.IntakePivot;

import edu.wpi.first.wpilibj2.command.Command;

public class SetPercent extends Command {
  /** Creates a new SetVoltage. */

  private final double mJoystick;
  private final IntakePivot intakePivot;


  public SetPercent(IntakePivot intakePivot, DoubleSupplier JoystickY) {

    this.mJoystick = JoystickY.getAsDouble();
    this.intakePivot = intakePivot;
    addRequirements(intakePivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    intakePivot.setPercent(mJoystick);
    System.out.println("im executing!!!");
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
