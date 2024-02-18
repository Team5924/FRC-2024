// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.wrist;

import org.first5924.frc2024.subsystems.wrist.Wrist;

import java.util.function.DoubleSupplier;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class AutoAimWrist extends Command {
  /** Creates a new SetWristAngle. */
  private final Wrist wrist;
  private final DoubleSupplier targetAngle;
  private final DoubleSupplier wristAngle;
  PIDController wristController;

  public AutoAimWrist(Wrist wrist, DoubleSupplier wristAngle, DoubleSupplier targetAngle) {
    this.wrist = wrist;
    this.targetAngle = targetAngle;
    this.wristAngle = wristAngle;
    
    addRequirements(wrist);
    wristController = new PIDController(.1, 0, 0);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setPercent(wristController.calculate(wristAngle.getAsDouble(), targetAngle.getAsDouble()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
