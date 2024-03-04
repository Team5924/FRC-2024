// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.vision.newton;

import java.util.function.DoubleSupplier;

import org.first5924.frc2024.subsystems.drive.Drive;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class TurnToSpeaker extends Command {
  Drive mDrive;
  DoubleSupplier robotYaw;
  DoubleSupplier targetYaw;
  PIDController controller = new PIDController(.12, 0, 0);

  /** Creates a new TurnToSpeaker. */
  public TurnToSpeaker(Drive mDrive, DoubleSupplier robotYaw, DoubleSupplier targetYaw) {
    this.mDrive = mDrive;
    this.robotYaw = robotYaw;
    this.targetYaw = targetYaw;
    addRequirements(mDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // mDrive.drive(0,0,controller.calculate(robotYaw.getAsDouble(), targetYaw.getAsDouble()), true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // mDrive.drive(0,0,0,true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
