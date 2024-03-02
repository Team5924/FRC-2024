// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.wrist;

import org.first5924.frc2024.subsystems.wrist.Wrist;

import java.util.function.DoubleSupplier;

import org.first5924.frc2024.subsystems.feeder.Feeder;
import org.first5924.frc2024.subsystems.shooter.Shooter;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopAimAndShoot extends Command {
  /** Creates a new SetWristAngle. */
  private final Wrist wrist;
  private final DoubleSupplier targetAngle;
  private final DoubleSupplier wristAngle;
  private final Shooter shooter;
  private final Feeder feeder;
  PIDController wristController;

  public TeleopAimAndShoot(Feeder feeder, Shooter shooter, Wrist wrist, DoubleSupplier wristAngle, DoubleSupplier targetAngle) {
    this.wrist = wrist;
    this.shooter = shooter;
    this.targetAngle = targetAngle;
    this.wristAngle = wristAngle;
    this.feeder = feeder;

    
    addRequirements(feeder, wrist, shooter);
    
    // Use addRequirements() here to declare subsystem dependencies.
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    while(wristAngle.getAsDouble() - 1 > targetAngle.getAsDouble() && targetAngle.getAsDouble() < wristAngle.getAsDouble() + 1){
      wrist.setAngle(-targetAngle.getAsDouble());
      shooter.setPercent(0.875);
    }
    feeder.setPercent(0.875);

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setVoltage(0);
    shooter.setPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return false;
  }
}
