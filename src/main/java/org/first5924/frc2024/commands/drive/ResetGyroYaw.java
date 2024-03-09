// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.drive;

import java.util.Optional;
import java.util.function.Supplier;

import org.first5924.frc2024.subsystems.drive.Drive;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetGyroYaw extends InstantCommand {
  private final Drive drive;
  private final Supplier<Optional<Alliance>> allianceSupplier;

  public ResetGyroYaw(Drive drive, Supplier<Optional<Alliance>> allianceSupplier) {
    this.drive = drive;
    this.allianceSupplier = allianceSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (allianceSupplier.get().get() == Alliance.Blue) {
      drive.setGyroYaw(0);
    } else {
      drive.setGyroYaw(180);
    }
  }
}
