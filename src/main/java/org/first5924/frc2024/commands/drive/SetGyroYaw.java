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
public class SetGyroYaw extends InstantCommand {
  private final Drive drive;
  private final double angle;
  private final Supplier<Optional<Alliance>> allianceSupplier;
  private final boolean enableFlipIfRed;

  public SetGyroYaw(Drive drive, double angle, Supplier<Optional<Alliance>> allianceSupplier, boolean enableFlipIfRed) {
    this.drive = drive;
    this.angle = angle;
    this.allianceSupplier = allianceSupplier;
    this.enableFlipIfRed = enableFlipIfRed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (allianceSupplier.get().get() == Alliance.Red && enableFlipIfRed) {
      double flippedYaw = 180 - angle;
      // Correcting for angle above 180 may be unnecessary, keep until tested to prove otherwise
      if (flippedYaw > 180) {
        flippedYaw -= 360;
      }
      drive.setGyroYaw(flippedYaw);
    } else {
      drive.setGyroYaw(angle);
    }
  }
}
