// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.intake;

import org.first5924.frc2024.subsystems.intake.Intake;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import org.first5924.frc2024.subsystems.intake.IntakeIO;


// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Spin extends InstantCommand {
  private double voltage;

  public void setRollerVoltage(double voltage) {
    this.voltage = voltage;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(/* what the fuck? */);
  }

  public void initialize() {
    Intake.setRollerVoltage(voltage);
  }
}
