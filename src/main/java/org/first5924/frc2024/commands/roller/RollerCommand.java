// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.roller;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.DoubleSupplier;

import org.first5924.frc2024.subsystems.roller.Roller;

// Command to run the roller with joystick inputs
public class RollerCommand extends Command {
  private final DoubleSupplier forward;
  // private final CANRollerSubsystem rollerSubsystem;
  private final Roller rollerSubsystem;

  public RollerCommand(
      DoubleSupplier forward, Roller rollerSubsystem) {
    this.forward = forward;
    this.rollerSubsystem = rollerSubsystem;

    addRequirements(this.rollerSubsystem);
  }

  @Override
  public void initialize() {
  }

  // Runs every cycle while the command is scheduled (~50 times per second)
  @Override
  public void execute() {
    SmartDashboard.putBoolean("cmd running", true);
    // Run the roller motor at the desired speed
    rollerSubsystem.runRoller(forward.getAsDouble());
  }

  // Runs each time the command ends via isFinished or being interrupted.
  @Override
  public void end(boolean isInterrupted) {
  }

  // Runs every cycle while the command is scheduled to check if the command is
  // finished
  @Override
  public boolean isFinished() {
    // Return false to indicate that this command never ends. It can be interrupted
    // by another command needing the same subsystem.
    return false;
  }
}
