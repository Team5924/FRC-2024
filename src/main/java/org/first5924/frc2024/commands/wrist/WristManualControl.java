package org.first5924.frc2024.commands.wrist;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import org.first5924.frc2024.subsystems.wrist.Wrist;

import edu.wpi.first.wpilibj2.command.Command;

public class WristManualControl extends Command {
  /** Creates a new SetWristAngle. */
  private final Wrist wrist;
  private final DoubleSupplier joystickY;

  public WristManualControl(Wrist wrist, DoubleSupplier joystickY) {
    this.wrist = wrist;
    this.joystickY = joystickY;
    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setVoltage(MathUtil.applyDeadband(joystickY.getAsDouble(), 0.2) * 4);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    wrist.setVoltage(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
