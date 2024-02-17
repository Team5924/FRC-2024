
package org.first5924.frc2024.commands.wrist;

import java.util.function.DoubleSupplier;

import org.first5924.frc2024.subsystems.wrist.Wrist;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class RotateWrist extends Command {
  /** Creates a new SetWristAngle. */
  private final Wrist wrist;
  private final DoubleSupplier mJoystick;

  public RotateWrist(Wrist wrist, DoubleSupplier joystickY) {
    this.wrist = wrist;
    mJoystick = joystickY;
    addRequirements(wrist);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setPercent(mJoystick.getAsDouble());
    System.out.println("im running!");
    SmartDashboard.putNumber("joystick", mJoystick.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // wrist.setPercent(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
