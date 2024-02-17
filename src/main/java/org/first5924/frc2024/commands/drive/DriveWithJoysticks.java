// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.first5924.frc2024.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.first5924.frc2024.constants.DriveConstants;
import org.first5924.frc2024.constants.InputConstants;
import org.first5924.frc2024.subsystems.drive.Drive;

public class DriveWithJoysticks extends Command {
  private final Drive drive;
  private final DoubleSupplier leftJoystickXSupplier;
  private final DoubleSupplier leftJoystickYSupplier;
  private final DoubleSupplier rightJoystickXSupplier;
  private final BooleanSupplier fieldCentricSupplier;
  private final BooleanSupplier slowMode;

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(Drive drive, DoubleSupplier leftXSupplier, DoubleSupplier leftYSupplier, DoubleSupplier rightXSupplier, BooleanSupplier fieldCentricSupplier, BooleanSupplier slowMode) {
    this.drive = drive;
    this.leftJoystickXSupplier = leftXSupplier;
    this.leftJoystickYSupplier = leftYSupplier;
    this.rightJoystickXSupplier = rightXSupplier;
    this.fieldCentricSupplier = fieldCentricSupplier;
    this.slowMode = slowMode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    // Get values from double suppliers
    double leftXJoystick = leftJoystickXSupplier.getAsDouble();
    double leftYJoystick = leftJoystickYSupplier.getAsDouble();
    double rightXJoystick = rightJoystickXSupplier.getAsDouble();

    double deadbandedLeftXJoystick = MathUtil.applyDeadband(leftXJoystick, InputConstants.kDriveDeadband);
    double deadbandedLeftYJoystick = MathUtil.applyDeadband(leftYJoystick, InputConstants.kDriveDeadband);
    double deadbandedRightXJoystick = MathUtil.applyDeadband(rightXJoystick, InputConstants.kDriveDeadband);

    // xPercent takes leftY and yPercent takes leftX because for ChassisSpeeds x is forward/backward
    // and y is left/right
    // Negative signs because y joystick up is - and because x joystick left is -
    double xPercent = -Math.copySign(deadbandedLeftYJoystick * deadbandedLeftYJoystick, deadbandedLeftYJoystick);
    double yPercent = -Math.copySign(deadbandedLeftXJoystick * deadbandedLeftXJoystick, deadbandedLeftXJoystick);
    double rotationPercent = -Math.copySign(deadbandedRightXJoystick * deadbandedRightXJoystick, deadbandedRightXJoystick) * DriveConstants.kAngularSpeedMultiplier;

    drive.drive(
      xPercent * DriveConstants.kMaxLinearSpeed,
      yPercent * DriveConstants.kMaxLinearSpeed,
      rotationPercent * DriveConstants.kMaxAngularSpeedRad,
      fieldCentricSupplier.getAsBoolean(),
      slowMode.getAsBoolean()
    );
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
