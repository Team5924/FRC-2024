// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.first5924.frc2024.commands.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.first5924.frc2024.constants.DriveConstants;
import org.first5924.frc2024.constants.InputConstants;
import org.first5924.frc2024.constants.DriveConstants.DriveState;
import org.first5924.frc2024.robot.RobotContainer;
import org.first5924.frc2024.subsystems.drive.Drive;

public class RunDriveStateMachine extends Command {
  private final Drive drive;
  private final DoubleSupplier leftJoystickXSupplier;
  private final DoubleSupplier leftJoystickYSupplier;
  private final DoubleSupplier rightJoystickXSupplier;
  private final BooleanSupplier fieldCentricSupplier;

  private final DoubleSupplier noteAngleXSupplier;
  private final DoubleSupplier noteAngleYSupplier;

  private final PIDController autoRotationPidController = new PIDController(DriveConstants.kRobotRotationKp, 0, 0);
  //PID values to be tested
  private final PIDController driveToNotePidController =  new PIDController(.12, 0, 0);
  private final PIDController rotateToNotePidController = new PIDController(.05, 0, 0);

  /** Creates a new DriveWithJoysticks. */

  public RunDriveStateMachine(Drive drive, DoubleSupplier leftXSupplier, DoubleSupplier leftYSupplier, DoubleSupplier rightXSupplier, BooleanSupplier fieldCentricSupplier, DoubleSupplier noteAngleXSupplier, DoubleSupplier noteAngleYSupplier) {
    this.drive = drive;
    this.leftJoystickXSupplier = leftXSupplier;
    this.leftJoystickYSupplier = leftYSupplier;
    this.rightJoystickXSupplier = rightXSupplier;
    this.fieldCentricSupplier = fieldCentricSupplier;
    this.noteAngleXSupplier = noteAngleXSupplier;
    this.noteAngleYSupplier = noteAngleYSupplier;
    autoRotationPidController.enableContinuousInput(-Math.PI, Math.PI);

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

    double noteAngleX = noteAngleXSupplier.getAsDouble();
    double noteAngleY = noteAngleYSupplier.getAsDouble();

    // xPercent takes leftY and yPercent takes leftX because for ChassisSpeeds x is forward/backward
    // and y is left/right
    // Negative signs because y joystick up is - and because x joystick left is -
    double xPercent = -Math.copySign(deadbandedLeftYJoystick * deadbandedLeftYJoystick, deadbandedLeftYJoystick);
    double yPercent = -Math.copySign(deadbandedLeftXJoystick * deadbandedLeftXJoystick, deadbandedLeftXJoystick);

    double omegaRadiansPerSecond;
    boolean slowMode;

    int allianceDirectionMultiplier = RobotContainer.getAlliance() == Alliance.Blue ? 1 : -1;

    switch(drive.getState()) {
      case NORMAL:
        omegaRadiansPerSecond = -Math.copySign(deadbandedRightXJoystick * deadbandedRightXJoystick, deadbandedRightXJoystick) * DriveConstants.kMaxAngularSpeedRad * DriveConstants.kNormalModeRotationMultiplier;
        slowMode = false;
        break;
      case SLOW:
        omegaRadiansPerSecond = -Math.copySign(deadbandedRightXJoystick * deadbandedRightXJoystick, deadbandedRightXJoystick) * DriveConstants.kMaxAngularSpeedRad * DriveConstants.kSlowModeRotationMultiplier;
        slowMode = true;
        break;
      case FACE_SPEAKER:
        omegaRadiansPerSecond = MathUtil.clamp(
          autoRotationPidController.calculate(
            drive.getYaw().getRadians(),
            drive.getFieldRotationRadiansToPointShooterAtSpeakerCenter(drive.getEstimatedPose())
          ) +
          drive.getRadiansPerSecondToAimWhileMoving(),
          -DriveConstants.kNormalModeRotationMultiplier * DriveConstants.kMaxAngularSpeedRad,
          DriveConstants.kNormalModeRotationMultiplier * DriveConstants.kMaxAngularSpeedRad
        );
        slowMode = false;
        break;
      case FACE_SPEAKER_AND_SLOW:
        omegaRadiansPerSecond = MathUtil.clamp(
          autoRotationPidController.calculate(
            drive.getYaw().getRadians(),
            drive.getFieldRotationRadiansToPointShooterAtSpeakerCenter(drive.getEstimatedPose())
          ) +
          drive.getRadiansPerSecondToAimWhileMoving(),
          -DriveConstants.kNormalModeRotationMultiplier * DriveConstants.kMaxAngularSpeedRad,
          DriveConstants.kNormalModeRotationMultiplier * DriveConstants.kMaxAngularSpeedRad
        );
        slowMode = true;
        break;
      case DRIVETONOTE:
        //the values inside this drive command are temporary
        slowMode = false;
        omegaRadiansPerSecond = 0;
        if(deadbandedLeftXJoystick == 0 && deadbandedLeftYJoystick == 0 && deadbandedRightXJoystick == 0){
          drive.drive(
            driveToNotePidController.calculate(noteAngleX, -1),
            0,
            rotateToNotePidController.calculate(noteAngleY, 0),
            fieldCentricSupplier.getAsBoolean(),
            false
            );
          break;
        } else {
          drive.setState(DriveState.NORMAL);
        }
        break;
      default:
        drive.setState(DriveState.NORMAL);
        slowMode = false;
        omegaRadiansPerSecond = 0;
        break;
    }

    double speedMultiplier = slowMode ? DriveConstants.kSlowModeMovementMultiplier : 1;

    if (drive.getState() != DriveState.DRIVETONOTE) {
      drive.drive(
        xPercent * DriveConstants.kMaxLinearSpeed * speedMultiplier * allianceDirectionMultiplier,
        yPercent * DriveConstants.kMaxLinearSpeed * speedMultiplier * allianceDirectionMultiplier,
        omegaRadiansPerSecond,
        fieldCentricSupplier.getAsBoolean(),
        slowMode
      );
    }
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}