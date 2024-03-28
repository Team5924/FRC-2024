// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.feeder;

import java.util.function.DoubleSupplier;

import org.first5924.frc2024.subsystems.DriverController;
import org.first5924.frc2024.subsystems.drive.Drive;
import org.first5924.frc2024.subsystems.elevator.Elevator;
import org.first5924.frc2024.constants.FeederConstants;
import org.first5924.frc2024.constants.IntakeConstants;
import org.first5924.frc2024.constants.WristAndElevatorState;
import org.first5924.frc2024.constants.DriveConstants.DriveState;
import org.first5924.frc2024.constants.FeederConstants.FeederState;
import org.first5924.frc2024.constants.IntakeConstants.IntakeState;
import org.first5924.frc2024.subsystems.feeder.Feeder;
import org.first5924.frc2024.subsystems.intake.Intake;
import org.first5924.frc2024.subsystems.shooter.Shooter;
import org.first5924.frc2024.subsystems.wrist.Wrist;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class RunFeederStateMachine extends Command {
  private final Feeder feeder;
  private final Intake intake;
  private final Drive drive;
  private final Shooter shooter;
  private final Elevator elevator;
  private final Wrist wrist;
  private final DoubleSupplier leftJoystickY;
  private final DriverController rumbleDriverController;

  private final Timer timer = new Timer();

  /** Creates a new FeederShoot. */
  public RunFeederStateMachine(Feeder feeder, Intake intake, Drive drive, Shooter shooter, Elevator elevator, Wrist wrist, DoubleSupplier leftJoystickY, DriverController rumbleDriverController) {
    this.feeder = feeder;
    this.intake = intake;
    this.drive = drive;
    this.shooter = shooter;
    this.elevator = elevator;
    this.wrist = wrist;
    this.leftJoystickY = leftJoystickY;
    this.rumbleDriverController = rumbleDriverController;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Logger.recordOutput("Feeder State", feeder.getState().toString());

    if (intake.isNoteIn() && !feeder.getIsNoteInRobotSystem() && !DriverStation.isAutonomous()) {
      rumbleDriverController.rumbleForTime(0.6);
      feeder.setIsNoteInRobotSystem(true);
    }

    switch (feeder.getState()) {
      case MANUAL:
        if (drive.isFacingSpeaker() &&
            drive.isStoppedToShoot() &&
            shooter.isUpToSpeed() &&
            (elevator.getWristAndElevatorState() == WristAndElevatorState.AIM_LOW || elevator.getWristAndElevatorState() == WristAndElevatorState.AIM_HIGH) &&
            wrist.isAtSetpoint()) {
          feeder.setState(FeederState.WAITING_TO_SHOOT);
        } else {
          feeder.setPercent(-MathUtil.applyDeadband(leftJoystickY.getAsDouble(), 0.2));
        }
        break;
      case WAITING_TO_SHOOT:
        feeder.setPercent(0);
        if (!(drive.isFacingSpeaker() &&
            drive.isStoppedToShoot() &&
            shooter.isUpToSpeed() &&
            (elevator.getWristAndElevatorState() == WristAndElevatorState.AIM_LOW || elevator.getWristAndElevatorState() == WristAndElevatorState.AIM_HIGH) &&
            wrist.isAtSetpoint())) {
          timer.stop();
          timer.reset();
          feeder.setState(FeederState.MANUAL);
        } else if (timer.get() == 0) {
          timer.start();
        } else if (timer.get() >= 0.25 && (drive.getState() == DriveState.FACE_SPEAKER || drive.getState() == DriveState.FACE_SPEAKER_AND_SLOW)) {
          timer.stop();
          timer.reset();
          feeder.setState(FeederState.FEED_SHOOTER);
        }
        break;
      // Stop x seconds after note detected or y seconds after exiting intake mode. y > x
      case INTAKE:
        if (feeder.isNoteFullyIn()) {
          timer.stop();
          timer.reset();
          feeder.setState(FeederState.ALIGN);
        } else if (timer.get() >= FeederConstants.kTimeInRetractToDisable) {
          timer.stop();
          timer.reset();
          feeder.setState(FeederState.MANUAL);
        } else if (timer.get() == 0 && intake.getState() != IntakeState.FLOOR && intake.getState() != IntakeState.FEEDER) {
          timer.start();
          feeder.setPercent(IntakeConstants.kFloorRollerPercent + 0.05);
        } else {
          feeder.setPercent(IntakeConstants.kFloorRollerPercent + 0.05);
        }
        break;
      case ALIGN:
        if (timer.get() == 0) {
          timer.start();
          feeder.setPercent(FeederConstants.kPushPercent);
        } else if (timer.get() >= FeederConstants.kPushTime) {
          timer.stop();
          timer.reset();
          feeder.setState(FeederState.POSITION_NOTE_REVERSE);
        } else {
          feeder.setPercent(FeederConstants.kPushPercent);
        }
        break;
      case POSITION_NOTE_REVERSE:
        if (timer.get() == 0) {
          timer.start();
          feeder.setPercent(FeederConstants.kAlignPercent);
        } else if (timer.get() >= FeederConstants.kAlignTime) {
          timer.stop();
          timer.reset();
          feeder.setState(FeederState.MANUAL);
        } else {
          feeder.setPercent(FeederConstants.kAlignPercent);
        }
        break;
      case EJECT:
        feeder.setIsNoteInRobotSystem(false);
        if (intake.isReadyToEject()) {
          feeder.setPercent(IntakeConstants.kEjectRollerPercent);
        } else {
          feeder.setPercent(0);
        }
        break;
      case FEED_SHOOTER:
        feeder.setIsNoteInRobotSystem(false);
        feeder.setPercent(IntakeConstants.kFeederRollerPercent);
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
