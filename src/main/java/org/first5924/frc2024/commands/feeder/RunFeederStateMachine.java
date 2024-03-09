// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.feeder;

import java.util.function.DoubleSupplier;

import org.first5924.frc2024.constants.FeederConstants;
import org.first5924.frc2024.constants.IntakeConstants;
import org.first5924.frc2024.constants.FeederConstants.FeederState;
import org.first5924.frc2024.constants.IntakeConstants.IntakeState;
import org.first5924.frc2024.subsystems.feeder.Feeder;
import org.first5924.frc2024.subsystems.intake.Intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class RunFeederStateMachine extends Command {
  private final Feeder feeder;
  private final Intake intake;
  private final DoubleSupplier leftJoystickY;

  private final Timer timer = new Timer();

  /** Creates a new FeederShoot. */
  public RunFeederStateMachine(Feeder feeder, Intake intake, DoubleSupplier leftJoystickY) {
    this.feeder = feeder;
    this.intake = intake;
    this.leftJoystickY = leftJoystickY;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch (feeder.getState()) {
      case MANUAL:
        feeder.setPercent(-MathUtil.applyDeadband(leftJoystickY.getAsDouble(), 0.2));
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
          feeder.setPercent(IntakeConstants.kFloorRollerPercent);
        } else {
          feeder.setPercent(IntakeConstants.kFloorRollerPercent);
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
        if (intake.isReadyToEject()) {
          feeder.setPercent(IntakeConstants.kEjectRollerPercent);
        } else {
          feeder.setPercent(0);
        }
        break;
      case FEED_SHOOTER:
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
