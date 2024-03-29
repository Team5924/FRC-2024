// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.autoRoutines;

import org.first5924.frc2024.subsystems.drive.Drive;
import org.first5924.frc2024.subsystems.elevator.Elevator;
import org.first5924.frc2024.subsystems.feeder.Feeder;
import org.first5924.frc2024.subsystems.shooter.Shooter;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import org.first5924.frc2024.commands.SetWristAndElevatorState;
import org.first5924.frc2024.commands.drive.SetDriveStartingPose;
import org.first5924.frc2024.commands.drive.SetDriveState;
import org.first5924.frc2024.commands.drive.SetGyroYaw;
import org.first5924.frc2024.commands.feeder.SetFeederState;
import org.first5924.frc2024.commands.shooter.SetShooterState;
import org.first5924.frc2024.constants.WristAndElevatorState;
import org.first5924.frc2024.constants.DriveConstants.DriveState;
import org.first5924.frc2024.constants.FeederConstants.FeederState;
import org.first5924.frc2024.constants.ShooterConstants.ShooterState;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ThreeNoteBelowAuto extends SequentialCommandGroup {
  /** Creates a new ThreeNoteBelowAuto. */
  public ThreeNoteBelowAuto(Drive drive, Shooter shooter, Elevator elevator, Feeder feeder) {
    final PathPlannerPath belowStartToSideShot = PathPlannerPath.fromPathFile("Below Start to Side Shot");
    final PathPlannerPath sideShotToFarD = PathPlannerPath.fromPathFile("Side Shot to Far D");
    final PathPlannerPath farDToSideShot = PathPlannerPath.fromPathFile("Far D to Side Shot");
    final PathPlannerPath sideShotToFarC = PathPlannerPath.fromPathFile("Side Shot to Far C");
    final PathPlannerPath farCToSideShot = PathPlannerPath.fromPathFile("Far C to Side Shot");

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetGyroYaw(drive, 0, DriverStation::getAlliance, true),
      new SetShooterState(shooter, ShooterState.ON),
      new SetWristAndElevatorState(elevator, WristAndElevatorState.AIM_LOW),
      new SetDriveStartingPose(drive, belowStartToSideShot.getStartingDifferentialPose()),
      AutoBuilder.followPath(belowStartToSideShot),
      new SetDriveState(drive, DriveState.FACE_SPEAKER),
      new WaitCommand(0.55),
      new SetFeederState(feeder, FeederState.FEED_SHOOTER),
      new WaitCommand(0.25),
      new SetDriveState(drive, DriveState.NORMAL),
      new SetShooterState(shooter, ShooterState.OFF),
      new SetWristAndElevatorState(elevator, WristAndElevatorState.INTAKE),
      new SetFeederState(feeder, FeederState.MANUAL),
      AutoBuilder.followPath(sideShotToFarD),
      AutoBuilder.followPath(farDToSideShot),
      new SetDriveState(drive, DriveState.FACE_SPEAKER),
      new WaitCommand(0.55),
      new SetFeederState(feeder, FeederState.FEED_SHOOTER),
      new WaitCommand(0.25),
      new SetDriveState(drive, DriveState.NORMAL),
      new SetShooterState(shooter, ShooterState.OFF),
      new SetWristAndElevatorState(elevator, WristAndElevatorState.INTAKE),
      new SetFeederState(feeder, FeederState.MANUAL),
      AutoBuilder.followPath(sideShotToFarC),
      AutoBuilder.followPath(farCToSideShot),
      new SetDriveState(drive, DriveState.FACE_SPEAKER),
      new WaitCommand(0.55),
      new SetFeederState(feeder, FeederState.FEED_SHOOTER),
      new WaitCommand(0.25),
      new SetDriveState(drive, DriveState.NORMAL),
      new SetShooterState(shooter, ShooterState.OFF),
      new SetWristAndElevatorState(elevator, WristAndElevatorState.INTAKE),
      new SetFeederState(feeder, FeederState.MANUAL)
    );
  }
}
