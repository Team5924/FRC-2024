// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import org.first5924.frc2024.commands.drive.DriveWithJoysticks;
import org.first5924.frc2024.commands.drive.SetGyroYaw;

import org.first5924.frc2024.commands.shooter.ShooterOn;
import org.first5924.frc2024.commands.wrist.RotateWrist;
import org.first5924.frc2024.constants.DriveConstants;

import org.first5924.frc2024.constants.RobotConstants;
import org.first5924.frc2024.subsystems.drive.Drive;
import org.first5924.frc2024.subsystems.drive.GyroIO;
import org.first5924.frc2024.subsystems.drive.GyroIOPigeon2;
import org.first5924.frc2024.subsystems.drive.ModuleIO;
import org.first5924.frc2024.subsystems.shooter.Shooter;
import org.first5924.frc2024.subsystems.shooter.ShooterIO;
import org.first5924.frc2024.subsystems.shooter.ShooterIOTalonFX;

import org.first5924.frc2024.subsystems.wrist.Wrist;
import org.first5924.frc2024.subsystems.wrist.WristIO;
import org.first5924.frc2024.subsystems.wrist.WristIOTalonFX;
import org.first5924.frc2024.subsystems.drive.ModuleIOTalonFX;
import org.first5924.frc2024.subsystems.vision.Vision;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems

  private final Shooter shooter;
  private final Wrist wrist;
  private final Drive drive;
  private final Vision vision;


  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final LoggedDashboardChooser<Boolean> swerveModeChooser = new LoggedDashboardChooser<>("Swerve Mode Chooser");
 // private final SendableChooser<Command> autoModeChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (RobotConstants.kCurrentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        shooter = new Shooter(new ShooterIOTalonFX());
        wrist = new Wrist(new WristIOTalonFX() {});
        drive = new Drive(
          new GyroIOPigeon2(),
          new ModuleIOTalonFX(0),
          new ModuleIOTalonFX(1),
          new ModuleIOTalonFX(2),
          new ModuleIOTalonFX(3)
        );
        vision = new Vision();

        break;

      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        wrist = new Wrist(new WristIO() {});
        drive = new Drive(

          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {}
        );
        shooter = new Shooter(new ShooterIO() {});
        vision = new Vision();
        break;

      // Replayed robot, disable IO implementations
      default:
        shooter = new Shooter(new ShooterIO() {});
        wrist = new Wrist(new WristIO() {});
        drive = new Drive(
          new GyroIOPigeon2(),
          new ModuleIOTalonFX(0),
          new ModuleIOTalonFX(1),
          new ModuleIOTalonFX(2),
          new ModuleIOTalonFX(3)
        );
        vision = new Vision();
        break;
    }

    swerveModeChooser.addDefaultOption("Field Centric", true);
    swerveModeChooser.addOption("Robot Centric", false);


    autoModeChooser = null;

    //SmartDashboard.putData("Auto Mode Chooser", autoModeChooser);

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    operatorController.a().whileTrue(new ShooterOn(shooter));
    wrist.setDefaultCommand(new RotateWrist(wrist, driverController::getLeftY));
    drive.setDefaultCommand(new DriveWithJoysticks(
      drive,
      driverController::getLeftX,
      driverController::getLeftY,
      driverController::getRightX,
      swerveModeChooser::get
    ));
    driverController.a().onTrue(new SetGyroYaw(drive, 0));
    //driverController.y().onTrue(FollowPath());
  }

  //public Command FollowPath()
  //{
  //  return Choreo.choreoSwerveCommand
  //  (Choreo.getTrajectory("NewPath"), //will need to make sendable chooser in the future
  //  () -> drive.getPose(),
  //  Choreo.choreoSwerveController(
  //    new PIDController(DriveConstants.kDriveKp, 0, 0), 
  //    new PIDController(DriveConstants.kDriveKp, 0, 0),
  //    new PIDController(DriveConstants.kDriveKp, 0, 0)),
  //  (ChassisSpeeds speeds) ->
  //    drive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false), 
  //  () -> false,
  //  drive);
  //}

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


  public Command getAutonomousCommand() {
    // return Choreo.choreoSwerveCommand
    // (Choreo.getTrajectory("NewPath"), //will need to make sendable chooser in the future
    // () -> drive.getPose(),
    // Choreo.choreoSwerveController(
    //   new PIDController(DriveConstants.kDriveKp, 0, 0),
    //   new PIDController(DriveConstants.kDriveKp, 0, 0),
    //   new PIDController(DriveConstants.kDriveKp, 0, 0)),
    // (ChassisSpeeds speeds) ->
    //   drive.drive(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond, false), 
    // () -> false,
    // drive);
    return null;
  }
}

