// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.BooleanSupplier;

import org.first5924.frc2024.commands.AutoAimAndShoot;
import org.first5924.frc2024.commands.SetWristAndElevatorState;
import org.first5924.frc2024.commands.TeleopAimAndShoot;
import org.first5924.frc2024.commands.drive.DriveWithJoysticks;
import org.first5924.frc2024.commands.drive.SetGyroYaw;

import org.first5924.frc2024.commands.feeder.FeederSlow;
import org.first5924.frc2024.commands.wrist.RunWrist;
import org.first5924.frc2024.commands.wrist.SetWristPosition;
import org.first5924.frc2024.commands.shooter.ShooterOn;
import org.first5924.frc2024.commands.vision.DriveToNote;
import org.first5924.frc2024.commands.vision.TurnToSpeaker;
import org.first5924.frc2024.commands.wrist.SetWristVoltage;
import org.first5924.frc2024.commands.elevator.RunElevator;
import org.first5924.frc2024.commands.elevator.RunElevatorVoltage;
import org.first5924.frc2024.commands.elevator.SetHeight;
import org.first5924.frc2024.constants.RobotConstants;
import org.first5924.frc2024.constants.WristAndElevatorState;
import org.first5924.frc2024.constants.IntakeConstants.IntakeState;
import org.first5924.frc2024.commands.intake.RunIntake;
import org.first5924.frc2024.commands.intake.SetIntakeState;
import org.first5924.frc2024.commands.intake.SetPivotVoltage;
import org.first5924.frc2024.commands.intake.SetRollerVoltage;
import org.first5924.frc2024.subsystems.intake.Intake;
import org.first5924.frc2024.subsystems.intake.IntakeIO;
import org.first5924.frc2024.subsystems.intake.IntakeIOTalonFX;

import org.first5924.frc2024.subsystems.drive.Drive;
import org.first5924.frc2024.subsystems.drive.GyroIO;
import org.first5924.frc2024.subsystems.drive.GyroIOPigeon2;
import org.first5924.frc2024.subsystems.drive.ModuleIO;
import org.first5924.frc2024.subsystems.feeder.Feeder;
import org.first5924.frc2024.subsystems.feeder.FeederIO;
import org.first5924.frc2024.subsystems.feeder.FeederIOTalonFX;
import org.first5924.frc2024.subsystems.shooter.Shooter;
import org.first5924.frc2024.subsystems.shooter.ShooterIO;
import org.first5924.frc2024.subsystems.shooter.ShooterIOTalonFX;
import org.first5924.frc2024.subsystems.vision.DetectorCam;
import org.first5924.frc2024.subsystems.vision.FieldCam;
import org.first5924.frc2024.subsystems.wrist.Wrist;
import org.first5924.frc2024.subsystems.wrist.WristIO;
import org.first5924.frc2024.subsystems.wrist.WristIOTalonFX;
import org.first5924.frc2024.subsystems.drive.ModuleIOTalonFX;
import org.first5924.frc2024.subsystems.elevator.Elevator;
import org.first5924.frc2024.subsystems.elevator.ElevatorIO;
import org.first5924.frc2024.subsystems.elevator.ElevatorIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems

  private final Feeder feeder;
  private final Shooter shooter;
  private final Wrist wrist;
  private final Drive drive;
  // private final DetectorCam dCam;
  // private final FieldCam fieldCam;
  private final Intake intake;
  private final Elevator elevator;
  // private final Vision vision;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);
  private final LoggedDashboardChooser<Boolean> swerveModeChooser = new LoggedDashboardChooser<>("Swerve Mode Chooser");


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

        feeder = new Feeder(new FeederIOTalonFX());
        // vision = new Vision();

        // feeder = new Feeder(new FeederIOTalonFX());
        // fieldCam = new FieldCam();
        // dCam = new DetectorCam();
        intake = new Intake(new IntakeIOTalonFX());
        elevator = new Elevator(new ElevatorIOTalonFX());
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
        feeder = new Feeder(new FeederIO() {});
        shooter = new Shooter(new ShooterIO() {});
        // fieldCam = new FieldCam();
        // dCam = new DetectorCam();
        intake = new Intake(new IntakeIO() {});
        elevator = new Elevator(new ElevatorIO() {});
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
        feeder = new Feeder(new FeederIO() {});
        // vision = new Vision();
        // feeder = new Feeder(new FeederIO() {});
        // fieldCam = new FieldCam();
        // dCam = new DetectorCam();
        intake = new Intake(new IntakeIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        break;
    }

    swerveModeChooser.addDefaultOption("Field Centric", true);
    swerveModeChooser.addOption("Robot Centric", false);

    //Logger.recordOutput("Is Note In", feeder.isNoteIn());
    // SmartDashboard.putData("Auto Mode Chooser", autoModeChooser);
    //SmartDashboard.putBoolean("is note in feeder?", feeder.isNoteIn());
    // autoModeChooser = null;
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
    //operatorController.y().whileTrue(new ShooterOn(shooter));
    operatorController.y().whileTrue(new ShooterOn(shooter));
    wrist.setDefaultCommand(new SetWristVoltage(wrist, operatorController::getLeftY));
    drive.setDefaultCommand(new DriveWithJoysticks(
      drive,
      driverController::getLeftX,
      driverController::getLeftY,
      driverController::getRightX,
      swerveModeChooser::get,
      false
    ));
    driverController.rightBumper().onTrue(new DriveWithJoysticks(
      drive,
      driverController::getLeftX,
      driverController::getLeftY,
      driverController::getRightX,
      swerveModeChooser::get,
      true
    ));
    // driverController.rightBumper().onFalse(new DriveWithJoysticks(
    //   drive,
    //   driverController::getLeftX,
    //   driverController::getLeftY,
    //   driverController::getRightX,
    //   swerveModeChooser::get,
    //   () -> false
    // ));
    // driverController.a().onTrue(new SetGyroYaw(drive, 0));
    // feeder.setDefaultCommand(new FeederSlow(feeder));

    //
    // THIS IS TEMPORARY, IT WILL BE IN AUTONOMOUS
    // driverController.b().onTrue(new DriveToNote(dCam::getNoteX, dCam::getNoteY, dCam.hasTarget(), drive));
    //feeder.setDefaultCommand(new FeederSlow(feeder));
    // operatorController.b().whileTrue(new FeederSlow(feeder, operatorController::getRightY));
    operatorController.leftTrigger(0.75).whileTrue(new FeederSlow(feeder));
    // operatorController.y().whileTrue(new TeleopAimAndShoot(feeder, shooter, wrist, wrist::getAngleDegrees, fieldCam::getRedShooterAngle));
    // operatorController.x().whileTrue(new PIDTest(wrist));
    // //driverController.y().onTrue(FollowPath());
    // driverController.leftTrigger().whileTrue(new TurnToSpeaker(drive, fieldCam::getBotYaw, fieldCam::getYawToRedSpeaker));
    // wrist.setDefaultCommand(new RotateWrist(wrist, operatorController::getRightY));
    // intake.setDefaultCommand(new RunIntake(intake));
    // operatorController.leftBumper().onTrue(new SetIntakeState(intake, IntakeState.RETRACT));
    // operatorController.rightBumper().onTrue(new SetIntakeState(intake, IntakeState.FLOOR));
    // operatorController.rightTrigger(0.75).onTrue(new SetIntakeState(intake, IntakeState.EJECT));
    // operatorController.rightTrigger(0.75).onFalse(new SetIntakeState(intake, intake.getIntakeStateBeforeEject()));
    // intake.setDefaultCommand(new RunIntake(intake));
    operatorController.leftBumper().onTrue(new SetIntakeState(intake, IntakeState.RETRACT));
    operatorController.rightBumper().onTrue(new SetIntakeState(intake, IntakeState.FLOOR));
    operatorController.rightTrigger(0.75).onTrue(new SetIntakeState(intake, IntakeState.EJECT));
    // operatorController.rightTrigger(0.75).onFalse(new SetIntakeState(intake, intake.getIntakeStateBeforeEject()));
    // operatorController.a().whileTrue(new SetRollerVoltage(intake, 4));
    // operatorController.b().whileTrue(new SetPivotVoltage(intake, 1));
    // operatorController.x().whileTrue(new SetPivotVoltage(intake, -1));

    // elevator.setDefaultCommand(new RunElevatorVoltage(elevator, operatorController::getRightY));
    //elevator.setDefaultCommand(new RunElevator(elevator, operatorController::getRightY));
    //wrist.setDefaultCommand(new RunWrist(wrist, elevator));
    operatorController.a().onTrue(new SetWristPosition(wrist, 45));
    operatorController.b().onTrue(new SetWristAndElevatorState(elevator, WristAndElevatorState.INTAKE));
    operatorController.x().onTrue(new SetWristAndElevatorState(elevator, WristAndElevatorState.AMP));
    driverController.a().onTrue(new SetGyroYaw(drive, 0));
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

    // return new SequentialCommandGroup(new AutoAimAndShoot(feeder, shooter, wrist, wrist::getAngleDegrees, fieldCam::getRedShooterAngle));
    return null;
  }
}

