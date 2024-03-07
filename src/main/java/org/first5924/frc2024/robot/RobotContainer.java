// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.DoubleSupplier;

import org.first5924.frc2024.commands.SetWristAndElevatorState;
import org.first5924.frc2024.commands.drive.DriveWithJoysticks;
import org.first5924.frc2024.commands.drive.ResetGyroYaw;
import org.first5924.frc2024.commands.elevator.ElevatorManualControl;
import org.first5924.frc2024.commands.elevator.RunElevatorStateMachine;
import org.first5924.frc2024.commands.feeder.RunFeederStateMachine;
import org.first5924.frc2024.commands.feeder.SetFeederState;
import org.first5924.frc2024.commands.wrist.RunWristStateMachine;
import org.first5924.frc2024.commands.wrist.SetWristPositionShuffleboard;
import org.first5924.frc2024.commands.wrist.WristManualControl;
import org.first5924.frc2024.commands.shooter.EnableShooter;
import org.first5924.frc2024.commands.vision.RunVisionPoseEstimation;
import org.first5924.frc2024.constants.RobotConstants;
import org.first5924.frc2024.constants.WristAndElevatorState;
import org.first5924.frc2024.constants.FeederConstants.FeederState;
import org.first5924.frc2024.constants.IntakeConstants.IntakeState;
import org.first5924.frc2024.commands.intake.RunIntakeStateMachine;
import org.first5924.frc2024.commands.intake.SetIntakeState;
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
import org.first5924.frc2024.subsystems.vision.Vision;
import org.first5924.frc2024.subsystems.vision.VisionIO;
import org.first5924.frc2024.subsystems.vision.VisionIOReal;
import org.first5924.frc2024.subsystems.wrist.Wrist;
import org.first5924.frc2024.subsystems.wrist.WristIO;
import org.first5924.frc2024.subsystems.wrist.WristIOTalonFX;
import org.first5924.frc2024.subsystems.drive.ModuleIOTalonFX;
import org.first5924.frc2024.subsystems.elevator.Elevator;
import org.first5924.frc2024.subsystems.elevator.ElevatorIO;
import org.first5924.frc2024.subsystems.elevator.ElevatorIOTalonFX;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.commands.PathPlannerAuto;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  private final Drive drive;
  private final Intake intake;
  private final Vision vision;
  // private final DetectorCam dCam;
  private final Feeder feeder;
  private final Shooter shooter;
  private final Elevator elevator;
  private final Wrist wrist;

  private final CommandXboxController driverController = new CommandXboxController(0);
  private final CommandXboxController operatorController = new CommandXboxController(1);

  private final LoggedDashboardChooser<Boolean> swerveModeChooser = new LoggedDashboardChooser<>("Swerve Mode Chooser");
  private final LoggedDashboardChooser<String> autoModeChooser = new LoggedDashboardChooser<>("Auto Mode Chooser");

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (RobotConstants.kCurrentMode) {
      // Real robot, instantiate hardware IO implementations
      case REAL:
        drive = new Drive(
          new GyroIOPigeon2(),
          new ModuleIOTalonFX(0),
          new ModuleIOTalonFX(1),
          new ModuleIOTalonFX(2),
          new ModuleIOTalonFX(3)
        );
        intake = new Intake(new IntakeIOTalonFX());
        vision = new Vision(new VisionIOReal());
        // dCam = new DetectorCam();
        feeder = new Feeder(new FeederIOTalonFX());
        shooter = new Shooter(new ShooterIOTalonFX());
        elevator = new Elevator(new ElevatorIOTalonFX());
        wrist = new Wrist(new WristIOTalonFX() {});
        break;
      // Sim robot, instantiate physics sim IO implementations
      case SIM:
        drive = new Drive(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {}
        );
        intake = new Intake(new IntakeIO() {});
        vision = new Vision(new VisionIO() {});
        // dCam = new DetectorCam();
        feeder = new Feeder(new FeederIO() {});
        shooter = new Shooter(new ShooterIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        wrist = new Wrist(new WristIO() {});
        break;
      // Replayed robot, disable IO implementations
      default:
        drive = new Drive(
          new GyroIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {},
          new ModuleIO() {}
        );
        intake = new Intake(new IntakeIO() {});
        vision = new Vision(new VisionIO() {});
        // dCam = new DetectorCam();
        feeder = new Feeder(new FeederIO() {});
        shooter = new Shooter(new ShooterIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        wrist = new Wrist(new WristIO() {});
        break;
    }

    swerveModeChooser.addDefaultOption("Field Centric", true);
    swerveModeChooser.addOption("Robot Centric", false);

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    // Driver
    drive.setDefaultCommand(new DriveWithJoysticks(
      drive,
      driverController::getLeftX,
      driverController::getLeftY,
      driverController::getRightX,
      swerveModeChooser::get,
      DriverStation::getAlliance,
      false,
      false
    ));
    driverController.rightBumper().whileTrue(new DriveWithJoysticks(
      drive,
      driverController::getLeftX,
      driverController::getLeftY,
      driverController::getRightX,
      swerveModeChooser::get,
      DriverStation::getAlliance,
      true,
      false
    ));
    driverController.a().whileTrue(new DriveWithJoysticks(
      drive,
      driverController::getLeftX,
      driverController::getLeftY,
      driverController::getRightX,
      swerveModeChooser::get,
      DriverStation::getAlliance,
      false,
      true
    ));
    driverController.x().whileTrue(new DriveWithJoysticks(
      drive,
      driverController::getLeftX,
      driverController::getLeftY,
      driverController::getRightX,
      swerveModeChooser::get,
      DriverStation::getAlliance,
      true,
      true
    ));
    driverController.b().onTrue(new ResetGyroYaw(drive, DriverStation::getAlliance));
    // Uncomment and bind to auto drive to amp
    // driverController.leftBumper();

    vision.setDefaultCommand(new RunVisionPoseEstimation(drive, vision));

    intake.setDefaultCommand(new RunIntakeStateMachine(intake));
    operatorController.back().onTrue(
      new SetIntakeState(intake, elevator, feeder, IntakeState.EJECT)
    ).onFalse(
      new SetIntakeState(intake, elevator, feeder, intake.getStateBeforeEject())
    );

    feeder.setDefaultCommand(new RunFeederStateMachine(feeder, intake, operatorController::getLeftY));
    operatorController.rightTrigger(0.75)
      .onTrue(new SetFeederState(feeder, FeederState.FEED_SHOOTER))
      .onFalse(new SetFeederState(feeder, FeederState.MANUAL));

    operatorController.leftTrigger().whileTrue(new ParallelCommandGroup(
      new SetIntakeState(intake, elevator, feeder, IntakeState.EJECT)
    )).onFalse(new ParallelCommandGroup(
      new SetIntakeState(intake, elevator, feeder, intake.getStateBeforeEject()),
      new SetFeederState(feeder, FeederState.MANUAL)
    ));

    operatorController.y().onTrue(new EnableShooter(shooter, elevator, true)).onFalse(new EnableShooter(shooter, elevator, false));
    // Triggers elevator and wrist state change to AIM_LOW
    operatorController.leftBumper().onTrue(new SetIntakeState(intake, elevator, feeder, IntakeState.START));
    // Triggers elevator and wrist state change to INTAKE
    operatorController.rightBumper().onTrue(new SetIntakeState(intake, elevator, feeder, IntakeState.FLOOR));

    wrist.setDefaultCommand(new RunWristStateMachine(wrist, elevator, drive));
    operatorController.leftStick().onTrue(new WristManualControl(wrist, operatorController::getRightY));
    // GenericEntry wristAngleSetter = Shuffleboard.getTab("SmartDashboard").add("Wrist Angle Setter", 30).getEntry();
    // DoubleSupplier doubleSupplier = () -> wristAngleSetter.getDouble(30);
    // operatorController.leftStick().onTrue(new SetWristPositionShuffleboard(wrist, elevator, doubleSupplier));

    elevator.setDefaultCommand(new RunElevatorStateMachine(elevator, operatorController::getRightY));
    operatorController.rightStick().onTrue(new ElevatorManualControl(elevator, operatorController::getRightY));

    operatorController.a().onTrue(new SetWristAndElevatorState(elevator, WristAndElevatorState.INTAKE));
    operatorController.b().onTrue(new SetWristAndElevatorState(elevator, WristAndElevatorState.AMP));
    operatorController.x().onTrue(new SetWristAndElevatorState(elevator, WristAndElevatorState.AIM_LOW));
    operatorController.start().onTrue(new SetWristAndElevatorState(elevator, WristAndElevatorState.CLIMB));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


  public Command getAutonomousCommand() {
    return new PathPlannerAuto(autoModeChooser.get());
  }
}

