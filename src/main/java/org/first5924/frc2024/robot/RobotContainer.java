// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.first5924.frc2024.constants.DriveConstants;
import org.first5924.frc2024.commands.SetWristAndElevatorState;
import org.first5924.frc2024.commands.drive.RunDriveStateMachine;
import org.first5924.frc2024.commands.drive.SetDriveState;
import org.first5924.frc2024.commands.drive.SetGyroYaw;
import org.first5924.frc2024.commands.elevator.ElevatorManualControl;
import org.first5924.frc2024.commands.elevator.RunElevatorStateMachine;
import org.first5924.frc2024.commands.feeder.RunFeederStateMachine;
import org.first5924.frc2024.commands.feeder.SetFeederState;
import org.first5924.frc2024.commands.wrist.RunWristStateMachine;
import org.first5924.frc2024.commands.wrist.SetWristPositionShuffleboard;
import org.first5924.frc2024.commands.wrist.WristManualControl;
import org.first5924.frc2024.commands.shooter.RunShooterStateMachine;
import org.first5924.frc2024.commands.shooter.SetShooterState;
import org.first5924.frc2024.commands.vision.RunVisionPoseEstimation;
import org.first5924.frc2024.constants.RobotConstants;
import org.first5924.frc2024.constants.WristAndElevatorState;
import org.first5924.frc2024.constants.DriveConstants.DriveState;
import org.first5924.frc2024.constants.FeederConstants.FeederState;
import org.first5924.frc2024.constants.IntakeConstants.IntakeState;
import org.first5924.frc2024.constants.ShooterConstants.ShooterState;
import org.first5924.frc2024.commands.intake.RunIntakeStateMachine;
import org.first5924.frc2024.commands.intake.SetIntakeState;
import org.first5924.frc2024.commands.intake.SetIntakeRollerPercent;
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

import com.pathplanner.lib.auto.NamedCommands;
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
  private final DetectorCam dCam;
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
        dCam = new DetectorCam();
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
        dCam = new DetectorCam();
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
        dCam = new DetectorCam();
        feeder = new Feeder(new FeederIO() {});
        shooter = new Shooter(new ShooterIO() {});
        elevator = new Elevator(new ElevatorIO() {});
        wrist = new Wrist(new WristIO() {});
        break;
    }

    NamedCommands.registerCommand("setIntakeStateFloor", new SetIntakeState(intake, elevator, feeder, IntakeState.FLOOR));
    NamedCommands.registerCommand("setIntakeStateFloorOff", new SetIntakeState(intake, elevator, feeder, IntakeState.FLOOR_OFF));
    NamedCommands.registerCommand("setIntakeStateRetract", new SetIntakeState(intake, elevator, feeder, IntakeState.RETRACT));
    NamedCommands.registerCommand("setWristAndElevatorStateAimLow", new SetWristAndElevatorState(elevator, WristAndElevatorState.AIM_LOW));
    NamedCommands.registerCommand("setShooterStateOn", new SetShooterState(shooter, ShooterState.ON));
    NamedCommands.registerCommand("setShooterStateOff", new SetShooterState(shooter, ShooterState.OFF));
    NamedCommands.registerCommand("setFeederStateFeedShooter", new SetFeederState(feeder, FeederState.FEED_SHOOTER));
    NamedCommands.registerCommand("setFeederStateManual", new SetFeederState(feeder, FeederState.MANUAL));
    NamedCommands.registerCommand("setGyroBottomStart", new SetGyroYaw(drive, DriveConstants.kBlueBottomAutoStartingYawDegrees, DriverStation::getAlliance, true));

    swerveModeChooser.addDefaultOption("Field Centric", true);
    swerveModeChooser.addOption("Robot Centric", false);

    autoModeChooser.addDefaultOption("4 Note Auto", "4 Note Auto");
    autoModeChooser.addDefaultOption("3 Note Bottom Auto", "3 Note Bottom Auto");
    autoModeChooser.addDefaultOption("1 Note Bottom Auto", "1 Note Bottom Auto");
    autoModeChooser.addOption("Nothing", "Nothing");
    autoModeChooser.addOption("SysId Quasistatic Forward", "SysId Quasistatic Forward");
    autoModeChooser.addOption("SysId Quasistatic Reverse", "SysId Quasistatic Reverse");
    autoModeChooser.addOption("SysId Dynamic Forward", "SysId Dynamic Forward");
    autoModeChooser.addOption("SysId Dynamic Reverse", "SysId Dynamic Reverse");

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
    drive.setDefaultCommand(new RunDriveStateMachine(
      drive,
      driverController::getLeftX,
      driverController::getLeftY,
      driverController::getRightX,
      swerveModeChooser::get,
      DriverStation::getAlliance,
      drive::getSlowMode,
      dCam::getNoteAngleX,
      dCam::getNoteAngleY
    ));

    driverController.rightBumper()
    .onTrue(new SetDriveState(drive, drive.getDriveState(), true))
    .onFalse(new SetDriveState(drive, drive.getDriveState(), false));

    driverController.a()
    .onTrue(new SetDriveState(drive, DriveState.LOOKATSPEAKER, drive.getSlowMode()))
    .onFalse(new SetDriveState(drive, DriveState.DRIVE, drive.getSlowMode()));

    driverController.x()
    .onTrue(new SetDriveState(drive, DriveState.DRIVETONOTE, drive.getSlowMode()))
    .onFalse(new SetDriveState(drive, DriveState.DRIVE, drive.getSlowMode()));
    driverController.b().onTrue(new SetGyroYaw(drive, 0, DriverStation::getAlliance, true));
    // Uncomment and bind to auto drive to amp
    // driverController.leftBumper();


    vision.setDefaultCommand(new RunVisionPoseEstimation(drive, vision));

    // intake.setDefaultCommand(new RunIntakeStateMachine(intake));
    // operatorController.a().onTrue(
    //   new SetIntakeState(intake, elevator, feeder, IntakeState.EJECT)
    // ).onFalse(
    //   new SetIntakeState(intake, elevator, feeder, intake.getStateBeforeEject())
    // );
    operatorController.a().onTrue(new SetIntakeRollerPercent(intake, 0.8)).onFalse(new SetIntakeRollerPercent(intake, 0));
    // operatorController.povUp().onTrue(new SetIntakeState(intake, elevator, feeder, IntakeState.START));

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

    shooter.setDefaultCommand(new RunShooterStateMachine(shooter, elevator));
    operatorController.y().onTrue(new SetShooterState(shooter, ShooterState.ON)).onFalse(new SetShooterState(shooter, ShooterState.OFF));
    // Triggers elevator and wrist state change to AIM_LOW
    operatorController.leftBumper().onTrue(new SetIntakeState(intake, elevator, feeder, IntakeState.RETRACT));
    // Triggers elevator and wrist state change to INTAKE
    operatorController.rightBumper().onTrue(new SetIntakeState(intake, elevator, feeder, IntakeState.FLOOR));

    wrist.setDefaultCommand(new RunWristStateMachine(wrist, elevator, drive));
    operatorController.leftStick().toggleOnTrue(new WristManualControl(wrist, operatorController::getRightY));
    operatorController.povDown().toggleOnTrue(new SetWristPositionShuffleboard(wrist));

    elevator.setDefaultCommand(new RunElevatorStateMachine(elevator, operatorController::getRightY));
    operatorController.rightStick().toggleOnTrue(new ElevatorManualControl(elevator, operatorController::getRightY));
    operatorController.b().onTrue(new SetWristAndElevatorState(elevator, WristAndElevatorState.AMP));
    operatorController.x().onTrue(new SetWristAndElevatorState(elevator, WristAndElevatorState.AIM_LOW));
    operatorController.start().onTrue(new SetWristAndElevatorState(elevator, WristAndElevatorState.CLIMB_MAX_HEIGHT));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */


  public Command getAutonomousCommand() {
    switch (autoModeChooser.get()) {
      case "SysId Quasistatic Forward":
        return drive.sysIdQuasistatic(SysIdRoutine.Direction.kForward);
      case "SysId Quasistatic Reverse":
        return drive.sysIdQuasistatic(SysIdRoutine.Direction.kReverse);
      case "SysId Dynamic Forward":
        return drive.sysIdDynamic(SysIdRoutine.Direction.kForward);
      case "SysId Dynamic Reverse":
        return drive.sysIdDynamic(SysIdRoutine.Direction.kReverse);
      case "Nothing":
        return new InstantCommand();
      default:
        return new PathPlannerAuto(autoModeChooser.get());
    }
  }
}

