// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023swerve.robot;

import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.first5924.frc2023swerve.commands.drive.DriveWithJoysticks;
import org.first5924.frc2023swerve.commands.drive.SetGyroYaw;
import org.first5924.frc2023swerve.constants.Constants;
import org.first5924.frc2023swerve.subsystems.drive.Drive;
import org.first5924.frc2023swerve.subsystems.drive.GyroIO;
import org.first5924.frc2023swerve.subsystems.drive.GyroIOPigeon2;
import org.first5924.frc2023swerve.subsystems.drive.ModuleIO;
import org.first5924.frc2023swerve.subsystems.drive.ModuleIOSparkMax;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  private final Drive drive;

  // Controller
  private final CommandXboxController driverController = new CommandXboxController(0);
  // private final CommandXboxController operatorController = new CommandXboxController(1);
  private final LoggedDashboardChooser<Boolean> swerveModeChooser =
      new LoggedDashboardChooser<>("Swerve Mode Chooser");
  private final SendableChooser<Command> autoModeChooser;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    switch (Constants.currentMode) {
        // Real robot, instantiate hardware IO implementations
      case REAL:
        drive =
            new Drive(
                new GyroIOPigeon2(),
                new ModuleIOSparkMax(0),
                new ModuleIOSparkMax(1),
                new ModuleIOSparkMax(2),
                new ModuleIOSparkMax(3));
        break;

        // Sim robot, instantiate physics sim IO implementations
      case SIM:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        break;

        // Replayed robot, disable IO implementations
      default:
        drive =
            new Drive(
                new GyroIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {},
                new ModuleIO() {});

        break;
    }



    swerveModeChooser.addDefaultOption("Field Centric", true);
    swerveModeChooser.addOption("Robot Centric", false);

    autoModeChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Mode Chooser", autoModeChooser);

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
    drive.setDefaultCommand(
        new DriveWithJoysticks(
            drive,
            driverController::getLeftX,
            driverController::getLeftY,
            driverController::getRightX,
            swerveModeChooser::get));
    driverController.a().onTrue(new SetGyroYaw(drive, 0));

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoModeChooser.getSelected();
  }
}
