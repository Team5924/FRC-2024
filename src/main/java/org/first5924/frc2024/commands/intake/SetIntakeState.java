// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.intake;

import org.first5924.frc2024.constants.WristAndElevatorState;
import org.first5924.frc2024.constants.FeederConstants.FeederState;
import org.first5924.frc2024.constants.IntakeConstants.IntakeState;
import org.first5924.frc2024.subsystems.elevator.Elevator;
import org.first5924.frc2024.subsystems.feeder.Feeder;
import org.first5924.frc2024.subsystems.intake.Intake;
import org.first5924.frc2024.subsystems.leds.Leds;

import edu.wpi.first.wpilibj2.command.InstantCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetIntakeState extends InstantCommand {
  private final Intake intake;
  private final Elevator elevator;
  private final Feeder feeder;
  private final IntakeState state;
  //private final Leds leds;

  public SetIntakeState(Intake intake, Elevator elevator, Feeder feeder, IntakeState state) {
    this.intake = intake;
    this.elevator = elevator;
    this.feeder = feeder;
    //this.leds = leds;
    this.state = state;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setState(state);
    if (state == IntakeState.RETRACT) {
      elevator.setWristAndElevatorState(WristAndElevatorState.AIM_LOW);
    } else if (state == IntakeState.FLOOR) {
      elevator.setWristAndElevatorState(WristAndElevatorState.INTAKE);
      feeder.setState(FeederState.INTAKE);
      //leds.setColors();
    } else if (state == IntakeState.EJECT) {
      feeder.setState(FeederState.EJECT);
    }
  }
}
