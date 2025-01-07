// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.roller;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/** Class to run the rollers over CAN */
public class Roller extends SubsystemBase {
  private final TalonSRX rollerMotor;

  public Roller() {
    // Set up the roller motor as a brushed motor
    rollerMotor = new TalonSRX(30);

    // Set can timeout. Because this project only sets parameters once on
    // construction, the timeout can be long without blocking robot operation. Code
    // which sets or gets parameters during operation may need a shorter timeout.

    // Create and apply configuration for roller motor. Voltage compensation helps
    // the roller behave the same as the battery
    // voltage dips. The current limit helps prevent breaker trips or burning out
    // the motor in the event the roller stalls.
    
  }

  @Override
  public void periodic() {
  }

  /** This is a method that makes the roller spin */
  public void runRoller(double forward) {
    SmartDashboard.putNumber("%", forward);
    rollerMotor.set(ControlMode.PercentOutput, forward);
  }
}
