// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.wrist;

import org.littletonrobotics.junction.Logger;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  /** Creates a new Wrist. */
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  //private final PIDController mPID = new PIDController(10, 0, 0);

  public Wrist(WristIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Wrist", inputs);
    SmartDashboard.putNumber("Wrist current angle", getWristAngle());
    // This method will be called once per scheduler run
  }
  
  public double getMotorTempCelsius() {
    return inputs.motorTempCelsius;
  }

  public double getUpperMotorCurrentAmps() {
    return inputs.motorCurrentAmps;
  }

  public double getMotorCurrentVelocity() {
    return inputs.motorCurrentVelocity;
  }

  public double getWristAngle(){
    return inputs.wristAngle;
  }


  public void setPercent(double percent) {
    io.setPercent(percent);
  }

  public void setAngle (double targetAngle){
    io.setAngle(targetAngle);
  }


  //public void setAngle(double angle) {
  //  io.setVoltage(mPID.calculate(inputs.encoderPosition/360, angle));
  //}


  
}
