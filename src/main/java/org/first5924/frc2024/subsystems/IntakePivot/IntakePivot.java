// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.intakePivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;


public class IntakePivot extends SubsystemBase {
  /** Creates a new IntakePivot. */
    private final IntakePivotIO io;
    private final IntakePivotIOInputsAutoLogged inputs = new IntakePivotIOInputsAutoLogged();
    private final PIDController mPID = new PIDController(0, 0, 0);
    
  public IntakePivot (IntakePivotIO io){
    this.io = io;
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Intake Pivot", inputs);
  }

  public void getPivotPosition(){
    return this.PivotPosition;
  }

  public void setPivotPosition (double position){
    MathUtil.clamp(
            mPID.calculate(getPivotPosition(), position),
            -PivotConstants.kMaxVoltage,
            PivotConstants.kMaxVoltage);
    SmartDashboard.putNumber("Pivot Voltage", voltage);
    io.setVoltage(voltage);
  }
}
