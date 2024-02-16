// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.intakePivot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import org.first5924.frc2024.constants.IntakePivotConstants;
import org.first5924.frc2024.constants.IntakePivotConstants.PivotPosition;


public class IntakePivot extends SubsystemBase {
  /** Creates a new IntakePivot. */
    private PivotPosition pivotPosition = PivotPosition.RETRACT;

    private final IntakePivotIO io;
    // private final IntakePivotIOTalonFX pivoton;Talon;
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
    SmartDashboard.putString("Pivot State", pivotPosition.toString());

  }

  public double getPivotPositionAngle(){
    return inputs.pivotPositionAngle;
  }

  public void setPivotPosition(double PivotPosition) {
    double voltage = 
      MathUtil.clamp(
              mPID.calculate(getPivotPositionAngle(), PivotPosition),
              -IntakePivotConstants.MaxVoltage,
              IntakePivotConstants.MaxVoltage);
    SmartDashboard.putNumber("Pivot Voltage", voltage);
    io.setVoltage(voltage);
  }

  public void setBrakeMode(boolean enabled) {
    io.setBrakeMode(enabled);
  }

  public void setPivotSate(PivotPosition pivotPosition ){
    this.pivotPosition = pivotPosition;
  }

  public final void setVoltage(double volts) {
    // pivotTalon.setVoltage(volts);
    io.setVoltage(volts);
  }


  
  
}
