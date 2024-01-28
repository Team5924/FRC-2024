// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.wrist;

import org.first5924.frc2024.constants.WristConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class WristIOTalonFX implements WristIO {
    //private final CANSparkMax mLeaderSpark = SparkMaxFactory.createSparkMax(PivotConstants.kLeaderSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);
    private final TalonFX mMotor = new TalonFX(WristConstants.motorID);
    private final CANcoder mEncoder = new CANcoder(WristConstants.encoderID);
    public WristIOTalonFX() {
    
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.motorTempCelsius = mMotor.getDeviceTemp().getValueAsDouble();
        inputs.motorCurrentAmps = mMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorCurrentVelocity = mMotor.getVelocity().getValueAsDouble();
        inputs.encoderPosition = mEncoder.getPosition().getValueAsDouble();
        inputs.wristAngle = mEncoder.getPosition().getValueAsDouble()*360;
    }

    @Override
    public void setPercent(double percent) {
        mMotor.set(percent);
    }
    public void setVoltage(double voltage) {
        mMotor.setVoltage(voltage);
    }
    
}