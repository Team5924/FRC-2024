// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.shooter;

import org.first5924.frc2024.constants.ShooterConstants;
import com.ctre.phoenix6.hardware.TalonFX;

/** Add your docs here. */
public class ShooterIOTalonFX implements ShooterIO {
    //private final CANSparkMax mLeaderSpark = SparkMaxFactory.createSparkMax(PivotConstants.kLeaderSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);
    private final TalonFX mUpperMotor = new TalonFX(ShooterConstants.upperMotorID);
    private final TalonFX mLowerMotor = new TalonFX(ShooterConstants.lowerMotorID);
    public ShooterIOTalonFX() {
    
    }

    @Override
    public void updateInputs(ShooterIOInputs inputs) {
        inputs.upperMotorTempCelsius = mUpperMotor.getDeviceTemp().getValueAsDouble();
        inputs.lowerMotorTempCelsius = mLowerMotor.getDeviceTemp().getValueAsDouble();
        inputs.upperMotorCurrentAmps = mUpperMotor.getSupplyCurrent().getValueAsDouble();
        inputs.lowerMotorCurrentAmps = mLowerMotor.getSupplyCurrent().getValueAsDouble();
        inputs.upperMotorCurrentVelocity = mUpperMotor.getVelocity().getValueAsDouble();
        inputs.lowerMotorCurrentVelocity = mLowerMotor.getVelocity().getValueAsDouble();
    
    }

    @Override
    public void setPercent(double percent) {
        mUpperMotor.set(percent);
        mLowerMotor.set(percent);
    }
}