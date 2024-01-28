// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.elevator;

import org.first5924.frc2024.constants.ElevatorConstants;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import au.grapplerobotics.LaserCan;

/** Add your docs here. */
public class ElevatorIOTalonFX implements ElevatorIO {
    //private final CANSparkMax mLeaderSpark = SparkMaxFactory.createSparkMax(PivotConstants.kLeaderSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);
    private final TalonFX mMotor = new TalonFX(ElevatorConstants.motorID);
    private final LaserCan lc = new LaserCan(ElevatorConstants.laserID);
    private final CANcoder mEncoder = new CANcoder(ElevatorConstants.encoderID);
    public ElevatorIOTalonFX() {
    
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        inputs.motorTempCelsius = mMotor.getDeviceTemp().getValueAsDouble();
        inputs.motorCurrentAmps = mMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorCurrentVelocity = mMotor.getVelocity().getValueAsDouble();
        inputs.motorPosition = mMotor.getPosition().getValueAsDouble();
        inputs.lastAxleEncoderPosition = mEncoder.getPosition().getValueAsDouble();
        inputs.height = lc.getMeasurement().distance_mm*Math.sin(ElevatorConstants.elevatorAngle);
    }

    @Override
    public void setPercent(double percent) {
        mMotor.set(percent);
    }

    public void setVoltage(double volts) {
        mMotor.setVoltage(volts);
    }

}