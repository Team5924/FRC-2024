// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.feeder;

import org.first5924.frc2024.constants.FeederConstants;
import com.ctre.phoenix6.hardware.TalonFX;

import au.grapplerobotics.LaserCan;

/** Add your docs here. */
public class FeederIOTalonFX implements FeederIO {
    //private final CANSparkMax mLeaderSpark = SparkMaxFactory.createSparkMax(PivotConstants.kLeaderSparkPort, MotorType.kBrushless, IdleMode.kBrake, 42);
    private final TalonFX mMotor = new TalonFX(FeederConstants.motorID);
    //private final LaserCan lc = new LaserCan(FeederConstants.laserID);
    public FeederIOTalonFX() {
    
    }

    @Override
    public void updateInputs(FeederIOInputs inputs) {
        inputs.motorTempCelsius = mMotor.getDeviceTemp().getValueAsDouble();
        inputs.motorCurrentAmps = mMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorCurrentVelocity = mMotor.getVelocity().getValueAsDouble();
        //inputs.distanceToNextObject = lc.getMeasurement().distance_mm;
    
    }

    @Override
    public void setPercent(double percent) {
        mMotor.set(percent);
    }


}
