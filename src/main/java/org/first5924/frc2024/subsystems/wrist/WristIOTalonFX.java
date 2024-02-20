// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.wrist;

import org.first5924.frc2024.constants.WristConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class WristIOTalonFX implements WristIO {
    private final TalonFX mMotor = new TalonFX(WristConstants.motorID);
    private final CANcoder mEncoder = new CANcoder(WristConstants.encoderID);
    PIDController wristController = new PIDController(.1, 0, 0);

    public WristIOTalonFX() {
        TalonFXConfiguration mMotor_cfg = new TalonFXConfiguration();
        mMotor_cfg.Feedback.FeedbackRemoteSensorID = mEncoder.getDeviceID();
        mMotor_cfg.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        mMotor_cfg.Feedback.SensorToMechanismRatio = 1.0;
        mMotor_cfg.Feedback.RotorToSensorRatio = 363;
        mMotor.getConfigurator().apply(mMotor_cfg);
  
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.motorTempCelsius = mMotor.getDeviceTemp().getValueAsDouble();
        inputs.motorCurrentAmps = mMotor.getSupplyCurrent().getValueAsDouble();
        inputs.motorCurrentVelocity = mMotor.getVelocity().getValueAsDouble();
        inputs.encoderPosition = mEncoder.getPosition().getValueAsDouble();
        inputs.wristAngle = mMotor.getPosition().getValueAsDouble();
    }

    @Override
    public void setPercent(double percent) {
        mMotor.set(percent);
    }
    public void setVoltage(double voltage) {
        mMotor.setVoltage(voltage);
    }
    public void setAngle(double targetAngle){
        mMotor.setVoltage(wristController.calculate(((mMotor.getPosition().getValueAsDouble()/363)*360)%360, targetAngle));     
    }

}

