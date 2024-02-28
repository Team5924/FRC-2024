// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.wrist;

import org.first5924.frc2024.constants.WristConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/** Add your docs here. */
public class WristIOTalonFX implements WristIO {
  private final TalonFX talon = new TalonFX(WristConstants.kTalonId);
  private final CANcoder canCoder = new CANcoder(WristConstants.kCanCoderId);

  private final PositionVoltage positionVoltage = new PositionVoltage(0).withEnableFOC(true).withSlot(0);
  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);

  public WristIOTalonFX() {
    MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
    magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    magnetSensorConfigs.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    magnetSensorConfigs.MagnetOffset = WristConstants.kCanCoderOffset;
    canCoder.getConfigurator().apply(magnetSensorConfigs);

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.SupplyCurrentLimit = 40;
    currentLimitsConfigs.SupplyCurrentThreshold = 40;
    currentLimitsConfigs.SupplyTimeThreshold = 0;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;

    FeedbackConfigs feedbackConfigs = new FeedbackConfigs();
    feedbackConfigs.FeedbackRemoteSensorID = canCoder.getDeviceID();
    feedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    feedbackConfigs.SensorToMechanismRatio = 1.0;
    feedbackConfigs.RotorToSensorRatio = 208.33;

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = WristConstants.kP;

    talon.getConfigurator().apply(
      new TalonFXConfiguration()
        .withMotorOutput(motorOutputConfigs)
        .withCurrentLimits(currentLimitsConfigs)
        .withFeedback(feedbackConfigs)
        .withSlot0(slot0Configs)
    );

    talon.setPosition(0);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.motorTempCelsius = talon.getDeviceTemp().getValueAsDouble();
    inputs.motorCurrentAmps = talon.getSupplyCurrent().getValueAsDouble();
    inputs.wristAngleDegrees = talon.getPosition().getValueAsDouble() * 360;
    SmartDashboard.putNumber("CANCoder Wrist", canCoder.getAbsolutePosition().getValueAsDouble());
  }

  @Override
  public void setAngle(double degrees){
    double rotations = degrees / 360;
    talon.setControl(positionVoltage.withPosition(rotations));
  }

  @Override
  public void setVoltage(double volts) {
    talon.setControl(voltageOut.withOutput(volts));
  }
}

