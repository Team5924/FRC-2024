// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023swerve.subsystems.pivot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.first5924.frc2023swerve.constants.PivotConstants;

/** Add your docs here. */
public class PivotIOTalonFX implements PivotIO {
  private final TalonFX pivotTalon = new TalonFX(PivotConstants.kTalonId);

  public PivotIOTalonFX() {
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    talonFXConfiguration.MotorOutput = motorOutputConfigs;

    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.SupplyCurrentLimit = 20;
    currentLimitsConfigs.SupplyCurrentThreshold = 20;
    currentLimitsConfigs.SupplyTimeThreshold = 0;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits = currentLimitsConfigs;

    pivotTalon.getConfigurator().apply(talonFXConfiguration);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotPositionDegrees =
        pivotTalon.getPosition().getValue() * 360 / PivotConstants.kGearRatio;
    inputs.pivotVelocityDegreesPerSecond =
        pivotTalon.getVelocity().getValue() * 360 / PivotConstants.kGearRatio;
    inputs.supplyCurrent = pivotTalon.getSupplyCurrent().getValue();
  }

  @Override
  public void setVoltage(double volts) {
    pivotTalon.setVoltage(volts);
  }

  @Override
  public void setEncoderPosition(double position) {
    pivotTalon.setPosition(position);
  }

  @Override
  public void setBrakeMode(boolean enable) {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = enable ? NeutralModeValue.Brake : NeutralModeValue.Coast;

    pivotTalon.getConfigurator().apply(motorOutputConfigs);
  }
}
