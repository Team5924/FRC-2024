// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.shooter;

import org.first5924.frc2024.constants.ShooterConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

/** Add your docs here. */
public class ShooterIOTalonFX implements ShooterIO {
  private final TalonFX upperTalon = new TalonFX(ShooterConstants.upperMotorID);
  private final TalonFX lowerTalon = new TalonFX(ShooterConstants.lowerMotorID);

  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withEnableFOC(true);

  public ShooterIOTalonFX() {
    MotorOutputConfigs upperMotorOutputConfigs = new MotorOutputConfigs();
    upperMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    upperMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    CurrentLimitsConfigs bothMotorsCurrentLimitsConfigs = new CurrentLimitsConfigs();
    bothMotorsCurrentLimitsConfigs.SupplyCurrentLimit = 40;
    bothMotorsCurrentLimitsConfigs.SupplyCurrentThreshold = 40;
    bothMotorsCurrentLimitsConfigs.SupplyTimeThreshold = 0.1;
    bothMotorsCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;

    upperTalon.getConfigurator().apply(
      new TalonFXConfiguration()
        .withMotorOutput(upperMotorOutputConfigs)
        .withCurrentLimits(bothMotorsCurrentLimitsConfigs)
    );

    MotorOutputConfigs lowerMotorOutputConfigs = new MotorOutputConfigs();
    lowerMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    lowerMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
  }

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    inputs.upperMotorTempCelsius = upperTalon.getDeviceTemp().getValueAsDouble();
    inputs.lowerMotorTempCelsius = lowerTalon.getDeviceTemp().getValueAsDouble();
    inputs.upperMotorCurrentAmps = upperTalon.getSupplyCurrent().getValueAsDouble();
    inputs.lowerMotorCurrentAmps = lowerTalon.getSupplyCurrent().getValueAsDouble();
    inputs.upperMotorVelocityRotationsPerSecond = upperTalon.getVelocity().getValueAsDouble();
    inputs.lowerMotorVelocityRotationsPerSecond = lowerTalon.getVelocity().getValueAsDouble();
  }

  @Override
  public void setPercent(double percent) {
    upperTalon.setControl(dutyCycleOut.withOutput(percent));
    lowerTalon.setControl(dutyCycleOut.withOutput(percent));
  }
}