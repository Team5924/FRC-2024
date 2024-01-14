// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023swerve.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.first5924.frc2023swerve.constants.IntakeConstants;

/** Add your docs here. */
public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX intakeTalon = new TalonFX(IntakeConstants.kTalonId);

  public IntakeIOTalonFX() {
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    talonFXConfiguration.MotorOutput = motorOutputConfigs;

    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.SupplyCurrentLimit = 40;
    currentLimitsConfigs.SupplyCurrentThreshold = 40;
    currentLimitsConfigs.SupplyTimeThreshold = 0;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits = currentLimitsConfigs;

    intakeTalon.getConfigurator().apply(talonFXConfiguration);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.supplyCurrent = intakeTalon.getSupplyCurrent().getValue();
  }

  @Override
  public void setVoltage(double voltage) {
    intakeTalon.setVoltage(voltage);
    ;
  }
}
