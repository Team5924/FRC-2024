// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.first5924.frc2024.constants.IntakeConstants;

/** Add your docs here. */
public class IntakeIOTalonFX implements IntakeIO {
  private final TalonFX rollerTalon = new TalonFX(IntakeConstants.kRollerTalonId);
  private final TalonFX pivotTalon = new TalonFX(IntakeConstants.kPivotTalonId);

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final PositionVoltage positionVoltage = new PositionVoltage(0).withEnableFOC(true).withSlot(0);

  public IntakeIOTalonFX() {
    MotorOutputConfigs rollerMotorOutputConfigs = new MotorOutputConfigs();
    rollerMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerMotorOutputConfigs.NeutralMode = NeutralModeValue.Coast;

    CurrentLimitsConfigs rollerCurrentLimitsConfigs = new CurrentLimitsConfigs();
    rollerCurrentLimitsConfigs.SupplyCurrentLimit = 40;
    rollerCurrentLimitsConfigs.SupplyCurrentThreshold = 40;
    rollerCurrentLimitsConfigs.SupplyTimeThreshold = 0;
    rollerCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;

    rollerTalon.getConfigurator().apply(
      new TalonFXConfiguration()
        .withMotorOutput(rollerMotorOutputConfigs)
        .withCurrentLimits(rollerCurrentLimitsConfigs)
    );

    MotorOutputConfigs pivotMotorOutputConfigs = new MotorOutputConfigs();
    rollerMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    rollerMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    CurrentLimitsConfigs pivotCurrentLimitsConfigs = new CurrentLimitsConfigs();
    rollerCurrentLimitsConfigs.SupplyCurrentLimit = 40;
    rollerCurrentLimitsConfigs.SupplyCurrentThreshold = 40;
    rollerCurrentLimitsConfigs.SupplyTimeThreshold = 0;
    rollerCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;

    FeedbackConfigs pivotFeedbackConfigs = new FeedbackConfigs();
    pivotFeedbackConfigs.SensorToMechanismRatio = IntakeConstants.kEncoderToPivotRatio;

    Slot0Configs pivotSlot0Configs = new Slot0Configs();
    pivotSlot0Configs.kP = IntakeConstants.kPivotKp;

    pivotTalon.getConfigurator().apply(
      new TalonFXConfiguration()
        .withMotorOutput(pivotMotorOutputConfigs)
        .withCurrentLimits(pivotCurrentLimitsConfigs)
        .withFeedback(pivotFeedbackConfigs)
        .withSlot0(pivotSlot0Configs)
    );
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerMotorTempCelsius = rollerTalon.getDeviceTemp().getValueAsDouble();
    inputs.rollerMotorCurrentAmps = rollerTalon.getSupplyCurrent().getValueAsDouble();
    inputs.pivotMotorTempCelsius = pivotTalon.getDeviceTemp().getValueAsDouble();
    inputs.pivotMotorCurrentAmps = pivotTalon.getSupplyCurrent().getValueAsDouble();
    inputs.pivotAngleDegrees = pivotTalon.getPosition().getValueAsDouble() / 360;
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setPivotPosition(double degrees) {
    double rotations = degrees * 360;
    pivotTalon.setControl(positionVoltage.withPosition(rotations));
  }
}