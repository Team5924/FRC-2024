// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.intake;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
    pivotMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    pivotMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    CurrentLimitsConfigs pivotCurrentLimitsConfigs = new CurrentLimitsConfigs();
    pivotCurrentLimitsConfigs.SupplyCurrentLimit = 40;
    pivotCurrentLimitsConfigs.SupplyCurrentThreshold = 40;
    pivotCurrentLimitsConfigs.SupplyTimeThreshold = 0;
    pivotCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;

    FeedbackConfigs pivotFeedbackConfigs = new FeedbackConfigs();
    pivotFeedbackConfigs.SensorToMechanismRatio = IntakeConstants.kEncoderToPivotRatio;

    VoltageConfigs pivotVoltageConfigs = new VoltageConfigs();
    pivotVoltageConfigs.PeakForwardVoltage = IntakeConstants.kPivotPeakVoltage;
    pivotVoltageConfigs.PeakReverseVoltage = -IntakeConstants.kPivotPeakVoltage;

    Slot0Configs pivotSlot0Configs = new Slot0Configs();
    pivotSlot0Configs.kP = IntakeConstants.kPivotKP;
    pivotSlot0Configs.kI = 0;
    pivotSlot0Configs.kD = 0;

    pivotTalon.getConfigurator().apply(
      new TalonFXConfiguration()
        .withMotorOutput(pivotMotorOutputConfigs)
        .withCurrentLimits(pivotCurrentLimitsConfigs)
        .withFeedback(pivotFeedbackConfigs)
        .withVoltage(pivotVoltageConfigs)
        .withSlot0(pivotSlot0Configs)
    );

    pivotTalon.setPosition(0);
  }

  @Override
  public void updateInputs(IntakeIOInputs inputs) {
    inputs.rollerMotorTempCelsius = rollerTalon.getDeviceTemp().getValueAsDouble();
    inputs.rollerMotorCurrentAmps = rollerTalon.getSupplyCurrent().getValueAsDouble();
    inputs.pivotMotorTempCelsius = pivotTalon.getDeviceTemp().getValueAsDouble();
    inputs.pivotMotorCurrentAmps = pivotTalon.getSupplyCurrent().getValueAsDouble();
    inputs.pivotAngleDegrees = pivotTalon.getPosition().getValueAsDouble() * 360;
    SmartDashboard.putNumber("Intake Pivot Rotations", pivotTalon.getPosition().getValueAsDouble());
  }

  @Override
  public void setRollerVoltage(double volts) {
    rollerTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setPivotPosition(double degrees) {
    double rotations = degrees / 360;
    pivotTalon.setControl(positionVoltage.withPosition(rotations));
    SmartDashboard.putNumber("Rotations Goal", rotations);
    SmartDashboard.putNumber("Volts", pivotTalon.getMotorVoltage().getValueAsDouble());
  }

  @Override
  public void setPivotVoltage(double volts) {
    pivotTalon.setControl(voltageOut.withOutput(volts));
  }
}