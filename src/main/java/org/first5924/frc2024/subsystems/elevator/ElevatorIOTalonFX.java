// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.elevator;

import org.first5924.frc2024.constants.ElevatorConstants;
import org.first5924.frc2024.constants.RobotConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;

/** Add your docs here. */
public class ElevatorIOTalonFX implements ElevatorIO {
  private final TalonFX leftTalon = new TalonFX(ElevatorConstants.kLeftTalonId);
  private final TalonFX rightTalon = new TalonFX(ElevatorConstants.kRightTalonId);
  private LaserCan laserCan;

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final PositionVoltage positionVoltage = new PositionVoltage(0).withEnableFOC(true).withSlot(0);

  public ElevatorIOTalonFX() {
    MotorOutputConfigs leftMotorOutputConfigs = new MotorOutputConfigs();
    leftMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    leftMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    FeedbackConfigs leftFeedbackConfigs = new FeedbackConfigs();
    leftFeedbackConfigs.SensorToMechanismRatio = ElevatorConstants.kEncoderToSpoolRatio;

    CurrentLimitsConfigs bothCurrentLimitsConfigs = new CurrentLimitsConfigs();
    bothCurrentLimitsConfigs.SupplyCurrentLimit = 40;
    bothCurrentLimitsConfigs.SupplyCurrentThreshold = 40;
    bothCurrentLimitsConfigs.SupplyTimeThreshold = 0;
    bothCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    bothCurrentLimitsConfigs.StatorCurrentLimit = 80;

    VoltageConfigs bothVoltageConfigs = new VoltageConfigs();
    bothVoltageConfigs.PeakForwardVoltage = ElevatorConstants.kPeakVoltage;
    bothVoltageConfigs.PeakReverseVoltage = -ElevatorConstants.kPeakVoltage;

    Slot0Configs slot0Configs = new Slot0Configs();
    slot0Configs.kP = ElevatorConstants.kP;

    leftTalon.getConfigurator().apply(
      new TalonFXConfiguration()
        .withMotorOutput(leftMotorOutputConfigs)
        .withCurrentLimits(bothCurrentLimitsConfigs)
        .withFeedback(leftFeedbackConfigs)
        .withVoltage(bothVoltageConfigs)
        .withSlot0(slot0Configs)
        .withClosedLoopRamps(RobotConstants.closedLoopRampsConfigs)
        .withOpenLoopRamps(RobotConstants.openLoopRampsConfigs)
    );

    MotorOutputConfigs rightMotorOutputConfigs = new MotorOutputConfigs();
    rightMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    rightMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    rightTalon.getConfigurator().apply(
      new TalonFXConfiguration()
        .withMotorOutput(rightMotorOutputConfigs)
        .withCurrentLimits(bothCurrentLimitsConfigs)
        .withVoltage(bothVoltageConfigs)
    );

    try {
      laserCan = new LaserCan(ElevatorConstants.kLaserCanId);
      laserCan.setRangingMode(RangingMode.SHORT);
      laserCan.setRegionOfInterest(new RegionOfInterest(8, 8, 6, 6));
      laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_100MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftMotorTempCelsius = leftTalon.getDeviceTemp().getValueAsDouble();
    inputs.leftMotorCurrentAmps = leftTalon.getSupplyCurrent().getValueAsDouble();
    inputs.rightMotorTempCelsius = rightTalon.getDeviceTemp().getValueAsDouble();
    inputs.rightMotorCurrentAmps = rightTalon.getSupplyCurrent().getValueAsDouble();
    inputs.drumPosition = leftTalon.getPosition().getValueAsDouble();
    inputs.elevatorHeightMeters = leftTalon.getPosition().getValueAsDouble() * ElevatorConstants.kSpoolCircumferenceMeters;
    LaserCan.Measurement measurement = laserCan.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      inputs.laserCanMillimetersAboveAtLowest = measurement.distance_mm - ElevatorConstants.kLaserCanReadingAtLowestMillimeters;
    } else {
      inputs.laserCanMillimetersAboveAtLowest = -1;
    }
  }

  @Override
  public void setElevatorHeight(double meters) {
    double spoolRotations = meters / ElevatorConstants.kSpoolCircumferenceMeters;
    leftTalon.setControl(positionVoltage.withPosition(spoolRotations));
    rightTalon.setControl(new StrictFollower(leftTalon.getDeviceID()));
  }

  @Override
  public void setVoltage(double volts) {
    leftTalon.setControl(voltageOut.withOutput(volts));
    rightTalon.setControl(new StrictFollower(leftTalon.getDeviceID()));
  }

  @Override
  public void setEncoder(double spoolRotations) {
    leftTalon.setPosition(spoolRotations);
  }
}