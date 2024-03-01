// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.elevator;

import org.first5924.frc2024.constants.ElevatorConstants;
import org.first5924.frc2024.constants.RobotConstants;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;

/** Add your docs here. */
public class ElevatorIOTalonFX implements ElevatorIO {
  // Leader
  private final TalonFX leftTalon = new TalonFX(ElevatorConstants.kLeftTalonId);
  // Follower
  private final TalonFX rightTalon = new TalonFX(ElevatorConstants.kRightTalonId);
  private LaserCan laserCan;

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);
  private final PositionVoltage positionVoltage = new PositionVoltage(0).withEnableFOC(true).withSlot(0);

  public ElevatorIOTalonFX() {
    MotorOutputConfigs bothMotorOutputConfigs = new MotorOutputConfigs();
    bothMotorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    bothMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    FeedbackConfigs bothFeedbackConfigs = new FeedbackConfigs();
    bothFeedbackConfigs.SensorToMechanismRatio = ElevatorConstants.kEncoderToSpoolRatio;

    CurrentLimitsConfigs bothCurrentLimitsConfigs = new CurrentLimitsConfigs();
    bothCurrentLimitsConfigs.SupplyCurrentLimit = 40;
    bothCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    bothCurrentLimitsConfigs.StatorCurrentLimit = 80;

    VoltageConfigs bothVoltageConfigs = new VoltageConfigs();
    bothVoltageConfigs.PeakForwardVoltage = ElevatorConstants.kPeakForwardVoltage;
    bothVoltageConfigs.PeakReverseVoltage = ElevatorConstants.kPeakReverseVoltage ;

    SoftwareLimitSwitchConfigs bothSoftwareLimitSwitchConfigs = new SoftwareLimitSwitchConfigs();
    bothSoftwareLimitSwitchConfigs.ForwardSoftLimitEnable = true;
    bothSoftwareLimitSwitchConfigs.ForwardSoftLimitThreshold = ElevatorConstants.kForwardSoftLimitThreshold;
    bothSoftwareLimitSwitchConfigs.ReverseSoftLimitEnable = true;
    bothSoftwareLimitSwitchConfigs.ReverseSoftLimitThreshold = ElevatorConstants.kReverseSoftLimitThreshold;

    Slot0Configs leftSlot0Configs = new Slot0Configs();
    leftSlot0Configs.kP = ElevatorConstants.kP;
    leftSlot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    leftSlot0Configs.kG = ElevatorConstants.kG;

    leftTalon.getConfigurator().apply(
      new TalonFXConfiguration()
        .withMotorOutput(bothMotorOutputConfigs)
        .withCurrentLimits(bothCurrentLimitsConfigs)
        .withFeedback(bothFeedbackConfigs)
        .withVoltage(bothVoltageConfigs)
        .withSoftwareLimitSwitch(bothSoftwareLimitSwitchConfigs)
        .withSlot0(leftSlot0Configs)
        .withClosedLoopRamps(RobotConstants.kClosedLoopRampsConfigs)
        .withOpenLoopRamps(RobotConstants.kOpenLoopRampsConfigs)
    );

    rightTalon.getConfigurator().apply(
      new TalonFXConfiguration()
        .withMotorOutput(bothMotorOutputConfigs)
        .withCurrentLimits(bothCurrentLimitsConfigs)
        .withFeedback(bothFeedbackConfigs)
        .withVoltage(bothVoltageConfigs)
        .withSoftwareLimitSwitch(bothSoftwareLimitSwitchConfigs)
        .withClosedLoopRamps(RobotConstants.kClosedLoopRampsConfigs)
        .withOpenLoopRamps(RobotConstants.kOpenLoopRampsConfigs)
    );

    leftTalon.setPosition(0);
    rightTalon.setPosition(0);

    try {
      laserCan = new LaserCan(ElevatorConstants.kLaserCanId);
      laserCan.setRangingMode(RangingMode.SHORT);
      laserCan.setRegionOfInterest(new RegionOfInterest(8, 8, 4, 4));
      laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_100MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    inputs.leftMotorTempCelsius = leftTalon.getDeviceTemp().getValueAsDouble();
    inputs.leftMotorCurrentAmps = leftTalon.getSupplyCurrent().getValueAsDouble();
    inputs.leftMotorAppliedVoltage = leftTalon.getMotorVoltage().getValueAsDouble();
    inputs.rightMotorTempCelsius = rightTalon.getDeviceTemp().getValueAsDouble();
    inputs.rightMotorCurrentAmps = rightTalon.getSupplyCurrent().getValueAsDouble();
    inputs.rightMotorAppliedVoltage = rightTalon.getMotorVoltage().getValueAsDouble();
    inputs.drumPosition = leftTalon.getPosition().getValueAsDouble();
    inputs.elevatorHeightMeters = leftTalon.getPosition().getValueAsDouble() * ElevatorConstants.kSpoolCircumferenceMeters;
    LaserCan.Measurement measurement = laserCan.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      inputs.laserCanMillimetersAboveAtLowest = measurement.distance_mm - ElevatorConstants.kLaserCanReadingAtLowestMillimeters;
    } else {
      inputs.laserCanMillimetersAboveAtLowest = -99;
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
    rightTalon.setPosition(spoolRotations);
  }
}