// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.feeder;

import org.first5924.frc2024.constants.FeederConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;

/** Add your docs here. */
public class FeederIOTalonFX implements FeederIO {
  private final TalonFX motor = new TalonFX(FeederConstants.talonId);
  private LaserCan laserCan;

  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withEnableFOC(true);

  public FeederIOTalonFX() {
    try {
      laserCan = new LaserCan(FeederConstants.laserCanId);
      laserCan.setRangingMode(RangingMode.SHORT);
      laserCan.setRegionOfInterest(new RegionOfInterest(8, 8, 6, 6));
      laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }
    CurrentLimitsConfigs feederCurrentLimitsConfigs = new CurrentLimitsConfigs();
    feederCurrentLimitsConfigs.SupplyCurrentLimit = 40;
    feederCurrentLimitsConfigs.SupplyCurrentThreshold = 40;
    feederCurrentLimitsConfigs.SupplyTimeThreshold = 0;
    feederCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    feederCurrentLimitsConfigs.StatorCurrentLimit = 80;
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.motorTempCelsius = motor.getDeviceTemp().getValueAsDouble();
    inputs.motorCurrentAmps = motor.getSupplyCurrent().getValueAsDouble();
    inputs.motorVelocityRotationsPerSecond = motor.getVelocity().getValueAsDouble();
    LaserCan.Measurement measurement = laserCan.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      inputs.laserCanMeasurementMillimeters = measurement.distance_mm;
    } else {
      inputs.laserCanMeasurementMillimeters = 400;
    }
  }

  @Override
  public void setPercent(double percent) {
    motor.setControl(dutyCycleOut.withOutput(percent));
  }
}
