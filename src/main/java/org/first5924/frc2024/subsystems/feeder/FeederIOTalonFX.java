// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.feeder;

import org.first5924.frc2024.constants.FeederConstants;
import org.first5924.frc2024.constants.RobotConstants;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.LaserCan.RangingMode;
import au.grapplerobotics.LaserCan.RegionOfInterest;
import au.grapplerobotics.LaserCan.TimingBudget;
import edu.wpi.first.wpilibj.DigitalInput;

/** Add your docs here. */
public class FeederIOTalonFX implements FeederIO {
  private final TalonFX talon = new TalonFX(FeederConstants.kTalonId);
  private LaserCan laserCan;
  private final DigitalInput limitSwitch = new DigitalInput(FeederConstants.kLimitSwitchChannel);

  private final DutyCycleOut dutyCycleOut = new DutyCycleOut(0).withEnableFOC(true);

  public FeederIOTalonFX() {
    try {
      laserCan = new LaserCan(FeederConstants.kLaserCanId);
      laserCan.setRangingMode(RangingMode.SHORT);
      laserCan.setRegionOfInterest(new RegionOfInterest(8, 8, 4, 4));
      laserCan.setTimingBudget(TimingBudget.TIMING_BUDGET_20MS);
    } catch (ConfigurationFailedException e) {
      System.out.println("Configuration failed! " + e);
    }

    MotorOutputConfigs feederMotorOutputConfigs = new MotorOutputConfigs();
    feederMotorOutputConfigs.Inverted = InvertedValue.Clockwise_Positive;
    feederMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    CurrentLimitsConfigs feederCurrentLimitsConfigs = new CurrentLimitsConfigs();
    feederCurrentLimitsConfigs.SupplyCurrentLimit = 40;
    feederCurrentLimitsConfigs.SupplyCurrentThreshold = 40;
    feederCurrentLimitsConfigs.SupplyTimeThreshold = 0;
    feederCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    feederCurrentLimitsConfigs.StatorCurrentLimit = 80;

    talon.getConfigurator().apply(
      new TalonFXConfiguration()
        .withMotorOutput(feederMotorOutputConfigs)
        .withCurrentLimits(feederCurrentLimitsConfigs)
        .withClosedLoopRamps(RobotConstants.kClosedLoopRampsConfigs)
        .withOpenLoopRamps(RobotConstants.kOpenLoopRampsConfigs)
    );
  }

  @Override
  public void updateInputs(FeederIOInputs inputs) {
    inputs.motorTempCelsius = talon.getDeviceTemp().getValueAsDouble();
    inputs.motorCurrentAmps = talon.getSupplyCurrent().getValueAsDouble();
    inputs.motorAppliedVolts = talon.getMotorVoltage().getValueAsDouble();
    inputs.motorVelocityRotationsPerSecond = talon.getVelocity().getValueAsDouble();
    inputs.limitSwitchTriggered = limitSwitch.get();
    LaserCan.Measurement measurement = laserCan.getMeasurement();
    if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
      inputs.laserCanMeasurementMillimeters = measurement.distance_mm;
    } else {
      inputs.laserCanMeasurementMillimeters = 400;
    }
  }

  @Override
  public void setPercent(double percent) {
    talon.setControl(dutyCycleOut.withOutput(percent));
  }
}
