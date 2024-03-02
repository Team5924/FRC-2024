// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.first5924.frc2024.subsystems.drive;

import org.first5924.frc2024.constants.DriveConstants;
import org.first5924.frc2024.constants.RobotConstants;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.util.Units;

public class ModuleIOTalonFX implements ModuleIO {
  private final TalonFX driveTalon;
  private final TalonFX turnTalon;
  private final CANcoder turnCanCoder;

  private final boolean isDriveMotorInverted;
  private final boolean isTurnMotorInverted;
  private final double absoluteEncoderOffsetRad;

  private final StatusSignal<Double> drivePosition;
  private final StatusSignal<Double> driveVelocity;
  private final StatusSignal<Double> driveAppliedVolts;
  private final StatusSignal<Double> driveCurrentAmps;
  private final StatusSignal<Double> driveTempCelsius;

  private final StatusSignal<Double> turnAbsolutePosition;
  private final StatusSignal<Double> turnVelocity;
  private final StatusSignal<Double> turnAppliedVolts;
  private final StatusSignal<Double> turnCurrentAmps;
  private final StatusSignal<Double> turnTempCelsius;

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);

  public ModuleIOTalonFX(int index) {
    switch (index) {
      case 0:
        driveTalon = new TalonFX(DriveConstants.kLeftFrontDriveTalonId, "drive");
        turnTalon = new TalonFX(DriveConstants.kLeftFrontTurnTalonId, "drive");
        turnCanCoder = new CANcoder(DriveConstants.kLeftFrontCanCoderId, "drive");
        isDriveMotorInverted = false;
        isTurnMotorInverted = true;
        absoluteEncoderOffsetRad = DriveConstants.kLeftFrontCanCoderOffsetRad;
        break;
      case 1:
        driveTalon = new TalonFX(DriveConstants.kRightFrontDriveTalonId, "drive");
        turnTalon = new TalonFX(DriveConstants.kRightFrontTurnTalonId, "drive");
        turnCanCoder = new CANcoder(DriveConstants.kRightFrontCanCoderId, "drive");
        isDriveMotorInverted = true;
        isTurnMotorInverted = true;
        absoluteEncoderOffsetRad = DriveConstants.kRightFrontCanCoderOffsetRad;
        break;
      case 2:
        driveTalon = new TalonFX(DriveConstants.kLeftBackDriveTalonId, "drive");
        turnTalon = new TalonFX(DriveConstants.kLeftBackTurnTalonId, "drive");
        turnCanCoder = new CANcoder(DriveConstants.kLeftBackCanCoderId, "drive");
        isDriveMotorInverted = false;
        isTurnMotorInverted = true;
        absoluteEncoderOffsetRad = DriveConstants.kLeftBackCanCoderOffsetRad;
        break;
      case 3:
        driveTalon = new TalonFX(DriveConstants.kRightBackDriveTalonId, "drive");
        turnTalon = new TalonFX(DriveConstants.kRightBackTurnTalonId, "drive");
        turnCanCoder = new CANcoder(DriveConstants.kRightBackCanCoderId, "drive");
        isDriveMotorInverted = true;
        isTurnMotorInverted = true;
        absoluteEncoderOffsetRad = DriveConstants.kRightBackCanCoderOffsetRad;
        break;
      default:
        throw new RuntimeException("Invalid module index for ModuleIOTalonFX");
    }

    MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
    magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    magnetSensorConfigs.MagnetOffset = Units.radiansToRotations(absoluteEncoderOffsetRad);
    turnCanCoder.getConfigurator().apply(magnetSensorConfigs);

    CurrentLimitsConfigs driveCurrentLimitsConfigs = new CurrentLimitsConfigs();
    driveCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    driveCurrentLimitsConfigs.SupplyCurrentLimit = 40;
    driveCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
    driveCurrentLimitsConfigs.StatorCurrentLimit = 80;

    FeedbackConfigs driveFeedbackConfigs = new FeedbackConfigs();
    driveFeedbackConfigs.SensorToMechanismRatio = DriveConstants.kEncoderToDriveRatio;

    driveTalon.getConfigurator().apply(
      new TalonFXConfiguration()
        .withCurrentLimits(driveCurrentLimitsConfigs)
        .withFeedback(driveFeedbackConfigs)
        .withClosedLoopRamps(RobotConstants.kClosedLoopRampsConfigs)
        .withOpenLoopRamps(RobotConstants.kOpenLoopRampsConfigs)
    );
    setDriveBrakeMode(true);

    CurrentLimitsConfigs turnCurrentLimitsConfigs = new CurrentLimitsConfigs();
    turnCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    turnCurrentLimitsConfigs.SupplyCurrentLimit = 30;
    turnCurrentLimitsConfigs.StatorCurrentLimitEnable = true;
    turnCurrentLimitsConfigs.StatorCurrentLimit = 80;

    FeedbackConfigs turnFeedbackConfigs = new FeedbackConfigs();
    turnFeedbackConfigs.FeedbackRemoteSensorID = turnCanCoder.getDeviceID();
    turnFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnFeedbackConfigs.RotorToSensorRatio = DriveConstants.kEncoderToTurnRatio;
    turnFeedbackConfigs.SensorToMechanismRatio = 1;

    turnTalon.getConfigurator().apply(
      new TalonFXConfiguration()
        .withCurrentLimits(turnCurrentLimitsConfigs)
        .withFeedback(turnFeedbackConfigs)
        .withOpenLoopRamps(RobotConstants.kOpenLoopRampsConfigs)
        .withClosedLoopRamps(RobotConstants.kClosedLoopRampsConfigs)
    );
    setTurnBrakeMode(true);

    drivePosition = driveTalon.getPosition();
    driveVelocity = driveTalon.getVelocity();
    driveAppliedVolts = driveTalon.getMotorVoltage();
    driveCurrentAmps = driveTalon.getSupplyCurrent();
    driveTempCelsius = driveTalon.getDeviceTemp();

    turnAbsolutePosition = turnCanCoder.getAbsolutePosition();
    turnVelocity = turnTalon.getVelocity();
    turnAppliedVolts = turnTalon.getMotorVoltage();
    turnCurrentAmps = turnTalon.getSupplyCurrent();
    turnTempCelsius = turnTalon.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
      100.0, drivePosition, turnAbsolutePosition); // Required for odometry, use faster rate
    BaseStatusSignal.setUpdateFrequencyForAll(
      50.0,
      driveVelocity,
      driveAppliedVolts,
      driveCurrentAmps,
      driveTempCelsius,
      turnVelocity,
      turnAppliedVolts,
      turnCurrentAmps,
      turnTempCelsius
    );
    driveTalon.optimizeBusUtilization();
    turnTalon.optimizeBusUtilization();
  }

  public void updateInputs(ModuleIOInputs inputs) {
    BaseStatusSignal.refreshAll(
      drivePosition,
      driveVelocity,
      driveAppliedVolts,
      driveCurrentAmps,
      driveTempCelsius,
      turnAbsolutePosition,
      turnVelocity,
      turnAppliedVolts,
      turnCurrentAmps,
      turnTempCelsius
    );

    inputs.drivePositionRad = Units.rotationsToRadians(drivePosition.getValueAsDouble());
    inputs.driveVelocityRadPerSec = Units.rotationsToRadians(driveVelocity.getValueAsDouble());
    inputs.driveCurrentAmps = driveCurrentAmps.getValueAsDouble();
    inputs.driveAppliedVolts = driveAppliedVolts.getValueAsDouble();
    inputs.driveTempCelcius = driveTempCelsius.getValueAsDouble();

    inputs.turnAbsolutePositionRad = Units.rotationsToRadians(turnAbsolutePosition.getValueAsDouble());
    inputs.turnVelocityRadPerSec = Units.rotationsToRadians(turnVelocity.getValueAsDouble());
    inputs.turnCurrentAmps = turnCurrentAmps.getValueAsDouble();
    inputs.turnAppliedVolts = turnAppliedVolts.getValueAsDouble();
    inputs.turnTempCelcius = turnTempCelsius.getValueAsDouble();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveTalon.setControl(voltageOut.withOutput(volts));
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnTalon.setControl(voltageOut.withOutput(volts));
  }

   @Override
  public void setDriveBrakeMode(boolean brake) {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = isDriveMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(motorOutputConfigs);
  }

  @Override
  public void setTurnBrakeMode(boolean brake) {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = isTurnMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(motorOutputConfigs);
  }
}
