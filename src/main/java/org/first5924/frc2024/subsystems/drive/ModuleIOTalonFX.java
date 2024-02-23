// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.first5924.frc2024.subsystems.drive;

import org.first5924.frc2024.constants.DriveConstants;

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

  private final CANcoder turnAbsoluteEncoder;

  private final boolean isDriveMotorInverted;
  private final boolean isTurnMotorInverted;
  private final double absoluteEncoderOffsetRad;

  private final VoltageOut voltageOut = new VoltageOut(0).withEnableFOC(true);

  public ModuleIOTalonFX(int index) {
    switch (index) {
      case 0:
        driveTalon = new TalonFX(DriveConstants.kLeftFrontDriveSparkId, "drive");
        turnTalon = new TalonFX(DriveConstants.kLeftFrontTurnSparkId, "drive");
        turnAbsoluteEncoder = new CANcoder(DriveConstants.kLeftFrontCANCoderId, "drive");
        isDriveMotorInverted = false;
        isTurnMotorInverted = true;
        absoluteEncoderOffsetRad = DriveConstants.kLeftFrontAbsoluteEncoderOffsetRad;
        break;
      case 1:
        driveTalon = new TalonFX(DriveConstants.kRightFrontDriveSparkId, "drive");
        turnTalon = new TalonFX(DriveConstants.kRightFrontTurnSparkId, "drive");
        turnAbsoluteEncoder = new CANcoder(DriveConstants.kRightFrontCANCoderId, "drive");
        isDriveMotorInverted = true;
        isTurnMotorInverted = true;
        absoluteEncoderOffsetRad = DriveConstants.kRightFrontAbsoluteEncoderOffsetRad;
        break;
      case 2:
        driveTalon = new TalonFX(DriveConstants.kLeftBackDriveSparkId, "drive");
        turnTalon = new TalonFX(DriveConstants.kLeftBackTurnSparkId, "drive");
        turnAbsoluteEncoder = new CANcoder(DriveConstants.kLeftBackCANCoderId, "drive");
        isDriveMotorInverted = false;
        isTurnMotorInverted = true;
        absoluteEncoderOffsetRad = DriveConstants.kLeftBackAbsoluteEncoderOffsetRad;
        break;
      case 3:
        driveTalon = new TalonFX(DriveConstants.kRightBackDriveSparkId, "drive");
        turnTalon = new TalonFX(DriveConstants.kRightBackTurnSparkId, "drive");
        turnAbsoluteEncoder = new CANcoder(DriveConstants.kRightBackCANCoderId, "drive");
        isDriveMotorInverted = true;
        isTurnMotorInverted = true;
        absoluteEncoderOffsetRad = DriveConstants.kRightBackAbsoluteEncoderOffsetRad;
        break;
      default:
        throw new RuntimeException("Invalid module index for ModuleIOSparkMax");
    }

    MagnetSensorConfigs magnetSensorConfigs = new MagnetSensorConfigs();
    magnetSensorConfigs.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    magnetSensorConfigs.MagnetOffset = -Units.radiansToRotations(absoluteEncoderOffsetRad);
    turnAbsoluteEncoder.getConfigurator().apply(magnetSensorConfigs);

    MotorOutputConfigs driveMotorOutputConfigs = new MotorOutputConfigs();
    driveMotorOutputConfigs.Inverted = isDriveMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    driveMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    CurrentLimitsConfigs driveCurrentLimitsConfigs = new CurrentLimitsConfigs();
    driveCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    driveCurrentLimitsConfigs.SupplyCurrentLimit = 40;
    driveCurrentLimitsConfigs.SupplyCurrentThreshold = 45;
    driveCurrentLimitsConfigs.SupplyTimeThreshold = 0.15;

    FeedbackConfigs driveFeedbackConfigs = new FeedbackConfigs();
    driveFeedbackConfigs.SensorToMechanismRatio = DriveConstants.kEncoderToDriveRatio;

    driveTalon.getConfigurator().apply(
      new TalonFXConfiguration()
        .withMotorOutput(driveMotorOutputConfigs)
        .withCurrentLimits(driveCurrentLimitsConfigs)
        .withFeedback(driveFeedbackConfigs)
    );

    MotorOutputConfigs turnMotorOutputConfigs = new MotorOutputConfigs();
    turnMotorOutputConfigs.Inverted = isTurnMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    turnMotorOutputConfigs.NeutralMode = NeutralModeValue.Brake;

    CurrentLimitsConfigs turnCurrentLimitsConfigs = new CurrentLimitsConfigs();
    turnCurrentLimitsConfigs.SupplyCurrentLimitEnable = true;
    turnCurrentLimitsConfigs.SupplyCurrentLimit = 30;
    turnCurrentLimitsConfigs.SupplyCurrentThreshold = 35;
    turnCurrentLimitsConfigs.SupplyTimeThreshold = 0.15;

    FeedbackConfigs turnFeedbackConfigs = new FeedbackConfigs();
    turnFeedbackConfigs.FeedbackRemoteSensorID = turnAbsoluteEncoder.getDeviceID();
    turnFeedbackConfigs.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    turnFeedbackConfigs.RotorToSensorRatio = 1;
    turnFeedbackConfigs.SensorToMechanismRatio = DriveConstants.kEncoderToTurnRatio;

    turnTalon.getConfigurator().apply(
      new TalonFXConfiguration()
        .withMotorOutput(turnMotorOutputConfigs)
        .withCurrentLimits(turnCurrentLimitsConfigs)
        .withFeedback(turnFeedbackConfigs)
    );
  }

  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionRad = driveTalon.getPosition().getValueAsDouble();
    inputs.driveVelocityRadPerSec = driveTalon.getVelocity().getValueAsDouble();
    inputs.driveCurrentAmps = driveTalon.getSupplyCurrent().getValueAsDouble();
    inputs.driveTempCelcius = driveTalon.getDeviceTemp().getValueAsDouble();

    inputs.turnAbsolutePositionRad = Units.rotationsToRadians(turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble());
    inputs.turnCurrentAmps = turnTalon.getSupplyCurrent().getValueAsDouble();
    inputs.turnTempCelcius = turnTalon.getDeviceTemp().getValueAsDouble();
  }

  public void setDriveVoltage(double volts) {
    driveTalon.setControl(voltageOut.withOutput(volts));
  }

  public void setTurnVoltage(double volts) {
    turnTalon.setControl(voltageOut.withOutput(volts));
  }

  public void setDriveBrakeMode(boolean brake) {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = isDriveMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    driveTalon.getConfigurator().apply(motorOutputConfigs);
  }

  public void setTurnBrakeMode(boolean brake) {
    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = isTurnMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = brake ? NeutralModeValue.Brake : NeutralModeValue.Coast;
    turnTalon.getConfigurator().apply(motorOutputConfigs);
  }
}
