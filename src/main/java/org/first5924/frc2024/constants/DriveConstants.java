// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2023swerve.constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveConstants {
  public static final int kLeftFrontDriveSparkId = 1;
  public static final int kLeftFrontTurnSparkId = 2;

  public static final int kRightFrontDriveSparkId = 7;
  public static final int kRightFrontTurnSparkId = 8;

  public static final int kLeftBackDriveSparkId = 3;
  public static final int kLeftBackTurnSparkId = 4;

  public static final int kRightBackDriveSparkId = 5;
  public static final int kRightBackTurnSparkId = 6;

  public static final int kLeftFrontCANCoderId = 9;
  public static final int kRightFrontCANCoderId = 12;
  public static final int kLeftBackCANCoderId = 10;
  public static final int kRightBackCANCoderId = 11;

  public static final double kLeftFrontAbsoluteEncoderOffsetRad = -3.785;
  public static final double kLeftBackAbsoluteEncoderOffsetRad = -1.247;
  public static final double kRightBackAbsoluteEncoderOffsetRad = -1.462;
  public static final double kRightFrontAbsoluteEncoderOffsetRad = -3.635;

  public static final double kTrackWidthX = Units.inchesToMeters(18.25);
  public static final double kTrackWidthY = Units.inchesToMeters(18.25);

  public static final double kMaxLinearSpeed = Units.feetToMeters(16.0);
  public static final double kMaxAngularSpeedRad =
      kMaxLinearSpeed / Math.hypot(kTrackWidthX / 2, kTrackWidthY / 2);
  public static final double kAngularSpeedMultiplier = 0.45;

  public static final double kWheelRadius = Units.inchesToMeters(2.0);

  public static final double kDriveKp = 0.1;
  public static final double kDriveKd = 0.0;
  public static final double kDriveKs = 0.18868;
  public static final double kDriveKv = 0.12825;
  public static final double kTurnKp = 10.0;
  public static final double kTurnKd = 0.0;

  public static final double kEncoderToDriveReduction = 1 / 6.12;
  public static final double kEncoderToTurnReduction = 1 / (150 / 7);
}
