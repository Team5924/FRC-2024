// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class DriveConstants {
  public static final int kLeftFrontDriveTalonId = 14;
  public static final int kLeftFrontTurnTalonId = 12;

  public static final int kRightFrontDriveTalonId = 3;
  public static final int kRightFrontTurnTalonId = 5;

  public static final int kLeftBackDriveTalonId = 11;
  public static final int kLeftBackTurnTalonId = 9;

  public static final int kRightBackDriveTalonId = 8;
  public static final int kRightBackTurnTalonId = 6;

  public static final int kLeftFrontCanCoderId = 13;
  public static final int kRightFrontCanCoderId = 4;
  public static final int kLeftBackCanCoderId = 10;
  public static final int kRightBackCanCoderId = 7;

  public static final double kLeftFrontCanCoderOffsetRad = -2.469;
  public static final double kRightFrontCanCoderOffsetRad = -0.2347;
  public static final double kLeftBackCanCoderOffsetRad = 1.6904;
  public static final double kRightBackCanCoderOffsetRad = -2.1514;

  public static final double kTrackWidthX = Units.inchesToMeters(22.75);
  public static final double kTrackWidthY = Units.inchesToMeters(22.75);

  public static final double kMaxLinearSpeed = Units.feetToMeters(16.0);
  public static final double kMaxAngularSpeedRad = kMaxLinearSpeed / Math.hypot(kTrackWidthX / 2, kTrackWidthY / 2);
  public static final double kAngularSpeedMultiplier = 0.45;

  public static final double kWheelRadius = Units.inchesToMeters(2.0);

  public static final double kDriveKp = 0.1;
  public static final double kDriveKd = 0.0;
  public static final double kDriveKs = 0.18868;
  public static final double kDriveKv = 0.12825;
  public static final double kTurnKp = 3.5;
  public static final double kTurnKd = 0;

  public static final double kEncoderToDriveRatio = 5.14;
  public static final double kEncoderToTurnRatio = 396/35;

  public static final double kSlowModeMovementMultiplier = 0.1;
  public static final double kSlowModeRotationMultiplier = 0.1;
}
