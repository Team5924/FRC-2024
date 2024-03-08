// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.constants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

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

  public static final double kLeftFrontCanCoderOffsetRad = -0.6427;
  public static final double kRightFrontCanCoderOffsetRad = -2.8939;
  public static final double kLeftBackCanCoderOffsetRad = -1.6884;
  public static final double kRightBackCanCoderOffsetRad = -1.0032;

  public static final double kTrackWidthX = Units.inchesToMeters(22.75);
  public static final double kTrackWidthY = Units.inchesToMeters(22.75);

  public static final double kMaxLinearSpeed = Units.feetToMeters(16.0);
  public static final double kMaxAngularSpeedRad = kMaxLinearSpeed / Math.hypot(kTrackWidthX / 2, kTrackWidthY / 2);

  public static final double kWheelRadius = Units.inchesToMeters(2.0);

  public static final double kDriveKp = 0.9;
  public static final double kDriveKd = 0.0;
  public static final double kDriveKs = 0.17957;
  public static final double kDriveKv = 2.1474;
  public static final double kTurnKp = 3.5;
  public static final double kTurnKd = 0;

  public static final double kRobotRotationKp = 0.8;

  public static final double kEncoderToDriveRatio = 5.14;
  public static final double kEncoderToTurnRatio = 396/35;

  public static final double kSlowModeMovementMultiplier = 0.19;
  public static final double kSlowModeRotationMultiplier = 0.2;

  public static final double kNormalModeRotationMultiplier = 0.5;

  public static final HolonomicPathFollowerConfig kHolonomicPathFollowerConfig = new HolonomicPathFollowerConfig(
    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
    new PIDConstants(5.0, 0.0, 0.0), // Rotation PID constants
    kMaxLinearSpeed, // Max module speed, in m/s
    Math.sqrt(Math.pow(kTrackWidthX / 2, 2) + Math.pow(kTrackWidthY / 2, 2)), // Drive base radius in meters. Distance from robot center to furthest module.
    new ReplanningConfig() // Default path replanning config. See the API for the options here
  );
}
