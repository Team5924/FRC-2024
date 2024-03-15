// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.constants;

/** Add your docs here. */
public class WristConstants {
  public static final int kTalonId = 54;
  public static final int kCanCoderId = 49;

  public static final double kP = 450;

  public static final double kCanCoderOffset = -0.275;

  public static final double kGearRatio = 282.24;

  public static final double kIntakeAngle = 39;
  public static final double kAmpAngle = -28;
  // Essentially just max it out, the max angle checker will prevent it from going past
  public static final double kClimbAngle = -90;

  public static final double kPeakVoltage = 7;
  public static final double kMaxAngle = 49;
}
