// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.constants;

/** Add your docs here. */
public class FeederConstants {
  public static final int kTalonId = 55;
  public static final int kLaserCanId = 16;

  public static final double kDistanceWhenNoteIn = 45;

  public static final double kPushPercent = 0.9;
  public static final double kPushTime = 0.7;

  public static final double kAlignPercent = -0.17;
  public static final double kAlignTime = 0.08;

  public static final double kTimeInRetractToDisable = 2;

  public static enum FeederState {
    MANUAL,
    INTAKE,
    ALIGN,
    FEED_SHOOTER,
    POSITION_NOTE_REVERSE,
    EJECT
  }
}
