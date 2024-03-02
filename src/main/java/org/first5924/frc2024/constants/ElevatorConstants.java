// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorConstants {
  public static final int kLeftTalonId = 52;
  public static final int kRightTalonId = 51;
  public static final int kLaserCanId = 15;

  public static final double kEncoderToSpoolRatio = 24.31;
  public static final double kSpoolDiameterMeters = Units.inchesToMeters(1.75);
  public static final double kSpoolCircumferenceMeters = Math.PI * kSpoolDiameterMeters;

  public static final int kLaserCanReadingAtLowestMillimeters = 20;

  public static final double kP = 16;
  public static final double kG = 0.25;

  public static final double kPeakForwardVoltage = 5;
  public static final double kPeakReverseVoltage = -4;

  public static final double kForwardSoftLimitThreshold = 3.5;
  public static final double kReverseSoftLimitThreshold = 0;

  public static final double kIntakeHeight = 0;
  public static final double kAmpHeight = 0;
  public static final double kAimLowHeight = 0;
  public static final double kAimHighHeight = 0;
}
