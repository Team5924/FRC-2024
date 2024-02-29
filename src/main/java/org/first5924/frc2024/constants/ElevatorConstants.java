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

  public static final double kLaserCanReadingAtLowestMillimeters = 22;

  public static final double kP = 1;

  public static final double kPeakVoltage = 8;
}
