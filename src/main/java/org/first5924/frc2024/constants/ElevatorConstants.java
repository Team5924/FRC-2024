// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.constants;

import edu.wpi.first.math.util.Units;

/** Add your docs here. */
public class ElevatorConstants {
  public static final int kLeftTalonId = 52;
  public static final int kRightTalonId = 51;
  public static final int kLaserCanId = 0;

  public static final double kEncoderToSpoolRatio = 1;
  public static final double kSpoolRadiusMeters = Units.inchesToMeters(2);
  public static final double kSpoolCircumference = 2 * Math.PI * kSpoolRadiusMeters;

  public static final double kP = 1;
}
