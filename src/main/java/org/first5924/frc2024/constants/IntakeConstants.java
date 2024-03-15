// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.constants;

/** Add your docs here. */
public class IntakeConstants {
  public static final int kRollerTalonId = 50;
  public static final int kPivotTalonId = 53;

  public static final double kEncoderToPivotRatio = 59.4;

  public static final double kPivotKP = 50.0;

  public static final double kPivotPeakVoltage = 6;

  public static final double kFloorPivotAngleDegrees = 117;
  public static final double kFloorRollerPercent = 0.8;

  public static final double kFeederPivotAngleDegrees = 35;
  public static final double kFeederRollerPercent = 0.6;

  public static final double kRetractPivotAngleDegrees = 15;
  public static final double kRetractRollerPercent = 0;

  public static final double kStartPivotAngleDegrees = 0;
  public static final double kStartRollerPercent = 0;

  public static final double kEjectPivotAngleDegrees = 95;
  public static final double kEjectRollerPercent = -0.9;

  public static enum IntakeState {
    FLOOR(kFloorPivotAngleDegrees, kFloorRollerPercent),
    FLOOR_OFF(kFloorPivotAngleDegrees, 0),
    FEEDER(kFeederPivotAngleDegrees, kFeederRollerPercent),
    RETRACT(kRetractPivotAngleDegrees, kRetractRollerPercent),
    EJECT(kEjectPivotAngleDegrees, kEjectRollerPercent),
    START(kStartPivotAngleDegrees, kStartRollerPercent);

    private final double pivotAngle;
    private final double rollerPercent;

    private IntakeState(double pivotAngle, double rollerPercent) {
      this.pivotAngle = pivotAngle;
      this.rollerPercent = rollerPercent;
    }

    public double getPivotAngle() {
      return pivotAngle;
    }

    public double getRollerPercent() {
      return rollerPercent;
    }
  }
}
