// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.constants;

/** Add your docs here. */
public class IntakeConstants {
  public static final int kRollerTalonId = 50;
  public static final int kPivotTalonId = 53;

  public static final double kEncoderToPivotRatio = 59.4;

  public static final double kPivotKP = 30.0;

  public static final double kPivotPeakVoltage = 3;

  public static final double kFloorPivotAngleDegrees = 90;
  public static final double kFloorRollerVoltage = 0;

  public static final double kFeederPivotAngleDegrees = 0;
  public static final double kFeederRollerVoltage = 0;

  public static final double kRetractPivotAngleDegrees = 0;
  public static final double kRetractRollerVoltage = 0;

  public static final double kEjectPivotAngleDegrees = 45;
  public static final double kEjectRollerVoltage = 0;

  public static enum IntakeState {
    FLOOR(kFloorPivotAngleDegrees, kFloorRollerVoltage),
    FEEDER(kFeederPivotAngleDegrees, kFeederRollerVoltage),
    RETRACT(kRetractPivotAngleDegrees, kRetractRollerVoltage),
    EJECT(kEjectPivotAngleDegrees, kEjectRollerVoltage);

    private final double pivotAngle;
    private final double intakeVoltage;

    private IntakeState(double pivotAngle, double intakeVoltage) {
      this.pivotAngle = pivotAngle;
      this.intakeVoltage = intakeVoltage;
    }

    public double getPivotAngle() {
      return pivotAngle;
    }

    public double getIntakeVoltage() {
      return intakeVoltage;
    }
  }
}
