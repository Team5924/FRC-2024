// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.constants;

/** Add your docs here. */
public class IntakePivotConstants {
    public static final double kGearRatio = 0.0;
    public static final int IntakePivotID = 51;
    public static final int pivotCancoderID = 0;
    public static final double PivotVoltage = 0.0;
    public static final double FloorAngle = 0.0;
    public static final double FeederAngle = 0.0;
    public static final double RetractAngle = 0.0;
    public static final double MaxVoltage = 0.0;

    public static enum PivotPosition {
        FLOOR(FloorAngle , IntakePivotConstants.PivotVoltage),
        FEEDER(FeederAngle , IntakePivotConstants.PivotVoltage),
        RETRACT(RetractAngle, IntakePivotConstants.PivotVoltage);

        private final double pivotAngle;
        private final double pivotVoltage;

        private PivotPosition(double pivotAngle, double PivotVoltage) {
            this.pivotAngle = pivotAngle;
            this.pivotVoltage = PivotVoltage;
        }

        public double getPivotAngle() {
            return pivotAngle;
        }

        public double getPivotVoltage() {
            return pivotVoltage;
        }

    }



}
