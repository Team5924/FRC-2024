package org.first5924.frc2024.constants;

public final class DetectorCamConstants {
    // TODO set measurements to actual values


    // distance from limelight to ground - h
    public static final double limelightHeight = 28.5;

    // how many degrees is limelight rotated from facing perfectly downwards? -a
    public static final double limelightMountAngleDegrees = 0.0;


    // ********** these don't need to be set **********

    // multiply to degrees to get in radians
    public static final double degreesToRadiansMultiplier = Math.PI / 180;

    // limelight mount angle in radians
    public static final double limelightMountAngleRadians = limelightMountAngleDegrees * degreesToRadiansMultiplier;

    // how far the point that the limelight is looking is from the base of the robot - d
    /*
        * ASCII REPRESENTATION
        * 
        *      CAMERA
        *        |a\
        *        |  \
        * _______h   \
        *  robot |    \
        * _______|     \     🍩 - will be used to find distance to this
        *         --d---
        *          ^ this distance here
        */
    public static final double limelightCrosshairFromBase = limelightHeight * Math.tan(limelightMountAngleRadians);

    public static String detectorLimelightName = "limelight-deteccam";
}
