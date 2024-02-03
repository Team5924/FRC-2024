// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.vision;

import org.first5924.dev.LimelightHelpers;
import org.first5924.frc2024.constants.VisionConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class Vision extends SubsystemBase {
    private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    
    // Vertical offset from crosshair to target
    private NetworkTableEntry ty = table.getEntry("ty");

    /** Creates a new Vision. */
    public Vision() {}

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public final Pose2d robotPose() {
        return LimelightHelpers.getBotPose2d(getName());
    }

    /**
     * Distance from the point the limelight is looking at to the target (🍩)
     * @return the distance ^
     */
    public double GetDistanceToTarget()
    {
        // angle from crosshair to 🍩 (vertical)
        double targetOffsetAngleVerticalRadians = ty.getDouble(0) * VisionConstants.degreesToRadiansMultiplier;

        // angle from limelight to 🍩 (vertical)
        double angleToGoal = VisionConstants.limelightMountAngleRadians + targetOffsetAngleVerticalRadians;

        return VisionConstants.limelightHeight * Math.tan(angleToGoal);
    }
}
