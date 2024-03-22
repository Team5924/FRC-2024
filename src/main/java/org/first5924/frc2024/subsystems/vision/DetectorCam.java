// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.vision;

import org.first5924.frc2024.constants.VisionConstants;
import org.first5924.frc2024.constants.VisionConstants.BestNote;
import org.first5924.lib.LimelightHelpers;
import org.first5924.lib.LimelightHelpers.LimelightResults;
import org.first5924.lib.LimelightHelpers.LimelightTarget_Detector;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class DetectorCam extends SubsystemBase {
    NetworkTable table = NetworkTableInstance.getDefault().getTable(VisionConstants.detectorLimelightName);
    // Vertical offset from crosshair to target
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tv = table.getEntry("tv");


    LimelightResults llresults;


    private double noteAngleX;
    private double noteAngleY;
    private double noteIsInSight;
    
    public DetectorCam() {
        
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        noteAngleX = tx.getDouble(0.0);
        noteAngleY = ty.getDouble(0.0);
        noteIsInSight = tv.getDouble(noteIsInSight);
        // SmartDashboard.putString("table", table.toString());
        // SmartDashboard.putNumber("distance", getDistanceToTargetInches());
        // SmartDashboard.putNumber("Number of targets in view", GetNumberOfTargets());
        // System.out.println("BANANA - table: " + table.containsKey("tx") + " / pos: (" + x + ", " + y + ")");

    }

    public double getNoteAngleX(){
        return noteAngleX;
    }

    public double getNoteAngleY(){
        return noteAngleY;
    }

    public boolean hasTarget(){
        if (noteIsInSight==1) {
            return true;
        }
        else {
            return false;
        }
    }


    public int GetNumberOfTargets()
    {
        llresults = LimelightHelpers.getLatestResults("");

        LimelightTarget_Detector[] targets = llresults.targetingResults.targets_Detector;
        return targets.length;
    }

    /*  returns a bool[] with length of 2
        bool[0] refers to the left note
        bool[1] refers to the right note
        false means that the note is not present or not recommended
        true means that the note is present and recommended
    */
    public boolean[] getBestNote(){

        boolean[] bestNote = new boolean[2];
        llresults = LimelightHelpers.getLatestResults("");
        LimelightTarget_Detector[] results = llresults.targetingResults.targets_Detector;

        if(results.length == 0){
            bestNote[0] = false;
            bestNote[1] = false;
            return bestNote;
        }

        if(results.length == 1){
            if(results[0].ty > 0){
                bestNote[0] = false;
                bestNote[1] = true;
                return bestNote;
            }
            else{
                bestNote[0] = true;
                bestNote[1] = false;
                return bestNote;
            }
        }

        if(results.length > 1){
            if(Math.abs(results[0].ty) > Math.abs(results[1].ty)){
                if(results[1].ty > 0){
                    bestNote[0] = false;
                    bestNote[1] = true;
                    return bestNote;
                }
                else{
                    bestNote[0] = true;
                    bestNote[1] = false;
                    return bestNote;
                }
            }
            else{
                if(results[1].ty > 0){
                    bestNote[0] = false;
                    bestNote[1] = true;
                    return bestNote;
                }
                else{
                    bestNote[0] = true;
                    bestNote[1] = false;
                    return bestNote;
                }
            }
        }
        
        bestNote[0] = false;
        bestNote[1] = false;
        return bestNote;
    }
    /**
     * Distance from the point the limelight is looking at to the target (🍩)
     * @return the distance ^
     */
    public double getVerticalDistanceToTargetInches()
    {
        // angle from crosshair to 🍩 (vertical)
        double targetOffsetAngleVerticalRadians = ty.getDouble(0) * VisionConstants.degreesToRadiansMultiplier;

        // angle from limelight to 🍩 (vertical)
        double angleToGoal = VisionConstants.limelightMountAngleRadians + targetOffsetAngleVerticalRadians;

        return VisionConstants.limelightHeight * Math.tan(angleToGoal);
    }

    public double getDistanceToTargetInches()
    {
        // angle from crosshair to 🍩 (vertical)
        double targetOffsetAngleVerticalRadians = ty.getDouble(0) * VisionConstants.degreesToRadiansMultiplier;

        // angle from limelight to 🍩 (vertical)
        double angleToGoal = VisionConstants.limelightMountAngleRadians + targetOffsetAngleVerticalRadians;

        return VisionConstants.limelightHeight / Math.tan(angleToGoal);
    }
}

