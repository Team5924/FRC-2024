// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.vision;

import org.first5924.frc2024.constants.VisionConstants;
import org.first5924.frc2024.subsystems.drive.Drive;
import org.first5924.frc2024.subsystems.vision.DetectorCam;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class DriveToNote extends Command {
  /** Creates a new LookAtNote. */
  private NetworkTable table = NetworkTableInstance.getDefault().getTable(VisionConstants.detectorLimelightName);
    // Vertical offset from crosshair to target
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry tv = table.getEntry("tv");
    PIDController visionControllerX;
    PIDController visionControllerTheta;

  
  private final Drive drive;
  public DriveToNote(Drive drive) {
    this.drive = drive;
    addRequirements(drive);
    visionControllerX = new PIDController(.12, 0, 0);
    visionControllerTheta = new PIDController(.05, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double x = tx.getDouble(0);
    double y = ty.getDouble(0);
    double v = tv.getDouble(0);
    //visionController.calculate()
    if(v == 1){
      // if(x > 5){
      //   drive.drive(visionController.calculate(x, 0), visionController.calculate(y, 0), -0.1, false);
      // }
      // else if(x < -5){
      //   drive.drive(visionController.calculate(x, 0), 0, 0.1, false);
      // }
      // else{
      //   drive.drive(visionController.calculate(), 0, 0, false);
      // }
      
      drive.drive(visionControllerX.calculate(y + 1, 0), 0, visionControllerTheta.calculate(x + 1, 0), false);
    }
    else
    {
      drive.drive(0, 0, 0, false);
    }
  
    SmartDashboard.putNumber("tx", x);
    SmartDashboard.putNumber("ty", y);
  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(0,0,0,false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}