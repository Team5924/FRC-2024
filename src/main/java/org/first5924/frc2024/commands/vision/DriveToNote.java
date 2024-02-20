// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.commands.vision;

import java.util.function.DoubleSupplier;

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
  
  private final PIDController visionControllerX;
  private final PIDController visionControllerTheta;
  private final DoubleSupplier x;
  private final DoubleSupplier y;
  private final boolean v;
  private final Drive drive;

  public DriveToNote(DoubleSupplier x, DoubleSupplier y, boolean v, Drive drive) {
    this.drive = drive;
    this.x = x;
    this.y = y;
    this.v = v;



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

    //visionController.calculate()
    if(v){
      
      drive.drive(visionControllerX.calculate(y.getAsDouble() + 1, 0), 0, visionControllerTheta.calculate(x.getAsDouble() + 1, 0), false);
    }
    else
    {
      drive.drive(0, 0, 0, false);
    }

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