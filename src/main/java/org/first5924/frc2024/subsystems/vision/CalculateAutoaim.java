// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CalculateAutoaim extends SubsystemBase {
  /** Creates a new CalculateAutoaim. */
  public CalculateAutoaim() {}

  private double[][] highDistanceAngleTable = {
    //list of distance values in meters
    {}, 
    //list of successful angles in degrees
    {}
  };

  private double[][] lowDistanceAngleTable = {
    //list of distance values in meters (truncate at 3 decimals)
    {1.075},
    //list of successful angles values in degrees (truncate at 3 decimals)
    {54.843}
  };

  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double calculateAngleHigh(double distanceFromSpeaker){
    int index;
    double distance;
    double slope;
    double mx1;
    double y1;

    //iterate through the array in order to find the distance that the input is in between to get the most accurate measure
    index = 0;
    distance = highDistanceAngleTable[0][index];
    while(distanceFromSpeaker > distance){
      ++index;
      distance = highDistanceAngleTable[0][index];
    }

    //find the slope
    slope = (highDistanceAngleTable[1][index] - highDistanceAngleTable[1][index - 1])/(highDistanceAngleTable[0][index] - highDistanceAngleTable[0][index - 1]);

    /*
      (y - y1) = m(x - x1)
      (y - y1) = mx - mx1
       y = mx - mx1 + y1
    */
    mx1 = slope*highDistanceAngleTable[0][index];
    y1 = highDistanceAngleTable[1][index];

    return (slope*distanceFromSpeaker - mx1 + y1);

  }




  public double calculateAngleLow(double distanceFromSpeaker){
    int index;
    double distance;
    double slope;
    double mx1;
    double y1;

    //iterate through the array in order to find the distance that the input is in between to get the most accurate measure
    index = 0;
    distance = lowDistanceAngleTable[0][index];
    while(distanceFromSpeaker > distance){
      ++index;
      distance = lowDistanceAngleTable[0][index];
    }

    //find the slope
    slope = (lowDistanceAngleTable[1][index] - lowDistanceAngleTable[1][index - 1])/(lowDistanceAngleTable[0][index] - lowDistanceAngleTable[0][index - 1]);

    /*
      (y - y1) = m(x - x1)
      (y - y1) = mx - mx1
       y = mx - mx1 + y1
    */
    mx1 = slope*lowDistanceAngleTable[0][index];
    y1 = lowDistanceAngleTable[1][index];

    return (slope*distanceFromSpeaker - mx1 + y1);

  }


}
