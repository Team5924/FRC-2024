// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.first5924.frc2023swerve.subsystems.drive;

import org.littletonrobotics.junction.AutoLog;

public interface GyroIO {
  @AutoLog
  public static class GyroIOInputs {
    public double pitchPositionRad = 0.0;
    public double yawPositionRad = 0.0;
    public double rollPositionRad = 0.0;
    public double pitchVelocityRadPerSec = 0.0;
    public double yawVelocityRadPerSec = 0.0;
    public double rollVelocityRadPerSec = 0.0;
  }

  public default void updateInputs(GyroIOInputs inputs) {}

  public default void setGyroYaw(double yaw) {}
}
