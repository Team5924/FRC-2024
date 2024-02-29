// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.constants;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.OpenLoopRampsConfigs;

/** Add your docs here. */
public class RobotConstants {
  public static final double kNominalVoltage = 12.0;

  public static final int kPigeonId = 2;

  public static final Mode kCurrentMode = Mode.REAL;
  
  public static ClosedLoopRampsConfigs kClosedLoopRampsConfigs = new ClosedLoopRampsConfigs()
  .withDutyCycleClosedLoopRampPeriod(0.02)
  .withTorqueClosedLoopRampPeriod(0.02)
  .withVoltageClosedLoopRampPeriod(0.02);

  public static OpenLoopRampsConfigs kOpenLoopRampsConfigs = new OpenLoopRampsConfigs()
  .withDutyCycleOpenLoopRampPeriod(0.02)
  .withTorqueOpenLoopRampPeriod(0.02)
  .withVoltageOpenLoopRampPeriod(0.02);
  
    
  
  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }
}
