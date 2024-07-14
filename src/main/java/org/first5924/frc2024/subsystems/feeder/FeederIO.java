// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.first5924.frc2024.subsystems.feeder;

import org.littletonrobotics.junction.AutoLog;


public interface FeederIO {
    @AutoLog
    public static class FeederIOInputs {
      public double motorTempCelsius = 0.0;
      public double motorCurrentAmps = 0.0;
      public double motorAppliedVolts = 0.0;
      public double motorVelocityRotationsPerSecond = 0.0;
      public double laserCanMeasurementMillimeters = 0;
      public boolean limitSwitchTriggered = false;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(FeederIOInputs inputs) {
    }

    public default void setPercent(double percent) {
    }
}
