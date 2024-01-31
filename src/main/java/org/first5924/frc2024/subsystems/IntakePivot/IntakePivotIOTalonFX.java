
package org.first5924.frc2024.subsystems.intakePivot;


import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.RelativeEncoder;

import org.first5924.frc2024.constants.IntakePivotConstants;

/** Add your docs here. */
public class IntakePivotIOTalonFX implements IntakePivotIO {
  private final TalonFX pivotTalon = new TalonFX(IntakePivotConstants.IntakePivotID);
  private final CANcoder pivotEncoder = new CANcoder(IntakePivotConstants.pivotCancoderID);


  public IntakePivotIOTalonFX() {
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

    MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
    motorOutputConfigs.Inverted = InvertedValue.CounterClockwise_Positive;
    motorOutputConfigs.NeutralMode = NeutralModeValue.Coast;
    talonFXConfiguration.MotorOutput = motorOutputConfigs;

    CurrentLimitsConfigs currentLimitsConfigs = new CurrentLimitsConfigs();
    currentLimitsConfigs.SupplyCurrentLimit = 40;
    currentLimitsConfigs.SupplyCurrentThreshold = 40;
    currentLimitsConfigs.SupplyTimeThreshold = 0;
    currentLimitsConfigs.SupplyCurrentLimitEnable = true;
    talonFXConfiguration.CurrentLimits = currentLimitsConfigs;

  
    
    pivotTalon.getConfigurator().apply(talonFXConfiguration);
  }

  @Override
  public void updateInputs(IntakePivotIOInputs inputs) {
      inputs.pivotMotorTempCelsius = pivotTalon.getDeviceTemp().getValueAsDouble();
    

  }





}
