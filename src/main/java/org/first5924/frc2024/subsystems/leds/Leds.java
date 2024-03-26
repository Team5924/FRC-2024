package org.first5924.frc2024.subsystems.leds;
 
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.first5924.frc2024.constants.LedConstants;

import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;
import org.first5924.frc2024.constants.LedConstants;

public class Leds extends SubsystemBase {
    private final CANdle m_candle = new CANdle(LedConstants.CANdleID);

    //private final int LedCount = 300;
    //private XboxController joystick;

    private RainbowAnimation rainbow = new RainbowAnimation(1,0.5,64);
    private Animation m_toAnimate = rainbow;
    public int LedCount = 0;

    public Leds() {
        
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        // it checks for animaitons and if none runs rainbow
        if(m_toAnimate == null) {
            m_candle.animate(null);
        } else {
            m_candle.animate(m_toAnimate);
        }
        // m_candle.modulateVBatOutput(0);
    }

    public enum AnimationTypes {    
        Rainbow,
        SetAll,
        RgbFade,
    }

    

    private AnimationTypes m_currentAnimation;

    public void CANdleSystem() {
        changeAnimation(AnimationTypes.SetAll);
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.GRB;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        m_candle.configAllSettings(configAll, 100);
    }

    public void BlueLEDs(){
        m_candle.setLEDs(0,0,255);
        m_toAnimate = null; 
    }
    
    public void EndLEDs(){
        m_candle.setLEDs(0,0,0);
        m_toAnimate = rainbow;


    }

    public void GreenLEDs() {
        m_toAnimate = null;
        m_candle.setLEDs(0, 255, 0);
    }

    public void incrementAnimation() {
        //fades into rainbow
        switch(m_currentAnimation) {
            case Rainbow: 
                changeAnimation(AnimationTypes.RgbFade);
                break;
        }
    }
    public void decrementAnimation() {
        //fades out of rainbow
        switch(m_currentAnimation) {
            case Rainbow:
                changeAnimation(AnimationTypes.Rainbow);
                break;       
        }
    }
    // public void setColors() {
    //     changeAnimation(AnimationTypes.SetAll);
    // }

    /* Wrappers so we can access the CANdle from the subsystem */
    public double getVbat() { return m_candle.getBusVoltage(); }
    public double get5V() { return m_candle.get5VRailVoltage(); }
    public double getCurrent() { return m_candle.getCurrent(); }
    public double getTemperature() { return m_candle.getTemperature(); }
    public void configBrightness(double percent) { m_candle.configBrightnessScalar(percent, 0); }
    public void configLos(boolean disableWhenLos) { m_candle.configLOSBehavior(disableWhenLos, 0); }
    public void configLedType(LEDStripType type) { m_candle.configLEDType(type, 0); }
    public void configStatusLedBehavior(boolean offWhenActive) { m_candle.configStatusLedState(offWhenActive, 0); }

    public void changeAnimation(AnimationTypes toChange) {
        m_currentAnimation = toChange;
        
        switch(toChange)
        {
            case Rainbow:
                m_toAnimate = new RainbowAnimation(1, 0.1, LedCount);
                break;
            
            case SetAll:
                m_toAnimate = null;
                break;
        }
        System.out.println("Changed to " + m_currentAnimation.toString());
    }

    
    

    
    
}

