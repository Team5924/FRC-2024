package org.first5924.frc2024.constants;

public class FieldConstants {
    //x coordinate of the back of the speaker looking at the field from the judge's table
    public static final double xRedSpeakerMeters = (326.61-9.8)/39.37; 
    public static final double xBlueSpeakerMeters = -326.61/39.37;
    
    //y coordinate of the speakers looking at the field from the judge's table
    public static final double ySpeakerMeters = 57/39.37;

    //this value is to be tested
    public static final double shooterExitVeloMPS = 32.6;

    //this should not be a constant. Instead, we should read the encoder from the elevator
    public static final double exampleShooterHeight = .7;

    public static final double speakerHeight = 2.0828041656;
}

