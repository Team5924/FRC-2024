package org.first5924.frc2024.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public class FieldConstants {
    public static final double kBlueSpeakerCenterFieldX = Units.inchesToMeters(9.1);
    public static final double kRedSpeakerCenterFieldX = Units.inchesToMeters(648) - kBlueSpeakerCenterFieldX;
    public static final double kBothSpeakerCenterFieldY = Units.inchesToMeters(219);

    public static final Translation2d kBlueSpeakerCenterFieldTranslation = new Translation2d(kBlueSpeakerCenterFieldX, kBothSpeakerCenterFieldY);
    public static final Translation2d kRedSpeakerCenterFieldTranslation = new Translation2d(kRedSpeakerCenterFieldX, kBothSpeakerCenterFieldY);

    //this value is to be tested
    public static final double shooterExitVeloMPS = 32.6;

    //this should not be a constant. Instead, we should read the encoder from the elevator
    public static final double exampleShooterHeight = .7;

    public static final double speakerHeight = 2.0828041656;
}

