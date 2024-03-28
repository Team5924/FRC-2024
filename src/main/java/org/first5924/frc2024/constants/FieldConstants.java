package org.first5924.frc2024.constants;

import org.first5924.frc2024.robot.RobotContainer;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
  public static final double kBlueSpeakerCenterFieldX = Units.inchesToMeters(9.1);
  public static final double kRedSpeakerCenterFieldX = Units.inchesToMeters(648) - kBlueSpeakerCenterFieldX;
  public static final double kBothSpeakerCenterFieldY = Units.inchesToMeters(219);

  public static final double kBlueAmpAreaFieldX = Units.inchesToMeters(60);
  public static final double kRedAmpAreaFieldX = Units.inchesToMeters(648) - kBlueAmpAreaFieldX;
  public static final double kBothAmpAreaFieldY = Units.inchesToMeters(298);

  public static final Translation2d kBlueSpeakerCenterTranslation = new Translation2d(kBlueSpeakerCenterFieldX, kBothSpeakerCenterFieldY);
  public static final Translation2d kRedSpeakerCenterTranslation = new Translation2d(kRedSpeakerCenterFieldX, kBothSpeakerCenterFieldY);

  public static final Translation2d kBlueAmpAreaTargetTranslation = new Translation2d(kBlueAmpAreaFieldX, kBothAmpAreaFieldY);
  public static final Translation2d kRedAmpAreaTargetTranslation = new Translation2d(kRedAmpAreaFieldX, kBothAmpAreaFieldY);

  public static Translation2d getAllianceSpeakerCenterTranslation() {
    return RobotContainer.getAlliance() == Alliance.Blue ? kBlueSpeakerCenterTranslation : kRedSpeakerCenterTranslation;
  }

  public static Translation2d getAllianceAmpAreaTargetTranslation() {
    return RobotContainer.getAlliance() == Alliance.Blue ? kBlueAmpAreaTargetTranslation : kRedAmpAreaTargetTranslation;
  }
}

