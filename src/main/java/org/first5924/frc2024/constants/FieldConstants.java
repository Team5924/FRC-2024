package org.first5924.frc2024.constants;

import org.first5924.frc2024.robot.RobotContainer;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class FieldConstants {
  public static final double kBlueSpeakerCenterFieldX = Units.inchesToMeters(9.1);
  public static final double kRedSpeakerCenterFieldX = Units.inchesToMeters(648) - kBlueSpeakerCenterFieldX;
  public static final double kBothSpeakerCenterFieldY = Units.inchesToMeters(219);

  public static final Translation2d kBlueSpeakerCenterFieldTranslation = new Translation2d(kBlueSpeakerCenterFieldX, kBothSpeakerCenterFieldY);
  public static final Translation2d kRedSpeakerCenterFieldTranslation = new Translation2d(kRedSpeakerCenterFieldX, kBothSpeakerCenterFieldY);

  public static Translation2d getAllianceSpeakerCenterFieldTranslation() {
    return RobotContainer.getAlliance() == Alliance.Blue ? kBlueSpeakerCenterFieldTranslation : kRedSpeakerCenterFieldTranslation;
  }
}

