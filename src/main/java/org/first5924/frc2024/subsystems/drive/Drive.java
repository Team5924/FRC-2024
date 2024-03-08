// Copyright (c) 2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package org.first5924.frc2024.subsystems.drive;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import org.first5924.frc2024.constants.DriveConstants;
import org.first5924.frc2024.constants.FieldConstants;
import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

public class Drive extends SubsystemBase {
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, BL, BR

  private SwerveDriveKinematics kinematics =
    new SwerveDriveKinematics(
      new Translation2d(DriveConstants.kTrackWidthX / 2, DriveConstants.kTrackWidthY / 2),
      new Translation2d(DriveConstants.kTrackWidthX / 2, -DriveConstants.kTrackWidthY / 2),
      new Translation2d(-DriveConstants.kTrackWidthX / 2, DriveConstants.kTrackWidthY / 2),
      new Translation2d(-DriveConstants.kTrackWidthX / 2, -DriveConstants.kTrackWidthY / 2)
    );

  private SwerveDrivePoseEstimator poseEstimator;

  // For SysId
  private final MutableMeasure<Voltage> appliedVoltageMutableMeasure = MutableMeasure.mutable(Units.Volts.of(0));
  private final MutableMeasure<Distance> distanceMutableMeasure = MutableMeasure.mutable(Units.Meters.of(0));
  private final MutableMeasure<Velocity<Distance>> velocityMutableMeasure = MutableMeasure.mutable(Units.MetersPerSecond.of(0));
  private SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(this::driveVoltageForCharacterization, this::logDriveForCharacterization, this)
  );

  public Drive(GyroIO gyroIO, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {
    this.gyroIO = gyroIO;
    modules[0] = new Module(flModuleIO, 0);
    modules[1] = new Module(frModuleIO, 1);
    modules[2] = new Module(blModuleIO, 2);
    modules[3] = new Module(brModuleIO, 3);
    for (var module : modules) {
      module.setBrakeMode(true);
    }
    poseEstimator = new SwerveDrivePoseEstimator(
      kinematics,
      new Rotation2d(gyroInputs.yawPositionRad),
      new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition(),
      },
      new Pose2d()
    );

    AutoBuilder.configureHolonomic(
            this::getEstimatedPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            this::driveRobotRelativeFromChassisSpeeds, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
            DriveConstants.kHolonomicPathFollowerConfig,
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }

  public void periodic() {
    Logger.recordOutput("Estimated Pose", getEstimatedPose());
    Logger.recordOutput("Distance to Center of Blue Speaker", getDistanceToSpeakerCenter(Alliance.Blue));
    Logger.recordOutput("Field Angle to Face Speaker", getFieldRotationRadiansToPointShooterAtSpeakerCenter(Alliance.Red));
    Logger.recordOutput("Estimated Rotation", getEstimatedPose().getRotation().getRadians());
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);
    for (var module : modules) {
      module.periodic();
    }

    poseEstimator.update(
      new Rotation2d(gyroInputs.yawPositionRad),
      new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition(),
      }
    );
  }

  public void drive(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond, boolean fieldCentric, boolean slowMode) {
    SmartDashboard.putBoolean("Slow Mode", slowMode);

    ChassisSpeeds speeds = fieldCentric ?
      ChassisSpeeds.fromFieldRelativeSpeeds(
        vxMetersPerSecond,
        vyMetersPerSecond,
        omegaRadiansPerSecond,
        new Rotation2d(gyroInputs.yawPositionRad)) :
      new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);
    SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(moduleStates, DriveConstants.kMaxLinearSpeed);
    for (int i = 0; i < 4; i++) {
      modules[i].runSetpoint(moduleStates[i]);
    }
  }

  public void driveVoltageForCharacterization(Measure<Voltage> voltsMeasure) {
    double volts = voltsMeasure.in(Units.Volts);
    for (int i = 0; i < 4; i++) {
      modules[i].driveVoltage(volts);
    }
  }

  public void logDriveForCharacterization(SysIdRoutineLog routineLog) {
    routineLog.motor("drive-left")
      .voltage(appliedVoltageMutableMeasure.mut_replace(modules[0].getLastVoltage(), Units.Volts))
      .linearPosition(distanceMutableMeasure.mut_replace(modules[0].getPositionMeters(), Units.Meters))
      .linearVelocity(velocityMutableMeasure.mut_replace(modules[0].getVelocityMetersPerSec(), Units.MetersPerSecond));
    routineLog.motor("drive-right")
      .voltage(appliedVoltageMutableMeasure.mut_replace(modules[1].getLastVoltage(), Units.Volts))
      .linearPosition(distanceMutableMeasure.mut_replace(modules[1].getPositionMeters(), Units.Meters))
      .linearVelocity(velocityMutableMeasure.mut_replace(modules[1].getVelocityMetersPerSec(), Units.MetersPerSecond));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  public ChassisSpeeds getChassisSpeeds() {
    return kinematics.toChassisSpeeds(
      modules[0].getState(),
      modules[1].getState(),
      modules[2].getState(),
      modules[3].getState()
    );
  }

  public void driveRobotRelativeFromChassisSpeeds(ChassisSpeeds chassisSpeeds) {
    drive(
      chassisSpeeds.vxMetersPerSecond,
      chassisSpeeds.vyMetersPerSecond,
      chassisSpeeds.omegaRadiansPerSecond,
      false,
      false);
  }

  /** Stops the drive. */
  public void stop() {
    for (Module module : modules) {
      module.stop();
    }
  }

  public void setGyroYaw(double yaw) {
    gyroIO.setGyroYaw(yaw);
  }

  public Rotation2d getYaw() {
    return new Rotation2d(gyroInputs.yawPositionRad);
  }

  public Rotation2d getPitch() {
    return new Rotation2d(gyroInputs.pitchPositionRad);
  }

  public Rotation2d getRoll() {
    return new Rotation2d(gyroInputs.rollPositionRad);
  }

  /** Returns the current yaw velocity (Z rotation) in radians per second. */
  public double getYawVelocity() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  /** Returns the current pitch velocity (Y rotation) in radians per second. */
  public double getPitchVelocity() {
    return gyroInputs.pitchVelocityRadPerSec;
  }

  /** Returns the current roll velocity (X rotation) in radians per second. */
  public double getRollVelocity() {
    return gyroInputs.rollVelocityRadPerSec;
  }

  public Pose2d getEstimatedPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void addVisionMeasurement(Pose2d visionPoseEstimate, double timestampSeconds) {
    poseEstimator.addVisionMeasurement(visionPoseEstimate, timestampSeconds);
  }

  public double getDistanceToSpeakerCenter(Alliance alliance) {
    return alliance == Alliance.Blue ?
      FieldConstants.kBlueSpeakerCenterFieldTranslation.getDistance(getEstimatedPose().getTranslation()) :
      FieldConstants.kRedSpeakerCenterFieldTranslation.getDistance(getEstimatedPose().getTranslation());
  }

  public double getFieldRotationRadiansToPointShooterAtSpeakerCenter(Alliance alliance) {
    return alliance == Alliance.Blue ?
      FieldConstants.kBlueSpeakerCenterFieldTranslation.minus(getEstimatedPose().getTranslation()).getAngle().plus(new Rotation2d(Math.PI)).getRadians() :
      FieldConstants.kRedSpeakerCenterFieldTranslation.minus(getEstimatedPose().getTranslation()).getAngle().plus(new Rotation2d(Math.PI)).getRadians();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(
      pose.getRotation(),
      new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition(),
      },
      pose
    );
  }
}
