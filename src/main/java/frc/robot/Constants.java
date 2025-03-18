// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class FieldPositions {

    private static double alignDistance = 2;

    private static double robotCenterToBumperEdge = .876/2;

    private static double robotCenterToCoralOffset = .343;

    private static double coralPoleOffset = .329;

    private static double reefWidth = 1.663;

    private static double scoreMovementVerticalTravel = (alignDistance - (reefWidth)) + .4;//- robotCenterToBumperEdge;

    private static double leftScoreTravel = (-coralPoleOffset/2) - robotCenterToCoralOffset;

    private static double rightScoreTravel = (coralPoleOffset/2) - robotCenterToCoralOffset;

    public static Transform2d leftScoreTransform = new Transform2d(scoreMovementVerticalTravel, leftScoreTravel, new Rotation2d());

    public static Transform2d rightScoreTransform = new Transform2d(scoreMovementVerticalTravel, rightScoreTravel, new Rotation2d());



    private static double angledXOffset = .5 * alignDistance;

    private static double angledYoffset = 0.866025403784 * alignDistance;


    public static Pose2d[] ABPose = {
      new Pose2d(4.5 - alignDistance,4, Rotation2d.fromDegrees(0)),
      new Pose2d(13. + alignDistance,4, Rotation2d.fromDegrees(180))
    };
    public static Pose2d[] CDPose = {
      new Pose2d(4.5 - angledXOffset, 4 - angledYoffset, Rotation2d.fromDegrees(60)),
      new Pose2d(13. + angledXOffset, 4 + angledYoffset, Rotation2d.fromDegrees(180+60))
    };
    public static Pose2d[] EFPose = {
      new Pose2d(4.5 + angledXOffset,4 - angledYoffset, Rotation2d.fromDegrees(120)),
      new Pose2d(13. - angledXOffset,4 + angledYoffset, Rotation2d.fromDegrees(180+120))
    };
    public static Pose2d[] GHPose = {
      new Pose2d(4.5 + alignDistance,4, Rotation2d.fromDegrees(180)),
      new Pose2d(13. - alignDistance,4, Rotation2d.fromDegrees(0))
    };
    public static Pose2d[] IJPose = {
      new Pose2d(4.5 + angledXOffset,4 + angledYoffset, Rotation2d.fromDegrees(-120)),
      new Pose2d(13. - angledXOffset,4 - angledYoffset, Rotation2d.fromDegrees(180-120))
    };
    public static Pose2d[] KLPose = {
      new Pose2d(4.5 - angledXOffset,4 + angledYoffset, Rotation2d.fromDegrees(-60)),
      new Pose2d(13. + angledXOffset,4 - angledYoffset, Rotation2d.fromDegrees(180-60))
    };

    public static Transform2d scoreOffset = new Transform2d(0, 0.06, new Rotation2d());

    public static Pose2d rightIntakeBlue =new Pose2d(1,1, Rotation2d.fromDegrees(-125+180)).transformBy(new Transform2d(0,.01, new Rotation2d()));
    public static Pose2d rightIntakeRed =new Pose2d(16.5,1, Rotation2d.fromDegrees(-55+180)).transformBy(new Transform2d(0,.01, new Rotation2d()));
    public static Pose2d leftIntakeBlue =new Pose2d(1,7, Rotation2d.fromDegrees(125+180)).transformBy(new Transform2d(0,.01, new Rotation2d()));
    public static Pose2d leftIntakeRed =new Pose2d(16.5,7, Rotation2d.fromDegrees(55+180)).transformBy(new Transform2d(0,.01, new Rotation2d()));

  }

  public static final class ManipulatorGearRatios {
    public static double kAlgaeWristRatio = 1/50d;
    public static double kCoralWristRatio = 1/50d;
  }

  public static final class ManipulatorPositions {
    public static double kCoralElevatorPosIntake = 5;
    public static double kCoralElevatorPosL4 = 132;
    public static double kCoralElevatorPosL3 = 72+3;
    public static double kCoralElevatorPosL2 = 36+3;
    public static double kCoralElevatorPosL1 = 0.1;

    public static double kCoralWristPosIntake = -0.959931088597;
    public static double kCoralWristPosL4 = 0.65;
    public static double kCoralWristPosL3 = 0.65;
    public static double kCoralWristPosL2 = 0.65;
    public static double kCoralWristPosL1 = 1.57254165605;
    

    public static double kAlgaeElevatorPosTop = 12.5;
    public static double kAlgaeElevatorPosBottom = 7.5;
    public static double kAlgaeElevatorPosProcesser = 2.5;

    public static double kAlgaeWristPosIntake =  -0.610865238198;
  }

  public static final class PIDConstants {
    public static double kElevatorP = 0.05;
    public static double kElevatorI = 0;
    public static double kElevatorD = 0;

    public static double kClimberP = 0.2;
    public static double kClimberI = 0.02;
    public static double kClimberD = 0;

    public static double kAlgaeWristP = 0.5;
    public static double kAlgaeWristI = 0.02;
    public static double kAlgaeWristD = 0;

    public static double kAlgaeElevatorStall = 0.3;

    public static double kAlgaeWristrStallMulti = 0.17;


    public static double kCoralWristP = 1;
    public static double kCoralWristI = 0.05;
    public static double kCoralWristD = 0.15;

    public static double kCoralWristMaxVel = 0.2;
    public static double kCoralWristMaxAccel = 0.1;

    public static double kCoralElevatorStall = 0.05;

    public static double kCoralWristrStallMulti = 0.3;
    public static double kNoCoralWristrStallMulti = 0.1;
  }

  public static final class MotorIDs {
    public static final int climberMotorID = 36;


    // public static final int leftAlgaeShooterMotorID = 22;
    // public static final int rightAlgaeShooterMotorID = 23;
    public static final int leftAlgaeIntakeMotorID = 9;
    public static final int rightAlgaeIntakeMotorID = 10;
    public static final int algaeWristMotorID = 11;
    public static final int leftAlgaeElevatorMotorID = 30;
    public static final int rightAlgaeElevatorMotorID = 31;

    public static final int coralMotorID = 43;
    public static final int coralWristMotorID = 40;
    public static final int leftCoralElevatorMotorID = 20;
    public static final int rightCoralElevatorMotorID = 21;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 3d; // speed in ms
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    public static final double kDirectionSlewRate = 2.5; // radians per second
    public static final double kMagnitudeSlewRate = 7.5; // percent per second (1 = 100%)
    public static final double kRotationalSlewRate = 10.0; // percent per second (1 = 100%)

    // Chassis configuration
    public static double robotWidth = 28; // wheel to wheel distance

    public static double robotRadius = (Math.sqrt(2*Math.pow(robotWidth, 2))/2)*0.254;

    

    public static final double kTrackWidth = Units.inchesToMeters(robotWidth);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(robotWidth);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;


    public static final int kFrontLeftDrivingCanId = 8;
    public static final int kRearLeftDrivingCanId = 6;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 2;

    public static final int kFrontLeftTurningCanId = 7;
    public static final int kRearLeftTurningCanId = 5;
    public static final int kFrontRightTurningCanId = 3;
    public static final int kRearRightTurningCanId = 1;

    public static final boolean kGyroReversed = false;

  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T, 13T, or 14T.
    // This changes the drive speed of the module (a pinion gear with more teeth will result in a
    // robot that drives faster).
    public static final double kDrivingMotorPinionTeeth = 14; //* 3/(3+ 0.3048 + 0.1016); // yeh i know you shouldent do this but im too lazy to actually figure out whats wrong (:

    // Invert the turning encoder, since the output shaft rotates in the opposite direction of
    // the steering motor in the MAXSwerve Module.
    public static final boolean kTurningEncoderInverted = true;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15 teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;

    public static final double kDrivingEncoderPositionFactor = ((kWheelDiameterMeters * Math.PI)/kDrivingMotorReduction);
    public static final double kDrivingEncoderVelocityFactor = ((kWheelDiameterMeters * Math.PI)
        / kDrivingMotorReduction) / 60.0; // meters per second

    public static final double kTurningEncoderPositionFactor = (2 * Math.PI); // radians
    public static final double kTurningEncoderVelocityFactor = (2 * Math.PI) / 60.0; // radians per second

    public static final double kTurningEncoderPositionPIDMinInput = 0; // radians
    public static final double kTurningEncoderPositionPIDMaxInput = kTurningEncoderPositionFactor; // radians

    public static final double kDrivingP = 0.04;
    public static final double kDrivingI = 0.d;
    public static final double kDrivingD = 0;
    public static final double kDrivingFF = 1 / kDriveWheelFreeSpeedRps;
    public static final double kDrivingMinOutput = -1;
    public static final double kDrivingMaxOutput = 1;

    public static final double kTurningP = 1;
    public static final double kTurningI = 0;
    public static final double kTurningD = 0;
    public static final double kTurningFF = 0;
    public static final double kTurningMinOutput = -1;
    public static final double kTurningMaxOutput = 1;

    public static final IdleMode kDrivingMotorIdleMode = IdleMode.kBrake;
    public static final IdleMode kTurningMotorIdleMode = IdleMode.kBrake;

    public static final int kDrivingMotorCurrentLimit = 50; // amps
    public static final int kTurningMotorCurrentLimit = 20; // amps
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 5676;
  }
}
