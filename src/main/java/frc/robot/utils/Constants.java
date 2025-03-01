// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
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
  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 5.74;
    public static final double kMaxAccelerationMetersPerSecond = 3;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(26.5);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(26.5);
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

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 2;
    public static final int kRearLeftDrivingCanId = 8;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearRightDrivingCanId = 6;

    public static final int kFrontLeftTurningCanId = 3;
    public static final int kRearLeftTurningCanId = 9;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearRightTurningCanId = 7;

    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kDriveDeadband = 0.05;
    public static final double kHeadingIncMult = 0.04;
    
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    public static final double kPTranslationController = 5;
    public static final double kDTranslationController = 0;
    public static final double kPRotationController = 5;
    public static final double kDRotationController = 0;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  public static final class ElevatorConstants {
    //controller settings
    public static final int elev_maxcurrent = 40;
    public static final int tilt_maxcurrent = 40;
    public static final double elev_posfactor = 0.05632; //meters
    public static final double elev_velfactor = elev_posfactor / 60; //meters per second
    public static final double elev_enc_maxrational = 0.01184; //meters per 20ms
    //PIDF settings, needs characterized
    public static final double elev_FFkS = 0; //needs measured
    public static final double elev_FFkV = 0.04933;
    public static final double elev_FFkG = 0; //needs measured
    public static final double elev_FFkA = 0; //not used currently, may not be needed
    public static final double elev_PIDkP = .1;
    public static final double elev_PIDkD = 0;
    public static final double elev_maxvel = 0.59207; //meters per second
    public static final double elev_maxacl = 0.25; //meters per second ^ 2
  }

  public static final class ManipulatorConstants {
    //controller settings
    public static final int wrist_maxcurrent = 40;
    public static final int spike_maxcurrent = 20;
    public static final double wrist_posfactor =  0.062831; //rads
    public static final double wrist_velfactor = 0.001047; //rads/s
    public static final double wrist_maxrational = 0;
    //PIDF settings, needs characterized
    public static final double wrist_maxVel = 5.94389;
    public static final double wrist_maxAccel = 0.5; 
    public static final double wrist_FFkS = 0;
    public static final double wrist_FFkV = 0.49532;
    public static final double wrist_FFkG = 0;
    public static final double wrist_FFkA = 0;
    public static final double wrist_PIDkP = 0.1;
    public static final double wrist_PIDkD = 0;
  }

  //these must be measured and adjusted accordingly
  public static final class ElevatorCalibration {
    //reef height calibration data
    public static final double elev_L1height = 0;
    public static final double elev_L2height = 0;
    public static final double elev_L3height = 0;
    public static final double elev_L4height = 0;
    public static final double elev_loadheight = 0;
    public static final double elev_maxheight = 0;
    //reef angle calibration data
    public static final double wrist_L1angle = 0;
    public static final double wrist_L2angle = 0;
    public static final double wrist_L3angle = 0;
    public static final double wrist_L4angle = 0;
    public static final double wrist_loadangle = 0;
  }

  //these must be measured and adjusted accordingly, angles for heading control
  public static final class DrivetrainCalibration {
    public static final double drive_leftcoralstation = 0;
    public static final double drive_rightcoralstation = 0;
    public static final double drive_reef0 = 0;
    public static final double drive_reef1 = 0;
    public static final double drive_reef2 = 0;
    public static final double drive_reef3 = 0;
    public static final double drive_reef4 = 0;
    public static final double drive_reef6 = 0;
  }

  //these must be measured and adjusted accordingly, poses for autonomous
  public static final class PoseCalibration {
    public static final Pose2d drive_startpose1 = new Pose2d();
    public static final Pose2d drive_startpose2 = new Pose2d();
    public static final Pose2d drive_startpose3 = new Pose2d();
  }
}