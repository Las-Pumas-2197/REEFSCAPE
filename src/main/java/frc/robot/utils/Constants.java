// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N3;
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
    public static final double kMaxSpeedMetersPerSecond = 4.92;
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
    public static final int kDrivingMotorPinionTeeth = 12;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762; //IN METERSSSS
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorButtonsPort = 1;
    public static final int kDriverButtonsPort = 2;
    public static final double kDriveDeadband = 0.05;
    public static final double kHeadingIncMult = 0.04;
    public static final double kDriveAxisExponent = 1;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 1;
    public static final double kMaxAccelerationMetersPerSecondSquared = 1;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    public static final double kPTranslationController = 7;
    public static final double kDTranslationController = 0;
    public static final double kPRotationController = 7;
    public static final double kDRotationController = 0;

    // Constraint for the motion profiled robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, 
        kMaxAngularSpeedRadiansPerSecondSquared);

    public static final double kMaxAngularVelRadsPerSecond = 1.645*(2*Math.PI);
    public static final double kHeadingFFkV = 12 / kMaxAngularVelRadsPerSecond;
    public static final double kHeadingFFkS = 0.1;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  public static final class VisionConstants {

    //string names for lime lights
    public static final String vis_LL1name = "limelight";
    
    //valid tag IDs, set the valid IDs to only ones that are normally visible by the LL based on it's location
    public static final int[] vis_validIDs = new int[] {
        6,
        7,
        8,
        9,
        10,
        11,
        17,
        18,
        19,
        20,
        21,
        22 //set according to what tags should be observed, reef only currently
    };

    //position of LL1 in 3d space in robot, center origin, NWU/CCW+ convention
    public static final double[] vis_LL1position = new double[] {
        0, //X forward
        0, //Y side
        0, //Z up
        0, //roll
        0, //pitch (up/down)
        0 //yaw (left/right)
    };

    public static final Vector<N3> vis_stddevs = VecBuilder.fill(0.6,0.6,9999999);
  }
  public static final class ElevatorLiftConstants {
    //node IDs
    public static final int elev_leftID = 0;
    public static final int elev_rightID = 0;
    //controller settings
    public static final int elev_maxcurrent = 40;
    public static final double elev_posfactor = 0.02149; //meters
    public static final double elev_velfactor = elev_posfactor / 60; //meters per second
    //PIDF settings
    public static final double elev_maxvel = 1.9; //meters per second
    public static final double elev_maxacl = elev_maxvel * 1; //meters per second ^ 2
    public static final double elev_FFkS = 0.15; //volts
    public static final double elev_FFkV = 12 / elev_maxvel; //volts per m/s
    public static final double elev_FFkG = 0.2; //volts
    public static final double elev_FFkA = 0;
    public static final double elev_PIDkP = 1.2;
    public static final double elev_PIDkD = 0;
  }

  public static final class ElevatorTiltConstants {
    //node IDs
    public static final int tilt_leftID = 0;
    public static final int tilt_rightID = 0;
    //controller settings
    public static final int tilt_maxcurrent = 40;
  }

  public static final class ManipulatorConstants {
    //controller settings
    public static final int wrist_maxcurrent = 40;
    public static final int spike_maxcurrent = 20;
    public static final double wrist_posfactor =  0.062831; //rads
    public static final double wrist_velfactor = wrist_posfactor / 60; //rads/s
    public static final double wrist_maxrational = 0;
    //PIDF settings, needs characterized
    public static final double wrist_maxvel = 5.94389; //rads
    public static final double wrist_maxaccel = 3*Math.PI; //rads/s, smol
    public static final double wrist_FFkS = 0.05;
    public static final double wrist_FFkV = 12 / wrist_maxvel;
    public static final double wrist_FFkG = 0.2;
    public static final double wrist_FFkA = 0;
    public static final double wrist_PIDkP = 1;
    public static final double wrist_PIDkD = 0;
  }

  //these must be measured and adjusted accordingly
  public static final class ElevatorCalibration {
    //reef height calibration data
    public static final double elev_L1height = 0.3;
    public static final double elev_L2height = 0.8;
    public static final double elev_L3height = 1.2;
    public static final double elev_L4height = 1.84;
    public static final double elev_dealgae1lowheight = 0.3;
    public static final double elev_dealgae1highheight = 0.6;
    public static final double elev_dealgae2lowheight = 0.7;
    public static final double elev_dealgae2highheight = 1;
    public static final double elev_homeheight= 0;
    public static final double elev_maxheight = 1.9;
    //reef angle calibration data
    public static final double wrist_startangle = 1;
    public static final double wrist_L1angle = 0;
    public static final double wrist_L2angle = -0.7;
    public static final double wrist_L3angle = -0.7;
    public static final double wrist_L4angle = -0.67;
    public static final double wrist_dealgae1angle = 0;
    public static final double wrist_dealgae2angle = 0;
    public static final double wrist_homeangle = 1.1;
  }

  //these must be measured and adjusted accordingly, angles for heading control
  public static final class DrivetrainCalibration {
    public static final double drive_leftcoralstation = 0;
    public static final double drive_rightcoralstation = 0;
    public static final double drive_reefA = 0;
    public static final double drive_reefB = 0;
    public static final double drive_reefC = 0;
    public static final double drive_reefD = 0;
    public static final double drive_reefE = 0;
    public static final double drive_reefF = 0;
  }
}