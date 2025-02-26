// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Manipulator;
import frc.robot.utils.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final Elevator m_Elevator = new Elevator();
  private final Manipulator m_Manipulator = new Manipulator();
  private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(OIConstants.kOperatorControllerPort);

  //auto routines
  private final PathPlannerAuto auto1 = new PathPlannerAuto("Auto1");

  //field 2d object for pose estimation visualization in elastic
  private final Field2d m_field = new Field2d();
  
  //used in operation of drivetrain
  private double headingtransformed;
  private boolean useHeadingCorrection;
  private boolean fieldOriented;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    // Configure default commands
    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                fieldOriented,
                useHeadingCorrection,
                headingtransformed
              ),
            m_robotDrive));
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {

    //set modules to be in X position to block
    m_driverController.rightBumper().whileTrue(run(() -> m_robotDrive.setX(), m_robotDrive));

    //zero heading and odometry as needed
    m_driverController.a().onTrue(runOnce(() -> m_robotDrive.zeroHeading()));
    m_driverController.x().onTrue(runOnce(() -> m_robotDrive.resetOdometry(new Pose2d())));

    //runs first lambda when depressed, runs second lambda when released
    m_driverController.y().whileTrue(runEnd(() -> fieldOriented = false, () -> fieldOriented = true));

    //runs every time right stick becomes true, functions as toggle
    m_driverController.rightStick().onTrue(runOnce(() -> useHeadingCorrection = useHeadingCorrection ? false : true));

    //increment desired heading data in inline command
    m_driverController.leftTrigger(OIConstants.kDriveDeadband)
      .or(m_driverController.rightTrigger(OIConstants.kDriveDeadband))
      .whileTrue(run(() -> headingtransformed = headingtransformed + 
      ((MathUtil.applyDeadband(m_driverController.getLeftTriggerAxis(), OIConstants.kDriveDeadband) + 
      -MathUtil.applyDeadband(m_driverController.getRightTriggerAxis(), OIConstants.kDriveDeadband))*0.04)));

    //tilt forward and back
    m_operatorController.povLeft().whileTrue(runEnd(() -> m_Elevator.tiltSetVoltage(3), () -> m_Elevator.tiltSetVoltage(0)));
    m_operatorController.povRight().whileTrue(runEnd(() -> m_Elevator.tiltSetVoltage(-3), () -> m_Elevator.tiltSetVoltage(0)));

    //operate elevator up and down manually by using left bumper and pov up/down as well as homing
    m_operatorController.povDown().and(m_operatorController.leftBumper()).whileTrue(run(() -> m_Elevator.elevatorSetVoltage(-3, true)));
    m_operatorController.povUp().and(m_operatorController.leftBumper()).whileTrue(run(() -> m_Elevator.elevatorSetVoltage(3, true)));
    m_operatorController.leftStick().and(m_operatorController.leftBumper().whileTrue(runOnce(() -> m_Elevator.elevatorHome())));
  }
  
  //should work, formatted into in-line command above in bindings
  public void updateHeading() {
    headingtransformed = 
    MathUtil.angleModulus(
      headingtransformed + 
      ((MathUtil.applyDeadband(m_driverController.getLeftTriggerAxis(), OIConstants.kDriveDeadband) + 
      -MathUtil.applyDeadband(m_driverController.getRightTriggerAxis(), OIConstants.kDriveDeadband))*0.04)
    );
  }

  public double getJoystickHeading(){
    return MathUtil.angleModulus(-(Math.atan2(m_driverController.getRawAxis(5), -m_driverController.getRawAxis(4))) + 0.5 * Math.PI);
  }

  public Command exampleauto() {
    //return auto1;
    return null;
  }

  public void telemetry() {
    m_field.setRobotPose(m_robotDrive.getPose());
    SmartDashboard.putNumber("Xpos", m_robotDrive.getPose().getX());
    SmartDashboard.putNumber("Ypos", m_robotDrive.getPose().getY());
    SmartDashboard.putNumber("Heading", m_robotDrive.getPose().getRotation().getRadians());
    SmartDashboard.putNumber("FL module drive speed", m_robotDrive.getStates()[0].speedMetersPerSecond);
    SmartDashboard.putNumber("Voltage", pdh.getVoltage());
    SmartDashboard.putNumber("x axis", m_driverController.getLeftX());
    SmartDashboard.putNumber("y axis", m_driverController.getLeftY());
    SmartDashboard.putNumber("z axis", m_driverController.getRightX());
    
    //Chassis Speeds
    SmartDashboard.putNumber("ChassisSpeedX", m_robotDrive.getSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeedY", m_robotDrive.getSpeeds().vyMetersPerSecond);
    SmartDashboard.putNumber("Radians Per Second", m_robotDrive.getSpeeds().omegaRadiansPerSecond);
    SmartDashboard.putNumber("Trigger Heading", headingtransformed);
    SmartDashboard.putNumber("Velocity", Math.sqrt(Math.pow(m_robotDrive.getSpeeds().vxMetersPerSecond, 2) + Math.pow(m_robotDrive.getSpeeds().vyMetersPerSecond, 2)));
    SmartDashboard.putData(m_field);

    //Elevator Encoders
    double[] ElevatorEncoders = m_Elevator.getEncoderPositions();
    SmartDashboard.putNumber("Elevator Encoder Right", ElevatorEncoders[0]);
    SmartDashboard.putNumber("Elevator Encoder Left", ElevatorEncoders[1]);
    SmartDashboard.putNumber("Elevator Encoder Avg", ElevatorEncoders[2]);
  }
}
