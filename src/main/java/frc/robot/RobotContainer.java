// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import java.net.CacheRequest;

import com.fasterxml.jackson.databind.util.Named;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.commands.elevator.manipIntake;
import frc.robot.commands.elevator.manipOuttake;
import frc.robot.commands.elevator.elevatorLock;
import frc.robot.commands.elevator.elevatorManual;
import frc.robot.commands.elevator.poseHome;
import frc.robot.commands.elevator.poseL1;
import frc.robot.commands.elevator.poseL2;
import frc.robot.commands.elevator.poseL3;
import frc.robot.commands.elevator.poseL4;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorLift;
import frc.robot.subsystems.elevator.ElevatorTilt;
import frc.robot.subsystems.elevator.ManipulatorSpike;
import frc.robot.subsystems.elevator.ManipulatorWrist;
import frc.robot.utils.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {
  // The robot's subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final ElevatorLift m_ElevatorLift = new ElevatorLift();
  private final ElevatorTilt m_ElevatorTilt = new ElevatorTilt();
  private final ManipulatorWrist m_ManipulatorWrist = new ManipulatorWrist();
  private final ManipulatorSpike m_ManipulatorSpike = new ManipulatorSpike();
  private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  // commands and subsystems to require
  private final elevatorLock c_ElevatorLock = new elevatorLock(m_ElevatorTilt);
  private final elevatorManual c_ElevatorManual = new elevatorManual(m_ElevatorLift, m_ManipulatorWrist);
  private final manipIntake c_ManipIntake = new manipIntake(m_ManipulatorSpike);
  private final manipOuttake c_ManipOuttake = new manipOuttake(m_ManipulatorSpike);
  private final poseHome c_PoseHome = new poseHome(m_ElevatorLift, m_ManipulatorWrist);
  private final poseL1 c_PoseL1 = new poseL1(m_ElevatorLift, m_ManipulatorWrist);
  private final poseL2 c_PoseL2 = new poseL2(m_ElevatorLift, m_ManipulatorWrist);
  private final poseL3 c_PoseL3 = new poseL3(m_ElevatorLift, m_ManipulatorWrist);
  private final poseL4 c_PoseL4 = new poseL4(m_ElevatorLift, m_ManipulatorWrist);


  // The driver's controller
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandJoystick m_buttons = new CommandJoystick(OIConstants.kOperatorControllerPort);

  // Auto Sendable chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // field 2d object for pose estimation visualization in elastic
  private final Field2d m_field = new Field2d();

  // used in operation of drivetrain
  private double headingtransformed;
  private boolean useHeadingCorrection;
  private boolean fieldOriented;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    //start datalog
    //DataLogManager.start();

    // run autobuilder
    m_robotDrive.runAutoBuilder();

    // register commands
    NamedCommands.registerCommand("Intake", c_ManipIntake);
    NamedCommands.registerCommand("Outtake", c_ManipOuttake);
    NamedCommands.registerCommand("Home", c_PoseHome);
    NamedCommands.registerCommand("Lift to L1", c_PoseL1);
    NamedCommands.registerCommand("Lift to L2", c_PoseL2);
    NamedCommands.registerCommand("Lift to L3", c_PoseL3);
    NamedCommands.registerCommand("Lift to L4", c_PoseL4);

    // add autos and post chooser to smart dashboard
    autoChooser.addOption("Auto 1", AutoBuilder.buildAuto("Auto 1"));
    SmartDashboard.putData("Auto Selector", autoChooser);

    // Configure the button bindings
    configureButtonBindings();

    // camera server
    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();

    // set starting options for drive
    useHeadingCorrection = false;
    fieldOriented = true;
    headingtransformed = 0;

    // Configure default commands

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        // Cubed to make fine control easier.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                fieldOriented,
                useHeadingCorrection,
                headingtransformed),
            m_robotDrive));

    m_ElevatorLift.setDefaultCommand(c_PoseHome); //default command so elevator retracts when no command is scheduled
  }

  private void configureButtonBindings() {

    // drive system bindings

    // set modules to be in X position to block
    m_driverController.rightBumper().whileTrue(run(() -> m_robotDrive.setX(), m_robotDrive));

    // zero heading and odometry as needed
    m_driverController.a().onTrue(runOnce(() -> m_robotDrive.zeroHeading()));
    m_driverController.x().onTrue(runOnce(() -> m_robotDrive.resetOdometry(new Pose2d())));

    // runs first lambda when depressed, runs second lambda when released
    m_driverController.y().whileTrue(runEnd(() -> fieldOriented = false, () -> fieldOriented = true));

    // runs every time right stick becomes true, functions as toggle
    m_driverController.b().onTrue(runOnce(() -> useHeadingCorrection = useHeadingCorrection ? false : true));

    // increment desired heading data in inline command
    m_driverController.leftTrigger(OIConstants.kDriveDeadband)
        .or(m_driverController.rightTrigger(OIConstants.kDriveDeadband))
        .whileTrue(run(() -> headingtransformed = headingtransformed +
            ((MathUtil.applyDeadband(m_driverController.getLeftTriggerAxis(), OIConstants.kDriveDeadband) +
                -MathUtil.applyDeadband(m_driverController.getRightTriggerAxis(), OIConstants.kDriveDeadband))
                * 0.04)));

    // reset encoders on elevator if needed
    // m_operatorController.x().onTrue(m_ManipulatorWrist.resetEncoder().andThen(m_ElevatorLift.resetEncoderPositions()));

    // elevator OL button bindings
    m_buttons.button(5).whileTrue(c_ElevatorManual); // run command when override is enabled, should override all other commands
    m_buttons.button(7).whileTrue(runEnd(() -> c_ElevatorManual.liftvolts(6), () -> c_ElevatorManual.liftvolts(0))); // up
    m_buttons.button(8).whileTrue(runEnd(() -> c_ElevatorManual.liftvolts(-6), () -> c_ElevatorManual.liftvolts(0))); // down
    m_buttons.button(9).whileTrue(runEnd(() -> c_ElevatorManual.tiltvolts(3), () -> c_ElevatorManual.tiltvolts(0))); // manip up
    m_buttons.button(10).whileTrue(runEnd(() -> c_ElevatorManual.tiltvolts(-3), () -> c_ElevatorManual.tiltvolts(0))); // manip down

    // manipulator in and out
    m_buttons.button(12).onTrue(c_ManipIntake);
    m_buttons.button(11).onTrue(c_ManipOuttake);

    // elevator CL button bindings
    m_buttons.button(7).and(() -> !c_ElevatorManual.isScheduled()).onTrue(c_PoseL4); // L4
    m_buttons.button(8).and(() -> !c_ElevatorManual.isScheduled()).onTrue(c_PoseL3); // L3
    m_buttons.button(9).and(() -> !c_ElevatorManual.isScheduled()).onTrue(c_PoseL2); // L2
    m_buttons.button(10).and(() -> !c_ElevatorManual.isScheduled()).onTrue(c_PoseL1); // L1
    m_buttons.button(6).and(() -> !c_ElevatorManual.isScheduled()).onTrue(c_PoseHome); // home

  }

  public Command cancelCommand() {
    return runOnce(() -> m_ElevatorLift.getCurrentCommand().cancel());
  }

  public Command selectedAutonomous() {
    // return c_ElevatorLock.andThen(autoChooser.getSelected()); //for comp only
    return autoChooser.getSelected();
  }

  public void telemetry() {
    m_field.setRobotPose(m_robotDrive.getPose());
    SmartDashboard.putNumber("Xpos", m_robotDrive.getPose().getX());
    SmartDashboard.putNumber("Ypos", m_robotDrive.getPose().getY());
    SmartDashboard.putNumber("Heading", m_robotDrive.getPose().getRotation().getRadians());
    SmartDashboard.putNumber("Voltage", pdh.getVoltage());
    SmartDashboard.putNumber("x axis", m_driverController.getLeftX());
    SmartDashboard.putNumber("y axis", m_driverController.getLeftY());
    SmartDashboard.putNumber("z axis", m_driverController.getRightX());
    SmartDashboard.putBoolean("heading control state", useHeadingCorrection);

    // Chassis Speeds
    SmartDashboard.putNumber("ChassisSpeedX", m_robotDrive.getSpeeds().vxMetersPerSecond);
    SmartDashboard.putNumber("ChassisSpeedY", m_robotDrive.getSpeeds().vyMetersPerSecond);
    SmartDashboard.putNumber("Radians Per Second", m_robotDrive.getSpeeds().omegaRadiansPerSecond);
    SmartDashboard.putNumber("Trigger Heading", headingtransformed);
    SmartDashboard.putNumber("Velocity", Math.sqrt(Math.pow(m_robotDrive.getSpeeds().vxMetersPerSecond, 2)
        + Math.pow(m_robotDrive.getSpeeds().vyMetersPerSecond, 2)));
    SmartDashboard.putData(m_field);

    // Elevator Encoders
    double[] ElevatorEncoders = m_ElevatorLift.getEncoderPositions();
    SmartDashboard.putNumber("Elevator Encoder Right", ElevatorEncoders[0]);
    SmartDashboard.putNumber("Elevator Encoder Left", ElevatorEncoders[1]);
    SmartDashboard.putNumber("Elevator Encoder Avg", ElevatorEncoders[2]);
    SmartDashboard.putNumber("elevator velocity avg", ElevatorEncoders[3]);

    // elevator limit switches
    SmartDashboard.putBoolean("lower limit", m_ElevatorLift.getSwitchStatuses()[0]);
    SmartDashboard.putBoolean("upper limit", m_ElevatorLift.getSwitchStatuses()[1]);

    // internal elevator stuff
    SmartDashboard.putNumber("elevator volts", m_ElevatorLift.getVolts()[0]);
    SmartDashboard.putNumber("slewed elevator volts", m_ElevatorLift.getVolts()[1]);
    SmartDashboard.putNumber("elevator pid volts", m_ElevatorLift.getVolts()[2]);
    SmartDashboard.putNumber("elevator ff volts", m_ElevatorLift.getVolts()[3]);

    // internal manipulator stuff
    SmartDashboard.putNumber("manipulator pid volts", m_ManipulatorWrist.getManipulatorData()[0]);
    SmartDashboard.putNumber("manipulator ff volts", m_ManipulatorWrist.getManipulatorData()[1]);

    // manipulator position
    SmartDashboard.putNumber("manipulator position", m_ManipulatorWrist.getAngle());
    SmartDashboard.putBoolean("manipulator at setpoint", m_ManipulatorWrist.atSetpoint());
    SmartDashboard.putBoolean("elevator at setpoint", m_ElevatorLift.atSetpoint());

    // current commands for subsytems
    SmartDashboard.putData(m_ElevatorTilt.getCurrentCommand());
    SmartDashboard.putData(m_ElevatorLift.getCurrentCommand());
    SmartDashboard.putData(m_ManipulatorWrist.getCurrentCommand());
    SmartDashboard.putData(m_ManipulatorSpike.getCurrentCommand());

    SmartDashboard.putBoolean("elevatorLock scheduled", c_ElevatorLock.isScheduled());
    SmartDashboard.putBoolean("elevatorManual scheduled", c_ElevatorManual.isScheduled());
    SmartDashboard.putBoolean("manipintake scheduled", c_ManipIntake.isScheduled());
    SmartDashboard.putBoolean("manipouttake scheduled", c_ManipOuttake.isScheduled());
    SmartDashboard.putBoolean("poseHome scheduled", c_PoseHome.isScheduled());
    SmartDashboard.putBoolean("poseL1 scheduled", c_PoseL1.isScheduled());
  }
}
