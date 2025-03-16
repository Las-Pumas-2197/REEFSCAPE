// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

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
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Manipulator;
import frc.robot.utils.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

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
  private final CommandJoystick m_buttons = new CommandJoystick(3);

  //Auto Sendable chooser
  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

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

    //run autobuilder
    m_robotDrive.runAutoBuilder();

    //register commands
    NamedCommands.registerCommand("Intake", null);
    NamedCommands.registerCommand("Outtake", null);
    NamedCommands.registerCommand("Lift to L1", null);
    NamedCommands.registerCommand("Lift to L4", null);

    //add autos and post data to smart dashboard
    autoChooser.addOption("Auto 1", AutoBuilder.buildAuto("Auto 1"));
    SmartDashboard.putData("Auto Selector", autoChooser);

    //Configure the button bindings
    configureButtonBindings();

    //camera server
    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();

    //set starting options for drive
    useHeadingCorrection = false;
    fieldOriented = true;
    headingtransformed = 0;

    //Configure default commands

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        //Cubed to make fine control easier.
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

  private void configureButtonBindings() {

    //drive system bindings

    //set modules to be in X position to block
    m_driverController.rightBumper().whileTrue(run(() -> m_robotDrive.setX(), m_robotDrive));

    //zero heading and odometry as needed
    m_driverController.a().onTrue(runOnce(() -> m_robotDrive.zeroHeading()));
    m_driverController.x().onTrue(runOnce(() -> m_robotDrive.resetOdometry(new Pose2d())));

    //runs first lambda when depressed, runs second lambda when released
    m_driverController.y().whileTrue(runEnd(() -> fieldOriented = false, () -> fieldOriented = true));

    //runs every time right stick becomes true, functions as toggle
    m_driverController.b().onTrue(runOnce(() -> useHeadingCorrection = useHeadingCorrection ? false : true));

    //increment desired heading data in inline command
    m_driverController.leftTrigger(OIConstants.kDriveDeadband)
      .or(m_driverController.rightTrigger(OIConstants.kDriveDeadband))
      .whileTrue(run(() -> headingtransformed = headingtransformed + 
      ((MathUtil.applyDeadband(m_driverController.getLeftTriggerAxis(), OIConstants.kDriveDeadband) + 
      -MathUtil.applyDeadband(m_driverController.getRightTriggerAxis(), OIConstants.kDriveDeadband))*0.04)));
    




    //reset encoders on elevator
    m_operatorController.x().onTrue(m_Manipulator.resetEncoder().andThen(m_Elevator.resetEncoderPositions()));

    //elevator OL button bindings
    m_buttons.button(7).whileTrue(null); //up
    m_buttons.button(8).whileTrue(null); //down
    m_buttons.button(9).whileTrue(null); //manpi up
    m_buttons.button(10).whileTrue(null); //manip down

    //elevator CL button bindings
    m_buttons.button(7).onTrue(null); //L4
    m_buttons.button(8).onTrue(null); //L3
    m_buttons.button(9).onTrue(null); //L2
    m_buttons.button(10).onTrue(null); //L1
    m_buttons.button(6).onTrue(null); //home

    //manipulator in and out
    m_buttons.button(11).whileTrue(null);
    m_buttons.button(12).whileTrue(null);

    //auto triggers
  }

  public Command selectedAutonomous() {
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
    SmartDashboard.putNumber("elevator velocity avg", ElevatorEncoders[3]);

    //elevator limit switches
    SmartDashboard.putBoolean("lower limit", m_Elevator.getSwitchStatuses()[0]);
    SmartDashboard.putBoolean("upper limit", m_Elevator.getSwitchStatuses()[1]);

    //internal elevator stuff
    SmartDashboard.putNumber("elevator volts", m_Elevator.getVolts()[0]);
    SmartDashboard.putNumber("slewed elevator volts", m_Elevator.getVolts()[1]);
    SmartDashboard.putNumber("elevator pid volts", m_Elevator.getVolts()[2]);
    SmartDashboard.putNumber("elevator ff volts", m_Elevator.getVolts()[3]);

    //internal manipulator stuff
    SmartDashboard.putNumber("manipulator pid volts", m_Manipulator.getManipulatorData()[0]);
    SmartDashboard.putNumber("manipulator ff volts", m_Manipulator.getManipulatorData()[1]);

    //manipulator position
    SmartDashboard.putNumber("manipulator position", m_Manipulator.getAngle());
    SmartDashboard.putBoolean("manipulator at setpoint", m_Manipulator.atSetpoint());
  }
}
