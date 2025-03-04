// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Manipulator;
import frc.robot.utils.Constants.ElevatorCalibration;
import frc.robot.utils.Constants.OIConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

  //used in operation of elevator
  private boolean elevator_enableCL; //true = CL enabled, false = OL enabled
  private double elevator_CLheight;
  private double elevator_OLvolts;
  private static final double elevator_CLheightinc = 0.001; //amount to increment per scheduler cycle. multiply by 50 to get meters per second

  //used in operation of manipulator
  private double manipulator_angleCL;
  private double manipulator_OLvolts;
  private double manipulator_spinvolts;

  //value to store which pose is desired for the elevator
  //0 = homed and retracted
  //1 = loading
  //2 = L1
  //3 = L2
  //4 = L3
  //5 = L4
  private int elevator_pose;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();

    //camera server
    CameraServer.startAutomaticCapture();
    CameraServer.startAutomaticCapture();
    //set starting options for drive
    useHeadingCorrection = false;
    fieldOriented = true;
    headingtransformed = 0;

    //set starting options for elevator
    elevator_enableCL = false;
    elevator_pose = 0;

    //set starting config for manipulator

    // Configure default commands
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

    //automatically run elevator
    m_Elevator.setDefaultCommand(
      new RunCommand(
        () -> m_Elevator.runElevator(
          elevator_enableCL,
          elevator_CLheight,
          elevator_OLvolts
        ),
      m_Elevator));

    //automatically run manipulator
    m_Manipulator.setDefaultCommand(
      new RunCommand(
        () -> m_Manipulator.runManipulator(
          elevator_enableCL,
          manipulator_angleCL,
          manipulator_OLvolts,
          manipulator_spinvolts
        ),
      m_Manipulator));
    
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
    m_driverController.rightStick().onTrue(runOnce(() -> useHeadingCorrection = useHeadingCorrection ? false : true));

    //increment desired heading data in inline command
    m_driverController.leftTrigger(OIConstants.kDriveDeadband)
      .or(m_driverController.rightTrigger(OIConstants.kDriveDeadband))
      .whileTrue(run(() -> headingtransformed = headingtransformed + 
      ((MathUtil.applyDeadband(m_driverController.getLeftTriggerAxis(), OIConstants.kDriveDeadband) + 
      -MathUtil.applyDeadband(m_driverController.getRightTriggerAxis(), OIConstants.kDriveDeadband))*0.04)));

    



    
    //elevator state toggle between OL and CL, temporarily disabled for testing
    m_operatorController.leftBumper().onTrue(runOnce(() -> elevator_enableCL = elevator_enableCL ? false : true));





    //manipulator bindings

    //manipulator primitives

    m_operatorController.y().whileTrue(runEnd(() -> manipulator_OLvolts = 3, () -> manipulator_OLvolts = 0));
    m_operatorController.a().whileTrue(runEnd(() -> manipulator_OLvolts = -3, () -> manipulator_OLvolts = 0));
    m_operatorController.x().whileTrue(runEnd(() -> manipulator_spinvolts = 12, () -> manipulator_spinvolts= 0));
    m_operatorController.b().whileTrue(runEnd(() -> manipulator_spinvolts = -12, () -> manipulator_spinvolts = 0));

    //reset manipulator position
    m_operatorController.rightBumper().whileTrue(runOnce(() -> m_Manipulator.resetEncoder()));
    
    //manipulator angle closed loop
    //m_operatorController.y().onTrue(runOnce(() -> manipulator_angleCL = -0.25 * Math.PI));
    //m_operatorController.x().onTrue(runOnce(() -> manipulator_angleCL = -1 * Math.PI));
    



  

    //tilt forward and back if locks fail
    m_operatorController.povLeft().whileTrue(runEnd(() -> m_Elevator.tiltSetVoltage(3), () -> m_Elevator.tiltSetVoltage(0)));
    m_operatorController.povRight().whileTrue(runEnd(() -> m_Elevator.tiltSetVoltage(-3), () -> m_Elevator.tiltSetVoltage(0)));






    //elevator up and down in OL
    m_operatorController.povUp().whileTrue(runEnd(() -> elevator_OLvolts = 6, () -> elevator_OLvolts = 0));
    m_operatorController.povDown().whileTrue(runEnd(() -> elevator_OLvolts = -6, () -> elevator_OLvolts = 0));

    //increment height setpoint up and down when in CL
    //m_operatorController.povUp().whileTrue(run(() -> elevator_CLheight = elevator_CLheight + elevator_CLheightinc));
    //m_operatorController.povDown().whileTrue(run(() -> elevator_CLheight = elevator_CLheight - elevator_CLheightinc));

    //cancels default command and calls homing routine should automatically call default command again when finished
    //m_operatorController.leftStick().onTrue();







    //triggers for actions when elevator pose is changed

    //check if button is presed and pose is within limits, then iterate up or down
    m_operatorController.start().and(() -> elevator_pose < 5).whileTrue(runOnce(() -> elevator_pose = elevator_pose + 1));
    m_operatorController.back().and(() -> elevator_pose > 0).whileTrue(runOnce(() -> elevator_pose = elevator_pose - 1));

    //new Trigger(() -> elevator_pose == 0).onTrue(null); //retracted
    new Trigger(() -> elevator_pose == 1).onTrue(runOnce(() -> elevator_CLheight = ElevatorCalibration.elev_loadheight)); //loading
    new Trigger(() -> elevator_pose == 2).onTrue(runOnce(() -> elevator_CLheight = ElevatorCalibration.elev_L1height)); //L1
    new Trigger(() -> elevator_pose == 3).onTrue(runOnce(() -> elevator_CLheight = ElevatorCalibration.elev_L2height)); //L2
    new Trigger(() -> elevator_pose == 4).onTrue(runOnce(() -> elevator_CLheight = ElevatorCalibration.elev_L3height)); //L3
    new Trigger(() -> elevator_pose == 5).onTrue(runOnce(() -> elevator_CLheight = ElevatorCalibration.elev_L4height)); //L4
  }

  public Command initializeElevator(){
    return m_Elevator.lockElevator();
  }

  public Command exampleauto() {
    return auto1;
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

    //elevator limit switches
    SmartDashboard.putBoolean("lower limit", m_Elevator.getSwitchStatuses()[0]);
    SmartDashboard.putBoolean("upper limit", m_Elevator.getSwitchStatuses()[1]);

    //elevator control states
    SmartDashboard.putBoolean("elevator CL state", elevator_enableCL);
    SmartDashboard.putNumber("elevator CL height", elevator_CLheight);
    SmartDashboard.putNumber("elevator pose", elevator_pose);
    SmartDashboard.putNumber("elevator OL volts", elevator_OLvolts);
    SmartDashboard.putBoolean("elevator homing command state", m_Elevator.elevatorHome().isFinished());
    SmartDashboard.putBoolean("elevator run command state", m_Elevator.getDefaultCommand().isScheduled());

    //internal elevator stuff
    SmartDashboard.putNumber("elevator volts", m_Elevator.getVolts()[0]);
    SmartDashboard.putNumber("slewed elevator volts", m_Elevator.getVolts()[1]);

    //manipulator position
    SmartDashboard.putNumber("manipulator position", m_Manipulator.getAngle());
  }
}
