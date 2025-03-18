// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

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
import frc.robot.commands.elevator.elevatorLock;
import frc.robot.commands.elevator.elevatorMain;
import frc.robot.commands.elevator.elevatorOverride;
import frc.robot.commands.elevator.manipOuttake;

import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.elevator.ElevatorLift;
import frc.robot.subsystems.elevator.ElevatorTilt;
import frc.robot.subsystems.elevator.ManipulatorSpike;
import frc.robot.subsystems.elevator.ManipulatorWrist;

import frc.robot.utils.Constants.ElevatorCalibration;
import frc.robot.utils.Constants.OIConstants;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {    
    // controllers
    private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
    private final CommandJoystick m_buttons = new CommandJoystick(OIConstants.kOperatorControllerPort);
    private final CommandXboxController m_operatorCOntroller = new CommandXboxController(2);
    
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ElevatorLift m_ElevatorLift = new ElevatorLift();
    private final ElevatorTilt m_ElevatorTilt = new ElevatorTilt();
    private final ManipulatorWrist m_ManipulatorWrist = new ManipulatorWrist();
    private final ManipulatorSpike m_ManipulatorSpike = new ManipulatorSpike();
    private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    // main commands
    private final elevatorLock c_ElevatorLock = new elevatorLock(m_ElevatorTilt);
    private final elevatorMain c_ElevatorMain = new elevatorMain(m_ElevatorLift, m_ManipulatorWrist);
    private final elevatorOverride c_ElevatorOverride = new elevatorOverride(m_ElevatorLift, m_ManipulatorWrist);
    private final manipIntake c_ManipIntake = new manipIntake(m_ManipulatorSpike);
    private final manipOuttake c_ManipOuttake = new manipOuttake(m_ManipulatorSpike);

    // setpoint commands for setting elevator pose for scoring
    private final InstantCommand c_PoseHome = new InstantCommand(
            () -> c_ElevatorMain.setReference(ElevatorCalibration.elev_homeheight, ElevatorCalibration.wrist_homeangle, true));
    private final InstantCommand c_PoseL1 = new InstantCommand(
            () -> c_ElevatorMain.setReference(ElevatorCalibration.elev_L1height, ElevatorCalibration.wrist_L1angle, true));
    private final InstantCommand c_PoseL2 = new InstantCommand(
            () -> c_ElevatorMain.setReference(ElevatorCalibration.elev_L2height, ElevatorCalibration.wrist_L2angle, false));
    private final InstantCommand c_PoseL3 = new InstantCommand(
            () -> c_ElevatorMain.setReference(ElevatorCalibration.elev_L3height, ElevatorCalibration.wrist_L3angle, false));
    private final InstantCommand c_PoseL4 = new InstantCommand(
            () -> c_ElevatorMain.setReference(ElevatorCalibration.elev_L4height, ElevatorCalibration.wrist_L4angle, false));

    // commands for setting elevator pose for de-algae routines
    private final InstantCommand c_PoseDealgae1Low = new InstantCommand(
            () -> c_ElevatorMain.setReference(ElevatorCalibration.elev_dealgae1lowheight,
                    ElevatorCalibration.wrist_L1angle, true));

    private final InstantCommand c_PoseDealgae2Low = new InstantCommand(
            () -> c_ElevatorMain.setReference(ElevatorCalibration.elev_dealgae2lowheight,
                    ElevatorCalibration.wrist_L1angle, true));

    private final SequentialCommandGroup c_DeAlgae1 = new SequentialCommandGroup(
            new InstantCommand(
                    () -> c_ElevatorMain.setReference(
                            ElevatorCalibration.elev_dealgae1highheight,
                            ElevatorCalibration.wrist_L1angle,
                            true)),
            waitUntil(
                    () -> m_ElevatorLift.atSetpoint())
                    .andThen(run(() -> m_robotDrive.drive(-0.25, 0, 0, false, false, 0))
                    .withTimeout(0.5)));
    private final SequentialCommandGroup c_DeAlgae2 = new SequentialCommandGroup(
            new InstantCommand(
                    () -> c_ElevatorMain.setReference(
                            ElevatorCalibration.elev_dealgae2highheight,
                            ElevatorCalibration.wrist_L1angle,
                            true)),
            waitUntil(
                    () -> m_ElevatorLift.atSetpoint())
                    .andThen(run(() -> m_robotDrive.drive(-0.25, 0, 0, false, false, 0))
                    .withTimeout(0.5)));

        private final SequentialCommandGroup c_BackOutHome = new SequentialCommandGroup(
                run(() -> m_robotDrive.drive(-0.2, 0, 0, false, false, 0)).withTimeout(0.55),
                c_PoseHome);

    // Auto chooser
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    //event triggers for path planner
    private final EventTrigger LockEvent = new EventTrigger("Lock Event");
    private final EventTrigger HomeEvent = new EventTrigger("Home Event");
    private final EventTrigger L1Event = new EventTrigger("L1 Event");
    private final EventTrigger L2Event = new EventTrigger("L2 Event");
    private final EventTrigger L3Event = new EventTrigger("L3 Event");
    private final EventTrigger L4Event = new EventTrigger("L4 Event");
    private final EventTrigger IntakeEvent = new EventTrigger("Intake Event");
    private final EventTrigger OuttakeEvent = new EventTrigger("Outtake Event");

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

        // start datalog
        DataLogManager.start();

        // run autobuilder for drivetrain
        m_robotDrive.runAutoBuilder();

        // triggers for path planner events
        LockEvent.onTrue(c_ElevatorLock);
        HomeEvent.onTrue(c_PoseHome);
        L1Event.onTrue(c_PoseL1);
        L2Event.onTrue(c_PoseL2);
        L3Event.onTrue(c_PoseL3);
        L4Event.onTrue(c_PoseL4);
        IntakeEvent.onTrue(c_ManipIntake);
        OuttakeEvent.onTrue(c_ManipOuttake);

        // register commands
        NamedCommands.registerCommand("Lock Cmd", c_ElevatorLock);
        NamedCommands.registerCommand("Home Cmd", c_PoseHome);
        NamedCommands.registerCommand("L1 Cmd", c_PoseL1);
        NamedCommands.registerCommand("L2 Cmd", c_PoseL2);
        NamedCommands.registerCommand("L3 Cmd", c_PoseL3);
        NamedCommands.registerCommand("L4 Cmd", c_PoseL4);
        NamedCommands.registerCommand("Intake Cmd", c_ManipIntake);
        NamedCommands.registerCommand("Outtake Cmd", c_ManipOuttake);
        
        // add autos and post chooser to smart dashboard
        // setDefaultOption() functions same as addOption, except it sets auto as
        // default
        autoChooser.setDefaultOption("Auto 1", AutoBuilder.buildAuto("Auto 1"));
        // autoChooser.addOption("Auto 2", AutoBuilder.buildAuto("Auto 2"));
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

        // set default command for elevator, requires both manipulator and elevator
        // subsystem
        m_ElevatorLift.setDefaultCommand(c_ElevatorMain);

        //set requirements for de-algae subroutines to allow priority to drive backwards
        c_DeAlgae1.addRequirements(m_robotDrive);
        c_DeAlgae2.addRequirements(m_robotDrive);

        c_BackOutHome.addRequirements(m_robotDrive);
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
                                -MathUtil.applyDeadband(m_driverController.getRightTriggerAxis(),
                                        OIConstants.kDriveDeadband))
                                * 0.04)));

        // schedule override command, should default back to automatic when canceled
        m_buttons.button(5).whileTrue(c_ElevatorOverride); // run command when override is enabled, should override main

        // buttons for override command
        m_buttons.button(7)
                .whileTrue(runEnd(() -> c_ElevatorOverride.liftVolts(6), () -> c_ElevatorOverride.liftVolts(0))); // up
        m_buttons.button(8)
                .whileTrue(runEnd(() -> c_ElevatorOverride.liftVolts(-6), () -> c_ElevatorOverride.liftVolts(0))); // down
        m_buttons.button(9)
                .whileTrue(runEnd(() -> c_ElevatorOverride.tiltVolts(3), () -> c_ElevatorOverride.tiltVolts(0))); // up
        m_buttons.button(10)
                .whileTrue(runEnd(() -> c_ElevatorOverride.tiltVolts(-3), () -> c_ElevatorOverride.tiltVolts(0))); // down

        // manipulator in and out commands
        m_buttons.button(12).onTrue(c_ManipIntake);
        m_buttons.button(11).onTrue(c_ManipOuttake);

        // elevator CL setpoint commands
        m_buttons.button(7).and(() -> !c_ElevatorOverride.isScheduled()).onTrue(c_PoseL4);
        m_buttons.button(8).and(() -> !c_ElevatorOverride.isScheduled()).onTrue(c_PoseL3);
        m_buttons.button(9).and(() -> !c_ElevatorOverride.isScheduled()).onTrue(c_PoseL2);
        m_buttons.button(10).and(() -> !c_ElevatorOverride.isScheduled()).onTrue(c_PoseL1);
        m_buttons.button(6).and(() -> !c_ElevatorOverride.isScheduled()).onTrue(c_BackOutHome);

        // elevator de-algae routines, change buttons!!!
        m_operatorCOntroller.a().and(() -> !c_ElevatorOverride.isScheduled()).onTrue(c_PoseDealgae1Low).onFalse(c_DeAlgae1);
        m_operatorCOntroller.b().and(() -> !c_ElevatorOverride.isScheduled()).onTrue(c_PoseDealgae2Low).onFalse(c_DeAlgae2);
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
        SmartDashboard.putBoolean("elevatorManual scheduled", c_ElevatorOverride.isScheduled());
        SmartDashboard.putBoolean("manipintake scheduled", c_ManipIntake.isScheduled());
        SmartDashboard.putBoolean("manipouttake scheduled", c_ManipOuttake.isScheduled());
        SmartDashboard.putBoolean("poseHome scheduled", c_PoseHome.isScheduled());
        SmartDashboard.putBoolean("poseL1 scheduled", c_PoseL1.isScheduled());
    }
}
