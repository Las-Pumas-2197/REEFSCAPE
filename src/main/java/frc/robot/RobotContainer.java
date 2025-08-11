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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.commands.elevator.manipIntake;
import frc.robot.commands.elevator.elevatorLock;
import frc.robot.commands.elevator.elevatorMain;
import frc.robot.commands.elevator.elevatorOverride;
import frc.robot.commands.elevator.manipOuttake;
import frc.robot.subsystems.ancilliary.LLVision;
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
    private final CommandXboxController m_driverController = new CommandXboxController(
            OIConstants.kDriverControllerPort);
    private final CommandJoystick m_operatorbuttons = new CommandJoystick(OIConstants.kOperatorButtonsPort);
    private final CommandJoystick m_driverbuttons = new CommandJoystick(OIConstants.kDriverButtonsPort);
    private final Timer debounceTimer;
    // The robot's subsystems
    private final DriveSubsystem m_robotDrive = new DriveSubsystem();
    private final ElevatorLift m_ElevatorLift = new ElevatorLift();
    private final ElevatorTilt m_ElevatorTilt = new ElevatorTilt();
    private final ManipulatorWrist m_ManipulatorWrist = new ManipulatorWrist();
    private final ManipulatorSpike m_ManipulatorSpike = new ManipulatorSpike();
    private final LLVision m_LLvision = new LLVision(m_robotDrive);

    // subclassed commands
    private final elevatorLock c_ElevatorLock = new elevatorLock(m_ElevatorTilt);
    private final elevatorMain c_ElevatorMain = new elevatorMain(m_ElevatorLift, m_ManipulatorWrist);
    private final elevatorOverride c_ElevatorOverride = new elevatorOverride(m_ElevatorLift, m_ManipulatorWrist);
    private final manipIntake c_ManipIntake = new manipIntake(m_ManipulatorSpike);
    private final manipOuttake c_ManipOuttake = new manipOuttake(m_ManipulatorSpike);

    // setpoint commands for setting elevator pose for scoring
    private final InstantCommand c_PoseHome = new InstantCommand(
            () -> c_ElevatorMain.setReference(ElevatorCalibration.elev_homeheight, ElevatorCalibration.wrist_homeangle));

    private final InstantCommand c_PoseL1 = new InstantCommand(
            () -> c_ElevatorMain.setReference(ElevatorCalibration.elev_L1height, ElevatorCalibration.wrist_L1angle));

    private final InstantCommand c_PoseL2 = new InstantCommand(
            () ->  c_ElevatorMain.setReference(ElevatorCalibration.elev_L2height, ElevatorCalibration.wrist_L2angle));

    private final InstantCommand c_PoseL3 = new InstantCommand(
            () -> c_ElevatorMain.setReference(ElevatorCalibration.elev_L3height, ElevatorCalibration.wrist_L3angle));

    private final InstantCommand c_PoseL4 = new InstantCommand(
            () -> c_ElevatorMain.setReference(ElevatorCalibration.elev_L4height, ElevatorCalibration.wrist_L4angle));

    private final InstantCommand c_PoseL4aim = new InstantCommand(
            () -> c_ElevatorMain.setReference(ElevatorCalibration.elev_L4height, ElevatorCalibration.wrist_L4angleaim));

    private final InstantCommand c_PoseLoad = new InstantCommand(
            () -> c_ElevatorMain.setReference(ElevatorCalibration.elev_loadheight, ElevatorCalibration.wrist_loadangle));

    // sequential command to back up and home
    private final SequentialCommandGroup c_OutBackHome = new SequentialCommandGroup(
            new manipOuttake(m_ManipulatorSpike),
            run(() -> m_robotDrive.drive(-0.25, 0, 0, false, false, 0), m_robotDrive)
                    .withTimeout(0.5),
            new InstantCommand(() -> c_ElevatorMain.setReference(ElevatorCalibration.elev_homeheight, ElevatorCalibration.wrist_homeangle)));

    private final SequentialCommandGroup c_HomeIntake = new SequentialCommandGroup(
            new InstantCommand(() -> c_ElevatorMain.setReference(ElevatorCalibration.elev_homeheight, ElevatorCalibration.wrist_homeangle)),
            waitSeconds(0.5),
            new manipIntake(m_ManipulatorSpike));


    // commands for setting elevator pose for de-algae routines
    private final InstantCommand c_PoseDealgae1Low = new InstantCommand(
            () -> c_ElevatorMain.setReference(
                    ElevatorCalibration.elev_dealgae1lowheight,
                    ElevatorCalibration.wrist_L1angle));

    private final InstantCommand c_PoseDealgae1High = new InstantCommand(
            () -> c_ElevatorMain.setReference(
                    ElevatorCalibration.elev_dealgae1highheight,
                    ElevatorCalibration.wrist_L1angle));

    private final InstantCommand c_PoseDealgae2Low = new InstantCommand(
            () -> c_ElevatorMain.setReference(
                    ElevatorCalibration.elev_dealgae2lowheight,
                    ElevatorCalibration.wrist_L1angle));

    private final InstantCommand c_PoseDealgae2High = new InstantCommand(
            () -> c_ElevatorMain.setReference(
                    ElevatorCalibration.elev_dealgae2highheight,
                    ElevatorCalibration.wrist_L1angle));
    
    private final RunCommand c_bumpleft = new RunCommand(
            () -> m_robotDrive.drive(0.0, 0.1, 0.0, false, false, 0), m_robotDrive);

    private final RunCommand c_bumpright = new RunCommand(
            () -> m_robotDrive.drive(0.0, -0.1, 0.0, false, false, 0), m_robotDrive);

    private final RunCommand c_bumpfwd = new RunCommand(
            () -> m_robotDrive.drive(0.1, 0, 0.0, false, false, 0), m_robotDrive);

    private final RunCommand c_bumprev = new RunCommand(
            () -> m_robotDrive.drive(-0.1, 0, 0.0, false, false, 0), m_robotDrive);

    //de-algae routines
    private final SequentialCommandGroup c_DeAlgae1 = new SequentialCommandGroup(
            c_PoseDealgae1High,
            waitSeconds(0.5),
            run(
                    () -> m_robotDrive.drive(-0.25, 0, 0, false, false, 0))
                    .withTimeout(0.5),
            new InstantCommand(
                    () -> c_ElevatorMain.setReference(ElevatorCalibration.elev_homeheight, ElevatorCalibration.wrist_homeangle)));

    private final SequentialCommandGroup c_DeAlgae2 = new SequentialCommandGroup(
            c_PoseDealgae2High,
            waitSeconds(0.5),
            run(
                    () -> m_robotDrive.drive(-0.25, 0, 0, false, false, 0))
                    .withTimeout(0.5),
            new InstantCommand(
                    () -> c_ElevatorMain.setReference(ElevatorCalibration.elev_homeheight, ElevatorCalibration.wrist_homeangle)));

    // Auto chooser
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // event triggers for path planner
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
    private double drivespeedmult;
    private boolean debounce = false;

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        m_LLvision.register();
        // start datalog
        DataLogManager.start();
        debounceTimer = new Timer();
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
        // setDefaultOption() functions same as addOption, except it sets auto as default
        autoChooser.setDefaultOption("2 Coral Right", AutoBuilder.buildAuto("Auto A"));
        autoChooser.addOption("1 Coral Right", AutoBuilder.buildAuto("Auto B"));
        autoChooser.addOption("Leave Left", AutoBuilder.buildAuto("Leave A"));
        autoChooser.addOption("Leave Middle", AutoBuilder.buildAuto("Leave B"));
        autoChooser.addOption("1 Coral Middle", AutoBuilder.buildAuto("Auto C"));
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
        drivespeedmult = 0.5;

        // Configure default commands

        m_robotDrive.setDefaultCommand(
                // The left stick controls translation of the robot.
                // Turning is controlled by the X axis of the right stick.
                // Cubed to make fine control easier.
                new RunCommand(
                        () -> m_robotDrive.drive(
                                -Math.pow(
                                        MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                                        OIConstants.kDriveAxisExponent) * drivespeedmult,
                                -Math.pow(
                                        MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                                        OIConstants.kDriveAxisExponent) * drivespeedmult,
                                -Math.pow(
                                        MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                                        OIConstants.kDriveAxisExponent),
                                fieldOriented,
                                useHeadingCorrection,
                                headingtransformed),
                        m_robotDrive));

        // set default command for elevator, requires both manipulator and elevator
        // subsystem
        m_ElevatorLift.setDefaultCommand(c_ElevatorMain);

        // set requirements for misc commands
        c_DeAlgae1.addRequirements(m_robotDrive);
        c_DeAlgae2.addRequirements(m_robotDrive);
    }

    private void configureButtonBindings() {

        // drive system bindings

        // set modules to be in X position to block
        m_driverController.rightBumper().whileTrue(run(() -> m_robotDrive.setX(), m_robotDrive));

        // zero heading and odometry as needed
        m_driverController.a().onTrue(runOnce(() -> m_robotDrive.zeroHeading()));

        //m_driverController.x().onTrue(c_ElevatorLock);

        // runs first lambda when depressed, runs second lambda when released
        m_driverController.y().whileTrue(runEnd(() -> fieldOriented = false, () -> fieldOriented = true));
        m_driverController.y().whileTrue(runEnd(() -> drivespeedmult = 0.15, () -> drivespeedmult = 0.5));

        m_driverController.leftBumper().whileTrue(runEnd(() -> drivespeedmult = 1, () -> drivespeedmult = 0.5));

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

        //bump left and right for positioning
        m_driverController.povLeft().whileTrue(c_bumpleft);
        m_driverController.povRight().whileTrue(c_bumpright);
        m_driverController.povUp().whileTrue(c_bumpfwd);
        m_driverController.povDown().whileTrue(c_bumprev);

        // schedule override command, should default back to automatic when canceled
        m_operatorbuttons.button(2).whileTrue(c_ElevatorOverride); // run command when override is enabled, should override main
        
        // elevator de-algae routines, change buttons!!!
        m_operatorbuttons.button(4).and(() -> !c_ElevatorOverride.isScheduled()).onTrue(c_PoseDealgae1Low)
                .onFalse(c_DeAlgae1);
        m_operatorbuttons.button(3).and(() -> !c_ElevatorOverride.isScheduled()).onTrue(c_PoseDealgae2Low)
                .onFalse(c_DeAlgae2);
        // buttons for override command
        m_operatorbuttons.button(5)
                .whileTrue(runEnd(() -> c_ElevatorOverride.liftVolts(6), () -> c_ElevatorOverride.liftVolts(0))); // up
        m_operatorbuttons.button(6)
                .whileTrue(runEnd(() -> c_ElevatorOverride.liftVolts(-6), () -> c_ElevatorOverride.liftVolts(0))); // down
        m_operatorbuttons.button(7)
                .whileTrue(runEnd(() -> c_ElevatorOverride.tiltVolts(3), () -> c_ElevatorOverride.tiltVolts(0))); // up
        m_operatorbuttons.button(8)
                .whileTrue(runEnd(() -> c_ElevatorOverride.tiltVolts(-3), () -> c_ElevatorOverride.tiltVolts(0))); // down

        // manipulator in and out commands
        m_operatorbuttons.button(9).onTrue(c_HomeIntake);
        m_operatorbuttons.button(10).onTrue(c_OutBackHome);

        // elevator CL setpoint commands
        m_operatorbuttons.button(5).and(() -> !c_ElevatorOverride.isScheduled()).onFalse(c_PoseL4);

        m_operatorbuttons.button(6).and(() -> !c_ElevatorOverride.isScheduled()).onFalse(c_PoseL3);
        m_operatorbuttons.button(7).and(() -> !c_ElevatorOverride.isScheduled()).onFalse(c_PoseL2);
        m_operatorbuttons.button(8).and(() -> !c_ElevatorOverride.isScheduled()).onFalse(c_PoseL1);

        // home routines
        m_operatorbuttons.button(11).and(() -> !c_ElevatorOverride.isScheduled()).onFalse(c_PoseLoad);
        m_operatorbuttons.button(12).and(() -> !c_ElevatorOverride.isScheduled()).onFalse(c_PoseHome);

        // heading setpoint buttons
        m_driverbuttons.button(5).onTrue(runOnce(() -> headingtransformed = Units.degreesToRadians(-60)));
        m_driverbuttons.button(6).onTrue(runOnce(() -> headingtransformed = Units.degreesToRadians(-120)));
        m_driverbuttons.button(7).onTrue(runOnce(() -> headingtransformed = Units.degreesToRadians(180)));
        m_driverbuttons.button(8).onTrue(runOnce(() -> headingtransformed = Units.degreesToRadians(120)));
        m_driverbuttons.button(9).onTrue(runOnce(() -> headingtransformed = Units.degreesToRadians(60)));
        m_driverbuttons.button(10).onTrue(runOnce(() -> headingtransformed = Units.degreesToRadians(0)));
        m_driverbuttons.button(11).onTrue(runOnce(() -> headingtransformed = Units.degreesToRadians(-125)));
        m_driverbuttons.button(12).onTrue(runOnce(() -> headingtransformed = Units.degreesToRadians(125)));

    }

    public Command selectedAutonomous() {
        // return c_ElevatorLock.andThen(autoChooser.getSelected()); //for comp only
        return autoChooser.getSelected();
    }

    public void telemetry() {
        m_field.setRobotPose(m_robotDrive.getPose());
        SmartDashboard.putData("odometry field", m_field);

        SmartDashboard.putNumber("odometry pose X", m_robotDrive.getPose().getX());
        SmartDashboard.putNumber("odometry pose Y", m_robotDrive.getPose().getY());
        SmartDashboard.putNumber("odometry pose Z", m_robotDrive.getPose().getRotation().getRadians());

        SmartDashboard.putNumber("vision pose X", m_LLvision.getVisionPose().getX());
        SmartDashboard.putNumber("vision pose Y", m_LLvision.getVisionPose().getY());
        SmartDashboard.putNumber("vision pose Z", m_LLvision.getVisionPose().getRotation().getRotations());

        // Elevator Encoders
        double[] ElevatorEncoders = m_ElevatorLift.getEncoderPositions();
        SmartDashboard.putNumber("Elevator Encoder Avg", ElevatorEncoders[2]);
        SmartDashboard.putNumber("Manipulator Position", m_ManipulatorWrist.getAngle());
        //NOT TELEMETRY
        if(debounce == true){
                if(debounceTimer.get() >= 5){
                        debounceTimer.stop();
                        debounceTimer.reset();
                        debounce = false;
         }
        }
    }
}
