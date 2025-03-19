// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorLift;
import frc.robot.subsystems.elevator.ManipulatorWrist;

public class elevatorMain extends Command {

  //injected subsystems
  private final ElevatorLift m_ElevatorLift;
  private final ManipulatorWrist m_ManipulatorWrist;

  //variables used for manipulating setpoint
  private double setpointAngle;
  private double setpointHeight;

  public elevatorMain(ElevatorLift elevatorlift, ManipulatorWrist manipulatorwrist) {

    // injected subsytems and add requirements for command
    m_ElevatorLift = elevatorlift;
    m_ManipulatorWrist = manipulatorwrist;
    addRequirements(m_ElevatorLift, m_ManipulatorWrist);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    // set angle of manipulator first
    m_ManipulatorWrist.setAngle(setpointAngle);
    m_ElevatorLift.setHeight(setpointHeight);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // used by other commands to set PID reference, so this command can be
  // continually
  // called and allow PID controllers to continually calculate, otherwise PID
  // output will "jump" and induce oscillation whenever commands for pose are
  // called
  public void setReference(double height, double angle) {
    setpointHeight = height;
    setpointAngle = angle;
  }
}
