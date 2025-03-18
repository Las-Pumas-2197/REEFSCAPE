// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorLift;
import frc.robot.subsystems.elevator.ManipulatorWrist;

public class elevatorMain extends Command {

  private final ElevatorLift m_ElevatorLift;
  private final ManipulatorWrist m_ManipulatorWrist;

  private double setpointHeight;
  private double setpointAngle;

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

    // call closed loop functions and set references to subsystems
    m_ElevatorLift.setHeight(setpointHeight);
    m_ManipulatorWrist.setAngle(setpointAngle);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  // used by other commands to set reference so this command can be continually
  // called and allow PID controllers to continually calculate, otherwise PID
  // output will "jump" whenever commands for pose are called
  public void setReference(double height, double angle) {
    setpointHeight = height;
    setpointAngle = angle;
  }
}
