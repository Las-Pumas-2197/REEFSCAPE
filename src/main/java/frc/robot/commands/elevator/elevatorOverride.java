// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorLift;
import frc.robot.subsystems.elevator.ManipulatorWrist;

public class elevatorOverride extends Command {

  // subsystems
  private final ElevatorLift m_ElevatorLift;
  private final ManipulatorWrist m_ManipulatorWrist;

  // voltages written from functions
  private double liftvolts;
  private double tiltvolts;

  public elevatorOverride(ElevatorLift elevatorlift, ManipulatorWrist manipulatorwrist) {

    // injected subsystems and add requirements for command
    m_ElevatorLift = elevatorlift;
    m_ManipulatorWrist = manipulatorwrist;
    addRequirements(m_ElevatorLift, m_ManipulatorWrist);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {

    // set passed volts to subsystems
    m_ElevatorLift.lift(liftvolts);
    m_ManipulatorWrist.tilt(tiltvolts);
  }

  @Override
  public void end(boolean interrupted) {

    // set motors to 0 when command ends
    m_ElevatorLift.lift(0);
    m_ManipulatorWrist.tilt(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public void liftVolts(double volts) {

    // pass volts into command
    liftvolts = volts;
  }

  public void tiltVolts(double volts) {

    // pass volts into command
    tiltvolts = volts;
  }
}
