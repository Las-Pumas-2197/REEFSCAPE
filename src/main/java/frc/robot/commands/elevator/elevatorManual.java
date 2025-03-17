// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorLift;
import frc.robot.subsystems.elevator.ManipulatorWrist;

public class elevatorManual extends Command {

  private final ElevatorLift m_ElevatorLift;
  private final ManipulatorWrist m_ManipulatorWrist;

  private double liftvolts;
  private double tiltvolts;

  public elevatorManual(ElevatorLift elevatorlift, ManipulatorWrist manipulatorwrist) {
    m_ElevatorLift = elevatorlift;
    m_ManipulatorWrist = manipulatorwrist;
    addRequirements(m_ElevatorLift, m_ManipulatorWrist);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
      m_ElevatorLift.lift(liftvolts);
      m_ManipulatorWrist.tilt(tiltvolts);
  }

  @Override
  public void end(boolean interrupted) {
    liftvolts = 0;
    tiltvolts = 0;
    m_ElevatorLift.lift(liftvolts);
    m_ManipulatorWrist.tilt(tiltvolts);
  }

  @Override
  public boolean isFinished() {
    return false;
  }

  public void liftvolts(double volts) {
    liftvolts = volts;
  }

  public void tiltvolts(double volts) {
    tiltvolts = volts;
  }
}
