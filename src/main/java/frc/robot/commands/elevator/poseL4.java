// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorLift;
import frc.robot.subsystems.elevator.ManipulatorWrist;
import frc.robot.utils.Constants.ElevatorCalibration;

public class poseL4 extends Command {
  
  private final ElevatorLift m_ElevatorLift;
  private final ManipulatorWrist m_ManipulatorWrist;

  public poseL4(ElevatorLift elevator, ManipulatorWrist manipulator) {
    m_ElevatorLift = elevator;
    m_ManipulatorWrist = manipulator;
    addRequirements(m_ElevatorLift, m_ManipulatorWrist);
  }

  @Override
  public void initialize() {
  }

  @Override
  public void execute() {
    m_ManipulatorWrist.setAngle(ElevatorCalibration.wrist_L4angle);
    m_ElevatorLift.setHeight(ElevatorCalibration.elev_L4height);
  }

  @Override
public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
