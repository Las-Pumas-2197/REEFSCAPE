// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorLift;
import frc.robot.subsystems.elevator.ManipulatorWrist;
import frc.robot.utils.Constants.ElevatorCalibration;

public class poseHome extends Command {
  
  private final ElevatorLift m_ElevatorLift;
  private final ManipulatorWrist m_ManipulatorWrist;

  public poseHome(ElevatorLift elevatorlift, ManipulatorWrist manipulatorwrist) {
    m_ElevatorLift = elevatorlift;
    m_ManipulatorWrist = manipulatorwrist;
    addRequirements(m_ElevatorLift, m_ManipulatorWrist);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    m_ManipulatorWrist.setAngle(ElevatorCalibration.wrist_homeangle);
    if (m_ManipulatorWrist.atSetpoint()) {
      m_ElevatorLift.setHeight(ElevatorCalibration.elev_homeheight);
    } else {
      m_ElevatorLift.setHeight(m_ElevatorLift.getEncoderPositions()[2]);
    }
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
