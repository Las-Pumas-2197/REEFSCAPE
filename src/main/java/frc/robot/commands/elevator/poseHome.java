// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;
import frc.robot.subsystems.elevator.Manipulator;

public class poseHome extends Command {

  private final Elevator m_Elevator;
  private final Manipulator m_Manipulator;

  public poseHome(Elevator elevator, Manipulator manipulator) {
    m_Elevator = elevator;
    m_Manipulator = manipulator;
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
