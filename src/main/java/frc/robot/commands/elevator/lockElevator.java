// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.Elevator;

public class lockElevator extends Command {

  private final Elevator m_Elevator;

  public lockElevator(Elevator elevator) {

    //pass subystem to class level
    m_Elevator = elevator;
    addRequirements(m_Elevator);
  }

  @Override
  public void initialize() {
    run(() -> m_Elevator.tilt(6)).withTimeout(1);
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
