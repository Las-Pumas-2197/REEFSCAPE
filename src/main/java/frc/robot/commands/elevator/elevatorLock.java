// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorTilt;

public class elevatorLock extends Command {

  private final ElevatorTilt m_ElevatortTilt;

  private boolean finished;

  public elevatorLock(ElevatorTilt elevatortilt) {

    // pass subystem to class level
    m_ElevatortTilt = elevatortilt;
    addRequirements(m_ElevatortTilt);
  }

  @Override
  public void execute() {
    run(() -> m_ElevatortTilt.tilt(6))
    .withTimeout(1)
    .finallyDo(() -> finished = true);
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
