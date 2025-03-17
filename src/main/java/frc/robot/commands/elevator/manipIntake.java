// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import static edu.wpi.first.wpilibj2.command.Commands.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ManipulatorSpike;

public class manipIntake extends Command {

  private final ManipulatorSpike m_ManipulatorSpike;

  private boolean finished;

  public manipIntake(ManipulatorSpike manipulatorspike) {
    m_ManipulatorSpike = manipulatorspike;
    addRequirements(m_ManipulatorSpike);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    runEnd(() -> m_ManipulatorSpike.shoot(12), () -> m_ManipulatorSpike.shoot(0))
    .withTimeout(1)
    .finallyDo(() -> finished = true);
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return finished;
  }
}
