// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ManipulatorSpike;

public class manipOuttake extends Command {

  private final ManipulatorSpike m_ManipulatorSpike;

  private final Timer onTimer;

  private boolean finished;

  public manipOuttake(ManipulatorSpike manipulatorspike) {
    m_ManipulatorSpike = manipulatorspike;
    addRequirements(m_ManipulatorSpike);
    onTimer = new Timer();

  }

  @Override
  public void initialize() {
    finished = false;
    onTimer.reset();
    onTimer.start();
  }

  @Override
  public void execute() {
    m_ManipulatorSpike.shoot(-12);

    if (onTimer.get() > 0.5) {
      finished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {
    m_ManipulatorSpike.shoot(0);
    onTimer.stop();
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
