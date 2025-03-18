// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ManipulatorSpike;

public class manipIntake extends Command {

  //subsystem
  private final ManipulatorSpike m_ManipulatorSpike;

  //timer to track time in command
  private final Timer onTimer;

  //boolean to end command
  private boolean finished;

  public manipIntake(ManipulatorSpike manipulatorspike) {

    // injected subsystem and add requirements
    m_ManipulatorSpike = manipulatorspike;
    addRequirements(m_ManipulatorSpike);

    // timer
    onTimer = new Timer();

    // set finished bool to false in case it persists across calls
    finished = false;

  }

  @Override
  public void initialize() {

    // reset timer and start
    onTimer.reset();
    onTimer.start();

    // set volts to motor
    m_ManipulatorSpike.shoot(12);
  }

  @Override
  public void execute() {

    // check timer time, if greater than time setpoint, end command
    if (onTimer.get() > 0.5) {
      finished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {

    // stop motor at end
    m_ManipulatorSpike.shoot(0);
    onTimer.stop();
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
