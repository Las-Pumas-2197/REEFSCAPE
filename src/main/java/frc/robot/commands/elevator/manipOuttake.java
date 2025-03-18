// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ManipulatorSpike;

public class manipOuttake extends Command {

  //subsystem
  private final ManipulatorSpike m_ManipulatorSpike;

  //timer to check time in command
  private final Timer onTimer;

  //bool used to end command as needed
  private boolean finished; 

  public manipOuttake(ManipulatorSpike manipulatorspike) {

    //injected subsystem and add requirements
    m_ManipulatorSpike = manipulatorspike;
    addRequirements(m_ManipulatorSpike);

    //timer to check time in command and reset to cover persistent calls
    onTimer = new Timer();
    onTimer.reset();

    //change to finished to false in case it persists across calls
    finished = false;
  }

  @Override
  public void initialize() {

    //start timer
    onTimer.start();

    //start motor
    m_ManipulatorSpike.shoot(-12);
  }

  @Override
  public void execute() {

    //check if time is greater than setpoint, end if true
    if (onTimer.get() > 0.5) {
      finished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {

    //stop motor and timer
    m_ManipulatorSpike.shoot(0);
    onTimer.stop();
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
