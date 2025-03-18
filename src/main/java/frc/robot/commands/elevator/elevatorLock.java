// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorTilt;

public class elevatorLock extends Command {

  private final ElevatorTilt m_ElevatortTilt;

  private boolean finished;

  private final Timer onTimer;

  public elevatorLock(ElevatorTilt elevatortilt) {

    //injected subsystem and add requirements
    m_ElevatortTilt = elevatortilt;
    addRequirements(m_ElevatortTilt);

    //timer
    onTimer = new Timer();

    //finished booean, ends command when true
    finished = false;
  }

  @Override
  public void initialize() {

    //reset and start timer due to persistence of instance across calls
    onTimer.reset();
    onTimer.start();
    
    //set volts to 6
    m_ElevatortTilt.tilt(6);
  }

  @Override
  public void execute() {

    //wait until timer elapsed, then end command
    if (onTimer.get() > 1) {
      finished = true;
    }
  }

  @Override
  public void end(boolean interrupted) {

    //stop timer and stop motor when ending
    onTimer.stop();
    m_ElevatortTilt.tilt(0);
  }

  @Override
  public boolean isFinished() {
    return finished;
  }
}
