// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.utils.Constants.ElevatorCalibration;

public class poseL4 extends Command {

  private final elevatorMain c_ElevatorMain;

  private boolean finished;

  public poseL4(elevatorMain elevatormain) {
    c_ElevatorMain = elevatormain;
    finished = false;
  }

  @Override
  public void initialize() {
    c_ElevatorMain.setReference(ElevatorCalibration.elev_L4height, ElevatorCalibration.wrist_L4angle);
    finished = true;
  }

  @Override
  public void execute() {}

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return finished;
  }
}
