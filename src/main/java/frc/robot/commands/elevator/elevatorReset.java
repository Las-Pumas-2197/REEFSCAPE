// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.elevator.ElevatorLift;
import frc.robot.subsystems.elevator.ManipulatorWrist;

public class elevatorReset extends Command {

    //injected subsystems
    private final ElevatorLift m_ElevatorLift;
    private final ManipulatorWrist m_ManipulatorWrist;

    private boolean finished;

    public elevatorReset(ElevatorLift elevatorlift, ManipulatorWrist manipulatorwrist) {

        //pass injected subsystems
        m_ElevatorLift = elevatorlift;
        m_ManipulatorWrist = manipulatorwrist;

    }

    @Override
    public void initialize() {
        
        //set bool to false so command doesn't immediately end between calls
        finished = false;
    }

    @Override
    public void execute() {
        if (m_ElevatorLift.getSwitchStatuses()[1] && m_ManipulatorWrist.getManipulatorData()[2] < 0.1) {
            m_ElevatorLift.resetEncoderPositions();
            m_ManipulatorWrist.resetEncoder();
            finished = true;
        } else {
            m_ElevatorLift.lift(-3);
            m_ManipulatorWrist.tilt(3);
        }
    }

    @Override
    public void end(boolean interrupted) {
        m_ElevatorLift.lift(0);
        m_ManipulatorWrist.tilt(0);
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}
