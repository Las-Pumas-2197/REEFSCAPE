// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Configs;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.ManipulatorConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
public class Manipulator extends SubsystemBase {

  //motors for manipulator
  private final SparkMax m_wrist;
  private final SparkMax m_shoot;

  //encoder for position on wrist
  private final RelativeEncoder enc_wrist;

  //feed forward and PID for wrist
  private final TrapezoidProfile.Constraints prof_wrist;
  private final ProfiledPIDController pid_wrist;
  private final ArmFeedforward ff_wrist;

  /** Creates a new manipulator. */
  public Manipulator() {

    //motors
    m_wrist = new SparkMax(14, MotorType.kBrushless);
    m_shoot = new SparkMax(15, MotorType.kBrushless);
    m_wrist.configure(Configs.ManipulatorConfigs.wristConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_shoot.configure(Configs.ManipulatorConfigs.shootConfig, com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //encoders
    enc_wrist = m_wrist.getEncoder();

    //pid and FFs
    prof_wrist = new TrapezoidProfile.Constraints(Constants.ManipulatorConstants.wrist_maxVel, Constants.ManipulatorConstants.wrist_maxAccel);
    pid_wrist = new ProfiledPIDController(Constants.ManipulatorConstants.wrist_PIDkP, 0, Constants.ManipulatorConstants.wrist_PIDkD, prof_wrist);
    ff_wrist = new ArmFeedforward(Constants.ManipulatorConstants.wrist_FFkS, Constants.ManipulatorConstants.wrist_FFkG, Constants.ManipulatorConstants.wrist_FFkV);
  }

public Command manipulatorSetAngle(double angle){
  return runOnce(() -> pid_wrist.setGoal(angle)).andThen(runEnd(() ->
  manipulatorTiltSetVoltage(
    ((pid_wrist.calculate(angle)/ManipulatorConstants.wrist_maxVel))//Maybe times 12
     + ff_wrist.calculate(enc_wrist.getPosition(), pid_wrist.getSetpoint().velocity)), () -> manipulatorTiltSetVoltage(0)));
}
public void manipulatorTiltSetVoltage(double volts) {
   m_wrist.setVoltage(volts);
}

public void manipulatorSpinSetVoltage(double volts) {
   m_shoot.setVoltage(volts);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
