// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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

    //encoders
    enc_wrist = m_wrist.getEncoder();

    //pid and FFs
    prof_wrist = new TrapezoidProfile.Constraints(0, 0);
    pid_wrist = new ProfiledPIDController(0, 0, 0, prof_wrist);
    ff_wrist = new ArmFeedforward(0, 0, 0);
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
