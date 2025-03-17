// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Configs.ElevatorConfigs;

public class ElevatorTilt extends SubsystemBase {

  private final SparkMax m_tiltright;
  private final SparkMax m_tiltleft;

  public ElevatorTilt() {
    // tilt motors and write configs
    m_tiltright = new SparkMax(10, MotorType.kBrushless);
    m_tiltleft = new SparkMax(11, MotorType.kBrushless);
    m_tiltright.configure(ElevatorConfigs.tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_tiltleft.configure(ElevatorConfigs.tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Primitve for operating elevator forward and back manually. Only used in case
   * failure has occured with elevator locks.
   * 
   * @param volts Passed volts to motors.
   */
  public void tilt(double volts) {
    // write the volts to the motors
    m_tiltright.setVoltage(volts);
    m_tiltleft.setVoltage(volts);
  }

  @Override
  public void periodic() {
  }
}
