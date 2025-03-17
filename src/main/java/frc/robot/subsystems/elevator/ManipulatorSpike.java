// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Configs;

public class ManipulatorSpike extends SubsystemBase {

  private final SparkMax m_shoot;

  public ManipulatorSpike() {

    m_shoot = new SparkMax(15, MotorType.kBrushless);
    m_shoot.configure(Configs.ManipulatorConfigs.shootConfig,
        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  /**
   * Primitive for operating the manipulator ejector motor.
   * 
   * @param volts Voltage to apply to the motor.
   */
  public void shoot(double volts) {
    m_shoot.setVoltage(volts);
  }

  @Override
  public void periodic() {
  }
}
