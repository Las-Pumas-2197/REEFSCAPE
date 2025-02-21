// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
public class Manipulator extends SubsystemBase {
  SparkMax tiltMotor;
  SparkMax spinMotor;
  /** Creates a new manipulator. */
  public Manipulator() {
    tiltMotor = new SparkMax(0, MotorType.kBrushless);
    spinMotor = new SparkMax(0, MotorType.kBrushless);
  }
  
public Command manipulatorTiltSetVoltage(double volts){
  return run(() -> tiltMotor.setVoltage(volts)).finallyDo(() -> tiltMotor.setVoltage(0));
}
public Command manipulatorSpinSetVoltage(double volts){
  return run(() -> spinMotor.setVoltage(volts)).finallyDo(() -> spinMotor.setVoltage(0));
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
