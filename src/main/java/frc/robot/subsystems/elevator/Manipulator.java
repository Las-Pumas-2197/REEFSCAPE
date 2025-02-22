// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
public class Manipulator extends SubsystemBase {
  SparkMax tiltMotor;
  SparkMax spinMotor;
  /** Creates a new manipulator. */
  public Manipulator() {
    tiltMotor = new SparkMax(14, MotorType.kBrushless);
    spinMotor = new SparkMax(15, MotorType.kBrushless);
  }
  
public void manipulatorTiltSetVoltage(double volts){
   tiltMotor.setVoltage(volts);
}
public void manipulatorSpinSetVoltage(double volts){
   spinMotor.setVoltage(volts);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
