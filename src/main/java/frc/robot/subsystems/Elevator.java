// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.EncoderManagerFlex;
import frc.robot.EncoderManagerMax;

public class Elevator extends SubsystemBase {
  SparkFlex elevatorMotor1;
  SparkFlex elevatorMotor2;
  
  SparkMax tiltMotor1;
  SparkMax tiltMotor2;

  EncoderManagerFlex elevatorEncoder1;
  EncoderManagerFlex elevatorEncoder2;

  EncoderManagerMax tiltEncoder1;
  EncoderManagerMax tiltEncoder2;

  double elevatorEncoder1Pos;
  double elevatorEncoder2Pos;

  double tiltEncoder1Pos;
  double tiltEncoder2Pos;
  
  DigitalInput limitSwitch1;
  DigitalInput limitSwitch2;
  DigitalInput limitSwitch3;
  DigitalInput limitSwitch4;


  /** Creates a new Elevator. */
  public Elevator() {
    elevatorMotor1 = new SparkFlex(1, MotorType.kBrushless);
    elevatorMotor2 = new SparkFlex(2, MotorType.kBrushless);

    tiltMotor1 = new SparkMax(3, MotorType.kBrushless);
    tiltMotor2 = new SparkMax(4, MotorType.kBrushless);

    elevatorEncoder1 = new EncoderManagerFlex(elevatorMotor1);
    elevatorEncoder2 = new EncoderManagerFlex(elevatorMotor2);

    tiltEncoder1 = new EncoderManagerMax(tiltMotor1);
    tiltEncoder2 = new EncoderManagerMax(tiltMotor2);
    
    limitSwitch1 = new DigitalInput(1);
    limitSwitch2 = new DigitalInput(2);
    limitSwitch3 = new DigitalInput(3);
    limitSwitch4 = new DigitalInput(4);
  }
  public Command elevatorSetSpeed(double volts){
    return parallel(
      run(() -> elevatorMotor1.setVoltage(volts)).finallyDo(() -> elevatorMotor1.setVoltage(0)),
      run(() -> elevatorMotor2.setVoltage(volts)).finallyDo(() -> elevatorMotor2.setVoltage(0)));
  }
  public Command tiltSetSpeed(double volts){
    return parallel(
      run(() -> tiltMotor1.setVoltage(volts)).finallyDo(() -> tiltMotor1.setVoltage(0)),
      run(() -> tiltMotor2.setVoltage(volts)).finallyDo(() -> tiltMotor2.setVoltage(0)));
  }
  public boolean[] getSwitchStatuses(){
    return new boolean[] {
      limitSwitch1.get(),
      limitSwitch2.get(),
      limitSwitch3.get(),
      limitSwitch4.get()
    };
  }
  @Override
  public void periodic() {
    elevatorEncoder1.runData();
    elevatorEncoder1Pos = elevatorEncoder1.getPos();
    elevatorEncoder2.runData();
    elevatorEncoder2Pos = elevatorEncoder2.getPos();
    tiltEncoder1.runData();
    tiltEncoder1Pos = tiltEncoder1.getPos();
    tiltEncoder2.runData();
    tiltEncoder2Pos = tiltEncoder2.getPos();
    // This method will be called once per scheduler run
  }
}
