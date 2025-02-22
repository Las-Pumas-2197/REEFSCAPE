// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import static edu.wpi.first.wpilibj2.command.Commands.parallel;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.simulation.DIOSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.EncoderManagerFlex;
import frc.robot.EncoderManagerMax;

public class Elevator extends SubsystemBase {
  SparkMax elevatorMotor1;
  SparkMax elevatorMotor2;
  
  SparkMax tiltMotor1;
  SparkMax tiltMotor2;

  EncoderManagerMax elevatorEncoder1;
  EncoderManagerMax elevatorEncoder2;

  EncoderManagerMax tiltEncoder1;
  EncoderManagerMax tiltEncoder2;

  double elevator_volts;
  double elevator_volts_slewed;

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
    elevatorMotor1 = new SparkMax(12, MotorType.kBrushless);
    elevatorMotor2 = new SparkMax(13, MotorType.kBrushless);
    elevatorMotor1.configure(Configs.ElevatorConfigs.rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorMotor2.configure(Configs.ElevatorConfigs.leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    tiltMotor1 = new SparkMax(10, MotorType.kBrushless);
    tiltMotor2 = new SparkMax(11, MotorType.kBrushless);
    tiltMotor1.configure(Configs.ElevatorConfigs.tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    tiltMotor2.configure(Configs.ElevatorConfigs.tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    elevatorEncoder1 = new EncoderManagerMax(elevatorMotor1);
    elevatorEncoder2 = new EncoderManagerMax(elevatorMotor2);

    tiltEncoder1 = new EncoderManagerMax(tiltMotor1);
    tiltEncoder2 = new EncoderManagerMax(tiltMotor2);
    
    limitSwitch1 = new DigitalInput(1);
    limitSwitch2 = new DigitalInput(2);
    limitSwitch3 = new DigitalInput(3);
    limitSwitch4 = new DigitalInput(4);
  }
  public void elevatorSetVoltage(double volts){
    elevatorMotor1.setVoltage(volts);
    elevatorMotor2.setVoltage(volts);
  }
  public void tiltSetVoltage(double volts){
    tiltMotor1.setVoltage(volts);
    tiltMotor2.setVoltage(volts);
  }

  public boolean[] getSwitchStatuses(){
    return new boolean[] {
      limitSwitch1.get(),
      limitSwitch2.get(),
      limitSwitch3.get(),
      limitSwitch4.get()
    };
  }
  public double[] getEncoderPositions(){
    return new double[] {elevatorEncoder1Pos, elevatorEncoder2Pos, tiltEncoder1Pos, tiltEncoder2Pos};
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
