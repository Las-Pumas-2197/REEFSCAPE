// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.elevator.utils.Configs;
import frc.robot.subsystems.elevator.utils.EncoderManagerMax;

public class Elevator extends SubsystemBase {
  private final SparkMax elevatorMotor1;
  private final SparkMax elevatorMotor2;
  private final EncoderManagerMax elevatorEncoder1;
  private final EncoderManagerMax elevatorEncoder2;

  private final SparkMax tiltMotor1;
  private final SparkMax tiltMotor2;
  private final EncoderManagerMax tiltEncoder1;
  private final EncoderManagerMax tiltEncoder2;

  double elevatorEncoder1Pos;
  double elevatorEncoder2Pos;

  double tiltEncoder1Pos;
  double tiltEncoder2Pos;
  
  DigitalInput elev_upperswitch;
  DigitalInput elev_lowerswitch;

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
    
    elev_upperswitch = new DigitalInput(1);
    elev_lowerswitch = new DigitalInput(2);
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
      elev_upperswitch.get(),
      elev_lowerswitch.get(),
    };
  }

  public double[] getEncoderPositions(){
    return new double[] {elevatorEncoder1Pos, elevatorEncoder2Pos};
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
