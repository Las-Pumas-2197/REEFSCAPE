// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;

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
    prof_wrist = new TrapezoidProfile.Constraints(Constants.ManipulatorConstants.wrist_maxvel, Constants.ManipulatorConstants.wrist_maxaccel);
    pid_wrist = new ProfiledPIDController(Constants.ManipulatorConstants.wrist_PIDkP, 0, Constants.ManipulatorConstants.wrist_PIDkD, prof_wrist);
  }

  /**Primitive for operating manipulator tilt.
   * @param volts Voltage to apply to the motor.
   */
  public void tiltSetVoltage(double volts) {
    m_wrist.setVoltage(volts);
  }

  /**Primitive for operating the manipulator ejector motor.
   * @param volts Voltage to apply to the motor.
   */
  public void spinSetVoltage(double volts) {
    m_shoot.setVoltage(volts);
  }

  /**Operate manipulator in closed loop to hold an angle.
   * @param angle The angle to hold at in rads.
   */
  public Command setAngle(double angle){
    return runOnce(() -> pid_wrist.setGoal(angle))
          .andThen(runEnd(
            () -> tiltSetVoltage(
              (((pid_wrist.calculate(angle) / ManipulatorConstants.wrist_maxvel)) * 12)), () -> tiltSetVoltage(0)));
  }

  public void runManipulator(boolean CL_enabled, double angle, double OL_volts, double spin_volts) {
    if (CL_enabled) {
      setAngle(angle);
    } else {
      tiltSetVoltage(OL_volts);
    }
  }

  /**Returns the current position of the manipulator in rads. */
  public double getAngle() {
    return enc_wrist.getPosition();
  }

  @Override
  public void periodic() {
  }
}
