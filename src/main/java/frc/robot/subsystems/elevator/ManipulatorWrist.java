// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
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

public class ManipulatorWrist extends SubsystemBase {

  // motors for manipulator
  private final SparkMax m_wrist;

  // encoder for position on wrist
  private final RelativeEncoder enc_wrist;

  // feed forward and PID for wrist
  private final TrapezoidProfile.Constraints prof_wrist;
  private final ProfiledPIDController pid_wrist;
  private final ArmFeedforward ff_wrist;

  // pidf volts
  private double var_pidvolts;
  private double var_ffvolts;

  /** Creates a new manipulator. */
  public ManipulatorWrist() {

    // motors
    m_wrist = new SparkMax(14, MotorType.kBrushless);
    m_wrist.configure(Configs.ManipulatorConfigs.wristConfig,
        com.revrobotics.spark.SparkBase.ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // encoders
    enc_wrist = m_wrist.getEncoder();

    // pid and FFs
    prof_wrist = new TrapezoidProfile.Constraints(Constants.ManipulatorConstants.wrist_maxvel,
        Constants.ManipulatorConstants.wrist_maxaccel);
    pid_wrist = new ProfiledPIDController(Constants.ManipulatorConstants.wrist_PIDkP, 0,
        Constants.ManipulatorConstants.wrist_PIDkD, prof_wrist);
    ff_wrist = new ArmFeedforward(ManipulatorConstants.wrist_FFkS, ManipulatorConstants.wrist_FFkG,
        ManipulatorConstants.wrist_FFkV);
  }

  /**
   * Primitive for operating manipulator tilt.
   * 
   * @param volts Voltage to apply to the motor.
   */
  public Command tilt(double volts) {
    return runEnd(() -> m_wrist.setVoltage(volts), () -> m_wrist.setVoltage(0));
  }

  /**
   * Operate manipulator in closed loop to hold an angle.
   * 
   * @param angle The angle to hold at in rads.
   */
  public void setAngle(double angle) {
    pid_wrist.setGoal(angle);
    var_pidvolts = pid_wrist.calculate(enc_wrist.getPosition()) / ManipulatorConstants.wrist_maxvel * 12;
    var_ffvolts = ff_wrist.calculate(enc_wrist.getPosition(), pid_wrist.getSetpoint().velocity);
    m_wrist.setVoltage(var_pidvolts + var_ffvolts);
  }

  /** Returns the current position of the manipulator in rads. */
  public double getAngle() {
    return enc_wrist.getPosition();
  }

  public Command resetEncoder() {
    return runOnce(() -> enc_wrist.setPosition(0));
  }

  public double[] getManipulatorData() {
    return new double[] {
        var_pidvolts,
        var_ffvolts
    };
  }

  public boolean atSetpoint() {
    //return new Trigger(() -> pid_wrist.atSetpoint()).debounce(1).getAsBoolean();
    return pid_wrist.atSetpoint();
  }

  public Command resetPIDF() {
    return runOnce(() -> pid_wrist.reset(0));
  }

  @Override
  public void periodic() {
  }
}
