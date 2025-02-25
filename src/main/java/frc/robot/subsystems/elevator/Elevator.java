// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import java.util.function.IntFunction;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Configs.ElevatorConfigs;
import frc.robot.utils.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  private final SparkMax m_elevright;
  private final SparkMax m_elevleft;
  private final SparkMax m_tiltright;
  private final SparkMax m_tiltleft;  

  //encoder managers, makes encoder position values persistent without use of duty cycle encoders
  private final EncoderManagerMax enc_elevright;
  private final EncoderManagerMax enc_elevleft;

  //PID controller, trapezoidal profile for height control
  private final TrapezoidProfile.Constraints prof_height;
  private final ProfiledPIDController pid_height;

  //limit switch status vars
  private final DigitalInput sw_elevupper;
  private final DigitalInput sw_elevlower;

  //slew limiter for open loop mode and bool to enable/disable
  private final SlewRateLimiter slew_rightmotor;
  private final SlewRateLimiter slew_leftmotor;
  private static final double slew_ratelimit = 12; //units per second
  private boolean var_enableslew;
  
  //raw pos values from encoder manager
  private double var_elevrightheight;
  private double var_elevleftheight;

  //applied volts to elevator and tilt motors
  private double var_elevvolts;
  private double var_tiltvolts;

  /** Creates a new Elevator. */
  public Elevator() {
    
    //elevator motors and write configs
    m_elevright = new SparkMax(12, MotorType.kBrushless);
    m_elevleft = new SparkMax(13, MotorType.kBrushless);
    m_elevright.configure(ElevatorConfigs.rightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevleft.configure(ElevatorConfigs.leftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //tilt motors and write configs
    m_tiltright = new SparkMax(10, MotorType.kBrushless);
    m_tiltleft = new SparkMax(11, MotorType.kBrushless);
    m_tiltright.configure(ElevatorConfigs.tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_tiltleft.configure(ElevatorConfigs.tiltConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    //encoder managers for height measurement
    enc_elevright = new EncoderManagerMax(m_elevright);
    enc_elevleft = new EncoderManagerMax(m_elevleft);

    //pid controller
    prof_height = new TrapezoidProfile.Constraints(ElevatorConstants.elev_maxvel, ElevatorConstants.elev_maxacl);
    pid_height = new ProfiledPIDController(ElevatorConstants.elev_PIDkP, 0, ElevatorConstants.elev_PIDkD, prof_height);
    
    //saftey switches
    sw_elevupper = new DigitalInput(1);
    sw_elevlower = new DigitalInput(2);

    //slews for open loop control
    slew_rightmotor = new SlewRateLimiter(slew_ratelimit);
    slew_leftmotor = new SlewRateLimiter(slew_ratelimit);
  }

  /**Operate the elevator in open-loop with safeties. Safeties can be disabled by passing a boolean.
   * @param volts Voltage to apply to the elevator motors.
   * @param enable_safties To override safeties or not, in case of limit switch failure. True = enabled.
   * @param slew_enabled To operate the elevator with slews enabled or not. True = enabled.
   */
  public void elevatorSetVoltage(double volts, boolean slew_enabled){

    //enable or disable slews
    var_enableslew = slew_enabled;

    //if either switch is triggered, check which one and transform volts
    if (sw_elevlower.get() || sw_elevupper.get()) {
      if (sw_elevlower.get()) {
        var_elevvolts = MathUtil.clamp(Math.abs(volts) + volts, -12, 12);
      }
      if (sw_elevupper.get()) {
        var_elevvolts = MathUtil.clamp(volts - Math.abs(volts), -12, 12);
      }
    } else {
      var_elevvolts = volts;
    }
  }

  public void elevatorSetHeight(double height) {
    
  }

  public void tiltSetVoltage(double volts){
    m_tiltright.setVoltage(volts);
    m_tiltleft.setVoltage(volts);
  }

  /**Returns an array containing the status of the limit switches for the elevator.
   * @return The array. Index 0 = upper, index 1 = lower.
   */
  public boolean[] getSwitchStatuses(){
    return new boolean[] {
      sw_elevupper.get(),
      sw_elevlower.get()
    };
  }

  /**Returns an array containing the positions returned by the encoders.
   * @return The array. Index 0 = right, index 1 = left.
   */
  public double[] getEncoderPositions(){
    return new double[] {
      var_elevrightheight,
      var_elevleftheight
    };
  }

  @Override
  public void periodic() {

    //write volts to motors, deactivate slews if disabled
  if (var_enableslew) {
    m_elevright.setVoltage(slew_rightmotor.calculate(var_elevvolts));
    m_elevleft.setVoltage(slew_leftmotor.calculate(var_elevvolts));
  } else {
    m_elevright.setVoltage(var_elevvolts);
    m_elevleft.setVoltage(var_elevvolts);
  }

  //write encoder position to internal var
  var_elevrightheight = enc_elevright.getPos();
  var_elevleftheight = enc_elevright.getPos();
  }
}
