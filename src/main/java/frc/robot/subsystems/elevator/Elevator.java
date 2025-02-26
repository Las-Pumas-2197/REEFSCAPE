// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
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
  private final ElevatorFeedforward ff_height;

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
  private double var_elevheightavg; //averaged height of elevator encoders

  //applied volts to elevator
  private double var_elevvolts;

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
    ff_height = new ElevatorFeedforward(ElevatorConstants.elev_FFkS, ElevatorConstants.elev_FFkG, ElevatorConstants.elev_FFkV); //kA ignored due to high power
    
    //saftey switches
    sw_elevupper = new DigitalInput(0);
    sw_elevlower = new DigitalInput(1);

    //slews for open loop control
    slew_rightmotor = new SlewRateLimiter(slew_ratelimit);
    slew_leftmotor = new SlewRateLimiter(slew_ratelimit);
  }

  /**Operate the elevator in open-loop with safeties. Safeties can be disabled by passing a boolean.
   * @param volts Voltage to apply to the elevator motors.
   * @param enable_safties To override safeties or not, in case of limit switch failure. True = enabled.
   * @param slew_enabled To operate the elevator with slews enabled or not. True = enabled. DO NOT USE SLEWS WHEN IN CLOSED LOOP, BAD THINGS HAPPEN
   */
  public void elevatorSetVoltage(double volts, boolean slew_enabled){

    //if either switch is triggered, check which one and transform volts
    if (sw_elevlower.get() || sw_elevupper.get()) {
      if (sw_elevlower.get()) {
        var_enableslew = false;
        var_elevvolts = MathUtil.clamp(Math.abs(volts) + volts, -12, 12);
      }
      if (sw_elevupper.get()) {
        var_enableslew = false;
        var_elevvolts = MathUtil.clamp(volts - Math.abs(volts), -12, 12);
      }
    } else {
      var_enableslew = slew_enabled;
      var_elevvolts = volts;
    }
  }

  /** 
   * Operate the elevator in closed-loop with safeties. Elevator will accelerate and decelerate to setpoint according to constraints.
   * If limit switches are triggered, PID controller output will be negated through transformations in elevatorSetVoltage function.
   * @param height The height in meters to set the eleelelelelevator to.
   */
  public void elevatorSetHeight(double height) {

    //set goal of PID controller to desired setpoint
    pid_height.setGoal(height);
    elevatorSetVoltage((
      pid_height.calculate(var_elevheightavg) / ElevatorConstants.elev_maxvel) * 12 + 
      ff_height.calculate(pid_height.getSetpoint().velocity), false);
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
   * @return The array. Index 0 = right, index 1 = left, 2 = averaged elevator height.
   */
  public double[] getEncoderPositions(){
    return new double[] {
      var_elevrightheight,
      var_elevleftheight,
      var_elevheightavg
    };
  }

  @Override
  public void periodic() {
  
  //write volts to motors, deactivates slews if var_enableslew = false
  if (var_enableslew) {
    m_elevright.setVoltage(slew_rightmotor.calculate(var_elevvolts));
    m_elevleft.setVoltage(slew_leftmotor.calculate(var_elevvolts));
  } else {
    m_elevright.setVoltage(var_elevvolts);
    m_elevleft.setVoltage(var_elevvolts);
  }

  //write encoder position to internal var
  var_elevrightheight = enc_elevright.getPos();
  var_elevleftheight = enc_elevleft.getPos();
  var_elevheightavg = (var_elevrightheight + var_elevleftheight) / 2;
  }
}
