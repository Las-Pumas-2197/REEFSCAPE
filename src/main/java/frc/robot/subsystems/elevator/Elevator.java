// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

//import static edu.wpi.first.wpilibj2.command.Commands.*;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Configs.ElevatorConfigs;
import frc.robot.utils.Constants.ElevatorCalibration;
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
  private final ElevatorFeedforward ff_height; //not used currently, not characterized

  //limit switch status vars
  private final DigitalInput sw_elevupper;
  private final DigitalInput sw_elevlower;

  //timer for elevator tilt
  private final Timer elTimer;

  //slew limiter for open loop mode and bool to enable/disable
  private final SlewRateLimiter slew_elev;
  private static final double slew_ratelimit = 12; //units per second
  
  //raw pos values from encoder manager
  private double var_elevrightheight;
  private double var_elevleftheight;
  private double var_elevheightavg; //averaged height of elevator encoders

  //bools for inverted limit switch states
  private boolean var_elevswupper;
  private boolean var_elevswlower;

  //applied volts to elevator
  private double var_slewedvolts;
  private double var_elevvolts;

  //multiplier for drive speed based on elevator height
  private double var_drivespeedmult;

  /** Creates a new Elevator. */
  public Elevator() {
    
    //elevator motors and write configs
    m_elevright = new SparkMax(12, MotorType.kBrushless);
    m_elevleft = new SparkMax(13, MotorType.kBrushless);
    m_elevright.configure(ElevatorConfigs.elevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevleft.configure(ElevatorConfigs.elevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
    slew_elev = new SlewRateLimiter(slew_ratelimit);

    //timer
    elTimer = new Timer();
  }

  /**
   * Method used to operate elevator in open-loop. Also used by elevatorCL() to operate elevator in CL with the safety switches in place.
   * @param volts Volts to apply to elevator.
   * @param slew_enabled
   */
  public void elevatorOL(double volts, boolean slew_enabled) {

    //check limit switch states, stop elevator and reset slew if either is triggered, otherwise pass volts through conditionals
    if (var_elevswlower || var_elevswupper) {
      slew_elev.reset(0);
      if (var_elevswlower) {
        var_elevvolts = Math.abs(volts) + volts / 2;
      }
      if (var_elevswupper) {
        var_elevvolts = volts - Math.abs(volts) / 2;
      }
    } else {
      var_elevvolts = volts;
    }

    //slew volts depending on passed variable
    var_slewedvolts = slew_enabled ? slew_elev.calculate(var_elevvolts) : var_elevvolts;

    //write volts to motors
    m_elevright.setVoltage(var_slewedvolts);
    m_elevleft.setVoltage(var_slewedvolts);
  }

  /**
   * Command used to send a setpoint to the elevator and have it hold there.
   * @param height Height desired.
   */
  public Command elevatorCL(double height) {
    return runOnce(() -> pid_height.setGoal(height))
          .andThen(run(() -> elevatorOL(pid_height.calculate(height) / ElevatorConstants.elev_maxvel * 12, false)));
  }

  /**Automatically runs a subroutine to retract the elevator slowly, then resets the encoders when the lower limit is reached. */
  public Command elevatorHome() {
    return runOnce(() -> elevatorOL(3, true))
          .until(() -> var_elevswlower = true)
          .andThen(runOnce(() -> elevatorOL(0, false)))
          .andThen(runOnce(() -> resetEncoderPositions())); //AAAAAAAAAAND THEEEEEEEEEEN
  }

  /**
   * Method to call and operate elevator.
   * @param CL_enable To enable closed loop control. True = CL enabled. False = OL
   * @param height Height at which to set the elevator to in CL.
   * @param OL_volts Volts to set the elevator to in OL.
   */
  public void runElevator(boolean CL_enable, double height, double OL_volts) {
    if (CL_enable) {
      elevatorCL(height);
    } else {
      elevatorOL(OL_volts, true);
    }
  }

  /**
   * Primitve for operating elevator forward and back manually. Only used in case failure has occured with elevator locks.
   * @param volts Passed volts to motors.
   */
  public void tiltSetVoltage(double volts) {
    m_tiltright.setVoltage(volts);
    m_tiltleft.setVoltage(volts);
  }

  /**
   * Subroutine to bump elevator forward and lock elevator.
   */
  public Command lockElevator() {
    return runOnce(() -> elTimer.start()).andThen(run(() ->
      tiltSetVoltage(3)
    )).until(() -> elTimer.get() > 0.5).andThen(runOnce(() -> 
    tiltSetVoltage(0)
    ));
  }
  
  /**Returns an array containing the status of the limit switches for the elevator.
   * @return The array. Index 0 = upper, index 1 = lower.
   */
  public boolean[] getSwitchStatuses(){
    return new boolean[] {
      var_elevswupper,
      var_elevswupper
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

  /**Returns the drive speed multipler that is calculated by elevator. */
  public double getDriveSpeedMult() {
    return var_drivespeedmult;
  }

  /**Resets the encoders for the elevator to zero. Only call when the elevator is in home state (fully retracted).*/
  public Command resetEncoderPositions() {
    return runOnce(() -> enc_elevleft.encoderReset()).andThen(runOnce(() -> enc_elevright.encoderReset()));
  }

  @Override
  public void periodic() {

  //write encoder position to internal var
  var_elevrightheight = enc_elevright.getPos();
  var_elevleftheight = enc_elevleft.getPos();
  var_elevheightavg = (var_elevrightheight + var_elevleftheight) / 2;

  //invert limit switches
  var_elevswlower = sw_elevlower.get() ? false : true;
  var_elevswupper = sw_elevupper.get() ? false : true;

  //calculate the multipler for the drive speed based on the height of the elevator
  var_drivespeedmult = var_elevheightavg - ElevatorCalibration.elev_maxheight + 0.2;
  }
}
