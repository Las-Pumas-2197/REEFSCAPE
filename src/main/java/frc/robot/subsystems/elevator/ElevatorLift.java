// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Configs.ElevatorConfigs;
import frc.robot.utils.Constants.ElevatorLiftConstants;

public class ElevatorLift extends SubsystemBase {

  // spark max instances
  private final SparkMax m_elevright;
  private final SparkMax m_elevleft;

  // encoders
  private final RelativeEncoder enc_elevright;
  private final RelativeEncoder enc_elevleft;

  // PID controller, trapezoidal profile for height control
  private final TrapezoidProfile.Constraints prof_height;
  private final ProfiledPIDController pid_height;
  private final ElevatorFeedforward ff_height;

  // limit switch status vars
  private final DigitalInput sw_elevupper;
  private final DigitalInput sw_elevlower;

  // slew limiter for open loop mode and bool to enable/disable
  private final SlewRateLimiter slew_elev;
  private static final double slew_ratelimit = 12; // units per second

  // raw pos values from encoder manager
  private double var_elevrightheight;
  private double var_elevleftheight;
  private double var_elevheightavg; // averaged height of elevator encoders

  // elevator velocities
  private double var_elevrightvel;
  private double var_elevleftvel;
  private double var_elevvelavg;

  // bools for inverted limit switch states
  private boolean var_elevswupper;
  private boolean var_elevswlower;

  // applied volts to elevator
  private double var_volts;
  private double var_voltslimited;

  // pid and ff volts
  private double var_pidvolts;
  private double var_ffvolts;

  // multiplier for drive speed based on elevator height
  private double var_drivespeedmult;

  public ElevatorLift() {

    // elevator motors and write configs
    m_elevright = new SparkMax(12, MotorType.kBrushless);
    m_elevleft = new SparkMax(13, MotorType.kBrushless);
    m_elevright.configure(ElevatorConfigs.elevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_elevleft.configure(ElevatorConfigs.elevConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // encoders for height measurement
    enc_elevright = m_elevleft.getEncoder();
    enc_elevleft = m_elevleft.getEncoder();

    // pid and FF controllers
    prof_height = new TrapezoidProfile.Constraints(ElevatorLiftConstants.elev_maxvel,
        ElevatorLiftConstants.elev_maxacl);
    pid_height = new ProfiledPIDController(ElevatorLiftConstants.elev_PIDkP, 0, ElevatorLiftConstants.elev_PIDkD,
        prof_height);
    ff_height = new ElevatorFeedforward(ElevatorLiftConstants.elev_FFkS, ElevatorLiftConstants.elev_FFkG,
        ElevatorLiftConstants.elev_FFkV);

    // pid controller tolerance for atSetpoint() function
    pid_height.setTolerance(0.1);

    // saftey switches
    sw_elevupper = new DigitalInput(0);
    sw_elevlower = new DigitalInput(1);

    // slew for open loop control
    slew_elev = new SlewRateLimiter(slew_ratelimit);
  }

  /**
   * Method used to operate elevator in open-loop.
   * 
   * @param volts        Volts to apply to elevator.
   * @param slew_enabled Slews enabled or not.
   */
  public void lift(double volts) {

    // apply slew to volts when enabled
    var_volts = slew_elev.calculate(volts);

    // check limit switch states, transforms volts to limit motion if either switch
    // is triggered
    if (var_elevswlower || var_elevswupper) {

      // limit switch transformations
      if (var_elevswlower) {
        var_voltslimited = (var_volts + Math.abs(var_volts)) / 2;
      }
      if (var_elevswupper) {
        var_voltslimited = (var_volts - Math.abs(var_volts)) / 2;
      }

    } else {

      // else pass volts through
      var_voltslimited = var_volts;
    }

    // calculate FF volts and pass to motors
    double ffvolts = ff_height.calculate(var_voltslimited);
    m_elevright.setVoltage(ffvolts);
    m_elevleft.setVoltage(ffvolts);
  }

  /**
   * Used to raise elevator to a given setpoint.
   * 
   * @param height Height desired.
   */
  public void setHeight(double height) {

    // set goal for PID controller
    pid_height.setGoal(height);

    // calculate volts
    var_pidvolts = pid_height.calculate(var_elevheightavg) / ElevatorLiftConstants.elev_maxvel * 12;
    var_ffvolts = ff_height.calculate(pid_height.getSetpoint().velocity);

    // write volts to motors
    m_elevright.setVoltage(var_pidvolts + var_ffvolts);
    m_elevleft.setVoltage(var_pidvolts + var_ffvolts);
  }

  /**
   * Returns an array containing the status of the limit switches for the
   * elevator.
   * 
   * @return The array. Index 0 = upper, index 1 = lower.
   */
  public boolean[] getSwitchStatuses() {
    return new boolean[] {
        var_elevswupper,
        var_elevswlower
    };
  }

  /**
   * Returns an array containing the positions returned by the encoders.
   * 
   * @return The array. Index 0 = right, index 1 = left, 2 = averaged elevator
   *         height.
   */
  public double[] getEncoderPositions() {
    return new double[] {
        var_elevrightheight,
        var_elevleftheight,
        var_elevheightavg,
        var_elevvelavg
    };
  }

  /**
   * Returns an array containing the current volts calculated by different
   * functions in the subsystem.
   * 0 = limited transformed volts from elevatorOL. 1 = volts passed before limit
   * switch transformations.
   * 2 = calculated PID volts used by elevator CL. 3 = calculated FF volts in
   * elevator CL.
   * 
   * @return The array.
   */
  public double[] getVolts() {
    return new double[] {
        var_voltslimited,
        var_volts,
        var_pidvolts,
        var_ffvolts
    };
  }

  /**
   * Checks if the subsystem is at setpoint for greater than 1 second.
   * 
   * @return The debounced bool.
   */
  public boolean atSetpoint() {
    //return new Trigger(() -> pid_height.atSetpoint()).debounce(1).getAsBoolean();
    return pid_height.atSetpoint();
  }

  /** Returns the drive speed multipler that is calculated by elevator. */
  public double getDriveSpeedMult() {
    return var_drivespeedmult;
  }

  /**
   * Resets the encoders for the elevator to zero. Only call when the elevator is
   * in home state (fully retracted).
   */
  public Command resetEncoderPositions() {
    return runOnce(() -> enc_elevleft.setPosition(0)).andThen(runOnce(() -> enc_elevright.setPosition(0)));
  }

  public Command resetPIDF(double position) {
    return runOnce(() -> pid_height.reset(position));
  }

  @Override
  public void periodic() {

    // write encoder position to internal var
    var_elevrightheight = enc_elevright.getPosition();
    var_elevleftheight = enc_elevleft.getPosition();
    var_elevrightvel = enc_elevright.getVelocity();
    var_elevleftvel = enc_elevleft.getVelocity();
    var_elevheightavg = (var_elevrightheight + var_elevleftheight) / 2;
    var_elevvelavg = (var_elevleftvel + var_elevrightvel) / 2;

    // invert limit switches
    var_elevswlower = sw_elevlower.get() ? false : true;
    var_elevswupper = sw_elevupper.get() ? false : true;

    // calculate the multipler for the drive speed based on the height of the
    // elevator
    var_drivespeedmult = 0;
  }
}
