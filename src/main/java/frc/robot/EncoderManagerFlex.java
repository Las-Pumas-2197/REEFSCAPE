// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EncoderManagerFlex extends SubsystemBase {

  private double enc_prevpos;
  private double enc_position;
  private boolean enc_at_zero;
  private double enc_delta;
  private boolean enc_delta_not_rational;

  RelativeEncoder encoder;
  SparkFlex spark;

  /** Creates a new EncoderManager. */
  public EncoderManagerFlex(SparkFlex sparkFlex) {
    spark = sparkFlex;
    encoder = sparkFlex.getEncoder();
  }
  public void encoderReset(){
    encoder.setPosition(0);
  }

  public double getPos(){
    return enc_position;
  }

  public void runData() {

    //check encoder state to allow encoder position update
    enc_at_zero = Math.abs(enc_delta) < 0.1;
    enc_delta_not_rational = Math.abs(enc_delta) > 4;

    //troubleshooting
    //SmartDashboard.putBoolean("enc at zero", enc_at_zero);
    //SmartDashboard.putBoolean("enc delta not rational", enc_delta_not_rational);
    
    //get change in encoder delta since last scheduler cycle
    enc_delta = encoder.getPosition() - enc_prevpos;

    //check encoder state to allow updates
    if (enc_at_zero != true && enc_delta_not_rational != true) {
      enc_position += enc_delta;
    }

    //update prev position with current position before ending
    enc_prevpos = encoder.getPosition();
  }

  @Override
  public void periodic() {

  }
}
