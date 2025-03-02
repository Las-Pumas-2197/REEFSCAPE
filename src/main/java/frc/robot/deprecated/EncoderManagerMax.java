// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.deprecated;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ElevatorConstants;

public class EncoderManagerMax extends SubsystemBase {

  private double enc_prevpos;
  private double enc_position;
  private double enc_delta;
  private boolean enc_delta_not_rational;

  RelativeEncoder encoder;
  SparkMax spark;

  /** Creates a new EncoderManager. */
  public EncoderManagerMax(SparkMax sparkMax) {
    spark = sparkMax;
    encoder = sparkMax.getEncoder();
  }

  public void encoderReset(){
    encoder.setPosition(0);
  }

  public double getPos(){
    return enc_position;
  }

  @Override
  public void periodic() {

    //check encoder state to allow encoder position update
    enc_delta_not_rational = Math.abs(enc_delta) > ElevatorConstants.elev_enc_maxrational;

    //get change in encoder reading since last scheduler cycle
    enc_delta = encoder.getPosition() - enc_prevpos;
    
    //check encoder state to allow updates, if encoder delta value is outside of rational range, throw out value
    if (enc_delta_not_rational != true) {
      enc_position += enc_delta;
    }
    
    //update prev position with current position before ending
    enc_prevpos = encoder.getPosition();
  }
}
