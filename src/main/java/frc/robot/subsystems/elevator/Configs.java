// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
/** Add your docs here. */
public final class Configs {
    public static final class ElevatorConfigs {
        public static final SparkMaxConfig tiltConfig = new SparkMaxConfig();
        public static final SparkMaxConfig leftConfig = new SparkMaxConfig();
        public static final SparkMaxConfig rightConfig = new SparkMaxConfig();
        static {
        tiltConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        rightConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);
        leftConfig
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .smartCurrentLimit(40);
        }

    }
}
