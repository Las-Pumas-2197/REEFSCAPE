// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.elevator.utils;

/** Add your docs here. */
public class Constants {
    public static final class ElevatorConstants {
        //controller settings
        public static final int elev_maxcurrent = 40;
        public static final int tilt_maxcurrent = 40;
        public static final double elev_posfactor = 0.16900; //meters, 0.55449 feet
        public static final double elev_velfactor = elev_posfactor / 60; //meters per second
        public static final double elev_enc_maxrational = 0.03552; //meters, 0.11656 feet, max possible feet traveled per 20ms at max speed
        //PIDF settings, needs characterized
        public static final double elev_FFkS = 0;
        public static final double elev_FFkV = 0;
        public static final double elev_FFkG = 0;
        public static final double elev_FFkA = 0;
        public static final double elev_PIDkP = 0;
        public static final double elev_PIDkD = 0;
        //reef height calibration data
        public static final double elev_L1height = 0; //TBD
        public static final double elev_L2height = 0; //TBD
        public static final double elev_L3height = 0; //TBD
        public static final double elev_L4height = 0; //TBD
        public static final double elev_loadheight = 0; //TBD
    }

    public static final class ManipulatorConstants {
        //controller settings
        public static final int wrist_maxcurrent = 40;
        public static final int spike_maxcurrent = 20;
        public static final double wrist_posfactor = 0; //rads
        public static final double wrist_velfactor = 0; //rads/s
        public static final double wrist_maxrational = 0;
        //PIDF settings, needs characterized
        public static final double wrist_FFkS = 0;
        public static final double wrist_FFkV = 0;
        public static final double wrist_FFkG = 0;
        public static final double wrist_FFkA = 0;
        public static final double wrist_PIDkP = 0;
        public static final double wrist_PIDkD = 0;
        //reef angle calibration data
        public static final double wrist_L1angle = 0; //TBD
        public static final double wrist_L2angle = 0; //TBD
        public static final double wrist_L3angle = 0; //TBD
        public static final double wrist_L4angle = 0; //TBD
        public static final double wrist_loadangle = 0; //TBD
    }
}
