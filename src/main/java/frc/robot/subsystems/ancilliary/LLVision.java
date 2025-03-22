// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.ancilliary;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.utils.LimelightHelpers;
import frc.robot.utils.Constants.VisionConstants;

public class LLVision extends SubsystemBase {

    //injected drive subsystem
    private final DriveSubsystem m_robotDrive;

    //vision robot pose
    private Pose2d visionPose = new Pose2d();

    public LLVision(DriveSubsystem drivesubsystem) {

        //inject drive subsystem
        m_robotDrive = drivesubsystem;

        //optimize number of tags LL looks for, configure in constants
        LimelightHelpers.SetFiducialIDFiltersOverride(VisionConstants.vis_LL1name, VisionConstants.vis_validIDs);

        //set position of LL on robot in 3d space, in meters, center robot origin, follows NWU/CCW+ convention
        //configure in constants
        //LimelightHelpers.setCameraPose_RobotSpace(
        //    VisionConstants.vis_LL1name,
        //    VisionConstants.vis_LL1position[0],
        //    VisionConstants.vis_LL1position[1],
        //    VisionConstants.vis_LL1position[2],
        //    VisionConstants.vis_LL1position[3],
        //    VisionConstants.vis_LL1position[4],
        //    VisionConstants.vis_LL1position[5]
        //);
    }

    @Override
    public void periodic() {

        //feed robot orientation to LL periodically to reduce tag ambiguity
        LimelightHelpers.SetRobotOrientation(
            VisionConstants.vis_LL1name,
            m_robotDrive.getPose().getRotation().getDegrees(),
            0,
            0,
            0,
            0,
            0
        );

        //MUST USE BLUE ORIGIN!!!!!!!!!!!!!
        LimelightHelpers.PoseEstimate visionPoseLL1 = LimelightHelpers.getBotPoseEstimate_wpiBlue(
            VisionConstants.vis_LL1name
        );

        //check if number of tags > 0 and turn rate is not > 1 rps, otherwise reject update
        boolean rejectVisionUpdate = false;

        if (visionPoseLL1.tagCount == 0) {
            rejectVisionUpdate = true;
        }

        if (m_robotDrive.getTurnRate() > 2 * Math.PI) {
            rejectVisionUpdate = true;
        }

        if(!rejectVisionUpdate) {
            m_robotDrive.updateVisionMeasurement(visionPoseLL1);
        }
    }

    public Pose2d getVisionPose() {
        return visionPose;
    }
}