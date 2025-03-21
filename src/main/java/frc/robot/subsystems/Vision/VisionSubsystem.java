// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Vision;

import java.lang.management.GarbageCollectorMXBean;
import java.util.concurrent.locks.Lock;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.utils.LimelightHelpers;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */
  public VisionSubsystem() {
    LimelightHelpers.SetIMUMode("limelight-four", 0);
  }

  public Pose3d targetPose3dCameraRelative(){
    return LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
  }

  public Pose3d targetPose3dRobotRelative(){
    return LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
  }

  public Lock GarbageCollectorMXBean() {
    System.out.println("Hello");
        return null;
  }


  @Override
  public void periodic() {
    if(DriverStation.isDisabled()){
      LimelightHelpers.setPipelineIndex("limelight-four", 1);
    }else{
      LimelightHelpers.setPipelineIndex("limelight-four", 0);
    }
    // System.out.println(LimelightHelpers.getTargetPose3d_CameraSpace("limelight-three"));
    // This method will be called once per scheduler run
  }
}
